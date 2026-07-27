"""Wall geometry helpers shared between FSM states."""

import math
import os
from typing import List, Optional, Tuple

import yaml

Point3 = Tuple[float, float, float]
Vec3 = Tuple[float, float, float]


def resolve_detected_walls_path(override: Optional[str] = None) -> Optional[str]:
    """Locate ``detected_walls.yaml``, written by robo_drill's wall_aggregator_node.

    Order: explicit override -> the robo_drill package share -> dev fallback to
    the source checkout under ``<ws>/src``. Under ``--symlink-install`` the share
    copy points back at the source tree, so the aggregator's live output is
    picked up. Returns the first existing file, or None.
    """
    candidates: List[str] = []
    if override:
        candidates.append(os.path.expanduser(str(override)))

    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("robo_drill")
        candidates.append(os.path.join(share, "rgb_detections", "detected_walls.yaml"))
        # colcon installs live under <ws>/install, sources under <ws>/src.
        if os.sep + "install" + os.sep in share:
            ws = share.split(os.sep + "install" + os.sep, 1)[0]
            candidates.append(
                os.path.join(ws, "src", "robo_drill", "rgb_detections", "detected_walls.yaml")
            )
    except Exception:
        pass  # ament unavailable / package not found: fall through to the candidates we have

    for cand in candidates:
        if cand and os.path.isfile(cand):
            return cand
    return None


def load_walls(yaml_path: str) -> List[dict]:
    """Load persisted walls from a ``detected_walls.yaml`` file.

    The file (written by robo_drill's wall_aggregator_node) stores map-frame
    planar walls under a top-level ``walls:`` list; each entry carries the
    horizontal outward ``normal`` [nx, ny], plane offset ``d`` (so
    ``nx*x + ny*y + d = 0`` on the wall), base endpoints ``p1``/``p2``, the
    vertical extent ``z_min``/``z_max`` and a ``confidence`` in 0..1.

    Returns the list of raw wall dicts (empty if the file has no walls). Raises
    ``FileNotFoundError``/``yaml.YAMLError`` to the caller on IO/parse errors.
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    walls = data.get("walls") or []
    return list(walls)


def associate_point_to_wall(
    point: Point3,
    walls: List[dict],
    *,
    min_confidence: float = 0.25,
    segment_margin_m: float = 0.5,
    z_margin_m: float = 0.3,
    max_plane_distance_m: float = 1.0,
) -> Optional[dict]:
    """Return the wall that ``point`` (map-frame x, y, z) lies on, or ``None``.

    Among walls whose XY projection of the point falls within the ``p1 -> p2``
    segment (padded by ``segment_margin_m``) and whose ``[z_min, z_max]`` band
    contains the point's z (padded by ``z_margin_m``), pick the one with the
    smallest perpendicular plane distance ``|nx*x + ny*y + d|``. Walls below
    ``min_confidence`` are ignored. If no wall passes the segment/z gates, fall
    back to the closest plane among the confident walls (drill points come from
    an operator and may sit slightly off).

    A candidate is only returned if its perpendicular plane distance is within
    ``max_plane_distance_m``. This rejects points that are not actually on any
    wall -- e.g. a drill target computed against a different map than the one the
    wall file was recorded on -- so the caller gets ``None`` (and can fail
    loudly) instead of a confidently-wrong normal from an unrelated wall.
    """
    px, py, pz = float(point[0]), float(point[1]), float(point[2])

    confident = [w for w in walls if float(w.get("confidence", 1.0)) >= min_confidence]
    if not confident:
        return None

    def plane_distance(w: dict) -> float:
        nx, ny = float(w["normal"][0]), float(w["normal"][1])
        return abs(nx * px + ny * py + float(w["d"]))

    def within_extent(w: dict) -> bool:
        p1, p2 = w["p1"], w["p2"]
        # Parametric projection of (px, py) onto the p1->p2 segment.
        ax, ay = float(p1[0]), float(p1[1])
        bx, by = float(p2[0]), float(p2[1])
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-9:
            return False
        t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
        margin_t = segment_margin_m / math.sqrt(seg_len2)
        if t < -margin_t or t > 1.0 + margin_t:
            return False
        z_min = float(w.get("z_min", -math.inf))
        z_max = float(w.get("z_max", math.inf))
        return (z_min - z_margin_m) <= pz <= (z_max + z_margin_m)

    gated = [w for w in confident if within_extent(w)]
    candidates = gated if gated else confident
    best = min(candidates, key=plane_distance)
    if plane_distance(best) > max_plane_distance_m:
        return None
    return best


def outward_normal_2d(wall: dict) -> Vec3:
    """Return a wall's unit outward horizontal normal as a 3D vector (nz=0)."""
    nx, ny = float(wall["normal"][0]), float(wall["normal"][1])
    norm = math.hypot(nx, ny)
    if norm < 1e-9:
        raise ValueError("Wall normal is degenerate.")
    return (nx / norm, ny / norm, 0.0)


def inward_normal_from_wall_points(p1: Point3, p2: Point3) -> Vec3:
    """Return the inward (scan-facing) wall normal as a unit vector in map frame.

    Convention: p1 is the bottom-left vertex, p2 is the top-right vertex of the
    wall. The exterior (scan) side lies on the clockwise perpendicular of the
    p1 -> p2 edge in the XY plane; the inward normal points the opposite way
    and is the direction the EE z-axis should look along while scanning.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        raise ValueError("Wall points must differ in the XY plane.")
    dx /= length
    dy /= length
    # CW perpendicular (dy, -dx) is the exterior side; flip for the inward normal.
    return (-dy, dx, 0.0)


def ee_rpy_deg_from_inward_normal(normal: Vec3) -> Vec3:
    """Return EE (roll, pitch, yaw) in degrees so its z-axis aligns with ``normal``.

    Roll is fixed at 0, preserving the original convention that keeps the EE
    x-axis pointing down for vertical walls. For a horizontal inward normal
    this reduces to pitch=90, yaw=atan2(ny, nx); the previous hard-coded
    (0, 90, 0) is recovered when the normal is (+1, 0, 0).
    """
    nx, ny, nz = normal
    pitch = math.degrees(math.atan2(math.hypot(nx, ny), nz))
    yaw = math.degrees(math.atan2(ny, nx))
    return (0.0, pitch, yaw)
