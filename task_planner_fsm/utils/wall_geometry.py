"""Wall geometry helpers shared between FSM states."""

import math
from typing import Tuple

Point3 = Tuple[float, float, float]
Vec3 = Tuple[float, float, float]


def inward_normal_from_wall_points(p1: Point3, p2: Point3) -> Vec3:
    """Legacy fallback normal derived only from the p1->p2 winding.

    NOTE: detected_walls.yaml provides p1/p2 as the two *bottom* endpoints of the
    wall (both on the ground) plus a per-wall ``normal`` that points OUT of the
    building. Prefer that normal (see ``build_wall_data``); this winding-based
    guess is only used for walls with no detected normal (e.g. hardcoded
    fallbacks), and its inside/outside assignment is not reliable.

    Returns the clockwise-perpendicular-flipped unit vector; the EE z-axis is
    meant to look along it while scanning.
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


def build_wall_data(p1, p2, offset: float = 0.6, scan_lines_z=None, outward_normal=None):
    """Build scan geometry for one wall.

    ``p1``/``p2`` are the wall's two bottom endpoints (map frame). ``outward_normal``
    is the detected wall normal (the RViz arrow) pointing OUT of the building; the
    robot scans from the interior (``-outward_normal``) side. When it is None the
    scan side is guessed from the p1->p2 winding (legacy fallback for walls with no
    detected normal).

    Returns the wall-data dict consumed by WallTargetSelection/NavigateToTarget/
    ScanWall: ``original``, ``scan_line`` (offset to the interior side), the
    ``inward_normal`` the EE z-axis looks along, ``ee_rpy_deg`` and ``scan_lines_z``.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        raise ValueError("Wall points must be different.")
    dx /= length
    dy /= length

    ounit = None
    if outward_normal is not None:
        onx, ony = float(outward_normal[0]), float(outward_normal[1])
        nlen = math.hypot(onx, ony)
        if nlen >= 1e-9:
            ounit = (onx / nlen, ony / nlen)

    if ounit is not None:
        # Scan from the interior (-outward_normal); EE z looks along +outward_normal
        # into the wall face. This is the reliable, detection-provided side.
        sx, sy = -ounit[0], -ounit[1]
        inward_normal = (ounit[0], ounit[1], 0.0)
    else:
        # Legacy: scan side = CW perpendicular of p1->p2; inward = its flip.
        sx, sy = dy, -dx
        inward_normal = (-dy, dx, 0.0)

    scan_start = (
        p1[0] + offset * dx + sx * offset,
        p1[1] + offset * dy + sy * offset,
        p1[2],
    )
    scan_end = (
        p2[0] - offset * dx + sx * offset,
        p2[1] - offset * dy + sy * offset,
        p2[2],
    )

    ee_rpy_deg = ee_rpy_deg_from_inward_normal(inward_normal)

    # Horizontal scan lines (heights), bottom-first. Default to a single line at
    # the endpoint z, preserving single-pass behaviour.
    if scan_lines_z:
        scan_lines_z = sorted(float(z) for z in scan_lines_z)
    else:
        scan_lines_z = [float(scan_start[2])]

    return {
        "original": (tuple(p1), tuple(p2)),
        "scan_line": (scan_start, scan_end),
        "inward_normal": inward_normal,
        "ee_rpy_deg": ee_rpy_deg,
        "scan_lines_z": scan_lines_z,
    }
