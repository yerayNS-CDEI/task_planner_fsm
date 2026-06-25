"""Floor-coverage planning helpers shared by the ScanFloor FSM state.

Pure, numpy-based geometry: no ROS objects. Given a decoded occupancy grid
(2D int8 array + map-frame origin + resolution) these functions estimate the
dominant wall orientation, carve out the reachable free region, and generate
straight parallel scan trajectories ("lines") in two perpendicular directions.

The grid follows the ``nav_msgs/OccupancyGrid`` convention: values are -1
(unknown), 0..100 (occupancy probability), arranged row-major with ``grid[r, c]``
at map-frame point ``(origin_x + (c + 0.5) * res, origin_y + (r + 0.5) * res)``.

A ``Segment`` is ``((x0, y0), (x1, y1))`` in the map frame, matching the
``target_point`` tuples used by ScanWall.
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:  # SciPy is already a project dependency (see arm_control), but stay graceful.
    from scipy import ndimage as _ndimage
except Exception:  # pragma: no cover - exercised only when SciPy is absent.
    _ndimage = None

try:  # OpenCV ships with the YOLO/perception stack; used for contour tracing.
    import cv2 as _cv2
except Exception:  # pragma: no cover - exercised only when OpenCV is absent.
    _cv2 = None

Point2 = Tuple[float, float]
Segment = Tuple[Point2, Point2]

# Default obstacle threshold for edge detection on a raw occupancy map.
_OCCUPIED_THRESH = 65

# Default max traversable cost. The grid is interpreted as a nav2 costmap
# (0 = free, rising to 99 inscribed / 100 lethal, -1 = unknown). With the
# default of 0, only fully-clear cells are usable, so generated lines stay
# entirely OUTSIDE the costmap's inflation layer (where the base cannot drive
# straight). Raising it trades clearance for coverage closer to walls.
_DEFAULT_MAX_COST = 0


# ----------------------------------------------------------------------------
# Masks
# ----------------------------------------------------------------------------
def occupied_mask(grid: np.ndarray) -> np.ndarray:
    """Lethal-ish cells, used only for wall-orientation edge detection."""
    return grid >= _OCCUPIED_THRESH


def blocked_mask(grid: np.ndarray, max_cost: int = _DEFAULT_MAX_COST) -> np.ndarray:
    """Cells the base must avoid: cost above ``max_cost`` OR unknown (-1).

    For a nav2 costmap this treats every inflated cell (cost > ``max_cost``) as
    blocked, keeping scan lines out of the inflation layer.
    """
    return (grid > max_cost) | (grid < 0)


def free_mask(grid: np.ndarray, max_cost: int = _DEFAULT_MAX_COST) -> np.ndarray:
    """Cells the base may traverse: known and cost within ``max_cost``."""
    return (grid >= 0) & (grid <= max_cost)


# ----------------------------------------------------------------------------
# Dominant wall orientation
# ----------------------------------------------------------------------------
def dominant_wall_angle(grid: np.ndarray, *, num_bins: int = 90,
                        min_edges: int = 30) -> float:
    """Estimate the dominant wall direction ``theta`` in ``[0, pi/2)`` radians.

    Walls show up as edges in the occupied mask; the gradient direction at an
    edge is perpendicular to the wall. Folding gradient angles into ``[0, pi/2)``
    (mod 90 degrees) collapses a wall and its perpendicular onto the same axis,
    so the histogram peak is the building's principal axis. ``theta`` and
    ``theta + pi/2`` are then the two scan directions.

    Returns ``0.0`` (axis-aligned) when there are too few edges to be reliable.
    """
    occ = occupied_mask(grid).astype(np.float64)
    if occ.sum() < min_edges:
        return 0.0

    # Sobel gradients (gy along rows = +y in map frame, gx along cols = +x).
    gx = np.zeros_like(occ)
    gy = np.zeros_like(occ)
    gx[:, 1:-1] = occ[:, 2:] - occ[:, :-2]
    gy[1:-1, :] = occ[2:, :] - occ[:-2, :]

    mag = np.hypot(gx, gy)
    edge = mag > 0.0
    if int(edge.sum()) < min_edges:
        return 0.0

    angles = np.arctan2(gy[edge], gx[edge])      # [-pi, pi]
    weights = mag[edge]
    folded = np.mod(angles, math.pi / 2.0)       # [0, pi/2)

    hist, edges = np.histogram(
        folded, bins=num_bins, range=(0.0, math.pi / 2.0), weights=weights
    )
    if hist.sum() <= 0.0:
        return 0.0

    peak = int(np.argmax(hist))
    # Refine with a weighted mean over the peak bin and its neighbours.
    lo, hi = max(0, peak - 1), min(num_bins, peak + 2)
    centers = 0.5 * (edges[lo:hi] + edges[lo + 1:hi + 1])
    w = hist[lo:hi]
    theta = float(np.dot(centers, w) / w.sum())
    return theta % (math.pi / 2.0)


# ----------------------------------------------------------------------------
# Obstacle inflation + reachability
# ----------------------------------------------------------------------------
def inflate_obstacles(blocked: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate the blocked mask by ``radius_cells`` to keep base clearance."""
    if radius_cells <= 0:
        return blocked.copy()

    if _ndimage is not None:
        structure = _disk_structure(radius_cells)
        return _ndimage.binary_dilation(blocked, structure=structure)

    return _max_filter_dilate(blocked, radius_cells)


def _disk_structure(radius_cells: int) -> np.ndarray:
    r = int(radius_cells)
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= (r * r)


def _max_filter_dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Numpy-only square dilation fallback when SciPy is unavailable."""
    out = mask.copy()
    r = int(radius_cells)
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc > r * r:
                continue
            shifted = np.roll(np.roll(mask, dr, axis=0), dc, axis=1)
            # Zero out wrap-around introduced by np.roll.
            if dr > 0:
                shifted[:dr, :] = False
            elif dr < 0:
                shifted[dr:, :] = False
            if dc > 0:
                shifted[:, :dc] = False
            elif dc < 0:
                shifted[:, dc:] = False
            out |= shifted
    return out


def reachable_free_mask(free: np.ndarray, start_cell: Tuple[int, int]) -> np.ndarray:
    """Free cells 4-connected to ``start_cell`` (row, col).

    If the start cell is not free, the nearest free cell is used as the seed.
    Returns an all-False mask if there is no free space at all.
    """
    if not free.any():
        return np.zeros_like(free)

    seed = _nearest_free_cell(free, start_cell)
    if seed is None:
        return np.zeros_like(free)

    if _ndimage is not None:
        labels, _ = _ndimage.label(free)
        seed_label = labels[seed]
        if seed_label == 0:
            return np.zeros_like(free)
        return labels == seed_label

    return _flood_fill(free, seed)


def _nearest_free_cell(free: np.ndarray, start_cell: Tuple[int, int]
                       ) -> Optional[Tuple[int, int]]:
    rows, cols = free.shape
    r0 = min(max(int(start_cell[0]), 0), rows - 1)
    c0 = min(max(int(start_cell[1]), 0), cols - 1)
    if free[r0, c0]:
        return (r0, c0)
    free_idx = np.argwhere(free)
    if free_idx.size == 0:
        return None
    d2 = (free_idx[:, 0] - r0) ** 2 + (free_idx[:, 1] - c0) ** 2
    nearest = free_idx[int(np.argmin(d2))]
    return (int(nearest[0]), int(nearest[1]))


def _flood_fill(free: np.ndarray, seed: Tuple[int, int]) -> np.ndarray:
    out = np.zeros_like(free)
    rows, cols = free.shape
    stack = [seed]
    out[seed] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and free[nr, nc] and not out[nr, nc]:
                out[nr, nc] = True
                stack.append((nr, nc))
    return out


# ----------------------------------------------------------------------------
# Coverage line generation
# ----------------------------------------------------------------------------
def generate_coverage_lines(reachable: np.ndarray, origin: Point2,
                            resolution: float, angle: float, spacing_m: float,
                            min_len_m: float) -> List[Segment]:
    """Straight scan segments along ``angle`` covering the reachable region.

    Parallel lines are swept ``spacing_m`` apart across the region's extent.
    Each line is sampled at ~half-cell steps; maximal runs of reachable-free
    cells become segments (so an obstacle splits a line into several). Runs
    shorter than ``min_len_m`` are dropped. Segments are returned sorted by
    their perpendicular offset (caller applies serpentine ordering).
    """
    if not reachable.any() or spacing_m <= 0.0 or resolution <= 0.0:
        return []

    dir_x, dir_y = math.cos(angle), math.sin(angle)        # along-line unit vec
    perp_x, perp_y = -math.sin(angle), math.cos(angle)     # line-to-line unit vec

    # World coordinates of every reachable cell center.
    idx = np.argwhere(reachable)
    cell_x = origin[0] + (idx[:, 1] + 0.5) * resolution
    cell_y = origin[1] + (idx[:, 0] + 0.5) * resolution

    along = cell_x * dir_x + cell_y * dir_y
    perp = cell_x * perp_x + cell_y * perp_y
    along_min, along_max = float(along.min()), float(along.max())
    perp_min, perp_max = float(perp.min()), float(perp.max())

    step = resolution * 0.5
    n_steps = max(1, int(math.ceil((along_max - along_min) / step)))

    segments: List[Segment] = []
    offset = perp_min + spacing_m * 0.5
    while offset <= perp_max:
        runs = _free_runs_along_line(
            reachable, origin, resolution,
            (dir_x, dir_y), (perp_x, perp_y),
            offset, along_min, along_max, n_steps, step,
        )
        for s_start, s_end in runs:
            if (s_end - s_start) < min_len_m:
                continue
            p0 = (offset * perp_x + s_start * dir_x,
                  offset * perp_y + s_start * dir_y)
            p1 = (offset * perp_x + s_end * dir_x,
                  offset * perp_y + s_end * dir_y)
            segments.append((p0, p1))
        offset += spacing_m

    return segments


def _free_runs_along_line(reachable: np.ndarray, origin: Point2, resolution: float,
                          direction: Point2, perp: Point2, offset: float,
                          along_min: float, along_max: float, n_steps: int,
                          step: float) -> List[Tuple[float, float]]:
    """Return ``(along_start, along_end)`` runs of reachable cells on one line."""
    rows, cols = reachable.shape
    dir_x, dir_y = direction
    perp_x, perp_y = perp

    runs: List[Tuple[float, float]] = []
    run_start: Optional[float] = None
    prev_along = along_min

    for i in range(n_steps + 1):
        a = along_min + i * step
        wx = offset * perp_x + a * dir_x
        wy = offset * perp_y + a * dir_y
        c = int((wx - origin[0]) / resolution)
        r = int((wy - origin[1]) / resolution)
        is_free = (0 <= r < rows and 0 <= c < cols and bool(reachable[r, c]))

        if is_free and run_start is None:
            run_start = a
        elif not is_free and run_start is not None:
            runs.append((run_start, prev_along))
            run_start = None
        prev_along = a

    if run_start is not None:
        runs.append((run_start, prev_along))
    return runs


# ----------------------------------------------------------------------------
# Ordering + top-level plan
# ----------------------------------------------------------------------------
def order_serpentine(segments: Sequence[Segment], angle: float,
                     start_xy: Optional[Point2] = None) -> List[Segment]:
    """Order lines for a serpentine sweep, entering each at its nearest endpoint.

    Lines are swept across the room in perpendicular-offset order (so coverage
    progresses line by line and does not skip around). Within that sweep, every
    segment is entered from whichever of its two endpoints is closest to the
    robot's current position — i.e. the end of the previously scanned segment.
    For clean full-width lines this reproduces a boustrophedon; for lines split
    by obstacles it still always picks the nearest endpoint.

    When ``start_xy`` (e.g. the robot's position) is given, the sweep starts at
    the wall-adjacent line nearest the robot. Without it, it starts at the
    minimum-offset edge.
    """
    if not segments:
        return []

    perp_x, perp_y = -math.sin(angle), math.cos(angle)
    dir_x, dir_y = math.cos(angle), math.sin(angle)

    def offset_of(seg: Segment) -> float:
        (x0, y0), _ = seg
        return x0 * perp_x + y0 * perp_y

    def along_of(pt: Point2) -> float:
        return pt[0] * dir_x + pt[1] * dir_y

    # Group segments by their line (perpendicular offset), keeping offsets sorted.
    groups: dict = {}
    for seg in segments:
        key = round(offset_of(seg), 6)
        groups.setdefault(key, []).append(seg)
    sorted_offsets = sorted(groups.keys())

    # Sweep direction across the room: start at the edge nearest the robot.
    if start_xy is not None:
        robot_offset = start_xy[0] * perp_x + start_xy[1] * perp_y
        if abs(robot_offset - sorted_offsets[-1]) < abs(robot_offset - sorted_offsets[0]):
            sorted_offsets.reverse()
        cur = start_xy
    else:
        cur = None  # default: enter first segment at its lower-along endpoint

    out: List[Segment] = []
    for off in sorted_offsets:
        remaining = list(groups[off])
        # Greedily consume this line's segment(s), each entered at its nearest
        # endpoint to the current position.
        while remaining:
            best_i, best_flip, best_d = 0, False, None
            for i, (p0, p1) in enumerate(remaining):
                if cur is None:
                    # No reference yet: enter at the lower-along endpoint.
                    flip = along_of(p0) > along_of(p1)
                    d = -1.0  # deterministic; only one pass needed below
                else:
                    d0 = math.dist(cur, p0)
                    d1 = math.dist(cur, p1)
                    flip = d1 < d0
                    d = min(d0, d1)
                if best_d is None or d < best_d:
                    best_i, best_flip, best_d = i, flip, d
                if cur is None:
                    break
            p0, p1 = remaining.pop(best_i)
            if best_flip:
                p0, p1 = p1, p0
            out.append((p0, p1))
            cur = p1
    return out


def nearest_region_corner(reachable: np.ndarray, origin: Point2, resolution: float,
                          angle: float, ref_xy: Point2) -> Point2:
    """Corner of the reachable region's oriented bounding box closest to ``ref_xy``.

    The bounding box axes are ``angle`` and ``angle + 90 deg`` (the two scan
    directions), so its four corners are the extreme room corners the snake can
    start from. Returns ``ref_xy`` if the region is empty.
    """
    idx = np.argwhere(reachable)
    if idx.size == 0:
        return ref_xy

    cx = origin[0] + (idx[:, 1] + 0.5) * resolution
    cy = origin[1] + (idx[:, 0] + 0.5) * resolution
    dx, dy = math.cos(angle), math.sin(angle)        # along-axis unit vector
    px, py = -math.sin(angle), math.cos(angle)       # perp-axis unit vector

    along = cx * dx + cy * dy
    perp = cx * px + cy * py
    a_lo, a_hi = float(along.min()), float(along.max())
    p_lo, p_hi = float(perp.min()), float(perp.max())

    best, best_d = ref_xy, math.inf
    for a in (a_lo, a_hi):
        for p in (p_lo, p_hi):
            corner = (a * dx + p * px, a * dy + p * py)
            d = math.dist(ref_xy, corner)
            if d < best_d:
                best, best_d = corner, d
    return best


def build_coverage_plan(grid: np.ndarray, origin: Point2, resolution: float,
                        start_cell: Tuple[int, int], *, spacing_m: float,
                        inflation_m: float, min_len_m: float,
                        max_cost: int = _DEFAULT_MAX_COST,
                        angle: Optional[float] = None) -> dict:
    """Full floor-coverage plan: two perpendicular serpentine line sets.

    ``dir_a`` runs along the dominant wall angle, ``dir_b`` perpendicular to it.
    The base sweeps one direction fully before switching. The whole snake is
    anchored at the reachable region's corner nearest the robot, so it begins
    exactly in a room corner (closest to the walls). Which direction is swept
    first is chosen by proximity: whichever group has its first scan point closest
    to the robot, so the run does not begin by crossing the room. Each direction
    is a strict edge-to-edge serpentine and the second begins near where the first
    ended, so the base never drives back across scanned space. ``start_cell`` is
    ``(row, col)``.

    The returned ``segments`` list is in execution order, and ``segment_angles``
    is a parallel list giving the line heading (turret yaw) for each segment.
    ``first_is_a`` reports whether ``dir_a`` was chosen to run first.

    ``grid`` is expected to be a nav2 costmap: cells with cost above ``max_cost``
    (i.e. the inflation layer and obstacles) are treated as untraversable, so the
    generated lines stay out of the inflation. ``inflation_m`` adds an OPTIONAL
    extra safety margin on top of the costmap's own inflation.
    """
    theta = dominant_wall_angle(grid) if angle is None else (angle % (math.pi / 2.0))

    blocked = blocked_mask(grid, max_cost)
    radius_cells = int(round(inflation_m / resolution)) if resolution > 0 else 0
    inflated = inflate_obstacles(blocked, radius_cells)

    free = free_mask(grid, max_cost) & ~inflated
    reachable = reachable_free_mask(free, start_cell)

    # Robot world position (map frame) from its start cell, used to begin the
    # sweep at the nearest wall-adjacent line instead of crossing the room.
    start_xy = (
        origin[0] + (start_cell[1] + 0.5) * resolution,
        origin[1] + (start_cell[0] + 0.5) * resolution,
    )

    theta_b = theta + math.pi / 2.0
    lines_a = generate_coverage_lines(reachable, origin, resolution, theta,
                                      spacing_m, min_len_m)
    lines_b = generate_coverage_lines(reachable, origin, resolution, theta_b,
                                      spacing_m, min_len_m)

    # Anchor the whole snake at the reachable region's corner nearest the robot,
    # so it always begins exactly in a room corner (closest to the walls) instead
    # of part-way along an edge.
    anchor = nearest_region_corner(reachable, origin, resolution, theta, start_xy)

    # Pick the direction to sweep first: the one whose first scan point (entered
    # from the robot) is closest to the robot, so the run does not begin by
    # crossing the room. Ties keep dir_a.
    a_from_start = order_serpentine(lines_a, theta, start_xy)
    b_from_start = order_serpentine(lines_b, theta_b, start_xy)

    def _entry_dist(ordered):
        return math.dist(start_xy, ordered[0][0]) if ordered else math.inf

    first_is_a = _entry_dist(a_from_start) <= _entry_dist(b_from_start)
    if first_is_a:
        first_lines, first_angle, second_lines, second_angle = lines_a, theta, lines_b, theta_b
    else:
        first_lines, first_angle, second_lines, second_angle = lines_b, theta_b, lines_a, theta

    # Strict snake: the first direction starts in the anchor corner and sweeps
    # edge-to-edge; the second direction begins near where the first ended so the
    # base never drives back across already-scanned space.
    first = order_serpentine(first_lines, first_angle, anchor)
    second_ref = first[-1][1] if first else anchor
    second = order_serpentine(second_lines, second_angle, second_ref)

    if first_is_a:
        dir_a, dir_b = first, second
    else:
        dir_a, dir_b = second, first

    segments = list(first) + list(second)
    segment_angles = [first_angle] * len(first) + [second_angle] * len(second)

    return {
        "angle": theta,
        "dir_a": dir_a,
        "dir_b": dir_b,
        "first_is_a": first_is_a,
        "segments": segments,
        "segment_angles": segment_angles,
    }


def _distinct_line_count(segments: Sequence[Segment], angle: float) -> int:
    """Number of distinct parallel lines (perpendicular offsets) in ``segments``.

    A single line split by obstacles yields several segments at the same offset,
    so counting offsets (not segments) measures how many lanes the sweep needs.
    """
    perp_x, perp_y = -math.sin(angle), math.cos(angle)
    offsets = {round(x0 * perp_x + y0 * perp_y, 3) for (x0, y0), _ in segments}
    return len(offsets)


def segment_rooms(reachable: np.ndarray, radius_cells: int) -> Tuple[np.ndarray, int]:
    """Split a reachable free mask into rooms separated by narrow passages.

    Eroding the free space by ``radius_cells`` pinches off any passage narrower
    than ~``2 * radius_cells`` (e.g. a doorway), so the eroded cores label as
    separate rooms. Every reachable cell is then assigned to its nearest core, so
    the rooms tile the whole region and the doorway is absorbed into the nearest
    room.

    Returns ``(labels, n_rooms)`` with ``labels`` in ``1..n_rooms`` on reachable
    cells and ``0`` elsewhere. Falls back to a single room when SciPy is
    unavailable, ``radius_cells <= 0``, or fewer than two cores survive erosion.
    """
    if not reachable.any():
        return np.zeros_like(reachable, dtype=np.int32), 0
    if radius_cells <= 0 or _ndimage is None:
        return reachable.astype(np.int32), 1

    structure = _disk_structure(int(radius_cells))
    core = _ndimage.binary_erosion(reachable, structure=structure, border_value=0)
    core_labels, n = _ndimage.label(core)
    if n <= 1:
        return reachable.astype(np.int32), 1

    # Assign every reachable cell to the nearest surviving core label.
    _, inds = _ndimage.distance_transform_edt(core_labels == 0, return_indices=True)
    nearest = core_labels[tuple(inds)]
    labels = np.where(reachable, nearest, 0).astype(np.int32)
    return labels, int(n)


def _room_centroids(labels: np.ndarray, room_ids: Sequence[int], origin: Point2,
                    resolution: float) -> dict:
    cents = {}
    for k in room_ids:
        idx = np.argwhere(labels == k)
        if idx.size == 0:
            continue
        cx = origin[0] + (float(idx[:, 1].mean()) + 0.5) * resolution
        cy = origin[1] + (float(idx[:, 0].mean()) + 0.5) * resolution
        cents[k] = (cx, cy)
    return cents


def _order_rooms(labels: np.ndarray, room_ids: Sequence[int], origin: Point2,
                 resolution: float, start_xy: Optional[Point2]) -> List[int]:
    """Greedy nearest-centroid room order, starting nearest the robot."""
    cents = _room_centroids(labels, room_ids, origin, resolution)
    remaining = [k for k in room_ids if k in cents]
    if not remaining:
        return []
    cur = start_xy if start_xy is not None else cents[remaining[0]]
    ordered: List[int] = []
    while remaining:
        k = min(remaining, key=lambda r: math.dist(cur, cents[r]))
        ordered.append(k)
        cur = cents[k]
        remaining.remove(k)
    return ordered


def build_single_sweep_plan(grid: np.ndarray, origin: Point2, resolution: float,
                            start_cell: Tuple[int, int], *, spacing_m: float,
                            inflation_m: float, min_len_m: float,
                            max_cost: int = _DEFAULT_MAX_COST,
                            angle: Optional[float] = None,
                            axis: str = "auto",
                            room_split_erosion_m: float = 0.0) -> dict:
    """One-direction serpentine coverage of the reachable free region.

    Unlike :func:`build_coverage_plan` (which sweeps BOTH perpendicular
    directions and therefore covers the room twice), this returns a SINGLE
    boustrophedon pass: the base visits every reachable cell once, in clean
    parallel lanes ``spacing_m`` apart, without doubling back. This is what an
    object-search sweep wants.

    Lanes are aligned to the dominant wall angle. ``axis`` selects which of the
    two wall-aligned directions to sweep:
      * ``"auto"`` (default): the direction with fewer lanes (i.e. lanes that run
        along the room's longer dimension -> fewer turns / shorter route);
      * ``"dominant"``: force the dominant wall angle ``theta``;
      * ``"perpendicular"``: force ``theta + 90 deg``.

    ``grid`` is a nav2 costmap; cells with cost above ``max_cost`` (inflation +
    obstacles) are untraversable, so lanes stay collision-free using the
    costmap's own inflation. ``inflation_m`` adds an OPTIONAL extra margin on top
    of that (raise it to keep lane endpoints further into free space, away from
    walls). ``start_cell`` is ``(row, col)``.

    ``room_split_erosion_m`` > 0 enables room-by-room coverage: the free space is
    split into rooms wherever a passage is narrower than ~``2 *
    room_split_erosion_m`` (e.g. a doorway), and each room is fully swept before
    moving to the next (rooms ordered nearest-first from the robot). This stops
    the base shuttling back and forth between rooms. 0 disables it (one global
    sweep).

    Returns ``{angle, segments, segment_angles, reachable, line_count,
    room_count}`` where ``segments`` is in execution order and ``segment_angles``
    is the parallel per-segment heading (all equal: a single sweep direction).
    """
    theta = dominant_wall_angle(grid) if angle is None else (angle % (math.pi / 2.0))

    blocked = blocked_mask(grid, max_cost)
    radius_cells = int(round(inflation_m / resolution)) if resolution > 0 else 0
    inflated = inflate_obstacles(blocked, radius_cells)

    free = free_mask(grid, max_cost) & ~inflated
    reachable = reachable_free_mask(free, start_cell)

    start_xy = (
        origin[0] + (start_cell[1] + 0.5) * resolution,
        origin[1] + (start_cell[0] + 0.5) * resolution,
    )

    # Pick the sweep direction once, globally, so lanes stay consistent across
    # rooms: fewer lanes (auto) means lanes run along the longer dimension.
    theta_a = theta
    theta_b = theta + math.pi / 2.0
    axis = (axis or "auto").strip().lower()
    if axis in ("dominant", "a"):
        chosen_angle = theta_a
    elif axis in ("perpendicular", "perp", "b"):
        chosen_angle = theta_b
    else:  # auto
        lines_a = generate_coverage_lines(reachable, origin, resolution, theta_a,
                                          spacing_m, min_len_m)
        lines_b = generate_coverage_lines(reachable, origin, resolution, theta_b,
                                          spacing_m, min_len_m)
        count_a = _distinct_line_count(lines_a, theta_a) if lines_a else math.inf
        count_b = _distinct_line_count(lines_b, theta_b) if lines_b else math.inf
        chosen_angle = theta_b if count_b < count_a else theta_a

    # Split into rooms (or a single room) and sweep each fully before the next.
    room_radius = int(round(room_split_erosion_m / resolution)) if resolution > 0 else 0
    room_labels, n_rooms = segment_rooms(reachable, room_radius)
    room_ids = [k for k in range(1, n_rooms + 1) if np.any(room_labels == k)]
    ordered_ids = _order_rooms(room_labels, room_ids, origin, resolution, start_xy)

    segments: List[Segment] = []
    current_xy = start_xy
    for k in ordered_ids:
        room_mask = (room_labels == k) & reachable
        room_lines = generate_coverage_lines(room_mask, origin, resolution,
                                             chosen_angle, spacing_m, min_len_m)
        room_route = order_serpentine(room_lines, chosen_angle, current_xy)
        if room_route:
            segments.extend(room_route)
            current_xy = room_route[-1][1]

    segment_angles = [chosen_angle] * len(segments)

    return {
        "angle": chosen_angle,
        "segments": segments,
        "segment_angles": segment_angles,
        "reachable": reachable,
        "line_count": _distinct_line_count(segments, chosen_angle) if segments else 0,
        "room_count": len(ordered_ids),
    }


# ----------------------------------------------------------------------------
# Perimeter (wall-following) coverage
# ----------------------------------------------------------------------------
def _outer_contour_cells(mask: np.ndarray) -> List[Tuple[int, int]]:
    """Ordered ``(row, col)`` cells tracing the outer boundary of ``mask``.

    Uses OpenCV's contour tracer when available (clean, ordered, handles the
    longest boundary), and a pure-numpy Moore-neighbour walk as a fallback.
    Only the outer boundary is returned; interior holes (columns) are ignored,
    which is what a single perimeter lap wants.
    """
    if not mask.any():
        return []
    m = mask.astype(np.uint8)
    if _cv2 is not None:
        contours, _ = _cv2.findContours(m, _cv2.RETR_EXTERNAL, _cv2.CHAIN_APPROX_NONE)
        if not contours:
            return []
        cnt = max(contours, key=_cv2.contourArea)
        # OpenCV points are (x=col, y=row); return (row, col).
        return [(int(p[0][1]), int(p[0][0])) for p in cnt]
    return _trace_boundary_moore(mask)


def _trace_boundary_moore(mask: np.ndarray) -> List[Tuple[int, int]]:
    """Moore-neighbour boundary trace (clockwise) with Jacob's stopping rule."""
    rows, cols = mask.shape
    idx = np.argwhere(mask)
    if idx.size == 0:
        return []
    # Start at the topmost, then leftmost, foreground cell.
    order = np.lexsort((idx[:, 1], idx[:, 0]))
    start = (int(idx[order[0], 0]), int(idx[order[0], 1]))

    def is_fg(r, c):
        return 0 <= r < rows and 0 <= c < cols and bool(mask[r, c])

    # Clockwise Moore neighbourhood offsets, starting from "west".
    nbrs = [(0, -1), (-1, -1), (-1, 0), (-1, 1),
            (0, 1), (1, 1), (1, 0), (1, -1)]
    boundary = [start]
    cur = start
    back_dir = 0  # came from the west of the start
    max_steps = 8 * int(mask.sum()) + 8
    for _ in range(max_steps):
        found = False
        for k in range(8):
            d = (back_dir + 1 + k) % 8
            nr, nc = cur[0] + nbrs[d][0], cur[1] + nbrs[d][1]
            if is_fg(nr, nc):
                # New backtrack direction points from the neighbour back to cur.
                back_dir = (d + 4) % 8
                cur = (nr, nc)
                found = True
                break
        if not found:
            break
        if cur == start and len(boundary) > 2:
            break
        boundary.append(cur)
    return boundary


def _resample_contour(contour_rc: Sequence[Tuple[int, int]], spacing_m: float,
                      resolution: float, rows: int, cols: int
                      ) -> List[Tuple[int, int]]:
    """Resample a closed contour to roughly ``spacing_m``-spaced ``(row, col)`` cells."""
    if len(contour_rc) < 3:
        return [tuple(map(int, p)) for p in contour_rc]
    spacing_cells = max(1.0, spacing_m / resolution) if resolution > 0 else 1.0
    pts = np.asarray(contour_rc, dtype=float)
    closed = np.vstack([pts, pts[0]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total <= 0.0:
        return [tuple(map(int, contour_rc[0]))]
    n = max(3, int(round(total / spacing_cells)))
    targets = np.linspace(0.0, total, n, endpoint=False)
    out: List[Tuple[int, int]] = []
    j = 0
    for t in targets:
        while j + 1 < len(cum) and cum[j + 1] < t:
            j += 1
        seg_len = cum[j + 1] - cum[j]
        frac = 0.0 if seg_len <= 0.0 else (t - cum[j]) / seg_len
        p = closed[j] * (1.0 - frac) + closed[j + 1] * frac
        r = min(max(int(round(p[0])), 0), rows - 1)
        c = min(max(int(round(p[1])), 0), cols - 1)
        if not out or (r, c) != out[-1]:
            out.append((r, c))
    return out


def _inward_headings(loop_rc: Sequence[Tuple[int, int]], mask: np.ndarray
                     ) -> List[float]:
    """Yaw (rad) facing the room interior at each boundary cell.

    The interior direction is the gradient of the distance transform of the
    free region (distance grows toward the room centre), lightly smoothed so the
    normal is stable along straight walls and rounds corners gracefully. Falls
    back to "point at the region centroid" when SciPy is unavailable or the
    gradient vanishes.
    """
    rows, cols = mask.shape
    idx = np.argwhere(mask)
    cen_r = float(idx[:, 0].mean()) if idx.size else 0.0
    cen_c = float(idx[:, 1].mean()) if idx.size else 0.0

    grad_y = grad_x = None
    if _ndimage is not None:
        dist = _ndimage.distance_transform_edt(mask)
        dist = _ndimage.gaussian_filter(dist, sigma=2.0)
        grad_y, grad_x = np.gradient(dist)  # d/drow (+y), d/dcol (+x)

    headings: List[float] = []
    for r, c in loop_rc:
        r = min(max(int(r), 0), rows - 1)
        c = min(max(int(c), 0), cols - 1)
        if grad_x is not None:
            vx = float(grad_x[r, c])
            vy = float(grad_y[r, c])
        else:
            vx = vy = 0.0
        if math.hypot(vx, vy) < 1e-6:
            vx, vy = cen_c - c, cen_r - r  # point at centroid
        headings.append(math.atan2(vy, vx))
    return headings


def _rotate_loop_to_nearest(loop_rc: List[Tuple[int, int]], ref_xy: Point2,
                            origin: Point2, resolution: float
                            ) -> List[Tuple[int, int]]:
    """Rotate a closed loop so it starts at the cell nearest ``ref_xy``."""
    if not loop_rc:
        return loop_rc
    rx, ry = ref_xy
    best_i, best_d = 0, math.inf
    for i, (r, c) in enumerate(loop_rc):
        x = origin[0] + (c + 0.5) * resolution
        y = origin[1] + (r + 0.5) * resolution
        d = (x - rx) ** 2 + (y - ry) ** 2
        if d < best_d:
            best_i, best_d = i, d
    return loop_rc[best_i:] + loop_rc[:best_i]


def build_perimeter_plan(grid: np.ndarray, origin: Point2, resolution: float,
                         start_cell: Tuple[int, int], *, spacing_m: float,
                         inflation_m: float, max_cost: int = _DEFAULT_MAX_COST,
                         angle: Optional[float] = None,
                         room_split_erosion_m: float = 0.0,
                         min_loop_len_m: float = 1.5) -> dict:
    """Single wall-following lap of the reachable free region's boundary.

    Instead of slicing the room into parallel lanes (see
    :func:`build_single_sweep_plan`), this traces ONE loop along the costmap
    edge — i.e. just inside the walls and inflation layer — and orients the base
    to face the room interior at every waypoint, so a forward-facing camera looks
    across the open space (long range) rather than at the adjacent wall.

    The drive ring keeps clear of the walls by the costmap's own inflation plus
    the optional extra ``inflation_m`` margin (raise it to stand further off the
    wall). ``spacing_m`` is the along-loop waypoint spacing. With
    ``room_split_erosion_m`` > 0 each room is lapped separately (nearest-first),
    so the base does not weave between rooms through a doorway.

    Coverage is the perimeter only: a single lap does NOT visit the centre of a
    room wider than ~2x the camera range. Use it for corridors / narrow spaces,
    or combine with a serpentine for large halls. ``start_cell`` is ``(row, col)``.

    Returns ``{angle, waypoints, segments, reachable, loop_count, room_count}``
    where ``waypoints`` is an ordered list of ``(x, y, yaw)`` (yaw faces the
    interior) and ``segments`` are consecutive point pairs for marker reuse.
    """
    theta = dominant_wall_angle(grid) if angle is None else (angle % (math.pi / 2.0))

    blocked = blocked_mask(grid, max_cost)
    radius_cells = int(round(inflation_m / resolution)) if resolution > 0 else 0
    inflated = inflate_obstacles(blocked, radius_cells)

    free = free_mask(grid, max_cost) & ~inflated
    reachable = reachable_free_mask(free, start_cell)
    rows, cols = reachable.shape

    start_xy = (
        origin[0] + (start_cell[1] + 0.5) * resolution,
        origin[1] + (start_cell[0] + 0.5) * resolution,
    )

    room_radius = int(round(room_split_erosion_m / resolution)) if resolution > 0 else 0
    room_labels, n_rooms = segment_rooms(reachable, room_radius)
    room_ids = [k for k in range(1, n_rooms + 1) if np.any(room_labels == k)]
    ordered_ids = _order_rooms(room_labels, room_ids, origin, resolution, start_xy)

    waypoints: List[Tuple[float, float, float]] = []
    segments: List[Segment] = []
    current_xy = start_xy
    loop_count = 0

    for k in ordered_ids:
        room_mask = (room_labels == k) & reachable
        contour_rc = _outer_contour_cells(room_mask)
        if len(contour_rc) < 3:
            continue
        loop_rc = _resample_contour(contour_rc, spacing_m, resolution, rows, cols)
        if len(loop_rc) < 3:
            continue
        # Drop tiny loops (slivers / noise) below the minimum lap length.
        perim_m = sum(
            math.hypot(loop_rc[i][0] - loop_rc[i - 1][0], loop_rc[i][1] - loop_rc[i - 1][1])
            for i in range(len(loop_rc))
        ) * resolution
        if perim_m < min_loop_len_m:
            continue

        loop_rc = _rotate_loop_to_nearest(loop_rc, current_xy, origin, resolution)
        headings = _inward_headings(loop_rc, room_mask)

        # Close the lap by returning to the entry cell.
        loop_rc_closed = loop_rc + [loop_rc[0]]
        headings_closed = headings + [headings[0]]

        pts_xy = [
            (origin[0] + (c + 0.5) * resolution, origin[1] + (r + 0.5) * resolution)
            for r, c in loop_rc_closed
        ]
        for (x, y), h in zip(pts_xy, headings_closed):
            waypoints.append((float(x), float(y), float(h)))
        for i in range(len(pts_xy) - 1):
            segments.append((pts_xy[i], pts_xy[i + 1]))

        current_xy = pts_xy[-1]
        loop_count += 1

    return {
        "angle": theta,
        "waypoints": waypoints,
        "segments": segments,
        "reachable": reachable,
        "loop_count": loop_count,
        "room_count": len(ordered_ids),
    }
