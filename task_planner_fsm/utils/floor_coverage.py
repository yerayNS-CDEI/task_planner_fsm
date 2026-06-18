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
