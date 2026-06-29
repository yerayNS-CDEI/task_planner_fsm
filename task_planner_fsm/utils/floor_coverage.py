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
# Waypoint placement mask + pocket-coverage fallback
# ----------------------------------------------------------------------------
def placement_mask(grid: np.ndarray, reachable: np.ndarray,
                   waypoint_max_cost: int) -> np.ndarray:
    """Cells inside ``reachable`` that are clear enough to host a waypoint.

    ``reachable`` is computed with the permissive *traversable* threshold (so it
    spans everything nav2 can drive to, including space joined only through the
    inflation layer). This narrows it to cells whose cost is within
    ``waypoint_max_cost`` -- typically 0, i.e. genuinely free space -- so points
    land just inside the free/inflation seam instead of within the inflation band.
    """
    return reachable & (grid >= 0) & (grid <= int(waypoint_max_cost))


def _representative_cell(component: np.ndarray) -> Optional[Tuple[int, int]]:
    """Most-interior ``(row, col)`` of one connected component (deepest cell).

    Uses the distance transform so the point sits as far from the component's
    edge as possible; falls back to the cell nearest the centroid when SciPy is
    unavailable. Returns ``None`` for an empty component.
    """
    idx = np.argwhere(component)
    if idx.size == 0:
        return None
    if _ndimage is not None:
        dist = _ndimage.distance_transform_edt(component)
        flat = int(np.argmax(dist))
        r, c = np.unravel_index(flat, component.shape)
        return int(r), int(c)
    cr, cc = float(idx[:, 0].mean()), float(idx[:, 1].mean())
    d2 = (idx[:, 0] - cr) ** 2 + (idx[:, 1] - cc) ** 2
    r, c = idx[int(np.argmin(d2))]
    return int(r), int(c)


def uncovered_pocket_cells(placement: np.ndarray,
                           covered_cells: Sequence[Tuple[int, int]]
                           ) -> List[Tuple[int, int]]:
    """One representative ``(row, col)`` per placement component left uncovered.

    Labels the connected components of ``placement`` and returns a single
    interior cell for every component that contains none of ``covered_cells``
    (the cells already touched by the generated route). This guarantees that
    every patch of free space -- however small -- receives at least one
    waypoint, since being free means the base reached it during mapping.
    """
    if not placement.any():
        return []

    if _ndimage is not None:
        labels, n = _ndimage.label(placement)
    else:  # graceful: treat the whole mask as a single component
        labels = placement.astype(np.int32)
        n = 1

    rows, cols = placement.shape
    covered_labels = set()
    for r, c in covered_cells:
        if 0 <= r < rows and 0 <= c < cols:
            lab = int(labels[r, c])
            if lab > 0:
                covered_labels.add(lab)

    reps: List[Tuple[int, int]] = []
    for lab in range(1, n + 1):
        if lab in covered_labels:
            continue
        cell = _representative_cell(labels == lab)
        if cell is not None:
            reps.append(cell)
    return reps


def _segment_covered_cells(segments: Sequence[Segment], origin: Point2,
                           resolution: float, shape: Tuple[int, int]
                           ) -> List[Tuple[int, int]]:
    """``(row, col)`` cells touched by each segment's endpoints and midpoint."""
    rows, cols = shape
    cells: List[Tuple[int, int]] = []
    for (x0, y0), (x1, y1) in segments:
        for x, y in ((x0, y0), (x1, y1), (0.5 * (x0 + x1), 0.5 * (y0 + y1))):
            c = int((x - origin[0]) / resolution)
            r = int((y - origin[1]) / resolution)
            if 0 <= r < rows and 0 <= c < cols:
                cells.append((r, c))
    return cells


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
    progresses line by line and never skips a line, only to come back for it
    later). Each LINE is then traversed strictly monotonically along the sweep
    axis: the base enters the line from the end nearest its current position and
    runs to the far end. A line split by obstacles into several collinear pieces is
    swept low->high (or high->low) as one run, so the base never enters a piece
    from its middle and doubles back across it. Consecutive lines alternate
    direction, reproducing a clean boustrophedon.

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

    def along_lo(seg: Segment) -> float:
        return min(along_of(seg[0]), along_of(seg[1]))

    out: List[Segment] = []
    for off in sorted_offsets:
        pieces = list(groups[off])

        # Sweep the whole line monotonically. Pick the direction (low->high or
        # high->low) by whichever end of the line is nearer the current position,
        # then emit every collinear piece in that order, each oriented to enter at
        # its near end. This stops the base entering a split line from the middle
        # (between two pieces) and crossing back over points it already passed --
        # the cause of the back-and-forth loops.
        line_lo = min(along_lo(s) for s in pieces)
        line_hi = max(max(along_of(s[0]), along_of(s[1])) for s in pieces)
        if cur is None:
            ascending = True  # no reference yet: sweep from the lower-along end
        else:
            cur_along = along_of(cur)
            ascending = abs(cur_along - line_lo) <= abs(cur_along - line_hi)

        pieces.sort(key=along_lo, reverse=not ascending)
        for p0, p1 in pieces:
            enters_low = along_of(p0) <= along_of(p1)
            if ascending != enters_low:
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
                            waypoint_max_cost: Optional[int] = None,
                            ensure_pocket_coverage: bool = False,
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
      * ``"auto"`` (default, aliases ``"fewer"``/``"long"``): the direction with
        fewer lanes (lanes run along the room's longer dimension -> fewest turns);
      * ``"more"`` (aliases ``"short"``/``"cross"``): the OTHER one -- more, shorter
        lanes running across the long dimension. Use it to try the opposite of auto;
      * ``"dominant"``/``"a"``: force the dominant wall angle ``theta``;
      * ``"perpendicular"``/``"perp"``/``"b"``: force ``theta + 90 deg``.

    ``grid`` is a nav2 costmap; cells with cost above ``max_cost`` (inflation +
    obstacles) are untraversable, so lanes stay collision-free using the
    costmap's own inflation. ``inflation_m`` adds an OPTIONAL extra margin on top
    of that (raise it to keep lane endpoints further into free space, away from
    walls). ``start_cell`` is ``(row, col)``.

    ``max_cost`` and ``waypoint_max_cost`` serve two different jobs. ``max_cost``
    is the *traversable* threshold: it decides which cells count as drivable for
    the reachability flood-fill, so keep it permissive (it may include the
    inflation band) to keep narrow-necked space connected. ``waypoint_max_cost``
    (default: same as ``max_cost``) is the *placement* threshold: lanes are only
    laid on cells within it. Set it low (e.g. 0) to keep waypoints in genuine
    free space -- just inside the free/inflation seam -- while still reaching
    pockets joined to the room only through inflation.

    ``ensure_pocket_coverage`` guarantees that every connected patch of placement
    (free) space gets at least one waypoint: after the sweep, any free component
    the lanes missed (e.g. one too small for a lane of length ``min_len_m``)
    receives a single interior point, returned under ``extra_points``.

    ``room_split_erosion_m`` > 0 enables room-by-room coverage: the free space is
    split into rooms wherever a passage is narrower than ~``2 *
    room_split_erosion_m`` (e.g. a doorway), and each room is fully swept before
    moving to the next (rooms ordered nearest-first from the robot). This stops
    the base shuttling back and forth between rooms. 0 disables it (one global
    sweep).

    Returns ``{angle, segments, segment_angles, extra_points, reachable,
    placement, line_count, room_count}`` where ``segments`` is in execution order,
    ``segment_angles`` is the parallel per-segment heading (all equal: a single
    sweep direction), and ``extra_points`` is a list of ``(x, y)`` fallback points
    (empty unless ``ensure_pocket_coverage``).
    """
    theta = dominant_wall_angle(grid) if angle is None else (angle % (math.pi / 2.0))

    blocked = blocked_mask(grid, max_cost)
    radius_cells = int(round(inflation_m / resolution)) if resolution > 0 else 0
    inflated = inflate_obstacles(blocked, radius_cells)

    free = free_mask(grid, max_cost) & ~inflated
    reachable = reachable_free_mask(free, start_cell)

    # Waypoints land only on placement cells (default: same as the traversable
    # set, so behaviour is unchanged unless waypoint_max_cost is lowered).
    wp_max_cost = max_cost if waypoint_max_cost is None else int(waypoint_max_cost)
    placement = placement_mask(grid, reachable, wp_max_cost) & ~inflated

    start_xy = (
        origin[0] + (start_cell[1] + 0.5) * resolution,
        origin[1] + (start_cell[0] + 0.5) * resolution,
    )

    # Pick the sweep direction once, globally, so lanes stay consistent across
    # rooms. The two wall-aligned options are theta_a (dominant) and theta_b
    # (perpendicular). ``axis`` selects between them:
    #   * "auto"/"fewer"/"long": fewer lanes -> lanes run along the longer
    #     dimension (longest lanes, fewest turns). Default.
    #   * "more"/"short"/"cross": the OTHER one -> more, shorter lanes running
    #     across the long dimension. Use this to try the opposite of "auto".
    #   * "dominant"/"a": force theta_a.   "perpendicular"/"perp"/"b": force theta_b.
    theta_a = theta
    theta_b = theta + math.pi / 2.0
    axis = (axis or "auto").strip().lower()
    if axis in ("dominant", "a"):
        chosen_angle = theta_a
    elif axis in ("perpendicular", "perp", "b"):
        chosen_angle = theta_b
    else:  # data-driven: compare how many lanes each direction needs
        lines_a = generate_coverage_lines(placement, origin, resolution, theta_a,
                                          spacing_m, min_len_m)
        lines_b = generate_coverage_lines(placement, origin, resolution, theta_b,
                                          spacing_m, min_len_m)
        count_a = _distinct_line_count(lines_a, theta_a) if lines_a else math.inf
        count_b = _distinct_line_count(lines_b, theta_b) if lines_b else math.inf
        fewer_angle = theta_b if count_b < count_a else theta_a
        more_angle = theta_a if count_b < count_a else theta_b
        if axis in ("more", "short", "shorter", "cross"):
            chosen_angle = more_angle
        else:  # auto / fewer / long / longer
            chosen_angle = fewer_angle

    # Anchor the sweep at the reachable region's corner nearest the robot, so it
    # begins in a corner (hugging the walls) -- exactly like ScanFloor's
    # build_coverage_plan -- instead of part-way along an edge.
    anchor = nearest_region_corner(reachable, origin, resolution, chosen_angle, start_xy)

    # Split into rooms only if asked (room_split_erosion_m > 0). With it disabled
    # (the default for densify) segment_rooms returns a SINGLE room spanning the
    # whole reachable region, so this is one global serpentine -- the same
    # computation ScanFloor does, just for one direction. Room membership uses the
    # full reachable region (so a pocket reachable only through inflation still
    # belongs to its room); lanes are restricted to placement cells within it.
    room_radius = int(round(room_split_erosion_m / resolution)) if resolution > 0 else 0
    room_labels, n_rooms = segment_rooms(reachable, room_radius)
    room_ids = [k for k in range(1, n_rooms + 1) if np.any(room_labels == k)]
    ordered_ids = _order_rooms(room_labels, room_ids, origin, resolution, start_xy)

    segments: List[Segment] = []
    current_xy = anchor
    for k in ordered_ids:
        room_mask = (room_labels == k) & placement
        room_lines = generate_coverage_lines(room_mask, origin, resolution,
                                             chosen_angle, spacing_m, min_len_m)
        room_route = order_serpentine(room_lines, chosen_angle, current_xy)
        if room_route:
            segments.extend(room_route)
            current_xy = room_route[-1][1]

    segment_angles = [chosen_angle] * len(segments)

    # Guarantee a point in every free pocket the lanes missed.
    extra_points: List[Point2] = []
    if ensure_pocket_coverage:
        covered = _segment_covered_cells(segments, origin, resolution, placement.shape)
        for r, c in uncovered_pocket_cells(placement, covered):
            extra_points.append((
                origin[0] + (c + 0.5) * resolution,
                origin[1] + (r + 0.5) * resolution,
            ))

    return {
        "angle": chosen_angle,
        "segments": segments,
        "segment_angles": segment_angles,
        "extra_points": extra_points,
        "reachable": reachable,
        "placement": placement,
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


def _snap_cell_to_mask(r: int, c: int, mask: np.ndarray) -> Tuple[int, int]:
    """Nearest ``(row, col)`` that is True in ``mask`` (the cell itself if already True).

    Guards against contour resampling rounding a point a cell off the free
    region; expands a small window first (the common case is a 1-cell miss) and
    only falls back to a full search if needed.
    """
    rows, cols = mask.shape
    r = min(max(int(r), 0), rows - 1)
    c = min(max(int(c), 0), cols - 1)
    if mask[r, c]:
        return (r, c)
    for rad in (1, 2, 3):
        r0, r1 = max(0, r - rad), min(rows, r + rad + 1)
        c0, c1 = max(0, c - rad), min(cols, c + rad + 1)
        sub = np.argwhere(mask[r0:r1, c0:c1])
        if sub.size:
            d2 = (sub[:, 0] + r0 - r) ** 2 + (sub[:, 1] + c0 - c) ** 2
            rr, cc = sub[int(np.argmin(d2))]
            return (int(rr + r0), int(cc + c0))
    idx = np.argwhere(mask)
    d2 = (idx[:, 0] - r) ** 2 + (idx[:, 1] - c) ** 2
    rr, cc = idx[int(np.argmin(d2))]
    return (int(rr), int(cc))


def _free_components(placement: np.ndarray) -> Tuple[np.ndarray, List[int]]:
    """Label the connected components of the placement mask.

    Each component is a contiguous patch of free space (a room, a pocket, ...).
    Falls back to a single component when SciPy is unavailable.
    """
    if not placement.any():
        return np.zeros_like(placement, dtype=np.int32), []
    if _ndimage is not None:
        labels, n = _ndimage.label(placement)
    else:  # graceful: whole mask as one component
        labels = placement.astype(np.int32)
        n = 1
    return labels, [k for k in range(1, n + 1) if np.any(labels == k)]


def _greedy_nearest_route(points: Sequence[Tuple[float, float, float]],
                          start_xy: Point2) -> List[Tuple[float, float, float]]:
    """Order ``(x, y, yaw)`` points by always taking the nearest remaining one.

    A pure greedy nearest-neighbour tour from ``start_xy``: at each step the
    closest unvisited point to the current position is appended. This ignores the
    boundary/lap structure entirely, so the base always drives to its nearest
    point next (at the cost of no longer following a clean wall-hugging lap).
    """
    remaining = list(points)
    out: List[Tuple[float, float, float]] = []
    cx, cy = start_xy
    while remaining:
        best_i, best_d = 0, math.inf
        for i, (x, y, _) in enumerate(remaining):
            d = (x - cx) ** 2 + (y - cy) ** 2
            if d < best_d:
                best_i, best_d = i, d
        p = remaining.pop(best_i)
        out.append(p)
        cx, cy = p[0], p[1]
    return out


def _two_opt_route(points: Sequence[Tuple[float, float, float]], start_xy: Point2,
                   max_passes: int = 20) -> List[Tuple[float, float, float]]:
    """2-opt refinement of an open route from ``start_xy`` through all ``points``.

    Greedy nearest-neighbour can start in the middle of a row of points, drive out
    to one endpoint and then double back across the middle to reach the other --
    an edge crossing. 2-opt repeatedly reverses any sub-path whose reversal makes
    the total route shorter, which removes such crossings and leaves a sweep that
    enters a near-collinear run at its nearest endpoint. ``start_xy`` is a fixed
    anchor (the robot) and is never reordered. Runs until no improvement or
    ``max_passes`` is reached (O(n^2) per pass; n is small here).
    """
    if len(points) < 3:
        return list(points)

    # Index 0 is the fixed start anchor; 1..n are the movable waypoints.
    nodes: List[Tuple[float, float]] = [start_xy] + [(p[0], p[1]) for p in points]
    order = list(range(1, len(nodes)))
    n = len(order)

    def dist(a: int, b: int) -> float:
        return math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])

    for _ in range(max_passes):
        improved = False
        for i in range(n):
            prev = 0 if i == 0 else order[i - 1]          # predecessor (anchor if first)
            for j in range(i + 1, n):
                a, b = order[i], order[j]
                # Edge removed before the segment: (prev, a); after: (b, next).
                if j + 1 < n:
                    nxt = order[j + 1]
                    delta = (dist(prev, b) + dist(a, nxt)
                             - dist(prev, a) - dist(b, nxt))
                else:  # reversing the tail: only the leading edge changes
                    delta = dist(prev, b) - dist(prev, a)
                if delta < -1e-9:
                    order[i:j + 1] = order[i:j + 1][::-1]
                    improved = True
        if not improved:
            break

    return [points[k - 1] for k in order]


def build_perimeter_plan(grid: np.ndarray, origin: Point2, resolution: float,
                         start_cell: Tuple[int, int], *, spacing_m: float,
                         inflation_m: float, max_cost: int = _DEFAULT_MAX_COST,
                         waypoint_max_cost: Optional[int] = None,
                         ensure_pocket_coverage: bool = True,
                         angle: Optional[float] = None,
                         min_loop_len_m: float = 1.5,
                         min_point_dist_m: Optional[float] = None,
                         drop_start_waypoint: bool = True,
                         greedy_nearest: bool = True) -> dict:
    """Wall-following lap of every free-space patch, traced on genuine free cells.

    The free (placement) space is split into its natural connected components --
    each room, corridor or pocket -- and they are visited nearest-first from the
    robot. A component large enough gets a single boundary lap (resampled to
    ``spacing_m``, base facing the interior so a forward camera looks across the
    open space); a component too small for a lap of length ``min_loop_len_m`` gets
    one interior point instead, so no free patch is ever skipped. This replaces
    erosion-based room splitting -- the components do the splitting for free.

    Two cost thresholds keep the points where they belong:
      * ``max_cost`` -- *traversable* threshold for the reachability flood-fill.
        Keep it permissive (it may include the inflation band) so a patch joined
        to the room only through inflation still counts as reachable.
      * ``waypoint_max_cost`` (default: same as ``max_cost``) -- *placement*
        threshold the laps are traced on. Set it to 0 to keep every waypoint on
        true free cells, sitting just inside the free/inflation seam rather than
        within the inflation band. ``inflation_m`` adds an OPTIONAL extra standoff:
        it erodes the placement region inward from the free/inflation seam, so a
        small value (e.g. 0.1-0.2 m) lifts the lap a little clear of the inflation
        layer instead of hugging it. 0 keeps the lap right on the seam.

    ``ensure_pocket_coverage`` (default True) controls whether sub-``min_loop_len_m``
    components still get their single interior point; set False to drop them.
    ``start_cell`` is ``(row, col)``.

    ``min_point_dist_m`` (default: ``0.5 * spacing_m``) is the minimum spacing kept
    between waypoints. Resampling a ragged or thin cost-0 boundary, plus snapping
    samples back onto free cells, can otherwise drop several points onto the same
    or adjacent cells; those near-duplicates make the base shuffle back and forth.
    A point is discarded when it lands within this distance of ANY waypoint kept so
    far -- not just the previous one -- so non-consecutive near-duplicates (a lap
    passing near itself, or two rooms' boundaries meeting) are removed too, as is
    the lap-closing return point. Set it to 0 to keep every resampled point.

    ``drop_start_waypoint`` (default True) discards any leading waypoint that lands
    within ``min_point_dist_m`` of the robot's start pose, since the first lap point
    is the boundary cell nearest the robot and is often right where it already
    stands -- navigating to it is a no-op. With it dropped, the route begins at the
    first point the base actually has to move to. Set False to keep that point.

    ``greedy_nearest`` (default True) re-orders the whole point set as a greedy
    nearest-neighbour tour from the robot, then refines it with 2-opt: every move
    goes to a near waypoint and edge crossings are removed, so a near-collinear row
    is entered at its nearest endpoint rather than from the middle. This minimises
    "skip the nearby point" hops and back-tracking, at the cost of no longer
    following clean wall-hugging laps. Set False to keep the per-component
    boundary-lap ordering.

    Returns ``{angle, waypoints, segments, reachable, placement, loop_count,
    pocket_count, room_count}`` where ``waypoints`` is an ordered list of
    ``(x, y, yaw)`` (yaw faces the interior) and ``segments`` are consecutive
    point pairs for marker reuse.
    """
    theta = dominant_wall_angle(grid) if angle is None else (angle % (math.pi / 2.0))

    blocked = blocked_mask(grid, max_cost)
    radius_cells = int(round(inflation_m / resolution)) if resolution > 0 else 0
    inflated = inflate_obstacles(blocked, radius_cells)

    free = free_mask(grid, max_cost) & ~inflated
    reachable = reachable_free_mask(free, start_cell)
    rows, cols = reachable.shape

    # Minimum spacing (in cells) enforced between consecutive lap waypoints, so
    # resampling/snapping near-duplicates do not make the base shuffle in place.
    # Floor of 1 cell still removes exact duplicates even when the request is 0.
    min_dist_m = 0.5 * spacing_m if min_point_dist_m is None else float(min_point_dist_m)
    min_sep_cells = max(1.0, min_dist_m / resolution) if resolution > 0 else 1.0
    min_sep_sq = min_sep_cells * min_sep_cells
    # Same threshold in world units, used to drop a waypoint that lands within
    # ``min_dist_m`` of ANY already-kept waypoint (not just the previous one), so
    # non-consecutive near-duplicates -- where a lap passes near itself or two
    # rooms' boundaries meet -- are removed too.
    min_dist_sq_m = min_dist_m * min_dist_m

    # Waypoints are placed only on genuine free cells (default: same as the
    # traversable set, so behaviour is unchanged unless waypoint_max_cost is set).
    wp_max_cost = max_cost if waypoint_max_cost is None else int(waypoint_max_cost)
    placement = placement_mask(grid, reachable, wp_max_cost)

    # Clearance from the inflation layer: erode the placement region away from the
    # free/inflation SEAM (any cell above wp_max_cost, plus unknown), not just away
    # from the inscribed obstacle. Cost-0 cells already sit at nav2's full inflation
    # radius, so eroding against the inscribed obstacle would need a clearance
    # larger than that radius to have any effect; eroding against the seam makes a
    # small ``inflation_m`` immediately pull the lap a little inside the seam.
    if radius_cells > 0:
        seam_blocked = (grid > wp_max_cost) | (grid < 0)
        placement = placement & ~inflate_obstacles(seam_blocked, radius_cells)

    start_xy = (
        origin[0] + (start_cell[1] + 0.5) * resolution,
        origin[1] + (start_cell[0] + 0.5) * resolution,
    )

    # Each free-space component is a room/pocket. World coords of every cell per
    # component, used to visit them nearest-first by their closest cell to the
    # robot (not by centroid, which for a big enclosing ring sits in the middle).
    labels, comp_ids = _free_components(placement)
    comp_xy = {}
    for k in comp_ids:
        idx = np.argwhere(labels == k)
        comp_xy[k] = (
            origin[0] + (idx[:, 1] + 0.5) * resolution,
            origin[1] + (idx[:, 0] + 0.5) * resolution,
        )

    waypoints: List[Tuple[float, float, float]] = []
    segments: List[Segment] = []
    accepted_xy: List[Point2] = []  # every kept waypoint, for global dedup
    # Seeding the start pose makes the global filter drop a leading waypoint that
    # coincides with the robot, so the route begins at the first point it must
    # actually drive to instead of "navigating" to where it already stands.
    if drop_start_waypoint:
        accepted_xy.append(start_xy)
    current_xy = start_xy
    loop_count = 0
    pocket_count = 0

    def _far_enough(x: float, y: float) -> bool:
        """True if (x, y) is at least ``min_dist_m`` from every kept waypoint."""
        for ax, ay in accepted_xy:
            if (x - ax) ** 2 + (y - ay) ** 2 < min_dist_sq_m:
                return False
        return True

    remaining = list(comp_ids)
    while remaining:
        # Visit next the component whose nearest cell to the current position is
        # closest, recomputed from where the previous lap actually ended -- so the
        # route always heads to the nearest unvisited region instead of doubling
        # back as a precomputed, centroid-based order could.
        best_k, best_d = remaining[0], math.inf
        for cand in remaining:
            xs, ys = comp_xy[cand]
            d = float(np.min((xs - current_xy[0]) ** 2 + (ys - current_xy[1]) ** 2))
            if d < best_d:
                best_d, best_k = d, cand
        k = best_k
        remaining.remove(k)
        comp = labels == k

        # Trace the component boundary and snap every sample back onto a free
        # cell, keeping only points at least ``min_sep_cells`` from the previous
        # kept one (drops near-duplicates that snapping/resampling introduce).
        contour_rc = _outer_contour_cells(comp)
        loop_rc: List[Tuple[int, int]] = []
        if len(contour_rc) >= 3:
            for r, c in _resample_contour(contour_rc, spacing_m, resolution, rows, cols):
                cell = _snap_cell_to_mask(r, c, comp)
                if not loop_rc:
                    loop_rc.append(cell)
                    continue
                dr = cell[0] - loop_rc[-1][0]
                dc = cell[1] - loop_rc[-1][1]
                if dr * dr + dc * dc >= min_sep_sq:
                    loop_rc.append(cell)

        perim_m = resolution * sum(
            math.hypot(loop_rc[i][0] - loop_rc[i - 1][0], loop_rc[i][1] - loop_rc[i - 1][1])
            for i in range(len(loop_rc))
        ) if len(loop_rc) >= 3 else 0.0

        if len(loop_rc) >= 3 and perim_m >= min_loop_len_m:
            loop_rc = _rotate_loop_to_nearest(loop_rc, current_xy, origin, resolution)
            headings = _inward_headings(loop_rc, comp)

            pts_xy = [
                (origin[0] + (c + 0.5) * resolution, origin[1] + (r + 0.5) * resolution)
                for r, c in loop_rc
            ]
            # Keep only points far enough from every waypoint kept so far (this
            # also drops the lap-closing return point, which coincides with the
            # entry). Segments connect consecutive kept points, closing the lap.
            lap_kept: List[Point2] = []
            for (x, y), h in zip(pts_xy, headings):
                if not _far_enough(x, y):
                    continue
                waypoints.append((float(x), float(y), float(h)))
                accepted_xy.append((x, y))
                lap_kept.append((x, y))
            for i in range(len(lap_kept) - 1):
                segments.append((lap_kept[i], lap_kept[i + 1]))
            if len(lap_kept) >= 3:
                segments.append((lap_kept[-1], lap_kept[0]))  # close the lap
            if lap_kept:
                current_xy = lap_kept[-1]
                loop_count += 1

        elif ensure_pocket_coverage:
            # Too small for a lap: drop one interior point so it is still visited,
            # unless that patch is already covered by a nearby kept waypoint.
            cell = _representative_cell(comp)
            if cell is None:
                continue
            r, c = cell
            x = origin[0] + (c + 0.5) * resolution
            y = origin[1] + (r + 0.5) * resolution
            if not _far_enough(x, y):
                continue
            yaw = _inward_headings([(r, c)], comp)[0]
            waypoints.append((float(x), float(y), float(yaw)))
            accepted_xy.append((x, y))
            current_xy = (x, y)
            pocket_count += 1

    # Optionally discard the lap structure and visit every point greedily nearest-
    # first from the robot, rebuilding the marker segments to match the new order.
    if greedy_nearest and waypoints:
        waypoints = _greedy_nearest_route(waypoints, start_xy)
        # 2-opt cleanup: removes crossings where greedy started mid-run and doubled
        # back, so a near-collinear row is entered at its nearest endpoint instead.
        waypoints = _two_opt_route(waypoints, start_xy)
        segments = [
            ((waypoints[i][0], waypoints[i][1]), (waypoints[i + 1][0], waypoints[i + 1][1]))
            for i in range(len(waypoints) - 1)
        ]

    return {
        "angle": theta,
        "waypoints": waypoints,
        "segments": segments,
        "reachable": reachable,
        "placement": placement,
        "loop_count": loop_count,
        "pocket_count": pocket_count,
        "room_count": len(comp_ids),
    }
