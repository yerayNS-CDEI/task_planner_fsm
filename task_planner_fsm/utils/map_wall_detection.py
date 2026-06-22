"""Wall- and column-feature detection from a nav2 / rtabmap occupancy grid.

Pure geometry: no ROS objects. Given a decoded occupancy grid (2D int array +
map-frame origin + resolution) this extracts the building's straight wall
segments and its free-standing columns/obstacles, all in the map frame.

The grid follows the ``nav_msgs/OccupancyGrid`` convention: values are -1
(unknown), 0..100 (occupancy probability), arranged row-major with ``grid[r, c]``
at map-frame point ``(origin_x + (c + 0.5) * res, origin_y + (r + 0.5) * res)``.

Why structure-aware (not raw Hough): running Hough on the occupancy pixels has
no notion of *structure*, so it (a) leaves the two faces of a thick wall as
duplicate collinear lines, (b) fires several short lines on each compact column
blob, and (c) bridges real gaps so walls that should be separate get joined.
This pipeline instead reasons about connected components:

  1. CLEAN     binarize at the occupied threshold; morphological open (kill
               speckle) then close (seal hairline gaps).
  2. LABEL     connected-component labelling of the occupied cells.
  3. CLASSIFY  per component, measure its oriented bounding box:
                 - small + compact (low aspect) + solid -> COLUMN (stored
                   separately); the aspect gate keeps short wall fragments out
                 - thin + straight + elongated -> WALL, taken as the box
                   CENTERLINE (one clean segment, no duplicate faces)
                 - anything larger/complex (e.g. the connected perimeter) ->
                   WALL, traced with a contour + ``approxPolyDP`` so it breaks
                   into one segment per face and SPLITS AT CORNERS.
  4. FILTER    drop sub-threshold-length walls; a light, conservative collinear
               merge cleans up tiny over-splits without bridging real gaps.

``Segment`` is ``((x0, y0), (x1, y1))`` in the map frame. ``Column`` is a dict
``{"center": (x, y), "radius_m": float, "area_m2": float}``.
"""

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

try:  # OpenCV powers the morphology / labelling / contour ops; stay graceful.
    import cv2 as _cv2
except Exception:  # pragma: no cover - exercised only when OpenCV is absent.
    _cv2 = None

Point2 = Tuple[float, float]
Segment = Tuple[Point2, Point2]
Column = Dict[str, object]

# Default occupancy probability at/above which a cell is treated as occupied.
_DEFAULT_OCCUPIED_THRESH = 65


def opencv_available() -> bool:
    """True when OpenCV could be imported (required for detection)."""
    return _cv2 is not None


# ----------------------------------------------------------------------------
# Pixel <-> map-frame conversion
# ----------------------------------------------------------------------------
def _px_to_world(col: float, row: float, origin: Point2, resolution: float) -> Point2:
    """Map a grid (col, row) index to the cell-center point in the map frame."""
    return (
        origin[0] + (col + 0.5) * resolution,
        origin[1] + (row + 0.5) * resolution,
    )


# ----------------------------------------------------------------------------
# Segment helpers + light collinear merge
# ----------------------------------------------------------------------------
def _seg_length(seg: Segment) -> float:
    (x0, y0), (x1, y1) = seg
    return math.hypot(x1 - x0, y1 - y0)


def _angle_diff_mod_pi(a: float, b: float) -> float:
    """Smallest angular difference between two undirected lines, in [0, pi/2]."""
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def _try_merge(a: Segment, b: Segment, angle_tol: float, dist_tol: float,
               gap_tol: float) -> Optional[Segment]:
    """Merge ``a`` and ``b`` when they are the same wall line.

    Returns the merged segment (endpoints snapped onto the longer segment's
    infinite line) or ``None`` when they are not collinear/close/overlapping.
    Kept conservative on purpose: ``gap_tol`` must stay small so genuinely
    separate walls (split by a column or doorway) are NOT bridged back together.
    """
    if _seg_length(b) > _seg_length(a):
        a, b = b, a

    (ax0, ay0), (ax1, ay1) = a
    angle_a = math.atan2(ay1 - ay0, ax1 - ax0)
    angle_b = math.atan2(b[1][1] - b[0][1], b[1][0] - b[0][0])
    if _angle_diff_mod_pi(angle_a, angle_b) > angle_tol:
        return None

    dx, dy = math.cos(angle_a), math.sin(angle_a)
    px, py = -dy, dx
    p0 = (ax0, ay0)
    pts = [a[0], a[1], b[0], b[1]]

    for q in (b[0], b[1]):
        perp = (q[0] - p0[0]) * px + (q[1] - p0[1]) * py
        if abs(perp) > dist_tol:
            return None

    ta = sorted([(a[0][0] - p0[0]) * dx + (a[0][1] - p0[1]) * dy,
                 (a[1][0] - p0[0]) * dx + (a[1][1] - p0[1]) * dy])
    tb = sorted([(b[0][0] - p0[0]) * dx + (b[0][1] - p0[1]) * dy,
                 (b[1][0] - p0[0]) * dx + (b[1][1] - p0[1]) * dy])
    gap = max(tb[0] - ta[1], ta[0] - tb[1], 0.0)
    if gap > gap_tol:
        return None

    t_vals = [(q[0] - p0[0]) * dx + (q[1] - p0[1]) * dy for q in pts]
    t_min, t_max = min(t_vals), max(t_vals)
    return (
        (p0[0] + t_min * dx, p0[1] + t_min * dy),
        (p0[0] + t_max * dx, p0[1] + t_max * dy),
    )


def merge_segments(segments: List[Segment], *, angle_tol_deg: float,
                   dist_tol_m: float, gap_tol_m: float) -> List[Segment]:
    """Greedily merge collinear, close, overlapping segments until stable."""
    angle_tol = math.radians(angle_tol_deg)
    merged = list(segments)
    changed = True
    while changed:
        changed = False
        out: List[Segment] = []
        used = [False] * len(merged)
        for i in range(len(merged)):
            if used[i]:
                continue
            current = merged[i]
            for j in range(i + 1, len(merged)):
                if used[j]:
                    continue
                combined = _try_merge(current, merged[j], angle_tol, dist_tol_m, gap_tol_m)
                if combined is not None:
                    current = combined
                    used[j] = True
                    changed = True
            used[i] = True
            out.append(current)
        merged = out
    return merged


# ----------------------------------------------------------------------------
# Junction splitting
# ----------------------------------------------------------------------------
def _project_onto(seg: Segment, pt: Point2) -> Tuple[float, float, float]:
    """Return (along-distance t, perpendicular distance, segment length)."""
    (x0, y0), (x1, y1) = seg
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length == 0.0:
        return 0.0, float("inf"), 0.0
    ux, uy = dx / length, dy / length
    t = (pt[0] - x0) * ux + (pt[1] - y0) * uy
    fx, fy = x0 + t * ux, y0 + t * uy
    return t, math.hypot(pt[0] - fx, pt[1] - fy), length


def split_walls_at_features(walls: List[Segment], columns: List[Column], *,
                            tol_m: float, margin_m: float,
                            min_len_m: float) -> List[Segment]:
    """Split each wall where another wall's endpoint or a column lands on it.

    A T/L-junction shows up as one wall's endpoint sitting on the interior of
    another wall; cutting there turns a wall that merely *passes* a junction
    into separate wall instances. ``margin_m`` keeps cuts away from the ends (so
    a shared corner does not create a stub), and pieces shorter than
    ``min_len_m`` are dropped. ``tol_m <= 0`` disables splitting.
    """
    if tol_m <= 0.0 or not walls:
        return list(walls)

    endpoints = []
    for w in walls:
        endpoints.append(w[0])
        endpoints.append(w[1])
    col_feats = [(c["center"], float(c.get("radius_m", 0.0))) for c in columns]

    out: List[Segment] = []
    for i, wall in enumerate(walls):
        (x0, y0), (x1, y1) = wall
        length = math.hypot(x1 - x0, y1 - y0)
        if length <= 0.0:
            continue
        ux, uy = (x1 - x0) / length, (y1 - y0) / length

        cuts = []
        for j, p in enumerate(endpoints):
            if j // 2 == i:                      # skip this wall's own endpoints
                continue
            t, perp, _ = _project_onto(wall, p)
            if perp <= tol_m and margin_m <= t <= length - margin_m:
                cuts.append(t)
        for center, radius in col_feats:
            t, perp, _ = _project_onto(wall, center)
            if perp <= radius + tol_m and margin_m <= t <= length - margin_m:
                cuts.append(t)

        if not cuts:
            out.append(wall)
            continue

        cuts.sort()
        deduped = []
        for t in cuts:
            if not deduped or t - deduped[-1] > min_len_m * 0.5:
                deduped.append(t)

        bounds = [0.0] + deduped + [length]
        for a, b in zip(bounds[:-1], bounds[1:]):
            if b - a < min_len_m:
                continue
            out.append(((x0 + a * ux, y0 + a * uy), (x0 + b * ux, y0 + b * uy)))
    return out


# ----------------------------------------------------------------------------
# Orientation refinement
# ----------------------------------------------------------------------------
def _refine_orientation(walls: List[Segment], binary: np.ndarray, origin: Point2,
                        resolution: float, *, band_m: float,
                        min_pts: int = 12) -> List[Segment]:
    """Re-align each wall's DIRECTION to the occupied pixels around it.

    Contour edges and merged double-faces can come out slightly skewed from the
    true wall axis. For each segment we fit a line (``cv2.fitLine``) to the
    occupied cells within ``band_m`` of it and rotate the segment onto that
    direction, keeping its midpoint and length fixed. Anchoring at the midpoint
    (rather than re-centering on the pixel centroid) keeps a face-wall on its
    scannable surface instead of pulling it to the wall centerline. ``band_m <=
    0`` disables refinement.
    """
    if band_m <= 0.0 or _cv2 is None or not walls:
        return list(walls)

    ys, xs = np.where(binary > 0)
    if len(xs) == 0:
        return list(walls)
    wx = origin[0] + (xs.astype(np.float64) + 0.5) * resolution
    wy = origin[1] + (ys.astype(np.float64) + 0.5) * resolution

    out: List[Segment] = []
    for (x0, y0), (x1, y1) in walls:
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length <= 0.0:
            out.append(((x0, y0), (x1, y1)))
            continue
        ux, uy = dx / length, dy / length
        along = (wx - x0) * ux + (wy - y0) * uy
        perp = -(wx - x0) * uy + (wy - y0) * ux
        sel = (np.abs(perp) <= band_m) & (along >= 0.0) & (along <= length)
        if int(sel.sum()) < min_pts:
            out.append(((x0, y0), (x1, y1)))
            continue

        pts = np.column_stack((wx[sel], wy[sel])).astype(np.float32)
        vx, vy, _lx, _ly = _cv2.fitLine(pts, _cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        if vx * ux + vy * uy < 0.0:          # keep the original orientation sense
            vx, vy = -vx, -vy
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        half = length / 2.0
        out.append((
            (float(mx - half * vx), float(my - half * vy)),
            (float(mx + half * vx), float(my + half * vy)),
        ))
    return out


# ----------------------------------------------------------------------------
# Inner-face snapping
# ----------------------------------------------------------------------------
def _snap_walls_to_inner_face(walls: List[Segment], binary: np.ndarray,
                              free_mask: np.ndarray, origin: Point2,
                              resolution: float, *, search_m: float,
                              min_pts: int = 8) -> List[Segment]:
    """Slide each wall perpendicular onto its room-facing (inner) face.

    Contour tracing puts perimeter walls on the *exterior* face, but the robot
    scans the room side. Along each wall we take the occupied span on the
    perpendicular and choose the span end adjacent to FREE (known-empty) space
    -- that is the room-facing surface. When both ends border free space (a
    free-standing partition with two scannable faces) the end nearest the
    current line is kept, so those lines do not move. The line is then shifted by
    the median offset, preserving its orientation. ``search_m <= 0`` disables it.
    """
    if search_m <= 0.0 or _cv2 is None or not walls:
        return list(walls)

    H, W = binary.shape
    reach = int(round(search_m / resolution)) + 1
    out: List[Segment] = []

    for (x0, y0), (x1, y1) in walls:
        length = math.hypot(x1 - x0, y1 - y0)
        if length <= 0.0:
            out.append(((x0, y0), (x1, y1)))
            continue
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        nx, ny = -uy, ux

        margin = min(0.5, length * 0.2)
        offsets = []
        t = margin
        while t <= length - margin:
            sx, sy = x0 + t * ux, y0 + t * uy
            occ_line, free_line = [], []
            for k in range(-reach, reach + 1):
                col = int((sx + k * resolution * nx - origin[0]) / resolution)
                row = int((sy + k * resolution * ny - origin[1]) / resolution)
                inb = 0 <= row < H and 0 <= col < W
                occ_line.append(1 if (inb and binary[row, col]) else 0)
                free_line.append(1 if (inb and free_mask[row, col]) else 0)

            best, best_d, i = None, 10 ** 9, 0
            while i < len(occ_line):
                if occ_line[i]:
                    j = i
                    while j + 1 < len(occ_line) and occ_line[j + 1]:
                        j += 1
                    d = 0 if i <= reach <= j else min(abs(reach - i), abs(reach - j))
                    if d < best_d:
                        best_d, best = d, (i, j)
                    i = j + 1
                else:
                    i += 1

            if best is None or best_d > 3:
                t += resolution
                continue
            a, b = best
            a_free = a - 1 >= 0 and free_line[a - 1]
            b_free = b + 1 < len(occ_line) and free_line[b + 1]
            off_a = (a - reach) * resolution
            off_b = (b - reach) * resolution
            if a_free and b_free:
                offsets.append(off_a if abs(off_a) < abs(off_b) else off_b)
            elif a_free:
                offsets.append(off_a)
            elif b_free:
                offsets.append(off_b)
            t += resolution

        if len(offsets) >= min_pts:
            shift = float(np.median(offsets))
            out.append(((x0 + shift * nx, y0 + shift * ny),
                        (x1 + shift * nx, y1 + shift * ny)))
        else:
            out.append(((x0, y0), (x1, y1)))
    return out


# ----------------------------------------------------------------------------
# Corner mending
# ----------------------------------------------------------------------------
def _line_intersection(p: Point2, d: Point2, q: Point2, e: Point2) -> Optional[Point2]:
    """Intersection of infinite lines (p + t d) and (q + s e); None if parallel."""
    det = e[0] * d[1] - d[0] * e[1]
    if abs(det) < 1e-9:
        return None
    t = ((q[0] - p[0]) * (-e[1]) - (-e[0]) * (q[1] - p[1])) / det
    return (p[0] + t * d[0], p[1] + t * d[1])


def mend_corners(walls: List[Segment], *, tol_m: float, max_extend_m: float,
                 min_angle_deg: float) -> List[Segment]:
    """Make non-parallel walls meet exactly at their shared corner.

    After inner-face snapping each wall shifts perpendicular to itself, so two
    walls that shared a corner no longer touch -- their ends overshoot and cross.
    Each corner end is matched to its single nearest qualifying partner using
    greedy global one-to-one matching (closest pairs first, every endpoint used
    once); both ends then move to the intersection of the two lines. The
    one-to-one rule is what keeps a T-junction correct -- e.g. each partition
    face joins the *same-side* wall half instead of being dragged across the
    junction to the opposite one. Pairs flatter than ``min_angle_deg`` are
    skipped (so collinear pieces split by a column/doorway keep their gap), and
    ``max_extend_m`` caps how far an endpoint may move. ``tol_m <= 0`` disables.
    """
    if tol_m <= 0.0 or len(walls) < 2:
        return list(walls)

    pts = [[list(a), list(b)] for a, b in walls]
    dirs = []
    for (ax, ay), (bx, by) in walls:
        length = math.hypot(bx - ax, by - ay) or 1.0
        dirs.append(((bx - ax) / length, (by - ay) / length))
    min_angle = math.radians(min_angle_deg)
    n = len(pts)

    # Collect all qualifying endpoint pairs, then match greedily by distance.
    candidates = []  # (distance, i, ei, j, ej)
    for i in range(n):
        ang_i = math.atan2(dirs[i][1], dirs[i][0])
        for j in range(i + 1, n):
            ang_j = math.atan2(dirs[j][1], dirs[j][0])
            if _angle_diff_mod_pi(ang_i, ang_j) < min_angle:
                continue
            for ei in (0, 1):
                for ej in (0, 1):
                    d = math.dist(pts[i][ei], pts[j][ej])
                    if d <= tol_m:
                        candidates.append((d, i, ei, j, ej))

    candidates.sort(key=lambda c: c[0])
    used = set()  # (wall_idx, end_idx) already mended

    for _d, i, ei, j, ej in candidates:
        if (i, ei) in used or (j, ej) in used:
            continue
        x = _line_intersection(pts[i][ei], dirs[i], pts[j][ej], dirs[j])
        if x is None:
            continue
        if math.dist(x, pts[i][ei]) > max_extend_m or math.dist(x, pts[j][ej]) > max_extend_m:
            continue
        pts[i][ei] = [x[0], x[1]]
        pts[j][ej] = [x[0], x[1]]
        used.add((i, ei))
        used.add((j, ej))

    return [((p[0][0], p[0][1]), (p[1][0], p[1][1])) for p in pts]


# ----------------------------------------------------------------------------
# Embedded-column (pilaster) detection
# ----------------------------------------------------------------------------
def detect_embedded_columns_and_split(
        walls: List[Segment], binary: np.ndarray, origin: Point2,
        resolution: float, *, min_protrusion_m: float, min_diameter_m: float,
        max_diameter_m: float, margin_m: float, min_len_m: float
) -> Tuple[List[Segment], List[Column]]:
    """Find columns fused into a wall (pilasters) and split the wall around them.

    A pilaster does not form its own connected component, so it is invisible to
    the component classifier; it shows up only as a local *thickening* of the
    wall. We walk along every wall and, at each step, measure the wall thickness
    as the perpendicular run of contiguous occupied cells through the wall (this
    captures narrow protrusions a distance transform would cap at the inscribed
    circle). A mid-span run thicker than the wall's median by ``min_protrusion_m``
    is a pilaster: it is returned as a column and the wall is cut on both sides
    of it (the thick span itself becomes the column, not wall). Endpoints are
    excluded by ``margin_m`` so corners/junctions are not mistaken for columns.
    """
    if _cv2 is None or min_protrusion_m <= 0.0 or not walls:
        return list(walls), []

    H, W = binary.shape
    reach = int(round(max_diameter_m / resolution)) + 1
    center_idx = reach

    out_walls: List[Segment] = []
    columns: List[Column] = []

    for (x0, y0), (x1, y1) in walls:
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 2.0 * margin_m + min_len_m:
            out_walls.append(((x0, y0), (x1, y1)))
            continue
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        nx, ny = -uy, ux

        ts, widths, centers = [], [], []
        t = margin_m
        while t <= length - margin_m:
            sx, sy = x0 + t * ux, y0 + t * uy
            line = []
            for k in range(-reach, reach + 1):
                col = int((sx + k * resolution * nx - origin[0]) / resolution)
                row = int((sy + k * resolution * ny - origin[1]) / resolution)
                line.append(1 if (0 <= row < H and 0 <= col < W and binary[row, col]) else 0)

            # Pick the contiguous occupied run nearest the wall line (center).
            best_run, best_d = None, 10 ** 9
            i = 0
            while i < len(line):
                if line[i]:
                    j = i
                    while j + 1 < len(line) and line[j + 1]:
                        j += 1
                    d = 0 if i <= center_idx <= j else min(abs(center_idx - i), abs(center_idx - j))
                    if d < best_d:
                        best_d, best_run = d, (i, j)
                    i = j + 1
                else:
                    i += 1

            if best_run is not None and best_d <= 3:
                a, b = best_run
                span = (b - a + 1) * resolution
                mid_k = (a + b) / 2.0 - reach
            else:
                span = 0.0
                mid_k = 0.0
            ts.append(t)
            widths.append(span)
            centers.append((sx + mid_k * resolution * nx, sy + mid_k * resolution * ny))
            t += resolution

        if not widths:
            out_walls.append(((x0, y0), (x1, y1)))
            continue

        nominal = float(np.median(widths))
        threshold = max(nominal + min_protrusion_m, min_diameter_m)
        thick = [w >= threshold for w in widths]

        cuts = []  # (t_start, t_end) spans that are columns, not wall
        i, n = 0, len(thick)
        while i < n:
            if not thick[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and thick[j + 1]:
                j += 1
            run_len = ts[j] - ts[i]
            if 0.1 <= run_len <= max_diameter_m:
                peak = max(range(i, j + 1), key=lambda m: widths[m])
                radius = max(widths[peak] / 2.0, run_len / 2.0)
                columns.append({
                    "center": centers[peak],
                    "radius_m": float(radius),
                    "area_m2": float(math.pi * radius * radius),
                    "embedded": True,
                })
                cuts.append((ts[i], ts[j]))
            i = j + 1

        if not cuts:
            out_walls.append(((x0, y0), (x1, y1)))
            continue

        bounds = [0.0]
        for a, b in cuts:
            bounds.extend([a, b])
        bounds.append(length)
        for idx in range(0, len(bounds) - 1, 2):   # keep wall spans, drop column spans
            a, b = bounds[idx], bounds[idx + 1]
            if b - a >= min_len_m:
                out_walls.append(((x0 + a * ux, y0 + a * uy), (x0 + b * ux, y0 + b * uy)))

    return out_walls, columns


# ----------------------------------------------------------------------------
# Per-component geometry extraction
# ----------------------------------------------------------------------------
def _clean_mask(binary: np.ndarray, open_radius_cells: int,
                close_radius_cells: int) -> np.ndarray:
    """Open (remove speckle) then close (seal hairline gaps)."""
    out = binary
    if open_radius_cells > 0:
        k = 2 * int(open_radius_cells) + 1
        kernel = _cv2.getStructuringElement(_cv2.MORPH_ELLIPSE, (k, k))
        out = _cv2.morphologyEx(out, _cv2.MORPH_OPEN, kernel)
    if close_radius_cells > 0:
        k = 2 * int(close_radius_cells) + 1
        kernel = _cv2.getStructuringElement(_cv2.MORPH_ELLIPSE, (k, k))
        out = _cv2.morphologyEx(out, _cv2.MORPH_CLOSE, kernel)
    return out


def _bar_centerline(rect, origin: Point2, resolution: float) -> Segment:
    """Centerline of a thin oriented box: connect the two short-edge midpoints."""
    box = _cv2.boxPoints(rect)  # 4x2 float (x=col, y=row)
    mids = []
    lengths = []
    for i in range(4):
        p = box[i]
        q = box[(i + 1) % 4]
        mids.append(((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0))
        lengths.append(math.hypot(q[0] - p[0], q[1] - p[1]))
    order = sorted(range(4), key=lambda i: lengths[i])  # two shortest = the ends
    m0, m1 = mids[order[0]], mids[order[1]]
    return (
        _px_to_world(m0[0], m0[1], origin, resolution),
        _px_to_world(m1[0], m1[1], origin, resolution),
    )


def _contour_segments(mask: np.ndarray, origin: Point2, resolution: float,
                      epsilon_m: float) -> List[Segment]:
    """Trace a component outline and split it into one segment per face/corner."""
    contours, _ = _cv2.findContours(mask, _cv2.RETR_EXTERNAL, _cv2.CHAIN_APPROX_SIMPLE)
    eps_px = max(1.0, epsilon_m / resolution)
    segs: List[Segment] = []
    for contour in contours:
        approx = _cv2.approxPolyDP(contour, eps_px, True).reshape(-1, 2)
        n = len(approx)
        if n < 2:
            continue
        for i in range(n):
            p = approx[i]
            q = approx[(i + 1) % n]
            segs.append((
                _px_to_world(float(p[0]), float(p[1]), origin, resolution),
                _px_to_world(float(q[0]), float(q[1]), origin, resolution),
            ))
    return segs


# ----------------------------------------------------------------------------
# Top-level detection
# ----------------------------------------------------------------------------
def detect_map_features(grid: np.ndarray, origin: Point2, resolution: float, *,
                        occupied_thresh: int = _DEFAULT_OCCUPIED_THRESH,
                        open_radius_cells: int = 0,
                        close_radius_cells: int = 2,
                        noise_min_area_cells: int = 4,
                        column_max_extent_m: float = 1.0,
                        column_max_aspect: float = 2.5,
                        column_min_solidity: float = 0.4,
                        obstacle_max_extent_m: float = 2.0,
                        obstacle_max_unknown_frac: float = 0.15,
                        wall_max_thickness_m: float = 0.4,
                        wall_min_aspect: float = 2.0,
                        wall_rect_fill_min: float = 0.55,
                        approx_epsilon_m: float = 0.4,
                        min_wall_length_m: float = 1.0,
                        merge_angle_deg: float = 14.0,
                        merge_dist_m: float = 0.5,
                        merge_gap_m: float = 0.5,
                        junction_split_tol_m: float = 0.35,
                        junction_split_margin_m: float = 1.0,
                        wall_fit_band_m: float = 0.3,
                        embedded_col_min_protrusion_m: float = 0.3,
                        embedded_col_min_diameter_m: float = 0.4,
                        embedded_col_max_diameter_m: float = 1.5,
                        inner_face_search_m: float = 1.0,
                        corner_mend_tol_m: float = 0.6,
                        corner_mend_max_extend_m: float = 0.8,
                        corner_mend_min_angle_deg: float = 20.0) -> Dict[str, list]:
    """Detect wall segments and columns in an occupancy grid.

    ``grid`` is the row-major occupancy array (``grid[row, col]``), ``origin`` the
    map-frame XY of the grid's (0, 0) corner, ``resolution`` metres per cell.

    Returns ``{"walls": [Segment, ...], "columns": [Column, ...]}`` with walls
    sorted longest first. Returns empty lists when OpenCV is unavailable or the
    grid has no occupied cells.
    """
    if _cv2 is None or resolution <= 0.0:
        return {"walls": [], "columns": []}

    binary = (grid >= occupied_thresh).astype(np.uint8) * 255
    if int(binary.sum()) == 0:
        return {"walls": [], "columns": []}

    binary = _clean_mask(binary, open_radius_cells, close_radius_cells)

    num_labels, labels, stats, _centroids = _cv2.connectedComponentsWithStats(binary, 8)

    # A real wall borders unknown space (outside the mapped area); an island
    # fully surrounded by known-free space is free-standing furniture/clutter.
    unknown_mask = grid < 0
    border_kernel = _cv2.getStructuringElement(_cv2.MORPH_RECT, (3, 3))

    walls: List[Segment] = []
    columns: List[Column] = []

    for lab in range(1, num_labels):
        area_cells = int(stats[lab, _cv2.CC_STAT_AREA])
        if area_cells < noise_min_area_cells:
            continue

        mask = (labels == lab).astype(np.uint8)
        ys, xs = np.where(mask > 0)
        pts = np.column_stack((xs, ys)).astype(np.int32)

        rect = _cv2.minAreaRect(pts)            # ((cx, cy), (w, h), angle_deg)
        (_rcx, _rcy), (rw, rh), _ang = rect
        long_px = max(rw, rh)
        short_px = min(rw, rh)
        long_m = long_px * resolution
        short_m = short_px * resolution
        rect_area = max(rw * rh, 1.0)
        solidity = area_cells / rect_area
        aspect = long_px / max(short_px, 1.0)

        # COLUMN: small, COMPACT and solid (a free-standing obstacle). The aspect
        # gate is what keeps short-but-elongated wall fragments out of the column
        # bucket -- they fall through to wall handling and get merged instead.
        if (long_m <= column_max_extent_m and aspect <= column_max_aspect
                and solidity >= column_min_solidity):
            (ccx, ccy), rad_px = _cv2.minEnclosingCircle(pts)
            columns.append({
                "center": _px_to_world(ccx, ccy, origin, resolution),
                "radius_m": float(rad_px * resolution),
                "area_m2": float(area_cells * resolution * resolution),
            })
            continue

        # OBSTACLE: a separate island whose border is almost entirely known-free
        # space (only a small fraction borders unknown) is free-standing
        # furniture/clutter, not a building wall -- a real wall fronts the
        # unmapped exterior. Capped by ``obstacle_max_extent_m`` so a genuinely
        # large free-standing structure is still treated as a wall.
        if obstacle_max_extent_m > 0.0 and long_m <= obstacle_max_extent_m:
            border = _cv2.dilate(mask, border_kernel) - mask
            border_cells = int(np.count_nonzero(border))
            unknown_frac = (
                float(np.count_nonzero(unknown_mask & (border > 0))) / border_cells
                if border_cells else 1.0
            )
            if unknown_frac <= obstacle_max_unknown_frac:
                (ccx, ccy), rad_px = _cv2.minEnclosingCircle(pts)
                columns.append({
                    "center": _px_to_world(ccx, ccy, origin, resolution),
                    "radius_m": float(rad_px * resolution),
                    "area_m2": float(area_cells * resolution * resolution),
                    "obstacle": True,
                })
                continue

        # WALL, thin straight bar: take the box centerline (one clean segment).
        if (short_m <= wall_max_thickness_m and aspect >= wall_min_aspect
                and solidity >= wall_rect_fill_min):
            walls.append(_bar_centerline(rect, origin, resolution))
            continue

        # WALL, complex/perimeter structure: trace the outline, split at corners.
        walls.extend(_contour_segments(mask, origin, resolution, approx_epsilon_m))

    walls = [s for s in walls if _seg_length(s) >= min_wall_length_m]
    walls = merge_segments(
        walls, angle_tol_deg=merge_angle_deg,
        dist_tol_m=merge_dist_m, gap_tol_m=merge_gap_m,
    )
    walls = [s for s in walls if _seg_length(s) >= min_wall_length_m]

    # Split walls that merely pass through a T/L-junction or a column footprint,
    # so each structurally distinct wall becomes its own instance.
    walls = split_walls_at_features(
        walls, columns,
        tol_m=junction_split_tol_m,
        margin_m=junction_split_margin_m,
        min_len_m=min_wall_length_m,
    )
    walls = [s for s in walls if _seg_length(s) >= min_wall_length_m]

    # Snap each wall's heading onto the actual wall pixels (fixes skewed lines).
    walls = _refine_orientation(walls, binary, origin, resolution, band_m=wall_fit_band_m)

    # Find columns fused into walls (pilasters) and split the walls around them.
    walls, embedded = detect_embedded_columns_and_split(
        walls, binary, origin, resolution,
        min_protrusion_m=embedded_col_min_protrusion_m,
        min_diameter_m=embedded_col_min_diameter_m,
        max_diameter_m=embedded_col_max_diameter_m,
        margin_m=junction_split_margin_m,
        min_len_m=min_wall_length_m,
    )
    columns = columns + embedded

    # Slide each wall onto its room-facing (inner) face for scanning.
    free_mask = (grid >= 0) & (binary == 0)
    walls = _snap_walls_to_inner_face(
        walls, binary, free_mask, origin, resolution, search_m=inner_face_search_m
    )

    # After snapping, make adjacent walls meet exactly at their shared corner.
    walls = mend_corners(
        walls,
        tol_m=corner_mend_tol_m,
        max_extend_m=corner_mend_max_extend_m,
        min_angle_deg=corner_mend_min_angle_deg,
    )

    walls = [s for s in walls if _seg_length(s) >= min_wall_length_m]
    walls.sort(key=_seg_length, reverse=True)

    return {"walls": walls, "columns": columns}


def detect_wall_segments(grid: np.ndarray, origin: Point2, resolution: float,
                         **kwargs) -> List[Segment]:
    """Backward-compatible helper returning only the wall segments.

    Accepts the same keyword arguments as :func:`detect_map_features`; any extra
    keys are ignored so older callers keep working.
    """
    accepted = {
        k: v for k, v in kwargs.items()
        if k in detect_map_features.__kwdefaults__
    }
    return detect_map_features(grid, origin, resolution, **accepted)["walls"]
