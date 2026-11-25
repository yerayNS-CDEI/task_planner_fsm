"""Where a scanned GPR line lies on the wall, in the ``map`` frame.

A GPR line record (``manifest.read_gpr_lines``) carries two kinds of geometry:

* ``seg_start`` / ``seg_end`` -- the PLANNED scan line from
  ``utils/wall_geometry.py``. It runs parallel to the wall but ~0.6 m into the
  room (the base standoff), at the row's height. It gives the sweep direction
  and the row height, nothing more.
* ``origin_xyz`` -- the plate TCP (``arm_tool0``) in ``map`` when the plate
  seated, i.e. the scan's d = 0, pressed on the wall surface. ScanWall records
  it with ``origin_along_m`` (its position along the planned line).

A GPR position is therefore ``origin_xyz + unit * x_m`` in x/y, with
``unit`` the planned line's horizontal direction and z the row height
(``seg_start[2]``). That is on the wall surface, where the HSI ``pose_map``
samples also are, so the two can be compared directly.

A record without ``origin_xyz`` (written before ScanWall kept it, or a line
whose plate pose could not be looked up) can only be placed on the planned
line, 0.6 m off the wall. That is kept as a fallback and labelled
``REFERENCE_SCAN_LINE`` so nobody mistakes it for a wall point.

Not corrected anywhere yet: the GPR antenna's offset from the plate TCP
(~8 cm along the wall).

Shared by :mod:`.gpr` (hyperbolae, global JSON) and :mod:`.no_drill` (zones,
scanned lines), so the two can never place a scan differently. Pure geometry;
no ROS.
"""

import numpy as np

MAP_FRAME = "map"
# On the wall surface: placed from the plate's seated pose.
REFERENCE_WALL_SURFACE = "wall_surface"
# On the planned scan line, ~0.6 m into the room: the line record has no
# seated plate pose to put it on the wall.
REFERENCE_SCAN_LINE = "planned_scan_line"


def _unit_xy(seg_start, seg_end):
    """Horizontal unit vector along ``seg_start -> seg_end``, or None."""
    axis = np.asarray(seg_end, dtype=float)[:2] - np.asarray(seg_start, dtype=float)[:2]
    norm = float(np.linalg.norm(axis))
    if norm < 1e-6:
        return None
    return axis / norm


def has_geometry(line):
    return bool(line) and bool(line.get("seg_start")) and bool(line.get("seg_end"))


def wall_origin(line):
    """The seated plate's ``[x, y, z]`` from the record, or None."""
    origin = (line or {}).get("origin_xyz")
    if not origin or len(origin) < 2:
        return None
    try:
        return [float(v) for v in origin]
    except (TypeError, ValueError):
        return None


def reference(line):
    """``REFERENCE_WALL_SURFACE`` or ``REFERENCE_SCAN_LINE`` for a line record."""
    return REFERENCE_WALL_SURFACE if wall_origin(line) is not None else REFERENCE_SCAN_LINE


def scan_to_map(x_m, seg_start, seg_end, origin_along_m=0.0, origin_xyz=None):
    """``[x, y, z]`` of the point ``x_m`` metres along a scan.

    With ``origin_xyz`` (the plate seated on the wall at d = 0): that point
    plus ``x_m`` along the line's horizontal direction -- on the wall surface,
    z the row height ``seg_start[2]``.

    Without it: on the planned line ``seg_start -> seg_end``, counted from
    ``origin_along_m`` metres past ``seg_start``. That line is ~0.6 m off the
    wall.
    """
    start = np.asarray(seg_start, dtype=float)
    unit = _unit_xy(seg_start, seg_end)
    if unit is None:
        return [round(float(v), 4) for v in start]
    z = float(start[2]) if start.size > 2 else 0.0
    if origin_xyz is not None:
        xy = np.asarray(origin_xyz, dtype=float)[:2] + unit * float(x_m)
    else:
        xy = start[:2] + unit * (float(origin_along_m) + float(x_m))
    return [round(float(xy[0]), 4), round(float(xy[1]), 4), round(z, 4)]


def transform(line):
    """``x_m -> [x, y, z]`` for a line record, or None without geometry.

    On the wall from ``origin_xyz`` when the record has it, else on the
    planned line (see :func:`scan_to_map`); :func:`reference` says which.
    """
    if not has_geometry(line):
        return None
    origin_xyz = wall_origin(line)
    along = float(line.get("origin_along_m") or 0.0)
    return lambda x_m: scan_to_map(x_m, line["seg_start"], line["seg_end"], along, origin_xyz)


def wall_segment(line):
    """The scanned segment as ``(start, end)`` on the wall, or None.

    The planned ``seg_start -> seg_end`` moved sideways (perpendicular to
    itself, in the horizontal plane) onto the line through ``origin_xyz``;
    extent and row height are unchanged. Without ``origin_xyz`` the planned
    segment itself, as with :func:`scan_to_map`.
    """
    if not has_geometry(line):
        return None
    start = [float(v) for v in line["seg_start"]]
    end = [float(v) for v in line["seg_end"]]
    origin = wall_origin(line)
    unit = _unit_xy(start, end)
    if origin is None or unit is None:
        return start, end
    offset = np.asarray(origin[:2]) - np.asarray(start[:2])
    shift = offset - float(offset @ unit) * unit
    moved = lambda p: [round(p[0] + float(shift[0]), 4),
                       round(p[1] + float(shift[1]), 4), *p[2:]]
    return moved(start), moved(end)
