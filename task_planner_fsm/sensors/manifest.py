"""The GPR line manifest: what ScanWall knows about each line the probe scanned.

The GP8800 keeps the traces; all the robot ever has is the start/stop it sent
and where the plate was. That is exactly what the processing needs later to put
a hyperbola found at "x = 0.83 m along the scan" back onto the wall, and it is
gone the moment the state exits -- so ScanWall appends one row per line here,
at line stop, and the GPR phase of SensorDataProcessing reads it back.

Append-only JSONL, one row per line, same reasoning as the hyperspectral raw
record: a crash mid-mission keeps every completed line. Written by
:func:`append_gpr_line`, read by :func:`read_gpr_lines`. No ROS.
"""

import json
import os
from datetime import datetime, timezone

from . import paths


def utc_now():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def line_key(wall_index, line_idx, seg_idx):
    """Stable, filesystem-safe name for one GPR line: ``w02_l01_s00``."""
    def _fmt(prefix, value):
        return f"{prefix}{int(value):02d}" if value is not None else f"{prefix}xx"
    return "_".join((_fmt("w", wall_index), _fmt("l", line_idx), _fmt("s", seg_idx)))


def append_gpr_line(ctx, row):
    """Append one line record. Returns the path written, or None on failure.

    ``row`` carries what the caller knows -- wall/line/seg indices, segment
    endpoints in the map frame, wall-clock start/stop, trigger count and travel,
    the measurement name given to the probe. ``key`` and ``t_written`` are
    filled in here so every reader sees the same shape.
    """
    path = paths.gpr_manifest_path(ctx)
    record = dict(row)
    record.setdefault("key", line_key(
        row.get("wall_index"), row.get("line_idx"), row.get("seg_idx")))
    record["t_written"] = utc_now()
    try:
        os.makedirs(path.parent, exist_ok=True)
        with open(path, "a") as handle:
            handle.write(json.dumps(record) + "\n")
    except (OSError, TypeError, ValueError):
        return None
    return str(path)


def read_gpr_lines(path):
    """Rows of a manifest in order, skipping unparseable lines."""
    if not path or not os.path.isfile(str(path)):
        return []
    rows = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue
    return rows
