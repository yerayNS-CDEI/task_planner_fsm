"""Unattended wall selection: scan every detected wall, skip the ones that fail.

With ``scan_all_walls`` on, nothing asks the operator for walls: ComputeWallPoints
(full flow) and the bootstrap (direct starts) take every wall in navi-wall's
detected_walls.yaml, in file order, each at the same scan-line heights. A wall
the robot cannot get to is skipped instead of ending the run in Error
(``skip_current_wall``), so the mission covers every wall it can.

ctx knobs:
  scan_all_walls          true: no wall prompts (default false)
  scan_all_walls_lines_z  map-frame heights of each wall's scan lines, a number
                          or a list (default [1.5], the height of nearly every
                          line typed in by hand so far)
"""

DEFAULT_LINES_Z = (1.5,)


def scan_all_walls(ctx) -> bool:
    return bool(ctx.get("scan_all_walls", False))


def auto_lines_z(ctx):
    """``scan_all_walls_lines_z`` as a sorted list of floats, bottom line first."""
    raw = ctx.get("scan_all_walls_lines_z", DEFAULT_LINES_Z)
    if isinstance(raw, str):
        raw = raw.replace(",", " ").strip("[] ").split()
    elif not isinstance(raw, (list, tuple)):
        raw = [raw]
    lines = sorted(float(z) for z in raw)
    if not lines:
        raise ValueError("scan_all_walls_lines_z is empty")
    return lines


def skip_current_wall(ctx, owner: str, reason: str):
    """Give up on ``ctx['current_wall_index']`` and count it as done.

    WallTargetSelection already marked the wall as visited, so it is not picked
    again. Returns True while other walls remain, False when that was the last.
    """
    idx = ctx.get("current_wall_index")
    ctx.setdefault("skipped_walls", []).append({"wall_index": idx, "reason": reason})
    ctx["walls_left"] = max(0, int(ctx.get("walls_left", 0)) - 1)
    remaining = ctx["walls_left"] > 0
    if not remaining:
        ctx["scan_done"] = True
    ctx["node"].get_logger().warn(
        f"[{owner}] Skipping wall #{idx}: {reason}. "
        + (f"{ctx['walls_left']} wall(s) left." if remaining
           else "No walls left; heading home.")
    )
    return remaining
