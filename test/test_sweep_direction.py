"""Sweep direction must follow the recorded start end, not the clamped target.

Wall 2 on 2026-10-08: the left 7.98 m of the line was unreachable, so the scan
start was clamped past the line's midpoint and the nearest-end guess reversed
the sweep, leaving the wall on the robot's right.
"""
import math

from task_planner_fsm.utils.wall_geometry import left_scan_endpoint, sweep_ends

LINE = ((-1.8006, 16.1219, 0.2207), (-13.9319, 10.2963, 0.2207))
INWARD = (-0.433, 0.901, 0.0)
CLAMPED = (-6.74, 13.75)


def _wall_on_left(near, far):
    hx, hy = far[0] - near[0], far[1] - near[1]
    return -hy * INWARD[0] + hx * INWARD[1] > 0.0


def test_clamped_start_past_midpoint_keeps_left_to_right():
    start = left_scan_endpoint(LINE, INWARD)
    near, far = sweep_ends(LINE, start, CLAMPED)
    assert near == LINE[1] and far == LINE[0]
    assert _wall_on_left(CLAMPED, far)


def test_nearest_end_fallback_without_start_end():
    near, far = sweep_ends(LINE, None, CLAMPED)
    assert near == LINE[0]
    assert math.isclose(far[0], LINE[1][0])
