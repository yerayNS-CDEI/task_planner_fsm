"""ScanWall never blocks its thread: the alignment wait before the press and
the pause after a sweep are tick-driven phases, not time.sleep.

The 10 s sleep froze the whole FSM node (timers, sensor callbacks,
telemetry, restarts) on every sweep. The alignment wait now ends as soon as
the plate is square to the wall, judged by the same plane fit the
alignment controller uses.

Run with:

    python3 -m pytest test/test_scan_wall_waits.py -v
"""

import inspect
import math

import pytest

from task_planner_fsm.states import scan_wall as scan_wall_mod
from task_planner_fsm.states.scan_wall import ScanWall


class _Logger:
    def info(self, msg, **kw):
        pass

    warn = error = debug = info


class _Node:
    def get_logger(self):
        return _Logger()


SENSOR_XY = [(0.0, 0.172), (-0.155, -0.17), (0.155, -0.17),
             (-0.152, 0.17), (0.152, 0.17), (0.0, -0.172)]


def ranges(tilt_deg, standoff=0.20):
    """The six ranges of a plate tilted about its X axis by ``tilt_deg``."""
    t = math.tan(math.radians(tilt_deg))
    return [standoff + y * t for _x, y in SENSOR_XY]


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(scan_wall_mod.time, "time", c)
    return c


@pytest.fixture
def state(clock):
    s = ScanWall("ScanWall")
    s._align_wait_start = clock.now
    s._align_stamp = None
    s._align_streak = 0
    return s


def feed(state, ctx, clock, tilt, dt=0.2):
    """One new /distance_sensors frame, dt seconds after the last tick."""
    clock.now += dt
    ctx["plate_distances"] = ranges(tilt)
    ctx["plate_distances_stamp"] = clock.now
    return state._plate_aligned(ctx)


def test_the_module_never_sleeps():
    source = inspect.getsource(scan_wall_mod)
    code = [line for line in source.splitlines() if not line.strip().startswith("#")]
    assert not any("time.sleep(" in line for line in code)


@pytest.mark.skipif(scan_wall_mod.fit_wall_normal is None, reason="arm_control not importable")
def test_a_square_plate_passes_after_the_start_up_minimum(state, clock):
    ctx = {"node": _Node()}
    results = [feed(state, ctx, clock, 0.3) for _ in range(15)]   # 3 s of frames
    first = results.index(True)
    assert 2.0 <= (first + 1) * 0.2 < 2.5


@pytest.mark.skipif(scan_wall_mod.fit_wall_normal is None, reason="arm_control not importable")
def test_a_tilted_plate_waits_and_one_bad_frame_restarts_the_count(state, clock):
    ctx = {"node": _Node()}
    assert not any(feed(state, ctx, clock, 3.0) for _ in range(12))
    assert not feed(state, ctx, clock, 0.2)
    assert not feed(state, ctx, clock, 0.2)
    assert not feed(state, ctx, clock, 2.0)     # breaks the streak
    assert not feed(state, ctx, clock, 0.2)
    assert not feed(state, ctx, clock, 0.2)
    assert feed(state, ctx, clock, 0.2)


@pytest.mark.skipif(scan_wall_mod.fit_wall_normal is None, reason="arm_control not importable")
def test_a_stale_frame_is_not_counted_twice(state, clock):
    ctx = {"node": _Node()}
    feed(state, ctx, clock, 0.1, dt=2.0)
    clock.now += 0.2
    assert not state._plate_aligned(ctx)        # same stamp: no new evidence
    assert state._align_streak == 1


def test_it_never_waits_longer_than_the_old_sleep(state, clock):
    ctx = {"node": _Node()}
    assert not feed(state, ctx, clock, 5.0, dt=9.0)
    assert feed(state, ctx, clock, 5.0, dt=1.0)


def test_without_the_plane_fit_it_waits_the_old_time(state, clock, monkeypatch):
    monkeypatch.setattr(scan_wall_mod, "fit_wall_normal", None)
    ctx = {"node": _Node()}
    assert not feed(state, ctx, clock, 0.0, dt=9.9)
    assert feed(state, ctx, clock, 0.0, dt=0.2)


def test_the_post_sweep_pause_hands_over_once_it_has_run_out(state, clock):
    seen = []
    state._advance_after_sweep = lambda ctx, *args: seen.append(args)
    state._settled_sweep = (True, "", (0, 0, 1), (1, 0, 1), True)
    state._settle_until = clock.now + 2.0
    state._segments = [((0, 0, 1), (1, 0, 1))]
    state._seg_idx = 0
    state._seg_phase = "sweep_settle"
    state.started = True
    ctx = {"node": _Node(), "current_wall_scan_lines": [1.0], "scan_wall_cross_pol": False}
    state._run_scan(ctx)
    assert seen == []
    clock.now += 2.1
    state._run_scan(ctx)
    assert seen == [(True, "", (0, 0, 1), (1, 0, 1), True)]
