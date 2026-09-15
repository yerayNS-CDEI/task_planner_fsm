"""The sweep samplers (GPR line, GPR triggers, hyperspectral) are armed once
per sweep -- with or without the GPR trigger timer.

The guard used to be "is the trigger timer running?". With
``gpr_trigger_enabled`` off there is no timer, so the samplers were re-armed
on every FSM tick of the sweep: the hyperspectral travel counter was reset
each second and, at 0.05 m/s, could never reach its 10 cm spacing. Found on
the 2026-09-15 hyperspectral bench; the default configuration (triggers on)
was never affected.

Run with:

    python3 -m pytest test/test_sweep_sampler_arming.py -v
"""

import types

import pytest

from task_planner_fsm.states.scan_wall import ScanWall


class _Logger:
    def info(self, msg, **kw):
        pass

    warn = error = debug = info


class _Node:
    def get_logger(self):
        return _Logger()


# A wall along +X at y = 0.55, swept left to right; the base faces +Y.
LINE = ((0.6, 0.55, 1.0), (1.4, 0.55, 1.0))


def make_ctx(**over):
    ctx = {
        "node": _Node(),
        "current_wall_index": 0,
        "wall_inward_normals": [(0.0, 1.0, 0.0)],
        "target_scan_wall": LINE,
        "target_scan_point": LINE[0],
        "current_wall_scan_lines": [1.0],
        "current_line_idx": 0,
        "sweep_use_arm": True,
        "gpr_enabled": False,
        "hyperspectral_enabled": True,
    }
    ctx.update(over)
    return ctx


@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s.started = True
    s._segments = [LINE]
    s._scan_poses = [(1.0, 0.0, 1.5708)]
    s._seg_idx = 0
    s._seg_phase = "sweep_wait"
    s._sweep_scanning = True        # the executor's "sweep" feedback arrived
    s._sweep_result = None          # ...and the sweep is still running
    return s


@pytest.mark.parametrize("triggers", [False, True])
def test_samplers_arm_once_per_sweep(state, triggers):
    ctx = make_ctx(gpr_trigger_enabled=triggers)
    calls = {"triggers": 0, "hs": 0}
    state._start_gpr_triggers = lambda *a, **kw: calls.__setitem__("triggers", calls["triggers"] + 1)
    state._start_hyperspectral = lambda *a, **kw: calls.__setitem__("hs", calls["hs"] + 1)

    for _ in range(5):
        state._run_scan(ctx)
        assert state._seg_phase == "sweep_wait"

    assert calls == {"triggers": 1, "hs": 1}
    assert state._sweep_samplers_armed is True


def test_nothing_arms_before_the_sweep_feedback(state):
    """The lead-in traverse and the executor's settle are not scan data."""
    state._sweep_scanning = False
    state._start_hyperspectral = lambda *a, **kw: pytest.fail("armed too early")
    state._run_scan(make_ctx())
    assert state._sweep_samplers_armed is False


def test_arming_resets_for_the_next_sweep_goal(state):
    """Each partition (and each height) is a new SweepLine goal, and its
    samplers must be armed afresh when ITS sweep feedback arrives."""
    state._sweep_samplers_armed = True
    state._sweep_client = types.SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda goal, feedback_callback: types.SimpleNamespace(
            add_done_callback=lambda cb: None),
    )
    ctx = make_ctx(sweep_executor_proc=None)
    state.current_line_z = 1.0
    assert state._send_sweep_goal(ctx, *LINE) is True
    assert state._sweep_samplers_armed is False
    assert state._sweep_scanning is False


def test_a_fresh_entry_starts_unarmed():
    s = ScanWall("ScanWall")
    assert s._sweep_samplers_armed is False


# ---------------------------------------------------------------------------
# Sample #0 is taken at contact, before the sweep goal goes out
# ---------------------------------------------------------------------------

def test_the_first_hyperspectral_sample_is_taken_at_contact_before_the_sweep(state):
    """press_settle: the plate is on the wall with its orientation corrected and
    nothing has moved laterally yet -- d = 0. The segment opens and sample #0
    fires there, and only then is the executor asked to sweep. Arming the
    distance timer stays with the executor's sweep feedback (the test above)."""
    state._seg_phase = "press_settle"
    state._sweep_scanning = False
    ctx = make_ctx(gpr_trigger_enabled=True)
    order = []
    state._wall_contact_ready = lambda _ctx: True
    state._stop_arm_processes = lambda _ctx, keep_sensors=False: order.append("stop_alignment")
    state._set_trajectory_bridge_hold = lambda _ctx, held: None
    state._send_sweep_goal = lambda _ctx, s, e, lead_in_only=False: (order.append("sweep_goal"), True)[1]
    state._hyperspectral_frame_and_axis = lambda _ctx, s, e: ("arm_base", (0.0, 1.0, 0.0))
    state._hs = types.SimpleNamespace(
        enabled=lambda _ctx: True,
        begin_segment=lambda _ctx, pose_fn, **kw: (order.append(("begin", kw["seg_idx"])), True)[1],
        capture_at_start=lambda _ctx: order.append("sample_0"),
        start_line=lambda *a, **kw: order.append("timer_armed"),
        stop_line=lambda *a, **kw: None,
    )

    state._run_scan(ctx)

    assert order[:2] == [("begin", 0), "sample_0"]
    assert "sweep_goal" in order and order.index("sample_0") < order.index("sweep_goal")
    assert "timer_armed" not in order            # that waits for the sweep feedback
    assert state._seg_phase == "sweep_wait"


def test_no_start_sample_before_contact(state):
    """While force mode is still pressing, nothing is sampled."""
    state._seg_phase = "press_settle"
    ctx = make_ctx()
    state._wall_contact_ready = lambda _ctx: False
    state._hs = types.SimpleNamespace(
        enabled=lambda _ctx: True,
        begin_segment=lambda *a, **kw: pytest.fail("segment opened before contact"),
        capture_at_start=lambda *a, **kw: pytest.fail("sampled before contact"),
    )
    state._run_scan(ctx)
    assert state._seg_phase == "press_settle"
