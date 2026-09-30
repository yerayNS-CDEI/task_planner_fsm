"""ScanWall starts the hyperspectral camera where it starts the GPR's clock.

Under the whole-body sweep that is the sweep node's first "running: seated":
sample #0 there, then one capture per spacing of plate travel along the
segment, measured in map like the GPR triggers. The sampler itself is tested
in test_hyperspectral.py; this pins only ScanWall's side.

No rclpy graph: the sampler and the GPR are stubs.
"""

from types import SimpleNamespace

import pytest

pytest.importorskip("arm_control.srv")

from task_planner_fsm.states.scan_wall import ScanWall  # noqa: E402

SEG = ((1.0, 2.0, 1.1), (4.0, 2.0, 1.1))


class _GprStub:
    armed = False

    def __init__(self):
        self.calls = []

    def arm_triggers(self, ctx, ref, seg_start, seg_end, speed):
        self.armed = True
        self.calls.append("arm")

    def note_contact(self, ctx, seated):
        pass

    def _lookup_plate_xyz(self, ctx, frame, timeout_s=1.0):
        return (1.0, 2.0, 1.1)


class _SamplerStub:
    def __init__(self, enabled=True):
        self._enabled = enabled
        self.calls = []

    def enabled(self, ctx):
        return self._enabled

    def begin_segment(self, ctx, pose_fn, ref="map", world="map", **ident):
        self.calls.append(("begin", ref, world, ident, pose_fn("map", 0.0)))
        return True

    def capture_at_start(self, ctx):
        self.calls.append(("capture_at_start",))

    def start_line(self, ctx, seg_start, seg_end, pose_fn, ref="map", axis=None,
                   world="map", **ident):
        self.calls.append(("start_line", ref, axis, ident))

    def new_mission(self):
        self.calls.append(("new_mission",))


@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s._gpr = _GprStub()
    s._hs = _SamplerStub()
    s._segments = [SEG]
    s._seg_idx = 0
    s._seg_phase = "sweep_wait"
    return s


def _ctx(**over):
    ctx = {"node": None, "sweep_use_wbc": True, "sim": True,
           "current_wall_index": 3, "current_line_idx": 1}
    ctx.update(over)
    return ctx


def _status(state, ctx, text):
    state._on_wbc_status(SimpleNamespace(data=text), ctx)


def test_the_camera_starts_on_the_first_seated_with_the_gpr(state):
    ctx = _ctx()
    _status(state, ctx, "running: approach")
    assert state._hs.calls == [] and state._gpr.calls == []
    _status(state, ctx, "running: seated")
    assert state._gpr.calls == ["arm"]
    kinds = [c[0] for c in state._hs.calls]
    assert kinds == ["begin", "capture_at_start", "start_line"]
    ident = {"wall_index": 3, "line_idx": 1, "seg_idx": 0}
    begin, _, start = state._hs.calls
    assert begin[1:4] == ("map", "map", ident)
    assert begin[4] == (1.0, 2.0, 1.1)            # the GPR's plate lookup
    assert start[1] == "map" and start[3] == ident
    assert start[2] == pytest.approx((1.0, 0.0, 0.0))   # along seg_start -> seg_end


def test_a_later_unseat_and_reseat_does_not_restart_the_camera(state):
    ctx = _ctx()
    _status(state, ctx, "running: seated")
    _status(state, ctx, "running: unseated")
    _status(state, ctx, "running: seated")
    assert [c[0] for c in state._hs.calls].count("start_line") == 1


def test_the_camera_off_costs_nothing(state):
    state._hs = _SamplerStub(enabled=False)
    _status(state, _ctx(), "running: seated")
    assert state._hs.calls == [] and state._gpr.calls == ["arm"]


def test_a_restarted_run_gets_a_new_session(state):
    state.reset_run(_ctx())
    assert state._hs.calls == [("new_mission",)]
