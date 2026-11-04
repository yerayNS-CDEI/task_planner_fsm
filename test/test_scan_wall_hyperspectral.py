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
    clock_stopped = False

    def __init__(self):
        self.calls = []

    def stop_clock(self, ctx):
        self.armed = False
        self.clock_stopped = True
        self.calls.append("stop_clock")

    def arm_triggers(self, ctx, ref, seg_start, seg_end, speed, travel_frame=None):
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

    def stop_timer(self, ctx):
        self.calls.append(("stop_timer",))


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


def test_the_cross_polarized_pass_samples_no_camera(state):
    """Same spot, same camera as the upright pass: the samples would be
    duplicates. The GPR still gets its clock — that pass is FOR the GPR."""
    state._cross_pol_pass = True
    _status(state, _ctx(), "running: seated")
    assert state._hs.calls == []
    assert state._gpr.calls == ["arm"]


def test_the_camera_off_costs_nothing(state):
    state._hs = _SamplerStub(enabled=False)
    _status(state, _ctx(), "running: seated")
    assert state._hs.calls == [] and state._gpr.calls == ["arm"]


def test_a_restarted_run_gets_a_new_session(state):
    state.reset_run(_ctx())
    assert state._hs.calls == [("new_mission",)]


def test_the_gpr_line_is_recorded_at_the_row_height_not_the_wall_lines():
    """The segments carry the detected wall line's z (0.23 m on 2026-10-02);
    the plate swept the row at current_line_z (2.2 m), where the hyperspectral
    poses -- read off TF -- were. The GPR line record must say the same."""
    s = ScanWall("ScanWall")
    wall_seg = ((1.0, 2.0, 0.2307), (4.0, 2.0, 0.2307))
    s.current_line_z = 2.2
    start, end = s._at_line_height(*wall_seg)
    assert start == (1.0, 2.0, 2.2) and end == (4.0, 2.0, 2.2)
    s.current_line_z = None                 # no row height known: left as it was
    assert s._at_line_height(*wall_seg) == wall_seg


def test_the_retreat_stops_both_clocks_and_nothing_rearms_them(state):
    """The sweep node reports "running: retreat" when the sweep is done and it
    starts backing the plate off the wall; the GPR and the camera must not
    record that, and a stray "seated" during it must not start them again."""
    ctx = _ctx()
    _status(state, ctx, "running: seated")
    _status(state, ctx, "running: retreat")
    assert state._gpr.calls == ["arm", "stop_clock"]
    assert state._hs.calls[-1] == ("stop_timer",)
    _status(state, ctx, "running: seated")
    assert state._gpr.calls == ["arm", "stop_clock"]
    assert [c[0] for c in state._hs.calls].count("start_line") == 1
