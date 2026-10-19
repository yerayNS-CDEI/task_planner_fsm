"""The arm approach leaves the plate 0.30 m off the wall, and the press can
still reach the wall from there.

At 0.20 m the planned approach collided with the dilated octomap in front of
the wall "sometimes" and the partition was skipped. Raising it is only safe if
force mode's Z reach grows with it, or the press stops short of the wall.

Run with:

    python3 -m pytest test/test_scan_wall_approach.py -v
"""

import pytest

from task_planner_fsm.states.scan_wall import ScanWall


class _Node:
    def get_logger(self):
        return self

    def warn(self, *a, **kw):
        pass


@pytest.fixture
def state():
    return ScanWall("ScanWall")


def test_the_approach_leaves_the_plate_thirty_centimetres_off(state):
    assert state._approach_standoff({}) == pytest.approx(0.30)
    assert state._approach_standoff({"sweep_approach_retract_m": 0.0}) == pytest.approx(0.20)


@pytest.mark.parametrize("ctx", [{}, {"sweep_approach_retract_m": 0.25}])
def test_force_mode_can_press_through_the_whole_approach_gap(state, ctx):
    ctx = dict(ctx, node=_Node())
    request = state._build_force_mode_request(ctx)
    # deviation_limits is float32 on the wire
    assert request.deviation_limits[2] >= state._approach_standoff(ctx) + 0.05 - 1e-6


def test_the_executor_backs_off_the_same_margin(state, monkeypatch):
    seen = []
    monkeypatch.setattr(state, "_start_arm_procs", lambda ctx, procs: seen.extend(procs) or True)
    state._start_sweep_executor({"sim": True})
    (_, cmd, _), = seen
    assert "approach_retract_m:=0.1" in cmd
    assert "scan_standoff_m:=0.2" in cmd
