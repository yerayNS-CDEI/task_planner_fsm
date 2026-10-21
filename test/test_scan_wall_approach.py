"""The arm approach stops 0.30 m off the wall at most, never backs off to it,
and the press can still reach the wall from there.

At 0.20 m the planned approach collided with the dilated octomap in front of
the wall "sometimes" and the partition was skipped. Raising it is only safe if
force mode's Z reach grows with it, or the press stops short of the wall. And
it is a ceiling: at the partition standoff the scanning pose already leaves the
plate ~0.25 m off, and pulling it back to 0.30 m runs the wrist into the mast.

Run with:

    python3 -m pytest test/test_scan_wall_approach.py -v
"""

import pytest

from task_planner_fsm.states.scan_wall import ScanWall


class _Node:
    def __init__(self):
        self.lines = []

    def get_logger(self):
        return self

    def warn(self, msg, *a, **kw):
        self.lines.append(msg)

    info = warn


@pytest.fixture
def state():
    state = ScanWall("ScanWall")
    # A current arm TF pose: arm moves are placed from it (1842a78).
    state._tool0_pose = lambda ctx: (object(), object(), None)
    return state


def test_the_approach_leaves_the_plate_thirty_centimetres_off(state):
    assert state._approach_standoff({}) == pytest.approx(0.30)
    assert state._approach_standoff({"sweep_approach_retract_m": 0.0}) == pytest.approx(0.20)


@pytest.mark.parametrize("ctx", [{}, {"sweep_approach_retract_m": 0.25}])
def test_force_mode_can_press_through_the_whole_approach_gap(state, ctx):
    ctx = dict(ctx, node=_Node())
    request = state._build_force_mode_request(ctx)
    # deviation_limits is float32 on the wire
    assert request.deviation_limits[2] >= state._approach_standoff(ctx) + 0.05 - 1e-6


def test_the_executor_has_no_retract_leg(state, monkeypatch):
    """A straight pull-back from a partition end curls the arm into the plate
    and the executor checks no collisions; the FSM's retracts are planned."""
    seen = []
    monkeypatch.setattr(state, "_start_arm_procs", lambda ctx, procs: seen.extend(procs) or True)
    state._start_sweep_executor({"sim": True})
    (_, cmd, _), = seen
    assert "approach_retract_m:=0.0" in cmd
    assert "scan_standoff_m:=0.2" in cmd


def _ctx_at(distance):
    import time
    return {"node": _Node(), "plate_distances": [distance] * 3 + [0.5] * 3,
            "plate_distances_stamp": time.time()}


def test_the_approach_never_pulls_the_plate_back(state, monkeypatch):
    sent = []
    monkeypatch.setattr(state, "_send_arm_z_goal", lambda ctx, advance: sent.append(advance) or True)
    # Scanning pose leaves the plate at 0.25 m: nothing to do.
    assert state._send_arm_to_clearance(_ctx_at(0.25), 0.30, advance_only=True) == "skip"
    # From further out it closes to 0.30 m.
    assert state._send_arm_to_clearance(_ctx_at(0.40), 0.30, advance_only=True) == "sent"
    assert sent == [pytest.approx(0.10)]


def test_retracts_after_a_sweep_go_to_the_sweep_standoff(state):
    assert state._retract_standoff({}) == pytest.approx(0.20)
    assert state._roll_clearance({}) == pytest.approx(0.20)
    assert state._roll_clearance({"scan_wall_roll_clearance_m": 0.15}) == pytest.approx(0.15)


def test_the_simulated_base_parks_further_from_the_wall():
    from task_planner_fsm.utils.costmap_utils import partition_base_standoff
    assert partition_base_standoff({}) == pytest.approx(1.15)
    assert partition_base_standoff({"sim": True}) == pytest.approx(1.30)
    assert partition_base_standoff({"sim": True, "partition_base_standoff_m": 1.2}) == pytest.approx(1.2)
