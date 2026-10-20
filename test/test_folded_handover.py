"""Arm-sweep wall scans hand the arm from ArmUnfolding to ScanWall FOLDED, and
ScanWall unfolds it once, after the base is at the first partition's scan pose.

Unfolding in ArmUnfolding cost an unfold -> fold -> unfold whenever
NavigateToTarget stopped outside that pose's tolerance (simulation,
2026-10-20: 0.22 m off), because ScanWall folds the arm for the correction.

Run with:

    python3 -m pytest test/test_folded_handover.py -v
"""

import inspect
from types import SimpleNamespace

import pytest

from task_planner_fsm.states.arm_unfolding import ArmUnfolding
from task_planner_fsm.states.scan_wall import ScanWall


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(msg)

    warn = error = debug = info


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def make_ctx(**over):
    ctx = {"node": _Node(), "sim": True, "scan_phase": 1}
    ctx.update(over)
    return ctx


# ---------------------------------------------------------------------------
# ArmUnfolding
# ---------------------------------------------------------------------------

@pytest.fixture
def unfolding():
    state = ArmUnfolding("ArmUnfolding")
    state.controller_gate = SimpleNamespace(ready=lambda node: False)   # no robot
    return state


def test_a_folded_arm_is_handed_to_the_wall_scan_folded(unfolding):
    ctx = make_ctx(arm_folded=True)
    unfolding.run(ctx)
    assert unfolding.movement_done and ctx["unfolding_success"]
    assert unfolding.check_transition(ctx) == "ScanWall"
    assert ctx["arm_folded"] is True


@pytest.mark.parametrize("over", [
    {"arm_folded": None},                          # unknown: checkpoint start
    {"arm_folded": False},                         # out already
    {"arm_folded": True, "scan_phase": 2},         # floor scan
    {"arm_folded": True, "sweep_use_arm": False},  # base-driven sweep
    {"arm_folded": True, "arm_unfolding_force_unfold": True},
])
def test_otherwise_it_still_unfolds(unfolding, over):
    ctx = make_ctx(**over)
    unfolding.run(ctx)
    assert not unfolding.movement_done            # waiting to send the pose


# ---------------------------------------------------------------------------
# ScanWall pre-approach
# ---------------------------------------------------------------------------

@pytest.fixture
def scan_wall():
    state = ScanWall("ScanWall")
    state.position_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda req: pytest.fail("the pre-approach must not unfold"),
    )
    state._controller_gate = SimpleNamespace(ready=lambda node: True, reset=lambda: None)
    state.column = SimpleNamespace(column_min_height_m=0.0, column_max_height_m=0.9)
    return state


def test_the_pre_approach_keeps_a_folded_arm_folded(scan_wall):
    ctx = make_ctx(arm_folded=True, current_wall_scan_lines=[1.5])
    scan_wall._run_pre_approach(ctx)      # the column waits for its inputs
    assert scan_wall.pose_reached and scan_wall._arm_folded
    assert scan_wall.current_line_z == 1.5


def test_the_pre_approach_still_unfolds_an_arm_of_unknown_state(scan_wall):
    sent = []
    scan_wall.position_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda req: sent.append(req.position_name))
    ctx = make_ctx(current_wall_scan_lines=[1.5])
    scan_wall._run_pre_approach(ctx)
    assert sent == ["unfolded_front_fsm"] and not scan_wall._arm_folded


def test_a_folded_arm_is_never_re_centred(scan_wall):
    """The re-centre's wait swallows failures: unfolding through it could
    sweep with the arm still folded. A folded arm unfolds on arrival instead,
    through the strict check (transit_unfold)."""
    source = inspect.getsource(ScanWall._run_scan)
    body = source[source.index('if self._seg_phase == "transit_recenter":'):]
    body = body[:body.index('if self._seg_phase == "transit_recenter_wait":')]
    assert body.index("if self._arm_folded:") < body.index("_send_named_pose")


def test_the_unfold_on_arrival_is_recorded_for_the_other_states(scan_wall):
    ctx = make_ctx(arm_folded=True)
    scan_wall._note_arm_folded(ctx, False)
    assert ctx["arm_folded"] is False and not scan_wall._arm_folded
