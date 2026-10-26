"""Wall scans take the arm over folded only when it is KNOWN folded.

ArmFolding records ctx["arm_folded"]; ArmUnfolding leaves the arm folded for
a wall scan only then, and ScanWall, handed a folded arm, skips the fold its
first transit and park would otherwise send again. Any named pose clears the
flag until a fold settles.

Run with:

    python3 -m pytest test/test_folded_handover.py -v
"""

from types import SimpleNamespace

import pytest

from task_planner_fsm.states.arm_unfolding import ArmUnfolding
from task_planner_fsm.states.scan_wall import ScanWall


class _Logger:
    def info(self, msg, **kw):
        pass

    warn = error = debug = info


class _Node:
    def get_logger(self):
        return _Logger()


def make_ctx(**over):
    ctx = {"node": _Node(), "sim": True, "scan_phase": 1}
    ctx.update(over)
    return ctx


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


@pytest.mark.parametrize("over", [
    {"arm_folded": None},                          # unknown: checkpoint start
    {"arm_folded": False},                         # out already
    {"arm_folded": True, "scan_phase": 2},         # floor scan
    {"arm_folded": True, "arm_unfolding_force_unfold": True},
])
def test_otherwise_it_still_unfolds(unfolding, over):
    unfolding.run(make_ctx(**over))
    assert not unfolding.movement_done            # waiting to send the pose


@pytest.fixture
def scan_wall():
    state = ScanWall("ScanWall")
    state._controller_gate = SimpleNamespace(ready=lambda node: True, reset=lambda: None)
    state.position_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda req: None)
    state.column = SimpleNamespace(column_min_height_m=0.0, column_max_height_m=0.9)
    state._fold_for_transit = lambda ctx: True
    return state


@pytest.mark.parametrize("handed_over, known", [(True, True), (None, False)])
def test_the_pre_approach_records_a_folded_hand_over(scan_wall, handed_over, known):
    ctx = make_ctx(arm_folded=handed_over, current_wall_scan_lines=[1.5])
    scan_wall._run_pre_approach(ctx)
    assert scan_wall.pose_reached
    assert scan_wall._arm_folded is known


def test_any_named_pose_clears_the_flag(scan_wall):
    ctx = make_ctx(arm_folded=True)
    assert scan_wall._send_named_pose(ctx, "unfolded_fsm")
    assert ctx["arm_folded"] is False
