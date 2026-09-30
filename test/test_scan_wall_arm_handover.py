"""Who unfolds the arm, and what must never sweep folded.

ArmUnfolding now hands the arm to ScanWall FOLDED for wall scans, because the
first transit would fold it again anyway (measured: 87 s unfold + 89 s fold of
pure waste per scan line). That moves an invariant: ScanWall may no longer assume
the pre-approach left the arm extended.

The failure this guards against is silent. Nothing errors if the sweep starts
folded — the plate simply sits nowhere near the wall and the whole segment is
scanned as garbage. So every path through "park" (and from there the sweep)
is pinned here, not just the common one.
"""

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("arm_control.srv")           # the FSM state imports these
pytest.importorskip("ur_msgs.srv")

from task_planner_fsm.states.scan_wall import ScanWall   # noqa: E402


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    node = rclpy.create_node("scan_wall_handover_test")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s.current_line_z = 1.5
    return s


def ctx_for(node, **overrides):
    ctx = {"node": node, "sim": True, "current_wall_index": 0}
    ctx.update(overrides)
    return ctx


# ---------------------------------------------------------------- park guard
#
# The park runs with the arm folded (rotating the chassis with it out has put the
# plate into the wall) and the unfold follows it. Both halves are pinned: a park
# with the arm out, and a sweep with it folded.

def test_park_folds_first_when_the_arm_is_out(state, node):
    state._arm_unfolded = True
    state._arm_folded = False
    state._begin_park_phase(ctx_for(node))
    assert state._seg_phase == "park_fold", "the chassis would rotate with the arm out"


def test_park_folds_first_when_the_arm_pose_is_unknown(state, node):
    """A new line starts here: the post-scan retract left the arm in unfolded_fsm,
    but nothing on this entry has seen it, so neither flag is set."""
    state._arm_unfolded = False
    state._arm_folded = False
    state._begin_park_phase(ctx_for(node))
    assert state._seg_phase == "park_fold"


def test_park_proceeds_directly_when_the_arm_is_folded(state, node):
    state._arm_folded = True
    state._begin_park_phase(ctx_for(node))
    assert state._seg_phase == "park"


def test_the_arm_unfolds_after_the_park(state):
    """The core of the handover change: nothing else unfolds it before the sweep."""
    state._arm_unfolded = False
    state._after_park_phase()
    assert state._seg_phase == "unfold", (
        "a folded arm reached the sweep: the plate would never touch the wall")


def test_the_sweep_follows_the_park_when_the_arm_is_already_out(state):
    state._arm_unfolded = True
    state._after_park_phase()
    assert state._seg_phase == "sweep_setup"


def test_no_fold_when_parking_is_disabled(state, node):
    """Nothing rotates, so the fold would be pure waste — but the unfold still
    has to happen."""
    state._arm_unfolded = False
    state._arm_folded = True
    state._begin_park_phase(ctx_for(node, scan_wall_park_base=False))
    assert state._seg_phase == "unfold"


def test_park_resets_the_parking_cycle_it_is_about_to_run(state, node):
    """Guard against the fold detour skipping _reset_park_state: a stale
    park_done from the previous segment would skip chassis alignment entirely."""
    state._arm_folded = False
    state.park_done = True
    state._park_phase = "settle"
    state._begin_park_phase(ctx_for(node))
    assert not state.park_done and state._park_phase == "enable"


# ------------------------------------------------------- swept-segment memory

SEG_A = ((1.0, 1.4, 1.5), (3.0, 1.4, 1.5))
SEG_B = ((5.0, 1.4, 1.5), (7.0, 1.4, 1.5))


def test_a_swept_segment_is_skipped_on_re_entry(state, node):
    ctx = ctx_for(node)
    assert not state._already_swept(ctx, *SEG_A)
    state._record_swept(ctx, *SEG_A)
    assert state._already_swept(ctx, *SEG_A)


def test_an_unswept_segment_on_the_same_line_is_not_skipped(state, node):
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    assert not state._already_swept(ctx, *SEG_B)


def test_the_record_survives_on_enter(state, node):
    """The whole point: on_enter wipes _segments and _seg_idx, so the record has
    to live in ctx. If it ever moves onto self, resume silently stops working."""
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    assert "scan_wall_swept_segments" in ctx
    fresh = ScanWall("ScanWall")
    fresh.current_line_z = 1.5
    assert fresh._already_swept(ctx, *SEG_A)


def test_a_different_scan_height_is_not_skipped(state, node):
    """Same x-span, different z, is a genuinely different scan line. Skipping it
    would silently drop a whole height from the wall."""
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    state.current_line_z = 2.1
    assert not state._already_swept(ctx, *SEG_A)


def test_a_different_wall_is_not_skipped(state, node):
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    ctx["current_wall_index"] = 1
    assert not state._already_swept(ctx, *SEG_A)


def test_the_segment_matches_when_the_serpentine_reverses_it(state, node):
    """A re-entry can approach the wall from the other end, which flips the
    segment's endpoints without changing the ground it covers."""
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    assert state._already_swept(ctx, SEG_A[1], SEG_A[0])


def test_endpoints_are_matched_with_tolerance_not_exactly(state, node):
    """Segmentation re-runs against a costmap that has moved on, so the same
    stretch comes back shifted by the sample step plus clearance padding. An
    exact match would make resume never fire in practice."""
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    nudged = ((1.12, 1.42, 1.5), (2.9, 1.38, 1.5))
    assert state._already_swept(ctx, *nudged)


def test_a_segment_shifted_beyond_tolerance_is_swept_again(state, node):
    """The tolerance must not be so loose that a neighbouring stretch of wall
    counts as done — that would leave real gaps unscanned."""
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    shifted = ((1.8, 1.4, 1.5), (3.8, 1.4, 1.5))
    assert not state._already_swept(ctx, *shifted)


def test_resume_can_be_turned_off(state, node):
    ctx = ctx_for(node, scan_wall_resume=False)
    state._record_swept(ctx, *SEG_A)
    assert not state._already_swept(ctx, *SEG_A)
