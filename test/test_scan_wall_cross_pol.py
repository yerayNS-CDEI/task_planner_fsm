"""Base placement's ScanWall: the cross-polarized pass, the fold rule between
reachable segments, and the column sized from the arm mount.

At each base stop, every height is swept upright and then straight back over
the same partition with the plate rolled, before the column moves on; the
plate is pulled off the wall before every roll and un-roll. The arm folds only
for a transit to a different reachable segment. No rclpy graph: the node and
TF buffer are fakes, and the phase machine is driven through its helpers.

Run with:

    python3 -m pytest test/test_scan_wall_cross_pol.py -v
"""

import math
from types import SimpleNamespace

import pytest

from task_planner_fsm.states import scan_wall as scan_wall_mod
from task_planner_fsm.states.scan_wall import (
    CROSS_POL_PLATE_ROLL,
    ScanWall,
    gpr_height_above_plate,
)


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


PART = ((0.0, 0.0, 1.0), (0.8, 0.0, 1.0))


def make_ctx(**over):
    ctx = {
        "node": _Node(),
        "current_wall_index": 1,
        "current_wall_scan_lines": [0.9, 1.4],
        "current_line_idx": 0,
        "current_wall_partition_sources": [0, 0, 1],
    }
    ctx.update(over)
    return ctx


@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s._segments = [PART, PART, PART]
    s._seg_idx = 0
    s._line_idx = 0
    s.current_line_z = 0.9
    return s


def _after(state, ctx, ok=True, reason="", skipped=False):
    start, end = PART if not state._xpol else (PART[1], PART[0])
    state._advance_after_sweep(ctx, ok, reason, start, end, True, skipped=skipped)


# ---------------------------------------------------------------------------
# Ordering at one base stop: (copol, xpol) per height, then the next height
# ---------------------------------------------------------------------------

def test_each_height_is_swept_upright_then_rolled_before_the_column_moves(state):
    ctx = make_ctx()
    _after(state, ctx)                                    # height 1, upright
    assert state._xpol and state._seg_phase == "xpol_clear"
    assert state._line_idx == 0                           # same height, same column

    _after(state, ctx)                                    # height 1, rolled
    assert not state._xpol
    assert state._line_idx == 1 and ctx["current_line_idx"] == 1
    assert state._seg_phase == "unroll_clear"             # upright again before height 2

    state.current_line_z = 1.4
    _after(state, ctx)                                    # height 2, upright
    assert state._xpol and state._seg_phase == "xpol_clear"
    _after(state, ctx)                                    # height 2, rolled
    assert not state._xpol
    assert state._seg_idx == 1 and state._line_idx == 0
    assert state._seg_phase == "transit_clear"            # next partition re-centres upright
    assert state._segments_ok == 2 and state._xpol_ok == 2


def test_a_failed_upright_sweep_gets_no_rolled_pass(state):
    ctx = make_ctx()
    _after(state, ctx, ok=False, reason="controller_timeout")
    assert not state._xpol
    assert state._seg_phase == "line_change" and state._line_idx == 1


def test_a_failed_rolled_sweep_keeps_the_upright_one_and_still_unrolls(state):
    ctx = make_ctx()
    _after(state, ctx)
    _after(state, ctx, ok=False, reason="ik_unreachable")
    assert state._segments_ok == 1 and state._xpol_ok == 0
    # Never re-cut a partition the upright pass already covered.
    assert state._seg_idx == 0 and state._seg_phase == "unroll_clear"


def test_without_cross_pol_heights_follow_each_other_directly(state):
    ctx = make_ctx(scan_wall_cross_pol=False)
    _after(state, ctx)
    assert not state._xpol and state._seg_phase == "line_change"
    assert state._polarization(ctx) is None


def test_a_rolled_pass_done_on_an_earlier_entry_is_not_rolled_for_again(state):
    ctx = make_ctx()
    state._xpol = True
    state._record_swept(ctx, PART[1], PART[0])            # the rolled pass, earlier
    state._xpol = False
    _after(state, ctx)
    assert not state._xpol and state._seg_phase == "line_change"


def test_swept_memory_keys_the_rolled_pass_apart(state):
    ctx = make_ctx()
    state._record_swept(ctx, *PART[:2])
    assert state._already_swept(ctx, PART[1], PART[0])    # unordered
    assert not state._already_swept(ctx, *PART[:2], xpol=True)
    assert not state._partition_fully_swept(ctx, *PART[:2])
    for z in (0.9, 1.4):
        for xpol in (False, True):
            ctx["scan_wall_swept_segments"].setdefault(
                state._swept_key(ctx, z=z, xpol=xpol), []).append(
                    ((PART[0][0], PART[0][1]), (PART[1][0], PART[1][1])))
    assert state._partition_fully_swept(ctx, *PART[:2])


# ---------------------------------------------------------------------------
# The rolled pass: pose, roll, height
# ---------------------------------------------------------------------------

def test_the_rolled_pass_uses_the_rolled_twin_of_the_scanning_pose(state):
    ctx = make_ctx()                                      # arm sweep: faces the wall
    assert state._scan_pose_name(ctx) == "unfolded_front_fsm"
    state._xpol = True
    assert state._scan_pose_name(ctx) == "unfolded_front_fsm_xpol"
    assert state._scan_pose_name(make_ctx(nav_face_wall=False)) == "unfolded_fsm_xpol"
    assert state._scan_pose_name(make_ctx(scan_wall_xpol_pose="bench_xpol")) == "bench_xpol"


def test_the_rolled_plate_is_lowered_so_the_gpr_stays_on_the_line(state):
    # GPR at plate x = -0.08: rolled -90 deg it sits 8 cm above the centre.
    assert gpr_height_above_plate(CROSS_POL_PLATE_ROLL) == pytest.approx(0.08)
    assert gpr_height_above_plate(0.0) == pytest.approx(0.0)
    assert state._plate_rise() == 0.0
    state._xpol = True
    assert state._plate_rise() == pytest.approx(-0.08)
    assert state._plate_roll() == pytest.approx(-math.pi / 2)


def test_the_sweep_goal_carries_the_rolled_height(state):
    sent = {}
    state._sweep_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda goal, feedback_callback: (
            sent.__setitem__("goal", goal),
            SimpleNamespace(add_done_callback=lambda cb: None))[1],
    )
    ctx = make_ctx(sweep_executor_proc=None)
    state._xpol = True
    assert state._send_sweep_goal(ctx, PART[1], PART[0]) is True
    assert sent["goal"].start.z == pytest.approx(0.9 - 0.08)
    assert (sent["goal"].start.x, sent["goal"].end.x) == (0.8, 0.0)


def test_the_alignment_controller_is_told_the_roll(state, monkeypatch):
    seen = []
    monkeypatch.setattr(state, "_start_arm_procs", lambda ctx, procs: seen.extend(procs) or True)
    state._xpol = True
    state._start_wall_alignment(make_ctx())
    (_, cmd, _), = seen
    assert f"plate_roll:={-math.pi / 2}" in cmd


def test_gpr_lines_are_named_by_polarization(state):
    ctx = make_ctx()
    assert state._polarization(ctx) == "copol"
    state._xpol = True
    assert state._polarization(ctx) == "xpol"


# ---------------------------------------------------------------------------
# Fold only between reachable segments
# ---------------------------------------------------------------------------

def test_the_arm_folds_only_for_another_reachable_segment(state):
    ctx = make_ctx()
    assert state._fold_before_transit(ctx)                # nothing visited yet
    state._last_source = 0
    state._seg_idx = 1                                    # same segment 0
    assert not state._fold_before_transit(ctx)
    state._seg_idx = 2                                    # segment 1
    assert state._fold_before_transit(ctx)
    assert not state._fold_before_transit(make_ctx(scan_wall_fold_between_segments=False))


def test_arrival_unfolds_a_folded_arm_before_the_column(state):
    ctx = make_ctx()
    state._seg_idx = 2
    state._arm_folded = True
    state._at_partition(ctx)
    assert state._seg_phase == "transit_unfold" and state._last_source == 1
    state._arm_folded = False
    state._at_partition(ctx)
    assert state._seg_phase == "line_column"


# ---------------------------------------------------------------------------
# Column sized from the arm mount
# ---------------------------------------------------------------------------

class _Tf:
    def __init__(self, z):
        self.z = z

    def lookup_transform(self, *a, **kw):
        t = SimpleNamespace(x=0.0, y=0.0, z=self.z)
        return SimpleNamespace(transform=SimpleNamespace(translation=t))


@pytest.fixture
def column_state(state):
    state.column = SimpleNamespace(column_min_height_m=0.0, column_max_height_m=0.9)
    return state


def test_the_column_waits_for_its_inputs_instead_of_guessing(column_state):
    assert column_state._column_target_for_line(make_ctx(), 1.0) is None
    ctx = make_ctx(column_current_height=0.0)
    assert column_state._column_target_for_line(ctx, 1.0) is None   # no TF yet


def test_the_column_stays_put_for_a_height_the_arm_reaches(column_state):
    # Mount at 0.78 m: the scanning pose's tool sits at 1.3017 m.
    ctx = make_ctx(column_current_height=0.0, tf_buffer=_Tf(0.78))
    assert column_state._column_target_for_line(ctx, 1.30) == 0.0


def test_the_column_covers_only_what_the_arm_cannot(column_state):
    """Both passes' heights count: the band 1.80/1.72 m is centred at 1.76 m,
    and the arm's 0.25 m window shrinks by half the band."""
    ctx = make_ctx(column_current_height=0.0, tf_buffer=_Tf(0.78))
    tool_z = 0.78 + scan_wall_mod.SCAN_POSE_TOOL_Z_ABOVE_ARM_BASE
    expected = (1.76 - tool_z) - (0.25 - 0.04)
    assert column_state._column_target_for_line(ctx, 1.80) == pytest.approx(expected)
    no_xpol = make_ctx(column_current_height=0.0, tf_buffer=_Tf(0.78),
                       scan_wall_cross_pol=False)
    assert column_state._column_target_for_line(no_xpol, 1.80) == pytest.approx(
        (1.80 - tool_z) - 0.25)


# ---------------------------------------------------------------------------
# Rolling in place
# ---------------------------------------------------------------------------

class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def test_the_in_place_roll_turns_the_plate_about_its_own_normal(state):
    from scipy.spatial.transform import Rotation as R
    q0 = R.from_euler("xyz", [0.3, -1.1, 0.7])
    x, y, z, w = q0.as_quat()
    p = SimpleNamespace(x=0.5, y=-0.4, z=0.6)
    state._tool0_pose = lambda ctx: (p, SimpleNamespace(x=x, y=y, z=z, w=w))
    state._arm_goal_pub = _Pub()
    ctx = make_ctx()
    from builtin_interfaces.msg import Time
    ctx["node"].get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    assert state._send_arm_roll_goal(ctx, CROSS_POL_PLATE_ROLL)
    (msg,) = state._arm_goal_pub.sent
    o = msg.pose.orientation
    got = R.from_quat([o.x, o.y, o.z, o.w])
    want = q0 * R.from_euler("z", CROSS_POL_PLATE_ROLL)
    assert (got.inv() * want).magnitude() == pytest.approx(0.0, abs=1e-9)
    # Same position, same plate normal: only the roll about it changed.
    assert (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) == (0.5, -0.4, 0.6)
    assert got.apply([0, 0, 1]) == pytest.approx(q0.apply([0, 0, 1]))
    assert ctx["execution_status"] is False and ctx["planner_goal_failed"] is False


def test_the_retract_for_the_roll_never_reaches_behind_the_scanning_pose(state):
    ctx = make_ctx()
    assert state._roll_in_place_clearance(ctx) == pytest.approx(0.30)   # sim: room to spare
    state._scan_pose_gap = 0.25                                          # robot's 1.15 m standoff
    assert state._roll_in_place_clearance(ctx) == pytest.approx(0.25)
    state._scan_pose_gap = 0.12
    assert state._roll_in_place_clearance(ctx) == pytest.approx(0.20)   # never under the first retract
    assert state._roll_in_place_clearance(
        make_ctx(scan_wall_roll_in_place_clearance_m=0.22)) == pytest.approx(0.22)


def test_the_roll_in_place_can_be_turned_off(state):
    assert state._roll_in_place_enabled(make_ctx())
    assert not state._roll_in_place_enabled(make_ctx(scan_wall_roll_in_place=False))
