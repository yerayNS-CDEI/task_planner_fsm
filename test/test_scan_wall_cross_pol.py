"""The cross-polarized pass: every line swept again with the plate rolled.

The GPR takes data in two modes, so each line is swept twice: upright, then with
the plate rolled CROSS_POL_PLATE_ROLL about the wall normal. What can go wrong
is mostly silent — a second pass the resume memory skips as "already swept", a
rolled plate the sweep turns back upright, a GPR scanning 8 cm off the line, a
camera recording duplicates — so each of those is pinned here.
"""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("arm_control.srv")           # the FSM state imports these
pytest.importorskip("ur_msgs.srv")

from task_planner_fsm.states.scan_wall import (   # noqa: E402
    CROSS_POL_PLATE_ROLL,
    CROSS_POL_SCAN_POSE,
    SCAN_POSE,
    SCAN_POSE_TOOL_Z_ABOVE_ARM_BASE,
    ScanWall,
    gpr_height_above_plate,
)

SEG_A = ((1.0, 2.0, 1.5), (2.0, 2.0, 1.5))
SEG_B = ((2.5, 2.0, 1.5), (4.0, 2.0, 1.5))


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    node = rclpy.create_node("scan_wall_cross_pol_test")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s.current_line_z = 1.5
    return s


def ctx_for(node, **overrides):
    ctx = {"node": node, "sim": True, "current_wall_index": 0, "current_line_idx": 0,
           "current_wall_scan_lines": [1.5, 1.1]}
    ctx.update(overrides)
    return ctx


def _plan(state, segments):
    """The sweep list as _run_scan builds it right after segmentation."""
    state._line_segments = list(segments)
    state._segments = list(segments)
    state._sweep_info = [(i, False) for i in range(len(segments))]
    state._seg_idx = 0


def _run_line(state, ctx, segments=(SEG_A, SEG_B), succeeds=lambda seg_no, rolled: True):
    """Walk the line's sweeps the way _run_scan does — sync, skip what is
    already swept, sweep, note, next — and finalize, until the line closes or
    errors. Returns what was swept, in
    order, as (segment number, rolled?, start, end)."""
    _plan(state, segments)
    swept = []
    for _ in range(4 * len(segments) + 2):            # bounded: a bug must not hang
        if state._seg_idx >= len(state._segments):
            state._finalize_line(ctx)
            if state.scan_swept or ctx.get("error_triggered"):
                return swept
            continue
        start, end = state._segments[state._seg_idx]
        state._sync_sweep()
        if state._already_swept(ctx, start, end):     # transit_clear's resume skip
            state._segments_ok += 1
            state._last_swept_point = end
            state._note_swept(ctx, start, end)
            state._seg_idx += 1
            continue
        if succeeds(state._seg_no, state._cross_pol_pass):
            swept.append((state._seg_no, state._cross_pol_pass, start, end))
            state._segments_ok += 1
            state._last_swept_point = end
            state._record_swept(ctx, start, end)
            state._note_swept(ctx, start, end)
        state._seg_idx += 1
    raise AssertionError("the line never closed")


def _rev(seg):
    return (seg[1], seg[0])


# ------------------------------------------------------------- the geometry

def test_the_upright_gpr_is_on_the_line():
    assert gpr_height_above_plate(0.0) == pytest.approx(0.0)


def test_the_chosen_roll_puts_the_gpr_above_the_plate_centre():
    """-90 deg: the GPR (plate x = -0.08) ends up 8 cm ABOVE the centre — the
    side asked for. FK over the URDF agrees (+8.7 cm at unfolded_fsm)."""
    assert CROSS_POL_PLATE_ROLL == pytest.approx(-math.pi / 2)
    assert gpr_height_above_plate(CROSS_POL_PLATE_ROLL) == pytest.approx(0.08)
    assert gpr_height_above_plate(math.pi / 2) == pytest.approx(-0.08)


def test_the_rolled_pass_lowers_the_plate_so_the_gpr_stays_on_the_line(state):
    assert state._plate_rise() == pytest.approx(0.0)
    state._cross_pol_pass = True
    assert state._plate_rise() == pytest.approx(-0.08)


def test_each_pass_unfolds_into_its_own_fixed_pose(state):
    assert state._scan_pose() == SCAN_POSE
    assert state._plate_roll() == 0.0
    state._cross_pol_pass = True
    assert state._scan_pose() == CROSS_POL_SCAN_POSE
    assert state._plate_roll() == CROSS_POL_PLATE_ROLL


# --------------------------------------------------------- sweep order

def test_by_default_each_segment_is_swept_both_ways_before_the_next(state, node):
    ctx = ctx_for(node)
    swept = _run_line(state, ctx)
    assert [(n, r) for n, r, _, _ in swept] == [(0, False), (0, True), (1, False), (1, True)]
    # The rolled sweep goes straight back over the segment it follows.
    assert [(s, e) for _, _, s, e in swept] == [SEG_A, _rev(SEG_A), SEG_B, _rev(SEG_B)]
    assert state.scan_swept and ctx["current_line_idx"] == 1
    assert ctx["target_scan_point"] == SEG_B[0]       # where the last sweep ended


def test_line_order_sweeps_the_line_upright_then_back_rolled(state, node):
    ctx = ctx_for(node, scan_wall_cross_pol_order="line")
    swept = _run_line(state, ctx)
    assert [(n, r) for n, r, _, _ in swept] == [(0, False), (1, False), (1, True), (0, True)]
    assert [(s, e) for _, _, s, e in swept] == [SEG_A, SEG_B, _rev(SEG_B), _rev(SEG_A)]
    assert state.scan_swept and ctx["current_line_idx"] == 1
    assert ctx["target_scan_point"] == SEG_A[0]


@pytest.mark.parametrize("order", ["segment", "line"])
def test_a_rolled_sweep_carries_its_own_segments_number(state, node, order):
    """The GPR line is named after the segment, so the rolled sweep of segment
    2 must say 2 whichever order it runs in — in "line" order it is the FIRST
    rolled sweep, which is what numbered it 1 before."""
    swept = _run_line(state, ctx_for(node, scan_wall_cross_pol_order=order))
    by_segment = {(n, r): (s, e) for n, r, s, e in swept}
    assert by_segment[(1, False)] == SEG_B and by_segment[(1, True)] == _rev(SEG_B)
    assert by_segment[(0, False)] == SEG_A and by_segment[(0, True)] == _rev(SEG_A)


@pytest.mark.parametrize("order", ["segment", "line"])
def test_only_segments_swept_upright_are_swept_rolled(state, node, order):
    ctx = ctx_for(node, scan_wall_cross_pol_order=order)
    swept = _run_line(state, ctx, succeeds=lambda n, rolled: n != 0 or rolled)
    assert [(n, r) for n, r, _, _ in swept] == [(1, False), (1, True)]
    assert state.scan_swept


@pytest.mark.parametrize("order", ["segment", "line"])
def test_rolled_sweeps_that_all_fail_keep_the_line(state, node, order):
    """The upright sweeps have the line's data; failing the wall would lose it."""
    ctx = ctx_for(node, scan_wall_cross_pol_order=order)
    _run_line(state, ctx, succeeds=lambda n, rolled: not rolled)
    assert state.scan_swept and not ctx.get("error_triggered")


@pytest.mark.parametrize("order", ["segment", "line"])
def test_upright_sweeps_that_all_fail_are_still_an_error(state, node, order):
    ctx = ctx_for(node, scan_wall_cross_pol_order=order)
    swept = _run_line(state, ctx, succeeds=lambda n, rolled: False)
    assert swept == [] and ctx.get("error_triggered")


def test_turned_off_each_segment_is_swept_once(state, node):
    ctx = ctx_for(node, scan_wall_cross_pol=False)
    swept = _run_line(state, ctx)
    assert [(n, r) for n, r, _, _ in swept] == [(0, False), (1, False)]
    assert state.scan_swept


@pytest.mark.parametrize("over, starts_at", [
    ({}, "far"),                                              # segment order
    ({"scan_wall_cross_pol_order": "line"}, "near"),
    ({"scan_wall_cross_pol": False}, "far"),
])
def test_the_next_line_starts_where_the_robot_ends(state, node, over, starts_at):
    """Normally the next line starts from the far end (serpentine). In "line"
    order the rolled sweeps bring the robot back to where the line began, and
    flipping would cost a transit along the whole wall."""
    near, far = (1.0, 2.0, 1.5), (4.0, 2.0, 1.5)
    state._sweep_from, state._sweep_to = near, far
    ctx = ctx_for(node, **over)
    _run_line(state, ctx)
    assert state.scan_swept
    assert ctx["target_scan_start_end"] == (far if starts_at == "far" else near)


def test_an_unknown_order_falls_back_to_the_default(state, node):
    ctx = ctx_for(node, scan_wall_cross_pol_order="sideways")
    assert state._cross_pol_order(ctx) == "segment"
    swept = _run_line(state, ctx)
    assert [(n, r) for n, r, _, _ in swept][:2] == [(0, False), (0, True)]


@pytest.mark.parametrize("order", ["segment", "line"])
def test_the_rolled_sweep_parks_the_chassis_turned_round(state, node, monkeypatch, order):
    """Reversed, the rolled sweep runs the other way along the turret, which is
    what turns the chassis round (park_target_phi) so the base still pushes."""
    monkeypatch.setattr(state, "_base_xy_yaw_map", lambda ctx: (0.0, 0.0, 0.0))
    ctx = ctx_for(node, scan_wall_cross_pol_order=order)
    _plan(state, [SEG_A])
    state._sync_sweep()
    assert state._park_target_for_segment(ctx) == 0.0
    state._note_swept(ctx, *SEG_A)
    state._seg_idx += 1
    if order == "line":
        state._finalize_line(ctx)
    state._sync_sweep()
    assert state._cross_pol_pass
    assert state._park_target_for_segment(ctx) == pytest.approx(math.pi)


class _GprStub:
    def __init__(self):
        self.opened = []

    def open_measurement(self, ctx, wall_index, line_idx, seg_idx, polarization=None,
                         plate_roll_deg=0.0):
        self.opened.append((seg_idx, polarization, plate_roll_deg))


def test_the_gpr_measurement_is_named_after_the_lines_segment(state, node):
    """Through sweep_setup itself: the rolled sweep of segment 2, first of the
    rolled sweeps in line order, opens 's01 xpol' — not 's00'."""
    ctx = ctx_for(node, scan_wall_cross_pol_order="line", sweep_use_wbc=True)
    state._gpr = _GprStub()
    state.started = True
    _plan(state, [SEG_A, SEG_B])
    for _ in range(2):                              # both upright sweeps covered
        state._sync_sweep()
        state._note_swept(ctx, *state._segments[state._seg_idx])
        state._seg_idx += 1
    state._finalize_line(ctx)
    state._seg_phase = "sweep_setup"
    state._run_scan(ctx)
    assert state._gpr.opened == [(1, "xpol", pytest.approx(-90.0))]
    assert state._seg_phase == "arm_approach"


# ------------------------------------------------- the plate actually rolls

def test_an_upright_arm_is_unfolded_again_for_the_rolled_pass(state):
    """The roll happens in the unfold. An arm that is out but upright (no fold
    before it: parking or the transit fold turned off) must not go straight to
    the sweep, or the 'cross-polarized' pass is a second upright one."""
    state._cross_pol_pass = True
    state._arm_unfolded = True
    state._unfolded_pose = SCAN_POSE
    state._after_park_phase()
    assert state._seg_phase == "unfold"


def test_an_arm_already_rolled_goes_straight_to_the_sweep(state):
    state._cross_pol_pass = True
    state._arm_unfolded = True
    state._unfolded_pose = CROSS_POL_SCAN_POSE
    state._after_park_phase()
    assert state._seg_phase == "sweep_setup"


def test_the_upright_pass_still_skips_the_unfold_when_the_arm_is_out(state):
    state._arm_unfolded = True
    state._after_park_phase()
    assert state._seg_phase == "sweep_setup"


# ----------------------------------------------------------- resume memory

def test_the_rolled_pass_is_not_skipped_as_already_swept(state, node):
    ctx = ctx_for(node)
    state._record_swept(ctx, *SEG_A)
    assert state._already_swept(ctx, *SEG_A)
    state._cross_pol_pass = True
    assert not state._already_swept(ctx, SEG_A[1], SEG_A[0]), (
        "the rolled pass was skipped on ground the upright one covered")
    state._record_swept(ctx, SEG_A[1], SEG_A[0])
    assert state._already_swept(ctx, SEG_A[1], SEG_A[0])


@pytest.mark.parametrize("order", ["segment", "line"])
def test_a_resumed_line_still_gets_its_rolled_sweeps(state, node, order):
    """Re-entry: the upright sweeps are skipped as already swept, but each still
    belongs to the line, so its rolled sweep must run — and a rolled sweep that
    is ALSO done is skipped in turn."""
    ctx = ctx_for(node, scan_wall_cross_pol_order=order)
    _run_line(state, ctx, succeeds=lambda n, rolled: not (n == 1 and rolled))
    fresh = ScanWall("ScanWall")
    fresh.current_line_z = state.current_line_z
    ctx["current_line_idx"] = 0
    redo = _run_line(fresh, ctx)
    assert [(n, r) for n, r, _, _ in redo] == [(1, True)]
    assert fresh.scan_swept


# ------------------------------------------------- what the sweep is told

def _captured_cmd(state, monkeypatch):
    seen = {}

    def capture(ctx, procs):
        seen["cmd"] = procs[0][1]
        return True
    monkeypatch.setattr(state, "_start_arm_procs", capture)
    return seen


def _param(cmd, name):
    for arg in cmd:
        if arg.startswith(f"{name}:="):
            return arg.split(":=", 1)[1]
    return None


def test_the_whole_body_sweep_holds_the_roll_and_the_lowered_row(state, node, monkeypatch):
    seen = _captured_cmd(state, monkeypatch)
    state._start_wbc_sweep(ctx_for(node), *SEG_A)
    assert float(_param(seen["cmd"], "plate_roll")) == 0.0
    assert float(_param(seen["cmd"], "row_z")) == pytest.approx(1.5)

    state._cross_pol_pass = True
    state._start_wbc_sweep(ctx_for(node), *SEG_A)
    assert float(_param(seen["cmd"], "plate_roll")) == pytest.approx(CROSS_POL_PLATE_ROLL)
    assert float(_param(seen["cmd"], "row_z")) == pytest.approx(1.42)


def test_the_wall_parallel_controller_is_told_the_roll(state, node, monkeypatch):
    """Its roll compensation rebuilds the orientation from scratch: without the
    roll, its first correction is a 90 deg wrist_3 snap on the wall."""
    seen = _captured_cmd(state, monkeypatch)
    state._start_wall_alignment(ctx_for(node))
    assert float(_param(seen["cmd"], "plate_roll")) == 0.0
    state._cross_pol_pass = True
    state._start_wall_alignment(ctx_for(node))
    assert float(_param(seen["cmd"], "plate_roll")) == pytest.approx(CROSS_POL_PLATE_ROLL)


def test_the_approach_lowers_the_plate_even_at_the_right_standoff(state, node, monkeypatch):
    """The Nav2/crawl sweeps have no height task: the approach is where the
    rolled plate is lowered, so it must not be skipped as 'already there'."""
    sent = []
    monkeypatch.setattr(state, "_send_arm_z_goal",
                        lambda ctx, advance, rise=0.0: sent.append((advance, rise)) or True)
    ctx = ctx_for(node, plate_distances=[0.20] * 3 + [0.20] * 3,
                  plate_distances_stamp=time.time())
    assert state._send_arm_to_clearance(ctx, 0.20) == "skip"
    assert state._send_arm_to_clearance(ctx, 0.20, rise=-0.08) == "sent"
    assert sent == [(0.0, -0.08)]


# ------------------------------------------------------------- the column

class _Buffer:
    def __init__(self, z):
        self.z = z

    def lookup_transform(self, target, source, when):
        z = self.z

        class _T:
            class transform:
                class translation:
                    pass
        _T.transform.translation.z = z
        return _T


MOUNT_Z = 1.1125
TOOL_Z = MOUNT_Z + SCAN_POSE_TOOL_Z_ABOVE_ARM_BASE


def _column(state, node, line_z, **over):
    ctx = {"node": node, "sim": True, "tf_buffer": _Buffer(MOUNT_Z),
           "column_current_height": 0.0}
    ctx.update(over)
    return state._column_target_for_line(ctx, line_z)


def test_the_column_covers_both_passes_heights(state, node):
    """A row 0.22 m below the tool is inside the arm's 0.25 m window — but the
    rolled pass holds the plate 8 cm lower still, at 0.30. One column height
    has to serve both, so it moves by the excess of the pair."""
    line_z = TOOL_Z - 0.22
    assert _column(state, node, line_z, scan_wall_cross_pol=False) == pytest.approx(0.0)
    # The column starts at 0 and would clamp at its floor; lower the floor so
    # the arithmetic is visible.
    state.column.column_min_height_m = -1.0
    target = _column(state, node, line_z)
    assert target == pytest.approx(-0.05)
    for z in (line_z, line_z - 0.08):          # both plate heights within reach
        assert abs(z - (TOOL_Z + target)) <= 0.25 + 1e-9


def test_a_line_both_passes_reach_leaves_the_column_alone(state, node):
    assert _column(state, node, TOOL_Z + 0.1) == pytest.approx(0.0)
