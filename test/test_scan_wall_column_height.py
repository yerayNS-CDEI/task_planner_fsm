"""What decides the lift column's height for a scan row.

The column is the OUTER loop: it exists for height the arm cannot cover itself.
It used to be sized by the gap between the row and wherever the tool happened to
be, which handed it the arm's own workspace too — and, worse, made the answer
depend on the arm's POSE at the instant the pre-approach ran. ArmUnfolding hands
the arm over folded on a wall's first line and post-scan leaves it unfolded on
every line after, so the same 1.5 m row raised the column ~7 cm on line 1 and not
at all on line 2. That is the bug these tests exist for, and
test_the_answer_does_not_depend_on_the_arms_pose is the one that would have
caught it.

Heights here are the real robot's, from FK over the URDF: at column=0 the mount
sits 1.1125 m above the map floor and the unfolded_fsm pose puts arm_tool0
0.5217 m above the mount, so the tool reaches 1.634 m with the column down.
"""

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("arm_control.srv")           # the FSM state imports these
pytest.importorskip("ur_msgs.srv")

from task_planner_fsm.states.scan_wall import (   # noqa: E402
    ARM_Z_WINDOW_M,
    SCAN_POSE_TOOL_Z_ABOVE_ARM_BASE,
    ScanWall,
)

# Arm mount height in map with the column fully down (FK: 0.2575 turret + 0.73
# mount above base_link, + 0.125 base_link above base_footprint).
MOUNT_Z_COLUMN_DOWN = 1.1125
# Where the scanning pose puts the tool from there.
TOOL_Z_COLUMN_DOWN = MOUNT_Z_COLUMN_DOWN + SCAN_POSE_TOOL_Z_ABOVE_ARM_BASE   # 1.634


class FakeBuffer:
    """Just enough tf2 to answer (or refuse) one lookup."""

    def __init__(self, arm_base_z):
        self.arm_base_z = arm_base_z
        self.frames = []

    def lookup_transform(self, target, source, when):
        self.frames.append((target, source))
        if self.arm_base_z is None:
            raise RuntimeError("no such transform")

        class _T:
            class transform:
                class translation:
                    z = self.arm_base_z
        return _T


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    node = rclpy.create_node("scan_wall_column_height_test")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def state():
    return ScanWall("ScanWall")


def ctx_for(node, arm_base_z=MOUNT_Z_COLUMN_DOWN, column=0.0, **overrides):
    ctx = {"node": node, "sim": True, "tf_buffer": FakeBuffer(arm_base_z)}
    if column is not None:
        ctx["column_current_height"] = column
    ctx.update(overrides)
    return ctx


# ------------------------------------------------- the reported bug, directly

def test_a_row_the_arm_can_reach_leaves_the_column_alone(state, node):
    """1.5 m is 0.134 m under the scanning pose's tool height — the arm's job."""
    ctx = ctx_for(node)
    assert state._column_target_for_line(ctx, 1.5) == pytest.approx(0.0)


def test_the_answer_does_not_depend_on_the_arms_pose(state, node):
    """The regression guard.

    The old code asked TF where the TOOL was, so a folded arm (1.428 m) and an
    unfolded one (1.634 m) gave different columns for the same row. The only
    way that cannot happen again is for no tool frame to be consulted at all:
    arm_base_link is the same transform in every arm configuration.
    """
    ctx = ctx_for(node)
    assert state._column_target_for_line(ctx, 1.5) == pytest.approx(0.0)

    consulted = {frame for _, frame in ctx["tf_buffer"].frames}
    assert consulted == {"arm_base_link"}, (
        f"the column was sized from {consulted - {'arm_base_link'}}, which moves "
        f"with the arm's pose")


def test_no_pose_dependent_input_is_read_at_all(state, node):
    """Belt and braces: a buffer that refuses every frame but the mount still
    produces the same answer, and the arm's joint state is never touched."""
    ctx = ctx_for(node)
    buffer = ctx["tf_buffer"]
    original = buffer.lookup_transform

    def mount_only(target, source, when):
        assert source == "arm_base_link", f"pose-dependent frame '{source}' consulted"
        return original(target, source, when)

    buffer.lookup_transform = mount_only
    node.current_joint_state = None      # what _capture_unfolded_joints reads
    assert state._column_target_for_line(ctx, 1.5) == pytest.approx(0.0)


def test_a_row_out_of_the_arms_window_moves_the_column_by_the_excess_only(state, node):
    """2.0 m is 0.366 m above the tool height: 0.25 m of that is the arm's."""
    ctx = ctx_for(node)
    expected = (2.0 - TOOL_Z_COLUMN_DOWN) - ARM_Z_WINDOW_M
    assert state._column_target_for_line(ctx, 2.0) == pytest.approx(expected, abs=1e-6)


def test_the_column_comes_back_down_for_a_low_row(state, node):
    """Raised 0.4 m for a high row, then asked for a low one: the excess is
    negative, so the column must retract rather than hold."""
    ctx = ctx_for(node, arm_base_z=MOUNT_Z_COLUMN_DOWN + 0.4, column=0.4)
    target = state._column_target_for_line(ctx, 1.5)
    assert target < 0.4
    assert target == pytest.approx(0.4 + (1.5 - (TOOL_Z_COLUMN_DOWN + 0.4)) + ARM_Z_WINDOW_M,
                                   abs=1e-6)


def test_the_column_height_is_measured_not_assumed(state, node):
    """The mount height already contains the column's extension, so the target
    must be expressed against the CURRENT height, not against zero."""
    raised = ctx_for(node, arm_base_z=MOUNT_Z_COLUMN_DOWN + 0.3, column=0.3)
    down = ctx_for(node)
    # Same row, same geometry apart from the column: the tool ends up at the
    # same map height either way, so the commanded targets differ by the 0.3 m.
    assert (state._column_target_for_line(raised, 2.2)
            == pytest.approx(state._column_target_for_line(down, 2.2), abs=1e-6))


# ---------------------------------------------------------------- travel caps

def test_the_target_is_capped_to_the_columns_travel(state, node):
    ctx = ctx_for(node)
    assert state._column_target_for_line(ctx, 4.0) == pytest.approx(0.9)
    assert state._column_target_for_line(ctx, 0.2) == pytest.approx(0.0)


# ------------------------------------------------- missing inputs never guess

def test_a_missing_transform_waits_instead_of_guessing(state, node):
    ctx = ctx_for(node, arm_base_z=None)
    assert state._column_target_for_line(ctx, 1.5) is None


def test_an_unknown_column_height_waits_instead_of_reading_as_retracted(state, node):
    """ctx.get(..., 0.0) used to make 'no joint state yet' and 'fully down'
    the same answer. They are not: one of them moves the axis."""
    ctx = ctx_for(node, column=None)
    assert "column_current_height" not in ctx
    assert state._column_target_for_line(ctx, 1.5) is None


def test_a_missing_tf_buffer_waits(state, node):
    ctx = ctx_for(node)
    ctx["tf_buffer"] = None
    assert state._column_target_for_line(ctx, 1.5) is None


# ------------------------------------------------------------------ the knobs

def test_the_arm_window_is_tunable_from_ctx(state, node):
    """Shrink the window to nothing and every millimetre becomes column travel —
    the old behaviour, still reachable as a param for a bench comparison."""
    ctx = ctx_for(node, scan_wall_arm_z_window=0.0)
    assert state._column_target_for_line(ctx, 1.5) == pytest.approx(0.0)   # clamped
    ctx = ctx_for(node, scan_wall_arm_z_window=0.0)
    assert (state._column_target_for_line(ctx, 1.8)
            == pytest.approx(1.8 - TOOL_Z_COLUMN_DOWN, abs=1e-6))
