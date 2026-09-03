"""Unit tests for the shared partition plan (ARM_SWEEP_PLAN §7.A, §3.6).

The invariant under test is an agreement between two states that never run at
the same time: NavigateToTarget picks its approach pose from partition 1, and
ScanWall then skips partition 1's transit as a no-op. If the two ever computed
different partitions, the base would drive to one end of the wall and ScanWall
would immediately transit to the other -- with the arm unfolded, at a standoff
where Nav2 barely manoeuvres. So what is pinned here is that both get the SAME
plan: same line ordering, same split, same poses.

No ROS, no simulator: a fake ctx with no "global_costmap" key skips the
standability search, and the default `turret_footprint` nav base frame makes the
turret offset zero without needing TF.

Run with:

    python3 -m pytest test/test_partition_plan.py -v
"""

import math

import pytest

from task_planner_fsm.utils.costmap_utils import (
    DEFAULT_SCAN_LINE_OFFSET,
    plan_wall_partitions,
)
from task_planner_fsm.utils.wall_approach import (
    should_approach_partition,
    should_face_wall,
)
from task_planner_fsm.utils.wall_partitioning import sweep_line_order


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warn(self, msg):
        self.messages.append(("warn", msg))

    def error(self, msg):
        self.messages.append(("error", msg))


class _FakeClock:
    """Enough of a clock for the marker publishers -- message field validation
    rejects anything but a real builtin_interfaces Time."""

    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _FakeClock()

    def create_publisher(self, *args, **kwargs):
        return _FakePublisher()


class _FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


# A wall running along +X at y=0, with free space on the -Y side. The codebase
# convention (see build_wall_data) has `inward_normal` pointing from that free
# space INTO the wall face, i.e. +Y here.
WALL = [(0.0, 0.0, 1.0), (4.0, 0.0, 1.0)]
INWARD_NORMAL = (0.0, 1.0)
# From the WALL FACE: the scan line the partitions lie on is
# DEFAULT_SCAN_LINE_OFFSET closer to the wall, so the pose sits PUSH off it.
STANDOFF = 1.6
PUSH = STANDOFF - DEFAULT_SCAN_LINE_OFFSET


def make_ctx(**extra):
    ctx = {
        "node": _FakeNode(),
        "current_wall_index": 0,
        "wall_inward_normals": [INWARD_NORMAL],
        "partition_base_standoff_m": STANDOFF,
        # No "global_costmap": reachable_wall_segments returns None, so callers
        # must supply `segments` explicitly. That is deliberate -- it keeps these
        # tests on the partition geometry rather than on costmap rasterisation.
    }
    ctx.update(extra)
    return ctx


WHOLE_WALL = [(tuple(WALL[0]), tuple(WALL[1]))]


def free_costmap():
    """An all-free 20x20 m grid, so reachable_wall_segments keeps the whole line.

    Used only by the caching tests, which need the real (non-degraded) code path;
    the geometry tests supply `segments` instead and stay off the rasteriser.
    """
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid()
    grid.info.resolution = 0.1
    grid.info.width = grid.info.height = 200
    grid.info.origin.position.x = -10.0
    grid.info.origin.position.y = -10.0
    grid.data = [0] * (grid.info.width * grid.info.height)
    return grid


# ---------------------------------------------------------------------------
# sweep_line_order -- the ordering both states must share
# ---------------------------------------------------------------------------

def test_line_order_puts_the_nearer_end_first():
    near, far = sweep_line_order(WALL, (0.3, -1.0, 1.0))
    assert near == WALL[0] and far == WALL[1]


def test_line_order_reverses_for_a_target_at_the_far_end():
    near, far = sweep_line_order(WALL, (3.7, -1.0, 1.0))
    assert near == WALL[1] and far == WALL[0]


def test_line_order_breaks_an_exact_tie_toward_the_first_endpoint():
    """Ties must resolve deterministically: both states run it independently and
    an arbitrary tie-break would let them disagree."""
    midpoint = (2.0, -1.0, 1.0)
    assert sweep_line_order(WALL, midpoint) == (WALL[0], WALL[1])
    assert sweep_line_order(WALL, midpoint) == (WALL[0], WALL[1])


# ---------------------------------------------------------------------------
# plan_wall_partitions
# ---------------------------------------------------------------------------

def test_returns_none_while_the_costmap_is_missing():
    """The caller's cue to keep polling rather than to give up on the wall."""
    assert plan_wall_partitions(make_ctx(), "T", WALL[0], WALL[1]) is None


def test_partitions_cover_the_line_and_each_carries_a_scan_pose():
    partitions, poses = plan_wall_partitions(
        make_ctx(), "T", WALL[0], WALL[1], segments=WHOLE_WALL
    )
    assert len(partitions) == len(poses) > 1
    assert partitions[0][0] == pytest.approx(WALL[0])
    assert partitions[-1][1] == pytest.approx(WALL[1])


def test_scan_pose_sits_a_standoff_out_from_the_partition_centre():
    partitions, poses = plan_wall_partitions(
        make_ctx(), "T", WALL[0], WALL[1], segments=WHOLE_WALL
    )
    for (pa, pb), (gx, gy, _) in zip(partitions, poses):
        cx, cy = 0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1])
        # Free space is -Y, so the base backs off to negative y.
        assert gx == pytest.approx(cx)
        assert gy == pytest.approx(cy - PUSH)


def test_scan_pose_yaw_points_into_the_wall():
    _, poses = plan_wall_partitions(
        make_ctx(), "T", WALL[0], WALL[1], segments=WHOLE_WALL
    )
    # Wall normal is +Y, so the base faces +Y: yaw = +pi/2.
    for _, _, yaw in poses:
        assert yaw == pytest.approx(math.pi / 2.0)


def test_the_two_states_get_the_same_partition_one():
    """The whole point: NavigateToTarget's approach pose is ScanWall's first
    transit, so ScanWall skips it."""
    ctx = make_ctx()
    near, far = sweep_line_order(WALL, (0.3, -1.0, 1.0))

    nav_partitions, nav_poses = plan_wall_partitions(ctx, "Nav", near, far, segments=WHOLE_WALL)
    scan_partitions, scan_poses = plan_wall_partitions(ctx, "Scan", near, far, segments=WHOLE_WALL)

    assert nav_partitions[0] == scan_partitions[0]
    assert nav_poses[0] == scan_poses[0]


def test_a_costmap_backed_plan_is_cached_and_reused():
    """Recomputing per state and per line height invites the two to disagree by
    one costmap update -- and breaks §3.6's identical-boundaries guarantee."""
    ctx = make_ctx(global_costmap=free_costmap())
    partitions, poses = plan_wall_partitions(ctx, "Nav", WALL[0], WALL[1])
    assert partitions and ctx.get("_partition_plan") is not None

    # Poison the source data: a second call that recomputed would now differ.
    ctx["wall_inward_normals"] = [(0.0, -1.0)]
    again = plan_wall_partitions(ctx, "Scan", WALL[0], WALL[1])
    assert again == (partitions, poses)


def test_a_degraded_plan_is_not_cached():
    """A plan built without the costmap skips obstacles it should have avoided.
    Caching it would hand that worse plan to every later line height."""
    ctx = make_ctx()
    partitions, _ = plan_wall_partitions(
        ctx, "T", WALL[0], WALL[1], segments=WHOLE_WALL
    )
    assert partitions
    assert ctx.get("_partition_plan") is None


def test_retuning_a_knob_invalidates_the_cache():
    ctx = make_ctx(global_costmap=free_costmap())
    coarse, _ = plan_wall_partitions(ctx, "T", WALL[0], WALL[1])
    ctx["partition_max_length_m"] = 0.4
    fine, _ = plan_wall_partitions(ctx, "T", WALL[0], WALL[1])
    assert len(fine) > len(coarse)


def test_a_new_wall_invalidates_the_cache():
    """The wall index selects the normal, which decides which SIDE the standoff
    is on -- reusing a stale plan would park the base inside the building."""
    ctx = make_ctx()
    ctx["_partition_plan"] = (("stale",), [("a", "b")], [(9.0, 9.0, 0.0)])
    _, poses = plan_wall_partitions(ctx, "T", WALL[0], WALL[1], segments=WHOLE_WALL)
    assert poses[0] != (9.0, 9.0, 0.0)


def test_no_wall_normal_yields_no_plan_rather_than_a_guess():
    """Sweeping from a guessed side of the wall is worse than not sweeping."""
    ctx = make_ctx(wall_inward_normals=[None])
    partitions, poses = plan_wall_partitions(ctx, "T", WALL[0], WALL[1], segments=WHOLE_WALL)
    assert partitions == [] and poses == []


def test_reversing_the_line_mirrors_the_same_partition_boundaries():
    """Serpentine reverses the sweep direction every line height. The boundaries
    must land in the same places or the GPR lines stop stitching (§3.6)."""
    ctx_fwd, ctx_rev = make_ctx(), make_ctx()
    fwd, _ = plan_wall_partitions(ctx_fwd, "T", WALL[0], WALL[1], segments=WHOLE_WALL)
    rev, _ = plan_wall_partitions(
        ctx_rev, "T", WALL[1], WALL[0],
        segments=[(tuple(WALL[1]), tuple(WALL[0]))],
    )
    assert len(fwd) == len(rev)
    for forward, reverse in zip(fwd, reversed(rev)):
        # Reversed partitions run end-to-start, so compare against the swap.
        assert forward[0][0] == pytest.approx(reverse[1][0])
        assert forward[1][0] == pytest.approx(reverse[0][0])


# ---------------------------------------------------------------------------
# should_approach_partition -- when NavigateToTarget uses the plan at all
# ---------------------------------------------------------------------------

def test_approach_needs_both_halves_of_the_arm_sweep():
    assert should_approach_partition({}) is True
    assert should_approach_partition({"sweep_use_arm": False}) is False
    # Facing off means the base would arrive at the partition centre on the
    # along-wall heading, which ScanWall's skip test would reject anyway.
    assert should_approach_partition({"nav_face_wall": False}) is False


def test_approach_can_be_forced_either_way():
    assert should_approach_partition({"nav_approach_partition": False}) is False
    assert should_approach_partition(
        {"nav_approach_partition": True, "sweep_use_arm": False}
    ) is True


def test_approach_never_outruns_the_heading_policy():
    """Guard on the pairing itself: approaching a partition centre only makes
    sense on the wall-facing heading."""
    for ctx in ({}, {"sweep_use_arm": True}, {"sweep_use_arm": False},
                {"nav_face_wall": False, "sweep_use_arm": True}):
        if should_approach_partition(ctx):
            assert should_face_wall(ctx)
