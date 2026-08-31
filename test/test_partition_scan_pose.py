"""Unit tests for the arm-sweep partition scan pose (ARM_SWEEP_PLAN §7.A).

The yaw convention is the part of S2 most likely to be silently wrong -- it is a
90 degree rotation away from the along-wall heading the base-driven sweep uses,
and a sign slip would face the robot away from the wall. These tests pin it down
with a fake ctx: no costmap (so the standability search is skipped) and
nav_base_frame left at its `turret_footprint` default (so the turret offset is
zero without needing TF).

Run with:

    python3 -m pytest test/test_partition_scan_pose.py -v
"""

import math

import pytest

from task_planner_fsm.utils.costmap_utils import partition_scan_pose

STANDOFF = 1.0


class _FakeLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def info(self, msg):
        pass

    def warn(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


def make_ctx(inward_normal, standoff=STANDOFF, **extra):
    """ctx with just enough for partition_scan_pose.

    ``inward_normal`` follows the codebase convention (see build_wall_data): it
    points from the robot's free space INTO the wall face, i.e. it is the
    detector's outward-from-the-building arrow.
    """
    ctx = {
        "node": _FakeNode(),
        "current_wall_index": 0,
        "wall_inward_normals": [inward_normal],
        "partition_base_standoff_m": standoff,
        # No "global_costmap" key: skips the standability search entirely.
    }
    ctx.update(extra)
    return ctx


def yaw_to_unit(yaw):
    return (math.cos(yaw), math.sin(yaw))


# ---------------------------------------------------------------------------
# Position: backed off from the partition centre, on the robot's side
# ---------------------------------------------------------------------------

def test_pose_is_centred_on_the_partition():
    """A wall along +X at y=0 with free space at y>0."""
    ctx = make_ctx((0.0, -1.0, 0.0))
    x, y, _ = partition_scan_pose(ctx, "T", (2.0, 0.0, 1.0), (3.0, 0.0, 1.0))
    # Centred laterally: the arm reaches +/- half the partition from here.
    assert x == pytest.approx(2.5)
    assert y == pytest.approx(STANDOFF)


def test_pose_backs_off_on_the_free_space_side_not_through_the_wall():
    ctx = make_ctx((0.0, -1.0, 0.0))
    _, y, _ = partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert y > 0.0, "scan pose landed behind the wall"


def test_standoff_distance_is_honoured():
    for standoff in (0.6, 1.0, 1.4):
        ctx = make_ctx((0.0, -1.0, 0.0), standoff=standoff)
        _, y, _ = partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        assert y == pytest.approx(standoff)


def test_standoff_is_measured_along_the_normal_for_a_diagonal_wall():
    n = (1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0)   # into the wall
    ctx = make_ctx(n)
    p_start, p_end = (0.0, 0.0, 0.0), (2.0, 2.0, 0.0)        # wall along the diagonal
    x, y, _ = partition_scan_pose(ctx, "T", p_start, p_end)
    centre = (1.0, 1.0)
    assert math.hypot(x - centre[0], y - centre[1]) == pytest.approx(STANDOFF)


# ---------------------------------------------------------------------------
# Orientation: turret +X into the wall (NOT the along-wall sweep heading)
# ---------------------------------------------------------------------------

def test_yaw_points_into_the_wall():
    ctx = make_ctx((0.0, -1.0, 0.0))        # wall-ward is -Y
    _, _, yaw = partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert yaw_to_unit(yaw) == pytest.approx((0.0, -1.0), abs=1e-9)


def test_yaw_flips_with_the_wall_side():
    """Same wall, robot on the other side -> heading must flip by 180 degrees."""
    _, _, near = partition_scan_pose(
        make_ctx((0.0, -1.0, 0.0)), "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    _, _, far = partition_scan_pose(
        make_ctx((0.0, 1.0, 0.0)), "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert abs(math.atan2(math.sin(near - far), math.cos(near - far))) == pytest.approx(math.pi)


def test_yaw_is_perpendicular_to_the_sweep_direction():
    """The regression that matters: this is NOT the along-wall heading that
    NavigateToTarget sends for the base-driven sweep."""
    ctx = make_ctx((0.0, -1.0, 0.0))
    p_start, p_end = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    _, _, yaw = partition_scan_pose(ctx, "T", p_start, p_end)
    heading = yaw_to_unit(yaw)
    tangent = (1.0, 0.0)
    assert heading[0] * tangent[0] + heading[1] * tangent[1] == pytest.approx(0.0, abs=1e-9)


def test_yaw_is_independent_of_endpoint_order():
    """Serpentine reverses partition endpoints; the base must still face the
    wall the same way."""
    ctx_a = make_ctx((0.0, -1.0, 0.0))
    ctx_b = make_ctx((0.0, -1.0, 0.0))
    _, _, forward = partition_scan_pose(ctx_a, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    _, _, reverse = partition_scan_pose(ctx_b, "T", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert forward == pytest.approx(reverse)


def test_yaw_is_wrapped_to_pi():
    for n in [(1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)]:
        ctx = make_ctx(n)
        _, _, yaw = partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        assert -math.pi <= yaw <= math.pi


def test_turret_offset_is_subtracted_on_the_diff_config():
    """When Nav2 steers the chassis instead of the turret, the commanded yaw must
    be reduced by the turret angle so turret_link still ends up facing the wall."""
    class _FakeTf:
        def lookup_transform(self, target, source, *_args, **_kwargs):
            class _Q:
                x = y = 0.0
                z = math.sin(math.pi / 4.0)      # turret at +90 deg
                w = math.cos(math.pi / 4.0)

            class _T:
                rotation = _Q()

            class _TF:
                transform = _T()

            return _TF()

    ctx = make_ctx((0.0, -1.0, 0.0), nav_base_frame="base_footprint", tf_buffer=_FakeTf())
    _, _, yaw = partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    # Wall-ward heading is -pi/2; with the turret already at +pi/2 the chassis
    # must be commanded to -pi so the turret lands on -pi/2.
    assert yaw == pytest.approx(-math.pi)


# ---------------------------------------------------------------------------
# Failure handling
# ---------------------------------------------------------------------------

def test_missing_wall_normal_returns_none():
    """No sane fallback here: a partition swept from a guessed side is worse than
    one not swept at all."""
    ctx = make_ctx(None)
    ctx["wall_inward_normals"] = []
    assert partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)) is None
    assert ctx["node"].get_logger().errors


def test_degenerate_partition_returns_none():
    ctx = make_ctx((0.0, -1.0, 0.0))
    assert partition_scan_pose(ctx, "T", (1.0, 1.0, 0.0), (1.0, 1.0, 0.0)) is None


def test_skewed_normal_warns_but_still_returns_a_pose():
    """Tangent not perpendicular to the normal means the partition and the normal
    probably describe different walls -- warn loudly, do not silently proceed."""
    ctx = make_ctx((1.0, 0.0, 0.0))                      # normal along +X
    pose = partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))  # tangent +X too
    assert pose is not None
    assert any("perpendicular" in w for w in ctx["node"].get_logger().warnings)


def test_perpendicular_normal_does_not_warn():
    ctx = make_ctx((0.0, -1.0, 0.0))
    partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert not ctx["node"].get_logger().warnings


# ---------------------------------------------------------------------------
# wall_facing_yaw: shared by NavigateToTarget and ScanWall, so pin it directly
# ---------------------------------------------------------------------------

def test_wall_facing_yaw_matches_the_partition_scan_pose_yaw():
    """Both states must approach a wall at the same heading, or the first
    partition transit will not be skippable as a no-op."""
    from task_planner_fsm.utils.costmap_utils import wall_facing_yaw
    ctx = make_ctx((0.0, -1.0, 0.0))
    _, _, pose_yaw = partition_scan_pose(ctx, "T", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert wall_facing_yaw(make_ctx((0.0, -1.0, 0.0)), "T") == pytest.approx(pose_yaw)


def test_wall_facing_yaw_points_along_the_inward_normal():
    from task_planner_fsm.utils.costmap_utils import wall_facing_yaw
    for n in [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)]:
        yaw = wall_facing_yaw(make_ctx(n), "T")
        # inward_normal already points INTO the wall, so the heading IS it.
        assert yaw_to_unit(yaw) == pytest.approx((n[0], n[1]), abs=1e-9)


def test_wall_facing_yaw_none_without_a_normal():
    from task_planner_fsm.utils.costmap_utils import wall_facing_yaw
    ctx = make_ctx(None)
    ctx["wall_inward_normals"] = []
    assert wall_facing_yaw(ctx, "T") is None
