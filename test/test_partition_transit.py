"""Tests for the partition transit: split axes, and verify before sweeping.

The field failure this pins (real-robot run, 2026-09-02): the SECOND base
placement of a wall parked at the wrong distance from it -- sometimes much
further, sometimes much closer -- and the turret and chassis ended up at an
arbitrary relative angle.

The cause is the platform, not a logic slip. A sideways move of the turret point
is produced by ROTATING THE CHASSIS (theta_c_dot = y_chassis / d1, d1 = 0.167 m),
so with max_angular_base = 0.2 rad/s the base can strafe along a wall at only
0.033 m/s, while driving along the wall NORMAL runs at wheel speed (~0.2 m/s).
A partition transit runs along the wall with the turret facing it, so it is a
move down the slow axis with a 6x faster axis sitting perpendicular to it. The
old crawl drove one constant-magnitude vector at the goal and stopped on an
isotropic 0.15 m ball, position only: it therefore converged on the wall-normal
axis first and stopped wherever that had carried it. Nothing downstream checked.

Run with:

    python3 -m pytest test/test_partition_transit.py -v
"""

import math

import pytest

from task_planner_fsm.states.scan_wall import ScanWall


class _FakeLogger:
    def __init__(self):
        self.infos, self.warnings, self.errors = [], [], []

    def info(self, msg, **kw):
        self.infos.append(msg)

    def warn(self, msg, **kw):
        self.warnings.append(msg)

    def error(self, msg, **kw):
        self.errors.append(msg)


class _FakeTimer:
    def cancel(self):
        pass


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger

    def create_timer(self, period, callback):
        return _FakeTimer()  # the tests drive _sweep_crawl_tick directly

    def destroy_timer(self, timer):
        pass


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


# A wall running along +Y with free space on the +X side, so the exterior normal
# is +X and the along-wall tangent is +Y. Matches wall_axes' convention:
# tangent = (-ny, nx).
INWARD = (-1.0, 0.0)          # from free space INTO the wall
NORMAL = (1.0, 0.0)           # exterior normal, what wall_axes returns
TANGENT = (0.0, 1.0)


def make_ctx(**over):
    ctx = {
        "node": _FakeNode(),
        "_cmd_vel_pub": _FakePub(),
        "current_wall_index": 0,
        "wall_inward_normals": [INWARD],
    }
    ctx.update(over)
    return ctx


@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s._sweep_crawl_timer = None
    s._sweep_crawl_deadline = float("inf")
    s._crawl_started = 0.0
    s._crawl_distance = 0.0
    s._nav_status = None
    return s


def crawl(state, ctx, target, pose, speed=0.15, tol=0.15, tol_normal=0.05):
    """Arm an axis-split crawl and run one tick from a stubbed pose."""
    state._base_xy_yaw_map = lambda _ctx: pose
    state._start_sweep_crawl(
        ctx, target, yaw=pose[2], speed=speed, tol=tol, tol_normal=tol_normal,
        axis_split=True, what="Partition transit",
    )
    state._sweep_crawl_deadline = float("inf")   # never time out mid-test
    state._sweep_crawl_tick(ctx)
    return ctx["_cmd_vel_pub"].sent[-1] if ctx["_cmd_vel_pub"].sent else None


# ---------------------------------------------------------------------------
# The strafe limit is derived from the platform, not guessed
# ---------------------------------------------------------------------------

def test_strafe_limit_comes_from_the_chassis_rotation_limit(state):
    """d1 * max_angular_base. This is THE number: 0.167 * 0.2 = 0.033 m/s, against
    the 0.15 m/s the transit was asking for."""
    assert state._max_lateral_speed(make_ctx()) == pytest.approx(0.167 * 0.2)


def test_strafe_limit_follows_a_retuned_platform(state):
    ctx = make_ctx(base_center_distance_m=0.25, base_max_angular_base=0.4)
    assert state._max_lateral_speed(ctx) == pytest.approx(0.1)


def test_strafe_limit_can_be_overridden_wholesale(state):
    assert state._max_lateral_speed(
        make_ctx(crawl_max_lateral_speed=0.08)) == pytest.approx(0.08)


# ---------------------------------------------------------------------------
# The command is split by axis
# ---------------------------------------------------------------------------

def test_along_wall_component_is_capped_at_the_strafe_limit(state):
    """Asking for 0.15 m/s sideways does not go faster -- the controller scales
    the WHOLE command down to the feasible set, shrinking the standoff component
    along with it. Command what the base can do instead."""
    ctx = make_ctx()
    # 1 m to go along the wall, standoff already correct.
    cmd = crawl(state, ctx, target=(0.0, 1.0), pose=(0.0, 0.0, 0.0))
    # Base yaw 0 => body frame == map frame here.
    assert cmd.linear.y == pytest.approx(0.167 * 0.2, abs=1e-9)
    assert cmd.linear.x == pytest.approx(0.0, abs=1e-9)


def test_standoff_component_runs_at_the_full_commanded_speed(state):
    """The wall-normal axis is the fast one; there is no reason to slow it."""
    ctx = make_ctx()
    cmd = crawl(state, ctx, target=(1.0, 0.0), pose=(0.0, 0.0, 0.0))
    assert cmd.linear.x == pytest.approx(0.15)
    assert cmd.linear.y == pytest.approx(0.0, abs=1e-9)


def test_a_satisfied_axis_stops_contributing(state):
    """The end of the move is along whichever axis is still out, not a diagonal --
    so closing the fast axis cannot drag the base along the slow one."""
    ctx = make_ctx()
    # Standoff already inside 0.05; 1 m still to go along the wall.
    cmd = crawl(state, ctx, target=(0.02, 1.0), pose=(0.0, 0.0, 0.0))
    assert cmd.linear.x == pytest.approx(0.0, abs=1e-9)
    assert cmd.linear.y > 0.0


def test_arrival_needs_both_axes_within_their_own_tolerance(state):
    """The old isotropic test could be satisfied on the wall-normal axis alone.
    0.12 m of standoff error is inside a 0.15 m ball but way outside 0.05 m."""
    ctx = make_ctx()
    crawl(state, ctx, target=(0.12, 0.0), pose=(0.0, 0.0, 0.0))
    assert state._nav_status is None, "0.12 m off the standoff is not arrival"
    ctx = make_ctx()
    crawl(state, ctx, target=(0.04, 0.10), pose=(0.0, 0.0, 0.0))
    assert state._nav_status is not None, "both axes inside tolerance IS arrival"


def test_a_tick_never_overshoots_the_remaining_distance(state):
    ctx = make_ctx()
    # 6 cm of standoff left: at 0.15 m/s a 0.1 s tick would cover 15 mm... fine,
    # but at 1 mm left it must not command the full speed.
    cmd = crawl(state, ctx, target=(0.0511, 0.0), pose=(0.0, 0.0, 0.0))
    step = 1.0 / state.SWEEP_CRAWL_RATE_HZ
    assert cmd.linear.x <= 0.0511 / step + 1e-9


def test_without_a_wall_normal_it_falls_back_to_one_vector(state):
    """No normal means no axes to split; the move still runs and the arrival check
    downstream is what catches a bad standoff."""
    ctx = make_ctx(wall_inward_normals=[])
    cmd = crawl(state, ctx, target=(1.0, 1.0), pose=(0.0, 0.0, 0.0))
    assert state._crawl_axes is None
    assert cmd.linear.x == pytest.approx(cmd.linear.y)      # straight at the goal
    assert any("no wall normal" in w for w in ctx["node"].get_logger().warnings)


# ---------------------------------------------------------------------------
# Arrival is verified before the sweep, in the wall frame
# ---------------------------------------------------------------------------

def arrival(state, ctx, pose, target=(1.0, 0.0, 0.0)):
    state._segments = [None]
    state._seg_idx = 0
    state._scan_poses = [target]
    state._base_xy_yaw_map = lambda _ctx: pose
    return state._transit_arrival_ok(ctx, seg_no=1)


def test_a_good_pose_passes_through_to_the_sweep(state):
    ctx = make_ctx()
    assert arrival(state, ctx, pose=(1.02, 0.10, 0.0)) is True


def test_a_standoff_miss_inside_the_old_ball_is_now_caught(state):
    """The regression this exists for: 0.12 m of standoff error passed the old
    single 0.15 m position test and went straight to the press."""
    ctx = make_ctx()
    assert arrival(state, ctx, pose=(0.88, 0.0, 0.0)) is False
    assert state._seg_phase == "transit", "should re-run the transit"
    assert any("standoff off by" in w for w in ctx["node"].get_logger().warnings)


def test_a_heading_miss_is_caught_too(state):
    """A parked arm sweep is only symmetric about the base centreline if the base
    is square to the wall."""
    ctx = make_ctx()
    assert arrival(state, ctx, pose=(1.0, 0.0, 0.6)) is False
    assert any("heading off by" in w for w in ctx["node"].get_logger().warnings)


def test_the_along_wall_axis_keeps_its_loose_tolerance(state):
    """Precision along the wall costs real time at 0.033 m/s and buys nothing --
    the executor's lead-in re-centres on the partition."""
    ctx = make_ctx()
    assert arrival(state, ctx, pose=(1.0, 0.12, 0.0)) is True


def test_retries_are_bounded_and_then_the_partition_is_skipped(state):
    """Sweeping from a standoff the arm cannot span is worse than not sweeping."""
    ctx = make_ctx()
    for attempt in range(state.PARTITION_TRANSIT_MAX_RETRIES):
        assert arrival(state, ctx, pose=(0.5, 0.0, 0.0)) is False
        assert state._seg_phase == "transit", f"retry {attempt} should re-transit"
    assert arrival(state, ctx, pose=(0.5, 0.0, 0.0)) is False
    assert state._seg_phase == "transit_clear", "should move on to the next partition"
    assert state._seg_idx == 1
    assert any("skipping this partition" in e for e in ctx["node"].get_logger().errors)


def test_a_good_arrival_refunds_the_retry_budget(state):
    ctx = make_ctx()
    arrival(state, ctx, pose=(0.5, 0.0, 0.0))
    assert state._transit_retries == 1
    arrival(state, ctx, pose=(1.0, 0.0, 0.0))
    assert state._transit_retries == 0


def test_missing_inputs_do_not_block_the_scan(state):
    """No pose or no normal means the check cannot run. Sweeping on an unverified
    pose is the old behaviour, and it beats refusing to scan the wall at all."""
    ctx = make_ctx(wall_inward_normals=[])
    assert arrival(state, ctx, pose=(1.0, 0.0, 0.0)) is True
    assert any("Cannot verify" in w for w in ctx["node"].get_logger().warnings)
