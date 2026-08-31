"""Unit tests for NavigateToTarget's in-place heading correction.

Why this exists: at the scan standoff DWB cannot plan at all (footprint 0.7 m +
inscribed 0.6 m => it needs ~1.3 m of wall clearance, and the arm needs the base
closer than that). So the final heading correction is servoed over /cmd_vel
instead. See ARM_SWEEP_PLAN §9.1 "MEASURED CONSTRAINT".

The two failure modes worth pinning:
  * requiring the POSITION tolerance to converge in rotation-only mode would
    spin forever, because turret_footprint sits ~0.23 m off the chassis rotation
    centre and a pure rotation walks it around a small circle;
  * silently accepting that drift would hand ScanWall a bad standoff pose.

Run with:

    python3 -m pytest test/test_fine_correction.py -v
"""

import pytest

from task_planner_fsm.states.navigate import NavigateToTarget


class _FakeLogger:
    def __init__(self):
        self.infos, self.warnings, self.errors = [], [], []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger

    def destroy_timer(self, timer):
        pass


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


@pytest.fixture
def state():
    s = NavigateToTarget("NavigateToTarget")
    s._servo_start = 1e18          # never time out during a test
    s._servo_timer = None
    s._goal_xy = (0.0, 0.0)
    s._goal_yaw = 0.0
    return s


def make_ctx():
    return {"node": _FakeNode(), "_cmd_vel_pub": _FakePub()}


def drive(state, ctx, pos_err, yaw_err, pose=(0.0, 0.0, 0.0)):
    """Run one servo tick with stubbed pose feedback."""
    state._pose_error = lambda _ctx: (pos_err, yaw_err)
    state._base_pose_map = lambda _ctx: pose
    state._fine_correction_tick(ctx)


# ---------------------------------------------------------------------------
# Rotation-only: yaw is the only convergence criterion
# ---------------------------------------------------------------------------

def test_rotation_only_reports_drift_instead_of_accepting_a_bad_pose(state):
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.55, yaw_err=0.01)      # yaw ok, position drifted
    assert state.navigation_done is False, "must not declare success on a drifted pose"
    assert state._nav_result_pending is True, "should hand back to the retry path"
    assert any("moved the base off the standoff" in w
               for w in ctx["node"].get_logger().warnings)


def test_rotation_only_declares_done_when_position_held(state):
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.10, yaw_err=0.01)      # both within tolerance
    assert state.navigation_done is True
    assert state._nav_result_pending is False


def test_rotation_only_keeps_turning_while_yaw_is_off(state):
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.10, yaw_err=0.9)
    assert state.navigation_done is False
    assert ctx["_cmd_vel_pub"].sent, "should have commanded a twist"


# ---------------------------------------------------------------------------
# Rotation-only commands NO translation
# ---------------------------------------------------------------------------

def test_rotation_only_never_translates(state):
    """The safety argument only covers rotation: a circular footprint turning
    about its centre sweeps no new area. Translation would not be safe here."""
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.20, yaw_err=0.9, pose=(0.4, -0.3, 0.2))
    cmd = ctx["_cmd_vel_pub"].sent[-1]
    assert cmd.linear.x == 0.0
    assert cmd.linear.y == 0.0
    assert cmd.angular.z != 0.0


def test_full_servo_still_translates(state):
    """The legacy full-pose servo is unchanged."""
    state._servo_rotate_only = False
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.5, yaw_err=0.9, pose=(0.4, -0.3, 0.2))
    cmd = ctx["_cmd_vel_pub"].sent[-1]
    assert cmd.linear.x != 0.0 or cmd.linear.y != 0.0


def test_full_servo_requires_both_tolerances(state):
    state._servo_rotate_only = False
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.55, yaw_err=0.01)      # yaw ok, position not
    assert state.navigation_done is False


# ---------------------------------------------------------------------------
# Mode is set explicitly on every start, never inherited
# ---------------------------------------------------------------------------

def test_mode_defaults_to_off_on_a_fresh_state():
    assert NavigateToTarget("N")._servo_rotate_only is False
