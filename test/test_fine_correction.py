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

Plus the rotation profile, which is what stopped the turret overshooting the
wall-facing heading on the first base placement: ramp in, brake into the target,
then stop and re-measure at rest before calling it converged.

Run with:

    python3 -m pytest test/test_fine_correction.py -v
"""

import math

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


def settle(state, ctx, pos_err, yaw_err, pose=(0.0, 0.0, 0.0)):
    """Reach the servo target, then let the settle wait expire and tick again.

    Convergence is deliberately judged only after the base has come to rest, so
    every "did it finish?" assertion needs the second tick.
    """
    drive(state, ctx, pos_err, yaw_err, pose)
    state._servo_settle_until = 0.0        # pretend the coast has elapsed
    drive(state, ctx, pos_err, yaw_err, pose)


# ---------------------------------------------------------------------------
# Rotation-only: yaw is the only convergence criterion
# ---------------------------------------------------------------------------

def test_rotation_only_reports_drift_instead_of_accepting_a_bad_pose(state):
    state._servo_rotate_only = True
    ctx = make_ctx()
    settle(state, ctx, pos_err=0.55, yaw_err=0.01)     # yaw ok, position drifted
    assert state.navigation_done is False, "must not declare success on a drifted pose"
    assert state._nav_result_pending is True, "should hand back to the retry path"
    assert any("moved the base off the standoff" in w
               for w in ctx["node"].get_logger().warnings)


def test_rotation_only_declares_done_when_position_held(state):
    state._servo_rotate_only = True
    ctx = make_ctx()
    settle(state, ctx, pos_err=0.10, yaw_err=0.01)     # both within tolerance
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
    settle(state, ctx, pos_err=0.55, yaw_err=0.01)     # yaw ok, position not
    assert state.navigation_done is False


# ---------------------------------------------------------------------------
# Rotation profile: ramp in, brake into the target, verify at rest
#
# The field failure this pins: the turret ran at the clamp until the accept
# tolerance tripped, then the command stepped to zero and the base coasted past
# the wall-facing heading, settling tilted.
# ---------------------------------------------------------------------------

def test_rotation_ramps_up_instead_of_stepping_to_the_ceiling(state):
    """First tick out of rest is one acceleration step, not the full speed."""
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.10, yaw_err=1.5, pose=(0.0, 0.0, -1.5))
    first = ctx["_cmd_vel_pub"].sent[-1].angular.z
    step = state.SERVO_ANG_ACCEL / state.SERVO_RATE_HZ
    assert first == pytest.approx(step, rel=1e-6)
    assert first < state.SERVO_MAX_ANG


def test_rotation_brakes_into_the_target(state):
    """Close to the goal the command is capped by the stopping distance left, so
    it cannot be running at the ceiling when the tolerance trips."""
    state._servo_rotate_only = True
    state._servo_ang_cmd = state.SERVO_MAX_ANG    # already at full speed
    ctx = make_ctx()
    dyaw = 0.06                                   # just outside the servo target
    drive(state, ctx, pos_err=0.10, yaw_err=dyaw, pose=(0.0, 0.0, -dyaw))
    cmd = ctx["_cmd_vel_pub"].sent[-1].angular.z
    remaining = dyaw - state.SERVO_YAW_TOL_RAD
    assert cmd <= math.sqrt(2.0 * state.SERVO_ANG_ACCEL * remaining) + 1e-9
    assert cmd < state.SERVO_MAX_ANG


def test_rotation_never_exceeds_the_ceiling(state):
    state._servo_rotate_only = True
    ctx = make_ctx()
    for _ in range(200):                          # long way to go, plenty of ramp
        drive(state, ctx, pos_err=0.10, yaw_err=3.0, pose=(0.0, 0.0, -3.0))
    assert abs(ctx["_cmd_vel_pub"].sent[-1].angular.z) <= state.SERVO_MAX_ANG + 1e-9


def test_reaching_the_target_stops_and_waits_before_declaring_done(state):
    """The tick that reaches the target must command zero and NOT finish: the
    base is still turning, so its yaw error is not the one it will rest at."""
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.10, yaw_err=0.01)
    assert state.navigation_done is False, "must not finish while still coasting"
    assert state._servo_settle_until is not None
    assert ctx["_cmd_vel_pub"].sent[-1].angular.z == 0.0


def test_overshoot_seen_at_rest_is_corrected_not_accepted(state):
    """Settling outside the accept tolerance re-opens the correction instead of
    handing ScanWall a tilted base."""
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.10, yaw_err=0.01)        # reach target, stop
    state._servo_settle_until = 0.0                      # coast elapsed
    drive(state, ctx, pos_err=0.10, yaw_err=0.4,         # ...it overshot
          pose=(0.0, 0.0, -0.4))
    assert state.navigation_done is False
    assert any("settled" in w for w in ctx["node"].get_logger().warnings)
    assert ctx["_cmd_vel_pub"].sent[-1].angular.z != 0.0, "should be turning back"


def test_settling_inside_the_accept_tolerance_is_good_enough(state):
    """The servo AIMS tighter than the gate, but it must not keep hunting for
    that tighter target once the base is at rest and inside the gate."""
    state._servo_rotate_only = True
    ctx = make_ctx()
    drive(state, ctx, pos_err=0.10, yaw_err=0.01)
    state._servo_settle_until = 0.0
    # Between the servo target (0.04) and the accept tolerance (0.25).
    drive(state, ctx, pos_err=0.10, yaw_err=0.10, pose=(0.0, 0.0, -0.10))
    assert state.navigation_done is True


def test_servo_target_is_never_looser_than_the_accept_tolerance(state):
    """A deployment that tightens nav_yaw_tolerance must tighten the servo with
    it, or the servo would stop short of the gate and time out."""
    state._servo_rotate_only = True
    ctx = make_ctx()
    ctx["nav_yaw_tolerance"] = 0.01               # tighter than SERVO_YAW_TOL_RAD
    drive(state, ctx, pos_err=0.10, yaw_err=0.02, pose=(0.0, 0.0, -0.02))
    assert state._servo_settle_until is None, "0.02 rad is not converged at a 0.01 gate"
    assert ctx["_cmd_vel_pub"].sent[-1].angular.z != 0.0


# ---------------------------------------------------------------------------
# Mode is set explicitly on every start, never inherited
# ---------------------------------------------------------------------------

def test_mode_defaults_to_off_on_a_fresh_state():
    assert NavigateToTarget("N")._servo_rotate_only is False
