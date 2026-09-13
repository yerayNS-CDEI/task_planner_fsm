"""The arm command path: integration, stop semantics, and the wind-up clamp.

The behaviour under test is mostly about what happens when things go WRONG —
the loop stops, the robot stops following, a joint reaches its limit — because
that is the whole reason the position interface was chosen over the velocity
one. A stub node stands in for rclpy so these stay fast and independent of a
running graph.
"""

import numpy as np
import pytest

pytest.importorskip("std_msgs")

from task_planner_fsm.wbc.streaming import (  # noqa: E402
    POSITION,
    VELOCITY,
    ArmStream,
    slew_limit,
)

JOINTS = ["j1", "j2", "j3"]


class _Recorder:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))

    @property
    def last(self):
        return self.sent[-1] if self.sent else None


class _StubNode:
    """Just enough Node for ArmStream: one publisher and a logger."""

    def __init__(self):
        self.publisher = _Recorder()
        self.logs = []

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher

    def get_logger(self):
        return self

    def error(self, message, **_kwargs):
        self.logs.append(message)

    warn = info = error


def _stream(mode=POSITION, **kwargs):
    node = _StubNode()
    return node, ArmStream(node, JOINTS, mode=mode, **kwargs)


# ----------------------------------------------------------------------
# Stop semantics — the reason position streaming is the default
# ----------------------------------------------------------------------
def test_a_position_stop_holds_the_pose_instead_of_running_to_zero():
    """The single most dangerous thing this module can get wrong.

    Zero is a valid stop command on a velocity interface and a full-speed run
    to the zero configuration on a position one. Any code path that stops the
    arm has to go through hold(), and hold() has to know the difference.
    """
    node, stream = _stream(POSITION)
    q = np.array([0.4, -1.2, 2.0])
    stream.reset(q)

    stream.hold(q)

    assert node.publisher.last == pytest.approx(list(q))
    assert node.publisher.last != [0.0, 0.0, 0.0]


def test_a_velocity_stop_is_a_zero():
    node, stream = _stream(VELOCITY)
    stream.send(np.array([0.1, 0.2, 0.3]), 0.02)

    stream.hold(np.array([0.4, -1.2, 2.0]))

    assert node.publisher.last == [0.0, 0.0, 0.0]


def test_hold_reseeds_at_the_measurement_so_a_resumed_sweep_starts_from_the_truth():
    """A setpoint left ahead of a stopped arm is a snap waiting to happen.

    If the reason for holding is that the arm was not following (protective
    stop, speed scaling at zero), re-publishing the last SETPOINT keeps the
    accumulated gap alive: the moment the stop clears, the arm jumps through
    it. Re-seeding at the measurement means "stop here".
    """
    _, stream = _stream(POSITION)
    stream.reset(np.zeros(3))
    for _ in range(20):
        stream.send(np.array([0.5, 0.0, 0.0]), 0.02, q_measured=np.zeros(3))

    # The arm never moved; the setpoint led it. Holding must abandon that lead.
    assert stream.lead(np.zeros(3)) > 0.0
    stream.hold(np.zeros(3))
    assert stream.lead(np.zeros(3)) == 0.0


def test_an_unseeded_position_stream_holds_rather_than_guessing_a_pose():
    node, stream = _stream(POSITION)

    stream.send(np.array([0.1, 0.1, 0.1]), 0.02, q_measured=np.array([1.0, 1.0, 1.0]))

    assert node.publisher.last == pytest.approx([1.0, 1.0, 1.0])
    assert any("never seeded" in line for line in node.logs)


def test_the_first_command_after_the_switch_is_the_arm_s_current_pose():
    """Closes the jump between the controller activating and the first cycle."""
    node, stream = _stream(POSITION)
    q = np.array([0.1, -0.9, 1.7])

    stream.initial_command(q)

    assert node.publisher.last == pytest.approx(list(q))


# ----------------------------------------------------------------------
# Integration
# ----------------------------------------------------------------------
def test_the_setpoint_integrates_the_commanded_velocity():
    _, stream = _stream(POSITION)
    stream.reset(np.zeros(3))
    qdot = np.array([0.5, -0.25, 0.0])

    for _ in range(10):
        # The arm follows perfectly, so nothing clamps.
        stream.send(qdot, 0.02, q_measured=stream.command)

    assert stream.command == pytest.approx(qdot * 0.2)


def test_velocity_mode_passes_the_command_straight_through():
    node, stream = _stream(VELOCITY)
    qdot = [0.5, -0.25, 0.1]

    stream.send(np.array(qdot), 0.02, q_measured=np.zeros(3))

    assert node.publisher.last == pytest.approx(qdot)


def test_the_setpoint_cannot_run_away_from_an_arm_that_is_not_moving():
    """Wind-up: the integrator advances whether or not the robot follows.

    A protective stop or a speed slider at zero freezes the arm while the loop
    keeps commanding. Without the clamp the setpoint banks the whole stopped
    interval and the arm snaps through it when the stop clears.
    """
    _, stream = _stream(POSITION, max_lead=0.1)
    stream.reset(np.zeros(3))
    frozen = np.zeros(3)

    for _ in range(500):                      # 10 s of commanding a stopped arm
        stream.send(np.array([1.0, 0.0, 0.0]), 0.02, q_measured=frozen)

    assert stream.command[0] == pytest.approx(0.1)
    assert stream.lead(frozen) == pytest.approx(0.1)


def test_a_joint_the_arm_does_not_follow_pins_the_setpoint_at_the_clamp():
    """Wind-up is bounded even when the arm is not tracking at all.

    The integrator advances whether or not the robot follows, so a joint being
    moved by something else — or not moving when it should — would let the
    setpoint run away and snap the arm through the gap once the cause cleared.
    The lead clamp is what makes that impossible.
    """
    _, stream = _stream(POSITION, max_lead=0.1)
    stream.reset(np.zeros(3))

    dragged = np.zeros(3)
    for step in range(50):
        dragged[0] = 0.5 * step
        stream.send(np.array([0.0, 1.0, 0.0]), 0.02, q_measured=dragged)

    assert stream.lead(dragged) == pytest.approx(0.1, abs=1e-9)


def test_the_setpoint_is_clamped_to_the_joint_limits():
    _, stream = _stream(POSITION)
    stream.set_position_limits([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0])
    stream.reset(np.array([0.9, 0.0, 0.0]))

    for _ in range(50):
        stream.send(np.array([1.0, 0.0, 0.0]), 0.02, q_measured=stream.command)

    assert stream.command[0] <= 1.0 + 1e-9


def test_lead_is_only_meaningful_in_position_mode():
    _, stream = _stream(VELOCITY)
    stream.send(np.array([1.0, 0.0, 0.0]), 0.02)
    assert stream.lead(np.zeros(3)) == 0.0


def test_an_unknown_mode_is_refused_at_construction():
    node = _StubNode()
    with pytest.raises(ValueError):
        ArmStream(node, JOINTS, mode="effort")


# ----------------------------------------------------------------------
# The acceleration bound
# ----------------------------------------------------------------------
def test_the_acceleration_bound_limits_a_step():
    u = slew_limit(np.array([1.0, -1.0]), np.array([0.0, 0.0]), 2.0, 0.02)
    assert u == pytest.approx([0.04, -0.04])


def test_the_bound_is_per_dof():
    u = slew_limit(np.array([1.0, 1.0]), np.array([0.0, 0.0]), [2.0, 50.0], 0.02)
    assert u == pytest.approx([0.04, 1.0])


def test_a_command_within_the_bound_passes_through_unchanged():
    u = slew_limit(np.array([0.01, -0.01]), np.array([0.0, 0.0]), 2.0, 0.02)
    assert u == pytest.approx([0.01, -0.01])


def test_the_first_command_after_a_stop_is_not_limited_against_a_stale_one():
    u = slew_limit(np.array([1.0, -1.0]), None, 2.0, 0.02)
    assert u == pytest.approx([1.0, -1.0])
