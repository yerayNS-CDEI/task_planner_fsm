"""Speed scaling and the safety gate: live on the UR, inert in simulation.

The default answers matter more than the live ones here. These topics do not
exist in Gazebo, and a monitor that reported "stopped" or "scaled to zero" in
their absence would stop a simulated robot that has no such concept — so the
no-hardware case is tested first and hardest.
"""

import pytest

pytest.importorskip("std_msgs")
from std_msgs.msg import Float64  # noqa: E402

from task_planner_fsm.wbc.hardware import HardwareMonitor  # noqa: E402

ur_msgs = pytest.importorskip("ur_dashboard_msgs.msg",
                              reason="UR driver not installed; the gate is inert without it")
RobotMode, SafetyMode = ur_msgs.RobotMode, ur_msgs.SafetyMode


class _StubNode:
    """Captures subscriptions by topic so tests can feed them messages."""

    def __init__(self):
        self.callbacks = {}
        self.logs = []

    def create_subscription(self, _msg_type, topic, callback, _qos):
        self.callbacks[topic] = callback
        return object()

    def get_logger(self):
        return self

    def info(self, message, **_kwargs):
        self.logs.append(message)

    warn = error = info


def _monitor():
    node = _StubNode()
    return node, HardwareMonitor(node)


def _send(node, topic, msg):
    node.callbacks[topic](msg)


SCALING = "/speed_scaling_state_broadcaster/speed_scaling"
SAFETY = "/io_and_status_controller/safety_mode"
ROBOT = "/io_and_status_controller/robot_mode"


# ----------------------------------------------------------------------
# Simulation: nothing publishes any of this
# ----------------------------------------------------------------------
def test_a_robot_that_says_nothing_is_running_at_full_speed_and_is_not_blocked():
    _, monitor = _monitor()

    assert monitor.seen() is False
    assert monitor.scaling() == 1.0
    assert monitor.blocked() is None
    assert monitor.describe() == "no hardware"


# ----------------------------------------------------------------------
# Speed scaling
# ----------------------------------------------------------------------
def test_the_reported_factor_is_used():
    node, monitor = _monitor()
    _send(node, SCALING, Float64(data=0.5))

    assert monitor.scaling() == pytest.approx(0.5)
    assert monitor.seen() is True


def test_a_factor_outside_zero_to_one_is_clamped_rather_than_trusted():
    """A factor above 1 would AMPLIFY the command, which is never intended."""
    node, monitor = _monitor()

    _send(node, SCALING, Float64(data=1.7))
    assert monitor.scaling() == 1.0

    _send(node, SCALING, Float64(data=-0.2))
    assert monitor.scaling() == 0.0


# ----------------------------------------------------------------------
# The safety gate
# ----------------------------------------------------------------------
def test_a_protective_stop_blocks_the_loop():
    """The case the gate exists for.

    ros2_control keeps running through a protective stop: the QP still solves
    and commands still publish, but the arm does not move. Without this the
    BASE would carry on sweeping along the wall with a frozen arm.
    """
    node, monitor = _monitor()
    _send(node, SAFETY, SafetyMode(mode=SafetyMode.PROTECTIVE_STOP))

    assert "PROTECTIVE_STOP" in monitor.blocked()


def test_normal_and_reduced_both_run():
    """REDUCED means limited, not stopped — and the limit already shows up as
    speed scaling, which is handled by scaling the whole command."""
    node, monitor = _monitor()

    _send(node, SAFETY, SafetyMode(mode=SafetyMode.NORMAL))
    assert monitor.blocked() is None

    _send(node, SAFETY, SafetyMode(mode=SafetyMode.REDUCED))
    assert monitor.blocked() is None


@pytest.mark.parametrize("mode", [
    SafetyMode.SAFEGUARD_STOP,
    SafetyMode.ROBOT_EMERGENCY_STOP,
    SafetyMode.VIOLATION,
    SafetyMode.FAULT,
])
def test_every_stop_state_blocks(mode):
    node, monitor = _monitor()
    _send(node, SAFETY, SafetyMode(mode=mode))
    assert monitor.blocked() is not None


def test_a_robot_that_is_not_running_blocks():
    node, monitor = _monitor()
    _send(node, SAFETY, SafetyMode(mode=SafetyMode.NORMAL))
    _send(node, ROBOT, RobotMode(mode=RobotMode.POWER_OFF))

    assert "POWER_OFF" in monitor.blocked()


def test_a_running_robot_in_a_normal_safety_state_is_clear():
    node, monitor = _monitor()
    _send(node, SAFETY, SafetyMode(mode=SafetyMode.NORMAL))
    _send(node, ROBOT, RobotMode(mode=RobotMode.RUNNING))
    _send(node, SCALING, Float64(data=1.0))

    assert monitor.blocked() is None
    assert "NORMAL" in monitor.describe()
