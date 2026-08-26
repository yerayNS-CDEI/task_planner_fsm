r"""What the real UR tells us that Gazebo does not: speed scaling and safety state.

Two facts about the physical robot have no simulated counterpart, and both break
a WHOLE-BODY loop in ways they would not break an arm-only one.

**Speed scaling.** The teach pendant's slider, and the safety configuration's
reduced mode, scale how fast the arm actually executes what it is told. UR ships
``ScaledJointTrajectoryController`` precisely because of this — it advances a
trajectory only by the fraction speed scaling allows. The forward/streaming
controllers get no such treatment: they hand the command through untouched.

For an arm on a bench that is harmless; the arm just runs slower. Here it is
not, because the base is a completely separate robot that has never heard of the
UR's slider. At 50% scaling the arm moves at half speed and the base at full
speed, so the coordination the QP just solved is quietly wrong: the base sweeps
along the wall while the arm fails to keep up with the standoff and orientation
corrections that were supposed to accompany it. The fix is to scale the WHOLE
command — base twist included — by the same factor, which turns the slider into
an honest speed control for the entire robot instead of a way to desynchronise
it. That is one line at the publisher; this class is where the factor comes from.

**Safety state.** When the UR protective-stops, ros2_control keeps running
happily and the arm simply stops moving. Nothing in the control loop notices:
the QP still solves, commands still publish, and — the dangerous part — the base
keeps driving along the wall with a frozen arm. So the loop needs to know, and
the only place it can learn it is ``io_and_status_controller``.

Both degrade to "no hardware here" when the topics never appear, which is what
happens in Gazebo, so nothing has to be gated on a ``sim`` flag.
"""

from std_msgs.msg import Float64

try:  # Only present where the UR driver is installed; absent is simply "sim".
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
    _HAVE_UR_MSGS = True
except ImportError:  # pragma: no cover - exercised only without the UR driver
    _HAVE_UR_MSGS = False

# Safety modes the sweep may run in. REDUCED is included deliberately: it means
# the robot is inside a safety plane and limited, not that it is stopped, and
# the resulting speed scaling is already handled by scaling the command.
_SAFE_MODES = (1, 2) if not _HAVE_UR_MSGS else (SafetyMode.NORMAL, SafetyMode.REDUCED)
_SAFETY_NAMES = {
    1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY",
    5: "SAFEGUARD_STOP", 6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION", 9: "FAULT", 10: "VALIDATE_JOINT_ID",
    11: "UNDEFINED_SAFETY_MODE", 12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
    13: "SYSTEM_THREE_POSITION_ENABLING_STOP",
}
_ROBOT_RUNNING = 7 if not _HAVE_UR_MSGS else RobotMode.RUNNING
_ROBOT_NAMES = {
    -1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING",
    3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING",
    8: "UPDATING_FIRMWARE",
}


class HardwareMonitor:
    """Speed scaling factor and safety gate, both no-ops until hardware speaks.

    Subscriptions are created unconditionally — a topic nobody publishes costs
    nothing — so the same node runs in Gazebo and on the robot with no flag.
    """

    def __init__(self, node, speed_scaling_topic="/speed_scaling_state_broadcaster/speed_scaling",
                 robot_mode_topic="/io_and_status_controller/robot_mode",
                 safety_mode_topic="/io_and_status_controller/safety_mode"):
        self.node = node
        self._scaling = None
        self._robot_mode = None
        self._safety_mode = None

        node.create_subscription(Float64, speed_scaling_topic, self._on_scaling, 10)
        if _HAVE_UR_MSGS:
            node.create_subscription(RobotMode, robot_mode_topic, self._on_robot_mode, 10)
            node.create_subscription(SafetyMode, safety_mode_topic, self._on_safety_mode, 10)
        else:
            node.get_logger().info(
                "ur_dashboard_msgs is not installed: running without the robot's "
                "safety gate. Expected in simulation.")

    # ------------------------------------------------------------------
    def _on_scaling(self, msg):
        self._scaling = float(msg.data)

    def _on_robot_mode(self, msg):
        self._robot_mode = int(msg.mode)

    def _on_safety_mode(self, msg):
        self._safety_mode = int(msg.mode)

    # ------------------------------------------------------------------
    def seen(self):
        """True once the robot has said anything about itself."""
        return (self._scaling is not None or self._robot_mode is not None
                or self._safety_mode is not None)

    def scaling(self):
        """Execution speed as a fraction in [0, 1]; 1.0 where nothing reports it.

        Defaulting to 1.0 rather than 0.0 is deliberate: a missing topic must
        not silently stop a simulated robot that has no such concept. The cost
        is that a driver which stops publishing mid-sweep leaves us commanding
        full speed — but that same failure takes the joint states with it, and
        the loop's staleness check already covers it.
        """
        if self._scaling is None:
            return 1.0
        # Clamp rather than trust: a factor above 1 would AMPLIFY the command.
        return max(0.0, min(1.0, self._scaling))

    def blocked(self):
        """Reason the robot must not be commanded right now, or None.

        Silent when the robot has never reported a mode, so this is inert in
        simulation and live the moment ``io_and_status_controller`` is up.
        """
        if self._safety_mode is not None and self._safety_mode not in _SAFE_MODES:
            name = _SAFETY_NAMES.get(self._safety_mode, str(self._safety_mode))
            return f"robot safety mode is {name}"
        if self._robot_mode is not None and self._robot_mode != _ROBOT_RUNNING:
            name = _ROBOT_NAMES.get(self._robot_mode, str(self._robot_mode))
            return f"robot mode is {name}, not RUNNING"
        return None

    def describe(self):
        """One-line state for the cycle log."""
        if not self.seen():
            return "no hardware"
        safety = _SAFETY_NAMES.get(self._safety_mode, "?")
        return f"scale={self.scaling():.2f} safety={safety}"
