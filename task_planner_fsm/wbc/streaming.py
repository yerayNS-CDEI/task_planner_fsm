r"""How the QP's joint velocities reach the arm, and what "stop" means there.

The control law produces a joint velocity every cycle. There are two ways to
hand that to a UR, and they differ in the only respect that matters when the
loop stops running:

  * ``velocity`` -> ``forward_velocity_controller`` -> ``speedj(qd, 40.0, dt)``.
    The command IS a velocity. If this node dies mid-sweep, ros2_control keeps
    the last array it received and the driver keeps re-issuing it, so the arm
    carries on at that velocity until something else intervenes. In Gazebo that
    is a nuisance. On a UR10e with the plate 20 cm off a wall it is a crash.

  * ``position`` -> ``forward_position_controller`` -> ``servoj(q, t=dt, ...)``.
    The velocity is integrated here and the SETPOINT is published. A command
    that stops arriving means "hold this pose", because the setpoint stops
    advancing. The same failure that ran the arm away above now freezes it.

That asymmetry is the whole reason ``position`` is the default. It is also what
``moveit_servo`` does on UR hardware, for the same reason.

Integrating in the controller is not free, and the two hazards it introduces are
what most of this module is about:

  1. **Never publish zero to stop.** Zero is a perfectly good velocity command
     and a catastrophic position command — it asks every joint to travel to
     0 rad at whatever speed servoj can manage. :meth:`ArmStream.hold` therefore
     dispatches on the mode, and no caller should be building a stop command of
     its own.

  2. **Wind-up.** The integrator advances whether or not the robot follows. A
     protective stop, a speed slider at zero, a safeguard stop — the arm holds
     still while ``q_cmd`` runs off, and the moment the stop clears the arm
     snaps through the accumulated gap. So the setpoint is clamped to stay
     within ``max_lead`` of the measured position: the integrator can lead the
     arm (it must, or there would be no motion) but never by more than one
     bounded step, and it cannot bank error while the robot is not moving.

``send`` integrates the COMMANDED position, never the measured one. Re-seeding
from measurement every cycle would stack a second position loop on top of the
robot's own servo and ring; the lead clamp is what keeps that open loop honest.
Only :meth:`reset` and :meth:`hold` touch the measurement, and both mean "start
again from where the arm actually is".
"""

import numpy as np
from std_msgs.msg import Float64MultiArray

VELOCITY = "velocity"
POSITION = "position"
MODES = (POSITION, VELOCITY)

# Controller each mode claims, and the topic it listens on. Both are
# overridable at construction, since a namespaced robot renames them.
DEFAULT_CONTROLLER = {
    POSITION: "forward_position_controller",
    VELOCITY: "forward_velocity_controller",
}
DEFAULT_TOPIC = {
    POSITION: "/forward_position_controller/commands",
    VELOCITY: "/forward_velocity_controller/commands",
}


class ArmStream:
    """The arm's command path: one publisher, one integrator, one stop.

    ``joint_names`` is the CONTROLLER's joint order, which is what goes on the
    wire; callers hand velocities in that same order.
    """

    def __init__(self, node, joint_names, mode=POSITION, topic=None,
                 max_lead=0.2, qos=10):
        if mode not in MODES:
            raise ValueError(f"arm stream mode must be one of {MODES}, got '{mode}'")
        self.node = node
        self.joint_names = list(joint_names)
        self.mode = mode
        self.max_lead = float(max_lead)
        self.topic = topic or DEFAULT_TOPIC[mode]
        self.controller = DEFAULT_CONTROLLER[mode]
        self.publisher = node.create_publisher(Float64MultiArray, self.topic, qos)
        # Position limits of the arm, once the URDF has been parsed. The QP's
        # joint_limit_bounds already stops the VELOCITY from driving into a
        # stop; this clamps the SETPOINT, which can still drift past one while
        # the arm lags behind it.
        self.lower = None
        self.upper = None
        self.command = None      # last published setpoint (position mode)
        self.published = None    # last published array, either mode

    # ------------------------------------------------------------------
    def set_position_limits(self, lower, upper):
        """Give the integrator the arm's joint limits (from the parsed URDF)."""
        self.lower = None if lower is None else np.asarray(lower, dtype=float)
        self.upper = None if upper is None else np.asarray(upper, dtype=float)

    def reset(self, q_measured):
        """Seed the integrator at the arm's current pose. Publishes nothing.

        Must be called before the first :meth:`send` in position mode, and
        should be called right after the controller is activated — see
        :meth:`initial_command`.
        """
        self.command = None if q_measured is None else np.asarray(q_measured, dtype=float).copy()

    def initial_command(self, q_measured):
        """Seed AND publish the current pose, for the moment the switch lands.

        ``forward_position_controller`` starts commanding from whatever its
        command interfaces already hold, which after a trajectory controller has
        been running is not necessarily this pose. Publishing the measured
        position immediately after the switch makes the first setpoint a no-op
        instead of a jump. There is still a window of a few controller cycles
        between the activation and this message that nothing here can close;
        keep the two calls adjacent.
        """
        self.reset(q_measured)
        if self.mode == POSITION and self.command is not None:
            return self._publish(self.command)
        return self.hold(q_measured)

    # ------------------------------------------------------------------
    def send(self, qdot, dt, q_measured=None):
        """Publish one cycle's command. Returns the array that went out.

        In velocity mode this is ``qdot`` unchanged. In position mode it is the
        running setpoint advanced by ``qdot * dt``, clamped to the joint limits
        and to ``max_lead`` either side of ``q_measured``.
        """
        qdot = np.asarray(qdot, dtype=float)
        if self.mode == VELOCITY:
            return self._publish(qdot)

        if self.command is None:
            # No seed: refusing to guess is the only safe option, because any
            # guess is a pose the arm would travel to. Hold instead, which needs
            # no history.
            self.node.get_logger().error(
                "Arm stream was never seeded with a starting pose; holding. "
                "(reset()/initial_command() must run before the first send().)")
            return self.hold(q_measured)

        command = self.command + qdot * float(dt)
        if self.lower is not None and self.upper is not None:
            command = np.clip(command, self.lower, self.upper)
        if q_measured is not None and self.max_lead > 0.0:
            measured = np.asarray(q_measured, dtype=float)
            command = np.clip(command, measured - self.max_lead, measured + self.max_lead)
        self.command = command
        return self._publish(command)

    def hold(self, q_measured=None):
        """Stop the arm where it is. The one correct way to stop in either mode.

        Velocity mode: zeros, which is a genuine stop command.

        Position mode: the MEASURED pose, re-seeding the integrator there. Not
        zeros — that would command a full-speed run to the zero configuration —
        and not the last setpoint either: if the reason for holding is that the
        arm was not following (a protective stop, speed scaling at zero), the
        last setpoint is ahead of the arm and re-publishing it keeps that gap
        alive for the moment the stop clears. Re-seeding at the measurement
        means "stop here", and makes the resumed sweep start from the truth.
        """
        if self.mode == VELOCITY:
            return self._publish(np.zeros(len(self.joint_names)))
        if q_measured is not None:
            self.command = np.asarray(q_measured, dtype=float).copy()
        if self.command is None:
            # Nothing has ever been measured or commanded, so there is no pose
            # to hold. Publishing anything here would be a guess; publishing
            # nothing leaves the controller on its own last value, which is the
            # arm's current pose if initial_command() ran.
            return None
        return self._publish(self.command)

    def lead(self, q_measured):
        """Worst-joint gap between the setpoint and the arm, in rad.

        Meaningful only in position mode. A lead that sits at ``max_lead`` is
        the arm failing to follow — speed scaling, a protective stop, or a
        commanded velocity the joint cannot deliver.
        """
        if self.mode != POSITION or self.command is None or q_measured is None:
            return 0.0
        return float(np.max(np.abs(self.command - np.asarray(q_measured, dtype=float))))

    # ------------------------------------------------------------------
    def _publish(self, values):
        self.published = np.asarray(values, dtype=float)
        self.publisher.publish(
            Float64MultiArray(data=[float(v) for v in self.published]))
        return self.published


def slew_limit(u, u_prev, accel_max, dt):
    """Bound how far a command vector may move in one cycle.

    The QP has no memory: its active set can change between cycles (a barrier
    engages, a joint limit releases, the sweep direction cap tightens), and the
    solution can step. Streamed straight out, that step becomes a step at the
    actuator — ``speedj`` is given 40 rad/s^2 to make it with, and the base's
    controller has its own opinion — so the plate gets kicked, which is both
    hard on the hardware and noise in the very range readings the loop is
    closing on.

    A per-DOF acceleration bound is the cheapest fix and costs the sweep
    nothing: the reference velocities are tenths of a metre or radian per
    second, so a limit of a few units per second squared is invisible in normal
    operation and only bites on the steps.

    ``accel_max`` is per-DOF (or a scalar). ``u_prev`` of None passes ``u``
    through — that is "no command history at all", as at the start of a sweep.
    A robot that has been STOPPED has a history, and it is zero: callers must
    pass zeros there, or the resuming cycle jumps to full speed unbounded.
    """
    u = np.asarray(u, dtype=float)
    if u_prev is None or dt is None or dt <= 0.0:
        return u
    step = np.abs(np.broadcast_to(np.asarray(accel_max, dtype=float), u.shape)) * float(dt)
    return np.clip(u, np.asarray(u_prev, dtype=float) - step,
                   np.asarray(u_prev, dtype=float) + step)
