#!/usr/bin/env python3
"""A pretend force/torque sensor, so the press can be exercised in Gazebo.

Local tooling. It lives under ``test/``, which .gitignore excludes, and it is
not a package entry point -- run it with python3 directly.

WHY THIS EXISTS
---------------
Gazebo has no force/torque sensor at all: the ``<sensor>`` block in
arm_control's ``ur.ros2_control.xacro`` sits inside an
``unless sim_gazebo or sim_ignition``. So ``press_enabled`` is off in
simulation, ``AdmittancePress`` is never constructed, and every line that
depends on it is unreachable -- including the base gate, which is the one
change most in need of checking before it meets a wall.

This closes that hole from the outside. The sweep reads its force from a topic
and has no opinion about who publishes it, so a node that models the wall and
publishes a plausible wrench makes the whole press path runnable in simulation:
tare, seek, contact, the gate opening, the sweep, the retreat.

What it does NOT do is tell you the press is tuned right. The stiffness here is
a number in a parameter, not concrete, and the real contact goes through four
rigid caster bars. Treat a good run as "the plumbing works", never as "the gain
is safe".

HOW IT MODELS THE WALL
----------------------
It reads the plate's own six ranges from ``/distance_sensors`` and fits them
with the SAME plane fit the sweep uses, so both sides agree about where the
wall is. Past ``contact_distance`` the wall behaves as a spring. Below it,
nothing touches and the force is zero.

Contact is at ~0.144 m of PLATE distance, not at zero: the GPR body has length
along the wall normal and four bars with caster wheels stand off the plate's
corners, so the wheel is loading while the plate still reads over 14 cm. That
is where the force first appeared on the robot on 2026-09-14 (the plate then
bottomed out at 13.75, which is the sweep's ``press_contact_distance``); a fake
wall at the stop itself sits inside the schedule's asymptote and flatters the
approach, which is the configuration that hid the real problem for a week.

The published force also carries a constant offset, because a real untared TCP
sensor reads several newtons of payload against a 5 N target. That offset is
not decoration -- it is what gives the tare something to measure, and a run
without it silently skips the most failure-prone part of the sequence.

The loop closes through Gazebo: the arm pushes in, the simulated ranges shrink,
the force here grows, and the press backs off. Nothing needs to be synchronised.

USAGE
-----
Start the simulation and the FSM first, then::

    cd ~/ros2_ws/src/task_planner_fsm
    python3 test/fake_wrench_publisher.py

The FSM must be told to press, which it will not do in simulation on its own::

    ros2 run task_planner_fsm fsm_node --sim true \
      --initial-state GeometryReconstruction \
      --ros-args -p sweep_use_wbc:=true -p wbc_press:=true

To check the thing the gate fix is actually about, make the wall drop out now
and then. The press will fall back to SEEK, and the base must NOT stop::

    python3 test/fake_wrench_publisher.py --ros-args \
      -p dropout_period:=8.0 -p dropout_duration:=1.0

To reproduce the head-on overshoot instead, raise the stiffness::

    python3 test/fake_wrench_publisher.py --ros-args -p stiffness:=1.0e6
"""

import sys

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from task_planner_fsm.wbc.surface import fit_wall_plane


class FakeWrenchPublisher(Node):

    def __init__(self):
        super().__init__("fake_wrench_publisher")

        # Plate distance at which the GPR wheel and all four caster bars are
        # riding the wall. This is the contact point, and it is nowhere near
        # zero -- see the module docstring.
        self.declare_parameter("contact_distance", 0.144)     # m
        # Wall stiffness. The default is a soft-ish concrete that the shipped
        # press gain settles against cleanly. Real concrete through rigid bars
        # is far stiffer; raise this to reproduce the overshoot at first touch
        # rather than to be realistic, because being realistic here mostly
        # means tripping press_force_limit on contact.
        self.declare_parameter("stiffness", 2.0e4)            # N/m
        # The untared payload offset. What a real TCP sensor reads with nothing
        # touching it, and what the tare exists to remove. Set it to 0.0 only if
        # you specifically want to skip testing the tare.
        self.declare_parameter("bias", 4.0)                   # N
        self.declare_parameter("noise", 0.05)                 # N, std dev
        # 100 Hz, matching the rate the real controller_manager runs at, so the
        # sweep sees the same message cadence it will see on the robot.
        self.declare_parameter("rate", 100.0)                 # Hz
        self.declare_parameter("wrench_topic",
                               "/force_torque_sensor_broadcaster/wrench")
        self.declare_parameter("distance_topic", "/distance_sensors")
        # Periodic loss of contact: a hollow, a gap, the wheel riding over a
        # lip. Zero disables it. This is what tells you whether the base gate
        # LATCHES: the press will drop back to SEEK during a dropout, and a base
        # that stops and restarts with it is the bug that broke two commits.
        self.declare_parameter("dropout_period", 0.0)         # s, 0 = never
        self.declare_parameter("dropout_duration", 1.0)       # s

        p = self.get_parameter
        self.contact_distance = float(p("contact_distance").value)
        self.stiffness = float(p("stiffness").value)
        self.bias = float(p("bias").value)
        self.noise = float(p("noise").value)
        self.dropout_period = float(p("dropout_period").value)
        self.dropout_duration = float(p("dropout_duration").value)

        self.distance = None
        self.rng = np.random.default_rng()
        self.started = self.get_clock().now().nanoseconds * 1e-9

        self.pub = self.create_publisher(
            WrenchStamped, str(p("wrench_topic").value), 10)
        self.create_subscription(
            Float32MultiArray, str(p("distance_topic").value),
            self._on_distances, 10)
        self.create_timer(1.0 / float(p("rate").value), self._tick)

        self.get_logger().warn(
            f"FAKE force/torque sensor on {p('wrench_topic').value}. Contact at "
            f"{self.contact_distance * 100:.0f} cm of plate distance, wall "
            f"{self.stiffness:.0e} N/m, {self.bias:+.1f} N of untared offset. "
            f"This is NOT a measurement of anything: it says the press PLUMBING "
            f"works, never that the gain is safe on concrete.")
        if self.dropout_period > 0.0:
            self.get_logger().warn(
                f"Contact will drop out for {self.dropout_duration:.1f}s every "
                f"{self.dropout_period:.1f}s. The press should fall back to SEEK "
                f"and the BASE SHOULD KEEP MOVING — if it stops and restarts, the "
                f"travel gate is following the live contact state instead of "
                f"latching.")

    def _on_distances(self, msg):
        """Same six ranges and the same plane fit the sweep itself uses."""
        if len(msg.data) != 6:
            return
        _, distance, n_valid = fit_wall_plane(np.array(msg.data, dtype=float))
        # A fit that fails is the plate not seeing the wall, which is a real
        # state and means no contact, not stale contact.
        self.distance = distance if n_valid >= 3 else None

    def _in_dropout(self, now):
        if self.dropout_period <= 0.0:
            return False
        return (now - self.started) % self.dropout_period < self.dropout_duration

    def _tick(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds * 1e-9

        if self.distance is None or self._in_dropout(seconds):
            press = 0.0
        else:
            # Spring past the caster standoff, nothing before it.
            penetration = max(0.0, self.contact_distance - self.distance)
            press = self.stiffness * penetration

        raw = press + self.bias + float(self.rng.normal(0.0, self.noise))

        msg = WrenchStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "tool0"
        # The sweep reads this as ``press_force = -wrench.force.z``: pushing the
        # wheel INTO the wall loads the sensor in -Z. Flip here so the sign
        # convention matches the real broadcaster.
        msg.wrench.force.z = -raw
        self.pub.publish(msg)

        self.get_logger().info(
            f"d={'--' if self.distance is None else f'{self.distance * 100:.1f}cm'} "
            f"press={press:+.2f}N raw={raw:+.2f}N"
            f"{'  [DROPOUT]' if self._in_dropout(seconds) else ''}",
            throttle_duration_sec=1.0)


def main():
    rclpy.init(args=sys.argv[1:])
    node = FakeWrenchPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
