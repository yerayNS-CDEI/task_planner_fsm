#!/usr/bin/env python3
"""Measure the arm command stream from outside, with nothing changed inside.

Local tooling. Lives under ``test/``, which .gitignore excludes, and is not a
package entry point -- run it with python3.

WHY THIS AND NOT THE DIAGNOSTICS TOPIC
--------------------------------------
The sweep node can publish a full per-cycle trace, but turning it on means
passing a parameter to the FSM, and on this setup the FSM's command line is
built elsewhere. So this reads what the sweep ALREADY publishes and needs
nothing switched on:

    /forward_position_controller/commands   the arm setpoints
    /cmd_vel                                the base twist

That is less than the diagnostics carry -- there is no view of what the solver
asked for before scaling, and none of what the arm actually did -- but it is
enough for the two questions worth asking first.

QUESTION 1: IS THE LOOP SLOW IN SIMULATION TOO?
On hardware the control loop was measured at 18-25 Hz against a nominal 50,
which is what turns a smooth control law into a staircase at the actuator.

READ THE SIM-CLOCK LINE FOR THIS, NOT THE WALL-CLOCK ONE. In simulation
scan_wall launches the sweep node with ``use_sim_time:=true``, so the loop
paces itself on Gazebo's clock. Gazebo on a loaded machine runs well below real
time, so a perfectly healthy 50 Hz loop shows up as ~15 Hz of wall clock. This
tool reported exactly that once and it was misread as a slow control loop.
Both clocks are printed now, with their ratio, so the two cannot be
confused.

QUESTION 2: ARE THE SETPOINTS CONTINUOUS?
The acceleration bound and the smoothness term are both always on and need no
flag, so a run on this branch against a run on 173f7f3 is a straight A/B. What
matters is the WORST step between consecutive setpoints, not the total travel:
total is fixed by where the arm has to get to, while the peak is what is felt
as a jolt.

Note this measures the setpoint STREAM, which is a position command, so the
step is in radians per cycle rather than a velocity. Do not divide it by the
measured period to recover one: the setpoint advances by a NOMINAL period of
motion whatever the wall clock actually did, so pairing a full step with a
short period invents a speed the arm was never asked for.

USAGE
-----
Start the simulation and the FSM as usual, then::

    cd ~/ros2_ws/src/task_planner_fsm
    python3 test/setpoint_monitor.py

It prints a summary every few seconds and a final one on Ctrl-C. Run it once on
this branch and once on 173f7f3, and compare.
"""

import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


class SetpointMonitor(Node):

    def __init__(self):
        super().__init__("setpoint_monitor")
        self.declare_parameter(
            "arm_topic", "/forward_position_controller/commands")
        self.declare_parameter("base_topic", "/cmd_vel")
        self.declare_parameter("report_period", 5.0)      # s
        # Only count cycles once the sweep is really moving. The first few
        # setpoints after the controller switch are a seeded no-op and would
        # otherwise sit in the statistics as a spurious zero-motion run.
        self.declare_parameter("min_motion", 1e-6)        # rad
        # The rate the sweep node is configured for. Match it to the sweep's
        # control_rate; the sim-clock period is compared against it.
        self.declare_parameter("nominal_rate", 50.0)      # Hz

        p = self.get_parameter
        self.min_motion = float(p("min_motion").value)
        self.nominal_rate = float(p("nominal_rate").value)

        self.prev_arm = None
        self.prev_stamp = None
        self.prev_wall = None
        self.periods = deque(maxlen=20000)
        self.wall_periods = deque(maxlen=20000)
        self.steps = deque(maxlen=20000)
        self.base = deque(maxlen=20000)
        self.base_steps = deque(maxlen=20000)
        self.prev_base = None
        # Latest /clock, in seconds. None until one arrives.
        self.sim_now = None

        self.create_subscription(
            Float64MultiArray, str(p("arm_topic").value), self._on_arm, 50)
        self.create_subscription(
            Twist, str(p("base_topic").value), self._on_base, 50)
        # Best-effort and shallow: /clock is high rate and only the newest
        # value is ever wanted.
        self.create_subscription(
            Clock, "/clock", self._on_clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_timer(float(p("report_period").value), self._report)

        self.get_logger().info(
            f"Watching {p('arm_topic').value} and {p('base_topic').value}. "
            f"Ctrl-C for the final summary.")

    def _on_arm(self, msg):
        # BOTH clocks on every sample, because getting this wrong already cost
        # one wrong conclusion.
        #
        # In simulation scan_wall launches the sweep node with
        # ``use_sim_time:=true``, so every period the control loop reasons about
        # is in SIM seconds. A loop ticking healthily at 50 Hz of sim time shows
        # up as ~15 Hz of wall clock when the simulator runs slow, and that is a
        # completely different problem from a slow control loop.
        #
        # Sim time is read from /clock DIRECTLY rather than from
        # self.get_clock(). That only follows /clock when the node ITSELF has
        # use_sim_time set, and this one does not -- so it returned the system
        # clock, and the "real-time factor" came out as the system clock over
        # the system clock, which is 1.00 by construction and means nothing.
        # Subscribing removes the dependence on remembering a launch argument.
        sim = self.sim_now
        wall = time.monotonic()
        q = np.array(msg.data, dtype=float)
        # No /clock yet, or none ever. Advance the history but record nothing,
        # rather than quietly substituting wall time and calling it sim time.
        usable = (sim is not None and self.prev_stamp is not None
                  and self.prev_wall is not None and self.prev_arm is not None
                  and len(q) == len(self.prev_arm))
        if usable:
            step = np.abs(q - self.prev_arm).max()
            dt = sim - self.prev_stamp
            dt_wall = wall - self.prev_wall
            if step > self.min_motion and dt_wall > 0.0 and dt > 0.0:
                self.steps.append(step)
                self.periods.append(dt)
                self.wall_periods.append(dt_wall)
        self.prev_arm, self.prev_wall = q, wall
        # Only ever advanced alongside a real /clock reading, so it can never
        # hold a wall-clock value.
        if sim is not None:
            self.prev_stamp = sim

    def _on_clock(self, msg):
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_base(self, msg):
        v = float(msg.linear.x)
        if self.prev_base is not None:
            self.base_steps.append(abs(v - self.prev_base))
        self.prev_base = v
        self.base.append(v)

    def _report(self):
        if not self.steps:
            self.get_logger().info(
                "No arm motion seen yet."
                + ("" if self.sim_now is not None else
                   "  (and no /clock either, so sim time is unavailable)"))
            return
        periods = np.array(self.periods)
        steps = np.array(self.steps)
        # 1 / mean(period), NOT mean(1 / period). The second is what this
        # printed first and it is badly biased: a handful of near-simultaneous
        # messages contribute enormous reciprocals and drag the average up, so
        # a loop genuinely running at 16 Hz reported 19, and reported 131 over
        # the first few seconds. Time per cycle is the honest quantity; rate is
        # derived from it.
        mean_period = float(periods.mean())
        nominal = 1.0 / float(self.nominal_rate)
        # Setpoint distance per second of REAL time. This replaces a
        # "arm tracks at N% of real time" line that was computed as
        # nominal / period — which silently assumed the node integrates the
        # nominal period, and would therefore have gone on reporting 31% after
        # the very change that stopped it doing so. Nothing here can see the
        # commanded velocity, so no honest percentage is available from
        # outside; what IS available is speed, and it is directly comparable
        # between two runs.
        speed = float(steps.sum() / periods.sum()) if periods.sum() > 0 else 0.0
        wall_periods = np.array(self.wall_periods)
        mean_wall = float(wall_periods.mean())
        # Sim seconds per wall second. 1.0 means the simulator is keeping up;
        # below that it is running slow, and every sim-clock rate the control
        # loop reports will look correct while the wall clock disagrees.
        rtf = mean_period / mean_wall if mean_wall > 0 else float("nan")
        self.get_logger().info(
            f"\n  cycles              {len(steps)}"
            f"\n  period, SIM clock   {mean_period * 1e3:.1f} ms mean "
            f"({1.0 / mean_period:.1f} Hz), {periods.std() * 1e3:.1f} ms std, "
            f"{periods.max() * 1e3:.1f} ms worst"
            f"\n  period, WALL clock  {mean_wall * 1e3:.1f} ms mean "
            f"({1.0 / mean_wall:.1f} Hz)"
            f"\n  real-time factor    {rtf:.2f}x   <- the loop is judged on the "
            f"SIM line, not the wall line"
            f"\n  vs nominal          {nominal * 1e3:.1f} ms "
            f"({self.nominal_rate:.0f} Hz)"
            f"\n  arm setpoint step   {steps.mean() * 1e3:.3f} mrad mean, "
            f"{steps.max() * 1e3:.3f} mrad worst"
            f"\n  setpoint speed      {speed:.4f} rad/s (worst joint, real time)"
            f"\n  base travel         "
            + (f"{np.array(self.base).max():+.4f} m/s peak, "
               f"{np.array(self.base_steps).max():.4f} m/s worst step"
               if self.base_steps else "nothing published"))


def main():
    rclpy.init(args=sys.argv[1:])
    node = SetpointMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Final summary:")
        node._report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
