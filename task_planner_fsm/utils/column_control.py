"""Reusable column (vertical lift) controller for FSM states.

Encapsulates the column-extension logic so states can raise/lower the arm base
to a target height without duplicating the controller plumbing. Sim uses the
``follow_joint_trajectory`` action; the real robot uses a position-command
topic. The current column height is read from ``ctx["column_current_height"]``,
which the FSM node publishes from ``/joint_states``.

Extracted from the proven implementation in ``ExhaustiveScan`` so it can be
shared (currently consumed by ``ScanWall``).
"""

from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from std_msgs.msg import Float64MultiArray


class ColumnController:
    """Drives the ``column_joint`` to a commanded height and reports settling."""

    def __init__(self, owner_name: str = "ColumnController"):
        self.name = owner_name

        # Controller handles (one of the two is set by configure()).
        self.column_client = None          # trajectory_action (sim)
        self.column_command_pub = None     # position_topic (real)
        self.column_control_mode = None

        # Per-command tracking.
        self.column_target_height = None
        self.column_move_deadline = None
        self.waiting_for_column = False
        self._column_stable_since = None

        # Physical / tuning parameters (mirrors ExhaustiveScan defaults).
        self.column_min_height_m = 0.0
        self.column_max_height_m = 0.9
        # Highest map-frame z the arm reaches at column=0 in the unfolded pose.
        # Column extension is only needed for line heights above this.
        self.arm_reachable_z_max = 1.1
        self.column_tolerance_m = 0.01
        self.column_wait_timeout_s = 40.0
        self.column_move_time_s = 7.0
        self.column_settle_time_s = 0.5
        self.column_skip_movement_threshold_m = 0.025

        self.column_joint_name = "column_joint"
        self.column_action_name = "/column_controller/follow_joint_trajectory"
        self.column_command_topic = "/column_position_controller/commands"

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def configure(self, node, ctx) -> None:
        """Select the controller backend based on simulation vs real robot."""
        use_sim = bool(ctx.get("sim", False))
        desired_mode = "trajectory_action" if use_sim else "position_topic"
        if self.column_control_mode == desired_mode:
            return

        self.column_control_mode = desired_mode

        if desired_mode == "trajectory_action":
            self.column_client = ActionClient(node, FollowJointTrajectory, self.column_action_name)
            self.column_command_pub = None
            node.get_logger().info(
                f"[{self.name}] Column control configured for simulation via {self.column_action_name}"
            )
            return

        self.column_command_pub = node.create_publisher(Float64MultiArray, self.column_command_topic, 10)
        self.column_client = None
        node.get_logger().info(
            f"[{self.name}] Column control configured for real robot via {self.column_command_topic}"
        )

    # ------------------------------------------------------------------
    # Height mapping
    # ------------------------------------------------------------------
    def height_for_line_z(self, line_z: float) -> float:
        """Map a map-frame line height to a clamped column-extension target.

        The column raises the arm base, so it is only extended for line heights
        the arm cannot reach at column=0. Below ``arm_reachable_z_max`` the
        column stays retracted. This offset is the main tuning knob (see plan).
        """
        required_height = max(0.0, float(line_z) - self.arm_reachable_z_max)
        return max(self.column_min_height_m, min(required_height, self.column_max_height_m))

    # ------------------------------------------------------------------
    # Commanding
    # ------------------------------------------------------------------
    def command(self, node, ctx, height_m: float) -> bool:
        """Command the column to ``height_m``. Returns False on failure.

        If the target is within the skip threshold of the current height the
        move is treated as already complete (returns True, not waiting).
        """
        self._column_stable_since = None
        target_h = float(height_m)

        column_current = ctx.get("column_current_height", 0.0)

        if abs(target_h - column_current) <= self.column_skip_movement_threshold_m:
            self.waiting_for_column = False
            self.column_target_height = None
            self.column_move_deadline = None
            node.get_logger().info(
                f"[{self.name}] Column already close to target {target_h:.3f}m "
                f"(current={column_current:.3f}m, skipping command)."
            )
            return True

        if self.column_control_mode == "trajectory_action":
            return self._command_via_action(node, target_h, column_current)

        if self.column_control_mode == "position_topic":
            return self._command_via_topic(node, target_h, column_current)

        node.get_logger().error(
            f"[{self.name}] Column control mode not configured. Expected 'trajectory_action' or "
            f"'position_topic', got '{self.column_control_mode}'."
        )
        return False

    def _command_via_action(self, node, target_h: float, column_current: float) -> bool:
        if self.column_client is None:
            node.get_logger().error(f"[{self.name}] Column action client is not initialized.")
            return False

        if not self.column_client.wait_for_server(timeout_sec=1.0):
            node.get_logger().error(
                f"[{self.name}] Column action server {self.column_action_name} not available."
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.column_joint_name]

        pt = JointTrajectoryPoint()
        pt.positions = [target_h]

        move_time_s = float(self.column_move_time_s)
        sec = int(move_time_s)
        nanosec = int((move_time_s - sec) * 1e9)
        pt.time_from_start = DurationMsg(sec=sec, nanosec=nanosec)

        goal.trajectory.points = [pt]

        # Coarse JTC-level tolerance; reached_target() does the tight stability
        # check. Velocity set to 0.0 to disable the JTC velocity check at goal
        # time (the sim column often arrives with residual velocity).
        goal_tol = JointTolerance()
        goal_tol.name = self.column_joint_name
        goal_tol.position = 0.05
        goal_tol.velocity = 0.0
        goal.goal_tolerance = [goal_tol]

        goal.goal_time_tolerance = DurationMsg(sec=10, nanosec=0)

        self.column_target_height = target_h
        self.column_move_deadline = node.get_clock().now() + Duration(seconds=self.column_wait_timeout_s)
        self.waiting_for_column = True

        try:
            self.column_client.send_goal_async(goal)
            node.get_logger().info(
                f"[{self.name}] Column JTC goal sent: target={target_h:.3f}m "
                f"(current={column_current:.3f}m, action={self.column_action_name})"
            )
            return True
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Failed to send column goal: {e}")
            return False

    def _command_via_topic(self, node, target_h: float, column_current: float) -> bool:
        if self.column_command_pub is None:
            node.get_logger().error(f"[{self.name}] Column command publisher is not initialized.")
            return False

        cmd = Float64MultiArray()
        cmd.data = [target_h]

        try:
            self.column_command_pub.publish(cmd)
            self.column_target_height = target_h
            self.column_move_deadline = node.get_clock().now() + Duration(seconds=self.column_wait_timeout_s)
            self.waiting_for_column = True
            node.get_logger().info(
                f"[{self.name}] Column position command sent: target={target_h:.3f}m "
                f"(current={column_current:.3f}m, topic={self.column_command_topic})"
            )
            return True
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Failed to publish column position command: {e}")
            return False

    # ------------------------------------------------------------------
    # Settling
    # ------------------------------------------------------------------
    def reached_target(self, node, ctx) -> bool:
        """True once the column is within tolerance and has settled."""
        if self.column_target_height is None:
            # Nothing pending (e.g. skipped move) — treat as reached.
            return not self.waiting_for_column

        column_current = ctx.get("column_current_height", 0.0)

        pos_error = abs(column_current - self.column_target_height)
        if pos_error > self.column_tolerance_m:
            self._column_stable_since = None
            return False

        now = node.get_clock().now()
        if self._column_stable_since is None:
            self._column_stable_since = now
            return False

        stable_duration = now - self._column_stable_since
        return stable_duration >= Duration(seconds=self.column_settle_time_s)

    def timed_out(self, node) -> bool:
        """True if the in-flight column move exceeded its deadline."""
        if self.column_move_deadline is None:
            return False
        return node.get_clock().now() > self.column_move_deadline

    def reset(self) -> None:
        """Clear per-command tracking (call when (re)entering a state)."""
        self.column_target_height = None
        self.column_move_deadline = None
        self.waiting_for_column = False
        self._column_stable_since = None
