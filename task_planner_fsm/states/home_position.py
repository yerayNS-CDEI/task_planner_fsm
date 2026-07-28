from ..state import State

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

NEXT_STATE_OPTIONS = [
    "Finished",
    "Error",
]


class HomePosition(State):
    """Drive the base back to the pose it started the run from.

    The home pose is recorded by Initialization (``home_position`` /
    ``home_orientation`` in context, the map origin) and driven with the same
    nav2 ``NavigateToPose`` action NavigateToTarget uses. The gantry is expected
    to be folded before entering this state (ManipulatorFolding -> HomePosition).
    """

    def __init__(self, name):
        super().__init__(name)
        self.nav_client = None
        self.goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._last_distance_log = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Returning the robot to its home pose.")
        ctx["home_position_reached"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        self.goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._last_distance_log = None

        action_name = str(ctx.get("nav_action_name", "/navigate_to_pose"))
        if self.nav_client is None:
            self.nav_client = ActionClient(node, NavigateToPose, action_name)

        node.get_logger().info(f"[{self.name}] Waiting for {action_name} action server...")
        server_timeout = float(ctx.get("nav_server_timeout", 10.0))
        if not self.nav_client.wait_for_server(timeout_sec=server_timeout):
            node.get_logger().error(
                f"[{self.name}] {action_name} action server not available after {server_timeout:.0f}s."
            )
            ctx["error_triggered"] = True
            return

    def run(self, ctx):
        # Stay here until the robot is home (or navigation fails); check_transition
        # only advances once one of these flags is set.
        if ctx.get("home_position_reached") or ctx.get("error_triggered"):
            return

        if not self.goal_sent:
            self._send_goal(ctx)
            return

        if self.nav_done:
            node = ctx["node"]
            if self.nav_failed:
                node.get_logger().error(f"[{self.name}] Navigation failed to reach the home pose.")
                ctx["error_triggered"] = True
            else:
                node.get_logger().info(f"[{self.name}] Home position reached.")
                ctx["home_position_reached"] = True

    def _home_pose(self, ctx):
        """Build the map-frame home goal from context.

        Falls back to the map origin when the pose is missing, which happens when
        the FSM is started past Initialization (``--initial-state``).
        """
        node = ctx["node"]
        position = ctx.get("home_position")
        orientation = ctx.get("home_orientation")
        if position is None or orientation is None:
            node.get_logger().warn(
                f"[{self.name}] No home pose in context (FSM started past Initialization?); "
                f"falling back to the map origin."
            )
            position = position or Point(x=0.0, y=0.0, z=0.0)
            orientation = orientation or Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        pose = PoseStamped()
        pose.header.frame_id = str(ctx.get("map_frame", "map"))
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = float(position.x)
        pose.pose.position.y = float(position.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = float(orientation.x)
        pose.pose.orientation.y = float(orientation.y)
        pose.pose.orientation.z = float(orientation.z)
        pose.pose.orientation.w = float(orientation.w)
        return pose

    def _send_goal(self, ctx):
        node = ctx["node"]
        pose = self._home_pose(ctx)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        p = pose.pose.position
        node.get_logger().info(
            f"[{self.name}] Sending nav2 home goal ({p.x:.2f}, {p.y:.2f})."
        )
        self.goal_sent = True
        self.nav_done = False
        self.nav_failed = False
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._feedback_callback(ctx, fb),
        )
        self._send_goal_future.add_done_callback(
            lambda fut: self._goal_response_callback(ctx, fut)
        )

    def _goal_response_callback(self, ctx, future):
        node = ctx["node"]
        try:
            goal_handle = future.result()
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Goal request failed: {e}")
            self.nav_failed = True
            self.nav_done = True
            return

        if not goal_handle.accepted:
            node.get_logger().error(f"[{self.name}] Home goal was rejected.")
            self.nav_failed = True
            self.nav_done = True
            return

        node.get_logger().info(f"[{self.name}] Goal accepted; waiting for result...")
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda fut: self._result_callback(ctx, fut)
        )

    def _result_callback(self, ctx, future):
        node = ctx["node"]
        try:
            status = int(future.result().status)
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Navigation result failed: {e}")
            self.nav_failed = True
            self.nav_done = True
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().warn(f"[{self.name}] Navigation ended with status {status}.")
            self.nav_failed = True
        self.nav_done = True

    def _feedback_callback(self, ctx, feedback_msg):
        node = ctx["node"]
        distance = getattr(feedback_msg.feedback, "distance_remaining", None)
        if distance is None:
            return
        # Log at most ~once per metre of progress to avoid flooding.
        if self._last_distance_log is None or abs(self._last_distance_log - distance) >= 1.0:
            self._last_distance_log = distance
            node.get_logger().info(f"[{self.name}] Distance remaining: {distance:.2f} m.")

    def check_transition(self, ctx):
        if not ctx.get("home_position_reached") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
