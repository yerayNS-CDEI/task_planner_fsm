from ..state import State

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

NEXT_STATE_OPTIONS = [
    "ManipulatorReachability",
    "Error",
]

class NavigateToTarget(State):
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
        node.get_logger().info(f"[{self.name}] Entering navigation state.")
        ctx["target_reached"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        self.goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._last_distance_log = None

        # Goal computed by BasePlacementComputation (a map-frame PoseStamped).
        if ctx.get("base_goal_pose") is None:
            node.get_logger().error(f"[{self.name}] No base_goal_pose in context.")
            ctx["error_triggered"] = True
            return

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
        # Stay in this state until the goal is reached (or fails). check_transition
        # only advances once one of these flags is set.
        if ctx.get("target_reached") or ctx.get("error_triggered"):
            return

        if not self.goal_sent:
            self._send_goal(ctx)
            return

        if self.nav_done:
            node = ctx["node"]
            if self.nav_failed:
                node.get_logger().error(f"[{self.name}] Navigation failed to reach the goal.")
                ctx["error_triggered"] = True
            else:
                node.get_logger().info(f"[{self.name}] Goal reached.")
                ctx["target_reached"] = True

    def _send_goal(self, ctx):
        node = ctx["node"]
        pose = ctx.get("base_goal_pose")
        if pose is None:
            node.get_logger().error(f"[{self.name}] No base_goal_pose in context.")
            ctx["error_triggered"] = True
            return

        pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        p = pose.pose.position
        node.get_logger().info(
            f"[{self.name}] Sending nav2 goal ({p.x:.2f}, {p.y:.2f})."
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
            node.get_logger().error(f"[{self.name}] Navigation goal was rejected.")
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
        if not ctx.get("target_reached") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
