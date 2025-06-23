from ..state import State
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class HomePosition(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_sent = False
        self.navigation_done = False
        self.future = None
        
    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering home state.")
        ctx["home"] = False
        ctx["error_triggered"] = False
        self.goal_sent = False
        self.navigation_done = False
        self.future = None

    def run(self, ctx):
        node = ctx["node"]        
        nav_client: ActionClient = ctx["nav_client"]
        
        if not self.goal_sent:
            target_point = ctx.get("home_position", None)
            target_orientation = ctx.get("home_orientation", None)

            if not target_point or not target_orientation:
                node.get_logger().error(f"[{self.name}] Missing home poisiton or orientation in context.")
                ctx["error_triggered"] = True
                return

            # Construct PoseStamped goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_point.x
            goal_msg.pose.pose.position.y = target_point.y
            goal_msg.pose.pose.position.z = target_point.z
            goal_msg.pose.pose.orientation = target_orientation

            node.get_logger().info(f"[{self.name}] Sending goal ({target_point.x:.2f}, {target_point.y:.2f}).")
            self.future = nav_client.send_goal_async(goal_msg)
            self.goal_sent = True

        elif self.future and self.future.done():
            goal_handle = self.future.result()
            if not goal_handle.accepted:
                node.get_logger().warn(f"[{self.name}] Navigation goal was rejected!")
                ctx["error_triggered"] = True
                return
            node.get_logger().info(f"[{self.name}] Goal accepted. Waiting for result...")

            result_future = goal_handle.get_result_async()

            def done_callback(fut):
                result = fut.result().result
                node.get_logger().info(f"[{self.name}] Home pose reached.")
                self.navigation_done = True

            result_future.add_done_callback(done_callback)

    def check_transition(self, ctx):
        if self.navigation_done:
            return "Finished"
        if ctx.get("error_triggered"):
            return "Error"
        return None
