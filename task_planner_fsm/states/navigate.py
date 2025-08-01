from ..state import State
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.task import Future
from math import atan2, sin, cos
import math

class NavigateToTarget(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_sent = False
        self.navigation_done = False
        self.future = None
        
    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering navigation state.")
        ctx["navigation_success"] = False
        ctx["error_triggered"] = False
        self.goal_sent = False
        self.navigation_done = False
        self.future = None

    def run(self, ctx):
        node = ctx["node"]
        nav_client: ActionClient = ctx["nav_client"]

        # if self.result_received:
        #     # Check result status
        #     if self.last_status == 4:  # SUCCEEDED
        #         node.get_logger().info(f"[{self.name}] Navigation successful.")
        #         ctx["navigation_success"] = True
        #     else:
        #         node.get_logger().warn(f"[{self.name}] Navigation failed. Status: {self.last_status}")
        #         ctx["error_triggered"] = True
        #     return
        
        if not self.goal_sent:
            wall_data = ctx.get("target_scan_wall")  # Tuple of (start_point, end_point)
            target_point = ctx.get("target_scan_point", None)

            if not wall_data or not target_point:
                node.get_logger().error(f"[{self.name}] Missing wall data or target point in context.")
                ctx["error_triggered"] = True
                return
            
            if len(wall_data) != 2:
                node.get_logger().error(f"[{self.name}] Invalid wall_data: expected 2 points.")
                ctx["error_triggered"] = True
                return

            if all(abs(target_point[i] - wall_data[0][i]) < 1e-6 for i in range(2)):
                other_point = wall_data[1]
            else:
                other_point = wall_data[0]
            
            # Orientation
            dx = other_point[0] - target_point[0]
            dy = other_point[1] - target_point[1]
            yaw = atan2(dy, dx)
            qz = sin(yaw / 2.0)
            qw = cos(yaw / 2.0)

            # Construct PoseStamped goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_point[0]
            goal_msg.pose.pose.position.y = target_point[1]
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            node.get_logger().info(f"[{self.name}] Sending goal ({target_point[0]:.2f}, {target_point[1]:.2f}) with yaw {yaw:.2f} rad.")
            self.future = nav_client.send_goal_async(goal_msg)
            # self.future.add_done_callback(lambda future: self.goal_response_callback(future, node))
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
                node.get_logger().info(f"[{self.name}] Navigation finished.")
                self.navigation_done = True

            result_future.add_done_callback(done_callback)

    # def goal_response_callback(self, future: Future, node):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         node.get_logger().warn(f"[{self.name}] Goal rejected by server.")
    #         self.result_received = True
    #         self.last_status = goal_handle.status
    #         return

    #     node.get_logger().info(f"[{self.name}] Goal accepted. Waiting for result...")
    #     self.result_future = goal_handle.get_result_async()
    #     self.result_future.add_done_callback(lambda f: self.result_callback(f, node))

    # def result_callback(self, future: Future, node):
    #         result = future.result()
    #         self.last_status = result.status
    #         self.result_received = True
    #         if self.last_status == 4:
    #             node.get_logger().info(f"[{self.name}] Goal reached successfully.")
    #         else:
    #             node.get_logger().warn(f"[{self.name}] Goal failed with status {self.last_status}.")

    def check_transition(self, ctx):
        if self.navigation_done:
            return "ArmUnfolding"
        if ctx.get("error_triggered"):
            return "Error"
        return None
