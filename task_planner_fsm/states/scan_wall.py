from ..state import State
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.action import GoalResponse, CancelResponse
from rclpy.task import Future
from math import atan2, sin, cos

class ScanWall(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False        # internal flag
        self.finished = False
        self.goal_sent = False
        self.waiting = False        
        
    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering scanning state.")
        self.started = False
        self.finished = False
        self.goal_sent = False
        self.waiting = False
        ctx["error_triggered"] = False

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("walls_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No walls left to scan.")
            ctx["scan_done"] = True
            self.finished = True
            return
        
        if not self.started:
            node.get_logger().info(f"[{self.name}] Initiating wall scan maneuver...")
            self.started = True

            wall_data = ctx.get("target_scan_wall", None)
            prev_target_point = ctx.get("target_scan_point", None)

            if not wall_data or not prev_target_point:
                node.get_logger().error(f"[{self.name}] Missing wall data or target point.")
                ctx["error_triggered"] = True
                return

            if all(abs(prev_target_point[i] - wall_data[0][i]) < 1e-6 for i in range(2)):
                target_point = wall_data[1]
            else:
                target_point = wall_data[0]
            other_point = prev_target_point

            dx = target_point[0] - other_point[0]
            dy = target_point[1] - other_point[1]
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

            nav_client = ctx.get("nav_client", None)
            if nav_client is None:
                node.get_logger().error(f"[{self.name}] Navigation client not found.")
                ctx["error_triggered"] = True
                return

            self._send_goal_future = nav_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            self.waiting = True

        elif self.waiting and self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                node.get_logger().error(f"[{self.name}] Goal was rejected.")
                ctx["error_triggered"] = True
                return
            node.get_logger().info(f"[{self.name}] Goal accepted. Waiting for result...")
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(lambda fut: self.result_callback(fut, ctx))

            self.waiting = False  # Dejar de esperar, ya está lanzado

    def goal_response_callback(self, future):
        pass

    def result_callback(self, future, ctx):
        node = ctx["node"]
        result = future.result().result
        status = future.result().status

        node.get_logger().info(f"[{self.name}] Scanning finished.")
        self.finished = True
        ctx["walls_left"] -= 1
    
        if ctx.get("walls_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No walls left to scan.")
            ctx["scan_done"] = True
            # self.finished = True
            return

    def check_transition(self, ctx):
        if self.finished and not ctx.get("scan_done"):
            return "WallTargetSelection"
        if ctx.get("scan_done"):
            return "HomePosition"
        if ctx.get("error_triggered"):
            return "Error"
        return None
