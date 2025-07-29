from ..state import State
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WallTargetSelection(State):     # necessari afegir un nou context per saber si hi ha scannable walls??
    def __init__(self, name):
        super().__init__(name)
        self.started = False    # internal flag
        self.current_position = None
        self.step_count = 0
        self.MAX_STEPS = 5

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Selecting wall to scan...")
        self.started = False
        self.step_count = 0
        ctx["target_selected"] = False
        ctx["error_triggered"] = False
        self.current_position = None

        node.create_subscription(Odometry, "/odometry/global", self.odometry_callback, 10)
        # node.get_logger().info(f"Advancing to state [{self.name}].")
    
    def odometry_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y)

    def run(self, ctx):
        node = ctx["node"]

        if self.step_count >= self.MAX_STEPS:
            if not ctx.get("error_triggered"):
                print(f"[{self.name}] ERROR: The maximum time was exceeded.")
                ctx["error_triggered"] = True
            return
        
        if not False:
            self.started = True

            # Robot position (real or simulated)
            if self.current_position is None:
                node.get_logger().warn(f"[{self.name}] Waiting for odometry data...")
                return
            # try:
            #     robot_position = tuple(map(float, input(">> Current robot position (x y z): ").strip().split()))
            #     if len(robot_position) != 3:
            #         raise ValueError("Robot position must have 3 coordinates.")
            # except ValueError as e:
            #     node.get_logger().error(f"[{self.name}] Invalid robot position: {e}")
            #     ctx["error_triggered"] = True
            #     return
            
            # Getting wall data
            walls_data = ctx.get("walls_data", [])
            if not walls_data:
                node.get_logger().error(f"[{self.name}] No walls_data found in context.")
                ctx["error_triggered"] = True
                return
            
            # Selecting closest wall point
            robot_x, robot_y = self.current_position
            node.get_logger().warn(f"[{self.name}] Robot  current position: ({robot_x},{robot_y})")
            min_dist = float('inf')
            selected_wall_idx = -1
            selected_point = None

            for idx, wall in enumerate(walls_data):
                for pt in wall["scan_line"]:
                    dist = ((robot_x - pt[0]) ** 2 + (robot_y - pt[1]) ** 2) ** 0.5
                    if dist < min_dist:
                        min_dist = dist
                        selected_wall_idx = idx
                        selected_point = pt

            if selected_wall_idx == -1:
                node.get_logger().error(f"[{self.name}] No wall point selected.")
                ctx["error_triggered"] = True
                return
            
            # Saving computed data
            ctx["current_wall_index"] = selected_wall_idx
            ctx["target_scan_wall"] = walls_data[selected_wall_idx]["scan_line"]
            ctx["target_scan_point"] = selected_point
            ctx["target_selected"] = True
            node.get_logger().info(f"[{self.name}] Wall #{selected_wall_idx} selected at point {selected_point}.")

        else:
            print(f"[{self.name}] Supervising wall selection. Step {self.step_count + 1}...")
            self.step_count += 1

    def check_transition(self, ctx):
        if ctx.get("target_selected"):
            return "NavigateToTarget"
        if ctx.get("error_triggered"):
            return "Error"
        return None
