from ..state import State
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WallTargetSelection(State):     # necessari afegir un nou context per saber si hi ha scannable walls??
    def __init__(self, name):
        super().__init__(name)
        self.current_position = None
        self.scanned_walls_idx = []
        self.scanned_panels_idx = []
        self.step_count = 0
        self.MAX_STEPS = 5

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Selecting wall to scan...")
        self.step_count = 0
        ctx["target_selected"] = False
        ctx["error_triggered"] = False

    #     node.create_subscription(Odometry, "/rtabmap/odom", self.odometry_callback, 10)
    
    # def odometry_callback(self, msg: Odometry):
    #     pos = msg.pose.pose.position
    #     self.current_position = (pos.x, pos.y)

    def run(self, ctx):
        node = ctx["node"]

        if self.step_count >= self.MAX_STEPS:
            if not ctx.get("error_triggered"):
                print(f"[{self.name}] ERROR: The maximum time was exceeded.")
                ctx["error_triggered"] = True
            return
        
        if ctx.get("scan_phase") == 1:
            # Robot position (real or simulated)
            self.current_position = ctx.get("base_position")
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
            robot_x = self.current_position.x
            robot_y = self.current_position.y
            node.get_logger().warn(f"[{self.name}] Robot  current position: ({robot_x},{robot_y})")
            min_dist = float('inf')
            selected_wall_idx = -1
            selected_point = None

            for idx, wall in enumerate(walls_data):
                if idx not in self.scanned_walls_idx:
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
            self.scanned_walls_idx.append(selected_wall_idx)
            ctx["current_wall_index"] = selected_wall_idx
            ctx["target_scan_wall"] = walls_data[selected_wall_idx]["scan_line"]
            ctx["target_scan_point"] = selected_point
            ctx["target_selected"] = True
            node.get_logger().info(f"[{self.name}] Wall #{selected_wall_idx} selected at point {selected_point}.")

        elif ctx.get("scan_phase") == 2:
            # Robot position (real or simulated)
            self.current_position = ctx.get("base_position")
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
            
            # Getting data
            base_positions_all = ctx.get("optimal_base_results", [])
            base_positions = []         # REVISAR !!
            for i in range(len(base_positions_all)):
                if i % 2 != 0:
                    base_positions.append(base_positions_all[i])
            if not base_positions:
                node.get_logger().error(f"[{self.name}] No optimal_base_results found in context.")
                ctx["error_triggered"] = True
                return

            # Selecting closest point
            robot_x = self.current_position.x
            robot_y = self.current_position.y
            node.get_logger().warn(f"[{self.name}] Robot  current position: ({robot_x},{robot_y})")
            min_dist = float('inf')
            selected_base_idx = -1
            selected_point = None

            for idx, base in enumerate(base_positions):
                if idx not in self.scanned_panels_idx:
                    dist = ((robot_x - base[0]) ** 2 + (robot_y - base[1]) ** 2) ** 0.5
                    if dist < min_dist:
                        min_dist = dist
                        selected_base_idx = idx
                        selected_point = base

            if selected_base_idx == -1:
                node.get_logger().error(f"[{self.name}] No base point selected.")
                ctx["error_triggered"] = True
                return
            

            self.scanned_panels_idx.append(selected_base_idx)
            node.get_logger().info(f"[{self.name}] Base #{selected_base_idx} selected at point {selected_point}.")
            ctx["selected_base"] = selected_point
            ctx["target_selected"] = True
            ctx["selected_base_idx"] = selected_base_idx

        else:
            print(f"[{self.name}] Scanning phase not selected correctly.")

    def check_transition(self, ctx):
        if ctx.get("target_selected"):
            return "NavigateToTarget"
        if ctx.get("error_triggered"):
            return "Error"
        return None
