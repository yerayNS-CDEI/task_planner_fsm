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
        ctx.setdefault("completed_base_indices", [])

    #     node.create_subscription(Odometry, "/rtabmap/odom", self.odometry_callback, 10)
    
    # def odometry_callback(self, msg: Odometry):
    #     pos = msg.pose.pose.position
    #     self.current_position = (pos.x, pos.y)

    def _closest_scan_line(self, walls_data, x, y):
        min_dist = float("inf")
        selected_line = None
        selected_point = None

        for wall in walls_data:
            scan_line = wall.get("scan_line")
            if not scan_line or len(scan_line) != 2:
                continue
            for pt in scan_line:
                dist = ((x - pt[0]) ** 2 + (y - pt[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    selected_line = scan_line
                    selected_point = pt

        return selected_line, selected_point

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
            
            # Getting data (dict {col_rank: (x,y)})
            base_positions = ctx.get("optimal_base_results", {})
            if not base_positions:
                node.get_logger().error(f"[{self.name}] No optimal_base_results found in context.")
                ctx["error_triggered"] = True
                return

            # Selecting closest point
            robot_x = self.current_position.x
            robot_y = self.current_position.y
            node.get_logger().warn(f"[{self.name}] Robot  current position: ({robot_x},{robot_y})")
            min_dist = float('inf')
            selected_col_rank = -1
            selected_point = None
            completed_base_indices = set(ctx.get("completed_base_indices", []))

            for col_rank, base in base_positions.items():
                if col_rank in completed_base_indices:
                    continue
                dist = ((robot_x - base[0]) ** 2 + (robot_y - base[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    selected_col_rank = col_rank
                    selected_point = base

            if selected_col_rank == -1:
                node.get_logger().error(f"[{self.name}] No base point selected.")
                ctx["error_triggered"] = True
                return
            
            node.get_logger().info(f"[{self.name}] Column {selected_col_rank} selected at point {selected_point}.")
            ctx["selected_base"] = selected_point
            ctx["target_selected"] = True
            ctx["selected_base_idx"] = selected_col_rank

            # NavigateToTarget expects target_scan_wall/target_scan_point for orientation.
            walls_data = ctx.get("walls_data", [])
            scan_line, scan_point = self._closest_scan_line(walls_data, selected_point[0], selected_point[1])
            if scan_line and scan_point:
                ctx["target_scan_wall"] = scan_line
                ctx["target_scan_point"] = scan_point
            else:
                node.get_logger().warn(
                    f"[{self.name}] Could not infer target_scan_wall from walls_data; "
                    "navigation yaw may not be wall-aligned."
                )

        else:
            print(f"[{self.name}] Scanning phase not selected correctly.")

    def check_transition(self, ctx):
        if ctx.get("target_selected"):
            return "NavigateToTarget"
        if ctx.get("error_triggered"):
            return "Error"
        return None
