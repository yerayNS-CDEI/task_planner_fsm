from ..state import State
from ..utils.wall_geometry import left_scan_endpoint
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
        """Return the wall whose scan line passes closest to (x, y), or None."""
        min_dist = float("inf")
        selected_wall = None

        for wall in walls_data:
            scan_line = wall.get("scan_line")
            if not scan_line or len(scan_line) != 2:
                continue
            for pt in scan_line:
                dist = ((x - pt[0]) ** 2 + (y - pt[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    selected_wall = wall

        return selected_wall

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

            # The closest endpoint above only picks the wall; always START the sweep
            # from the LEFT endpoint (as seen facing the wall) so the robot's left
            # flank -- where the sensor plate unfolds -- faces the wall. Downstream
            # (NavigateToTarget/ScanWall) derive the heading from start->far end, so
            # forcing a left-to-right sweep here fixes the base orientation.
            selected_wall = walls_data[selected_wall_idx]
            selected_point = left_scan_endpoint(
                selected_wall["scan_line"], selected_wall.get("inward_normal")
            )

            # Saving computed data
            self.scanned_walls_idx.append(selected_wall_idx)
            ctx["current_wall_index"] = selected_wall_idx
            ctx["target_scan_wall"] = selected_wall["scan_line"]
            ctx["target_scan_point"] = selected_point
            ctx["target_selected"] = True

            # Horizontal scan lines (heights) for this wall, bottom-first.
            # ScanWall consumes them one at a time, self-looping per line.
            scan_lines_z = walls_data[selected_wall_idx].get("scan_lines_z")
            if not scan_lines_z:
                # Fallback to a single line at the wall's scan-line z.
                scan_lines_z = [walls_data[selected_wall_idx]["scan_line"][0][2]]
            ctx["current_wall_scan_lines"] = list(scan_lines_z)
            ctx["current_line_idx"] = 0

            node.get_logger().info(
                f"[{self.name}] Wall #{selected_wall_idx} selected at point {selected_point}. "
                f"{len(scan_lines_z)} line(s) at z={[round(z, 3) for z in scan_lines_z]}."
            )

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
            # Use the LEFT scan endpoint so the base parks with its left flank (sensor
            # plate side) facing the wall, matching the phase-1 convention.
            walls_data = ctx.get("walls_data", [])
            selected_wall = self._closest_scan_line(walls_data, selected_point[0], selected_point[1])
            if selected_wall and selected_wall.get("scan_line"):
                ctx["target_scan_wall"] = selected_wall["scan_line"]
                ctx["target_scan_point"] = left_scan_endpoint(
                    selected_wall["scan_line"], selected_wall.get("inward_normal")
                )
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
