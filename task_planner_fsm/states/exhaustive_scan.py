from ..state import State
from statistics import median
from typing import List, Tuple, Dict
from collections import deque
import numpy as np
import rclpy.time
import math

from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from arm_control.srv import ScriptCommand

class ExhaustiveScan(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_pub = None
        self.goals_queue = deque()
        self.current_goal_idx = -1
        self.waiting_for_arrival = False
        self.movement_done = False
        self.tf_buffer = None
        self.tf_listener = None
        self.goals_initialized = False
        self._pending_panel_cells = None
        self._pending_panel_vertices = None
        self.wall_orientation = None  # Store computed orientation
        
        # Script command service client
        self.script_command_client = None
        self.pending_service_future = None
        self.total_goals_count = 0  # Track total number of goals
        
        # Column extension tracking
        self.column_client = None
        self.column_target_height = None
        self.column_move_deadline = None
        self.waiting_for_column = False
        self._column_stable_since = None
        
        # Goals grouped by column height
        self.goals_by_height: Dict[float, deque] = {}
        self.height_sequence = []  # Ordered list of heights to process
        self.current_height_idx = -1
        
        # Column parameters (similar to goal_router)
        self.column_admissible_heights = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        self.arm_reachable_z_min = 0.0
        self.arm_reachable_z_max = 1.1
        self.column_tolerance_m = 0.01  # Increased from 0.005 to 0.01 (10mm tolerance)
        self.column_wait_timeout_s = 12.0  # Increased from 8.0 to 12.0 seconds
        self.column_move_time_s = 3.0
        self.column_joint_name = "column_joint"
        self.column_action_name = "/column_controller/follow_joint_trajectory"
        self.column_vel_tol = 0.002
        self.column_settle_time_s = 0.5  # Increased from 0.25 to 0.5 seconds for better stability
        
        # Movement parameters for movel command
        self.movel_acceleration = 1.2  # m/s²
        self.movel_velocity = 0.25  # m/s
        self.movel_blend_radius = 0.0  # Stop at each point

    def _choose_column_height_for_goal_z(self, goal_z_in_arm_base: float) -> float:
        """
        Choose minimal column height that makes the goal vertically reachable.
        Similar to goal_router's choose_column_height_for_goal.
        Returns selected height (or 0.0 if goal is below arm_base or already reachable).
        """
        # If target is below arm_base, no column extension needed
        if goal_z_in_arm_base < 0.0:
            return 0.0
        
        # Check each admissible height (prefer minimal extension)
        heights_sorted = sorted(self.column_admissible_heights)
        
        for h in heights_sorted:
            # When column extends by h, goal appears h lower in arm_base frame
            adjusted_z = goal_z_in_arm_base - h
            if self.arm_reachable_z_min <= adjusted_z <= self.arm_reachable_z_max:
                return h
        
        # If no height works, return maximum (last resort)
        return heights_sorted[-1] if heights_sorted else 0.0
    
    def _command_column(self, node, ctx, height_m: float):
        """
        Send column extension command (similar to goal_router's command_column).
        """
        self._column_stable_since = None
        target_h = float(height_m)
        
        column_current = ctx.get("column_current_height", 0.0)
        
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
        
        self.column_target_height = target_h
        self.column_move_deadline = node.get_clock().now() + Duration(seconds=self.column_wait_timeout_s)
        self.waiting_for_column = True
        
        try:
            send_future = self.column_client.send_goal_async(goal)
            node.get_logger().info(
                f"[{self.name}] Column extension command sent: target={target_h:.3f}m "
                f"(current={column_current:.3f}m)"
            )
            return True
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Failed to send column goal: {e}")
            return False
    
    def _column_reached_target(self, node, ctx) -> bool:
        """
        Check if column has reached target height with stability (similar to goal_router).
        """
        if self.column_target_height is None:
            return False
        
        column_current = ctx.get("column_current_height", 0.0)
        
        # Check position tolerance
        pos_error = abs(column_current - self.column_target_height)
        if pos_error > self.column_tolerance_m:
            self._column_stable_since = None
            return False
        
        # Position is within tolerance - check stability
        now = node.get_clock().now()
        if self._column_stable_since is None:
            self._column_stable_since = now
            return False
        
        stable_duration = now - self._column_stable_since
        return stable_duration >= Duration(seconds=self.column_settle_time_s)

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Convert a 3x3 rotation matrix to a quaternion (x, y, z, w).
        """
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)
    
    def _quaternion_to_rotation_vector(self, qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
        """
        Convert quaternion to rotation vector (axis-angle representation).
        Returns (rx, ry, rz) in radians.
        """
        # Normalize quaternion
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-6:
            return (0.0, 0.0, 0.0)
        
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        # Handle numerical issues
        if qw > 1.0:
            qw = 1.0
        elif qw < -1.0:
            qw = -1.0
        
        # Compute angle
        angle = 2.0 * math.acos(qw)
        
        # Compute axis
        sin_half_angle = math.sqrt(1.0 - qw*qw)
        
        if sin_half_angle < 1e-6:
            # Angle is close to 0, return zero rotation
            return (0.0, 0.0, 0.0)
        
        axis_x = qx / sin_half_angle
        axis_y = qy / sin_half_angle
        axis_z = qz / sin_half_angle
        
        # Rotation vector = axis * angle
        return (axis_x * angle, axis_y * angle, axis_z * angle)

    def _compute_wall_normal_from_vertices(self, node, tf_buffer, panel_vertices: List, resolution: float = None) -> np.ndarray:
        """
        Compute wall normal from panel vertices in arm_base frame.
        panel_vertices is a list of Pose objects (4 vertices per panel).
        Returns normalized wall normal vector in arm_base frame.
        """
        if len(panel_vertices) < 4:
            node.get_logger().error(f"[{self.name}] Not enough vertices to compute wall normal.")
            return None
        
        # Check if transform is available before attempting lookup
        if not tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.1)):
            node.get_logger().debug(f"[{self.name}] Transform map->arm_base not yet available, waiting...")
            return None
        
        # Transform first panel's vertices from map to arm_base
        try:
            tf = tf_buffer.lookup_transform(
                'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
            )
        except Exception as e:
            node.get_logger().warn(f"[{self.name}] TF lookup map->arm_base failed: {e}")
            return None
        
        # Take first 4 vertices (first panel)
        vertices_arm_base = []
        now = node.get_clock().now().to_msg()
        for i in range(min(4, len(panel_vertices))):
            v = panel_vertices[i]
            ps_map = PoseStamped()
            ps_map.header.frame_id = 'map'
            ps_map.header.stamp = now
            ps_map.pose = v
            ps_arm = do_transform_pose_stamped(ps_map, tf)
            vertices_arm_base.append(np.array([
                ps_arm.pose.position.x,
                ps_arm.pose.position.y,
                ps_arm.pose.position.z
            ]))
        
        # Compute two edge vectors from the panel vertices
        # Assuming vertices form a quad: v0, v1, v2, v3
        edge1 = vertices_arm_base[1] - vertices_arm_base[0]
        edge2 = vertices_arm_base[2] - vertices_arm_base[0]
        
        # Wall normal via cross product (right-hand rule)
        wall_normal = np.cross(edge1, edge2)
        
        if np.linalg.norm(wall_normal) < 1e-6:
            node.get_logger().error(f"[{self.name}] Degenerate panel - cannot compute normal.")
            return None
        
        wall_normal = wall_normal / np.linalg.norm(wall_normal)
        
        # Ensure normal points away from origin (robot is typically at origin in arm_base)
        # Use centroid of the panel to determine direction
        centroid = np.mean(vertices_arm_base, axis=0)
        if np.dot(wall_normal, centroid) < 0:
            wall_normal = -wall_normal
        
        node.get_logger().info(
            f"[{self.name}] Computed wall normal from vertices: "
            f"n=({wall_normal[0]:.3f}, {wall_normal[1]:.3f}, {wall_normal[2]:.3f})"
        )
        
        return wall_normal
    
    def _compute_orientation_towards_wall(self, wall_normal: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Compute orientation quaternion where z-axis points towards the wall.
        wall_normal is the normal vector of the wall in arm_base frame.
        Returns quaternion (x, y, z, w).
        """
        # Build a rotation matrix with z-axis = wall_normal (approach direction)
        z_axis = wall_normal / np.linalg.norm(wall_normal)
        
        # Choose y-axis to be roughly upward (world Z direction)
        world_up = np.array([0.0, 0.0, 1.0])
        
        # Compute x-axis = world_up × z_axis (cross product)
        x_axis = np.cross(world_up, z_axis)
        
        # Handle edge case: z_axis parallel to world_up
        if np.linalg.norm(x_axis) < 1e-6:
            # Choose a different reference direction
            world_up = np.array([0.0, 1.0, 0.0])
            x_axis = np.cross(world_up, z_axis)
        
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Recompute y-axis = z × x to ensure orthogonality and right-handed frame
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Build rotation matrix: R = [x_axis | y_axis | z_axis]
        R = np.column_stack([x_axis, y_axis, z_axis])
        
        # Convert to quaternion
        return self._rotation_matrix_to_quaternion(R)

    def _estimate_step_threshold(self, sorted_vals: List[float], resolution: float = None) -> float:
        if len(sorted_vals) < 2:
            return float('inf')
        diffs = [abs(sorted_vals[i+1] - sorted_vals[i]) for i in range(len(sorted_vals)-1)]
        if resolution and resolution > 0:
            return 0.6 * float(resolution)
        m = median(diffs) if diffs else 0.0
        return max(1e-6, 1.5 * m)

    def _serpentine_order_vertical_z(self, node, tf_buffer, panel_cells_centers: List[Pose], resolution: float = None) -> List[Pose]:
        """
        Orders panel cells in a serpentine pattern based on their position in arm_base frame.
        Returns list of Pose objects in MAP frame (not transformed yet).
        The actual transformation to arm_base should happen when sending goals (after column is positioned).
        """
        if not panel_cells_centers:
            return []

        # Check if transform is available before attempting lookup
        if not tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.1)):
            node.get_logger().debug(f"[{self.name}] Transform map->arm_base not yet available for ordering, waiting...")
            return []

        try:
            tf = tf_buffer.lookup_transform(
                'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
            )
        except Exception as e:
            node.get_logger().warn(f"[{self.name}] TF lookup map->arm_base failed: {e}")
            return []

        # Transform temporarily to get ordering, but return original poses in map frame
        transformed: List[PoseStamped] = []
        now = node.get_clock().now().to_msg()
        for p in panel_cells_centers:
            ps_map = PoseStamped()
            ps_map.header.frame_id = 'map'
            ps_map.header.stamp = now
            ps_map.pose = p
            ps_arm = do_transform_pose_stamped(ps_map, tf)
            ps_arm.header.frame_id = 'arm_base'
            transformed.append(ps_arm)

        X = np.array([p.pose.position.x for p in transformed], dtype=float)
        Y = np.array([p.pose.position.y for p in transformed], dtype=float)
        Z = np.array([p.pose.position.z for p in transformed], dtype=float)

        XY = np.vstack([X, Y])
        XY_centered = XY - XY.mean(axis=1, keepdims=True)
        cov = np.cov(XY_centered)
        eigvals, eigvecs = np.linalg.eigh(cov)
        u_hat = eigvecs[:, np.argmax(eigvals)]
        u_hat = u_hat / np.linalg.norm(u_hat)

        U = XY_centered.T @ u_hat

        idx = np.argsort(-Z)
        U_sorted = U[idx]
        Z_sorted = Z[idx]
        poses_sorted = [panel_cells_centers[i] for i in idx]  # Keep original map-frame poses

        thr = self._estimate_step_threshold(list(Z_sorted), resolution=resolution)

        rows_idx = [[0]]
        for i in range(1, len(Z_sorted)):
            if abs(Z_sorted[i] - Z_sorted[i-1]) > thr:
                rows_idx.append([i])
            else:
                rows_idx[-1].append(i)

        ordered = []
        for r, row in enumerate(rows_idx):
            row_items = [(U_sorted[i], poses_sorted[i]) for i in row]
            row_items.sort(key=lambda t: t[0])
            if r % 2 == 1:
                row_items.reverse()
            ordered.extend([p for _, p in row_items])

        return ordered  # Returns Pose objects in map frame
        
    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering exhaustive scanning state.")

        if self.tf_buffer is None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, node)
        
        if self.column_client is None:
            self.column_client = ActionClient(node, FollowJointTrajectory, self.column_action_name)
        
        if self.script_command_client is None:
            self.script_command_client = node.create_client(ScriptCommand, '/send_script_command')
            if not self.script_command_client.wait_for_service(timeout_sec=5.0):
                node.get_logger().error(f"[{self.name}] Script command service not available!")
                ctx["error_triggered"] = True
                return

        self.goal_pub = node.create_publisher(PoseStamped, '/arm/goal_pose', 10)
        self.goals_queue.clear()
        self.goals_by_height.clear()
        self.height_sequence.clear()
        self.current_goal_idx = -1
        self.current_height_idx = -1
        self.waiting_for_arrival = False
        self.waiting_for_column = False
        self.movement_done = False
        self.goals_initialized = False
        self._pending_panel_cells = None
        self._pending_panel_vertices = None
        self.wall_orientation = None
        self._column_stable_since = None
        self.column_target_height = None
        self.pending_service_future = None
        self.total_goals_count = 0
        ctx["error_triggered"] = False
        ctx.setdefault("base_recompute_retry_counts", {})

        res = ctx.get("wall_discretization_results")
        if not res:
            node.get_logger().error(f"[{self.name}] Missing 'wall_discretization_results' in ctx.")
            ctx["error_triggered"] = True
            return

        selected_base_idx = ctx.get("selected_base_idx")
        if selected_base_idx is None:
            node.get_logger().error(f"[{self.name}] Missing 'selected_base_idx' in ctx.")
            ctx["error_triggered"] = True
            return

        base_to_panel_indices = ctx.get("base_to_panel_indices", {})
        all_panel_cells = res.get("panel_cells_centers", [])
        all_panel_vertices_by_wall = res.get("wall_panels_vertices", [])
        
        panel_indices = base_to_panel_indices.get(selected_base_idx, [])
        panel_cells_centers = []
        panel_vertices = []
        
        for pi in panel_indices:
            panel_cells_centers.extend(all_panel_cells[pi])
            
            # Extract vertices for this panel (4 vertices per panel)
            # Need to find which wall this panel belongs to and extract its vertices
            # panel_vertices are stored as flat list: [wall0_panels, wall1_panels, ...]
            # Each wall has multiple panels, each panel has 4 vertices
            # We need to map global panel index to (wall_idx, local_panel_idx)
            panels_seen = 0
            for wall_idx, wall_verts in enumerate(all_panel_vertices_by_wall):
                num_panels_in_wall = len(wall_verts) // 4
                if panels_seen <= pi < panels_seen + num_panels_in_wall:
                    local_panel_idx = pi - panels_seen
                    # Extract 4 vertices for this panel
                    start_idx = local_panel_idx * 4
                    panel_vertices.extend(wall_verts[start_idx:start_idx + 4])
                    break
                panels_seen += num_panels_in_wall
        
        node.get_logger().info(
            f"[{self.name}] Column {selected_base_idx}: merging {len(panel_indices)} panel(s) "
            f"(indices {panel_indices}) → {len(panel_cells_centers)} total cell(s), "
            f"{len(panel_vertices)} vertices."
        )

        if not panel_cells_centers:
            node.get_logger().error(f"[{self.name}] No 'panel_cells_centers' to scan.")
            ctx["error_triggered"] = True
            return
        
        if not panel_vertices:
            node.get_logger().error(f"[{self.name}] No 'panel_vertices' found for orientation computation.")
            ctx["error_triggered"] = True
            return

        # Store cells and vertices; actual TF lookup + ordering happens in run() once the TF buffer is populated
        self._pending_panel_cells = panel_cells_centers
        self._pending_panel_vertices = panel_vertices
        node.get_logger().info(f"[{self.name}] Stored {len(panel_cells_centers)} panel cells and {len(panel_vertices)} vertices. Waiting for TF to be ready...")
    
    def _handle_service_failure(self, ctx, error_message: str):
        """
        Handle script command service failure.
        Logs error and triggers state transition to error handling.
        """
        node = ctx["node"]
        node.get_logger().error(f"[{self.name}] Script command failed: {error_message}")
        ctx["error_triggered"] = True
        self.waiting_for_arrival = False
        self.pending_service_future = None

    def _send_next_goal(self, ctx):
        """Send goal to robot via script command service (must be called AFTER column is at correct height)."""
        node = ctx["node"]

        if not self.goals_queue:
            # Queue empty - will trigger height change in run()
            return

        # Take next goal (still in MAP frame)
        next_goal_map = self.goals_queue.popleft()
        self.current_goal_idx += 1

        # Transform to arm_base NOW (after column is at correct height)
        ps_map = PoseStamped()
        ps_map.header.frame_id = 'map'
        ps_map.header.stamp = node.get_clock().now().to_msg()
        ps_map.pose = next_goal_map
        
        # Verify transform is available
        if not self.tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.5)):
            node.get_logger().error(f"[{self.name}] Transform map->arm_base not available when sending goal")
            ctx["error_triggered"] = True
            return
        
        try:
            tf = self.tf_buffer.lookup_transform(
                'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
            )
            ps_arm = do_transform_pose_stamped(ps_map, tf)
            ps_arm.header.frame_id = 'arm_base'
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Failed to transform goal to arm_base: {e}")
            ctx["error_triggered"] = True
            return
        
        # Apply wall-facing orientation
        if self.wall_orientation:
            qx, qy, qz, qw = self.wall_orientation
            ps_arm.pose.orientation.x = qx
            ps_arm.pose.orientation.y = qy
            ps_arm.pose.orientation.z = qz
            ps_arm.pose.orientation.w = qw
        else:
            qx = ps_arm.pose.orientation.x
            qy = ps_arm.pose.orientation.y
            qz = ps_arm.pose.orientation.z
            qw = ps_arm.pose.orientation.w
        
        # Convert quaternion to rotation vector for movel command
        rx, ry, rz = self._quaternion_to_rotation_vector(qx, qy, qz, qw)
        
        # Determine if this is first or last goal
        is_first_goal = (self.current_goal_idx == 0)
        # Calculate remaining goals across all heights
        remaining_in_current_queue = len(self.goals_queue)
        remaining_heights = len(self.height_sequence) - self.current_height_idx - 1
        remaining_in_future_heights = sum(
            len(self.goals_by_height[self.height_sequence[i]]) 
            for i in range(self.current_height_idx + 1, len(self.height_sequence))
        )
        total_remaining = remaining_in_current_queue + remaining_in_future_heights
        is_last_goal = (total_remaining == 0)
        
        # Prepare service request
        request = ScriptCommand.Request()
        request.command_name = 'movel'
        request.numeric_params = [
            float(ps_arm.pose.position.x),
            float(ps_arm.pose.position.y),
            float(ps_arm.pose.position.z),
            float(rx),
            float(ry),
            float(rz),
            float(self.movel_acceleration),
            float(self.movel_velocity),
            0.0,  # time parameter (0 = use velocity control)
            float(self.movel_blend_radius)
        ]
        request.string_params = []
        request.stop_program = is_first_goal
        request.restart_program = is_last_goal
        
        # Call service asynchronously
        self.pending_service_future = self.script_command_client.call_async(request)
        self.waiting_for_arrival = True
        
        node.get_logger().info(
            f"[{self.name}] Sending goal {self.current_goal_idx + 1}/{self.total_goals_count} via script command. "
            f"pos=({ps_arm.pose.position.x:.3f}, {ps_arm.pose.position.y:.3f}, {ps_arm.pose.position.z:.3f}), "
            f"rot_vec=({rx:.3f}, {ry:.3f}, {rz:.3f}). "
            f"stop_program={is_first_goal}, restart_program={is_last_goal}"
        )

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("panels_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No panels left to exhaustively scan.")
            ctx["exhaustive_scan_done"] = True
            return

        if ctx.get("error_triggered") or self.movement_done:
            return

        # Populate queue via TF lookup on first run() tick (wait for TF to be ready)
        if not self.goals_initialized:
            if self._pending_panel_cells is None or self._pending_panel_vertices is None:
                return
            
            # Wait for TF to be available before proceeding
            if not self.tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.1)):
                node.get_logger().info(f"[{self.name}] Waiting for TF map->arm_base to become available...")
                return
            
            # Compute wall normal from vertices first
            wall_normal = self._compute_wall_normal_from_vertices(
                node, self.tf_buffer, self._pending_panel_vertices, resolution=0.1
            )
            if wall_normal is None:
                # TF not ready yet or error — retry next tick
                node.get_logger().debug(f"[{self.name}] Wall normal computation failed, retrying...")
                return
            
            ordered_goals = self._serpentine_order_vertical_z(node, self.tf_buffer, self._pending_panel_cells, resolution=0.1)
            if not ordered_goals:
                # TF not ready yet — retry next tick silently
                node.get_logger().debug(f"[{self.name}] TF not ready yet, retrying...")
                return
            
            # Compute orientation where z-axis points towards the wall using vertex-based normal
            qx, qy, qz, qw = self._compute_orientation_towards_wall(wall_normal)
            node.get_logger().info(
                f"[{self.name}] Computed wall-facing orientation: "
                f"q=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
            )
            
            # Store orientation for later use
            self.wall_orientation = (qx, qy, qz, qw)
            
            # Group goals by required column height
            # Goals are still in MAP frame at this point
            for p in ordered_goals:
                # Transform to arm_base temporarily to determine required height
                ps_map = PoseStamped()
                ps_map.header.frame_id = 'map'
                ps_map.header.stamp = node.get_clock().now().to_msg()
                ps_map.pose = p
                
                try:
                    # Double-check transform is still available
                    if not self.tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.1)):
                        node.get_logger().warn(f"[{self.name}] TF temporarily unavailable during grouping")
                        goal_z = 0.5  # Default fallback
                    else:
                        tf = self.tf_buffer.lookup_transform(
                            'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
                        )
                        ps_arm = do_transform_pose_stamped(ps_map, tf)
                        goal_z = ps_arm.pose.position.z
                except Exception as e:
                    node.get_logger().warn(f"[{self.name}] TF lookup failed during grouping: {e}")
                    goal_z = 0.5  # Default fallback
                
                required_height = self._choose_column_height_for_goal_z(goal_z)
                
                # Store original map-frame pose (will transform when sending)
                if required_height not in self.goals_by_height:
                    self.goals_by_height[required_height] = deque()
                self.goals_by_height[required_height].append(p)  # Store map-frame Pose
            
            # Sort heights in DESCENDING order to start from highest (column extends first, then retracts)
            self.height_sequence = sorted(self.goals_by_height.keys(), reverse=True)
            
            # Log grouping summary
            self.total_goals_count = sum(len(q) for q in self.goals_by_height.values())
            node.get_logger().info(
                f"[{self.name}] Grouped {self.total_goals_count} goals into {len(self.height_sequence)} "
                f"column heights (highest first): {[f'{h:.2f}m' for h in self.height_sequence]}"
            )
            for h in self.height_sequence:
                node.get_logger().info(
                    f"  Height {h:.2f}m: {len(self.goals_by_height[h])} goals"
                )
            
            self.goals_initialized = True
            return  # Next tick will start processing first height group

        # Handle column movement if waiting
        if self.waiting_for_column:
            column_current = ctx.get("column_current_height", 0.0)
            if self.column_move_deadline and node.get_clock().now() > self.column_move_deadline:
                node.get_logger().error(
                    f"[{self.name}] Column movement timeout! "
                    f"Target={self.column_target_height:.3f}m, Current={column_current:.3f}m"
                )
                ctx["error_triggered"] = True
                return
            
            if self._column_reached_target(node, ctx):
                node.get_logger().info(
                    f"[{self.name}] Column reached target height {self.column_target_height:.3f}m"
                )
                self.waiting_for_column = False
                self.column_target_height = None
                self._column_stable_since = None
            else:
                # Still waiting for column
                return
        
        # Check if we need to move to next height group
        if not self.waiting_for_arrival:
            # Current height group exhausted?
            if len(self.goals_queue) == 0:
                # Move to next height
                self.current_height_idx += 1
                
                if self.current_height_idx >= len(self.height_sequence):
                    # All heights processed
                    self.movement_done = True
                    node.get_logger().info(f"[{self.name}] All scan goals completed.")
                    completed_base_indices = ctx.setdefault("completed_base_indices", [])
                    selected_base_idx = ctx.get("selected_base_idx")
                    if selected_base_idx is not None and selected_base_idx not in completed_base_indices:
                        completed_base_indices.append(selected_base_idx)
                    ctx["panels_left"] -= 1
                    if ctx.get("panels_left", 0) <= 0:
                        node.get_logger().info(f"[{self.name}] No panels left to exhaustively scan.")
                        ctx["exhaustive_scan_done"] = True
                    return
                
                # Load next height's goals
                next_height = self.height_sequence[self.current_height_idx]
                self.goals_queue = self.goals_by_height[next_height].copy()
                
                column_current = ctx.get("column_current_height", 0.0)
                node.get_logger().info(
                    f"[{self.name}] Starting height group {self.current_height_idx + 1}/{len(self.height_sequence)}: "
                    f"{next_height:.2f}m with {len(self.goals_queue)} goals"
                )
                
                # Command column to move to this height
                if abs(next_height - column_current) > self.column_tolerance_m:
                    if not self._command_column(node, ctx, next_height):
                        node.get_logger().error(f"[{self.name}] Failed to command column movement.")
                        ctx["error_triggered"] = True
                        return
                    # Wait for column before sending arm goals
                    return
                else:
                    node.get_logger().info(
                        f"[{self.name}] Column already at target height {next_height:.2f}m"
                    )
            
            # Send next arm goal from current queue
            self._send_next_goal(ctx)
            return
        
        # Check if service call is pending
        if self.pending_service_future is not None:
            if self.pending_service_future.done():
                try:
                    response = self.pending_service_future.result()
                    if response.success:
                        node.get_logger().info(
                            f"[{self.name}] Goal {self.current_goal_idx + 1} completed successfully: {response.message}"
                        )
                        self.waiting_for_arrival = False
                        self.pending_service_future = None
                    else:
                        self._handle_service_failure(ctx, response.message)
                        return
                except Exception as e:
                    self._handle_service_failure(ctx, str(e))
                    return
            else:
                # Still waiting for service response
                return

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        if self.movement_done:
            return "ArmFolding"
        return None
