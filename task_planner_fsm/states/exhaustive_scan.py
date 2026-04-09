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
from visualization_msgs.msg import Marker
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from std_msgs.msg import Float64MultiArray
from arm_control.srv import ScriptCommand

class ExhaustiveScan(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_pub = None
        self.goal_pub_ptp = None
        self.goal_pub_lin = None
        self.goal_pub_ompl = None
        self.goal_marker_pub = None
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
        self.pre_approach_goal = None
        self.pre_approach_goals = deque()
        self.pre_approach_total_stages = 0
        self.pre_approach_stage_idx = 0
        self.pre_approach_sent = False
        self.pre_approach_done = False
        self.waiting_for_pre_approach = False
        self.pre_approach_verbose = False
        
        # Post-scan retraction
        self.post_scan_delay_start = None
        self.waiting_for_post_scan_delay = False
        self.post_scan_retraction_sent = False
        self.waiting_for_post_scan_retraction = False
        
        # Script command service client
        self.script_command_client = None
        self.pending_service_future = None
        self.service_call_deadline = None
        self.service_timeout_s = 45.0  # Timeout for movel service calls
        self.total_goals_count = 0  # Track total number of goals
        self.scan_motion_backend = "auto"
        self.active_goal_backend = None
        
        # Column extension tracking
        self.column_client = None
        self.column_command_pub = None
        self.column_target_height = None
        self.column_move_deadline = None
        self.waiting_for_column = False
        self._column_stable_since = None
        self.column_control_mode = None
        
        # Goals grouped by column height
        self.goals_by_height: Dict[float, deque] = {}
        self.height_sequence = []  # Ordered list of heights to process
        self.current_height_idx = -1
        
        # Column parameters (similar to goal_router)
        self.column_min_height_m = 0.0
        self.column_max_height_m = 0.9
        self.arm_reachable_z_min = 0.0
        self.arm_reachable_z_max = 1.1
        self.column_tolerance_m = 0.01  # Increased from 0.005 to 0.01 (10mm tolerance)
        self.column_wait_timeout_s = 40.0  # Increased to 40.0 seconds to accommodate simulation timing
        self.column_move_time_s = 7.0  # Increased to 7.0 seconds (real column does 0.7m in ~5s, added margin for simulation)
        self.column_joint_name = "column_joint"
        self.column_action_name = "/column_controller/follow_joint_trajectory"
        self.column_command_topic = "/column_position_controller/commands"
        self.column_vel_tol = 0.002
        self.column_settle_time_s = 0.5  # Increased from 0.25 to 0.5 seconds for better stability
        self.column_skip_movement_threshold_m = 0.025  # Skip movements smaller than 25mm to avoid oscillation issues
        
        # Movement parameters for movel command
        self.movel_acceleration = 1.2  # m/s²
        self.movel_velocity = 0.1  # m/s
        self.movel_blend_radius = 0.0  # Stop at each point
        self.pre_approach_retract_distance_m = 0.12
        self.pre_approach_max_step_xy_m = 0.15
        self.pre_approach_max_step_z_m = 0.12
        self.pre_approach_max_rotation_step_rad = 0.25
        self.scan_goal_stride = 1
        self.skipped_lin_goals = 0
        self.post_scan_delay_s = 4.0  # Delay before final retraction
        self.post_scan_retract_distance_m = 0.15  # Final retraction distance from wall

    def _status_data(self, ctx, **extra) -> Dict:
        data = {
            "selected_base_idx": ctx.get("selected_base_idx"),
            "panels_left": ctx.get("panels_left"),
            "goals_initialized": self.goals_initialized,
            "current_goal": self.current_goal_idx + 1 if self.current_goal_idx >= 0 else 0,
            "total_goals": self.total_goals_count,
            "height_group": self.current_height_idx + 1 if self.current_height_idx >= 0 else 0,
            "height_groups": len(self.height_sequence),
            "column_target_height": self.column_target_height,
            "column_current_height": ctx.get("column_current_height"),
            "column_control_mode": self.column_control_mode,
            "waiting_for_pre_approach": self.waiting_for_pre_approach,
            "waiting_for_column": self.waiting_for_column,
            "waiting_for_arrival": self.waiting_for_arrival,
            "waiting_for_post_scan_delay": self.waiting_for_post_scan_delay,
            "waiting_for_post_scan_retraction": self.waiting_for_post_scan_retraction,
            "scan_backend": self.active_goal_backend or self.scan_motion_backend,
            "planner_backend": ctx.get("planner_backend"),
        }
        data.update(extra)
        return {key: value for key, value in data.items() if value is not None}

    def _set_status(
        self,
        ctx,
        summary: str,
        *,
        phase: str = "running",
        data: Dict = None,
        progress_current: int = None,
        progress_total: int = None,
        level: str = "info",
    ):
        setter = ctx.get("set_fsm_status")
        if callable(setter):
            setter(
                self.name,
                phase=phase,
                summary=summary,
                data=self._status_data(ctx, **(data or {})),
                progress_current=progress_current,
                progress_total=progress_total,
                level=level,
            )

    def _publish_event(self, ctx, event_type: str, summary: str, *, details: Dict = None, level: str = "info"):
        publisher = ctx.get("publish_fsm_event")
        if callable(publisher):
            publisher(
                event_type,
                state_name=self.name,
                summary=summary,
                details=self._status_data(ctx, **(details or {})),
                level=level,
            )

    def _normalize_quaternion(self, quat: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        q = np.array(quat, dtype=float)
        norm = np.linalg.norm(q)
        if norm < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        q /= norm
        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    def _nlerp_quaternion(
        self,
        q_start: Tuple[float, float, float, float],
        q_end: Tuple[float, float, float, float],
        alpha: float,
    ) -> Tuple[float, float, float, float]:
        q0 = np.array(self._normalize_quaternion(q_start), dtype=float)
        q1 = np.array(self._normalize_quaternion(q_end), dtype=float)
        if float(np.dot(q0, q1)) < 0.0:
            q1 = -q1
        q = (1.0 - alpha) * q0 + alpha * q1
        return self._normalize_quaternion((q[0], q[1], q[2], q[3]))

    def _quaternion_angular_distance(
        self,
        q_start: Tuple[float, float, float, float],
        q_end: Tuple[float, float, float, float],
    ) -> float:
        q0 = np.array(self._normalize_quaternion(q_start), dtype=float)
        q1 = np.array(self._normalize_quaternion(q_end), dtype=float)
        dot = float(np.dot(q0, q1))
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    def _resolve_scan_motion_backend(self) -> str:
        backend = str(self.scan_motion_backend).strip().lower()
        if backend in {"moveit_lin", "urscript_movel"}:
            return backend
        if self.goal_pub_lin is not None and self.goal_pub_lin.get_subscription_count() > 0:
            return "moveit_lin"
        return "urscript_movel"

    def _sample_scan_goals(self, ordered_goals: List[Pose]) -> List[Pose]:
        stride = max(1, int(self.scan_goal_stride))
        if stride <= 1:
            return ordered_goals
        return ordered_goals[::stride]

    def _publish_lin_goal_marker(self, node, pose_stamped: PoseStamped):
        if self.goal_marker_pub is None:
            return

        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id or "map"
        # Use the latest available TF in RViz instead of requiring an exact-time
        # transform for this marker message.
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "exhaustive_scan_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 0.95
        marker.lifetime = DurationMsg(sec=0, nanosec=0)
        marker.frame_locked = True
        self.goal_marker_pub.publish(marker)

    def _choose_column_height_for_goal_z(self, goal_z_in_arm_base: float) -> float:
        """
        Compute the exact column extension needed to make the goal vertically reachable.
        The column only moves upward, which lowers the goal by the same amount in arm_base.
        If the goal is already within the arm's vertical reach, return the minimum column height.
        If the goal is too high, extend just enough to bring it onto arm_reachable_z_max.
        Clamp the result to the physical column range.
        """
        required_height = max(0.0, float(goal_z_in_arm_base) - self.arm_reachable_z_max)
        return max(self.column_min_height_m, min(required_height, self.column_max_height_m))
    
    def _command_column(self, node, ctx, height_m: float):
        """
        Send column extension command using the controller configured for sim or real mode.
        """
        self._column_stable_since = None
        target_h = float(height_m)
        
        column_current = ctx.get("column_current_height", 0.0)

        if abs(target_h - column_current) <= self.column_skip_movement_threshold_m:
            self.waiting_for_column = False
            self.column_target_height = None
            self.column_move_deadline = None
            node.get_logger().info(
                f"[{self.name}] Column already close to target {target_h:.3f}m "
                f"(current={column_current:.3f}m, skipping command)."
            )
            return True

        if self.column_control_mode == "trajectory_action":
            return self._command_column_via_action(node, target_h, column_current)

        if self.column_control_mode == "position_topic":
            return self._command_column_via_topic(node, target_h, column_current)

        node.get_logger().error(
            f"[{self.name}] Column control mode not configured. Expected 'trajectory_action' or "
            f"'position_topic', got '{self.column_control_mode}'."
        )
        return False

    def _command_column_via_action(self, node, target_h: float, column_current: float) -> bool:
        if self.column_client is None:
            node.get_logger().error(f"[{self.name}] Column action client is not initialized.")
            return False

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

        # Set goal tolerances to override controller defaults
        # This prevents aborts due to tight default tolerances (e.g., 0.1m)
        goal_tol = JointTolerance()
        goal_tol.name = self.column_joint_name
        goal_tol.position = 0.015  # 15mm position tolerance at goal
        goal_tol.velocity = self.column_vel_tol  # Velocity must be small at goal
        goal.goal_tolerance = [goal_tol]

        # Set goal time tolerance (allow extra time to settle)
        goal.goal_time_tolerance = DurationMsg(sec=2, nanosec=0)

        self.column_target_height = target_h
        self.column_move_deadline = node.get_clock().now() + Duration(seconds=self.column_wait_timeout_s)
        self.waiting_for_column = True

        try:
            send_future = self.column_client.send_goal_async(goal)
            node.get_logger().info(
                f"[{self.name}] Column JTC goal sent: target={target_h:.3f}m "
                f"(current={column_current:.3f}m, action={self.column_action_name})"
            )
            return True
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Failed to send column goal: {e}")
            return False

    def _command_column_via_topic(self, node, target_h: float, column_current: float) -> bool:
        if self.column_command_pub is None:
            node.get_logger().error(f"[{self.name}] Column command publisher is not initialized.")
            return False

        cmd = Float64MultiArray()
        cmd.data = [target_h]

        try:
            self.column_command_pub.publish(cmd)
            self.column_target_height = target_h
            self.column_move_deadline = node.get_clock().now() + Duration(seconds=self.column_wait_timeout_s)
            self.waiting_for_column = True
            node.get_logger().info(
                f"[{self.name}] Column position command sent: target={target_h:.3f}m "
                f"(current={column_current:.3f}m, topic={self.column_command_topic})"
            )
            return True
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Failed to publish column position command: {e}")
            return False

    def _configure_column_control(self, node, ctx) -> None:
        use_sim = bool(ctx.get("sim", False))
        desired_mode = "trajectory_action" if use_sim else "position_topic"
        if self.column_control_mode == desired_mode:
            return

        self.column_control_mode = desired_mode

        if desired_mode == "trajectory_action":
            self.column_client = ActionClient(node, FollowJointTrajectory, self.column_action_name)
            self.column_command_pub = None
            node.get_logger().info(
                f"[{self.name}] Column control configured for simulation via {self.column_action_name}"
            )
            return

        self.column_command_pub = node.create_publisher(Float64MultiArray, self.column_command_topic, 10)
        self.column_client = None
        node.get_logger().info(
            f"[{self.name}] Column control configured for real robot via {self.column_command_topic}"
        )
    
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
            ps_map.header.frame_id = "map"
            ps_map.header.stamp = now
            ps_map.pose = v
            ps_arm = do_transform_pose_stamped(ps_map, tf)
            ps_arm.header.frame_id = "arm_base"
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

    def _build_pre_approach_goals(
        self,
        node,
        tf_buffer,
        first_scan_goal_map: Pose,
        wall_normal: np.ndarray,
        planner_backend: str,
    ) -> List[PoseStamped]:
        """
        Build pre-approach goals in arm_base frame.

        For MoveIt:
        - Retract from the first scan goal by retract_distance along wall normal
        - Send a single LIN-compatible pre-approach goal at the retracted scan-start pose

        For legacy:
        - Keep a single simpler staging pose using the generic goal topic
        """
        # Get current end-effector position from TF
        # Try common UR robot end-effector frame names (with arm_ prefix)
        ee_frame_candidates = ["arm_tool0", "arm_wrist_3_link", "arm_ee_link", "arm_flange", "tool0", "wrist_3_link", "ee_link", "flange"]
        current_ee_pos = None
        current_ee_orientation = None
        used_ee_frame = None
        
        for ee_frame in ee_frame_candidates:
            try:
                if tf_buffer.can_transform('arm_base', ee_frame, rclpy.time.Time(), Duration(seconds=0.1)):
                    tf_ee = tf_buffer.lookup_transform(
                        'arm_base', ee_frame, rclpy.time.Time(), Duration(seconds=1.0)
                    )
                    current_ee_pos = np.array([
                        tf_ee.transform.translation.x,
                        tf_ee.transform.translation.y,
                        tf_ee.transform.translation.z
                    ])
                    current_ee_orientation = (
                        tf_ee.transform.rotation.x,
                        tf_ee.transform.rotation.y,
                        tf_ee.transform.rotation.z,
                        tf_ee.transform.rotation.w,
                    )
                    used_ee_frame = ee_frame
                    node.get_logger().info(
                        f"[{self.name}] Current EE position from TF ({ee_frame}): "
                        f"({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f})"
                    )
                    break
            except Exception as e:
                continue
        
        if current_ee_pos is None:
            node.get_logger().warn(
                f"[{self.name}] Could not get current EE position from TF. "
                f"Tried frames: {ee_frame_candidates}. Using fallback approach."
            )
            # Fallback: use a safe default Z height
            current_ee_pos = np.array([0.0, 0.0, 0.5])  # Safe assumed height
            current_ee_orientation = self.wall_orientation
        
        # Transform first scan goal to arm_base frame
        ps_map = PoseStamped()
        ps_map.header.frame_id = "map"
        ps_map.header.stamp = node.get_clock().now().to_msg()
        ps_map.pose = first_scan_goal_map

        tf = tf_buffer.lookup_transform(
            'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
        )
        first_goal_arm = do_transform_pose_stamped(ps_map, tf)
        first_goal_pos = np.array([
            first_goal_arm.pose.position.x,
            first_goal_arm.pose.position.y,
            first_goal_arm.pose.position.z
        ])
        
        # Compute retreat direction (normalized wall normal pointing away from wall)
        retreat_dir = wall_normal / np.linalg.norm(wall_normal)
        retreat = float(self.pre_approach_retract_distance_m)
        
        # Pre-approach position: first_goal retracted along wall normal
        retracted_pos = first_goal_pos - retreat_dir * retreat
        
        # Preserve the current EE orientation for the staging move to avoid an
        # unnecessary IK jump before the first scan segment.
        if current_ee_orientation:
            qx, qy, qz, qw = current_ee_orientation
        elif self.wall_orientation:
            qx, qy, qz, qw = self.wall_orientation
        else:
            qx, qy, qz, qw = (0.0, 0.0, 0.0, 1.0)

        staged_goals: List[PoseStamped] = []
        now = node.get_clock().now().to_msg()

        if planner_backend == "moveit":
            target_orientation = self.wall_orientation if self.wall_orientation else (qx, qy, qz, qw)
            ps_arm = PoseStamped()
            ps_arm.header.frame_id = 'arm_base'
            ps_arm.header.stamp = now
            ps_arm.pose.position.x = float(retracted_pos[0])
            ps_arm.pose.position.y = float(retracted_pos[1])
            ps_arm.pose.position.z = float(retracted_pos[2])
            ps_arm.pose.orientation.x = target_orientation[0]
            ps_arm.pose.orientation.y = target_orientation[1]
            ps_arm.pose.orientation.z = target_orientation[2]
            ps_arm.pose.orientation.w = target_orientation[3]
            staged_goals.append(ps_arm)

            node.get_logger().info(
                f"[{self.name}] MoveIt pre-approach goal computed: "
                f"pos=({retracted_pos[0]:.3f}, {retracted_pos[1]:.3f}, {retracted_pos[2]:.3f}), "
                f"ee_frame={used_ee_frame}, retract={retreat:.3f}m"
            )
        else:
            pre_approach_pos = np.array([
                retracted_pos[0],
                retracted_pos[1],
                current_ee_pos[2],
            ])
            ps_arm = PoseStamped()
            ps_arm.header.frame_id = 'arm_base'
            ps_arm.header.stamp = now
            ps_arm.pose.position.x = float(pre_approach_pos[0])
            ps_arm.pose.position.y = float(pre_approach_pos[1])
            ps_arm.pose.position.z = float(pre_approach_pos[2])
            ps_arm.pose.orientation.x = qx
            ps_arm.pose.orientation.y = qy
            ps_arm.pose.orientation.z = qz
            ps_arm.pose.orientation.w = qw
            staged_goals.append(ps_arm)

            node.get_logger().info(
                f"[{self.name}] Legacy pre-approach goal computed: "
                f"pos=({pre_approach_pos[0]:.3f}, {pre_approach_pos[1]:.3f}, {pre_approach_pos[2]:.3f}), "
                f"ee_frame={used_ee_frame}, retract={retreat:.3f}m"
            )

        return staged_goals

    def _send_pre_approach_goal(self, ctx):
        """Send pre-approach arm goal using the same execution-status handshake used by fold/unfold."""
        node = ctx["node"]

        if not self.pre_approach_goals:
            node.get_logger().error(f"[{self.name}] Pre-approach goals were not initialized.")
            ctx["error_triggered"] = True
            return

        self.pre_approach_goal = self.pre_approach_goals[0]

        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()
        if planner_backend == "moveit":
            pre_approach_pub = self.goal_pub_ompl
            goal_topic = "/arm/goal_pose_ompl"
            goal_label = "MoveIt OMPL"
        else:
            pre_approach_pub = self.goal_pub
            goal_topic = "/arm/goal_pose"
            goal_label = "legacy planner"

        # Reset execution status before publishing a new goal.
        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        if pre_approach_pub is None or pre_approach_pub.get_subscription_count() == 0:
            node.get_logger().error(
                f"[{self.name}] No subscriber detected on {goal_topic} for the pre-approach goal."
            )
            ctx["error_triggered"] = True
            return

        pre_approach_pub.publish(self.pre_approach_goal)
        self.pre_approach_sent = True
        self.waiting_for_pre_approach = True
        self.pre_approach_verbose = False

        p = self.pre_approach_goal.pose.position
        node.get_logger().info(
            f"[{self.name}] Sent pre-approach stage {self.pre_approach_stage_idx + 1}/{self.pre_approach_total_stages} "
            f"via {goal_topic} using {goal_label}: "
            f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"retract={self.pre_approach_retract_distance_m:.3f}m"
        )

    def _prepare_pre_approach_sequence(self, ctx) -> bool:
        node = ctx["node"]

        if self.pre_approach_goals:
            return True
        if not self.goals_queue:
            node.get_logger().error(f"[{self.name}] Cannot prepare pre-approach without scan goals.")
            ctx["error_triggered"] = True
            return False
        if self.wall_orientation_normal is None:
            node.get_logger().error(f"[{self.name}] Missing wall normal for pre-approach planning.")
            ctx["error_triggered"] = True
            return False

        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()

        try:
            staged_pre_approach_goals = self._build_pre_approach_goals(
                node,
                self.tf_buffer,
                self.goals_queue[0],
                self.wall_orientation_normal,
                planner_backend,
            )
            self.pre_approach_goals = deque(staged_pre_approach_goals)
            self.pre_approach_total_stages = len(staged_pre_approach_goals)
            self.pre_approach_stage_idx = 0

            if not self.pre_approach_goals:
                raise RuntimeError("No pre-approach goals were generated")

            self.pre_approach_goal = self.pre_approach_goals[0]
            p = self.pre_approach_goals[-1].pose.position
            node.get_logger().info(
                f"[{self.name}] Pre-approach sequence prepared: "
                f"{self.pre_approach_total_stages} stage(s), final_pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
                f"retract={self.pre_approach_retract_distance_m:.3f}m"
            )
            return True
        except Exception as exc:
            node.get_logger().error(f"[{self.name}] Failed to build pre-approach goal: {exc}")
            ctx["error_triggered"] = True
            return False

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
            ps_map.header.frame_id = "map"
            ps_map.header.stamp = now
            ps_map.pose = p
            ps_arm = do_transform_pose_stamped(ps_map, tf)
            ps_arm.header.frame_id = "arm_base"
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
        
        self._configure_column_control(node, ctx)
        
        if self.script_command_client is None:
            self.script_command_client = node.create_client(ScriptCommand, '/send_script_command')

        self.goal_pub = node.create_publisher(PoseStamped, '/arm/goal_pose', 10)
        self.goal_pub_ptp = node.create_publisher(PoseStamped, '/arm/goal_pose_ptp', 10)
        self.goal_pub_lin = node.create_publisher(PoseStamped, '/arm/goal_pose_lin', 10)
        self.goal_pub_ompl = node.create_publisher(PoseStamped, '/arm/goal_pose_ompl', 10)
        self.goal_marker_pub = node.create_publisher(Marker, '/scan_goal_marker', 10)
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
        self.wall_orientation_normal = None  # Store wall normal for post-scan retraction
        self.pre_approach_goal = None
        self.pre_approach_goals.clear()
        self.pre_approach_total_stages = 0
        self.pre_approach_stage_idx = 0
        self.pre_approach_sent = False
        self.pre_approach_done = False
        self.waiting_for_pre_approach = False
        self.pre_approach_verbose = False
        self.post_scan_delay_start = None
        self.waiting_for_post_scan_delay = False
        self.post_scan_retraction_sent = False
        self.waiting_for_post_scan_retraction = False
        self._column_stable_since = None
        self.column_target_height = None
        self.pending_service_future = None
        self.service_call_deadline = None
        self.total_goals_count = 0
        self.skipped_lin_goals = 0
        self.scan_motion_backend = str(ctx.get("scan_motion_backend", "auto")).strip().lower()
        self.active_goal_backend = None
        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()
        ctx["error_triggered"] = False
        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        ctx.setdefault("base_recompute_retry_counts", {})
        self._set_status(
            ctx,
            "Preparing exhaustive scan inputs",
            phase="running",
            data={
                "planner_backend": planner_backend,
                "scan_motion_backend": self.scan_motion_backend,
            },
        )

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
        self._set_status(
            ctx,
            "Waiting for TF to initialize exhaustive scan goals",
            phase="waiting",
            data={
                "panel_indices": panel_indices,
                "panel_cells": len(panel_cells_centers),
                "panel_vertices": len(panel_vertices),
            },
        )
    
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
        self.service_call_deadline = None
        self.active_goal_backend = None
        self._set_status(
            ctx,
            f"Goal execution failed: {error_message}",
            phase="error",
            data={"error_message": error_message},
            level="error",
        )
        self._publish_event(
            ctx,
            "goal_failed",
            "Exhaustive scan goal failed",
            details={"error_message": error_message},
            level="error",
        )

    def _send_next_goal(self, ctx):
        """Send the next goal after the column reaches the required height."""
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
            node.get_logger().error(f"[{self.name}] Failed to transform goal to arm base frame: {e}")
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

        backend = self._resolve_scan_motion_backend()
        self.active_goal_backend = backend

        if backend == "moveit_lin":
            if self.goal_pub_lin is None or self.goal_pub_lin.get_subscription_count() == 0:
                node.get_logger().warn(
                    f"[{self.name}] MoveIt LIN publisher has no subscribers; falling back to URScript movel."
                )
                backend = "urscript_movel"
                self.active_goal_backend = backend
            else:
                ctx["execution_status"] = False
                ctx["planner_goal_failed"] = False
                self.pending_service_future = None
                self.waiting_for_arrival = True
                self._publish_lin_goal_marker(node, ps_map)
                self.goal_pub_lin.publish(ps_arm)
                node.get_logger().info(
                    f"[{self.name}] Sending goal {self.current_goal_idx + 1}/{self.total_goals_count} via MoveIt LIN. "
                    f"pos=({ps_arm.pose.position.x:.3f}, {ps_arm.pose.position.y:.3f}, {ps_arm.pose.position.z:.3f}), "
                    f"q=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})."
                )
                self._set_status(
                    ctx,
                    "Executing scan goal via MoveIt LIN",
                    phase="running",
                    progress_current=self.current_goal_idx + 1,
                    progress_total=self.total_goals_count,
                    data={"goal_backend": backend},
                )
                return

        # Determine if this is first or last goal
        is_first_goal = (self.current_goal_idx == 0)
        # Panel goals are never the last command - the post-scan retraction comes after
        # So all panel goals should have restart_program=False
        is_last_goal = False
        
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

        if not self.script_command_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().error(f"[{self.name}] Script command service not available!")
            ctx["error_triggered"] = True
            self.active_goal_backend = None
            return
        
        # Call service asynchronously
        self.pending_service_future = self.script_command_client.call_async(request)
        self.service_call_deadline = node.get_clock().now() + Duration(seconds=self.service_timeout_s)
        self.waiting_for_arrival = True
        
        node.get_logger().info(
            f"[{self.name}] Sending goal {self.current_goal_idx + 1}/{self.total_goals_count} via script command. "
            f"pos=({ps_arm.pose.position.x:.3f}, {ps_arm.pose.position.y:.3f}, {ps_arm.pose.position.z:.3f}), "
            f"rot_vec=({rx:.3f}, {ry:.3f}, {rz:.3f}). "
            f"stop_program={is_first_goal}, restart_program={is_last_goal}"
        )
        self._set_status(
            ctx,
            "Executing scan goal via URScript movel",
            phase="running",
            progress_current=self.current_goal_idx + 1,
            progress_total=self.total_goals_count,
            data={"goal_backend": self.active_goal_backend},
        )

    def _send_post_scan_retraction(self, ctx):
        """Send final wall retraction movel command after completing all scan goals."""
        node = ctx["node"]
        
        # Get current end-effector position from TF
        ee_frame_candidates = ["arm_tool0", "arm_wrist_3_link", "arm_ee_link", "arm_flange"]
        current_ee_pos = None
        
        for ee_frame in ee_frame_candidates:
            try:
                if self.tf_buffer.can_transform('arm_base', ee_frame, rclpy.time.Time(), Duration(seconds=0.1)):
                    tf_ee = self.tf_buffer.lookup_transform(
                        'arm_base', ee_frame, rclpy.time.Time(), Duration(seconds=1.0)
                    )
                    current_ee_pos = np.array([
                        tf_ee.transform.translation.x,
                        tf_ee.transform.translation.y,
                        tf_ee.transform.translation.z
                    ])
                    break
            except Exception:
                continue
        
        if current_ee_pos is None:
            node.get_logger().error(f"[{self.name}] Could not get current EE position for post-scan retraction.")
            ctx["error_triggered"] = True
            return
        
        # Compute retracted position along wall normal
        retreat_dir = self.wall_orientation_normal / np.linalg.norm(self.wall_orientation_normal)
        retreat = float(self.post_scan_retract_distance_m)
        
        retracted_pos = current_ee_pos - retreat_dir * retreat
        
        # Build movel command
        if self.wall_orientation:
            qx, qy, qz, qw = self.wall_orientation
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        
        rx, ry, rz = self._quaternion_to_rotation_vector(qx, qy, qz, qw)
        
        request = ScriptCommand.Request()
        request.command_name = 'movel'
        request.numeric_params = [
            float(retracted_pos[0]),
            float(retracted_pos[1]),
            float(retracted_pos[2]),
            float(rx),
            float(ry),
            float(rz),
            float(self.movel_acceleration),
            float(self.movel_velocity),
            0.0,
            float(self.movel_blend_radius)
        ]
        request.string_params = []
        request.stop_program = False
        request.restart_program = True  # Final command, restart program
        
        self.pending_service_future = self.script_command_client.call_async(request)
        self.service_call_deadline = node.get_clock().now() + Duration(seconds=self.service_timeout_s)
        self.waiting_for_post_scan_retraction = True
        self.post_scan_retraction_sent = True
        
        node.get_logger().info(
            f"[{self.name}] Sending post-scan retraction movel: "
            f"pos=({retracted_pos[0]:.3f}, {retracted_pos[1]:.3f}, {retracted_pos[2]:.3f}), "
            f"retract={retreat:.3f}m"
        )
        self._set_status(
            ctx,
            "Retracting end effector after exhaustive scan",
            phase="running",
            data={"post_scan_retract_distance_m": retreat},
        )

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("panels_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No panels left to exhaustively scan.")
            ctx["exhaustive_scan_done"] = True
            self._set_status(ctx, "No panels left to scan", phase="completed")
            return

        if ctx.get("error_triggered") or self.movement_done:
            return

        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()

        # Populate queue via TF lookup on first run() tick (wait for TF to be ready)
        if not self.goals_initialized:
            if self._pending_panel_cells is None or self._pending_panel_vertices is None:
                self._set_status(ctx, "Waiting for panel geometry from discretization", phase="waiting")
                return
            
            # Wait for TF to be available before proceeding
            if not self.tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.1)):
                node.get_logger().info(f"[{self.name}] Waiting for TF map->arm_base to become available...")
                self._set_status(ctx, "Waiting for TF map->arm_base", phase="waiting")
                return
            
            # Compute wall normal from vertices first
            wall_normal = self._compute_wall_normal_from_vertices(
                node, self.tf_buffer, self._pending_panel_vertices, resolution=0.1
            )
            if wall_normal is None:
                # TF not ready yet or error — retry next tick
                node.get_logger().debug(f"[{self.name}] Wall normal computation failed, retrying...")
                self._set_status(ctx, "Waiting for stable TF to compute wall normal", phase="waiting")
                return
            
            ordered_goals = self._serpentine_order_vertical_z(node, self.tf_buffer, self._pending_panel_cells, resolution=0.1)
            if not ordered_goals:
                # TF not ready yet — retry next tick silently
                node.get_logger().debug(f"[{self.name}] TF not ready yet, retrying...")
                self._set_status(ctx, "Waiting for TF to order scan goals", phase="waiting")
                return

            sampled_goals = self._sample_scan_goals(ordered_goals)
            if not sampled_goals:
                node.get_logger().error(f"[{self.name}] Scan goal sampling removed all goals.")
                ctx["error_triggered"] = True
                return
            if len(sampled_goals) != len(ordered_goals):
                node.get_logger().info(
                    f"[{self.name}] Sampling scan goals with stride {self.scan_goal_stride}: "
                    f"{len(ordered_goals)} -> {len(sampled_goals)} goal(s)."
                )
            ordered_goals = sampled_goals
            
            # Compute orientation where z-axis points towards the wall using vertex-based normal
            qx, qy, qz, qw = self._compute_orientation_towards_wall(wall_normal)
            node.get_logger().info(
                f"[{self.name}] Computed wall-facing orientation: "
                f"q=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
            )
            
            # Store orientation and wall normal for later use
            self.wall_orientation = (qx, qy, qz, qw)
            self.wall_orientation_normal = wall_normal  # Store for post-scan retraction

            # Group goals by required column height
            # Goals are still in MAP frame at this point
            for p in ordered_goals:
                # Transform to arm-base frame temporarily to determine required height
                ps_map = PoseStamped()
                ps_map.header.frame_id = "map"
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
                f"column target heights (highest first): {[f'{h:.3f}m' for h in self.height_sequence]}"
            )
            for h in self.height_sequence:
                node.get_logger().info(
                    f"  Height {h:.3f}m: {len(self.goals_by_height[h])} goals"
                )
            
            self.goals_initialized = True
            self._set_status(
                ctx,
                "Prepared exhaustive scan goal groups",
                phase="running",
                progress_current=0,
                progress_total=self.total_goals_count,
                data={
                    "goal_groups": len(self.height_sequence),
                    "sampled_goals": self.total_goals_count,
                },
            )
            self._publish_event(
                ctx,
                "goals_initialized",
                "Prepared exhaustive scan goal groups",
                details={"goal_groups": len(self.height_sequence), "sampled_goals": self.total_goals_count},
            )
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
                self._set_status(
                    ctx,
                    "Column movement timed out",
                    phase="error",
                    data={"column_current_height": column_current},
                    level="error",
                )
                return
            
            if self._column_reached_target(node, ctx):
                node.get_logger().info(
                    f"[{self.name}] Column reached target height {self.column_target_height:.3f}m"
                )
                self.waiting_for_column = False
                self.column_target_height = None
                self._column_stable_since = None
                self._set_status(ctx, "Column reached target height", phase="running")
            else:
                # Still waiting for column
                self._set_status(
                    ctx,
                    "Waiting for column to reach target height",
                    phase="waiting",
                    data={"column_current_height": column_current},
                )
                return
        
        # Check if we need to move to next height group
        if not self.waiting_for_arrival:
            # Current height group exhausted?
            if len(self.goals_queue) == 0:
                # Move to next height
                self.current_height_idx += 1
                
                if self.current_height_idx >= len(self.height_sequence):
                    # All heights processed - start post-scan delay before retraction
                    if not self.waiting_for_post_scan_delay and not self.post_scan_retraction_sent:
                        self.post_scan_delay_start = node.get_clock().now()
                        self.waiting_for_post_scan_delay = True
                        node.get_logger().info(
                            f"[{self.name}] All scan goals completed. "
                            f"Waiting {self.post_scan_delay_s:.1f}s before final retraction..."
                        )
                        self._set_status(
                            ctx,
                            "Waiting before post-scan retraction",
                            phase="waiting",
                            progress_current=self.total_goals_count,
                            progress_total=self.total_goals_count,
                        )
                        return
                    
                    # Check if delay has elapsed
                    if self.waiting_for_post_scan_delay:
                        elapsed = (node.get_clock().now() - self.post_scan_delay_start).nanoseconds / 1e9
                        if elapsed >= self.post_scan_delay_s:
                            self.waiting_for_post_scan_delay = False
                            node.get_logger().info(f"[{self.name}] Post-scan delay complete. Sending retraction movel...")
                            self._send_post_scan_retraction(ctx)
                            # Fall through to service call check below
                        else:
                            # Still waiting for delay
                            self._set_status(
                                ctx,
                                "Waiting before post-scan retraction",
                                phase="waiting",
                                data={"delay_elapsed_s": round(elapsed, 2), "delay_total_s": self.post_scan_delay_s},
                            )
                            return
                    
                    # If retraction sent, skip rest of goal processing and check service response below
                    if self.post_scan_retraction_sent and self.waiting_for_post_scan_retraction:
                        # Fall through to service call check at the end of run()
                        pass
                    
                    # Retraction complete - mark panel as done
                    elif self.post_scan_retraction_sent and not self.waiting_for_post_scan_retraction:
                        self.movement_done = True
                        node.get_logger().info(f"[{self.name}] Panel scan and retraction completed.")
                        completed_base_indices = ctx.setdefault("completed_base_indices", [])
                        selected_base_idx = ctx.get("selected_base_idx")
                        if selected_base_idx is not None and selected_base_idx not in completed_base_indices:
                            completed_base_indices.append(selected_base_idx)
                        ctx["panels_left"] -= 1
                        if ctx.get("panels_left", 0) <= 0:
                            node.get_logger().info(f"[{self.name}] No panels left to exhaustively scan.")
                            ctx["exhaustive_scan_done"] = True
                        self._set_status(
                            ctx,
                            "Completed exhaustive scan for selected base",
                            phase="completed",
                            progress_current=self.total_goals_count,
                            progress_total=self.total_goals_count,
                        )
                        self._publish_event(
                            ctx,
                            "panel_completed",
                            "Completed exhaustive scan for selected base",
                            details={"remaining_panels": ctx.get("panels_left", 0)},
                        )
                        return
                    
                    # If we reach here, we're in an unexpected state in post-scan phase
                    # Just fall through to service check
                else:
                    # Not yet beyond all heights - load next height's goals
                    next_height = self.height_sequence[self.current_height_idx]
                    self.goals_queue = self.goals_by_height[next_height].copy()
                    
                    column_current = ctx.get("column_current_height", 0.0)
                    node.get_logger().info(
                        f"[{self.name}] Starting height group {self.current_height_idx + 1}/{len(self.height_sequence)}: "
                        f"{next_height:.3f}m with {len(self.goals_queue)} goals"
                    )
                    
                    # Command column to move to this height
                    # Use threshold to avoid commanding tiny movements that may oscillate
                    height_diff = abs(next_height - column_current)
                    if height_diff > self.column_skip_movement_threshold_m:
                        if not self._command_column(node, ctx, next_height):
                            node.get_logger().error(f"[{self.name}] Failed to command column movement.")
                            ctx["error_triggered"] = True
                            return
                        self._set_status(
                            ctx,
                            "Commanded column movement for next goal group",
                            phase="running",
                            data={"next_height": next_height, "goals_in_group": len(self.goals_queue)},
                        )
                        # Wait for column before sending arm goals
                        return
                    else:
                        # Column is close enough - clear any pending column wait state
                        self.waiting_for_column = False
                        self.column_target_height = None
                        self.column_move_deadline = None
                        self._column_stable_since = None
                        node.get_logger().info(
                            f"[{self.name}] Column already at target height {next_height:.3f}m "
                            f"(diff={height_diff*1000:.1f}mm, skipping movement)"
                        )
                        self._set_status(
                            ctx,
                            "Starting next height group",
                            phase="running",
                            data={"next_height": next_height, "goals_in_group": len(self.goals_queue)},
                        )

            if not self.pre_approach_done:
                if not self._prepare_pre_approach_sequence(ctx):
                    return

                if not self.pre_approach_sent:
                    self._set_status(ctx, "Sending pre-approach sequence", phase="running")
                    self._send_pre_approach_goal(ctx)
                    return

                if self.waiting_for_pre_approach:
                    if ctx.get("planner_goal_failed"):
                        node.get_logger().error(
                            f"[{self.name}] Pre-approach stage {self.pre_approach_stage_idx + 1}/{self.pre_approach_total_stages} failed."
                        )
                        ctx["planner_goal_failed"] = False
                        self.waiting_for_pre_approach = False
                        ctx["error_triggered"] = True
                        self._set_status(
                            ctx,
                            "Pre-approach stage failed",
                            phase="error",
                            level="error",
                        )
                        return

                    exec_status = ctx.get("execution_status")
                    if exec_status is True:
                        self.pre_approach_sent = False
                        self.waiting_for_pre_approach = False
                        self.pre_approach_verbose = False
                        ctx["execution_status"] = False
                        self.pre_approach_stage_idx += 1
                        if self.pre_approach_goals:
                            self.pre_approach_goals.popleft()

                        if not self.pre_approach_goals:
                            self.pre_approach_done = True
                            node.get_logger().info(
                                f"[{self.name}] Final pre-approach stage reached. Starting exhaustive scan sequence."
                            )
                            self._set_status(ctx, "Pre-approach complete", phase="running")
                        else:
                            node.get_logger().info(
                                f"[{self.name}] Pre-approach stage {self.pre_approach_stage_idx}/{self.pre_approach_total_stages} reached. "
                                f"Continuing staged approach."
                            )
                            self._set_status(
                                ctx,
                                "Continuing staged pre-approach",
                                phase="running",
                                progress_current=self.pre_approach_stage_idx,
                                progress_total=self.pre_approach_total_stages,
                            )
                        return

                    if not self.pre_approach_verbose:
                        node.get_logger().info(
                            f"[{self.name}] Waiting for pre-approach stage {self.pre_approach_stage_idx + 1}/{self.pre_approach_total_stages} completion..."
                        )
                        self.pre_approach_verbose = True
                    self._set_status(
                        ctx,
                        "Waiting for pre-approach completion",
                        phase="waiting",
                        progress_current=self.pre_approach_stage_idx + 1,
                        progress_total=self.pre_approach_total_stages,
                    )
                    return
            else:
                # Current height still has goals - send next one
                self._send_next_goal(ctx)
            
            # Don't return if waiting for retraction - let it fall through to service check
            if not (self.post_scan_retraction_sent and self.waiting_for_post_scan_retraction):
                return
        
        if self.active_goal_backend == "moveit_lin":
            if ctx.get("planner_goal_failed"):
                self.skipped_lin_goals += 1
                node.get_logger().warn(
                    f"[{self.name}] MoveIt LIN goal {self.current_goal_idx + 1}/{self.total_goals_count} failed. "
                    f"Skipping this scan point and continuing "
                    f"(skipped {self.skipped_lin_goals} total)."
                )
                ctx["planner_goal_failed"] = False
                self.waiting_for_arrival = False
                self.active_goal_backend = None
                self._set_status(
                    ctx,
                    "MoveIt LIN goal failed, skipping scan point",
                    phase="warning",
                    progress_current=self.current_goal_idx + 1,
                    progress_total=self.total_goals_count,
                    data={"skipped_lin_goals": self.skipped_lin_goals},
                    level="warn",
                )
                return

            if ctx.get("execution_status") is True:
                node.get_logger().info(
                    f"[{self.name}] Goal {self.current_goal_idx + 1} completed successfully via MoveIt LIN."
                )
                ctx["execution_status"] = False
                ctx["planner_goal_failed"] = False
                self.waiting_for_arrival = False
                self.active_goal_backend = None
                self._set_status(
                    ctx,
                    "Scan goal completed successfully via MoveIt LIN",
                    phase="running",
                    progress_current=self.current_goal_idx + 1,
                    progress_total=self.total_goals_count,
                )
                return

            self._set_status(
                ctx,
                "Waiting for MoveIt LIN goal completion",
                phase="waiting",
                progress_current=self.current_goal_idx + 1,
                progress_total=self.total_goals_count,
            )
            return

        # Check if service call is pending
        if self.pending_service_future is not None:
            # Check for timeout
            if self.service_call_deadline and node.get_clock().now() > self.service_call_deadline:
                node.get_logger().error(
                    f"[{self.name}] Service call timeout! No response received within {self.service_timeout_s}s"
                )
                self.pending_service_future = None
                self.service_call_deadline = None
                ctx["error_triggered"] = True
                self._set_status(
                    ctx,
                    "Service call timed out",
                    phase="error",
                    data={"service_timeout_s": self.service_timeout_s},
                    level="error",
                )
                return
            
            if self.pending_service_future.done():
                try:
                    response = self.pending_service_future.result()
                    if response.success:
                        if self.waiting_for_post_scan_retraction:
                            node.get_logger().info(
                                f"[{self.name}] Post-scan retraction completed successfully: {response.message}"
                            )
                            self.waiting_for_post_scan_retraction = False
                            self.pending_service_future = None
                            self.service_call_deadline = None
                            self._set_status(ctx, "Post-scan retraction complete", phase="running")
                        else:
                            node.get_logger().info(
                                f"[{self.name}] Goal {self.current_goal_idx + 1} completed successfully: {response.message}"
                            )
                            self.waiting_for_arrival = False
                            self.pending_service_future = None
                            self.service_call_deadline = None
                            self._set_status(
                                ctx,
                                "Scan goal completed successfully",
                                phase="running",
                                progress_current=self.current_goal_idx + 1,
                                progress_total=self.total_goals_count,
                                data={"response_message": response.message},
                            )
                        node.get_logger().info(
                            f"[{self.name}] Goal {self.current_goal_idx + 1} completed successfully: {response.message}"
                        )
                        self.waiting_for_arrival = False
                        self.pending_service_future = None
                        self.active_goal_backend = None
                    else:
                        self._handle_service_failure(ctx, response.message)
                        return
                except Exception as e:
                    self._handle_service_failure(ctx, str(e))
                    return
            else:
                # Still waiting for service response
                self._set_status(
                    ctx,
                    "Waiting for goal service response",
                    phase="waiting",
                    progress_current=self.current_goal_idx + 1,
                    progress_total=self.total_goals_count,
                )
                return

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        if self.movement_done:
            return "ArmFolding"
        return None
