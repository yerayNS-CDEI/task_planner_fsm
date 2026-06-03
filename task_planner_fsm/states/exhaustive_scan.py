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
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (
    RobotState,
)

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
        self.pre_approach_planner_sequence = deque()  # planner per stage: 'ompl' or 'ptp'
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
        self._post_scan_retraction_pose: PoseStamped = None
        self._post_scan_retraction_ptp_tried = False
        self._post_scan_retraction_ompl_tried = False
        
        self.total_goals_count = 0
        self.scan_motion_backend = "moveit_lin"
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
        
        self.pre_approach_retract_distance_m = 0.12
        # How far below the first scan row the OMPL pre-approach target is placed.
        # Must be large enough to escape the self-collision zone: arm_forearm_link hits
        # arm_plate_link when z > ~0.85m in arm_base with small horizontal reach.
        # 0.6m offset → pre-approach at z≈0.5m (46% of UR10e reach), well clear of plate.
        self.pre_approach_height_offset_m = 0.6
        self.pre_approach_max_step_xy_m = 0.15
        self.pre_approach_max_step_z_m = 0.12
        self.pre_approach_max_rotation_step_rad = 0.25
        self.scan_goal_stride = 1
        self.skipped_lin_goals = 0
        self.lin_goal_retry_count = 0
        self.lin_goal_max_retries = 3
        self._inflight_lin_goal_arm: PoseStamped = None
        self._inflight_lin_goal_map: PoseStamped = None
        self.lin_subscriber_wait_deadline = None
        self.lin_subscriber_wait_timeout_s = 15.0
        # Distance the EE is kept in front of the wall surface for scan goals.
        # Wall discretization places goals ON the surface; MoveIt requires a collision-free
        # standoff so the arm never reaches inside the wall geometry.
        self.scan_standoff_distance_m = 0.20
        self.post_scan_delay_s = 4.0  # Delay before final retraction
        self.post_scan_retract_distance_m = 0.15  # Final retraction distance from wall

        # Pre-approach candidate search:
        # Some walls (especially behind the robot) have no collision-free IK at the
        # default pre-approach pose because random IK at the goal hits sensor mounts.
        # We pre-screen multiple candidate (z-offset, tilt-about-wall-normal) pairs
        # via compute_ik before sending any goal to the planner.  The first candidate
        # that returns a valid (collision-free) IK solution is used.  If all fail we
        # mark the current column-height group as unreachable and continue with the
        # next group rather than aborting the panel.
        self.pre_approach_candidate_offsets_m = [0.6, 0.8, 0.4, 1.0, 0.2, 0.9, 0.3]
        self.pre_approach_candidate_tilts_rad = [
            0.0,
            math.radians(20.0),
            math.radians(-20.0),
            math.radians(45.0),
            math.radians(-45.0),
        ]
        # IK feasibility check (per-candidate) parameters
        self.pre_approach_ik_timeout_s = 0.5      # compute_ik service timeout
        self.pre_approach_ik_call_timeout_s = 2.0  # max wait for future before giving up

        # Service client + joint state cache (initialised in on_enter)
        self.compute_ik_client = None
        self.current_joint_state: JointState = None
        self.joint_state_sub = None

        # One-shot scan-goal reachability pre-check (runs before 35-candidate loop)
        self.scan_goal_precheck_done: bool = False
        self.scan_goal_precheck_pending: bool = False
        self.scan_goal_precheck_future = None
        self.scan_goal_precheck_deadline = None

        # Pre-approach candidate search state
        self.pre_approach_candidates: deque = deque()
        self.pre_approach_search_initialized = False
        self.pre_approach_search_failed = False
        self.pre_approach_ik_pending = False
        self.pre_approach_ik_future = None
        self.pre_approach_ik_current_candidate = None
        self.pre_approach_ik_deadline = None
        # Two-phase IK validation memo (per-orientation):
        #   'scan' phase tests IK at the first scan-row position with the candidate
        #         orientation.  If invalid, all z-offsets for that orientation
        #         are skipped — no point checking the pre-approach pose when LIN
        #         can't possibly land its goal.
        #   'pre'  phase tests IK at the pre-approach offset position.  Per
        #         z-offset.
        # Phase order: 'pre' fires first (seeded by current joint state); on success its
        # result joint state is captured and used to seed the 'scan' IK check.  This
        # ensures Pilz LIN, which starts from the post-pre-approach joint state, finds
        # an IK solution in the same family as the pre-approach — eliminating the
        # IK-family mismatch that caused systematic -31 failures.
        self.pre_approach_ik_phase = 'pre'  # current phase of the in-flight call
        self.pre_approach_passed_orient_indices: set = set()   # kept for compat, unused
        self.pre_approach_failed_orient_indices: set = set()   # kept for compat, unused
        self.unreachable_heights = set()

        # --- Goal IK pre-screening (runs before any sequence is sent) ---
        # Each goal in the height group is async-validated via compute_ik using the

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

    def _sample_scan_goals(self, ordered_goals: List[Pose]) -> List[Pose]:
        stride = max(1, int(self.scan_goal_stride))
        if stride <= 1:
            return ordered_goals
        return ordered_goals[::stride]

    def _reduce_to_row_endpoints(self, ordered_goals: List[Pose], resolution: float = 0.1) -> List[Pose]:
        """Return only the first and last goal of each horizontal Z-row.

        Goals arrive in serpentine order (alternating direction per row).
        Two consecutive Pose objects belong to the same row when their Z values
        differ by less than *resolution*.  Rows with a single goal are kept as-is.
        """
        if not ordered_goals:
            return ordered_goals

        endpoints: List[Pose] = []
        row_start = ordered_goals[0]
        row_last  = ordered_goals[0]

        for pose in ordered_goals[1:]:
            if abs(pose.position.z - row_start.position.z) < resolution:
                row_last = pose
            else:
                # Flush current row
                endpoints.append(row_start)
                if row_last is not row_start:
                    endpoints.append(row_last)
                row_start = pose
                row_last  = pose

        # Flush final row
        endpoints.append(row_start)
        if row_last is not row_start:
            endpoints.append(row_last)

        return endpoints

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

    def _choose_column_height_for_goal_z(self, goal_z_world_relative: float) -> float:
        """
        Compute the column extension needed to make the goal vertically reachable.
        goal_z_world_relative is the goal z measured from the robot base (world-relative),
        independent of the current column position. The column raises the arm_base, so each
        metre of column extension lowers the goal by one metre in arm_base z.
        Returns the minimum required column height, clamped to the physical range.
        """
        required_height = max(0.0, float(goal_z_world_relative) - self.arm_reachable_z_max)
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

        # Position tolerance at JTC level (coarse — FSM _column_reached_target does the
        # tight 10 mm stability check).  Velocity set to 0.0 to disable the JTC velocity
        # check at goal time: the simulation column often arrives with residual velocity
        # that triggers an abort even though the position is correct.
        goal_tol = JointTolerance()
        goal_tol.name = self.column_joint_name
        goal_tol.position = 0.05   # 50 mm JTC-level tolerance; FSM uses 10 mm
        goal_tol.velocity = 0.0    # 0.0 = disabled; FSM handles settling independently
        goal.goal_tolerance = [goal_tol]

        # Generous extra window so the action succeeds rather than aborting early.
        goal.goal_time_tolerance = DurationMsg(sec=10, nanosec=0)

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

    # ---------------- Pre-approach candidate IK pre-screening ----------------

    def _joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def _axis_angle_to_quaternion(
        self, axis: np.ndarray, angle: float
    ) -> Tuple[float, float, float, float]:
        n = np.linalg.norm(axis)
        if n < 1e-9 or abs(angle) < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        unit = axis / n
        half = 0.5 * float(angle)
        s = math.sin(half)
        return (
            float(unit[0] * s),
            float(unit[1] * s),
            float(unit[2] * s),
            float(math.cos(half)),
        )

    def _quaternion_multiply(
        self,
        q1: Tuple[float, float, float, float],
        q2: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    def _build_pose_stamped_arm_base(
        self,
        node,
        pos: Tuple[float, float, float],
        q: Tuple[float, float, float, float],
    ) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'arm_base'
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x = float(pos[0])
        ps.pose.position.y = float(pos[1])
        ps.pose.position.z = float(pos[2])
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])
        return ps

    def _generate_pre_approach_candidates(
        self,
        node,
        effective_first_pos: np.ndarray,
        wall_normal: np.ndarray,
    ) -> deque:
        """
        Build a list of candidate dicts, each containing BOTH a pre-approach pose
        and the first-scan pose with the same orientation.

        The pre-approach must reach the offset position; the first LIN scan goal
        must be IK-valid at the scan row z with the SAME orientation (otherwise
        LIN SLERPs orientation between two IK families and fails with -31).  Both
        poses are pre-validated by compute_ik before sending any goal to the
        planner.

        Candidates vary by:
          - tilt rotation about the wall normal axis (outer loop) — preserves the
            EE z-axis (approach direction) so the scan footprint is unchanged
          - z offset below the scan row (inner loop) — only the pre-approach pose
            uses this offset; the scan pose stays at the first scan row z
        """
        if self.wall_orientation:
            q_wall = tuple(self.wall_orientation)
        else:
            q_wall = (0.0, 0.0, 0.0, 1.0)

        candidates = deque()

        for orient_idx, tilt in enumerate(self.pre_approach_candidate_tilts_rad):
            if abs(tilt) > 1e-9:
                q_tilt = self._axis_angle_to_quaternion(wall_normal, tilt)
                q_candidate = self._quaternion_multiply(q_tilt, q_wall)
            else:
                q_candidate = q_wall
            q_candidate = (
                float(q_candidate[0]),
                float(q_candidate[1]),
                float(q_candidate[2]),
                float(q_candidate[3]),
            )

            # Scan pose: at the first scan row z, candidate orientation.
            # Independent of z_offset so it's shared across all candidates with
            # the same orientation.
            scan_pose = self._build_pose_stamped_arm_base(
                node,
                (effective_first_pos[0], effective_first_pos[1], effective_first_pos[2]),
                q_candidate,
            )

            for z_off in self.pre_approach_candidate_offsets_m:
                pre_z = max(0.15, float(effective_first_pos[2]) - float(z_off))
                pre_pose = self._build_pose_stamped_arm_base(
                    node,
                    (effective_first_pos[0], effective_first_pos[1], pre_z),
                    q_candidate,
                )
                description = (
                    f"orient_idx={orient_idx} (tilt={math.degrees(tilt):+.0f}°), "
                    f"z_off={z_off:.2f}m, pre_z={pre_z:.3f}m"
                )
                candidates.append({
                    'orient_idx': orient_idx,
                    'orientation': q_candidate,
                    'pre_pose': pre_pose,
                    'scan_pose': scan_pose,
                    'description': description,
                    'planner': 'ompl',
                })
        return candidates

    def _start_ik_request(self, node, candidate: Dict, phase: str, seed_joint_state=None) -> bool:
        """
        Fire an async compute_ik request for `candidate` in the given `phase`.

        phase='pre'  validates the pre-approach pose (seeded with current joint state).
        phase='scan' validates the first-scan row pose (seeded with the pre-approach IK
                     result joint state so Pilz LIN starts in the same IK family).

        seed_joint_state overrides self.current_joint_state when provided.

        Returns True on dispatch, False if the service isn't ready.
        """
        if phase == 'scan':
            pose = candidate['scan_pose']
        elif phase == 'pre':
            pose = candidate['pre_pose']
        else:
            node.get_logger().error(f"[{self.name}] Unknown IK phase '{phase}'.")
            return False

        if self.compute_ik_client is None or not self.compute_ik_client.service_is_ready():
            node.get_logger().warn(
                f"[{self.name}] compute_ik service not ready; "
                f"skipping candidate '{candidate['description']}' (phase={phase})."
            )
            return False

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm_manipulator'
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = DurationMsg(
            sec=int(self.pre_approach_ik_timeout_s),
            nanosec=int((self.pre_approach_ik_timeout_s % 1.0) * 1e9),
        )
        seed = seed_joint_state if seed_joint_state is not None else self.current_joint_state
        if seed is not None:
            robot_state = RobotState()
            robot_state.joint_state = seed
            robot_state.is_diff = False
            request.ik_request.robot_state = robot_state

        self.pre_approach_ik_future = self.compute_ik_client.call_async(request)
        self.pre_approach_ik_pending = True
        self.pre_approach_ik_current_candidate = candidate
        self.pre_approach_ik_phase = phase
        self.pre_approach_ik_deadline = node.get_clock().now() + Duration(
            seconds=self.pre_approach_ik_call_timeout_s
        )
        return True

    def _build_pre_approach_goals(
        self,
        node,
        tf_buffer,
        first_scan_goal_map: Pose,
        wall_normal: np.ndarray,
        planner_backend: str,
    ) -> List[Tuple[PoseStamped, str]]:
        """
        Build pre-approach stages as (PoseStamped, planner_type) pairs.

        For MoveIt — single OMPL stage using an "approach from below" strategy:
          The pre-approach is placed at the same x,y as the first scan goal (at standoff
          distance from wall), but pre_approach_height_offset_m BELOW it. This avoids
          the backward-retraction position that sits close to the arm base at high z,
          where all IK solutions self-collide (arm_forearm_link hits arm_plate_link).
          Wall orientation is used directly so LIN can start without a reorientation
          stage: the first LIN move raises the arm straight up to the first scan row.

        For legacy: single generic pose goal.
        """
        # Transform first scan goal to arm_base frame
        ps_map = PoseStamped()
        ps_map.header.frame_id = "map"
        ps_map.header.stamp = node.get_clock().now().to_msg()
        ps_map.pose = first_scan_goal_map
        tf = tf_buffer.lookup_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0))
        first_goal_arm = do_transform_pose_stamped(ps_map, tf)
        first_goal_pos = np.array([
            first_goal_arm.pose.position.x,
            first_goal_arm.pose.position.y,
            first_goal_arm.pose.position.z,
        ])

        retreat_dir = wall_normal / np.linalg.norm(wall_normal)

        # Effective first scan position: standoff distance in front of wall surface.
        effective_first_pos = first_goal_pos - retreat_dir * self.scan_standoff_distance_m

        now = node.get_clock().now().to_msg()
        staged_goals: List[Tuple[PoseStamped, str]] = []

        if planner_backend == "moveit":
            # Approach from below: same x,y as first scan goal at standoff distance,
            # z lowered by pre_approach_height_offset_m.
            # Rationale: backward retraction (pulling arm toward base) at high z puts
            # the arm in a near-vertical configuration where ALL IK solutions cause
            # arm_forearm_link to collide with arm_plate_link → "unable to sample goal tree".
            # Staying at the scan goal's horizontal distance from base while lowering z
            # keeps the arm in a comfortable, non-self-colliding configuration.
            # Wall orientation is used directly so no PTP reorientation stage is needed
            # before LIN: the first LIN move is a clean vertical rise to the first scan row.
            height_offset = float(self.pre_approach_height_offset_m)
            # Floor at 0.25m: keeps arm above base plate even if first scan row is very low.
            pre_approach_z = max(0.25, float(effective_first_pos[2]) - height_offset)
            pre_approach_pos = np.array([
                effective_first_pos[0],
                effective_first_pos[1],
                pre_approach_z,
            ])

            if self.wall_orientation:
                qx, qy, qz, qw = self.wall_orientation
            else:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

            ps1 = PoseStamped()
            ps1.header.frame_id = 'arm_base'
            ps1.header.stamp = now
            ps1.pose.position.x = float(pre_approach_pos[0])
            ps1.pose.position.y = float(pre_approach_pos[1])
            ps1.pose.position.z = float(pre_approach_pos[2])
            ps1.pose.orientation.x = qx
            ps1.pose.orientation.y = qy
            ps1.pose.orientation.z = qz
            ps1.pose.orientation.w = qw
            staged_goals.append((ps1, 'ompl'))

            node.get_logger().info(
                f"[{self.name}] Pre-approach (approach-from-below): OMPL to "
                f"pos=({pre_approach_pos[0]:.3f}, {pre_approach_pos[1]:.3f}, {pre_approach_pos[2]:.3f}), "
                f"wall_orient q=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}), "
                f"height_offset={height_offset:.3f}m below first scan row z={effective_first_pos[2]:.3f}m"
            )
        else:
            # Legacy: approach with horizontal retraction from first scan position
            ee_frame_candidates = ["arm_tool0", "arm_wrist_3_link", "arm_ee_link",
                                    "arm_flange", "tool0", "wrist_3_link", "ee_link", "flange"]
            current_ee_pos = np.array([0.0, 0.0, 0.5])
            current_ee_orientation = (0.0, 0.0, 0.0, 1.0)
            for ee_frame in ee_frame_candidates:
                try:
                    if tf_buffer.can_transform('arm_base', ee_frame, rclpy.time.Time(), Duration(seconds=0.1)):
                        tf_ee = tf_buffer.lookup_transform(
                            'arm_base', ee_frame, rclpy.time.Time(), Duration(seconds=1.0)
                        )
                        current_ee_pos = np.array([
                            tf_ee.transform.translation.x,
                            tf_ee.transform.translation.y,
                            tf_ee.transform.translation.z,
                        ])
                        current_ee_orientation = (
                            tf_ee.transform.rotation.x,
                            tf_ee.transform.rotation.y,
                            tf_ee.transform.rotation.z,
                            tf_ee.transform.rotation.w,
                        )
                        break
                except Exception:
                    continue

            retracted_pos = effective_first_pos - retreat_dir * float(self.pre_approach_retract_distance_m)
            qx, qy, qz, qw = current_ee_orientation
            ps_arm = PoseStamped()
            ps_arm.header.frame_id = 'arm_base'
            ps_arm.header.stamp = now
            ps_arm.pose.position.x = float(retracted_pos[0])
            ps_arm.pose.position.y = float(retracted_pos[1])
            ps_arm.pose.position.z = float(current_ee_pos[2])
            ps_arm.pose.orientation.x = qx
            ps_arm.pose.orientation.y = qy
            ps_arm.pose.orientation.z = qz
            ps_arm.pose.orientation.w = qw
            staged_goals.append((ps_arm, 'legacy'))
            node.get_logger().info(
                f"[{self.name}] Pre-approach goal computed (legacy): "
                f"pos=({retracted_pos[0]:.3f}, {retracted_pos[1]:.3f}, {current_ee_pos[2]:.3f}m)"
            )

        return staged_goals

    def _send_pre_approach_goal(self, ctx):
        """Send the current pre-approach stage, selecting planner from the per-stage sequence."""
        node = ctx["node"]

        if not self.pre_approach_goals:
            node.get_logger().error(f"[{self.name}] Pre-approach goals were not initialized.")
            ctx["error_triggered"] = True
            return

        self.pre_approach_goal = self.pre_approach_goals[0]
        planner_type = self.pre_approach_planner_sequence[0] if self.pre_approach_planner_sequence else 'ompl'

        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()
        if planner_backend == "moveit":
            if planner_type == 'ptp':
                pre_approach_pub = self.goal_pub_ptp
                goal_topic = "/arm/goal_pose_ptp"
                goal_label = "MoveIt PTP"
            else:
                # OMPL (APS): collision-aware free-space move to approach-from-below
                # position with wall orientation. Wall orientation is used directly so
                # LIN can start without a separate reorientation stage.
                pre_approach_pub = self.goal_pub_ompl
                goal_topic = "/arm/goal_pose_ompl"
                goal_label = "MoveIt OMPL-APS"
        else:
            pre_approach_pub = self.goal_pub
            goal_topic = "/arm/goal_pose"
            goal_label = "legacy planner"

        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        if pre_approach_pub is None or pre_approach_pub.get_subscription_count() == 0:
            node.get_logger().error(
                f"[{self.name}] No subscriber on {goal_topic} for pre-approach stage "
                f"{self.pre_approach_stage_idx + 1}/{self.pre_approach_total_stages}."
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
            f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )

    def _prepare_pre_approach_sequence(self, ctx) -> bool:
        """
        Drive the per-tick pre-approach candidate search.

        Returns True when self.pre_approach_goals is populated with a validated
        candidate (IK confirmed collision-free).  Returns False otherwise — caller
        should check self.pre_approach_search_failed (search exhausted, current
        height-group unreachable) and ctx["error_triggered"] (fatal misconfig).
        """
        node = ctx["node"]

        # Already have a validated candidate queued.
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

        # Legacy backend: keep original single-shot behaviour (no IK pre-screening).
        if planner_backend != 'moveit':
            try:
                staged = self._build_pre_approach_goals(
                    node, self.tf_buffer, self.goals_queue[0],
                    self.wall_orientation_normal, planner_backend,
                )
                if not staged:
                    raise RuntimeError("No pre-approach goals were generated")
                self.pre_approach_goals = deque([g for g, _ in staged])
                self.pre_approach_planner_sequence = deque([p for _, p in staged])
                self.pre_approach_total_stages = len(staged)
                self.pre_approach_stage_idx = 0
                self.pre_approach_goal = self.pre_approach_goals[0]
                return True
            except Exception as exc:
                node.get_logger().error(f"[{self.name}] Failed to build pre-approach goal: {exc}")
                ctx["error_triggered"] = True
                return False

        # MoveIt: candidate search with IK pre-screening.
        # ----- Step 0: one-shot scan-goal reachability pre-check -----
        # Check whether the first scan goal is reachable at all before spending
        # ~70 s cycling all 35 pre-approach candidates.  If compute_ik returns
        # NO_IK_SOLUTION with a neutral seed, the position is outside the
        # workspace — bail in <0.5 s instead of timing out 35 times.
        if not self.scan_goal_precheck_done:
            if self.scan_goal_precheck_pending and self.scan_goal_precheck_future is not None:
                now = node.get_clock().now()
                timed_out = (
                    self.scan_goal_precheck_deadline is not None
                    and now > self.scan_goal_precheck_deadline
                )
                if timed_out or self.scan_goal_precheck_future.done():
                    ok = False
                    if not timed_out:
                        try:
                            resp = self.scan_goal_precheck_future.result()
                            ok = resp is not None and resp.error_code.val == resp.error_code.SUCCESS
                            err_val = resp.error_code.val if resp is not None else 'none'
                        except Exception:
                            err_val = 'exception'
                    else:
                        err_val = 'timeout'
                    self.scan_goal_precheck_pending = False
                    self.scan_goal_precheck_future = None
                    if ok:
                        node.get_logger().info(
                            f"[{self.name}] Scan-goal pre-check PASSED — goal is reachable. "
                            f"Proceeding with candidate search."
                        )
                        self.scan_goal_precheck_done = True
                        # fall through to Step 1
                    else:
                        node.get_logger().warn(
                            f"[{self.name}] Scan-goal pre-check FAILED (err={err_val}): "
                            f"first scan goal is outside workspace. Skipping all "
                            f"{self._pre_approach_candidate_count_total()} candidates."
                        )
                        self.pre_approach_search_failed = True
                        return False
                else:
                    return False  # future still in-flight
            elif not self.scan_goal_precheck_pending:
                # Fire IK for the first scan goal with neutral (current joint state) seed.
                try:
                    ps_map = PoseStamped()
                    ps_map.header.frame_id = "map"
                    ps_map.header.stamp = node.get_clock().now().to_msg()
                    ps_map.pose = self.goals_queue[0]
                    tf = self.tf_buffer.lookup_transform(
                        'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
                    )
                    scan_goal_arm = do_transform_pose_stamped(ps_map, tf)
                    # Pull standoff (same transform as _send_next_goal)
                    if self.wall_orientation_normal is not None and self.scan_standoff_distance_m > 0.0:
                        n = self.wall_orientation_normal / np.linalg.norm(self.wall_orientation_normal)
                        scan_goal_arm.pose.position.x -= float(n[0]) * self.scan_standoff_distance_m
                        scan_goal_arm.pose.position.y -= float(n[1]) * self.scan_standoff_distance_m
                        scan_goal_arm.pose.position.z -= float(n[2]) * self.scan_standoff_distance_m
                    qx, qy, qz, qw = self.wall_orientation
                    scan_goal_arm.pose.orientation.x = qx
                    scan_goal_arm.pose.orientation.y = qy
                    scan_goal_arm.pose.orientation.z = qz
                    scan_goal_arm.pose.orientation.w = qw

                    request = GetPositionIK.Request()
                    request.ik_request.group_name = 'arm_manipulator'
                    request.ik_request.pose_stamped = scan_goal_arm
                    request.ik_request.timeout = DurationMsg(
                        sec=int(self.pre_approach_ik_timeout_s),
                        nanosec=int((self.pre_approach_ik_timeout_s % 1.0) * 1e9),
                    )
                    if self.current_joint_state is not None:
                        robot_state = RobotState()
                        robot_state.joint_state = self.current_joint_state
                        robot_state.is_diff = False
                        request.ik_request.robot_state = robot_state

                    self.scan_goal_precheck_future = self.compute_ik_client.call_async(request)
                    self.scan_goal_precheck_pending = True
                    self.scan_goal_precheck_deadline = node.get_clock().now() + Duration(
                        seconds=self.pre_approach_ik_call_timeout_s
                    )
                    node.get_logger().info(
                        f"[{self.name}] Firing scan-goal reachability pre-check "
                        f"(z_arm={scan_goal_arm.pose.position.z:.3f}m) before candidate search."
                    )
                except Exception as exc:
                    node.get_logger().warn(
                        f"[{self.name}] Scan-goal pre-check setup failed ({exc}); skipping pre-check."
                    )
                    self.scan_goal_precheck_done = True  # skip and proceed to candidate search
                return False  # wait for next tick

        # ----- Step 1: initialise the candidate list if not done yet -----
        if not self.pre_approach_search_initialized:
            try:
                ps_map = PoseStamped()
                ps_map.header.frame_id = "map"
                ps_map.header.stamp = node.get_clock().now().to_msg()
                ps_map.pose = self.goals_queue[0]
                tf = self.tf_buffer.lookup_transform(
                    'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
                )
                first_goal_arm = do_transform_pose_stamped(ps_map, tf)
                first_goal_pos = np.array([
                    first_goal_arm.pose.position.x,
                    first_goal_arm.pose.position.y,
                    first_goal_arm.pose.position.z,
                ])
                retreat_dir = self.wall_orientation_normal / np.linalg.norm(self.wall_orientation_normal)
                effective_first_pos = first_goal_pos - retreat_dir * self.scan_standoff_distance_m

                self.pre_approach_candidates = self._generate_pre_approach_candidates(
                    node, effective_first_pos, self.wall_orientation_normal,
                )
                self.pre_approach_search_initialized = True
                node.get_logger().info(
                    f"[{self.name}] Pre-approach candidate search initialised: "
                    f"{len(self.pre_approach_candidates)} candidate(s) to evaluate "
                    f"(first scan at z={effective_first_pos[2]:.3f}m)."
                )
            except Exception as exc:
                node.get_logger().error(f"[{self.name}] Failed to initialise candidate search: {exc}")
                ctx["error_triggered"] = True
                return False

        # ----- Step 2: poll the in-flight IK request (if any) -----
        if self.pre_approach_ik_pending and self.pre_approach_ik_future is not None:
            now = node.get_clock().now()
            timed_out = (
                self.pre_approach_ik_deadline is not None
                and now > self.pre_approach_ik_deadline
            )
            if timed_out or self.pre_approach_ik_future.done():
                candidate = self.pre_approach_ik_current_candidate
                description = candidate['description']
                phase = self.pre_approach_ik_phase
                orient_idx = candidate['orient_idx']

                if timed_out:
                    node.get_logger().warn(
                        f"[{self.name}] IK timeout for [{description}] phase={phase}."
                    )
                    ok = False
                    err_code = 'timeout'
                else:
                    try:
                        response = self.pre_approach_ik_future.result()
                        ok = (
                            response is not None
                            and response.error_code.val == response.error_code.SUCCESS
                        )
                        err_code = response.error_code.val if response is not None else 'unknown'
                    except Exception as exc:
                        node.get_logger().warn(
                            f"[{self.name}] IK raised for [{description}] phase={phase}: {exc}"
                        )
                        ok = False
                        err_code = 'exception'

                self.pre_approach_ik_pending = False
                self.pre_approach_ik_future = None

                if phase == 'pre':
                    if ok:
                        # Pre-approach IK valid — capture the result joint state so the
                        # scan-row IK check can be seeded with it.  Pilz LIN starts from
                        # the post-pre-approach configuration, so seeding scan-row IK with
                        # that configuration guarantees family consistency (no SLERP jump).
                        candidate['pre_ik_passed'] = True
                        try:
                            candidate['pre_result_joint_state'] = response.solution.joint_state
                        except Exception:
                            candidate['pre_result_joint_state'] = None
                        node.get_logger().info(
                            f"[{self.name}] Pre-approach IK passed for [{description}]; "
                            f"validating scan-row IK with pre-approach joint state as seed."
                        )
                        # Do NOT pop — dispatch loop fires phase='scan' for this candidate.
                    else:
                        node.get_logger().info(
                            f"[{self.name}] Pre-approach IK FAILED for [{description}] "
                            f"(err={err_code}); trying next z-offset."
                        )
                        if self.pre_approach_candidates:
                            self.pre_approach_candidates.popleft()
                else:  # phase == 'scan'
                    if ok:
                        # Both pre-approach and scan-row IK valid in the same IK family.
                        # Accept this candidate.
                        pre_pose = candidate['pre_pose']
                        p = pre_pose.pose.position
                        node.get_logger().info(
                            f"[{self.name}] Pre-approach candidate ACCEPTED [{description}]: "
                            f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}). "
                            f"Pre-approach and scan-row IK collision-free in same IK family."
                        )
                        # IMPORTANT: align self.wall_orientation with the accepted
                        # candidate orientation so subsequent LIN scan goals use
                        # the same orientation throughout (no SLERP family change).
                        self.wall_orientation = candidate['orientation']
                        self.pre_approach_goals = deque([pre_pose])
                        self.pre_approach_planner_sequence = deque([candidate['planner']])
                        self.pre_approach_total_stages = 1
                        self.pre_approach_stage_idx = 0
                        self.pre_approach_goal = pre_pose
                        # Search done.
                        self.pre_approach_candidates.clear()
                        return True
                    else:
                        # Scan-row IK failed with pre-approach seed → LIN cannot bridge
                        # the two poses in a single IK family.  Try next candidate.
                        node.get_logger().info(
                            f"[{self.name}] Scan-row IK FAILED for [{description}] "
                            f"(err={err_code}) using pre-approach seed; trying next candidate."
                        )
                        if self.pre_approach_candidates:
                            self.pre_approach_candidates.popleft()
            else:
                # IK still in flight — wait next tick.
                return False

        # ----- Step 3: dispatch the next IK request, or declare search failed -----
        while self.pre_approach_candidates:
            next_candidate = self.pre_approach_candidates[0]  # peek

            if next_candidate.get('pre_ik_passed'):
                # Pre-approach IK already validated — fire scan-row IK seeded with
                # the pre-approach result joint state to ensure IK family continuity
                # for Pilz LIN (pre-approach → first scan goal).
                seed = next_candidate.get('pre_result_joint_state')
                if self._start_ik_request(node, next_candidate, phase='scan', seed_joint_state=seed):
                    return False
            else:
                # Validate pre-approach IK first (seeded with current joint state).
                if self._start_ik_request(node, next_candidate, phase='pre'):
                    return False

            # service_is_ready returned False — skip this candidate this tick.
            self.pre_approach_candidates.popleft()

        # No candidates left.
        node.get_logger().error(
            f"[{self.name}] All {self._pre_approach_candidate_count_total()} pre-approach "
            f"candidates failed IK validation for current height-group; "
            f"marking unreachable."
        )
        self.pre_approach_search_failed = True
        return False

    def _pre_approach_candidate_count_total(self) -> int:
        return (
            len(self.pre_approach_candidate_offsets_m)
            * len(self.pre_approach_candidate_tilts_rad)
        )

    def _reset_pre_approach_search(self):
        self.scan_goal_precheck_done = False
        self.scan_goal_precheck_pending = False
        self.scan_goal_precheck_future = None
        self.scan_goal_precheck_deadline = None
        self.pre_approach_candidates.clear()
        self.pre_approach_search_initialized = False
        self.pre_approach_search_failed = False
        self.pre_approach_ik_pending = False
        self.pre_approach_ik_future = None
        self.pre_approach_ik_current_candidate = None
        self.pre_approach_ik_deadline = None
        self.pre_approach_ik_phase = 'pre'
        self.pre_approach_passed_orient_indices = set()
        self.pre_approach_failed_orient_indices = set()
        self.pre_approach_goals.clear()
        self.pre_approach_planner_sequence.clear()
        self.pre_approach_total_stages = 0
        self.pre_approach_stage_idx = 0
        self.pre_approach_goal = None
        self.pre_approach_sent = False
        self.pre_approach_done = False
        self.waiting_for_pre_approach = False
        self.pre_approach_verbose = False

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

        self.goal_pub = node.create_publisher(PoseStamped, '/arm/goal_pose', 10)
        self.goal_pub_ptp = node.create_publisher(PoseStamped, '/arm/goal_pose_ptp', 10)
        self.goal_pub_lin = node.create_publisher(PoseStamped, '/arm/goal_pose_lin', 10)
        self.goal_pub_ompl = node.create_publisher(PoseStamped, '/arm/goal_pose_ompl', 10)
        self.goal_marker_pub = node.create_publisher(Marker, '/scan_goal_marker', 10)

        # MoveIt IK service for candidate pre-approach pre-screening.
        if self.compute_ik_client is None:
            self.compute_ik_client = node.create_client(GetPositionIK, 'compute_ik')
        if self.joint_state_sub is None:
            self.joint_state_sub = node.create_subscription(
                JointState, '/joint_states', self._joint_state_cb, 10
            )
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
        self.pre_approach_planner_sequence.clear()
        self.pre_approach_total_stages = 0
        self.pre_approach_stage_idx = 0
        self.pre_approach_sent = False
        self.pre_approach_done = False
        self.waiting_for_pre_approach = False
        self.pre_approach_verbose = False
        # Reset candidate-search state for this entry.
        self.pre_approach_candidates.clear()
        self.pre_approach_search_initialized = False
        self.pre_approach_search_failed = False
        self.pre_approach_ik_pending = False
        self.pre_approach_ik_future = None
        self.pre_approach_ik_current_candidate = None
        self.pre_approach_ik_deadline = None
        self.pre_approach_ik_phase = 'pre'
        self.pre_approach_passed_orient_indices = set()
        self.pre_approach_failed_orient_indices = set()
        self.unreachable_heights = set()
        self.post_scan_delay_start = None
        self.waiting_for_post_scan_delay = False
        self.post_scan_retraction_sent = False
        self.waiting_for_post_scan_retraction = False
        self._post_scan_retraction_pose = None
        self._post_scan_retraction_ptp_tried = False
        self._post_scan_retraction_ompl_tried = False
        self._column_stable_since = None
        self.column_target_height = None
        self.total_goals_count = 0
        self.skipped_lin_goals = 0
        self.lin_goal_retry_count = 0
        self._inflight_lin_goal_arm = None
        self._inflight_lin_goal_map = None
        self.lin_subscriber_wait_deadline = None
        self.scan_motion_backend = "moveit_lin"
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

    def _send_next_goal(self, ctx):
        """Send the next goal via MoveIt LIN."""
        node = ctx["node"]

        if not self.goals_queue:
            return

        # Peek at the next goal (MAP frame) without popping — don't commit until
        # we know the publisher is ready.
        next_goal_map = self.goals_queue[0]

        ps_map = PoseStamped()
        ps_map.header.frame_id = 'map'
        ps_map.header.stamp = node.get_clock().now().to_msg()
        ps_map.pose = next_goal_map

        if not self.tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.5)):
            node.get_logger().error(f"[{self.name}] Transform map->arm_base not available when sending goal")
            ctx["error_triggered"] = True
            return

        try:
            tf = self.tf_buffer.lookup_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0))
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

        # Pull the goal away from the wall surface by scan_standoff_distance_m so the
        # EE stays at a safe, collision-free distance and LIN IK always has a solution.
        # wall_orientation_normal points TOWARD the wall, so subtracting moves EE away.
        if self.wall_orientation_normal is not None and self.scan_standoff_distance_m > 0.0:
            n = self.wall_orientation_normal / np.linalg.norm(self.wall_orientation_normal)
            ps_arm.pose.position.x -= float(n[0]) * self.scan_standoff_distance_m
            ps_arm.pose.position.y -= float(n[1]) * self.scan_standoff_distance_m
            ps_arm.pose.position.z -= float(n[2]) * self.scan_standoff_distance_m

        # Wait for MoveIt LIN subscriber with a deadline before committing
        if self.goal_pub_lin is None or self.goal_pub_lin.get_subscription_count() == 0:
            if self.lin_subscriber_wait_deadline is None:
                self.lin_subscriber_wait_deadline = node.get_clock().now() + Duration(
                    seconds=self.lin_subscriber_wait_timeout_s
                )
                node.get_logger().warn(
                    f"[{self.name}] Waiting up to {self.lin_subscriber_wait_timeout_s:.0f}s "
                    f"for MoveIt LIN subscriber on /arm/goal_pose_lin..."
                )
            elif node.get_clock().now() > self.lin_subscriber_wait_deadline:
                node.get_logger().error(
                    f"[{self.name}] Timed out waiting for MoveIt LIN subscriber "
                    f"after {self.lin_subscriber_wait_timeout_s:.0f}s."
                )
                ctx["error_triggered"] = True
                self.lin_subscriber_wait_deadline = None
            return  # retry next tick

        # Subscriber available — commit and send
        self.goals_queue.popleft()
        self.current_goal_idx += 1
        self.lin_subscriber_wait_deadline = None
        self.lin_goal_retry_count = 0
        self._inflight_lin_goal_arm = ps_arm
        self._inflight_lin_goal_map = ps_map

        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        self.active_goal_backend = "moveit_lin"
        self.waiting_for_arrival = True
        self._publish_lin_goal_marker(node, ps_map)
        self.goal_pub_lin.publish(ps_arm)
        node.get_logger().info(
            f"[{self.name}] Sending goal {self.current_goal_idx + 1}/{self.total_goals_count} via MoveIt LIN. "
            f"pos=({ps_arm.pose.position.x:.3f}, {ps_arm.pose.position.y:.3f}, {ps_arm.pose.position.z:.3f}), "
            f"q=({ps_arm.pose.orientation.x:.3f}, {ps_arm.pose.orientation.y:.3f}, "
            f"{ps_arm.pose.orientation.z:.3f}, {ps_arm.pose.orientation.w:.3f})."
        )
        self._set_status(
            ctx,
            "Executing scan goal via MoveIt LIN",
            phase="running",
            progress_current=self.current_goal_idx + 1,
            progress_total=self.total_goals_count,
            data={"goal_backend": "moveit_lin"},
        )

    def _send_post_scan_retraction(self, ctx):
        """Send final wall retraction via MoveIt LIN after completing all scan goals."""
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

        if self.goal_pub_lin is None or self.goal_pub_lin.get_subscription_count() == 0:
            node.get_logger().error(f"[{self.name}] MoveIt LIN publisher has no subscribers; cannot send retraction.")
            ctx["error_triggered"] = True
            return

        # Compute retracted position along wall normal
        retreat_dir = self.wall_orientation_normal / np.linalg.norm(self.wall_orientation_normal)
        retreat = float(self.post_scan_retract_distance_m)
        retracted_pos = current_ee_pos - retreat_dir * retreat

        if self.wall_orientation:
            qx, qy, qz, qw = self.wall_orientation
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        ps = PoseStamped()
        ps.header.frame_id = 'arm_base'
        ps.pose.position.x = float(retracted_pos[0])
        ps.pose.position.y = float(retracted_pos[1])
        ps.pose.position.z = float(retracted_pos[2])
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        self.active_goal_backend = "moveit_lin"
        self.waiting_for_post_scan_retraction = True
        self.post_scan_retraction_sent = True
        self._post_scan_retraction_pose = ps
        self._post_scan_retraction_ptp_tried = False
        self._post_scan_retraction_ompl_tried = False

        self.goal_pub_lin.publish(ps)
        node.get_logger().info(
            f"[{self.name}] Sending post-scan retraction via MoveIt LIN: "
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

            # Reduce each horizontal row to its first and last goal so a single
            # LIN sweep covers the full row instead of N individual goal steps.
            reduced_goals = self._reduce_to_row_endpoints(ordered_goals, resolution=0.1)
            if len(reduced_goals) != len(ordered_goals):
                node.get_logger().info(
                    f"[{self.name}] Row-endpoint reduction: "
                    f"{len(ordered_goals)} -> {len(reduced_goals)} goal(s)."
                )
            ordered_goals = reduced_goals

            # Compute orientation where z-axis points towards the wall using vertex-based normal
            qx, qy, qz, qw = self._compute_orientation_towards_wall(wall_normal)
            node.get_logger().info(
                f"[{self.name}] Computed wall-facing orientation: "
                f"q=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
            )
            
            # Store orientation and wall normal for later use
            self.wall_orientation = (qx, qy, qz, qw)
            self.wall_orientation_normal = wall_normal  # Store for post-scan retraction

            # Group goals by required column height.
            # Goals are still in MAP frame at this point.
            # We transform to arm_base to get current z, then add the current column
            # height back to obtain a world-relative z that is independent of the column
            # position at the time of grouping. This keeps grouping stable across retries
            # (re-entering the state when the column is at a non-zero position must
            # produce the same height groups as the initial entry at column=0).
            column_current_for_grouping = float(ctx.get("column_current_height", 0.0))
            for p in ordered_goals:
                ps_map = PoseStamped()
                ps_map.header.frame_id = "map"
                ps_map.header.stamp = node.get_clock().now().to_msg()
                ps_map.pose = p

                try:
                    if not self.tf_buffer.can_transform('arm_base', 'map', rclpy.time.Time(), Duration(seconds=0.1)):
                        node.get_logger().warn(f"[{self.name}] TF temporarily unavailable during grouping")
                        goal_z_arm_base = 0.5
                    else:
                        tf = self.tf_buffer.lookup_transform(
                            'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
                        )
                        ps_arm = do_transform_pose_stamped(ps_map, tf)
                        goal_z_arm_base = ps_arm.pose.position.z
                except Exception as e:
                    node.get_logger().warn(f"[{self.name}] TF lookup failed during grouping: {e}")
                    goal_z_arm_base = 0.5

                # World-relative z: arm_base z + current column height
                goal_z_world = goal_z_arm_base + column_current_for_grouping
                required_height = self._choose_column_height_for_goal_z(goal_z_world)
                
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
            if len(self.goals_queue) == 0:
                # Move to next height
                self.current_height_idx += 1
                
                if self.current_height_idx >= len(self.height_sequence):
                    # All heights processed - start post-scan delay before retraction
                    if not self.waiting_for_post_scan_delay and not self.post_scan_retraction_sent:
                        # Mark panel complete here — retraction is best-effort cleanup and
                        # must not cause a re-scan of an already-completed panel on retry.
                        _completed = ctx.setdefault("completed_base_indices", [])
                        _idx = ctx.get("selected_base_idx")
                        if _idx is not None and _idx not in _completed:
                            _completed.append(_idx)
                            ctx["panels_left"] = ctx.get("panels_left", 1) - 1
                            if ctx.get("panels_left", 0) <= 0:
                                ctx["exhaustive_scan_done"] = True
                            node.get_logger().info(
                                f"[{self.name}] All scan goals completed. Panel marked done. "
                                f"Remaining: {ctx['panels_left']} panel(s)."
                            )
                        self.post_scan_delay_start = node.get_clock().now()
                        self.waiting_for_post_scan_delay = True
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
                            node.get_logger().info(f"[{self.name}] Post-scan delay complete. Sending retraction via MoveIt LIN...")
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

                    # If retraction sent, skip rest of goal processing and check MoveIt completion below
                    if self.post_scan_retraction_sent and self.waiting_for_post_scan_retraction:
                        pass

                    # Retraction done (succeeded or skipped) - emit completion event and return
                    elif self.post_scan_retraction_sent and not self.waiting_for_post_scan_retraction:
                        self.movement_done = True
                        node.get_logger().info(f"[{self.name}] Panel scan complete (retraction done/skipped).")
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
                    # Not yet beyond all heights - load next height's goals.
                    # NOTE: do NOT reset pre_approach state here.  Once the first
                    # height's pre-approach succeeded the arm is already in
                    # wall-scanning configuration (wall orientation at standoff).
                    # Subsequent heights only need a LIN translation to the next
                    # scan goal — no new OMPL pre-approach is required.  The
                    # failure handler below handles the case where the *first*
                    # pre-approach attempt fails: it resets state so the next
                    # height can try a fresh candidate search.
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
                    # Candidate IK search exhausted → mark this height as unreachable
                    # and let the next tick advance to the next height group.
                    if self.pre_approach_search_failed:
                        current_height = (
                            self.height_sequence[self.current_height_idx]
                            if 0 <= self.current_height_idx < len(self.height_sequence)
                            else None
                        )
                        skipped_count = len(self.goals_queue)
                        if current_height is not None:
                            self.unreachable_heights.add(current_height)
                        node.get_logger().warn(
                            f"[{self.name}] Height group {self.current_height_idx + 1}/"
                            f"{len(self.height_sequence)} (col={current_height}) is unreachable "
                            f"(no IK-feasible pre-approach). Skipping {skipped_count} scan goal(s)."
                        )
                        self.goals_queue.clear()
                        self.skipped_lin_goals += skipped_count
                        self._reset_pre_approach_search()
                        self._set_status(
                            ctx,
                            "Pre-approach unreachable — skipping height group",
                            phase="warning",
                            level="warn",
                            data={
                                "skipped_height": current_height,
                                "skipped_goals_this_height": skipped_count,
                            },
                        )
                    return

                if not self.pre_approach_sent:
                    self._set_status(ctx, "Sending pre-approach sequence", phase="running")
                    self._send_pre_approach_goal(ctx)
                    return

                if self.waiting_for_pre_approach:
                    if ctx.get("planner_goal_failed"):
                        # IK was validated before send, but the planner still couldn't
                        # path to it (constrained workspace).  Skip this height group
                        # rather than aborting the entire panel.
                        ctx["planner_goal_failed"] = False
                        self.waiting_for_pre_approach = False
                        current_height = (
                            self.height_sequence[self.current_height_idx]
                            if 0 <= self.current_height_idx < len(self.height_sequence)
                            else None
                        )
                        skipped_count = len(self.goals_queue)
                        if current_height is not None:
                            self.unreachable_heights.add(current_height)
                        node.get_logger().warn(
                            f"[{self.name}] Pre-approach planner failed for height group "
                            f"{self.current_height_idx + 1}/{len(self.height_sequence)} "
                            f"(col={current_height}) despite IK validation. "
                            f"Skipping {skipped_count} scan goal(s) and continuing."
                        )
                        self.goals_queue.clear()
                        self.skipped_lin_goals += skipped_count
                        self._reset_pre_approach_search()
                        self._set_status(
                            ctx,
                            "Pre-approach planner failed — skipping height group",
                            phase="warning",
                            level="warn",
                            data={
                                "skipped_height": current_height,
                                "skipped_goals_this_height": skipped_count,
                            },
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
                        if self.pre_approach_planner_sequence:
                            self.pre_approach_planner_sequence.popleft()

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
                # Pre-approach done — send next scan goal
                self._send_next_goal(ctx)

            # Don't return if waiting for retraction - let it fall through to service check
            if not (self.post_scan_retraction_sent and self.waiting_for_post_scan_retraction):
                return

        if self.active_goal_backend == "moveit_lin":
            if ctx.get("planner_goal_failed"):
                ctx["planner_goal_failed"] = False
                if self.waiting_for_post_scan_retraction:
                    pose = self._post_scan_retraction_pose
                    # Fallback order: LIN (initial) → PTP (joint-space, robust near
                    # singularities) → OMPL (free-space, last resort) → skip.
                    # Pilz LIN fails when the straight Cartesian path requires joint
                    # velocities that exceed limits (near-singularity effect on time
                    # parametrisation). PTP plans in joint space and avoids this entirely.
                    ptp_available = (
                        not self._post_scan_retraction_ptp_tried
                        and self.goal_pub_ptp is not None
                        and self.goal_pub_ptp.get_subscription_count() > 0
                        and pose is not None
                    )
                    ompl_available = (
                        not self._post_scan_retraction_ompl_tried
                        and self.goal_pub_ompl is not None
                        and self.goal_pub_ompl.get_subscription_count() > 0
                        and pose is not None
                    )
                    if ptp_available:
                        self._post_scan_retraction_ptp_tried = True
                        self.goal_pub_ptp.publish(pose)
                        node.get_logger().warn(
                            f"[{self.name}] Post-scan retraction via MoveIt LIN failed; "
                            "retrying via PTP (joint-space)."
                        )
                        self._set_status(ctx, "Retrying post-scan retraction via PTP", phase="running")
                    elif ompl_available:
                        self._post_scan_retraction_ompl_tried = True
                        self.goal_pub_ompl.publish(pose)
                        node.get_logger().warn(
                            f"[{self.name}] Post-scan retraction via PTP failed; "
                            "retrying via OMPL."
                        )
                        self._set_status(ctx, "Retrying post-scan retraction via OMPL", phase="running")
                    else:
                        node.get_logger().warn(
                            f"[{self.name}] Post-scan retraction failed (LIN → PTP → OMPL). "
                            "Arm left at scan position; skipping retraction."
                        )
                        self.waiting_for_post_scan_retraction = False
                        self.waiting_for_arrival = False
                        self.active_goal_backend = None
                        self._set_status(ctx, "Post-scan retraction skipped; continuing", phase="warning", level="warn")
                elif self.lin_goal_retry_count < self.lin_goal_max_retries:
                    self.lin_goal_retry_count += 1
                    ctx["execution_status"] = False
                    ctx["planner_goal_failed"] = False
                    self._publish_lin_goal_marker(node, self._inflight_lin_goal_map)

                    self.active_goal_backend = "moveit_lin"
                    self.goal_pub_lin.publish(self._inflight_lin_goal_arm)

                    node.get_logger().warn(
                        f"[{self.name}] MoveIt LIN goal {self.current_goal_idx + 1}/{self.total_goals_count} failed. "
                        f"Retry {self.lin_goal_retry_count}/{self.lin_goal_max_retries} via LIN..."
                    )
                    self._set_status(
                        ctx,
                        "Retrying scan goal via LIN",
                        phase="running",
                        progress_current=self.current_goal_idx + 1,
                        progress_total=self.total_goals_count,
                        data={"retry": self.lin_goal_retry_count, "max_retries": self.lin_goal_max_retries},
                    )
                else:
                    self.skipped_lin_goals += 1
                    self.waiting_for_arrival = False
                    self.active_goal_backend = None
                    node.get_logger().warn(
                        f"[{self.name}] MoveIt LIN goal {self.current_goal_idx + 1}/{self.total_goals_count} failed "
                        f"after {self.lin_goal_max_retries} retries. Skipping "
                        f"(skipped {self.skipped_lin_goals} total)."
                    )
                    self._set_status(
                        ctx,
                        "MoveIt LIN goal failed after retries, skipping scan point",
                        phase="warning",
                        progress_current=self.current_goal_idx + 1,
                        progress_total=self.total_goals_count,
                        data={"skipped_lin_goals": self.skipped_lin_goals, "max_retries": self.lin_goal_max_retries},
                        level="warn",
                    )
                return

            if ctx.get("execution_status") is True:
                ctx["execution_status"] = False
                ctx["planner_goal_failed"] = False
                self.waiting_for_arrival = False
                self.active_goal_backend = None
                if self.waiting_for_post_scan_retraction:
                    node.get_logger().info(f"[{self.name}] Post-scan retraction completed successfully via MoveIt LIN.")
                    self.waiting_for_post_scan_retraction = False
                    self._set_status(ctx, "Post-scan retraction complete", phase="running")
                else:
                    node.get_logger().info(
                        f"[{self.name}] Goal {self.current_goal_idx + 1} completed successfully via MoveIt LIN."
                    )
                    self._set_status(
                        ctx,
                        "Scan goal completed successfully via MoveIt LIN",
                        phase="running",
                        progress_current=self.current_goal_idx + 1,
                        progress_total=self.total_goals_count,
                    )
                return

            if self.waiting_for_post_scan_retraction:
                self._set_status(ctx, "Waiting for post-scan retraction via MoveIt LIN", phase="waiting")
            else:
                self._set_status(
                    ctx,
                    "Waiting for MoveIt LIN goal completion",
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
