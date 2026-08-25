#!/usr/bin/env python3
"""Whole-body compliant sweep of one wall segment.

Replaces the ``sweep_wait`` phase of ``ScanWall`` — a Nav2 position goal for the
base plus ``wall_parallel_controller`` streaming IK setpoints to the arm, two
controllers each unaware of the other — with ONE control law over base + arm::

    read the six plate ranges -> sensed surface frame (normal, tangent)
    build the task twist:  sweep along the tangent
                           hold the standoff along the normal
                           hold the row height
                           keep the plate parallel and upright
    resolve it across 9 DOF with a QP that respects the base's actuator limits
    publish  base Twist (turret frame)  +  arm joint velocities

Started per segment by the FSM with the segment endpoints as parameters, it
reports on ``status_topic`` ("running" / "succeeded" / "failed: <reason>") and
the FSM's ``sweep_wait`` waits on that exactly as it waits on a Nav2 result.
Termination is by ARC LENGTH along the sensed surface, not by a base pose: the
sweep is done when the plate has travelled the segment.

The column is not touched — it holds the row height set before the sweep. On the
real robot ``force_mode`` cannot coexist with the streaming velocity controller
this node needs, so it is sim-first by design; the FSM gates it accordingly.

Run standalone (outside the FSM) for bench tests::

    ros2 run task_planner_fsm wbc_sweep_controller --ros-args \
      -p seg_start:="[1.0, 2.0, 1.2]" -p seg_end:="[3.0, 2.0, 1.2]" \
      -p sweep_speed:=0.03
"""

import math
import signal
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from .avoidance import AvoidanceConfig, ObstacleField, avoidance_rows
from .base_model import BaseLimits, box_bounds, constraint_rows, wheel_and_turret_rates
from .kinematics import SerialChain, rotation_error, whole_body_jacobian
from .qp import Task, joint_limit_bounds, solve_velocity_qp
from .surface import SurfaceEstimator, plate_orientation_target, sweep_tangent

ARM_JOINTS = [
    "arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint",
    "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint",
]


class WholeBodySweepNode(Node):

    def __init__(self, **kwargs):
        # kwargs are passed through to rclpy's Node so tests can inject
        # parameter_overrides without a launch file or a running robot.
        super().__init__("wbc_sweep_controller", **kwargs)

        # --- What to sweep ---------------------------------------------------
        # Segment endpoints are ON the wall, in the map frame, at the row height
        # (the same points the FSM's reachable_wall_segments produces).
        self.declare_parameter("seg_start", [0.0, 0.0, 0.0])
        self.declare_parameter("seg_end", [0.0, 0.0, 0.0])
        self.declare_parameter("row_z", float("nan"))     # NaN -> hold the height we start at
        self.declare_parameter("sweep_speed", 0.03)       # m/s along the wall
        self.declare_parameter("standoff", 0.20)          # m, plate to wall
        self.declare_parameter("arrive_tolerance", 0.03)  # m of arc length
        self.declare_parameter("timeout_pad", 30.0)       # s beyond length/speed
        # Where to leave the plate when the sweep ends, so the base can transit
        # with the arm clear of the wall. Match the FSM's
        # scan_wall_transit_plate_offset. 0 disables the retreat.
        self.declare_parameter("retreat_standoff", 0.40)
        self.declare_parameter("retreat_speed", 0.05)     # m/s along the normal
        self.declare_parameter("retreat_timeout", 20.0)   # s on the node clock
        # Arm configuration to finish in, normally the unfolded pose the FSM's
        # planner last placed the arm in. Retreating along the normal alone
        # cannot leave the arm somewhere the planner will accept: too little and
        # A* dies on the wall's dilation, too much and wrist_3 enters the mast's
        # keep-out cylinder, and the safe window moves with the base's standoff.
        # Empty disables the return.
        # Declared with an explicit DOUBLE_ARRAY descriptor, NOT `[]`: rclpy
        # infers the element type from the default, and an empty list gives it
        # nothing to infer, so it settles on BYTE_ARRAY and then rejects the
        # override at startup — the node dies before it can report anything.
        self.declare_parameter(
            "return_joints", None,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter("return_gain", 1.0)        # 1/s toward the target
        self.declare_parameter("return_tolerance", 0.05)  # rad, per joint
        self.declare_parameter("return_timeout", 25.0)    # s on the node clock
        # Fail a sweep that has stopped advancing this long, instead of waiting
        # out the whole length/speed budget with the plate stuck on something.
        self.declare_parameter("no_progress_timeout", 25.0)

        # --- Frames and I/O --------------------------------------------------
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "turret_footprint")
        # The chain root doubles as the TF frame for the arm mount, so the model
        # and the measurement can never disagree about which frame that is. Note
        # it is 'arm_base_link' (the URDF link), NOT the FSM's 'arm_base' — the
        # UR description carries both, rotated pi about z from each other.
        self.declare_parameter("arm_root_link", "arm_base_link")
        # The plate ranges are measured at 'arm_plate_link', which shares its
        # orientation with 'arm_tool0' (both hang off arm_ee_cylinder_link with
        # the same rpy) and sits 0.15 m further along the shared +Z. Using tool0
        # as the tip therefore keeps the sensed normal valid, and the standoff
        # keeps the same meaning it has everywhere else in the FSM.
        self.declare_parameter("arm_tip_link", "arm_tool0")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("arm_command_topic", "/forward_velocity_controller/commands")
        self.declare_parameter("distance_topic", "/distance_sensors")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("status_topic", "/wbc_sweep/status")
        self.declare_parameter("turret_joint", "turret_joint")
        self.declare_parameter("arm_joints", ARM_JOINTS)

        # --- Control law -----------------------------------------------------
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("k_standoff", 1.0)      # 1/s on the normal error
        self.declare_parameter("k_height", 1.0)        # 1/s on the row-height error
        self.declare_parameter("k_align", 1.5)         # 1/s on the plate orientation error
        self.declare_parameter("v_normal_max", 0.05)   # m/s cap on the standoff correction
        self.declare_parameter("w_align_max", 0.30)    # rad/s cap on the orientation correction
        self.declare_parameter("ema_alpha", 0.3)       # surface-normal low-pass
        self.declare_parameter("weight_linear", 1.0)
        self.declare_parameter("weight_angular", 0.5)
        # Per-DOF damping. The arm is damped ~10x harder than the base so the
        # redundancy resolves the way the task wants it to: the BASE carries the
        # along-wall travel (it can go forever) and the arm spends its motion on
        # the standoff, height and parallelism corrections it is quick at.
        self.declare_parameter("damping_base", [0.01, 0.01, 0.02])
        # Keep this a regulariser, not a competing objective. At 0.1 the cost of
        # a 0.5 rad/s arm motion is the same order as the task cost of missing
        # the reference by 0.05 m/s, so once the base is (correctly) discouraged
        # from taking normal corrections, the QP simply declines to close the
        # standoff error at all. It only looked fine while the base was doing the
        # arm's job for it.
        self.declare_parameter("damping_arm", 0.03)
        # How hard the base is discouraged from moving along the wall normal, so
        # the arm takes the standoff corrections instead. Costs the sweep nothing
        # (the sweep is tangential) but is a soft term, not a constraint, so the
        # base can still follow a wall that is not straight.
        self.declare_parameter("weight_base_normal", 100.0)
        self.declare_parameter("weight_posture", 0.02)
        self.declare_parameter("k_posture", 0.5)
        # Ceiling on how fast the posture task may pull, in rad/s. Without it the
        # posture pull grows with joint displacement while the standoff reference
        # is saturated at v_normal_max, so the secondary task eventually
        # overpowers the primary one and the standoff settles with a large steady
        # error. A background term must stay in the background.
        self.declare_parameter("posture_rate_max", 0.05)
        self.declare_parameter("arm_qdot_max", 0.5)     # rad/s
        self.declare_parameter("joint_limit_margin", 0.15)  # rad

        # --- Base actuator limits (mirror the sim_controller config) ---------
        self.declare_parameter("center_distance", 0.167)
        self.declare_parameter("wheel_radius", 0.125)
        self.declare_parameter("wheel_separation", 0.3655)
        self.declare_parameter("max_x_velocity", 0.2)
        self.declare_parameter("max_y_velocity", 0.2)
        self.declare_parameter("max_angular_turret", 0.35)
        self.declare_parameter("max_angular_base", 0.2)
        self.declare_parameter("max_turret_motor_speed", 0.5)
        self.declare_parameter("sweep_speed_margin", 0.9)   # of the computed cap

        # --- Obstacle avoidance ----------------------------------------------
        # The sweep drives the base itself, so nothing else is watching for
        # obstacles while it runs; these turn the local costmap into barrier rows
        # in the same QP. See wbc/avoidance.py.
        self.declare_parameter("avoid_obstacles", True)
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("avoid_safety_margin", 0.15)
        self.declare_parameter("avoid_influence", 1.0)
        self.declare_parameter("avoid_alpha", 1.0)
        self.declare_parameter("avoid_obstacle_cost", 100)
        self.declare_parameter("avoid_unknown_is_obstacle", False)
        self.declare_parameter("avoid_footprint_radius", 0.45)
        self.declare_parameter("avoid_footprint_offset", [0.0, 0.0])
        self.declare_parameter("avoid_n_samples", 8)
        self.declare_parameter("avoid_max_rows", 6)
        # Must exceed the costmap's inflation radius, or the scanned wall's
        # inflated ring survives the mask and pushes the base off the wall.
        self.declare_parameter("avoid_mask_halfwidth", 0.7)
        self.declare_parameter("avoid_mask_extension", 3.0)
        self.declare_parameter("avoid_slack_weight", 1e3)
        # A costmap that never arrives is a warning, not a failure: the sweep is
        # slow, deliberate motion over ground the FSM already vetted with
        # reachable_wall_segments, and refusing to scan because Nav2 is not
        # publishing would be its own kind of unsafe. Flip this to insist.
        # Floor on the base's own distance to the wall being scanned, measured
        # from the plate's sensed standoff rather than from any map. Nav2 models
        # this base as a 0.6 m disc, so a base closer than that to the wall is
        # "in collision" by its own reckoning and no transit will plan.
        self.declare_parameter("base_wall_min_gap", 0.75)
        self.declare_parameter("avoid_require_costmap", False)
        self.declare_parameter("costmap_max_age", 10.0)

        # --- Safety ----------------------------------------------------------
        self.declare_parameter("max_data_age", 1.0)        # s
        self.declare_parameter("max_standoff_error", 0.25)  # m before we call it lost
        self.declare_parameter("standoff_error_cycles", 5)  # ... for this many cycles
        # How long the robot may sit stopped waiting for usable inputs before the
        # sweep is called off. In SECONDS, not cycles: at 50 Hz a cycle budget
        # turns a routine TF hiccup (rtabmap republishing map->odom) into a
        # failed sweep a fifth of a second later.
        self.declare_parameter("max_hold_seconds", 3.0)

        p = self.get_parameter
        self.seg_start = np.array(p("seg_start").value, dtype=float)
        self.seg_end = np.array(p("seg_end").value, dtype=float)
        self.row_z = float(p("row_z").value)
        self.standoff = float(p("standoff").value)
        self.arrive_tolerance = float(p("arrive_tolerance").value)
        self.map_frame = str(p("map_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.arm_root_link = str(p("arm_root_link").value)
        self.arm_tip_link = str(p("arm_tip_link").value)
        self.turret_joint = str(p("turret_joint").value)
        self.arm_joints = list(p("arm_joints").value)
        self.control_rate = float(p("control_rate").value)
        self.max_data_age = float(p("max_data_age").value)
        self.max_standoff_error = float(p("max_standoff_error").value)
        self.standoff_error_cycles = int(p("standoff_error_cycles").value)
        self.max_hold_seconds = float(p("max_hold_seconds").value)

        self.limits = BaseLimits(
            center_distance=float(p("center_distance").value),
            wheel_radius=float(p("wheel_radius").value),
            wheel_separation=float(p("wheel_separation").value),
            vx_max=float(p("max_x_velocity").value),
            vy_max=float(p("max_y_velocity").value),
            wz_max=float(p("max_angular_turret").value),
            w_chassis_max=float(p("max_angular_base").value),
            w_turret_motor_max=float(p("max_turret_motor_speed").value),
        )
        # The along-wall speed is set by the base, not chosen: sweeping sideways
        # costs chassis yaw rate (w = v_lateral / center_distance). Asking for
        # more just makes sim_controller rescale the command, which silently
        # distorts the sweep, so clamp the reference here and say so.
        # How fast the base can sweep depends on the direction it sweeps in, and
        # that is not known until the surface is sensed — so the reference is
        # capped per cycle (see _control_step), and this only says what to expect.
        self.sweep_speed = float(p("sweep_speed").value)
        sideways = self.limits.max_lateral_speed() * float(p("sweep_speed_margin").value)
        if self.sweep_speed > sideways:
            self.get_logger().warn(
                f"sweep_speed {self.sweep_speed:.3f} m/s is above the {sideways:.3f} m/s "
                f"this base holds sweeping SIDEWAYS (d1 x chassis yaw rate), which is the "
                f"usual geometry with the chassis parked square to the wall. It will be "
                f"capped per cycle by the actual sweep direction."
            )

        segment_length = float(np.linalg.norm(self.seg_end[:2] - self.seg_start[:2]))
        self.segment_length = segment_length
        self.deadline = None
        self.timeout = segment_length / max(self.sweep_speed, 1e-3) + float(p("timeout_pad").value)

        # --- State -----------------------------------------------------------
        self.chain = None
        self.urdf = None
        self.joint_positions = {}
        self.joint_stamp = None
        self.distances = None
        self.distance_stamp = None
        self.surface = SurfaceEstimator(ema_alpha=float(p("ema_alpha").value))
        self.q_posture = None
        self.holding_since = None
        self.standoff_strikes = 0
        # "sweep" -> "retreat" (backing the plate off the wall) -> "done". The
        # terminal status is withheld until "done", because the FSM kills this
        # process as soon as it sees one.
        self.phase = "sweep"
        self.pending_status = None
        self.retreat_deadline = 0.0
        self.return_deadline = 0.0
        # A typed-but-unset parameter RAISES on .value rather than returning
        # None, so an absent return target has to be caught, not defaulted.
        try:
            raw = p("return_joints").value
        except rclpy.exceptions.ParameterUninitializedException:
            raw = None
        self.return_joints = [float(v) for v in (raw or [])]
        self.status = "running"
        self.control_timer = None
        self.progress = 0.0
        self.best_progress = 0.0
        self.progress_stamp = None   # set on the first cycle, not at construction

        self.avoid = bool(p("avoid_obstacles").value)
        self.avoid_config = AvoidanceConfig(
            safety_margin=float(p("avoid_safety_margin").value),
            influence=float(p("avoid_influence").value),
            alpha=float(p("avoid_alpha").value),
            obstacle_cost=int(p("avoid_obstacle_cost").value),
            unknown_is_obstacle=bool(p("avoid_unknown_is_obstacle").value),
            footprint_radius=float(p("avoid_footprint_radius").value),
            footprint_offset=tuple(p("avoid_footprint_offset").value),
            n_samples=int(p("avoid_n_samples").value),
            max_rows=int(p("avoid_max_rows").value),
            mask_halfwidth=float(p("avoid_mask_halfwidth").value),
            mask_extension=float(p("avoid_mask_extension").value),
        )
        self.costmap_msg = None
        self.costmap_stamp = None
        self.obstacle_field = None
        self._field_from = None      # the message the cached field was built from
        self.closest_obstacle = float("inf")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- ROS I/O ---------------------------------------------------------
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, str(p("status_topic").value), latched)
        self.cmd_vel_pub = self.create_publisher(Twist, str(p("cmd_vel_topic").value), 10)
        self.arm_cmd_pub = self.create_publisher(
            Float64MultiArray, str(p("arm_command_topic").value), 10)
        self.create_subscription(
            String, str(p("robot_description_topic").value), self._on_robot_description, latched)
        self.create_subscription(
            JointState, str(p("joint_states_topic").value), self._on_joint_states, 10)
        self.create_subscription(
            Float32MultiArray, str(p("distance_topic").value), self._on_distances, 10)
        if self.avoid:
            self.create_subscription(
                OccupancyGrid, str(p("costmap_topic").value), self._on_costmap, 1)
        # Republished, not just latched: the FSM may subscribe after we start.
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f"Whole-body sweep: ({self.seg_start[0]:.2f}, {self.seg_start[1]:.2f}) -> "
            f"({self.seg_end[0]:.2f}, {self.seg_end[1]:.2f})  [{segment_length:.2f} m] "
            f"at {self.sweep_speed:.3f} m/s, standoff {self.standoff:.2f} m."
        )

    def _now(self):
        """Seconds on the NODE's clock — sim time when ``use_sim_time`` is set.

        Never ``time.time()``: Gazebo on a loaded machine runs well below
        real-time (0.2x is normal with rtabmap and Nav2 alongside), so a
        wall-clock staleness window would declare fresh sensor frames stale, and
        a wall-clock deadline would time a sweep out several times faster than
        the robot can travel it. Everything here — data ages, the sweep deadline,
        the control period — must tick on the same clock the robot moves on.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_robot_description(self, msg):
        if self.chain is not None:
            return
        self.urdf = msg.data
        try:
            self.chain = SerialChain.from_urdf(self.urdf, self.arm_root_link, self.arm_tip_link)
        except Exception as exc:
            self.get_logger().error(f"Cannot build the arm chain from the URDF: {exc}")
            return
        self.get_logger().info(
            f"Arm chain {self.arm_root_link} -> {self.arm_tip_link}: "
            f"{self.chain.joint_names}"
        )
        # The QP solves for the CHAIN's joints; the command goes out in the
        # velocity controller's joint order. If the two sets disagree the
        # commands would be silently zeroed, so refuse the sweep instead.
        missing = set(self.chain.joint_names) ^ set(self.arm_joints)
        if missing:
            self.get_logger().error(
                f"The chain's joints and the 'arm_joints' parameter disagree "
                f"({sorted(missing)}); the arm would receive no command. "
                f"Set arm_joints to the controller's joint list."
            )
            self.chain = None

    def _on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = float(position)
        self.joint_stamp = self._now()

    def _on_distances(self, msg):
        if len(msg.data) == 6:
            self.distances = np.array(msg.data, dtype=float)
            self.distance_stamp = self._now()

    def _on_costmap(self, msg):
        self.costmap_msg = msg
        self.costmap_stamp = self._now()

    def _obstacle_field(self):
        """The cached distance field, rebuilt when a new costmap arrives.

        The costmap has its own frame (``odom`` for a rolling local costmap), and
        a grid cannot be rotated into another frame without resampling, so the
        field stays in that frame and the query points are brought TO it. The
        scan segment is transformed the same way before being masked out.
        """
        msg = self.costmap_msg
        if msg is None:
            return None
        if self._field_from is msg:
            return self.obstacle_field

        frame = msg.header.frame_id or self.map_frame
        segment = self._segment_in(frame)
        try:
            data = np.asarray(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width)
            self.obstacle_field = ObstacleField.from_grid(
                data, msg.info.resolution,
                (msg.info.origin.position.x, msg.info.origin.position.y),
                self.avoid_config, mask_segment=segment)
        except Exception as exc:
            self.get_logger().warn(f"Unusable costmap: {exc}", throttle_duration_sec=10.0)
            self.obstacle_field = None
        self._field_from = msg
        return self.obstacle_field

    def _segment_in(self, frame):
        """The scan segment's endpoints in ``frame``, or None if TF cannot say.

        Masking the wrong strip is worse than masking none: the wall would keep
        pushing the base while some innocent patch of floor went unguarded. So a
        failed transform drops the mask rather than guessing.
        """
        if frame == self.map_frame:
            return (self.seg_start[:2], self.seg_end[:2])
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.map_frame, rclpy.time.Time())
        except Exception:
            self.get_logger().warn(
                f"No TF {self.map_frame}->{frame}; sweeping without the wall mask, "
                f"so the scanned wall itself may push the base away.",
                throttle_duration_sec=10.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        R = _quat_to_matrix(np.array([q.x, q.y, q.z, q.w]))[:2, :2]
        shift = np.array([t.x, t.y])
        return (R @ self.seg_start[:2] + shift, R @ self.seg_end[:2] + shift)

    def _avoidance_rows(self, n_arm):
        """Barrier rows for this cycle, or empty when there is nothing to avoid."""
        empty = (np.zeros((0, 3 + n_arm)), np.zeros(0))
        if not self.avoid:
            return empty
        field = self._obstacle_field()
        if field is None:
            self.get_logger().warn(
                "No costmap yet: sweeping with NO obstacle avoidance on the base.",
                throttle_duration_sec=5.0)
            return empty
        age = self._now() - (self.costmap_stamp or 0.0)
        if age > float(self.get_parameter("costmap_max_age").value):
            # STOP using it, don't just grumble. The local costmap is a rolling
            # window anchored to the robot: once it stops updating, the grid
            # describes where the robot USED to be, and the barrier starts
            # steering around obstacles that are no longer there while missing
            # anything that has appeared. Observed in Gazebo — a phantom
            # obstacle 0.07 m away pushed the base backwards and stalled the
            # last 6 cm of a sweep. Better to say plainly that there is no
            # avoidance than to act confidently on a stale picture.
            self.get_logger().error(
                f"Costmap is {age:.0f}s old — DROPPING base obstacle avoidance rather "
                f"than steering by a stale map. The sweep continues unguarded.",
                throttle_duration_sec=5.0)
            self.closest_obstacle = float("inf")
            return empty

        frame = self.costmap_msg.header.frame_id or self.map_frame
        pose = self._lookup(self.base_frame, source_frame=frame)
        if pose is None:
            self.get_logger().warn(
                f"No TF {frame}->{self.base_frame}; no obstacle rows this cycle.",
                throttle_duration_sec=5.0)
            return empty
        quat, position = pose
        yaw = math.atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                         1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2))
        A, lower, _, closest = avoidance_rows(
            field, yaw, position, self.avoid_config, n_arm=n_arm)
        self.closest_obstacle = closest
        return A, lower

    # ------------------------------------------------------------------
    # Startup / teardown
    # ------------------------------------------------------------------
    def wait_until_ready(self, timeout_s=30.0):
        """Block (spinning) until model, joints, sensors and TF are all live.

        Everything the control law reads must be present BEFORE the first cycle:
        a whole-body command computed from a half-populated state would move the
        base with a stale arm pose. Returns True when ready.
        """
        if not self._wait_for_clock():
            self.get_logger().error(
                "use_sim_time is set but no /clock arrived; refusing to start a sweep "
                "whose timing would be meaningless."
            )
            return False
        deadline = self._now() + timeout_s
        missing = None
        while rclpy.ok() and self._now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            missing = self._missing_inputs()
            if not missing:
                # The posture task pulls back toward the pose the sweep starts
                # from — the one the FSM's pre-approach chose for this row.
                self.q_posture = self._arm_positions()
                if math.isnan(self.row_z):
                    self.row_z = float(self._tip_pose()[2, 3])
                    self.get_logger().info(
                        f"Holding the row height we start at: z={self.row_z:.3f} m.")
                return True
        self.get_logger().error(f"Not ready after {timeout_s:.0f}s; missing: {missing}.")
        return False

    def _wait_for_clock(self, timeout_s=15.0):
        """Wait for ROS time to become real before anything is timed against it.

        With ``use_sim_time`` the node's clock reads 0 until the first ``/clock``
        message lands, and then jumps to the simulator's time. A deadline
        computed in that window is instantly in the past — which is exactly how
        a healthy sweep reported "inputs never became available" a second after
        starting. This is the one place a wall clock is the right tool: we are
        waiting for the other clock to exist.
        """
        if not self.get_parameter("use_sim_time").value:
            return True
        end = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < end:
            if self._now() > 0.0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._now() > 0.0

    def _missing_inputs(self):
        missing = []
        if self.chain is None:
            missing.append("robot_description")
        if self._arm_positions() is None:
            missing.append("joint_states")
        if self.distances is None:
            missing.append("distance_sensors")
        if self._mount_pose() is None:
            missing.append(f"TF {self.map_frame}->{self.arm_root_link}")
        if self._base_pose() is None:
            missing.append(f"TF {self.map_frame}->{self.base_frame}")
        if (self.avoid and self.costmap_msg is None
                and bool(self.get_parameter("avoid_require_costmap").value)):
            missing.append("costmap")
        return missing

    def start(self):
        """Begin the control loop."""
        self.deadline = self._now() + self.timeout
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_step)
        self.get_logger().info(
            f"Sweeping at {self.control_rate:.0f} Hz (timeout {self.timeout:.0f}s)."
        )

    def halt(self):
        """Zero every command this node owns. Safe to call repeatedly."""
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.control_timer = None
        self.cmd_vel_pub.publish(Twist())
        self.arm_cmd_pub.publish(Float64MultiArray(data=[0.0] * len(self.arm_joints)))

    def finish(self, status, reason=""):
        """End the sweep — but back the plate off the wall before saying so.

        The plate ends a sweep at its scanning standoff, centimetres off the
        surface, and everything that happens next needs it further out: the FSM
        retracts it before letting the base transit, and its planner has to solve
        that as a Cartesian goal through an octomap that contains the wall the
        plate is nearly touching. From there the search routinely fails ("no path
        found"), the FSM shrugs and carries on, and the base then transits with
        the arm still extended.

        This node is in a much better position to do it: it already holds the
        arm's velocity controller and it knows, from the plate's own sensors,
        which way "away from the wall" is. So retreat straight back along the
        sensed normal first, then hand the arm over — by which time the FSM's
        retraction has nothing left to do and never calls the planner at all.

        The terminal status is deliberately withheld until the retreat finishes:
        the FSM kills this process the moment it sees one.
        """
        self.pending_status = status if not reason else f"{status}: {reason}"
        level = self.get_logger().info if status == "succeeded" else self.get_logger().error
        level(f"Sweep {self.pending_status} (progress {self.progress:.2f}/"
              f"{self.segment_length:.2f} m).")

        target = float(self.get_parameter("retreat_standoff").value)
        if self.phase != "sweep" or target <= 0.0 or self.surface.distance is None:
            self._terminate()
            return
        self.cmd_vel_pub.publish(Twist())        # the base is done moving
        self.phase = "retreat"
        self.retreat_deadline = self._now() + float(
            self.get_parameter("retreat_timeout").value)
        self.get_logger().info(
            f"Retreating the plate from {self.surface.distance:.2f} m to {target:.2f} m "
            f"along the sensed normal before handing the arm back."
        )

    def _terminate(self):
        """Publish the withheld status and stop commanding anything."""
        self.phase = "done"
        self.halt()
        self.status = self.pending_status or "failed: ended without a status"
        self._publish_status()

    def _publish_status(self):
        self.status_pub.publish(String(data=self.status))

    # ------------------------------------------------------------------
    # State readers
    # ------------------------------------------------------------------
    def _arm_positions(self):
        try:
            return np.array([self.joint_positions[name] for name in self.arm_joints])
        except KeyError:
            return None

    def _turret_angle(self):
        """Turret joint angle (turret heading relative to the chassis).

        It decides how a commanded turret-frame twist lands on the chassis axes,
        so the base's actuator constraints are rebuilt around it every cycle.
        Falls back to 0 (chassis parked square, which is how a sweep starts).
        """
        return self.joint_positions.get(self.turret_joint, 0.0)

    def _lookup(self, target_frame, source_frame=None):
        try:
            tf = self.tf_buffer.lookup_transform(
                source_frame or self.map_frame, target_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.array([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])

    def _mount_pose(self):
        """4x4 pose of the arm mount in the map frame.

        Straight from TF, so it carries the turret angle, the column's forward
        offset from the turret axis and the column's live height — none of which
        this node has to know about. Distinct from ``_base_pose``, which is the
        turret AXIS: the two are about 0.29 m apart horizontally.
        """
        result = self._lookup(self.arm_root_link)
        if result is None:
            return None
        quat, position = result
        T = np.eye(4)
        T[:3, :3] = _quat_to_matrix(quat)
        T[:3, 3] = position
        return T

    def _tip_pose(self):
        """4x4 pose of the plate tip in the map frame, or None if inputs are missing."""
        T_mount = self._mount_pose()
        q_arm = self._arm_positions()
        if T_mount is None or q_arm is None or self.chain is None:
            return None
        return T_mount @ self.chain.fk(q_arm)

    def _base_pose(self):
        """(yaw, xy) of the TURRET AXIS in the map frame.

        ``turret_footprint`` is the ground projection of ``turret_link``, i.e. of
        the axis the base rotates about and the point sim_controller's twist
        applies to — which is what the base Jacobian's lever arm must start from.
        """
        result = self._lookup(self.base_frame)
        if result is None:
            return None
        quat, position = result
        yaw = math.atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                         1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2))
        return yaw, position

    # ------------------------------------------------------------------
    # The control loop
    # ------------------------------------------------------------------
    def _retreat_step(self):
        """Back the plate off along the sensed normal, arm only, base held still.

        Deliberately the same QP as the sweep with the base pinned to zero: the
        arm's joint limits and the plate's orientation task still apply, so the
        retreat cannot fling a joint into a stop or twist the plate on the way
        out. Any input it cannot trust ends the retreat rather than guessing a
        direction to move the arm in.
        """
        now = self._now()
        target = float(self.get_parameter("retreat_standoff").value)

        if now > self.retreat_deadline:
            self.get_logger().warn(
                f"Retreat timed out at {self.surface.distance:.2f} m (wanted "
                f"{target:.2f} m); returning the arm from here.")
            self._begin_return()
            return
        if self._stale_inputs(now) or not self.surface.update(self.distances):
            self.get_logger().warn("Lost the plate ranges mid-retreat; stopping here.")
            self._terminate()
            return

        T_mount = self._mount_pose()
        base = self._base_pose()
        q_arm = self._arm_positions()
        if T_mount is None or base is None or q_arm is None:
            self._terminate()
            return
        yaw, p_base = base
        J, T_plate = whole_body_jacobian(self.chain, q_arm, T_mount, yaw, p_base)
        R_plate = T_plate[:3, :3]
        m_hat = R_plate @ self.surface.normal_plate

        remaining = target - self.surface.distance
        if remaining <= 0.01:
            self.get_logger().info(
                f"Plate retreated to {self.surface.distance:.2f} m; arm is clear of "
                f"the wall.")
            self._begin_return()
            return

        p = self.get_parameter
        speed = min(float(p("retreat_speed").value), remaining)
        n_arm = self.chain.n_joints
        xdot = np.concatenate((-speed * m_hat, np.zeros(3)))
        weights = np.concatenate((np.full(3, float(p("weight_linear").value)),
                                  np.zeros(3)))
        damping = np.concatenate((np.array(p("damping_base").value, dtype=float),
                                  np.full(n_arm, float(p("damping_arm").value))))
        tasks = [Task(J, xdot, weights),
                 Task(np.diag(damping), np.zeros(3 + n_arm), 1.0)]

        lower_arm, upper_arm = self.chain.position_limits()
        arm_lo, arm_hi = joint_limit_bounds(
            q_arm, lower_arm, upper_arm, float(p("arm_qdot_max").value),
            margin=float(p("joint_limit_margin").value))
        solution = solve_velocity_qp(
            tasks,
            np.concatenate((np.zeros(3), arm_lo)),      # base pinned: arm only
            np.concatenate((np.zeros(3), arm_hi)))
        if not solution.ok:
            self.get_logger().warn("Retreat QP did not solve; stopping here.")
            self._terminate()
            return

        self._publish(solution.u, n_arm)
        self.get_logger().info(
            f"Retreating: d={self.surface.distance * 100:.1f}cm -> {target * 100:.0f}cm",
            throttle_duration_sec=1.0)

    def _begin_return(self):
        """Move to the joint-space return, or stop if there is nothing to return to."""
        if not self.return_joints or self.chain is None:
            self._terminate()
            return
        self.phase = "return"
        self.return_deadline = self._now() + float(
            self.get_parameter("return_timeout").value)
        self.get_logger().info(
            "Returning the arm to the configuration the planner left it in.")

    def _return_step(self):
        """Drive the arm joints back to ``return_joints``; the base stays put.

        Joint space on purpose. The target is a configuration the FSM's planner
        actually reached, so it is reachable, clear of the mast cylinder and
        something A* can plan out of — none of which any particular Cartesian
        standoff can guarantee.
        """
        now = self._now()
        q_arm = self._arm_positions()
        if q_arm is None or len(self.return_joints) != len(q_arm):
            self.get_logger().warn("No usable joint state for the return; stopping here.")
            self._terminate()
            return

        error = np.asarray(self.return_joints, dtype=float) - q_arm
        worst = float(np.max(np.abs(error)))
        if worst <= float(self.get_parameter("return_tolerance").value):
            self.get_logger().info(
                f"Arm back at the unfolded configuration (worst joint error "
                f"{worst:.3f} rad).")
            self._terminate()
            return
        if now > self.return_deadline:
            self.get_logger().warn(
                f"Return timed out with {worst:.2f} rad still to go; handing the arm "
                f"back as it is.")
            self._terminate()
            return

        limit = float(self.get_parameter("arm_qdot_max").value)
        qdot = np.clip(error * float(self.get_parameter("return_gain").value), -limit, limit)
        self.cmd_vel_pub.publish(Twist())
        self._publish(np.concatenate((np.zeros(3), qdot)), len(q_arm))
        self.get_logger().info(
            f"Returning: worst joint error {worst:.2f} rad", throttle_duration_sec=1.0)

    def _control_step(self):
        now = self._now()
        if self.phase == "done":
            return
        if self.phase == "retreat":
            self._retreat_step()
            return
        if self.phase == "return":
            self._return_step()
            return
        if self.status != "running":
            return
        if self.deadline is not None and now > self.deadline:
            self.finish("failed", f"timed out after {self.timeout:.0f}s")
            return

        stale = self._stale_inputs(now)
        if stale:
            self._strike(f"stale input: {stale}")
            return
        if not self.surface.update(self.distances):
            self._strike(f"only {self.surface.n_valid}/6 plate ranges valid — no plane fit")
            return

        T_mount = self._mount_pose()
        base = self._base_pose()
        q_arm = self._arm_positions()
        if T_mount is None or base is None or q_arm is None:
            self._strike("lost TF or joint states")
            return
        yaw, p_base = base
        # The turret joint angle relates the frame the twist is commanded in to
        # the chassis that has to yaw for it; both the speed cap and the actuator
        # rows below are built around it.
        phi = self._turret_angle()

        J, T_plate = whole_body_jacobian(self.chain, q_arm, T_mount, yaw, p_base)
        R_plate, p_plate = T_plate[:3, :3], T_plate[:3, 3]

        # --- Where are we, along the sensed surface? -------------------------
        direction = self.seg_end[:2] - self.seg_start[:2]
        length = np.linalg.norm(direction)
        if length < 1e-6:
            self.finish("failed", "degenerate segment (zero length)")
            return
        u_hat = np.array([direction[0] / length, direction[1] / length, 0.0])
        self.progress = float(np.dot(p_plate - np.array(
            [self.seg_start[0], self.seg_start[1], p_plate[2]]), u_hat))
        remaining = length - self.progress
        if remaining <= self.arrive_tolerance:
            self.finish("succeeded")
            return

        # Give up on a sweep that has stopped advancing, rather than sitting on
        # the wall until the full length/speed budget expires. Something is
        # holding the plate — an obstacle it is pressed against, a barrier it
        # cannot satisfy — and none of it improves by waiting several more
        # minutes with the arm loaded against the surface.
        if self.progress_stamp is None or self.progress > self.best_progress + 0.02:
            self.best_progress = self.progress
            self.progress_stamp = now
        else:
            stall = float(self.get_parameter("no_progress_timeout").value)
            if now - self.progress_stamp > stall:
                self.finish(
                    "failed",
                    f"no progress for {stall:.0f}s at {self.progress:.2f}/{length:.2f} m "
                    f"(something is holding the plate)")
                return

        # --- The sensed surface frame ---------------------------------------
        m_hat = R_plate @ self.surface.normal_plate      # plate -> wall, world axes
        t_hat = sweep_tangent(m_hat, u_hat)
        if t_hat is None:
            self._strike("scan direction is normal to the sensed surface")
            return
        distance = self.surface.distance
        if abs(distance - self.standoff) > self.max_standoff_error:
            # A gap this far off is the plate meeting something the wall plane
            # does not explain (a pilaster, a fixture) or losing the wall
            # entirely — the case the old sweep could not see. Require several
            # cycles in a row so one bad plane fit cannot end a good sweep.
            self.standoff_strikes += 1
            if self.standoff_strikes >= self.standoff_error_cycles:
                self.finish(
                    "failed",
                    f"standoff {distance:.3f} m is {abs(distance - self.standoff):.3f} m off "
                    f"the {self.standoff:.2f} m target (obstacle or lost surface)")
                return
        else:
            self.standoff_strikes = 0

        # --- Task twist ------------------------------------------------------
        p = self.get_parameter
        v_normal = _clamp(float(p("k_standoff").value) * (distance - self.standoff),
                          float(p("v_normal_max").value))
        v_height = _clamp(float(p("k_height").value) * (self.row_z - p_plate[2]),
                          float(p("v_normal_max").value))
        # Cap the reference by what the base can actually hold in THIS direction
        # (sideways is ~6x slower than straight ahead), then ease off over the
        # last few centimetres so the sweep stops ON the segment end instead of
        # overshooting past it between cycles. Asking for more than the base can
        # give does not make it go faster — sim_controller would rescale the
        # command, and the arm would try to take up the difference until it ran
        # out of reach.
        heading = math.atan2(t_hat[1], t_hat[0])
        reachable = self.limits.max_speed_along(
            heading - yaw, heading - (yaw - phi)) * float(p("sweep_speed_margin").value)
        speed = min(self.sweep_speed, remaining, reachable)
        v_ref = speed * t_hat + v_normal * m_hat + v_height * np.array([0.0, 0.0, 1.0])

        R_target = plate_orientation_target(m_hat)
        if R_target is None:
            self._strike("sensed surface normal is vertical")
            return
        w_ref = rotation_error(R_plate, R_target) * float(p("k_align").value)
        w_max = float(p("w_align_max").value)
        if np.linalg.norm(w_ref) > w_max:
            w_ref = w_ref * (w_max / np.linalg.norm(w_ref))
        xdot = np.concatenate((v_ref, w_ref))

        # --- Resolve it across the 9 DOF -------------------------------------
        n_arm = self.chain.n_joints
        weights = np.concatenate((np.full(3, float(p("weight_linear").value)),
                                  np.full(3, float(p("weight_angular").value))))
        damping = np.concatenate((np.array(p("damping_base").value, dtype=float),
                                  np.full(n_arm, float(p("damping_arm").value))))
        # The BASE owns the tangent, the ARM owns the normal — the task-frame
        # ownership the design calls for. Without this the damping weights decide
        # it instead, and they say the base is ~100x cheaper than the arm on
        # EVERY axis, so the base ends up carrying the standoff corrections too:
        # it creeps toward or away from the wall for the whole sweep while the
        # arm barely moves. A steady 1 cm of standoff error is then enough to
        # walk the base out of position over a few minutes, until it sits inside
        # Nav2's collision radius and cannot transit to the next segment.
        # Penalising only the base's velocity ALONG the sensed normal costs the
        # sweep nothing, since the sweep runs along the tangent, which is
        # perpendicular to it.
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        base_normal_row = np.zeros((1, 3 + n_arm))
        base_normal_row[0, :2] = m_hat[:2] @ rotation
        tasks = [
            Task(J, xdot, weights),
            Task(np.diag(damping), np.zeros(3 + n_arm), 1.0),
            Task(base_normal_row, np.zeros(1), float(p("weight_base_normal").value)),
            # Posture: pull the arm back toward the pose the sweep started from,
            # so the redundancy is spent keeping the arm workable instead of
            # slowly stretching it out over the length of the wall.
            Task(np.hstack((np.zeros((n_arm, 3)), np.eye(n_arm))),
                 _clamp_norm(float(p("k_posture").value) * (self.q_posture - q_arm),
                             float(p("posture_rate_max").value)),
                 float(p("weight_posture").value)),
        ]

        lower_arm, upper_arm = self.chain.position_limits()
        arm_lo, arm_hi = joint_limit_bounds(
            q_arm, lower_arm, upper_arm, float(p("arm_qdot_max").value),
            margin=float(p("joint_limit_margin").value))
        base_lo, base_hi = box_bounds(self.limits)
        A_ineq, ineq_lo, ineq_hi = constraint_rows(self.limits, phi)
        A_ineq = np.hstack((A_ineq, np.zeros((A_ineq.shape[0], n_arm))))

        # Obstacle barriers go in SOFT: a base pinned inside its margin can be
        # asked to retreat faster than it is able to strafe, and a hard row there
        # would make the whole solve infeasible — stopping the robot instead of
        # backing it off. Actuator limits stay hard.
        A_avoid, avoid_lo = self._avoidance_rows(n_arm)

        # The scanned wall is masked out of the costmap barrier by design, which
        # leaves nothing at all watching the one obstacle the base is guaranteed
        # to be near. Close that with a barrier against the SENSED wall instead:
        # the plate measures where the surface is, so the base's distance to it
        # is (plate standoff + how far the base sits behind the plate) along the
        # normal. Keep that above a floor, or the base ends up closer to the wall
        # than Nav2's own footprint allows and the next transit cannot plan.
        base_gap = float(np.dot(p_plate[:2] - p_base[:2], m_hat[:2])) + distance
        floor = float(p("base_wall_min_gap").value)
        if floor > 0.0:
            A_avoid = np.vstack((A_avoid, -base_normal_row))
            avoid_lo = np.concatenate((
                avoid_lo, [-float(p("avoid_alpha").value) * (base_gap - floor)]))
            if base_gap < floor:
                self.get_logger().warn(
                    f"Base is {base_gap:.2f} m from the wall, inside the {floor:.2f} m "
                    f"floor; backing it off before it gets stuck against the surface.",
                    throttle_duration_sec=5.0)

        solution = solve_velocity_qp(
            tasks,
            np.concatenate((base_lo, arm_lo)), np.concatenate((base_hi, arm_hi)),
            A_ineq=A_ineq, ineq_lo=ineq_lo, ineq_hi=ineq_hi,
            soft_ineq=A_avoid if A_avoid.shape[0] else None, soft_lo=avoid_lo,
            soft_weight=float(p("avoid_slack_weight").value))
        if not solution.ok:
            self._strike(f"QP did not solve ({solution.status})")
            return
        if solution.slack > 1e-3:
            self.get_logger().warn(
                f"Obstacle {self.closest_obstacle:.2f} m away: retreating as fast as "
                f"the base allows, still {solution.slack:.3f} m/s short of the barrier.",
                throttle_duration_sec=2.0)
        if solution.solver.endswith("(box-only)"):
            self.get_logger().warn(
                "OSQP unavailable: solving with a box-only fallback, so the base's "
                "chassis/turret/wheel limits are NOT enforced by the solver.",
                throttle_duration_sec=10.0)

        self.holding_since = None
        self._publish(solution.u, n_arm)
        self._log_cycle(solution, distance, remaining, phi)

    def _stale_inputs(self, now):
        stale = []
        if self.joint_stamp is None or now - self.joint_stamp > self.max_data_age:
            stale.append("joint_states")
        if self.distance_stamp is None or now - self.distance_stamp > self.max_data_age:
            stale.append("distance_sensors")
        return ", ".join(stale)

    def _strike(self, reason):
        """One unusable cycle: stop the robot, and give up if it keeps happening.

        Commanding zero (rather than the last command) matters — every consumer
        here is a velocity interface that would happily keep driving.
        """
        now = self._now()
        if self.holding_since is None:
            self.holding_since = now
        self.cmd_vel_pub.publish(Twist())
        self.arm_cmd_pub.publish(Float64MultiArray(data=[0.0] * len(self.arm_joints)))
        held = now - self.holding_since
        if held >= self.max_hold_seconds:
            self.finish("failed", f"{reason} (held {held:.1f}s)")
        else:
            self.get_logger().warn(f"Holding: {reason}.", throttle_duration_sec=1.0)

    def _publish(self, u, n_arm):
        """Base Twist (turret frame, as sim_controller's ``cmd_type: relative``
        expects) plus arm joint velocities, in the controller's joint order."""
        twist = Twist()
        twist.linear.x, twist.linear.y = float(u[0]), float(u[1])
        twist.angular.z = float(u[2])
        self.cmd_vel_pub.publish(twist)

        by_name = dict(zip(self.chain.joint_names, u[3:3 + n_arm]))
        self.arm_cmd_pub.publish(Float64MultiArray(
            data=[float(by_name.get(name, 0.0)) for name in self.arm_joints]))

    def _log_cycle(self, solution, distance, remaining, phi):
        w_left, w_right, phi_dot, w_chassis = wheel_and_turret_rates(
            self.limits, solution.u[:3], phi)
        clearance = ("--" if not np.isfinite(self.closest_obstacle)
                     else f"{self.closest_obstacle:.2f}m")
        self.get_logger().info(
            f"d={distance * 100:.1f}cm tilt={math.degrees(self.surface.tilt()):.1f}deg "
            f"left={remaining:.2f}m | base=({solution.u[0]:+.3f}, {solution.u[1]:+.3f}, "
            f"{solution.u[2]:+.3f}) w_chassis={w_chassis:+.2f} phi_dot={phi_dot:+.2f} "
            f"| arm max={np.max(np.abs(solution.u[3:])):.3f} rad/s "
            f"| clear={clearance} | residual={solution.task_residual:.4f}",
            throttle_duration_sec=1.0)


def _clamp(value, limit):
    return float(max(-limit, min(limit, value)))


def _clamp_norm(vector, limit):
    """Scale ``vector`` down to ``limit`` in norm, preserving its direction."""
    norm = float(np.linalg.norm(vector))
    if limit <= 0.0 or norm <= limit or norm < 1e-12:
        return vector
    return vector * (limit / norm)


def _quat_to_matrix(q):
    """Rotation matrix from (x, y, z, w) — avoids a scipy import in the loop."""
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def main(args=None):
    from rclpy.signals import SignalHandlerOptions

    from .controller_switch import ArmControllerSwitch

    # Handle the stop signal ourselves. rclpy's default handler invalidates the
    # context the moment SIGINT lands, and this node has real work to do on the
    # way out: zero the base and arm commands, and hand the arm's command
    # interfaces back to the trajectory controller. With the default handler
    # both of those throw ("publisher's context is invalid") and the arm is left
    # parked on a velocity controller nobody is feeding — every later FSM arm
    # goal would then silently do nothing. SIGTERM is treated the same way, so a
    # supervisor that stops the node also gets a clean handover.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WholeBodySweepNode()
    switch = ArmControllerSwitch(node)

    stopping = False

    def _request_stop(signum, _frame):
        nonlocal stopping
        stopping = True
        # A second signal should still be able to kill us outright.
        signal.signal(signum, signal.SIG_DFL)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        if not node.wait_until_ready():
            node.finish("failed", "inputs never became available")
        elif not switch.to_velocity_control():
            node.finish("failed", "could not activate the arm velocity controller")
        else:
            node.start()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.halt()
        switch.restore()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
