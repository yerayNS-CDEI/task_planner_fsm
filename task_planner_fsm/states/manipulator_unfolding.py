import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ..state import State
from ..utils.gantry_kinematics import TIP_OFFSET_DEFAULT, gantry_ik

NEXT_STATE_OPTIONS = [
    "DrillApproach",
    "Error",
]

# gantry_position_controller joints, in command order (must match
# config/diffdrive_controllers.yaml). Same order the IK result uses.
GANTRY_JOINTS = ["stage1_lift_joint", "stage2_lift_joint", "stage3_rotate_joint", "stage4_horizontal_joint"]


class ManipulatorUnfolding(State):
    """Deploy the gantry so the ``drill_tip`` frame reaches the drill point,
    retracted ``drill_retract_m`` from the wall along the wall's outward normal.

    The gantry can't move in depth (no forward axis); it sets the tool tip's
    height (two lifts) and lateral position (lateral joint), while gantry_rotate
    aims the drill at the wall. The base is assumed parked parallel to the wall
    (see BasePlacementComputation), so the tip's along-wall coordinate is fixed
    by geometry and only sanity-checked here.
    """

    def __init__(self, name):
        super().__init__(name)
        self.cmd_pub = None
        self.joint_sub = None
        self.latest_joints = None
        self.target_joints = None
        self.phase = "idle"
        self.compute_start = None
        self.wait_start = None
        self.settle_start = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Unfolding gantry to the retracted drill pose.")
        ctx["manipulator_unfolding_success"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        self.latest_joints = None
        self.target_joints = None
        self.phase = "computing"
        self.compute_start = time.time()
        self.wait_start = None
        self.settle_start = None

        self.topic = str(ctx.get("gantry_command_topic", "/gantry_position_controller/commands"))
        self.joint_names = list(ctx.get("gantry_joint_names", GANTRY_JOINTS))
        self.tol = float(ctx.get("gantry_position_tolerance", 0.02))

        if self.cmd_pub is None:
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.VOLATILE
            self.cmd_pub = node.create_publisher(Float64MultiArray, self.topic, qos)

        if self.joint_sub is None:
            self.joint_sub = node.create_subscription(
                JointState,
                str(ctx.get("joint_states_topic", "/joint_states")),
                self._on_joint_states,
                10,
            )

    def _on_joint_states(self, msg):
        self.latest_joints = dict(zip(msg.name, msg.position))

    # ── Target computation ────────────────────────────────────────────────
    def _compute_target_joints(self, ctx):
        """Return the (lift1, lift2, lateral, rotate) command, or None if not ready
        yet (retry) — sets ctx['error_triggered'] on a hard failure."""
        node = ctx["node"]

        target = ctx.get("current_drill_target")
        if target is None:
            node.get_logger().error(f"[{self.name}] No current_drill_target in context.")
            self.fail(ctx, "no drilling target selected")
            return None

        retract = float(ctx.get("drill_retract_m", 0.20))
        map_frame = str(ctx.get("map_frame", "map"))
        gantry_frame = str(ctx.get("gantry_base_frame", "turret_link"))

        # Transform the drill point to the turret frame, then retract along the
        # gantry's lateral axis (+Y_turret moves away from the wall at -Y_turret).
        # NOTE: lookups must be non-blocking. fsm_node runs on a single-threaded
        # rclpy.spin(), so blocking here would starve the TransformListener.
        dp_map = PointStamped()
        dp_map.header.frame_id = map_frame
        dp_map.point.x = float(target[0])
        dp_map.point.y = float(target[1])
        dp_map.point.z = float(target[2])

        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            node.get_logger().error(f"[{self.name}] No tf_buffer in context.")
            self.fail(ctx, "no TF buffer is available to locate the gantry frame")
            return None

        try:
            if not tf_buffer.can_transform(
                gantry_frame, map_frame, rclpy.time.Time(), Duration(seconds=0.0)
            ):
                node.get_logger().warn(
                    f"[{self.name}] Waiting for TF {map_frame}->{gantry_frame}...",
                    throttle_duration_sec=2.0,
                )
                return None  # TF not ready yet; retry next tick
            dp_turret = tf_buffer.transform(dp_map, gantry_frame, timeout=Duration(seconds=0.0))
        except Exception as e:  # tf2 raises several exception types
            node.get_logger().warn(
                f"[{self.name}] Waiting for TF {map_frame}->{gantry_frame} ({e})...",
                throttle_duration_sec=2.0,
            )
            return None

        # Retracted tip = drill point + drill_retract_m along +Y_turret.
        tip_target = (dp_turret.point.x, dp_turret.point.y + retract, dp_turret.point.z)

        # Wall drilling: the rotary stage stays at 0, which aligns the horizontal
        # slide with the turret ±Y axis so it provides the drill depth/retract while
        # the lifts set the height. (θ = ±pi/2 would instead feed the slide vertically
        # for ceiling/floor drilling — see gantry_kinematics.py.)
        rotate_angle = float(ctx.get("gantry_rotate_angle_rad", 0.0))

        tip_offset = tuple(ctx.get("drill_tip_offset", TIP_OFFSET_DEFAULT))
        lateral_tol = float(ctx.get("gantry_lateral_tol_m", 0.05))
        result = gantry_ik(tip_target, rotate_angle, tip_offset, lateral_tol=lateral_tol)

        along_tol = float(ctx.get("gantry_along_wall_tol_m", 0.50))
        if abs(result.along_wall_residual) > along_tol:
            node.get_logger().error(
                f"[{self.name}] Drill point mis-aligned along the wall by "
                f"{result.along_wall_residual:.3f} m (> {along_tol:.3f}); reposition the base."
            )
            self.fail(
                ctx,
                f"the drilling point is off along the wall by "
                f"{result.along_wall_residual:.2f} m (> {along_tol:.2f} m)",
            )
            return None
        if not result.feasible:
            node.get_logger().error(
                f"[{self.name}] Retracted drill pose not reachable by the gantry: {result.reason}."
            )
            self.fail(ctx, f"the gantry cannot reach the drilling pose ({result.reason})")
            return None

        node.get_logger().info(
            f"[{self.name}] Tip target (turret) ({tip_target[0]:.3f}, {tip_target[1]:.3f}, "
            f"{tip_target[2]:.3f}), retract {retract:.2f} m -> joints "
            f"lift1={result.joints[0]:.3f} lift2={result.joints[1]:.3f} "
            f"rotate={math.degrees(result.joints[2]):.1f}deg horizontal={result.joints[3]:.3f} "
            f"(along-wall residual {result.along_wall_residual:.3f} m)."
        )
        return list(result.joints)

    _PHASE_ACTIVITY = {
        "computing": "Computing the gantry pose that reaches the drilling point",
        "commanding": "Commanding the gantry to the retracted drilling pose",
        "settling": "Waiting for the gantry to reach the retracted drilling pose",
    }

    def run(self, ctx):
        node = ctx["node"]
        activity = self._PHASE_ACTIVITY.get(self.phase)
        if activity is not None:
            self.set_activity(ctx, activity)

        if ctx.get("manipulator_unfolding_success") or ctx.get("error_triggered"):
            return

        # ── Phase 0: compute the joint target (needs the wall map + TF). ──
        if self.phase == "computing":
            compute_timeout = float(ctx.get("unfolding_compute_timeout", 15.0))
            self.target_joints = self._compute_target_joints(ctx)
            if ctx.get("error_triggered"):
                return
            if self.target_joints is None:
                if time.time() - self.compute_start > compute_timeout:
                    node.get_logger().error(
                        f"[{self.name}] Could not compute the unfolding target within "
                        f"{compute_timeout:.0f}s (TF/wall map unavailable)."
                    )
                    self.fail(
                        ctx,
                        f"the unfolding target could not be computed within "
                        f"{compute_timeout:.0f} s (TF/wall map unavailable)",
                    )
                return
            self.phase = "commanding"
            self.wait_start = time.time()
            return

        # ── Phase 1: publish the joint command once the controller is listening. ──
        if self.phase == "commanding":
            timeout = float(ctx.get("gantry_command_timeout", 15.0))
            if self.cmd_pub.get_subscription_count() < 1:
                if time.time() - self.wait_start > timeout:
                    node.get_logger().error(
                        f"[{self.name}] No subscriber on {self.topic} after {timeout:.0f}s; "
                        f"is gantry_position_controller running?"
                    )
                    self.fail(
                        ctx,
                        f"no gantry_position_controller listening on {self.topic} after "
                        f"{timeout:.0f} s",
                    )
                else:
                    node.get_logger().warn(
                        f"[{self.name}] Waiting for gantry_position_controller on {self.topic}...",
                        throttle_duration_sec=2.0,
                    )
                    self.set_activity(ctx, "Waiting for the gantry controller to come up")
                return

            msg = Float64MultiArray()
            msg.data = self.target_joints
            self.cmd_pub.publish(msg)
            node.get_logger().info(
                f"[{self.name}] Sent gantry unfolding command {self.target_joints}; "
                f"waiting for the gantry to reach it..."
            )
            self.settle_start = time.time()
            self.phase = "settling"
            return

        # ── Phase 2: wait until the gantry joints settle at the target. ──
        if self.phase == "settling":
            settle_timeout = float(ctx.get("gantry_settle_timeout", 30.0))
            elapsed = time.time() - self.settle_start

            if self.latest_joints is None:
                self._settle_wait_or_fail(
                    ctx, elapsed, settle_timeout,
                    waiting_msg=f"Waiting for {ctx.get('joint_states_topic', '/joint_states')}...",
                    fail_msg=f"No joint states received after {settle_timeout:.0f}s; "
                             f"cannot confirm the gantry reached the target.",
                )
                return

            missing = [j for j in self.joint_names if j not in self.latest_joints]
            if missing:
                self._settle_wait_or_fail(
                    ctx, elapsed, settle_timeout,
                    waiting_msg=f"Waiting for gantry joint(s) {missing} in joint states...",
                    fail_msg=f"Gantry joint(s) {missing} absent from joint states after "
                             f"{settle_timeout:.0f}s.",
                )
                return

            errors = {
                j: abs(self.latest_joints[j] - target)
                for j, target in zip(self.joint_names, self.target_joints)
            }
            worst = max(errors.values())
            if worst <= self.tol:
                node.get_logger().info(
                    f"[{self.name}] Gantry reached the retracted drill pose (max joint error "
                    f"{worst:.4f} <= {self.tol}); unfolding complete."
                )
                ctx["manipulator_unfolding_success"] = True
                return

            if elapsed > settle_timeout:
                detail = ", ".join(f"{j}={self.latest_joints[j]:.3f}" for j in self.joint_names)
                node.get_logger().error(
                    f"[{self.name}] Gantry did not reach the target within {settle_timeout:.0f}s "
                    f"(max joint error {worst:.4f} > {self.tol}); positions: {detail}."
                )
                self.fail(
                    ctx,
                    f"the gantry did not reach the drilling pose within "
                    f"{settle_timeout:.0f} s (max joint error {worst:.4f} > {self.tol})",
                )
            else:
                node.get_logger().warn(
                    f"[{self.name}] Waiting for gantry to reach the target "
                    f"(max joint error {worst:.4f} > {self.tol})...",
                    throttle_duration_sec=2.0,
                )

    def _settle_wait_or_fail(self, ctx, elapsed, timeout, *, waiting_msg, fail_msg):
        node = ctx["node"]
        if elapsed > timeout:
            node.get_logger().error(f"[{self.name}] {fail_msg}")
            self.fail(ctx, fail_msg)
        else:
            node.get_logger().warn(f"[{self.name}] {waiting_msg}", throttle_duration_sec=2.0)
            self.set_activity(ctx, "Waiting for gantry joint feedback")

    def check_transition(self, ctx):
        if not ctx.get("manipulator_unfolding_success") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
