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
from ..utils.gantry_kinematics import HORIZONTAL_LIMITS, TIP_OFFSET_DEFAULT, gantry_ik

NEXT_STATE_OPTIONS = [
    "SuctionDrillStart",
    "Error",
]

# gantry_position_controller joints, in command order (must match
# config/diffdrive_controllers.yaml and the gantry_ik result order).
GANTRY_JOINTS = ["stage1_lift_joint", "stage2_lift_joint", "stage3_rotate_joint", "stage4_horizontal_joint"]

# Index of the horizontal slide (stage 4) within GANTRY_JOINTS / the command array.
# This is the only stage moved during the approach; the other three are held.
SLIDE_IDX = 3


class DrillApproach(State):
    """Advance the drill tip from the retracted unfolding pose up to the drill point.

    ``ManipulatorUnfolding`` parks the ``drill_tip`` frame at the drill point but
    retracted ``drill_retract_m`` from the wall along the wall's outward normal
    (+Y_turret). This state closes that gap by commanding **only the last gantry
    stage** — the ``stage4_horizontal_joint`` slide, which in wall-drilling mode
    (rotate ≈ 0) feeds the tool along the turret ±Y (depth) axis. The two lifts and
    the rotary stage are held at their current measured positions.

    The slide is driven to the value that places the tip exactly on the drill point
    (retract 0, i.e. touching the wall without pressing), clamped to the slide's
    hardware travel so the tip gets as close as possible **without surpassing** the
    drill point (never pushing into the wall). A nonzero ``drill_approach_standoff_m``
    can be set to stop a fixed distance short of the wall; it defaults to 0.
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
        node.get_logger().info(f"[{self.name}] Approaching the drill point with the gantry slide.")
        ctx["drill_approach_success"] = False
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
        """Return the 4-joint command that advances the slide to the drill point,
        holding the lifts and rotary stage at their measured positions.

        Returns None to retry (TF / joint states not ready yet) and sets
        ctx['error_triggered'] on a hard failure.
        """
        node = ctx["node"]

        # Need the current gantry joints so we can hold everything but the slide.
        if self.latest_joints is None:
            node.get_logger().warn(
                f"[{self.name}] Waiting for {ctx.get('joint_states_topic', '/joint_states')}...",
                throttle_duration_sec=2.0,
            )
            return None
        missing = [j for j in self.joint_names if j not in self.latest_joints]
        if missing:
            node.get_logger().warn(
                f"[{self.name}] Waiting for gantry joint(s) {missing} in joint states...",
                throttle_duration_sec=2.0,
            )
            return None
        held = [float(self.latest_joints[j]) for j in self.joint_names]
        # Remember the pre-approach gantry pose (the retracted unfolding pose) so
        # DrillRetract can return the arm exactly here once drilling is done.
        ctx["gantry_approach_start_joints"] = list(held)

        target = ctx.get("current_drill_target")
        if target is None:
            node.get_logger().error(f"[{self.name}] No current_drill_target in context.")
            ctx["error_triggered"] = True
            return None

        standoff = float(ctx.get("drill_approach_standoff_m", 0.0))
        map_frame = str(ctx.get("map_frame", "map"))
        gantry_frame = str(ctx.get("gantry_base_frame", "turret_link"))

        # Transform the drill point into the turret frame. The base is parked and
        # the turret is rigid to it, so this TF is stable while only the gantry moves.
        # NOTE: lookups must be non-blocking — fsm_node runs a single-threaded
        # rclpy.spin(), so blocking here would starve the TransformListener.
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            node.get_logger().error(f"[{self.name}] No tf_buffer in context.")
            ctx["error_triggered"] = True
            return None

        dp_map = PointStamped()
        dp_map.header.frame_id = map_frame
        dp_map.point.x = float(target[0])
        dp_map.point.y = float(target[1])
        dp_map.point.z = float(target[2])
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

        # Target tip = the drill point itself, held back by the (default 0) standoff
        # along +Y_turret (the wall's outward normal). retract is 0 here — the tip
        # should just touch the wall, not press into it.
        tip_target = (dp_turret.point.x, dp_turret.point.y + standoff, dp_turret.point.z)

        # Solve the slide about the measured rotary angle so the slide-feed direction
        # matches the stage we are actually holding. Wall mode keeps rotate ≈ 0, which
        # aligns the slide with turret ±Y (depth).
        rotate_angle = held[2]
        tip_offset = tuple(ctx.get("drill_tip_offset", TIP_OFFSET_DEFAULT))
        lateral_tol = float(ctx.get("gantry_lateral_tol_m", 0.05))
        result = gantry_ik(tip_target, rotate_angle, tip_offset, lateral_tol=lateral_tol)

        # The unfolding step already gated on along-wall alignment; just re-check it
        # is still sane (the base has not moved since).
        along_tol = float(ctx.get("gantry_along_wall_tol_m", 0.50))
        if abs(result.along_wall_residual) > along_tol:
            node.get_logger().error(
                f"[{self.name}] Drill point mis-aligned along the wall by "
                f"{result.along_wall_residual:.3f} m (> {along_tol:.3f}); cannot approach."
            )
            ctx["error_triggered"] = True
            return None

        ideal_slide = result.joints[SLIDE_IDX]
        # Clamp to the slide's hardware travel so we get as close as possible without
        # commanding an out-of-range joint. Because the approach only ever reduces the
        # retract from the (feasible) unfolding pose, clamping can stop us short of the
        # wall but never drives the tip past the drill point.
        target_slide = max(HORIZONTAL_LIMITS[0], min(HORIZONTAL_LIMITS[1], ideal_slide))

        # Safety guard: verify the clamped slide does not surpass the drill point along
        # the outward normal (+Y_turret). tip_y for a given slide is
        #   c*(slide + off_y) - s*off_z   with (c, s) = (cos, sin)(rotate).
        c, s = math.cos(rotate_angle), math.sin(rotate_angle)
        off_y, off_z = float(tip_offset[1]), float(tip_offset[2])
        achieved_tip_y = c * (target_slide + off_y) - s * off_z
        surpass_tol = float(ctx.get("drill_approach_surpass_tol_m", 1e-3))
        if achieved_tip_y < dp_turret.point.y + standoff - surpass_tol:
            node.get_logger().error(
                f"[{self.name}] Slide clamp would surpass the drill point "
                f"(tip Y {achieved_tip_y:.3f} < target {dp_turret.point.y + standoff:.3f} m); "
                f"aborting approach to avoid pressing into the wall."
            )
            ctx["error_triggered"] = True
            return None

        remaining = achieved_tip_y - (dp_turret.point.y + standoff)  # >= 0: how far short
        cmd = list(held)
        cmd[SLIDE_IDX] = target_slide

        if abs(target_slide - ideal_slide) > 1e-6:
            node.get_logger().warn(
                f"[{self.name}] Slide travel-limited: ideal {ideal_slide:.3f} clamped to "
                f"{target_slide:.3f} m; tip stops {remaining:.3f} m short of the drill point."
            )
        else:
            node.get_logger().info(
                f"[{self.name}] Slide {held[SLIDE_IDX]:.3f} -> {target_slide:.3f} m "
                f"advances the tip to the drill point (standoff {standoff:.3f} m, "
                f"remaining {remaining:.3f} m)."
            )
        return cmd

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("drill_approach_success") or ctx.get("error_triggered"):
            return

        # ── Phase 0: compute the joint target (needs joint states + TF). ──
        if self.phase == "computing":
            compute_timeout = float(ctx.get("approach_compute_timeout", 15.0))
            self.target_joints = self._compute_target_joints(ctx)
            if ctx.get("error_triggered"):
                return
            if self.target_joints is None:
                if time.time() - self.compute_start > compute_timeout:
                    node.get_logger().error(
                        f"[{self.name}] Could not compute the approach target within "
                        f"{compute_timeout:.0f}s (TF/joint states unavailable)."
                    )
                    ctx["error_triggered"] = True
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
                    ctx["error_triggered"] = True
                else:
                    node.get_logger().warn(
                        f"[{self.name}] Waiting for gantry_position_controller on {self.topic}...",
                        throttle_duration_sec=2.0,
                    )
                return

            msg = Float64MultiArray()
            msg.data = self.target_joints
            self.cmd_pub.publish(msg)
            node.get_logger().info(
                f"[{self.name}] Sent gantry approach command {self.target_joints}; "
                f"waiting for the slide to reach it..."
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
                             f"cannot confirm the slide reached the target.",
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
                    f"[{self.name}] Drill tip reached the drill point (max joint error "
                    f"{worst:.4f} <= {self.tol}); approach complete."
                )
                ctx["drill_approach_success"] = True
                return

            if elapsed > settle_timeout:
                detail = ", ".join(f"{j}={self.latest_joints[j]:.3f}" for j in self.joint_names)
                node.get_logger().error(
                    f"[{self.name}] Gantry did not reach the target within {settle_timeout:.0f}s "
                    f"(max joint error {worst:.4f} > {self.tol}); positions: {detail}."
                )
                ctx["error_triggered"] = True
            else:
                node.get_logger().warn(
                    f"[{self.name}] Waiting for the slide to reach the target "
                    f"(max joint error {worst:.4f} > {self.tol})...",
                    throttle_duration_sec=2.0,
                )

    def _settle_wait_or_fail(self, ctx, elapsed, timeout, *, waiting_msg, fail_msg):
        node = ctx["node"]
        if elapsed > timeout:
            node.get_logger().error(f"[{self.name}] {fail_msg}")
            ctx["error_triggered"] = True
        else:
            node.get_logger().warn(f"[{self.name}] {waiting_msg}", throttle_duration_sec=2.0)

    def check_transition(self, ctx):
        if not ctx.get("drill_approach_success") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
