import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ..state import State

NEXT_STATE_OPTIONS = [
    "SampleScanning",
    "Error",
]

# gantry_position_controller joints, in command order (must match
# config/diffdrive_controllers.yaml).
GANTRY_JOINTS = ["stage1_lift_joint", "stage2_lift_joint", "stage3_rotate_joint", "stage4_horizontal_joint"]


class DrillRetract(State):
    """Return the gantry to the pre-approach pose — the reverse of ``DrillApproach``.

    ``DrillApproach`` recorded the gantry pose it started from (the retracted
    unfolding pose) in ``gantry_approach_start_joints``. This state commands the
    gantry straight back to that pose and waits for it to settle. Unlike
    ``TakeOutDrill`` (a velocity-ramped withdrawal), this mirrors the approach's own
    single position command since the bit is already clear of the wall.
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
        node.get_logger().info(f"[{self.name}] Retracting the gantry to the pre-approach pose.")
        ctx["drill_retract_success"] = False
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

    def _compute_target_joints(self, ctx):
        """Return the pre-approach joint command, or None to retry (joint states not
        ready). Falls back to holding the current pose if no approach pose was stored."""
        node = ctx["node"]

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

        target = ctx.get("gantry_approach_start_joints")
        if target is None or len(target) != len(self.joint_names):
            # No recorded approach pose (e.g. manual entry): hold the current pose so
            # we don't command a wild motion.
            node.get_logger().warn(
                f"[{self.name}] No recorded pre-approach pose; holding the current gantry pose."
            )
            return [float(self.latest_joints[j]) for j in self.joint_names]

        node.get_logger().info(
            f"[{self.name}] Returning gantry to the pre-approach pose {list(target)}."
        )
        return [float(v) for v in target]

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("drill_retract_success") or ctx.get("error_triggered"):
            return

        # ── Phase 0: resolve the target pose (needs joint states). ──
        if self.phase == "computing":
            compute_timeout = float(ctx.get("gantry_command_timeout", 15.0))
            self.target_joints = self._compute_target_joints(ctx)
            if self.target_joints is None:
                if time.time() - self.compute_start > compute_timeout:
                    node.get_logger().error(
                        f"[{self.name}] Could not resolve the retract target within "
                        f"{compute_timeout:.0f}s (no joint states)."
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
                f"[{self.name}] Sent gantry retract command {self.target_joints}; "
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
                             f"cannot confirm the gantry retracted.",
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
                    f"[{self.name}] Gantry back at the pre-approach pose (max joint error "
                    f"{worst:.4f} <= {self.tol}); retract complete."
                )
                ctx["drill_retract_success"] = True
                return

            if elapsed > settle_timeout:
                detail = ", ".join(f"{j}={self.latest_joints[j]:.3f}" for j in self.joint_names)
                node.get_logger().error(
                    f"[{self.name}] Gantry did not reach the pre-approach pose within "
                    f"{settle_timeout:.0f}s (max joint error {worst:.4f} > {self.tol}); "
                    f"positions: {detail}."
                )
                ctx["error_triggered"] = True
            else:
                node.get_logger().warn(
                    f"[{self.name}] Waiting for the gantry to retract "
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
        if not ctx.get("drill_retract_success") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
