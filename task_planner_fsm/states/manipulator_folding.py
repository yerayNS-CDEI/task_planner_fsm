import time

from ..state import State
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

NEXT_STATE_OPTIONS = [
    "BasePlacementComputation",
    "HomePosition",
    "Error",
]

# robo_drill gantry_position_controller joints, in command order
# (must match the `joints:` list in config/diffdrive_controllers.yaml).
GANTRY_JOINTS = ["stage1_lift_joint", "stage2_lift_joint", "stage3_rotate_joint", "stage4_horizontal_joint"]

# Coming from either of these states means there is no drilling left to do --
# StoringToDatabase after the last location was drilled, or WaitForData when
# oliwall reported it needs no more holes -- so this is the final fold of the run
# and the robot heads home right after it. Arriving from TargetSelection instead
# is the mid-run fold before the base is repositioned, which keeps the operator's
# next-state choice.
FINAL_FOLD_PREDECESSORS = ("StoringToDatabase", "WaitForData")


class ManipulatorFolding(State):
    def __init__(self, name):
        super().__init__(name)
        self.cmd_pub = None
        self.joint_sub = None
        self.latest_joints = None  # dict: joint name -> position
        self.phase = "idle"
        self.wait_start = None
        self.settle_start = None
        self.final_fold = False

    def on_enter(self, ctx):
        node = ctx["node"]
        ctx["manipulator_folding_success"] = False
        ctx["error_triggered"] = False
        self._user_choice = None
        self.phase = "commanding"
        self.wait_start = time.time()
        self.settle_start = None
        self.latest_joints = None

        # A retry re-enters this state with last_state == our own name, so keep
        # the decision made on the original entry.
        last_state = ctx.get("last_state")
        if last_state != self.name:
            self.final_fold = last_state in FINAL_FOLD_PREDECESSORS

        # robo_drill gantry_position_controller (a ForwardCommandController) takes
        # a Float64MultiArray on this topic; joint order is
        # [stage1_lift, stage2_lift, stage3_rotate, stage4_horizontal]. Folding = send it to origin.
        self.topic = str(ctx.get("gantry_command_topic", "/gantry_position_controller/commands"))
        self.joint_names = list(ctx.get("gantry_joint_names", GANTRY_JOINTS))
        self.home = [float(v) for v in ctx.get("gantry_home_positions", [0.0] * len(self.joint_names))]
        self.tol = float(ctx.get("gantry_position_tolerance", 0.02))

        if self.cmd_pub is None:
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.VOLATILE
            self.cmd_pub = node.create_publisher(Float64MultiArray, self.topic, qos)

        # Read back the actual joint positions so we can confirm the gantry
        # physically reached origin (the controller gives no motion feedback).
        if self.joint_sub is None:
            self.joint_sub = node.create_subscription(
                JointState,
                str(ctx.get("joint_states_topic", "/joint_states")),
                self._on_joint_states,
                10,
            )

        node.get_logger().info(
            f"[{self.name}] Folding gantry to origin {tuple(self.home)} via {self.topic}."
        )
        if self.final_fold:
            node.get_logger().info(
                f"[{self.name}] Drilling work is over (came from {last_state}); "
                f"returning home once the gantry is folded."
            )

    def _on_joint_states(self, msg):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Folding the gantry back to its origin")

        if ctx.get("manipulator_folding_success") or ctx.get("error_triggered"):
            return

        # ── Phase 1: publish the origin command once the controller is listening. ──
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
            msg.data = self.home
            self.cmd_pub.publish(msg)
            node.get_logger().info(
                f"[{self.name}] Sent gantry origin command {self.home}; "
                f"waiting for the gantry to reach it..."
            )
            self.settle_start = time.time()
            self.phase = "settling"
            return

        # ── Phase 2: wait until the gantry joints settle at the origin. ──
        if self.phase == "settling":
            self.set_activity(ctx, "Waiting for the gantry to reach its folded pose")
            settle_timeout = float(ctx.get("gantry_settle_timeout", 30.0))
            elapsed = time.time() - self.settle_start

            if self.latest_joints is None:
                self._settle_wait_or_fail(
                    ctx, elapsed, settle_timeout,
                    waiting_msg=f"Waiting for {ctx.get('joint_states_topic', '/joint_states')}...",
                    fail_msg=f"No joint states received after {settle_timeout:.0f}s; "
                             f"cannot confirm the gantry reached origin.",
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
                for j, target in zip(self.joint_names, self.home)
            }
            worst = max(errors.values())
            if worst <= self.tol:
                node.get_logger().info(
                    f"[{self.name}] Gantry reached origin (max joint error "
                    f"{worst:.4f} <= {self.tol}); folding complete."
                )
                ctx["manipulator_folding_success"] = True
                return

            if elapsed > settle_timeout:
                detail = ", ".join(f"{j}={self.latest_joints[j]:.3f}" for j in self.joint_names)
                node.get_logger().error(
                    f"[{self.name}] Gantry did not reach origin within {settle_timeout:.0f}s "
                    f"(max joint error {worst:.4f} > {self.tol}); positions: {detail}."
                )
                self.fail(
                    ctx,
                    f"the gantry did not reach its folded pose within {settle_timeout:.0f} s "
                    f"(max joint error {worst:.4f} > {self.tol})",
                )
            else:
                node.get_logger().warn(
                    f"[{self.name}] Waiting for gantry to reach origin "
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
        if not ctx.get("manipulator_folding_success") and not ctx.get("error_triggered"):
            return None
        if self.final_fold and not ctx.get("error_triggered"):
            # Nothing left to drill and the gantry is folded: go home without
            # asking the operator to pick a state.
            return "HomePosition"
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
