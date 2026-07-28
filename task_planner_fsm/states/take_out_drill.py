import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ..state import State
from ..utils.gantry_kinematics import HORIZONTAL_LIMITS

NEXT_STATE_OPTIONS = [
    "SuctionDrillStop",
    "Error",
]

# gantry_position_controller joints, in command order (must match
# config/diffdrive_controllers.yaml).
GANTRY_JOINTS = ["stage1_lift_joint", "stage2_lift_joint", "stage3_rotate_joint", "stage4_horizontal_joint"]

# Index of the horizontal slide (stage 4) — the stage that feeds/withdraws the bit.
SLIDE_IDX = 3


class TakeOutDrill(State):
    """Withdraw the drill bit from the wall — the reverse of ``Drilling``.

    Stage 4 is pulled back out at the same velocity used to drill
    (``drill_velocity_mps``) until the bit is completely clear of the wall:
    ``drill_takeout_clearance_m`` (default 1 cm) beyond the pose where the tip was
    touching the wall at the start of drilling (``gantry_drill_start_slide``, stored
    by ``Drilling``). The lifts and rotary stage are frozen; only the slide moves.

    ``DrillRetract`` afterwards returns the whole arm to the pre-approach pose.
    """

    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.ctx = None
        self.cmd_pub = None
        self.joint_sub = None
        self.latest_joints = None
        self.control_timer = None
        self.outcome = None  # None | "reached" | "error"

    def on_enter(self, ctx):
        node = ctx["node"]
        self.node = node
        self.ctx = ctx
        node.get_logger().info(f"[{self.name}] Withdrawing the drill bit from the wall.")
        ctx["take_out_drill_success"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        # ── Tunable parameters (overridable via ROS params -> ctx) ──
        self.velocity = abs(float(ctx.get("drill_velocity_mps", 0.005)))          # m/s (== drilling)
        self.clearance = abs(float(ctx.get("drill_takeout_clearance_m", 0.01)))   # m past touch pose
        self.sign = float(ctx.get("drill_slide_sign", -1.0))                      # slide dir into wall
        self.control_rate = float(ctx.get("drill_control_rate_hz", 50.0))         # Hz
        self.settle_timeout = float(ctx.get("drill_settle_timeout_s", 5.0))       # s after full pull
        self.command_timeout = float(ctx.get("gantry_command_timeout", 15.0))
        self.tol = float(ctx.get("gantry_position_tolerance", 0.02))
        # Fallback depth if the drilling start pose was never recorded (manual entry).
        self.depth = abs(float(ctx.get("drill_depth_m", 0.15)))

        self.topic = str(ctx.get("gantry_command_topic", "/gantry_position_controller/commands"))
        self.joint_names = list(ctx.get("gantry_joint_names", GANTRY_JOINTS))

        # ── Per-run state ──
        self.latest_joints = None
        self.outcome = None
        self.started = False
        self.enter_time = time.time()
        self.held = None
        self.start_slide = None
        self.target_slide = None
        self.distance = None
        self.dir = 1.0
        self.ramp_duration = None
        self.move_t0 = None
        self._logged_outcome = False

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

        period = 1.0 / self.control_rate if self.control_rate > 0 else 0.02
        self.control_timer = node.create_timer(period, self._control_step)

    def on_exit(self, ctx):
        if self.control_timer is not None:
            self.node.destroy_timer(self.control_timer)
            self.control_timer = None

    def _on_joint_states(self, msg):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def _publish(self, slide):
        cmd = list(self.held)
        cmd[SLIDE_IDX] = slide
        msg = Float64MultiArray()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def _control_step(self):
        if self.outcome is not None:
            return
        node = self.node
        now = time.time()
        slide_joint = self.joint_names[SLIDE_IDX]

        # ── Startup: wait for joint feedback + a controller subscriber. ──
        if not self.started:
            ready = (
                self.latest_joints is not None
                and all(j in self.latest_joints for j in self.joint_names)
                and self.cmd_pub.get_subscription_count() >= 1
            )
            if not ready:
                if now - self.enter_time > self.command_timeout:
                    node.get_logger().error(
                        f"[{self.name}] Gantry not ready after {self.command_timeout:.0f}s "
                        f"(joint states / gantry_position_controller subscriber missing)."
                    )
                    self.outcome = "error"
                else:
                    node.get_logger().warn(
                        f"[{self.name}] Waiting for joint states and the "
                        f"gantry_position_controller on {self.topic}...",
                        throttle_duration_sec=2.0,
                    )
                return

            self.held = [float(self.latest_joints[j]) for j in self.joint_names]
            self.start_slide = self.held[SLIDE_IDX]

            # Withdraw direction is the opposite of the drilling feed.
            outward = -self.sign
            touch_slide = self.ctx.get("gantry_drill_start_slide")
            if touch_slide is None:
                # No recorded touch pose: assume we are at full drilled depth and
                # back out the whole depth plus the clearance.
                node.get_logger().warn(
                    f"[{self.name}] No recorded drilling start pose; withdrawing "
                    f"{self.depth + self.clearance:.3f} m (depth + clearance) from here."
                )
                raw_target = self.start_slide + outward * (self.depth + self.clearance)
            else:
                raw_target = float(touch_slide) + outward * self.clearance

            self.target_slide = max(HORIZONTAL_LIMITS[0], min(HORIZONTAL_LIMITS[1], raw_target))
            self.distance = abs(self.target_slide - self.start_slide)
            self.dir = 1.0 if self.target_slide >= self.start_slide else -1.0
            self.ramp_duration = self.distance / self.velocity if self.velocity > 0 else 0.0
            self.move_t0 = now
            self.started = True
            self._publish(self.start_slide)
            node.get_logger().info(
                f"[{self.name}] Withdrawing stage 4 {self.start_slide:.3f} -> "
                f"{self.target_slide:.3f} m ({self.distance:.3f} m at {self.velocity:.4f} m/s, "
                f"~{self.ramp_duration:.1f} s)."
            )
            return

        if slide_joint not in self.latest_joints:
            return  # transient gap in joint states; retry next tick
        actual = float(self.latest_joints[slide_joint])

        elapsed = now - self.move_t0
        moved = min(self.distance, self.velocity * elapsed)
        self._publish(self.start_slide + self.dir * moved)

        # Done once the bit has physically reached the clear-of-wall target.
        if abs(actual - self.target_slide) <= self.tol:
            self.outcome = "reached"
            return

        # Full stroke commanded but not settled: give it a little time, then fail.
        if elapsed - self.ramp_duration > self.settle_timeout:
            node.get_logger().error(
                f"[{self.name}] Stage 4 did not reach the withdraw target "
                f"(at {actual:.3f} m, target {self.target_slide:.3f} m) after the full stroke."
            )
            self.outcome = "error"
            return

    def run(self, ctx):
        node = ctx["node"]
        if self.outcome == "reached":
            if not self._logged_outcome:
                node.get_logger().info(f"[{self.name}] Drill bit clear of the wall.")
                self._logged_outcome = True
            ctx["take_out_drill_success"] = True
        elif self.outcome == "error":
            ctx["error_triggered"] = True

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        if ctx.get("take_out_drill_success"):
            return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
        return None
