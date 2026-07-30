import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ..state import State
from ..utils.gantry_kinematics import HORIZONTAL_LIMITS

NEXT_STATE_OPTIONS = [
    "TakeOutDrill",
    "TargetSelection",
    "Error",
]

# gantry_position_controller joints, in command order (must match
# config/diffdrive_controllers.yaml).
GANTRY_JOINTS = ["stage1_lift_joint", "stage2_lift_joint", "stage3_rotate_joint", "stage4_horizontal_joint"]

# Index of the horizontal slide (stage 4) — the only stage that feeds the drill in.
SLIDE_IDX = 3


class Drilling(State):
    """Feed the drill into the wall by advancing gantry stage 4 at a set velocity.

    The drill tip is already touching the wall (from ``DrillApproach``), so this
    state ramps the ``stage4_horizontal_joint`` position setpoint from its current
    value inward by ``drill_depth_m`` at ``drill_velocity_mps``. The gantry uses a
    position ``ForwardCommandController`` (no velocity interface), so the velocity is
    realised by advancing the setpoint each control tick; the two lifts and the
    rotary stage are frozen at the values held when drilling began.

    The actual slide motion (from ``/joint_states``) is monitored throughout:

    * When the slide has physically moved ``drill_depth_m`` the hole is complete
      (-> ``TakeOutDrill`` via the picker).
    * If the slide barely moves once commanded, or its motion stalls part-way
      through, the material is deemed **not drillable**. The bit is first pulled
      slowly back to the approach pose and then a further ``drill_extra_retract_m``
      clear of the wall, after which ``material_not_drillable`` is set and the FSM
      routes back to ``TargetSelection`` (which already excludes this target, so the
      next-nearest point is picked).
    """

    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.ctx = None
        self.cmd_pub = None
        self.joint_sub = None
        self.latest_joints = None
        self.control_timer = None
        self.outcome = None  # None | "reached" | "not_drillable" | "error"
        # Concrete cause recorded by _control_step when outcome == "error"; run()
        # hands it to self.fail so the panel shows why drilling was abandoned.
        self.error_reason = None

    def on_enter(self, ctx):
        node = ctx["node"]
        self.node = node
        self.ctx = ctx
        node.get_logger().info(f"[{self.name}] Drilling: feeding stage 4 into the wall.")
        ctx["drill_success"] = False
        ctx["material_not_drillable"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        # ── Tunable parameters (overridable via ROS params -> ctx) ──
        self.velocity = abs(float(ctx.get("drill_velocity_mps", 0.005)))     # m/s
        self.depth = abs(float(ctx.get("drill_depth_m", 0.15)))              # m
        self.depth_tol = float(ctx.get("drill_depth_tol_m", 0.002))         # m
        self.sign = float(ctx.get("drill_slide_sign", -1.0))                # slide dir into wall
        self.control_rate = float(ctx.get("drill_control_rate_hz", 50.0))   # Hz
        self.stall_window = float(ctx.get("drill_stall_window_s", 2.0))     # s
        self.stall_fraction = float(ctx.get("drill_stall_fraction", 0.3))   # of expected travel
        self.stall_grace = float(ctx.get("drill_stall_grace_s", 1.5))       # s before checking
        self.settle_timeout = float(ctx.get("drill_settle_timeout_s", 5.0)) # s after full feed
        self.command_timeout = float(ctx.get("gantry_command_timeout", 15.0))
        self.tol = float(ctx.get("gantry_position_tolerance", 0.02))
        # Not-drillable retraction: pull slowly back to the approach pose, then this
        # much further clear of the wall, before handing back to TargetSelection.
        self.retract_velocity = abs(float(ctx.get("drill_retract_velocity_mps", 0.005)))  # m/s
        self.extra_retract = abs(float(ctx.get("drill_extra_retract_m", 0.10)))           # m
        self.retract_settle_timeout = float(ctx.get("drill_retract_settle_timeout_s", 5.0))

        self.topic = str(ctx.get("gantry_command_topic", "/gantry_position_controller/commands"))
        self.joint_names = list(ctx.get("gantry_joint_names", GANTRY_JOINTS))

        # ── Per-run state ──
        self.latest_joints = None
        self.outcome = None
        self.error_reason = None
        self.phase = "feeding"  # "feeding" | "retracting"
        self.started = False
        self.enter_time = time.time()
        self.held = None
        self.start_slide = None
        self.target_slide = None
        self.achievable_depth = None
        self.ramp_duration = None
        self.drill_t0 = None
        self.history = []
        self.final_depth = None
        self.last_cmd_slide = None
        # Retraction (populated when a not-drillable surface triggers the pull-out).
        self.retract_start_slide = None
        self.retract_target = None
        self.retract_dir = 1.0
        self.retract_distance = None
        self.retract_ramp_duration = None
        self.retract_t0 = None
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

        # Dedicated high-rate loop so the feed velocity is smooth and stalls are
        # caught promptly, independent of the ~1 Hz FSM step. Single-threaded
        # executor => it never runs concurrently with run()/joint callbacks.
        period = 1.0 / self.control_rate if self.control_rate > 0 else 0.02
        self.control_timer = node.create_timer(period, self._control_step)

    def on_exit(self, ctx):
        if self.control_timer is not None:
            self.node.destroy_timer(self.control_timer)
            self.control_timer = None

    def _on_joint_states(self, msg):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def _publish(self, slide):
        self.last_cmd_slide = slide
        cmd = list(self.held)
        cmd[SLIDE_IDX] = slide
        msg = Float64MultiArray()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    # ── High-rate control + monitoring loop ───────────────────────────────
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
                    self.error_reason = (
                        f"the gantry was not ready after {self.command_timeout:.0f} s "
                        f"(no joint states or no gantry_position_controller)"
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
            # Remember the pose at which the tip is touching the wall (drilling start)
            # so TakeOutDrill can pull the bit back out to just clear of it.
            self.ctx["gantry_drill_start_joints"] = list(self.held)
            self.ctx["gantry_drill_start_slide"] = self.start_slide
            raw_target = self.start_slide + self.sign * self.depth
            self.target_slide = max(HORIZONTAL_LIMITS[0], min(HORIZONTAL_LIMITS[1], raw_target))
            self.achievable_depth = abs(self.target_slide - self.start_slide)

            if self.achievable_depth < self.depth_tol:
                node.get_logger().error(
                    f"[{self.name}] Stage 4 already at its travel limit "
                    f"({self.start_slide:.3f} m); cannot drill. Check the approach pose."
                )
                self.error_reason = (
                    f"the drill slide is already at its travel limit "
                    f"({self.start_slide:.3f} m), leaving no depth to drill"
                )
                self.outcome = "error"
                return
            if self.achievable_depth < self.depth - 1e-6:
                node.get_logger().warn(
                    f"[{self.name}] Requested depth {self.depth:.3f} m exceeds stage-4 travel; "
                    f"drilling the reachable {self.achievable_depth:.3f} m instead."
                )

            self.ramp_duration = self.achievable_depth / self.velocity if self.velocity > 0 else 0.0
            self.drill_t0 = now
            self.history = [(now, self.start_slide)]
            self.started = True
            self._publish(self.start_slide)
            node.get_logger().info(
                f"[{self.name}] Feeding stage 4 {self.start_slide:.3f} -> {self.target_slide:.3f} m "
                f"({self.achievable_depth:.3f} m at {self.velocity:.4f} m/s, "
                f"~{self.ramp_duration:.1f} s)."
            )
            return

        if slide_joint not in self.latest_joints:
            return  # transient gap in joint states; retry next tick
        actual = float(self.latest_joints[slide_joint])

        if self.phase == "feeding":
            self._feed_step(node, now, actual)
        elif self.phase == "retracting":
            self._retract_step(node, now, actual)

    def _feed_step(self, node, now, actual):
        elapsed = now - self.drill_t0
        commanded_move = min(self.achievable_depth, self.velocity * elapsed)
        self._publish(self.start_slide + self.sign * commanded_move)

        # ── Success: the slide has physically travelled the full depth. ──
        actual_move = abs(actual - self.start_slide)
        if actual_move >= self.achievable_depth - self.depth_tol:
            self.final_depth = actual_move
            self.outcome = "reached"
            return

        # ── Monitor actual motion for a non-drillable surface. ──
        self.history.append((now, actual))
        while len(self.history) > 1 and now - self.history[0][0] > self.stall_window:
            self.history.pop(0)

        still_feeding = commanded_move < self.achievable_depth - 1e-9
        if still_feeding:
            # Actively pushing deeper: the actual slide should be keeping up with the
            # commanded feed. If, past the initial grace, its progress over the recent
            # window falls well below what the feed velocity should have produced, the
            # material is resisting -> not drillable (covers "never moved" and
            # "stopped suddenly in the middle").
            if elapsed > self.stall_grace:
                t0, s0 = self.history[0]
                window_dt = now - t0
                if window_dt >= self.stall_window * 0.8:
                    progress = abs(actual - s0)
                    expected = self.velocity * window_dt
                    if progress < self.stall_fraction * expected:
                        node.get_logger().warn(
                            f"[{self.name}] Stage 4 barely moving ({progress*1000:.2f} mm in "
                            f"{window_dt:.1f}s, expected ~{expected*1000:.2f} mm) at "
                            f"{actual_move*1000:.1f}/{self.achievable_depth*1000:.1f} mm depth."
                        )
                        self._begin_retract(node, now)
                        return
        else:
            # Full depth commanded; give the slide time to settle onto the target.
            if elapsed - self.ramp_duration > self.settle_timeout:
                node.get_logger().warn(
                    f"[{self.name}] Stage 4 stopped {actual_move*1000:.1f} mm short of the "
                    f"{self.achievable_depth*1000:.1f} mm target after full feed; "
                    f"treating the surface as not drillable."
                )
                self._begin_retract(node, now)
                return

    def _begin_retract(self, node, now):
        """Switch from feeding to a slow pull-out to the approach pose + extra clearance."""
        self.phase = "retracting"
        self.retract_start_slide = self.last_cmd_slide
        outward = -self.sign  # opposite of the feed direction (out of the wall)
        raw_target = self.start_slide + outward * self.extra_retract
        self.retract_target = max(HORIZONTAL_LIMITS[0], min(HORIZONTAL_LIMITS[1], raw_target))
        self.retract_distance = abs(self.retract_target - self.retract_start_slide)
        self.retract_dir = 1.0 if self.retract_target >= self.retract_start_slide else -1.0
        self.retract_ramp_duration = (
            self.retract_distance / self.retract_velocity if self.retract_velocity > 0 else 0.0
        )
        self.retract_t0 = now
        node.get_logger().warn(
            f"[{self.name}] Not drillable: retracting stage 4 {self.retract_start_slide:.3f} -> "
            f"{self.retract_target:.3f} m (approach pose + {self.extra_retract:.3f} m clear) "
            f"at {self.retract_velocity:.4f} m/s before returning to TargetSelection."
        )

    def _retract_step(self, node, now, actual):
        elapsed = now - self.retract_t0
        moved = min(self.retract_distance, self.retract_velocity * elapsed)
        self._publish(self.retract_start_slide + self.retract_dir * moved)

        if abs(actual - self.retract_target) <= self.tol:
            node.get_logger().info(
                f"[{self.name}] Retracted clear of the wall (stage 4 at {actual:.3f} m); "
                f"returning to TargetSelection."
            )
            self.outcome = "not_drillable"
            return
        if elapsed - self.retract_ramp_duration > self.retract_settle_timeout:
            node.get_logger().warn(
                f"[{self.name}] Retract did not fully settle (stage 4 at {actual:.3f} m, "
                f"target {self.retract_target:.3f} m); continuing to TargetSelection anyway."
            )
            self.outcome = "not_drillable"
            return

    def run(self, ctx):
        node = ctx["node"]
        if self.outcome == "reached":
            if not self._logged_outcome:
                node.get_logger().info(
                    f"[{self.name}] Reached target depth ({self.final_depth*1000:.1f} mm); "
                    f"drilling complete."
                )
                self._logged_outcome = True
            self.set_activity(
                ctx,
                f"Hole drilled to {self.final_depth * 1000:.0f} mm",
                progress_current=int(round(self.final_depth * 1000)),
                progress_total=int(round(self.achievable_depth * 1000)),
            )
            ctx["drill_success"] = True
        elif self.outcome == "not_drillable":
            self.set_activity(
                ctx,
                "Material is not drillable; the bit has been pulled clear of the wall",
                level="warn",
            )
            ctx["material_not_drillable"] = True
        elif self.outcome == "error":
            self.fail(ctx, self.error_reason or "the drilling feed could not be run")
        else:
            self._report_progress(ctx)

    def _report_progress(self, ctx):
        """Describe the live feed/retract for the panel (the motion itself is driven
        by the high-rate _control_step timer, not by run())."""
        if not self.started:
            self.set_activity(ctx, "Waiting for the gantry before drilling")
            return

        slide_joint = self.joint_names[SLIDE_IDX]
        actual = (self.latest_joints or {}).get(slide_joint)

        if self.phase == "retracting":
            self.set_activity(
                ctx,
                "Material is not drillable; pulling the bit clear of the wall",
                level="warn",
            )
            return

        if actual is None:
            self.set_activity(ctx, "Drilling into the wall")
            return
        depth_mm = abs(float(actual) - self.start_slide) * 1000.0
        total_mm = self.achievable_depth * 1000.0
        self.set_activity(
            ctx,
            f"Drilling into the wall ({depth_mm:.0f} / {total_mm:.0f} mm)",
            progress_current=int(round(depth_mm)),
            progress_total=int(round(total_mm)),
        )

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        # Not drillable auto-routes back to target selection (no manual picker), which
        # already excludes this target and will choose the next-nearest one.
        if ctx.get("material_not_drillable"):
            ctx["node"].get_logger().warn(
                f"[{self.name}] Surface not drillable; returning to TargetSelection."
            )
            return "TargetSelection"
        if ctx.get("drill_success"):
            return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
        return None
