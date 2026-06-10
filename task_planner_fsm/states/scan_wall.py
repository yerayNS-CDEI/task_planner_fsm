from ..state import State
from ..utils.column_control import ColumnController
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.action import GoalResponse, CancelResponse
from rclpy.task import Future
from rclpy.duration import Duration
import rclpy.time
from arm_control.srv import SendPosition
import subprocess, os, signal
from math import atan2, sin, cos
import time

class ScanWall(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False        # scan-phase flag (base sweep)
        self.finished = False
        self.goal_sent = False
        self.waiting = False
        self.more_lines = False     # set when the wall has further lines to scan

        # Pre-approach (base fixed): column to line height + unfolded_fsm pose.
        self.column = ColumnController(self.name)
        self.position_client = None
        self.pose_sent = False
        self.pose_reached = False
        self.column_commanded = False
        self.preapproach_done = False
        self.pose_future = None
        self.preapproach_verbose = False
        self.current_line_z = None

        # Post-scan retraction (after the base sweep, before transitioning):
        # re-send the pose so the arm pulls back from the wall, and retract the
        # column on the last line of the wall.
        self.scan_swept = False
        self.postscan_done = False
        self.retract_pose_sent = False
        self.retract_future = None
        self.arm_retracted = False
        self.retract_verbose = False
        self.column_retract_commanded = False
        self.sweep_target_point = None   # endpoint the current sweep drives to

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering scanning state.")
        self.started = False
        self.finished = False
        self.goal_sent = False
        self.waiting = False
        self.more_lines = False
        self.pose_sent = False
        self.pose_reached = False
        self.column_commanded = False
        self.preapproach_done = False
        self.pose_future = None
        self.preapproach_verbose = False
        self.current_line_z = None
        self.scan_swept = False
        self.postscan_done = False
        self.retract_pose_sent = False
        self.retract_future = None
        self.arm_retracted = False
        self.retract_verbose = False
        self.column_retract_commanded = False
        self.sweep_target_point = None
        ctx["error_triggered"] = False

        self.column.reset()
        self.column.configure(node, ctx)

        if self.position_client is None:
            self.position_client = node.create_client(SendPosition, "/send_position")

        if ctx.get("nav_client") is None:
            node.get_logger().info(f"[{self.name}] Navigation client missing. Creating one for scan trajectory.")
            ctx["nav_client"] = ActionClient(node, NavigateToPose, "/navigate_to_pose")
            if not ctx["nav_client"].wait_for_server(timeout_sec=10.0):
                node.get_logger().error(f"[{self.name}] NavigateToPose action server not available.")
                ctx["error_triggered"] = True
                return

        # NOTE: the sensor-alignment processes (arduino_sensors_sim, align_ee_to_wall)
        # are intentionally NOT started here. They are launched only once the arm is
        # pre-positioned at the line height and the base is about to actively scan
        # (see _start_arm_processes, called from the scan phase).

    # ------------------------------------------------------------------
    # Per-line resolution
    # ------------------------------------------------------------------
    def _resolve_current_line_z(self, ctx):
        """Return the z height of the line currently being scanned.

        Falls back to the wall's scan-line z when per-line heights are absent
        (e.g. bootstrapped contexts), preserving single-pass behaviour.
        """
        lines = ctx.get("current_wall_scan_lines")
        if not lines:
            wall_data = ctx.get("target_scan_wall")
            fallback_z = wall_data[0][2] if wall_data else 0.0
            lines = [fallback_z]
            ctx["current_wall_scan_lines"] = lines
            ctx["current_line_idx"] = 0
        idx = ctx.get("current_line_idx", 0)
        idx = max(0, min(idx, len(lines) - 1))
        return lines[idx]

    # ------------------------------------------------------------------
    # Arm process control (scan phase only)
    # ------------------------------------------------------------------
    def _start_arm_processes(self, ctx):
        """Activate arduino_sensors_sim + align_ee_to_wall for the active scan."""
        node = ctx["node"]
        arm_procs = [
            ("arduino_sensors_proc", ["ros2", "run", "arm_control", "arduino_sensors_sim", "--ros-args", "-p", "autostart:=true"], "arduino_sensors_sim"),
            ("align_ee_proc",        ["ros2", "run", "arm_control", "align_ee_to_wall"],    "align_ee_to_wall"),
        ]
        for proc_key, cmd_args, proc_name in arm_procs:
            if ctx.get(proc_key) and ctx[proc_key].poll() is None:
                node.get_logger().info(f"[{self.name}] '{proc_name}' ya estaba activo (pid={ctx[proc_key].pid}).")
            else:
                try:
                    log_file = open(f"/tmp/{proc_name}.log", "w")
                    ctx[proc_key] = subprocess.Popen(
                        cmd_args,
                        preexec_fn=os.setsid,
                        stdout=log_file,
                        stderr=log_file
                    )
                    node.get_logger().info(
                        f"[{self.name}] '{proc_name}' activado (pid={ctx[proc_key].pid})."
                    )
                except Exception as e:
                    node.get_logger().error(
                        f"[{self.name}] No se pudo activar '{proc_name}': {e}"
                    )
                    ctx["error_triggered"] = True
                    return False
        return True

    def _stop_arm_processes(self, ctx):
        node = ctx["node"]
        for proc_key, proc_name in [
            ("arduino_sensors_proc", "arduino_sensors_sim"),
            ("align_ee_proc",        "align_ee_to_wall"),
        ]:
            proc = ctx.get(proc_key)
            if proc and proc.poll() is None:
                node.get_logger().info(f"[{self.name}] Deteniendo '{proc_name}'...")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    node.get_logger().warn(f"[{self.name}] Forzado SIGKILL a '{proc_name}'.")
            ctx[proc_key] = None

    # ------------------------------------------------------------------
    # Column height from the map-frame line z
    # ------------------------------------------------------------------
    def _lookup_ee_world_z(self, ctx):
        """Return the current end-effector height in the map frame, or None."""
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        ee_frames = [
            "arm_tool0", "arm_wrist_3_link", "arm_ee_link", "arm_flange",
            "tool0", "wrist_3_link", "ee_link", "flange",
        ]
        for frame in ee_frames:
            try:
                if tf_buffer.can_transform("map", frame, rclpy.time.Time(), Duration(seconds=0.2)):
                    tf = tf_buffer.lookup_transform("map", frame, rclpy.time.Time(), Duration(seconds=0.5))
                    return float(tf.transform.translation.z)
            except Exception:
                continue
        return None

    def _column_target_for_line(self, ctx, line_z):
        """Column extension that places the EE at the map-frame ``line_z``.

        The column raises the arm 1:1, so the required extension is the current
        column height plus the gap between the desired line z and the EE's
        current map-frame height (measured via TF while the arm is in the
        unfolded_fsm pose). Falls back to the static mapping if TF is missing.
        """
        node = ctx["node"]
        column_current = float(ctx.get("column_current_height", 0.0))
        ee_z = self._lookup_ee_world_z(ctx)
        if ee_z is None:
            node.get_logger().warn(
                f"[{self.name}] EE transform unavailable; using static column mapping."
            )
            return self.column.height_for_line_z(line_z)

        target = column_current + (float(line_z) - ee_z)
        clamped = max(self.column.column_min_height_m, min(target, self.column.column_max_height_m))
        node.get_logger().info(
            f"[{self.name}] Column calc: line_z={line_z:.3f}m, EE_z={ee_z:.3f}m, "
            f"column_now={column_current:.3f}m -> target={target:.3f}m (clamped {clamped:.3f}m)."
        )
        return clamped

    # ------------------------------------------------------------------
    # Pre-approach (base fixed)
    # ------------------------------------------------------------------
    def _run_pre_approach(self, ctx):
        """Reset the arm to the unfolded_fsm scanning pose, then raise/lower the
        column to the line height. The base does not move during this phase, and
        the column is only commanded once the arm motion has fully finished.

        Sets self.preapproach_done = True once the arm and column are in position.
        """
        node = ctx["node"]

        # --- Phase 1: send the arm to the unfolded_fsm pose and wait for it. ---
        if not self.pose_reached:
            # Step 1a: send the named pose.
            if not self.pose_sent:
                if not self.position_client.service_is_ready():
                    node.get_logger().warn(f"[{self.name}] Waiting for /send_position service...")
                    return
                self.current_line_z = self._resolve_current_line_z(ctx)
                node.get_logger().info(
                    f"[{self.name}] Pre-approach for line z={self.current_line_z:.3f}m "
                    f"(line {ctx.get('current_line_idx', 0) + 1}/"
                    f"{len(ctx.get('current_wall_scan_lines', [self.current_line_z]))}): "
                    f"sending unfolded_fsm pose."
                )
                request = SendPosition.Request()
                request.position_name = "unfolded_fsm"
                ctx["execution_status"] = False
                ctx["planner_goal_failed"] = False
                self.pose_future = self.position_client.call_async(request)
                self.pose_sent = True
                return

            # Step 1b: confirm the service accepted the request.
            if self.pose_future is not None:
                if not self.pose_future.done():
                    return
                try:
                    response = self.pose_future.result()
                    if not response.success:
                        node.get_logger().error(f"[{self.name}] Pre-approach pose rejected: {response.message}")
                        ctx["error_triggered"] = True
                        return
                    node.get_logger().info(f"[{self.name}] Pre-approach pose accepted: {response.message}")
                except Exception as e:
                    node.get_logger().error(f"[{self.name}] Pre-approach service exception: {e}")
                    ctx["error_triggered"] = True
                    return
                self.pose_future = None

            # Step 1c: wait for the arm to actually reach the pose.
            if ctx.get("planner_goal_failed"):
                ctx["planner_goal_failed"] = False
                node.get_logger().error(f"[{self.name}] Planner reported failure during pre-approach.")
                ctx["error_triggered"] = True
                return

            if ctx.get("execution_status") is True:
                ctx["execution_status"] = False
                self.pose_reached = True
                node.get_logger().info(f"[{self.name}] Arm at unfolded_fsm pose.")
            else:
                if not self.preapproach_verbose:
                    node.get_logger().info(f"[{self.name}] Waiting for arm to reach pre-approach pose...")
                    self.preapproach_verbose = True
                return

        # --- Phase 2: arm motion finished — now move the column. ---
        if not self.column_commanded:
            target_h = self._column_target_for_line(ctx, self.current_line_z)
            node.get_logger().info(
                f"[{self.name}] Arm settled; commanding column to {target_h:.3f}m "
                f"for line z={self.current_line_z:.3f}m."
            )
            if not self.column.command(node, ctx, target_h):
                ctx["error_triggered"] = True
            self.column_commanded = True
            return

        if not self.column.reached_target(node, ctx):
            if self.column.timed_out(node):
                node.get_logger().error(f"[{self.name}] Column did not reach target in time.")
                ctx["error_triggered"] = True
            return

        self.preapproach_done = True
        node.get_logger().info(f"[{self.name}] Pre-approach complete. Arm in scanning position.")

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("walls_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No walls left to scan.")
            ctx["scan_done"] = True
            self.finished = True
            return

        # Phase A: pre-approach (base fixed, sensors off).
        if not self.preapproach_done:
            self._run_pre_approach(ctx)
            return

        # Phase B: active scan (sensors on, base sweeps the line).
        if not self.scan_swept:
            self._run_scan(ctx)
            return

        # Phase C: post-scan retraction (arm pulls back, column retracts on last line).
        if not self.postscan_done:
            self._run_post_scan(ctx)
            return

    def _run_scan(self, ctx):
        node = ctx["node"]

        if not self.started:
            node.get_logger().info(f"[{self.name}] Initiating wall scan maneuver...")
            self.started = True

            # Activate sensor alignment now that the arm is located.
            if not self._start_arm_processes(ctx):
                return

            node.get_logger().info(f"[{self.name}] Waiting 10s for arm processes to stabilise...")
            time.sleep(10.0)

            wall_data = ctx.get("target_scan_wall", None)
            prev_target_point = ctx.get("target_scan_point", None)

            if not wall_data or not prev_target_point:
                node.get_logger().error(f"[{self.name}] Missing wall data or target point.")
                ctx["error_triggered"] = True
                return

            # Sweep to the wall end opposite the robot's current end. Updating
            # target_scan_point after each sweep (see result_callback) makes the
            # next line scan the other direction (serpentine), so the base
            # actually moves on every line.
            if all(abs(prev_target_point[i] - wall_data[0][i]) < 1e-6 for i in range(2)):
                target_point = wall_data[1]
            else:
                target_point = wall_data[0]
            self.sweep_target_point = target_point

            # Keep a FIXED base heading for the whole wall (computed on the first
            # line). The omnidirectional base then strafes back and forth without
            # turning, so the arm keeps facing the wall on every line. Recomputed
            # at line 0 so a new wall picks up its own heading.
            heading = ctx.get("scan_heading_yaw")
            if heading is None or ctx.get("current_line_idx", 0) == 0:
                heading = atan2(
                    target_point[1] - prev_target_point[1],
                    target_point[0] - prev_target_point[0],
                )
                ctx["scan_heading_yaw"] = heading
            qz = sin(heading / 2.0)
            qw = cos(heading / 2.0)

            # Construct PoseStamped goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_point[0]
            goal_msg.pose.pose.position.y = target_point[1]
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            nav_client = ctx.get("nav_client", None)
            if nav_client is None:
                node.get_logger().error(f"[{self.name}] Navigation client not found.")
                ctx["error_triggered"] = True
                return

            self._send_goal_future = nav_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            self.waiting = True

        elif self.waiting and self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                node.get_logger().error(f"[{self.name}] Goal was rejected.")
                ctx["error_triggered"] = True
                return
            node.get_logger().info(f"[{self.name}] Goal accepted. Waiting for result...")
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(lambda fut: self.result_callback(fut, ctx))

            self.waiting = False  # Dejar de esperar, ya está lanzado

    def goal_response_callback(self, future):
        pass

    def result_callback(self, future, ctx):
        node = ctx["node"]
        result = future.result().result
        status = future.result().status

        node.get_logger().info(f"[{self.name}] Scanning finished. Stopping arm processes for safety...")
        self._stop_arm_processes(ctx)
        time.sleep(2)   # delay to avoid errors in the arm goals

        # The robot is now at the end it swept to; record it so the next line
        # sweeps back toward the opposite end.
        if self.sweep_target_point is not None:
            ctx["target_scan_point"] = self.sweep_target_point

        # Advance to the next horizontal line on this wall. more_lines drives
        # both the self-loop and whether this was the last line of the wall.
        ctx["current_line_idx"] = ctx.get("current_line_idx", 0) + 1
        lines = ctx.get("current_wall_scan_lines", [])
        self.more_lines = ctx["current_line_idx"] < len(lines)
        if self.more_lines:
            node.get_logger().info(
                f"[{self.name}] Line done. {len(lines) - ctx['current_line_idx']} more line(s) "
                f"on this wall; will retract arm then re-enter for next height."
            )

        # The base sweep is done; hand control to the post-scan retraction phase.
        # walls_left is only decremented once that phase completes (see
        # _run_post_scan), so the column retract on the last line is not skipped.
        self.scan_swept = True

    def _run_post_scan(self, ctx):
        """Retract the arm from the wall (re-send the pose); on the wall's last
        line also retract the column. Sets self.finished when complete."""
        node = ctx["node"]

        # Step 1: pull the arm back from the wall by re-sending the pose.
        if not self.retract_pose_sent:
            if not self.position_client.service_is_ready():
                node.get_logger().warn(f"[{self.name}] Waiting for /send_position service (retract)...")
                return
            request = SendPosition.Request()
            request.position_name = "unfolded_fsm"
            ctx["execution_status"] = False
            ctx["planner_goal_failed"] = False
            self.retract_future = self.position_client.call_async(request)
            self.retract_pose_sent = True
            node.get_logger().info(f"[{self.name}] Re-sending pose to retract arm from wall.")
            return

        if self.retract_future is not None:
            if not self.retract_future.done():
                return
            try:
                response = self.retract_future.result()
                if not response.success:
                    node.get_logger().error(f"[{self.name}] Retract pose rejected: {response.message}")
                    ctx["error_triggered"] = True
                    return
                node.get_logger().info(f"[{self.name}] Retract pose accepted: {response.message}")
            except Exception as e:
                node.get_logger().error(f"[{self.name}] Retract service exception: {e}")
                ctx["error_triggered"] = True
                return
            self.retract_future = None

        if not self.arm_retracted:
            if ctx.get("planner_goal_failed"):
                ctx["planner_goal_failed"] = False
                node.get_logger().error(f"[{self.name}] Planner reported failure during arm retract.")
                ctx["error_triggered"] = True
                return
            if ctx.get("execution_status") is True:
                ctx["execution_status"] = False
                self.arm_retracted = True
                node.get_logger().info(f"[{self.name}] Arm retracted from wall.")
            else:
                if not self.retract_verbose:
                    node.get_logger().info(f"[{self.name}] Waiting for arm to retract...")
                    self.retract_verbose = True
                return

        # Step 2: on the last line of the wall, retract the column. Commanding
        # the minimum height is a no-op when the column was never extended.
        if not self.more_lines:
            if not self.column_retract_commanded:
                node.get_logger().info(f"[{self.name}] Last line of wall: retracting column.")
                if not self.column.command(node, ctx, self.column.column_min_height_m):
                    ctx["error_triggered"] = True
                self.column_retract_commanded = True
                return
            if not self.column.reached_target(node, ctx):
                if self.column.timed_out(node):
                    node.get_logger().error(f"[{self.name}] Column did not retract in time.")
                    ctx["error_triggered"] = True
                return

        # Post-scan retraction complete.
        self.postscan_done = True
        self.finished = True
        if not self.more_lines:
            ctx["walls_left"] -= 1
            if ctx.get("walls_left", 0) <= 0:
                node.get_logger().info(f"[{self.name}] No walls left to scan.")
                ctx["scan_done"] = True

    def on_exit(self, ctx):
        ## Desactivacion de arduino_sensors_sim y align_ee_to_wall (safety net)
        self._stop_arm_processes(ctx)

    def check_transition(self, ctx):
        if self.finished and self.more_lines:
            return "ScanWall"        # same wall, next height (base stays put)
        if self.finished:
            return "SensorDataProcessing"
        if ctx.get("error_triggered"):
            return "Error"
        return None
