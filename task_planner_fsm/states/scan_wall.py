from ..state import State
from ..utils.column_control import ColumnController
from ..utils.costmap_utils import (
    COSTMAP_WAIT_TIMEOUT_S,
    base_standoff_goal,
    nav_bt_xml,
    reachable_wall_segments,
    publish_wall_segment_markers,
)
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.action import GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from rclpy.duration import Duration
import rclpy.time
from arm_control.srv import SendPosition
from ur_msgs.srv import SetForceMode
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
import subprocess, os, signal
import math
from math import atan2, sin, cos
import time
import requests

class ScanWall(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False        # scan-phase flag (base sweep)
        self.finished = False
        self.goal_sent = False
        self.waiting = False
        self.more_lines = False     # set when the wall has further lines to scan

        # Base parking: align the diff-drive chassis with the turret's (wall-facing)
        # heading via sim_controller's /sim_controller/park_now service. The turret is
        # squared to the wall by NavigateToTarget (Nav2 drives turret_footprint), but the
        # chassis wheels sit at an arbitrary heading, so park to face the whole robot at
        # the wall. Parking runs PER SEGMENT, right after the base reaches the segment
        # start and just before the sweep (see the "park" seg-phase in _run_scan): doing
        # it there means no base repositioning follows the alignment to undo it. The arm
        # is already in the unfolded_fsm pose by then; the turret joint compensates to
        # hold the arm world-stationary while the chassis rotates.
        # park_now is live-gated by the controller's enable_park_service parameter
        # (kept false by default): this state flips it true via /set_parameters just
        # for the maneuver, then false again. Driven by _park_phase through
        # enable -> request -> settle -> disable. See _run_parking.
        self.park_client = None            # ~/park_now Trigger client (created once)
        self.park_enable_client = None     # /set_parameters client (created once)
        self.park_done = False
        self._park_phase = "enable"
        self.park_future = None
        self.park_param_future = None
        self._park_saw_active = False
        self._park_wait_start = None

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
        self._costmap_wait_start = None  # per-line costmap-readiness poll timer
        self._sweep_qz = 0.0
        self._sweep_qw = 1.0

        # Reachability-first sweep: the scan line is split into the sub-segments
        # where a base cell exists within arm reach (reachable_wall_segments);
        # each segment is swept press->release, unreachable gaps are skipped by
        # transiting (sensors off) to the next segment.
        self._segments = None        # [((sx,sy,z),(ex,ey,z)), ...] robot-end first
        self._seg_idx = 0
        self._seg_phase = "transit"  # transit|transit_wait|park|sweep_setup|press_settle|sweep_wait
        self._segments_ok = 0        # segments actually swept on this line
        self._last_swept_point = None
        self._nav_status = None      # set by _on_nav_result / rejection
        self._sweep_from = None      # wall end the sweep starts from (robot's end)
        self._sweep_to = None        # wall end the sweep heads toward

        # Force mode (real robot only): press the GPR wheel against the wall (Z)
        # while the distance-sensor alignment holds the plate orientation.
        self.force_mode_start_client = None
        self.force_mode_stop_client = None
        self.force_mode_active = False

        # Wall-contact gate (real robot only): before starting the GPR and moving
        # the base, wait until force_mode has driven the GPR wheel against the wall,
        # detected via the TCP force/torque sensor (ur_ros2_driver's
        # /force_torque_sensor_broadcaster/wrench). The press is along task-frame
        # Z (arm_tool0), and the broadcaster reports the wrench in the arm_tool0
        # frame, so wrench.force.z is the wall-normal press force. Pressing into the
        # wall reads negative on Z, so contact = Fz reaching the commanded press
        # (~-5 N). See _wall_contact_ready.
        self.ft_topic = "/force_torque_sensor_broadcaster/wrench"
        self._press_settle_start = None

        # GPR (GP Proceq8800) HTTP API: connect + run a LINE_SCAN measurement
        # while the wheel is pressed against the wall. Real robot only (mirrors
        # force_mode gating); ctx overrides allow bench testing. See gpr_api memory.
        self.gpr_base_url = "http://192.168.42.33:9000"
        self.gpr_serial = "GP88-007-0081"
        self.gpr_timeout = 30.0
        self.gpr_measurement_active = False
        self.gpr_line_active = False

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering scanning state.")
        self.started = False
        self.finished = False
        self.goal_sent = False
        self.waiting = False
        self.more_lines = False
        self.park_done = False
        self._park_phase = "enable"
        self.park_future = None
        self.park_param_future = None
        self._park_saw_active = False
        self._park_wait_start = None
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
        self._costmap_wait_start = None
        self._segments = None
        self._seg_idx = 0
        self._seg_phase = "transit"
        self._segments_ok = 0
        self._last_swept_point = None
        self._nav_status = None
        self._sweep_from = None
        self._sweep_to = None
        self.force_mode_active = False
        self._press_settle_start = None
        self.gpr_measurement_active = False
        self.gpr_line_active = False
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

        # NOTE: the alignment processes (distance sensors + wall_parallel_controller)
        # and, on the real robot, force_mode are intentionally NOT started here. They
        # are activated only once the arm is pre-positioned at the line height and the
        # base is about to actively scan (see _start_arm_processes / _start_force_mode,
        # called from the scan phase).

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
        """Activate the distance-sensor reader + wall_parallel_controller for the
        active scan. Sim uses the simulated sensors; real uses the Arduino reader.
        """
        node = ctx["node"]
        use_sim = bool(ctx.get("sim", False))
        if use_sim:
            sensor_cmd = ["ros2", "run", "arm_control", "arduino_sensors_sim", "--ros-args",
                          "-p", "autostart:=true", "-p", "publish_rate:=5.0", "-p", "batch_size:=2"]
            sensor_name = "arduino_sensors_sim"
        else:
            sensor_cmd = ["ros2", "run", "arm_control", "arduino_sensors", 
                          "--ros-args", "-p", "autostart:=true",]
            sensor_name = "arduino_sensors"
        arm_procs = [
            ("arduino_sensors_proc", sensor_cmd, sensor_name),
            ("wall_parallel_proc",
             ["ros2", "run", "arm_control", "wall_parallel_controller", "--ros-args", "-p", "control_rate:=2.0"],
             "wall_parallel_controller"),
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
            ("arduino_sensors_proc", "distance sensors"),
            ("wall_parallel_proc",   "wall_parallel_controller"),
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
    # Force mode (real robot only): GPR press against the wall
    # ------------------------------------------------------------------
    def _build_force_mode_request(self):
        """Press the GPR (offset -0.08 m in tool0 X) against the wall with 5 N
        along the task-frame Z; only Z is compliant so the sensor alignment keeps
        full control of orientation."""
        req = SetForceMode.Request()
        req.task_frame.header.frame_id = "arm_tool0"
        req.task_frame.pose.position.x = -0.08
        req.task_frame.pose.orientation.w = 1.0
        req.selection_vector_z = True          # Z compliant; all others stiff
        req.wrench.force.z = 5.0
        req.type = 2                           # NO_TRANSFORM (task frame as given)
        req.speed_limits.linear.z = 0.1
        req.deviation_limits = [0.1, 0.1, 0.15, 0.1, 0.1, 0.1]
        req.damping_factor = 0.025
        req.gain_scaling = 0.5
        return req

    def _log_force_mode_result(self, node, future, label):
        try:
            res = future.result()
            node.get_logger().info(
                f"[{self.name}] force_mode {label}: success={getattr(res, 'success', True)}"
            )
        except Exception as e:
            node.get_logger().error(f"[{self.name}] force_mode {label} call failed: {e}")

    def _start_force_mode(self, ctx):
        """Real robot only. Start the GPR press for this line sweep."""
        if bool(ctx.get("sim", False)):
            return
        node = ctx["node"]
        if self.force_mode_start_client is None:
            self.force_mode_start_client = node.create_client(
                SetForceMode, "/force_mode_controller/start_force_mode")
        if not self.force_mode_start_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"[{self.name}] start_force_mode service unavailable.")
            self.fail(ctx, "force-mode controller service unavailable")
            return
        node.get_logger().info(f"[{self.name}] Starting force mode (GPR press 5N on Z).")
        fut = self.force_mode_start_client.call_async(self._build_force_mode_request())
        fut.add_done_callback(lambda f: self._log_force_mode_result(node, f, "start"))
        self.force_mode_active = True

    def _stop_force_mode(self, ctx):
        """Real robot only. Stop the GPR press (safe no-op if never started)."""
        if bool(ctx.get("sim", False)) or not self.force_mode_active:
            return
        node = ctx["node"]
        if self.force_mode_stop_client is None:
            self.force_mode_stop_client = node.create_client(
                Trigger, "/force_mode_controller/stop_force_mode")
        self.force_mode_active = False
        if not self.force_mode_stop_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"[{self.name}] stop_force_mode service unavailable.")
            return
        node.get_logger().info(f"[{self.name}] Stopping force mode.")
        fut = self.force_mode_stop_client.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self._log_force_mode_result(node, f, "stop"))

    # ------------------------------------------------------------------
    # Wall-contact detection from the TCP force/torque sensor
    # ------------------------------------------------------------------
    def _measured_press_force(self, ctx):
        """Latest wall-normal press force (N, signed) from the TCP FT sensor, or
        None if no reading has arrived yet. The press is commanded along task-frame
        Z (arm_tool0), which force_torque_sensor_broadcaster reports in the arm_tool0
        frame, so ``wrench.force.z`` is the wall-normal component. Pressing into the
        wall produces a negative Z reaction, so the value drops toward the commanded
        press (~-5 N) on contact."""
        wrench = ctx.get("ft_wrench")
        if wrench is None:
            return None
        return float(wrench.force.z)

    def _wall_contact_ready(self, ctx):
        """Return True once the GPR wheel is pressed against the wall, or
        immediately when there is nothing to wait for (sim has no FT sensor and
        force_mode is a no-op, or the press was never started). Returns False while
        still waiting so the caller can hand control back to the tick loop.

        Contact is detected from the TCP force/torque sensor
        (``self.ft_topic``): force_mode drives the plate along task-frame Z until
        the sensor's Z force reaches the commanded press, which is the moment the
        sensor plate meets the wall. Pressing into the wall reads *negative* on Z,
        so contact is Fz <= a negative threshold (default -5 N) — a signed check
        (not magnitude) so the noisy/oscillating idle band around 0 never trips it.
        A bounded timeout lets the sweep proceed anyway if contact is never
        reported, so a mis-reading FT sensor cannot deadlock the run. Tunable via
        the ``scan_wall_touch_force_n`` / ``scan_wall_touch_force_timeout_s`` ctx
        params.
        """
        node = ctx["node"]
        # Nothing to wait for in sim (no FT sensor) or if the press never started.
        if bool(ctx.get("sim", False)) or not self.force_mode_active:
            return True

        threshold = float(ctx.get("scan_wall_touch_force_n", -5.0))
        timeout_s = float(ctx.get("scan_wall_touch_force_timeout_s", 120.0))

        now = time.time()
        if self._press_settle_start is None:
            self._press_settle_start = now
            node.get_logger().info(
                f"[{self.name}] Waiting for the GPR wheel to touch the wall "
                f"(Fz <= {threshold:.1f} N on {self.ft_topic}, up to {timeout_s:.0f}s)..."
            )

        force_z = self._measured_press_force(ctx)
        if force_z is not None and force_z <= threshold:
            node.get_logger().info(
                f"[{self.name}] Wall contact detected (Fz={force_z:.2f} N <= "
                f"{threshold:.1f} N); starting GPR + base sweep."
            )
            return True

        if now - self._press_settle_start >= timeout_s:
            reading = "no FT reading" if force_z is None else f"Fz={force_z:.2f} N"
            node.get_logger().warn(
                f"[{self.name}] Wall contact not confirmed after {timeout_s:.0f}s "
                f"({reading} > {threshold:.1f} N); proceeding with the sweep anyway."
            )
            return True
        return False

    # ------------------------------------------------------------------
    # GPR (GP Proceq8800) HTTP API — real robot only
    # ------------------------------------------------------------------
    def _gpr_enabled(self, ctx):
        """GPR is DISABLED for now: the probe is not yet on the real robot's
        network topology, so scans run without it. Re-enable by setting the
        ``gpr_enabled`` ctx flag True (bench testing or once the probe is wired
        in). Previously defaulted to on for the real robot (``not sim``)."""
        return bool(ctx.get("gpr_enabled", False))

    def _gpr_request(self, ctx, method, path, json_body=None):
        """Issue one GPR HTTP request (blocking). Returns the response, or None on
        a connection/timeout error. Any 2xx is success (starts return 200, stops
        return 204); errors carry a ``{"error":{"code","message"}}`` body."""
        node = ctx["node"]
        base_url = ctx.get("gpr_base_url", self.gpr_base_url).rstrip("/")
        url = f"{base_url}{path}"
        try:
            resp = requests.request(
                method, url, json=json_body,
                timeout=ctx.get("gpr_timeout", self.gpr_timeout),
            )
        except requests.exceptions.RequestException as e:
            node.get_logger().error(f"[{self.name}] GPR {method} {path} failed: {e}")
            return None
        if resp.status_code >= 400:
            node.get_logger().error(
                f"[{self.name}] GPR {method} {path} -> HTTP {resp.status_code}: "
                f"{resp.text.strip()[:200]}"
            )
        else:
            node.get_logger().info(
                f"[{self.name}] GPR {method} {path} -> HTTP {resp.status_code}"
            )
        return resp

    def _gpr_connect(self, ctx):
        """Connect to the probe. 200 = connected, 406 = already connected (both OK).
        Measurement calls 403 unless connected first, so a failure here aborts the
        scan (a sweep with no GPR data is pointless).

        TODO (auto-connect): /probe/connect also accepts an optional ``ip`` field.
        Without it, the connection request must be **accepted manually on the GP
        App (iPad)** before this returns — so the scan is not fully autonomous yet.
        Once the probe is assigned a static IP, pass ``ip`` here (e.g. from
        ``ctx.get("gpr_ip")``) so the connection completes without operator input.
        """
        node = ctx["node"]
        serial = ctx.get("gpr_serial", self.gpr_serial)
        body = {"serialNumber": serial}
        # gpr_ip is intentionally unset for now (no static IP assigned to the probe).
        gpr_ip = ctx.get("gpr_ip")
        if gpr_ip:
            body["ip"] = gpr_ip
        node.get_logger().info(f"[{self.name}] GPR: connecting to probe {serial}.")
        self.set_activity(ctx, "Connecting to the GPR probe", publish=True)
        resp = self._gpr_request(ctx, "POST", "/probe/connect", body)
        if resp is not None and (resp.status_code < 400 or resp.status_code == 406):
            node.get_logger().info(f"[{self.name}] GPR probe connected.")
            return True
        node.get_logger().error(f"[{self.name}] GPR probe connection failed; aborting scan.")
        self.fail(ctx, "GPR probe connection failed")
        return False

    def _gpr_start_measurement_and_line(self, ctx):
        """Connect, create a LINE_SCAN measurement, then start the line. Called
        once per line right after the GPR wheel is pressed against the wall. Any
        start-path failure aborts the scan via ``error_triggered``."""
        if not self._gpr_enabled(ctx):
            return
        node = ctx["node"]
        if not self._gpr_connect(ctx):
            return
        line_no = ctx.get("current_line_idx", 0) + 1
        seg_no = self._seg_idx + 1
        body = {"type": "LINE_SCAN", "name": f"scan_wall line {line_no} seg {seg_no}"}
        self.set_activity(ctx, "Starting the GPR line-scan measurement", publish=True)
        resp = self._gpr_request(ctx, "POST", "/measurement/start", body)
        if resp is None or resp.status_code >= 400:
            node.get_logger().error(f"[{self.name}] GPR start measurement failed; aborting scan.")
            self.fail(ctx, "GPR failed to start the measurement")
            return
        self.gpr_measurement_active = True
        self.set_activity(ctx, "Starting the GPR scan line", publish=True)
        resp = self._gpr_request(ctx, "POST", "/measurement/line/start")
        if resp is None or resp.status_code >= 400:
            node.get_logger().error(f"[{self.name}] GPR start line failed; aborting scan.")
            self.fail(ctx, "GPR failed to start the scan line")
            return
        self.gpr_line_active = True
        node.get_logger().info(f"[{self.name}] GPR measurement + line started.")

    def _gpr_stop_line_and_measurement(self, ctx):
        """Stop the line then the measurement. Best-effort: logs failures but does
        not abort (the sweep is already done). Guarded by flags so it is a safe
        no-op if the line/measurement was never started."""
        if not self._gpr_enabled(ctx):
            return
        node = ctx["node"]
        if self.gpr_line_active or self.gpr_measurement_active:
            self.set_activity(ctx, "Stopping the GPR line and measurement", publish=True)
        if self.gpr_line_active:
            self._gpr_request(ctx, "POST", "/measurement/line/stop")
            self.gpr_line_active = False
        if self.gpr_measurement_active:
            self._gpr_request(ctx, "POST", "/measurement/stop")
            self.gpr_measurement_active = False
            node.get_logger().info(f"[{self.name}] GPR line + measurement stopped.")

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
                if tf_buffer.can_transform("map", frame, rclpy.time.Time(), Duration(seconds=1.0)):
                    tf = tf_buffer.lookup_transform("map", frame, rclpy.time.Time(), Duration(seconds=1.0))
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
    # Base parking (Phase 0): align the chassis with the wall-facing turret
    # ------------------------------------------------------------------
    def _send_park_enabled(self, ctx, value):
        """Flip the controller's enable_park_service parameter via /set_parameters.

        park_now is live-gated by that parameter (always created but only acts while
        it is true), so ScanWall sets it true just for the maneuver and false again
        afterwards. Returns the call future, or None if the parameter service is
        unavailable.
        """
        node = ctx["node"]
        set_param_srv = ctx.get("park_set_param_service", "/sim_controller/set_parameters")
        if self.park_enable_client is None:
            self.park_enable_client = node.create_client(SetParameters, set_param_srv)
        if not self.park_enable_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().warn(
                f"[{self.name}] Parameter service '{set_param_srv}' unavailable; "
                f"cannot toggle enable_park_service."
            )
            return None
        req = SetParameters.Request()
        pmsg = ParameterMsg()
        pmsg.name = "enable_park_service"
        pmsg.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
        req.parameters = [pmsg]
        return self.park_enable_client.call_async(req)

    def _param_set_ok(self, ctx, future, label):
        """True if a /set_parameters future succeeded; logs and returns False on an
        exception or a rejected result."""
        node = ctx["node"]
        try:
            results = future.result().results
        except Exception as e:
            node.get_logger().warn(
                f"[{self.name}] set enable_park_service ({label}) call failed: {e}"
            )
            return False
        ok = bool(results) and all(r.successful for r in results)
        if not ok:
            reason = results[0].reason if results else "no result"
            node.get_logger().warn(
                f"[{self.name}] set enable_park_service ({label}) rejected: {reason}"
            )
        return ok

    def _reset_park_state(self):
        """Reset the per-maneuver parking sub-state so _run_parking starts a fresh
        enable->request->settle->disable cycle for the next segment."""
        self.park_done = False
        self._park_phase = "enable"
        self.park_future = None
        self.park_param_future = None
        self._park_saw_active = False
        self._park_wait_start = None

    def _run_parking(self, ctx):
        """Align the diff-drive chassis with the turret before the sweep.

        After NavigateToTarget the Nav2-driven turret faces the wall, but the
        chassis wheels sit at an arbitrary heading (and each sweep strafes by
        rotating the chassis further). sim_controller's /sim_controller/park_now
        service rotates the chassis to the turret's current world heading while the
        turret joint compensates to hold the turret (and the mounted arm)
        world-stationary, so the whole robot ends up squared to the wall before the
        arm extends.

        park_now is live-gated by the controller's enable_park_service parameter
        (kept false by default), so the maneuver runs as a small sequence:
        ``enable`` (set the parameter true) -> ``request`` (call park_now) ->
        ``settle`` (wait on the latched /sim_controller/parking_active flag:
        active->inactive = aligned, or a short grace if it never goes active) ->
        ``disable`` (set the parameter false again). Best-effort at every step: a
        missing service or a failed/rejected call skips gracefully so the scan still
        runs. Sets self.park_done when finished or skipped.
        """
        node = ctx["node"]

        # Opt-out hook (e.g. benches without the base controller).
        if not bool(ctx.get("scan_wall_park_base", True)):
            self.park_done = True
            return

        # --- Phase 0a: enable the live-gated park service for this maneuver. ---
        if self._park_phase == "enable":
            self.park_param_future = self._send_park_enabled(ctx, True)
            if self.park_param_future is None:
                self.park_done = True  # no param service -> skip (parameter stays false)
                return
            node.get_logger().info(
                f"[{self.name}] Parking: enabling park service before aligning the base."
            )
            self._park_phase = "enable_wait"
            return

        if self._park_phase == "enable_wait":
            if not self.park_param_future.done():
                return
            if not self._param_set_ok(ctx, self.park_param_future, "enable"):
                # Couldn't enable -> park_now would refuse; skip. The parameter is
                # unchanged (still false), so no revert is needed.
                self.park_done = True
                self.park_param_future = None
                return
            self.park_param_future = None
            self._park_phase = "request"
            return

        # --- Phase 0b: request the maneuver. ---
        if self._park_phase == "request":
            park_service = ctx.get("park_service", "/sim_controller/park_now")
            if self.park_client is None:
                self.park_client = node.create_client(Trigger, park_service)
            if not self.park_client.wait_for_service(timeout_sec=2.0):
                node.get_logger().warn(
                    f"[{self.name}] Park service '{park_service}' unavailable; skipping "
                    f"base alignment (scan proceeds)."
                )
                self._park_phase = "disable"  # revert the parameter we just enabled
                return
            node.get_logger().info(
                f"[{self.name}] Parking: aligning the chassis to the turret (wall) heading."
            )
            self.park_future = self.park_client.call_async(Trigger.Request())
            self._park_saw_active = False
            self._park_wait_start = time.time()
            self._park_phase = "accept_wait"
            return

        if self._park_phase == "accept_wait":
            if not self.park_future.done():
                return
            try:
                resp = self.park_future.result()
                if not resp.success:
                    node.get_logger().warn(
                        f"[{self.name}] Park request not accepted ({resp.message}); "
                        f"proceeding without base alignment."
                    )
                    self._park_phase = "disable"
                    self.park_future = None
                    return
                node.get_logger().info(f"[{self.name}] Park started: {resp.message}")
            except Exception as e:
                node.get_logger().warn(
                    f"[{self.name}] Park service call failed ({e}); proceeding without "
                    f"base alignment."
                )
                self._park_phase = "disable"
                self.park_future = None
                return
            self.park_future = None
            self._park_phase = "settle"
            return

        # --- Phase 0c: wait for the maneuver to finish (latched parking_active). ---
        if self._park_phase == "settle":
            grace_s = float(ctx.get("scan_wall_park_grace_s", 5.0))
            timeout_s = float(ctx.get("scan_wall_park_timeout_s", 120.0))
            active = ctx.get("parking_active")
            elapsed = time.time() - self._park_wait_start

            if elapsed > timeout_s:
                node.get_logger().warn(
                    f"[{self.name}] Parking did not confirm after {timeout_s:.0f}s; "
                    f"proceeding with the sweep anyway."
                )
                self._park_phase = "disable"
                return
            if active:
                self._park_saw_active = True
                return
            # active is False or None here.
            if self._park_saw_active:
                node.get_logger().info(f"[{self.name}] Chassis aligned to the wall.")
                self._park_phase = "disable"
                return
            if elapsed >= grace_s:
                node.get_logger().info(
                    f"[{self.name}] Chassis already aligned (parking never went active)."
                )
                self._park_phase = "disable"
            return

        # --- Phase 0d: disable the park service again, then finish. ---
        if self._park_phase == "disable":
            self.park_param_future = self._send_park_enabled(ctx, False)
            if self.park_param_future is None:
                node.get_logger().warn(
                    f"[{self.name}] Could not disable park service (param service gone); "
                    f"leaving enable_park_service as-is."
                )
                self.park_done = True
                return
            self._park_phase = "disable_wait"
            return

        if self._park_phase == "disable_wait":
            if not self.park_param_future.done():
                return
            self._param_set_ok(ctx, self.park_param_future, "disable")  # best-effort log
            self.park_param_future = None
            node.get_logger().info(f"[{self.name}] Parking done; park service disabled.")
            self.park_done = True
            return

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
                        self.fail(ctx, f"arm rejected the scanning pose ({response.message})")
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
                self.fail(ctx, "arm planner failed to reach the scanning pose")
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
                self.fail(ctx, "column did not reach the target height in time")
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
        # NOTE: chassis parking is NOT done here. It runs per segment, right after the
        # base reaches the segment start and just before the sweep (the "park" seg-phase
        # in _run_scan), so no base repositioning follows the alignment and undoes it.
        if not self.preapproach_done:
            self.set_activity(ctx, "Moving arm and column to the wall-scanning pose")
            self._run_pre_approach(ctx)
            return

        # Phase B: active scan (sensors on, base sweeps the line).
        if not self.scan_swept:
            self._run_scan(ctx)
            return

        # Phase C: post-scan retraction (arm pulls back, column retracts on last line).
        if not self.postscan_done:
            self.set_activity(ctx, "Retracting the arm after the wall sweep")
            self._run_post_scan(ctx)
            return

    def _run_scan(self, ctx):
        node = ctx["node"]

        # --- One-time per-line setup: sweep direction + fixed heading. ---
        if not self.started:
            wall_data = ctx.get("target_scan_wall", None)
            prev_target_point = ctx.get("target_scan_point", None)

            if not wall_data or not prev_target_point:
                node.get_logger().error(f"[{self.name}] Missing wall data or target point.")
                self.fail(ctx, "missing wall data or target point")
                return

            # Serpentine: sweep from the wall end the robot is at toward the
            # other end. Nearest-endpoint match — the current point may be a
            # clamped reachable-segment endpoint, not the raw wall end.
            d0 = math.hypot(prev_target_point[0] - wall_data[0][0],
                            prev_target_point[1] - wall_data[0][1])
            d1 = math.hypot(prev_target_point[0] - wall_data[1][0],
                            prev_target_point[1] - wall_data[1][1])
            near_end, far_end = (
                (wall_data[0], wall_data[1]) if d0 <= d1 else (wall_data[1], wall_data[0])
            )
            self._sweep_from, self._sweep_to = near_end, far_end

            # Keep a FIXED base heading for the whole wall (computed on the first
            # line). The omnidirectional base then strafes back and forth without
            # turning, so the arm keeps facing the wall on every line. Recomputed
            # at line 0 so a new wall picks up its own heading.
            heading = ctx.get("scan_heading_yaw")
            if heading is None or ctx.get("current_line_idx", 0) == 0:
                heading = atan2(far_end[1] - near_end[1], far_end[0] - near_end[0])
                ctx["scan_heading_yaw"] = heading
            self._sweep_qz = sin(heading / 2.0)
            self._sweep_qw = cos(heading / 2.0)

            node.get_logger().info(
                f"[{self.name}] Initiating wall scan maneuver "
                f"({near_end[0]:.2f}, {near_end[1]:.2f}) -> ({far_end[0]:.2f}, {far_end[1]:.2f})."
            )
            # Hand back to the tick loop so the costmap subscription can populate
            # before segmentation (the executor is single-threaded).
            self._costmap_wait_start = None
            self.started = True
            return

        # TODO(wall-surface obstacles): the sweep assumes a flat wall — the moving
        # sensor plate can hit surface features the 2D costmap never sees: small
        # bumps, wall-mounted fixtures, or columns/pilasters protruding near the
        # wall at plate height. Consider (a) checking the 3D map/octomap along the
        # scan line at line_z and splitting segments around protrusions (like
        # reachable_wall_segments does for base obstacles), and (b) reacting
        # online to sudden distance-sensor jumps during the sweep (plate
        # approaching a protrusion) by pausing/retracting instead of pressing on.

        # --- Split the line into reachable segments; gaps get skipped. ---
        if self._segments is None:
            segments = reachable_wall_segments(ctx, self._sweep_from, self._sweep_to)
            if segments is None:  # costmap not received yet — bounded poll
                now = time.time()
                if self._costmap_wait_start is None:
                    node.get_logger().info(
                        f"[{self.name}] Waiting for the global costmap to segment the "
                        f"scan line (up to {COSTMAP_WAIT_TIMEOUT_S:.0f}s)..."
                    )
                    self._costmap_wait_start = now
                    return
                if now - self._costmap_wait_start < COSTMAP_WAIT_TIMEOUT_S:
                    return
                node.get_logger().warn(
                    f"[{self.name}] No costmap after {COSTMAP_WAIT_TIMEOUT_S:.0f}s; "
                    f"sweeping the whole line unsegmented."
                )
                segments = [(tuple(self._sweep_from), tuple(self._sweep_to))]
            if not segments:
                node.get_logger().error(
                    f"[{self.name}] No reachable portion of this scan line (no base cell "
                    f"within arm reach anywhere along it); cannot scan this wall."
                )
                self.fail(ctx, "no reachable portion of the scan line within arm reach")
                return
            self._segments = segments
            self._seg_idx = 0
            self._seg_phase = "transit"
            self._segments_ok = 0
            publish_wall_segment_markers(ctx, segments)
            node.get_logger().info(
                f"[{self.name}] Scan line split into {len(segments)} reachable segment(s): "
                + "; ".join(
                    f"({s[0]:.2f}, {s[1]:.2f})->({e[0]:.2f}, {e[1]:.2f})" for s, e in segments
                )
            )
            return

        # --- All segments processed: finalize the line. ---
        if self._seg_idx >= len(self._segments):
            self._finalize_line(ctx)
            return

        seg_start, seg_end = self._segments[self._seg_idx]
        seg_no, seg_total = self._seg_idx + 1, len(self._segments)

        if self._seg_phase == "transit":
            self.set_activity(
                ctx,
                f"Repositioning base to wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # Reposition (sensors off, no press) to the segment start. Skipped
            # when the base is already there — e.g. the first segment right
            # after NavigateToTarget, or contiguous serpentine turnarounds.
            pos = ctx.get("base_position")
            skip_tol = float(ctx.get("segment_transit_skip_tol", 0.4))
            goal_xy = base_standoff_goal(ctx, self.name, seg_start)
            if pos is not None and math.hypot(pos.x - goal_xy[0], pos.y - goal_xy[1]) <= skip_tol:
                node.get_logger().info(
                    f"[{self.name}] Base already at segment {self._seg_idx + 1} start; "
                    f"skipping transit."
                )
                self._reset_park_state()
                self._seg_phase = "park"
                return
            node.get_logger().info(
                f"[{self.name}] Transit to segment {self._seg_idx + 1}/{len(self._segments)} "
                f"start ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) (sensors off)."
            )
            if not self._send_base_goal(ctx, goal_xy):
                return
            self._seg_phase = "transit_wait"
            return

        if self._seg_phase == "transit_wait":
            if self._nav_status is None:
                return
            if self._nav_status == GoalStatus.STATUS_SUCCEEDED:
                self._reset_park_state()
                self._seg_phase = "park"
            else:
                node.get_logger().warn(
                    f"[{self.name}] Transit to segment {self._seg_idx + 1} failed "
                    f"(status={self._nav_status}); skipping this segment."
                )
                self._seg_idx += 1
                self._seg_phase = "transit"
            return

        if self._seg_phase == "park":
            # Square the diff-drive chassis to the wall now that the base has reached the
            # segment start (transit done/skipped). Parking is the LAST base motion before
            # the sweep — nothing navigates the base back to the segment start afterwards —
            # so the alignment is preserved. The arm is already in the unfolded_fsm pose;
            # the turret joint compensates to hold it world-stationary while the chassis
            # rotates. Driven by _run_parking's enable->request->settle->disable sequence.
            self.set_activity(
                ctx,
                f"Parking chassis to square it against wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            self._run_parking(ctx)
            if self.park_done:
                self._seg_phase = "sweep_setup"
            return

        if self._seg_phase == "sweep_setup":
            # Force mode is a real-robot-only press (no controller exists in sim),
            # so only mention it when it will actually run.
            self.set_activity(
                ctx,
                "Enabling wall-parallel controller"
                if bool(ctx.get("sim", False))
                else "Enabling wall-parallel controller and starting force-mode press",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            node.get_logger().info(
                f"[{self.name}] Segment {self._seg_idx + 1}/{len(self._segments)}: "
                f"activating alignment + press for the sweep."
            )
            # Activate sensor alignment now that the base is at the segment start.
            if not self._start_arm_processes(ctx):
                return
            node.get_logger().info(f"[{self.name}] Waiting 10s for arm processes to stabilise...")
            time.sleep(10.0)

            # Real robot: press the GPR against the wall for this segment sweep
            # (sim has no force_mode controller -> no-op).
            self._start_force_mode(ctx)
            if ctx.get("error_triggered"):
                return

            # Do NOT start the GPR or move the base yet: force_mode is still
            # driving the plate toward the wall. Wait in press_settle until the TCP
            # FT sensor reports contact, so the measurement + sweep begin only once
            # the sensor plate is actually pressed against the wall.
            self._press_settle_start = None
            self._seg_phase = "press_settle"
            return

        if self._seg_phase == "press_settle":
            # In sim there is no force-mode press or FT wall-contact wait; this
            # phase just falls through, so describe the plate alignment instead.
            self.set_activity(
                ctx,
                "Aligning the sensor plate to the wall"
                if bool(ctx.get("sim", False))
                else "Stretching the arm against the wall with force-mode control",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            if not self._wall_contact_ready(ctx):
                return

            # Wheel is pressed against the wall. GPR: connect, create the LINE_SCAN
            # measurement and start the line now (real robot only).
            self._gpr_start_measurement_and_line(ctx)
            if ctx.get("error_triggered"):
                return

            goal_xy = base_standoff_goal(ctx, self.name, seg_end)
            node.get_logger().info(
                f"[{self.name}] Sweeping segment {self._seg_idx + 1} to "
                f"({goal_xy[0]:.2f}, {goal_xy[1]:.2f})."
            )
            self.set_activity(
                ctx,
                f"Sweeping wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            if not self._send_base_goal(ctx, goal_xy):
                return
            self._seg_phase = "sweep_wait"
            return

        if self._seg_phase == "sweep_wait":
            self.set_activity(
                ctx,
                f"Sweeping wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            if self._nav_status is None:
                return
            status = self._nav_status
            # Release hardware/process state first (safety), regardless of outcome.
            node.get_logger().info(
                f"[{self.name}] Segment sweep finished (status={status}). Stopping GPR, "
                f"force mode + arm processes..."
            )
            self._gpr_stop_line_and_measurement(ctx)   # stop line + measurement before releasing the press
            self._stop_force_mode(ctx)      # release the press before the arm retracts
            self._stop_arm_processes(ctx)
            time.sleep(2)   # delay to avoid errors in the arm goals

            if status == GoalStatus.STATUS_SUCCEEDED:
                self._segments_ok += 1
                self._last_swept_point = seg_end
            else:
                node.get_logger().warn(
                    f"[{self.name}] Segment {self._seg_idx + 1} sweep did not succeed "
                    f"(status={status}); skipping to the next segment."
                )
            self._seg_idx += 1
            self._seg_phase = "transit"
            return

    def _finalize_line(self, ctx):
        """All segments of the current line processed. The line counts as scanned
        when at least one segment swept; otherwise flag an error (completing
        would silently drop the wall)."""
        node = ctx["node"]
        if self._segments_ok == 0:
            node.get_logger().error(
                f"[{self.name}] No segment of this line could be swept; the wall was "
                f"not scanned. Flagging error instead of completing."
            )
            ctx["error_triggered"] = True
            return

        # The robot is at the last swept segment end; record it so the next line
        # sweeps back toward the opposite wall end (serpentine).
        if self._last_swept_point is not None:
            ctx["target_scan_point"] = tuple(self._last_swept_point)

        # Advance to the next horizontal line on this wall. more_lines drives
        # both the self-loop and whether this was the last line of the wall.
        ctx["current_line_idx"] = ctx.get("current_line_idx", 0) + 1
        lines = ctx.get("current_wall_scan_lines", [])
        self.more_lines = ctx["current_line_idx"] < len(lines)
        node.get_logger().info(
            f"[{self.name}] Line done ({self._segments_ok}/{len(self._segments)} segment(s) "
            f"swept)."
            + (
                f" {len(lines) - ctx['current_line_idx']} more line(s) on this wall; "
                f"will retract arm then re-enter for next height."
                if self.more_lines else " Last line of this wall."
            )
        )
        # The base sweep is done; hand control to the post-scan retraction phase.
        # walls_left is only decremented once that phase completes (see
        # _run_post_scan), so the column retract on the last line is not skipped.
        self.scan_swept = True

    # ------------------------------------------------------------------
    # Base goal plumbing (segment transit + sweep goals)
    # ------------------------------------------------------------------
    def _send_base_goal(self, ctx, goal_xy):
        """Send a NavigateToPose goal at the fixed sweep heading. The result
        lands in ``self._nav_status`` (a GoalStatus value, or -1 on
        rejection/exception); ``None`` while in flight."""
        node = ctx["node"]
        nav_client = ctx.get("nav_client", None)
        if nav_client is None:
            node.get_logger().error(f"[{self.name}] Navigation client not found.")
            ctx["error_triggered"] = True
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_xy[0])
        goal_msg.pose.pose.position.y = float(goal_xy[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = self._sweep_qz
        goal_msg.pose.pose.orientation.w = self._sweep_qw
        # No base-reversing recoveries (BackOutFromObstacle/BackUp): backing the
        # base out mid-sweep ruins the scan geometry — fail fast to the FSM.
        goal_msg.behavior_tree = nav_bt_xml(ctx, self.name)
        self._nav_status = None
        self._send_goal_future = nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception:
            self._nav_status = -1
            return
        if not handle.accepted:
            self._nav_status = -1
            return
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        try:
            self._nav_status = future.result().status
        except Exception:
            self._nav_status = -1

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
        ## Safety net: stop any running GPR measurement, release force mode (real)
        ## and stop sensor + alignment nodes.
        self._gpr_stop_line_and_measurement(ctx)
        self._stop_force_mode(ctx)
        self._stop_arm_processes(ctx)

    def check_transition(self, ctx):
        if self.finished and self.more_lines:
            return "ScanWall"        # same wall, next height (base stays put)
        if self.finished:
            return "SensorDataProcessing"
        if ctx.get("error_triggered"):
            return "Error"
        return None
