from ..state import State
from ..utils.column_control import ColumnController
from ..utils.costmap_utils import (
    COSTMAP_WAIT_TIMEOUT_S,
    base_standoff_goal,
    nav_bt_xml,
    plan_wall_partitions,
    reachable_wall_segments,
    publish_wall_segment_markers,
    wall_parallel_goal,
)
from ..utils.wall_partitioning import next_backoff_length, sweep_line_order
from ..utils.wall_approach import unfolded_pose_name
from arm_control.action import SweepLine
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.action import GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from rclpy.duration import Duration
import rclpy.time
from arm_control.srv import SendPosition
from ur_msgs.srv import SetForceMode
from std_srvs.srv import Trigger
from std_msgs.msg import UInt32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
import subprocess, os, signal
import math
from math import atan2, sin, cos
import time
import requests

# Validity window (m) for each of the six plate ranges, in the publish order of
# arm_control's arduino_sensors(_sim): [C/U1, A/U2, B/U3] ultrasonic then
# [S1, S2, S3] ToF. Copied from wall_parallel_controller's valid_lo/valid_hi so
# both nodes agree on which readings mean anything — keep them in sync. Mind the
# ToF ceiling: at the standoff the base parks at, the plate is normally further
# out than 0.258 m, so the reading that sizes the arm's approach is ultrasonic-only.
PLATE_SENSOR_VALID_LO = (0.02, 0.02, 0.02, 0.011, 0.011, 0.011)
PLATE_SENSOR_VALID_HI = (3.90, 3.90, 3.90, 0.258, 0.258, 0.258)

# Oldest /distance_sensors frame (s) still trusted as a live measurement. The
# reader publishes at ~5 Hz, so anything older means it died or was stopped.
PLATE_DISTANCE_MAX_AGE_S = 3.0


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
        #
        # Base and arm own different axes. The base only ever slides PARALLEL to
        # the wall, at whatever standoff it arrived with (wall_parallel_goal strips
        # the wall-ward component off every transit goal); the distance to the wall
        # belongs to the arm, which measures it with the plate sensors and travels
        # along its own Z axis. Hence the phase order per segment:
        #   transit_clear(_wait)  retract the plate to a safe gap before the base moves
        #   transit(_wait)   slide along the wall to the segment start
        #   park             square the chassis against the wall
        #   sweep_setup      cap the base speed for the sweep
        #   arm_approach(_wait)  extend the arm to the approach standoff
        #   lead_in(_wait)   arm crosses to the partition START, still off the wall
        #   press_prepare    alignment controller + force-mode press
        #   press_settle     wait for the FT sensor to confirm wall contact
        #   sweep_wait       the actual scan (GPR triggers fire every X cm here)
        # The lead-in comes BEFORE the press on purpose: the base parks at the
        # partition centre, so pressing first would drag the GPR wheel half a
        # partition sideways across the wall under load. It is also why the
        # triggers are armed off the executor's "sweep" feedback and not here:
        # the lead-in slides the plate along the wall without scanning it.
        self._segments = None        # [((sx,sy,z),(ex,ey,z)), ...] robot-end first
        # Arm-sweep mode only: one (x, y, yaw) Nav2 scan pose per entry of
        # self._segments (which then holds partitions, not raw segments).
        # Resolved up front so an unplaceable partition is visible in RViz before
        # the robot moves. None in the legacy base-driven path.
        self._scan_poses = None
        # Per reachable segment, the max partition length currently in force. Only
        # differs from partition_max_length_m after a backoff retry (§7.B).
        self._segment_max_len = {}
        # Which scan-line height is being swept at the CURRENT partition. Always 0
        # unless nest_lines_in_partition is on (§8).
        self._line_idx = 0
        # Base goal held between deciding to transit and actually sending it, so
        # the column can be retracted in between.
        self._pending_transit = None
        self._seg_idx = 0
        self._seg_phase = "transit_clear"
        # Arm-driven sweep: the SweepLine action client and the latest result,
        # (succeeded, reason, detail), or None while a sweep is in flight.
        self._sweep_client = None
        self._sweep_result = None
        self._sweep_goal_handle = None
        # Set from the executor's feedback once the plate reaches the wall. A flag
        # rather than the GPR call itself: that call blocks on HTTP round trips to
        # the Proceq and must not run inside an executor callback.
        self._sweep_scanning = False
        self._arm_goal_pub = None        # /arm/goal_pose publisher (created once)
        self._arm_goal_start = None      # planner wait deadline for the Z move
        self._recenter_future = None     # /send_position call for the pre-transit re-pose
        self._plate_wait_start = None    # bounded wait for the first plate reading
        self._segments_ok = 0        # segments actually swept on this line
        self._last_swept_point = None
        self._nav_status = None      # set by _on_nav_result / rejection / crawl tick
        self._sweep_from = None      # wall end the sweep starts from (robot's end)
        self._sweep_to = None        # wall end the sweep heads toward

        # Sweep crawl: the active scan drags the base at a slow, constant velocity
        # over /cmd_vel instead of a Nav2 goal, so the arm's contact/alignment
        # controllers can keep the plate on the wall without being "left behind".
        # (Nav2's DWB sweep can't hold a stable ~0.05 m/s: it trips
        # SimpleProgressChecker (needs 0.1 m/s) and sits below trans_stopped_velocity.)
        # Transit between segments still uses the Nav2 goal at normal speed.
        self._sweep_crawl_target = None    # (x, y) base-standoff goal in map frame
        self._crawl_yaw = None             # heading held during a crawl (None = sweep heading)
        self._crawl_speed = None
        self._crawl_tol = None             # arrival tolerance (None = sweep_arrive_tol)
        self._crawl_started = 0.0
        self._crawl_distance = 0.0
        self._sweep_crawl_deadline = None
        self._sweep_crawl_timer = None

        # Nav2 slow-sweep route (default): cap DWB to a crawl speed via a Nav2
        # SpeedLimit and relax the progress checker so the sub-0.1 m/s sweep is
        # not aborted as "no progress". Both are scoped to the sweep and restored
        # afterwards. Set ctx["sweep_use_crawl"]=True to use the /cmd_vel crawl
        # instead (exact speed, but no DWB collision checking during the drag).
        self._speed_limit_pub = None
        self._controller_param_client = None
        self._sweep_speed_applied = False

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

        # GPR encoder trigger: the probe normally clocks its traces off an encoder
        # wheel that has to roll on the wall, which is hard to keep in contact even
        # when the plate is at the right standoff. A fake encoder replaces it, and
        # the computer decides when to pulse it: one trigger per
        # ``gpr_trigger_distance_m`` of SENSOR-PLATE travel (measured in the map
        # frame, so base sweep + arm motion both count). Sampled by its own timer
        # during the sweep, because the FSM itself only ticks at 1 Hz — far too
        # coarse for a 5 mm spacing at the sweep speed. See _gpr_trigger_tick.
        self._gpr_trigger_pub = None
        self._gpr_trigger_timer = None
        self._gpr_trigger_last_xyz = None   # previous plate sample (map frame)
        self._gpr_trigger_axis = None       # unit sweep direction (map frame, XY)
        self._gpr_trigger_residual = 0.0    # travel not yet converted to a trigger
        self._gpr_trigger_travel = 0.0      # total plate travel this segment
        self._gpr_trigger_count = 0         # triggers fired this segment
        self._gpr_trigger_tf_warned = False
        self._ee_frame = None               # cached map->EE frame name

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
        self._scan_poses = None
        # Per reachable segment, the max partition length currently in force. Only
        # differs from partition_max_length_m after a backoff retry (§7.B).
        self._segment_max_len = {}
        # Which scan-line height is being swept at the CURRENT partition. Always 0
        # unless nest_lines_in_partition is on (§8).
        self._line_idx = 0
        # Base goal held between deciding to transit and actually sending it, so
        # the column can be retracted in between.
        self._pending_transit = None
        self._seg_idx = 0
        self._seg_phase = "transit_clear"
        self._arm_goal_start = None
        self._recenter_future = None
        self._plate_wait_start = None
        self._segments_ok = 0
        self._last_swept_point = None
        self._nav_status = None
        self._sweep_result = None
        self._sweep_goal_handle = None
        self._sweep_scanning = False
        self._sweep_from = None
        self._sweep_to = None
        self._stop_sweep_crawl(ctx)   # clear any stale timer from a re-entry
        self._sweep_crawl_target = None
        self._sweep_crawl_deadline = None
        self._crawl_yaw = None       # heading held during a crawl (None = sweep heading)
        self._crawl_speed = None
        self._restore_sweep_speed(ctx)   # clear any stale slow-sweep limit from a re-entry
        self.force_mode_active = False
        self._press_settle_start = None
        self.gpr_measurement_active = False
        self.gpr_line_active = False
        self._stop_gpr_triggers(ctx, log_summary=False)   # clear any stale timer
        self._ee_frame = None
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

        # NOTE: nothing that touches the arm is started here. The distance-sensor
        # reader comes up at the end of the pre-approach, once the arm is unfolded at
        # the line height, because every base move from then on is sized by a
        # measured plate clearance (_start_distance_sensors). The
        # wall_parallel_controller and, on the real robot, force_mode start later
        # still — only after the arm's own Z approach has finished, since that node
        # streams IK setpoints and would fight the approach goal
        # (_start_wall_alignment / _start_force_mode, from the press_prepare phase).

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
    def _start_distance_sensors(self, ctx):
        """Activate the distance-sensor reader alone. Sim uses the simulated
        sensors; real uses the Arduino reader.

        Split out of the alignment controller and started as soon as the arm is
        unfolded, because its /distance_sensors frames are what size every arm Z
        move — including the retraction that clears the plate BEFORE the base
        slides to the next segment, which happens long before the controller runs.
        """
        use_sim = bool(ctx.get("sim", False))
        if use_sim:
            sensor_cmd = ["ros2", "run", "arm_control", "arduino_sensors_sim", "--ros-args",
                          "-p", "autostart:=true", "-p", "publish_rate:=5.0", "-p", "batch_size:=2"]
            sensor_name = "arduino_sensors_sim"
        else:
            sensor_cmd = ["ros2", "run", "arm_control", "arduino_sensors",
                          "--ros-args", "-p", "autostart:=true",]
            sensor_name = "arduino_sensors"
        return self._start_arm_procs(
            ctx, [("arduino_sensors_proc", sensor_cmd, sensor_name)]
        )

    def _start_wall_alignment(self, ctx):
        """Activate the wall_parallel_controller for the sweep.

        Deliberately NOT started with the sensors: the controller streams IK
        setpoints continuously, so while it runs nothing else may command the arm.
        It comes up only after the arm's Z approach has finished (press_prepare),
        never during the approach or a transit.
        """
        return self._start_arm_procs(ctx, [
            ("wall_parallel_proc",
             ["ros2", "run", "arm_control", "wall_parallel_controller", "--ros-args",
              "-p", "control_rate:=2.0"],
             "wall_parallel_controller"),
        ])

    def _start_sweep_executor(self, ctx):
        """Bring up the arm-sweep executor for the whole line.

        Unlike wall_parallel_controller, this node does not stream anything: it
        sits idle until a SweepLine goal arrives. So it is safe to leave running
        across transits and arm approaches, and starting it once per line rather
        than once per partition keeps its TF/joint-state warm-up off the critical
        path of all 19 of them.

        The controller it drives differs by platform: Gazebo runs
        joint_trajectory_controller, the real robot passthrough_trajectory_controller
        (the only one the UR driver will run alongside force mode, ARM_SWEEP_PLAN
        §3.1). Goal tolerance follows the same split — meaningful in sim, empty on
        the robot, where the endpoint legitimately differs along the compliant Z
        axis (§3.4).
        """
        use_sim = bool(ctx.get("sim", False))
        controller = str(ctx.get(
            "sweep_controller_name",
            "joint_trajectory_controller" if use_sim else "passthrough_trajectory_controller",
        ))
        goal_tolerance = float(ctx.get("sweep_goal_tolerance_rad", 0.05 if use_sim else 0.0))
        cmd = [
            "ros2", "run", "arm_control", "wall_sweep_executor", "--ros-args",
            # The trajectory controller advances trajectories on the sim clock, and
            # the executor's watchdog compares against a trajectory's planned
            # duration -- so the two must share a clock, or the watchdog fires
            # early on motion that is running fine.
            "-p", f"use_sim_time:={'true' if use_sim else 'false'}",
            "-p", f"controller_name:={controller}",
            "-p", f"erase_path_tolerance:={'true' if use_sim else 'false'}",
            "-p", f"goal_tolerance_rad:={goal_tolerance}",
            "-p", f"sweep_speed_mps:={float(ctx.get('sweep_speed_mps', 0.05))}",
            "-p", f"waypoint_spacing_m:={float(ctx.get('waypoint_spacing_m', 0.03))}",
            "-p", f"max_joint_step_rad:={float(ctx.get('max_joint_step_rad', 0.35))}",
            "-p", f"sweep_contact_loss_timeout_s:="
                  f"{float(ctx.get('sweep_contact_loss_timeout_s', 0.20))}",
            # How far off the wall the plate sweeps when NOTHING is pressing it
            # (Gazebo, and the no-press first run on hardware per §11.2).
            # Deliberately its own knob rather than scan_wall_plate_offset: that
            # one sizes the gap force mode is expected to press through, whereas
            # this one is the gap that has to survive the whole sweep untouched.
            # Under Force Mode the executor ignores it entirely and sweeps in the
            # plane the press established.
            "-p", f"scan_standoff_m:={float(ctx.get('sweep_scan_standoff_m', 0.20))}",
            "-p", f"approach_retract_m:={float(ctx.get('sweep_approach_retract_m', 0.0))}",
            "-p", f"max_traverse_m:={float(ctx.get('sweep_max_traverse_m', 1.10))}",
            "-p", f"trajectory_timeout_factor:="
                  f"{float(ctx.get('sweep_timeout_factor', 4.0))}",
            "-p", f"trajectory_timeout_pad_s:="
                  f"{float(ctx.get('sweep_timeout_pad_s', 30.0))}",
            # Bench diagnostics (ARM_SWEEP_PLAN §7.B). The topic is always on.
            "-p", f"diagnostics_rate_hz:={float(ctx.get('sweep_diagnostics_rate_hz', 20.0))}",
        ]
        # The per-sweep CSV only when a directory is configured. Appended
        # conditionally because `-p name:=` with an EMPTY value is not parseable:
        # rcl rejects the whole argument list and the node dies before it can
        # advertise its action, which the FSM then sees only as
        # "'sweep_line' action never came up".
        csv_dir = str(ctx.get("sweep_diagnostics_csv_dir", "")).strip()
        if csv_dir:
            cmd += ["-p", f"diagnostics_csv_dir:={csv_dir}"]
        return self._start_arm_procs(
            ctx, [("sweep_executor_proc", cmd, "wall_sweep_executor")]
        )

    def _start_arm_procs(self, ctx, arm_procs):
        node = ctx["node"]
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

    def _stop_arm_processes(self, ctx, keep_sensors=False):
        """Stop the alignment controller and, unless ``keep_sensors``, the reader.

        Between segments the reader stays up: the plate distance is still needed to
        retract the arm before the base slides along the wall.

        The sweep executor is tied to the reader rather than to the alignment
        controller. It is stopped mid-line only by the calls that also take the
        sensors down (line/wall teardown) — between partitions it must survive,
        because that is the whole reason it is started once per line.
        """
        node = ctx["node"]
        procs = [("wall_parallel_proc", "wall_parallel_controller")]
        if not keep_sensors:
            procs.append(("arduino_sensors_proc", "distance sensors"))
            procs.append(("sweep_executor_proc", "wall_sweep_executor"))
        for proc_key, proc_name in procs:
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
    # Arm Z moves: the ARM, not the base, owns the distance to the wall
    # ------------------------------------------------------------------
    def _plate_wall_distance(self, ctx):
        """Mean plate-to-wall distance (m) from the last /distance_sensors frame,
        or None when there is no usable reading.

        Averages whichever of the six ranges are inside their validity window.
        A single valid reading is enough — wall_parallel_controller insists on
        three because it fits a *plane*, but a plain standoff needs no fit.
        """
        readings = ctx.get("plate_distances")
        stamp = ctx.get("plate_distances_stamp")
        if not readings or len(readings) != 6 or stamp is None:
            return None
        if time.time() - float(stamp) > PLATE_DISTANCE_MAX_AGE_S:
            return None
        valid = [
            d
            for d, lo, hi in zip(readings, PLATE_SENSOR_VALID_LO, PLATE_SENSOR_VALID_HI)
            if math.isfinite(d) and lo < d < hi
        ]
        if not valid:
            return None
        return sum(valid) / len(valid)

    def _tool0_pose(self, ctx):
        """(translation, rotation) of the plate TCP in the arm planning frame, or
        None. Read from TF rather than /end_effector_pose: that topic ticks at 1 Hz
        and goes stale, which is exactly why wall_parallel_controller reads TF too.
        """
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        base_frame = str(ctx.get("arm_base_frame", "arm_base"))
        tool_frame = str(ctx.get("arm_tool_frame", "arm_tool0"))
        try:
            if not tf_buffer.can_transform(
                base_frame, tool_frame, rclpy.time.Time(), Duration(seconds=1.0)
            ):
                return None
            tf = tf_buffer.lookup_transform(
                base_frame, tool_frame, rclpy.time.Time(), Duration(seconds=1.0)
            )
        except Exception as e:
            ctx["node"].get_logger().warn(
                f"[{self.name}] {base_frame}->{tool_frame} lookup failed: {e}"
            )
            return None
        return tf.transform.translation, tf.transform.rotation

    @staticmethod
    def _tool_z_axis(q):
        """Unit +Z axis of a quaternion-oriented frame, in the parent frame (the
        third column of its rotation matrix). That axis is the one the plate
        sensors measure along and the one force_mode presses along, so an arm move
        along it is a pure approach to / retreat from the wall face."""
        return (
            2.0 * (q.x * q.z + q.w * q.y),
            2.0 * (q.y * q.z - q.w * q.x),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )

    def _send_arm_z_goal(self, ctx, advance):
        """Send the arm a pose goal ``advance`` metres along the plate's +Z axis
        from where it is now, orientation untouched. Positive advances move toward
        the wall, negative retract. Returns True once published.

        Goes out on /arm/goal_pose — the legacy planner_node's Cartesian goal topic,
        which plans in the arm_base frame and reports back on the same
        /execution_status + /planner/goal_failed pair the pre-approach waits on.
        Note the legacy backend has no LIN planner: it runs IK and plans in joint
        space, so the EE path is only approximately straight. Fine over the few
        centimetres these moves cover.
        """
        node = ctx["node"]
        pose = self._tool0_pose(ctx)
        if pose is None:
            node.get_logger().error(
                f"[{self.name}] No arm TF available; cannot place the Z goal."
            )
            return False
        p, q = pose
        zx, zy, zz = self._tool_z_axis(q)

        if self._arm_goal_pub is None:
            topic = str(ctx.get("arm_goal_pose_topic", "/arm/goal_pose"))
            self._arm_goal_pub = node.create_publisher(PoseStamped, topic, 10)

        msg = PoseStamped()
        msg.header.frame_id = str(ctx.get("arm_base_frame", "arm_base"))
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = p.x + zx * advance
        msg.pose.position.y = p.y + zy * advance
        msg.pose.position.z = p.z + zz * advance
        msg.pose.orientation = q
        # Same handshake the named-pose pre-approach uses: the planner flips
        # /execution_status when the motion completes, /planner/goal_failed if it
        # cannot plan. Clear both before publishing so a stale flag is not read as
        # this goal's result.
        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        self._arm_goal_pub.publish(msg)
        node.get_logger().info(
            f"[{self.name}] Arm Z goal: {advance:+.3f} m along the plate normal -> "
            f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, "
            f"{msg.pose.position.z:.3f}) in {msg.header.frame_id}."
        )
        return True

    def _approach_standoff(self, ctx):
        """Plate-to-wall distance the arm approaches a partition at.

        The sweep standoff plus an optional retraction margin. The plate travels
        sideways to the partition start at THIS distance, so the lateral move never
        drags the sensor plate along the wall face -- which in Gazebo means real
        contact forces, and on the robot means scrubbing the GPR wheel across a
        surface it is not measuring.

        ``sweep_approach_retract_m`` is 0 by default: on the robot the margin only
        widened the gap force mode then had to press through (0.40 m against a
        0.25 m Z deviation limit), and the lateral move is now made BEFORE the
        press, so it needs no extra clearance of its own. Raise it to put the
        approach plane back outside the sweep plane -- for the no-press path, where
        nothing holds the plate off the wall, that margin is also the executor's
        post-sweep retract.

        Keyed off ``sweep_scan_standoff_m`` in arm-sweep mode so raising the sweep
        standoff carries the approach out with it; the legacy base-driven path
        keeps using the press offset it was tuned against.
        """
        base = (float(ctx.get("sweep_scan_standoff_m", 0.20))
                if self._use_arm_sweep(ctx)
                else float(ctx.get("scan_wall_plate_offset", 0.20)))
        return base + float(ctx.get("sweep_approach_retract_m", 0.0))

    def _send_arm_to_clearance(self, ctx, target, contact_default=False,
                               retract_only=False):
        """Move the plate so it ends up ``target`` metres off the wall.

        Returns "sent" once a goal is out, "skip" when the move is unnecessary or
        cannot be sized, or "wait" while still waiting for a first sensor frame.

        ``retract_only`` refuses to move the plate TOWARD the wall. A move whose
        whole purpose is to clear the wall should never close on it, whatever the
        arithmetic says -- and the arithmetic can say so, because the standoff
        knobs are independent and drift apart when one is retuned.

        With ``contact_default`` the plate is assumed to be ON the wall when no
        reading is usable — true after a force-mode press, where the ranges sit
        below their valid_lo floor. Everywhere else a missing reading means no
        move at all: retracting blind is survivable, but advancing blind is not,
        and the two share this path.
        """
        node = ctx["node"]
        d = self._plate_wall_distance(ctx)
        if d is None:
            if contact_default:
                d = 0.0
                node.get_logger().info(
                    f"[{self.name}] No valid plate reading (expected: the plate is "
                    f"pressed against the wall); assuming 0.00 m for the retraction."
                )
            else:
                timeout = float(ctx.get("scan_wall_plate_reading_timeout_s", 15.0))
                now = time.time()
                if self._plate_wait_start is None:
                    self._plate_wait_start = now
                    node.get_logger().info(
                        f"[{self.name}] Waiting up to {timeout:.0f}s for a plate "
                        f"distance reading on /distance_sensors..."
                    )
                    return "wait"
                if now - self._plate_wait_start < timeout:
                    return "wait"
                node.get_logger().warn(
                    f"[{self.name}] No plate distance after {timeout:.0f}s; skipping "
                    f"the arm Z move rather than travelling blind toward the wall."
                )
                self._plate_wait_start = None   # so the next segment waits afresh
                return "skip"

        self._plate_wait_start = None
        advance = d - float(target)
        if retract_only and advance > 0.0:
            node.get_logger().info(
                f"[{self.name}] Plate already {d:.3f} m off the wall, further out "
                f"than the {target:.2f} m clearance target; leaving it there rather "
                f"than moving {advance:+.3f} m back toward the wall."
            )
            return "skip"
        max_step = float(ctx.get("scan_wall_arm_z_max_step", 0.6))
        if abs(advance) > max_step:
            node.get_logger().warn(
                f"[{self.name}] Arm Z move of {advance:+.3f} m exceeds the "
                f"{max_step:.2f} m safety step; clamping."
            )
            advance = math.copysign(max_step, advance)
        if abs(advance) < 0.01:
            node.get_logger().info(
                f"[{self.name}] Plate already {d:.3f} m off the wall (target "
                f"{target:.2f} m); no arm move needed."
            )
            return "skip"

        node.get_logger().info(
            f"[{self.name}] Plate measured {d:.3f} m off the wall; moving the arm "
            f"{advance:+.3f} m to sit at {target:.2f} m."
        )
        return "sent" if self._send_arm_z_goal(ctx, advance) else "skip"

    def _send_named_pose(self, ctx, pose_name):
        """Ask the arm to go to a named pose. Returns "sent" / "wait" / "skip".

        Same /send_position + execution_status handshake the pre-approach uses,
        so :meth:`_arm_goal_settled` waits on it unchanged. "skip" means the
        request could not be placed at all -- the caller carries on rather than
        stranding the wall over a re-pose that is an optimisation, not a
        requirement.
        """
        node = ctx["node"]
        if self._recenter_future is not None:
            if not self._recenter_future.done():
                return "wait"
            try:
                response = self._recenter_future.result()
                if not response.success:
                    node.get_logger().warn(
                        f"[{self.name}] Arm rejected the '{pose_name}' pose "
                        f"({response.message}); moving the base from the current "
                        f"arm configuration instead."
                    )
                    self._recenter_future = None
                    return "skip"
                node.get_logger().info(
                    f"[{self.name}] '{pose_name}' accepted: {response.message}"
                )
            except Exception as e:
                node.get_logger().warn(f"[{self.name}] '{pose_name}' service exception: {e}")
                self._recenter_future = None
                return "skip"
            self._recenter_future = None
            return "sent"

        if not self.position_client.service_is_ready():
            node.get_logger().warn(
                f"[{self.name}] Waiting for /send_position to send '{pose_name}'..."
            )
            return "wait"

        node.get_logger().info(f"[{self.name}] Sending the arm to '{pose_name}'.")
        request = SendPosition.Request()
        request.position_name = pose_name
        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        self._recenter_future = self.position_client.call_async(request)
        return "wait"

    def _arm_goal_settled(self, ctx):
        """True once the planner reports the Z move finished, failed, or ran out of
        time. Failures are logged and swallowed: the press and its own bounded
        contact wait degrade gracefully, so a planner hiccup should not strand the
        whole wall."""
        node = ctx["node"]
        timeout = float(ctx.get("scan_wall_arm_goal_timeout_s", 60.0))
        now = time.time()
        if self._arm_goal_start is None:
            self._arm_goal_start = now

        if ctx.get("planner_goal_failed"):
            ctx["planner_goal_failed"] = False
            node.get_logger().warn(
                f"[{self.name}] Planner could not reach the arm Z goal; continuing "
                f"from the current pose."
            )
            return True
        if ctx.get("execution_status") is True:
            ctx["execution_status"] = False
            node.get_logger().info(f"[{self.name}] Arm Z move complete.")
            return True
        if now - self._arm_goal_start >= timeout:
            node.get_logger().warn(
                f"[{self.name}] Arm Z move did not report completion after "
                f"{timeout:.0f}s; continuing anyway."
            )
            return True
        return False

    # ------------------------------------------------------------------
    # Force mode (real robot only): GPR press against the wall
    # ------------------------------------------------------------------
    def _build_force_mode_request(self, ctx):
        """Press the GPR (offset -0.08 m in tool0 X) against the wall with 5 N
        along the task-frame Z; only Z is compliant so the sensor alignment keeps
        full control of orientation."""
        req = SetForceMode.Request()
        req.task_frame.header.frame_id = "arm_tool0"
        req.task_frame.pose.position.x = -0.08
        req.task_frame.pose.orientation.w = 1.0
        req.selection_vector_z = True          # Z compliant; all others stiff
        # ARM_SWEEP_PLAN §7.D: keep the tested value as the DEFAULT, but make it a
        # knob so the press force can be retuned deliberately in one campaign
        # rather than by editing code.
        req.wrench.force.z = float(ctx.get("scan_wall_press_force_n", 5.0))
        if bool(ctx.get("scan_wall_gimbal_press", False)):
            # Experimental passive RX/RY gimbal. Tested once and found unreliable
            # (§4.2), so it is off unless explicitly asked for: with RX/RY
            # compliant the PTC trajectory no longer commands plate orientation
            # and wall_parallel_controller's alignment stops being authoritative.
            req.selection_vector_rx = True
            req.selection_vector_ry = True
            ctx["node"].get_logger().warn(
                f"[{self.name}] scan_wall_gimbal_press is ON: RX/RY are compliant, "
                f"so the commanded plate orientation is no longer enforced. This "
                f"was unreliable when last tested."
            )
        req.type = 2                           # NO_TRANSFORM (task frame as given)
        req.speed_limits.linear.z = 0.1
        # The Z deviation limit is what the press is allowed to travel, so it must
        # cover the gap the arm's approach deliberately leaves plus a margin.
        # Sized from the knob rather than fixed: raising the offset without raising
        # this would leave force_mode short of the wall, and _wall_contact_ready
        # would burn its timeout and sweep without contact.
        press_reach = float(ctx.get("scan_wall_plate_offset", 0.20)) + 0.05
        req.deviation_limits = [0.1, 0.1, max(0.15, press_reach), 0.1, 0.1, 0.1]
        # ...and in arm-sweep mode the approach standoff is a DIFFERENT knob, so
        # check rather than assume. ARM_SWEEP_PLAN §7.C says not to raise the
        # deviation limits automatically -- log it and let the press force /
        # standoff be retuned deliberately in one campaign.
        approach = self._approach_standoff(ctx) if self._use_arm_sweep(ctx) else press_reach
        if approach > req.deviation_limits[2]:
            ctx["node"].get_logger().warn(
                f"[{self.name}] Force mode may not reach the wall: the arm approaches "
                f"at {approach:.2f} m but the Z deviation limit is only "
                f"{req.deviation_limits[2]:.2f} m. Lower sweep_scan_standoff_m / "
                f"sweep_approach_retract_m, or raise the limit deliberately — do not "
                f"expect contact on hardware as configured."
            )
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
        fut = self.force_mode_start_client.call_async(self._build_force_mode_request(ctx))
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

        # Default paired with scan_wall_press_force_n: contact is declared when the
        # sensor reads the commanded press back. §7.C is explicit that retuning one
        # without the other is how a scan ends up sweeping without contact.
        press = float(ctx.get("scan_wall_press_force_n", 5.0))
        threshold = float(ctx.get("scan_wall_touch_force_n", -press))
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
    # GPR fake-encoder trigger — one pulse per X cm of sensor-plate travel
    # ------------------------------------------------------------------
    # Distance the plate must travel between two triggers. 0.5 cm by default;
    # override with ctx/ROS param ``gpr_trigger_distance_m``.
    GPR_TRIGGER_DISTANCE_M = 0.005
    # Sampling rate of the plate pose. Must be well above
    # sweep_speed / trigger_distance (0.05 m/s / 0.005 m = 10 Hz) so the trigger
    # position is quantised to a fraction of the spacing; 50 Hz => ~1 mm.
    GPR_TRIGGER_RATE_HZ = 50.0
    # Per-sample deadband: displacement below this is TF jitter, not motion, and
    # is left on the anchor rather than added to the travel (a 1 mm/s numerical
    # drift would otherwise fire a spurious trigger every 5 s while parked).
    GPR_TRIGGER_MIN_STEP_M = 0.0005
    # Per-sample sanity cap: a jump larger than this is a localisation/TF
    # discontinuity, not plate travel. Re-anchor instead of firing a burst.
    GPR_TRIGGER_MAX_JUMP_M = 0.05
    # Log every Nth trigger at info (0.5 cm spacing means ~200 per metre, so
    # logging each one would drown the console); the rest go to debug.
    GPR_TRIGGER_LOG_EVERY = 20

    def _gpr_trigger_enabled(self, ctx):
        """Whether to emit distance triggers during the sweep.

        Deliberately independent of ``_gpr_enabled``: the probe itself is still
        off by default (not on the robot network yet), but the triggers are what
        we want to watch on the topic/log while validating the spacing.
        """
        return bool(ctx.get("gpr_trigger_enabled", True))

    def _start_gpr_triggers(self, ctx, seg_start=None, seg_end=None):
        """Arm the distance-trigger sampler for the segment sweep that is about to
        start, and fire the d = 0 trigger at the segment start. Counters restart
        per segment, which is also per GPR line scan, so a segment of length L
        yields floor(L / spacing) + 1 triggers.

        ``seg_start``/``seg_end`` give the direction the plate is about to travel
        along the wall; the tick measures progress along it (see _gpr_trigger_tick).
        Falls back to the fixed sweep heading when they are not given (or when the
        segment is degenerate) — only a guard: every caller passes the segment it
        is about to sweep, and on a zero-length segment the plate does not travel
        anyway (the wall heading is fixed per wall, so on a serpentine line swept
        the other way it would point backwards).
        """
        self._stop_gpr_triggers(ctx, log_summary=False)   # never two timers
        if not self._gpr_trigger_enabled(ctx):
            return
        node = ctx["node"]
        axis = self._sweep_axis(seg_start, seg_end)
        if axis is None:
            sweep_yaw = 2.0 * atan2(self._sweep_qz, self._sweep_qw)
            axis = (cos(sweep_yaw), sin(sweep_yaw))
        self._gpr_trigger_axis = axis
        if self._gpr_trigger_pub is None:
            self._gpr_trigger_pub = node.create_publisher(
                UInt32, str(ctx.get("gpr_trigger_topic", "/gpr/trigger")), 50
            )
        self._gpr_trigger_residual = 0.0
        self._gpr_trigger_travel = 0.0
        self._gpr_trigger_count = 0
        self._gpr_trigger_tf_warned = False
        # Anchor on the current plate pose (full timeout: this runs once, off the
        # FSM tick, and TF has to be there before the sweep is worth starting).
        self._gpr_trigger_last_xyz = self._lookup_ee_world_xyz(ctx)
        rate = max(1.0, float(ctx.get("gpr_trigger_rate_hz", self.GPR_TRIGGER_RATE_HZ)))
        # Sub-Nyquist sampling would quantise the triggers into bursts; warn rather
        # than silently mis-space them. Each sweep route carries the plate at its
        # own speed, so read the knob that actually governs this one.
        if self._use_arm_sweep(ctx):
            speed = float(ctx.get("sweep_speed_mps", 0.05))
        elif bool(ctx.get("sweep_use_crawl", False)):
            speed = float(ctx.get("sweep_crawl_speed", self.SWEEP_CRAWL_SPEED_MS))
        else:
            speed = float(ctx.get("sweep_speed_limit", self.SWEEP_SPEED_LIMIT_MS))
        speed = speed or self.SWEEP_SPEED_LIMIT_MS   # 0.0 means "uncapped"
        spacing = self._gpr_trigger_spacing(ctx)
        if rate < 2.0 * speed / spacing:
            node.get_logger().warn(
                f"[{self.name}] GPR trigger sampling at {rate:.0f} Hz is coarse for "
                f"{spacing * 100.0:.2f} cm spacing at {speed:.3f} m/s; triggers will "
                f"come in bursts. Raise gpr_trigger_rate_hz."
            )
        self._gpr_trigger_timer = node.create_timer(
            1.0 / rate, lambda: self._gpr_trigger_tick(ctx)
        )
        node.get_logger().info(
            f"[{self.name}] GPR triggers armed: one every {spacing * 100.0:.2f} cm of "
            f"plate travel, sampled at {rate:.0f} Hz on "
            f"'{ctx.get('gpr_trigger_topic', '/gpr/trigger')}'."
        )
        # The start of the sweep is itself a trigger position (d = 0), so trace #1
        # sits at the segment start and every later one a whole spacing along it.
        if self._gpr_trigger_last_xyz is not None:
            self._emit_gpr_trigger(ctx)
        else:
            node.get_logger().warn(
                f"[{self.name}] GPR triggers: no map->EE transform yet; the d=0 "
                f"trigger fires on the first pose the sampler gets."
            )

    def _stop_gpr_triggers(self, ctx, log_summary=True):
        """Disarm the sampler (segment sweep finished, or state left)."""
        timer = getattr(self, "_gpr_trigger_timer", None)
        if timer is not None:
            timer.cancel()
            ctx["node"].destroy_timer(timer)
            self._gpr_trigger_timer = None
            self._gpr_trigger_axis = None
            if log_summary:
                ctx["node"].get_logger().info(
                    f"[{self.name}] GPR triggers: {self._gpr_trigger_count} fired over "
                    f"{self._gpr_trigger_travel:.3f} m of plate travel."
                )
        self._gpr_trigger_last_xyz = None

    def _gpr_trigger_spacing(self, ctx):
        """Configured trigger spacing in metres (never zero/negative)."""
        spacing = float(ctx.get("gpr_trigger_distance_m", self.GPR_TRIGGER_DISTANCE_M))
        if spacing <= 0.0:
            ctx["node"].get_logger().warn(
                f"[{self.name}] gpr_trigger_distance_m={spacing} is not positive; "
                f"falling back to {self.GPR_TRIGGER_DISTANCE_M} m."
            )
            return self.GPR_TRIGGER_DISTANCE_M
        return spacing

    @staticmethod
    def _sweep_axis(seg_start, seg_end):
        """Unit XY vector from ``seg_start`` to ``seg_end``, or None."""
        if seg_start is None or seg_end is None:
            return None
        dx, dy = seg_end[0] - seg_start[0], seg_end[1] - seg_start[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        return (dx / norm, dy / norm)

    def _gpr_trigger_tick(self, ctx):
        """Timer callback: accumulate how far the plate has advanced along the
        sweep and fire a trigger for every whole ``gpr_trigger_distance_m``.

        Advance is measured as the SIGNED projection of the plate displacement
        onto the sweep axis — the same thing the encoder wheel would roll off as
        it tracks along the wall. Using the raw 3-D path length instead would let
        TF jitter accumulate as phantom travel (a random walk never cancels once
        you take its magnitude), firing triggers with the robot standing still;
        projected, perpendicular noise drops out and along-axis noise averages to
        zero. The axis is always set while the sampler is armed; the path-length
        branch below is only a guard for a sampler ticking without one.

        The residual carries across samples, so triggers stay on a fixed distance
        grid however irregular the sampling is; one sample spanning several
        spacings fires several triggers. Motion backwards along the axis holds the
        triggers instead of firing them, so re-covered ground is not re-triggered
        beyond one spacing of hysteresis.
        """
        node = ctx["node"]
        xyz = self._lookup_ee_world_xyz(ctx, timeout_s=0.0)
        if xyz is None:
            if not self._gpr_trigger_tf_warned:
                node.get_logger().warn(
                    f"[{self.name}] GPR triggers: map->EE transform unavailable; "
                    f"no triggers until it returns."
                )
                self._gpr_trigger_tf_warned = True
            return
        self._gpr_trigger_tf_warned = False

        last = self._gpr_trigger_last_xyz
        if last is None:
            # Late anchor (TF was not up when the sampler was armed): this pose is
            # the sweep's d = 0 position, so it carries the first trigger.
            self._gpr_trigger_last_xyz = xyz
            if self._gpr_trigger_count == 0:
                self._emit_gpr_trigger(ctx)
            return

        axis = self._gpr_trigger_axis
        if axis is None:
            step = math.dist(xyz, last)
        else:
            step = (xyz[0] - last[0]) * axis[0] + (xyz[1] - last[1]) * axis[1]

        min_step = float(ctx.get("gpr_trigger_min_step_m", self.GPR_TRIGGER_MIN_STEP_M))
        if abs(step) < min_step:
            # Below the noise floor: keep the old anchor so real slow motion still
            # accumulates across ticks instead of being discarded sample by sample.
            return
        max_jump = float(ctx.get("gpr_trigger_max_jump_m", self.GPR_TRIGGER_MAX_JUMP_M))
        if abs(step) > max_jump:
            node.get_logger().warn(
                f"[{self.name}] GPR triggers: plate pose jumped {step:.3f} m in one "
                f"sample (> {max_jump:.3f} m); re-anchoring without firing."
            )
            self._gpr_trigger_last_xyz = xyz
            return

        self._gpr_trigger_last_xyz = xyz
        self._gpr_trigger_travel += step
        self._gpr_trigger_residual += step
        spacing = self._gpr_trigger_spacing(ctx)
        # A retreat (a Nav2 recovery, say) only ever costs one spacing: without the
        # clamp, backing up 1 m would leave a 1 m dead zone with no triggers while
        # the sweep re-covers that ground.
        if self._gpr_trigger_residual < -spacing:
            self._gpr_trigger_residual = -spacing
        # Integer grid rather than a subtract-while loop: at an exact multiple
        # (0.04 m of travel at 0.005 m spacing) repeated subtraction leaves
        # 4.9999...e-3 and silently drops the last trigger.
        pending = int(math.floor(self._gpr_trigger_residual / spacing + 1e-9))
        if pending <= 0:
            return
        self._gpr_trigger_residual -= pending * spacing
        for _ in range(pending):
            self._emit_gpr_trigger(ctx)

    def _emit_gpr_trigger(self, ctx):
        """Fire one trigger.

        For now this is the debug stand-in for the fake encoder: a message on
        ``gpr_trigger_topic`` carrying the trigger index within this segment, plus
        a log line. Swap the publish for the fake-encoder hardware call once that
        interface exists — the distance bookkeeping above does not change.
        """
        node = ctx["node"]
        self._gpr_trigger_count += 1
        if self._gpr_trigger_pub is not None:
            msg = UInt32()
            msg.data = self._gpr_trigger_count
            self._gpr_trigger_pub.publish(msg)
        text = (
            f"[{self.name}] GPR trigger #{self._gpr_trigger_count} "
            f"(plate travel {self._gpr_trigger_travel:.3f} m, line "
            f"{ctx.get('current_line_idx', 0) + 1}, segment {self._seg_idx + 1})"
        )
        every = int(ctx.get("gpr_trigger_log_every", self.GPR_TRIGGER_LOG_EVERY))
        if every > 0 and self._gpr_trigger_count % every == 0:
            node.get_logger().info(text)
        else:
            node.get_logger().debug(text)

    # ------------------------------------------------------------------
    # Column height from the map-frame line z
    # ------------------------------------------------------------------
    def _lookup_ee_world_xyz(self, ctx, timeout_s=1.0):
        """Return the end-effector (sensor-plate TCP) position in the map frame as
        ``(x, y, z)``, or None.

        Unlike ``_tool0_pose`` (arm_base -> arm_tool0), this is a WORLD pose: it
        moves when the base slides along the wall, which is what the GPR trigger
        has to measure. The first frame of the fallback chain that resolves is
        cached in ``self._ee_frame``, so the high-rate trigger sampler does not
        re-probe eight frames per tick; pass ``timeout_s=0.0`` there so a momentary
        TF gap cannot block the executor.
        """
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        if self._ee_frame is not None:
            frames = [self._ee_frame]
        else:
            frames = [str(ctx.get("arm_tool_frame", "arm_tool0"))] + [
                f for f in (
                    "arm_tool0", "arm_wrist_3_link", "arm_ee_link", "arm_flange",
                    "tool0", "wrist_3_link", "ee_link", "flange",
                )
                if f != str(ctx.get("arm_tool_frame", "arm_tool0"))
            ]
        timeout = Duration(seconds=float(timeout_s))
        for frame in frames:
            try:
                if tf_buffer.can_transform("map", frame, rclpy.time.Time(), timeout):
                    tf = tf_buffer.lookup_transform("map", frame, rclpy.time.Time(), timeout)
                    self._ee_frame = frame
                    t = tf.transform.translation
                    return (float(t.x), float(t.y), float(t.z))
            except Exception:
                continue
        return None

    def _lookup_ee_world_z(self, ctx):
        """Return the current end-effector height in the map frame, or None."""
        xyz = self._lookup_ee_world_xyz(ctx)
        return None if xyz is None else xyz[2]

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
                pose_name = self._unfolded_pose_name(ctx)
                node.get_logger().info(
                    f"[{self.name}] Pre-approach for line z={self.current_line_z:.3f}m "
                    f"(line {ctx.get('current_line_idx', 0) + 1}/"
                    f"{len(ctx.get('current_wall_scan_lines', [self.current_line_z]))}): "
                    f"sending {pose_name} pose."
                )
                request = SendPosition.Request()
                request.position_name = pose_name
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

        # The arm is out and at line height, so from here on every base move needs a
        # measured plate clearance — including the very first transit. Bring the
        # reader up now and leave it up for the whole line; the alignment controller
        # still waits until the sweep (see _start_wall_alignment).
        self._start_distance_sensors(ctx)
        # Arm-driven sweep: bring the executor up now, not per segment. It needs
        # joint_states and a TF calibration before it can plan, and doing that
        # once per partition would add its startup to all 19 of them.
        if self._use_arm_sweep(ctx):
            self._start_sweep_executor(ctx)

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

    # ------------------------------------------------------------------
    # Arm-driven sweep: partitioning + fixed scan pose (ARM_SWEEP_PLAN §7.A)
    # ------------------------------------------------------------------
    def _unfolded_pose_name(self, ctx):
        """Named arm pose the scan approaches and retracts to.

        Shared with ArmUnfolding and keyed on the same approach decision as
        NavigateToTarget's heading, so the plate is always aimed at the wall
        rather than along it.
        """
        return unfolded_pose_name(ctx)

    def _nest_lines(self, ctx):
        """True when every scan-line height is swept at one base stop (§8).

        Step 2 of ARM_SWEEP_PLAN. Without it the base re-traverses every partition
        for every height: the worked example in §6 is 4 x 8 = 32 base stops for a
        6 m wall at 4 heights, against 8 with nesting. The column is faster and far
        more repeatable than repositioning the base, so trading base transits for
        column moves is a straight win.

        Requires the arm sweep: in the base-driven path the base IS the sweep, so
        there is nothing to nest.
        """
        return bool(ctx.get("nest_lines_in_partition", True)) and self._use_arm_sweep(ctx)

    def _line_heights(self, ctx):
        """Scan-line heights for the current wall, bottom-first."""
        lines = ctx.get("current_wall_scan_lines")
        if not lines:
            lines = [self._resolve_current_line_z(ctx)]
        return list(lines)

    def _use_arm_sweep(self, ctx):
        """True when the arm performs the lateral sweep and the base stays parked.

        The master A/B switch against the legacy base-driven sweep. Everything
        the base-crawl path owns (speed caps, progress-checker relaxation, the
        /cmd_vel drag, chassis parking) is gated off this.

        Defaults to True: wall_sweep_executor has landed, so the arm sweeps and
        the base only repositions between partitions. Set sweep_use_arm False to
        fall back to the base-driven crawl — but set nav_face_wall False with it,
        or the base parks facing the wall and is then dragged sideways at a
        heading meant for driving along it, which is worse than either path
        alone.
        """
        return bool(ctx.get("sweep_use_arm", True))

    # Failures that mean "this partition is too long for the arm", as opposed to
    # a transient or a setup problem. Only these are worth re-cutting for: the
    # arm could not reach along the partition, so a shorter one might.
    LENGTH_FAILURE_REASONS = frozenset({
        "ik_unreachable", "near_singular", "joint_velocity", "branch_discontinuity",
    })

    def _shorten_failed_partition(self, ctx, reason):
        """Re-cut the reachable segment a rejected partition came from, shorter.

        ARM_SWEEP_PLAN §7.B: a partition the validator refuses is NOT halved and
        NOT skipped. The affected reachable segment is re-partitioned as a whole at
        a maximum length reduced by ``partition_length_backoff_m``, so equal
        lengths, full coverage and the configured overlap all survive. Skipping
        instead — which is what happened before this existed — silently drops that
        stretch of wall from the scan.

        Returns True when the segment was re-cut and the sweep should be retried,
        False when it cannot be (floor reached, or the failure is not a
        length problem) and the caller should fall back to skipping.
        """
        node = ctx["node"]
        if reason not in self.LENGTH_FAILURE_REASONS:
            return False
        sources = ctx.get("current_wall_partition_sources") or []
        if self._seg_idx >= len(sources):
            return False
        source = sources[self._seg_idx]

        segments = ctx.get("current_wall_segments") or []
        if source >= len(segments):
            return False

        current = self._segment_max_len.get(
            source, float(ctx.get("partition_max_length_m", 0.8))
        )
        shorter = next_backoff_length(
            current,
            float(ctx.get("partition_length_backoff_m", 0.20)),
            float(ctx.get("partition_min_length_m", 0.20)),
        )
        if shorter is None:
            node.get_logger().error(
                f"[{self.name}] Partition {self._seg_idx + 1} failed [{reason}] and "
                f"its segment is already cut to {current:.2f} m "
                f"(partition_min_length_m); reporting this stretch of wall as "
                f"unreachable rather than shortening further."
            )
            return False

        plan = plan_wall_partitions(
            ctx, self.name, self._sweep_from, self._sweep_to,
            segments=[segments[source]], max_len=shorter,
        )
        if plan is None or not plan[0]:
            node.get_logger().error(
                f"[{self.name}] Re-cutting segment {source} at {shorter:.2f} m "
                f"produced nothing; skipping it."
            )
            return False
        new_partitions, new_poses = plan

        # Splice the replacements in where the old ones were, so the partitions
        # stay in sweep order and every other segment is untouched.
        old_sources = list(sources)
        first = old_sources.index(source)
        last = len(old_sources) - 1 - old_sources[::-1].index(source)
        self._segments = (self._segments[:first] + list(new_partitions)
                          + self._segments[last + 1:])
        self._scan_poses = ((self._scan_poses or [])[:first] + list(new_poses)
                            + (self._scan_poses or [])[last + 1:])
        ctx["current_wall_partition_sources"] = (
            old_sources[:first] + [source] * len(new_partitions) + old_sources[last + 1:]
        )
        self._segment_max_len[source] = shorter
        self._seg_idx = first

        node.get_logger().warn(
            f"[{self.name}] Partition failed [{reason}]; re-cut its segment at "
            f"{shorter:.2f} m into {len(new_partitions)} partition(s) "
            f"(was {last - first + 1}) and retrying from partition {first + 1}. "
            f"Coverage of this stretch is preserved."
        )
        return True

    def _plan_line(self, ctx):
        """Segments to sweep for this scan line, with a base scan pose each.

        Returns ``(segments, scan_poses)`` -- ``scan_poses`` is ``None`` in the
        legacy base-driven mode -- or ``None`` while still polling for the global
        costmap. ``([], None)`` means the state has already been failed.

        In arm-sweep mode this is normally a **cache lookup, not a computation**:
        NavigateToTarget built the same plan to choose its approach pose, so
        partition 1 here is the partition the base is already parked on and its
        transit below collapses to a no-op. Sharing the plan is also what keeps
        partition boundaries identical at every line height (ARM_SWEEP_PLAN §3.6),
        which is what lets the GPR lines stitch.
        """
        node = ctx["node"]
        arm_sweep = self._use_arm_sweep(ctx)

        if arm_sweep:
            plan = plan_wall_partitions(
                ctx, self.name, self._sweep_from, self._sweep_to
            )
            segments, poses = (None, None) if plan is None else plan
        else:
            segments = reachable_wall_segments(ctx, self._sweep_from, self._sweep_to)
            poses = None

        if segments is None:      # costmap not received yet — bounded poll
            now = time.time()
            if self._costmap_wait_start is None:
                node.get_logger().info(
                    f"[{self.name}] Waiting for the global costmap to segment the "
                    f"scan line (up to {COSTMAP_WAIT_TIMEOUT_S:.0f}s)..."
                )
                self._costmap_wait_start = now
                return None
            if now - self._costmap_wait_start < COSTMAP_WAIT_TIMEOUT_S:
                return None
            node.get_logger().warn(
                f"[{self.name}] No costmap after {COSTMAP_WAIT_TIMEOUT_S:.0f}s; "
                f"sweeping the whole line unsegmented."
            )
            whole = [(tuple(self._sweep_from), tuple(self._sweep_to))]
            if arm_sweep:
                # Only the reachability split needs the costmap; the partition
                # geometry and its scan poses need the wall normal alone. So the
                # arm sweep degrades to partitioning the raw line rather than
                # abandoning the wall — obstacles along it are simply not skipped.
                segments, poses = plan_wall_partitions(
                    ctx, self.name, self._sweep_from, self._sweep_to, segments=whole
                )
            else:
                segments = whole

        if not segments:
            # plan_wall_partitions / the arm-reach check logged the specific reason.
            self.fail(
                ctx,
                "no partition of this scan line has a usable base scan pose"
                if arm_sweep
                else "no reachable portion of the scan line within arm reach",
            )
            return [], None

        if not arm_sweep:
            publish_wall_segment_markers(ctx, segments)
            node.get_logger().info(
                f"[{self.name}] Scan line split into {len(segments)} reachable "
                f"segment(s): "
                + "; ".join(
                    f"({s[0]:.2f}, {s[1]:.2f})->({e[0]:.2f}, {e[1]:.2f})"
                    for s, e in segments
                )
            )
        return segments, poses

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
            # other end. Shared with NavigateToTarget — it orders the line the
            # same way to pick which partition to approach, and the two must not
            # disagree or its approach pose belongs to a different partition.
            near_end, far_end = sweep_line_order(wall_data, prev_target_point)
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
            plan = self._plan_line(ctx)
            if plan is None:
                return                    # still polling for the global costmap
            segments, poses = plan
            if not segments:
                return                    # _plan_line already failed the state

            self._segments = segments
            self._scan_poses = poses
            self._seg_idx = 0
            self._seg_phase = "transit_clear"
            self._segments_ok = 0
            return

        # --- All segments processed: finalize the line. ---
        if self._seg_idx >= len(self._segments):
            self._finalize_line(ctx)
            return

        seg_start, seg_end = self._segments[self._seg_idx]
        # Every height sweeps the SAME way by default: one direction per partition,
        # so consecutive GPR lines are acquired identically and nothing downstream
        # has to know which way a line was recorded.
        #
        # §8.2 proposes alternating per height instead ("free and strictly better"),
        # because the arm would finish each height where the next one begins and
        # save a return traverse. It is available behind sweep_serpentine_heights,
        # but it is off until the scan data from a single-direction run is known
        # good -- a reversed line is the kind of thing that only shows up much
        # later, in stitching.
        if (self._nest_lines(ctx)
                and bool(ctx.get("sweep_serpentine_heights", False))
                and self._line_idx % 2 == 1):
            seg_start, seg_end = seg_end, seg_start
        seg_no, seg_total = self._seg_idx + 1, len(self._segments)

        if self._seg_phase == "transit_clear":
            # Pull the plate back before ANY base motion. The arm has been stretched
            # out since the pre-approach, and the transit slides the whole robot
            # sideways along the wall — at the gap the previous sweep left (roughly
            # zero, the plate was pressed against the wall) that drags the plate
            # along the surface. Retract to scan_wall_transit_plate_offset first.
            self.set_activity(
                ctx,
                f"Retracting the arm clear of the wall before moving to segment "
                f"{seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # In arm-sweep mode the executor's own retract leg has already left the
            # plate at the approach standoff, so use that same number here. Using a
            # separate knob made the two drift apart the moment the sweep standoff
            # was retuned, and this move then pushed the plate 10 cm back TOWARD the
            # wall before transit_recenter folded the arm away anyway.
            target = (self._approach_standoff(ctx) if self._use_arm_sweep(ctx)
                      else float(ctx.get("scan_wall_transit_plate_offset", 0.20)))
            # After a sweep the plate sits against the wall with its ranges below
            # their valid floor, so a missing reading there means "in contact", not
            # "unknown" — but only from the second segment on. Before the first
            # transit nothing has touched the wall yet and no such assumption holds.
            outcome = self._send_arm_to_clearance(
                ctx, target, contact_default=self._segments_ok > 0,
                retract_only=True,
            )
            if outcome == "wait":
                return
            if outcome == "sent":
                self._arm_goal_start = None
                self._seg_phase = "transit_clear_wait"
                return
            # "skip" still has to re-centre. The Z retract is usually a no-op now
            # (the executor's own retract leg already left the plate at the
            # approach standoff), and routing that case straight to the transit is
            # what stopped the arm folding back between partitions.
            self._seg_phase = "transit_recenter"
            return

        if self._seg_phase == "transit_clear_wait":
            if not self._arm_goal_settled(ctx):
                return
            self._arm_goal_start = None
            self._seg_phase = "transit_recenter"
            return

        if self._seg_phase == "transit_recenter":
            # Fold the arm back to its named pose before the base moves.
            #
            # The Z retract above only pulls the plate off the wall along the plate
            # normal; it leaves the arm wherever the sweep ended, which is up to
            # half a partition off to one side. Strafing the base with the arm
            # extended sideways swings the plate through a much larger arc than the
            # base itself covers, and the dynamic footprint has to grow to match.
            # Re-posing first puts the plate back on the robot's centreline, so the
            # transit is the base's own footprint moving and nothing more.
            #
            # This is also the pose the next partition's approach starts from, so
            # every partition begins from the same arm configuration rather than
            # from wherever the last sweep happened to finish.
            if not self._use_arm_sweep(ctx):
                self._seg_phase = "transit"
                return
            self.set_activity(
                ctx,
                f"Centring the arm before moving to segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_named_pose(ctx, self._unfolded_pose_name(ctx))
            if outcome == "wait":
                return
            if outcome == "sent":
                self._arm_goal_start = None
                self._seg_phase = "transit_recenter_wait"
                return
            self._seg_phase = "transit"
            return

        if self._seg_phase == "transit_recenter_wait":
            if not self._arm_goal_settled(ctx):
                return
            self._arm_goal_start = None
            self._recenter_future = None
            self._seg_phase = "transit"
            return

        if self._seg_phase == "transit":
            self.set_activity(
                ctx,
                f"Repositioning base to wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # Slide along the wall to the segment start (sensors on, no press). The
            # goal is made wall-parallel first: the arm is out, Nav2 plans against a
            # base-only footprint, and the distance to the wall belongs to the arm's
            # Z approach — the base keeps whatever standoff it already has. Skipped
            # when it is already there: the first segment right after
            # NavigateToTarget, or contiguous serpentine turnarounds.
            pos = ctx.get("base_position")
            skip_tol = float(ctx.get("segment_transit_skip_tol", 0.4))

            # Arm-driven sweep: the Nav2 goal IS the final scan pose — centred on
            # the partition, backed off along the wall normal, facing the wall.
            # No wall_parallel_goal (that strips the wall-ward component to keep
            # the base at its arrived standoff; here the standoff is the point of
            # the goal) and no separate park correction afterwards.
            arm_sweep = self._use_arm_sweep(ctx)
            if arm_sweep:
                pose = (self._scan_poses or [None] * len(self._segments))[self._seg_idx]
                if pose is None:
                    node.get_logger().warn(
                        f"[{self.name}] Partition {seg_no} has no scan pose; skipping."
                    )
                    self._seg_idx += 1
                    self._seg_phase = "transit_clear"
                    return
                goal_xy, goal_yaw = (pose[0], pose[1]), pose[2]
            else:
                goal_yaw = None
                goal_xy = base_standoff_goal(ctx, self.name, seg_start)
                goal_xy = wall_parallel_goal(
                    ctx, self.name, goal_xy, (pos.x, pos.y) if pos is not None else None
                )

            if arm_sweep:
                # Skipping needs BOTH position and heading here. The base-driven
                # sweep only cared about position (it re-parked afterwards), but a
                # parked arm sweep depends on facing the wall square-on, and the
                # base could be at the right spot left over from a previous
                # partition at a completely different heading. Compared in the map
                # frame — ctx["base_position"] is odom and would be off by map->odom.
                here = self._base_xy_yaw_map(ctx)
                # Defaults to NavigateToTarget's own convergence tolerance rather
                # than something tighter: the skip test cannot demand a heading
                # more accurate than navigation is able to deliver, or partition 1
                # re-transits every time despite the base already standing on its
                # scan pose — the exact trip this approach exists to avoid.
                yaw_tol = float(
                    ctx.get("partition_scan_yaw_tol", ctx.get("nav_yaw_tolerance", 0.25))
                )
                if here is not None:
                    yaw_err = abs(atan2(sin(here[2] - goal_yaw), cos(here[2] - goal_yaw)))
                    if math.hypot(here[0] - goal_xy[0], here[1] - goal_xy[1]) <= skip_tol \
                            and yaw_err <= yaw_tol:
                        node.get_logger().info(
                            f"[{self.name}] Base already at partition {seg_no} scan pose "
                            f"(yaw err {yaw_err:.3f} rad); skipping transit."
                        )
                        # No transit happened, so the column was never lowered;
                        # line_column is a no-op unless nesting has moved on to a
                        # different height.
                        self.column_commanded = False
                        self._seg_phase = "line_column"
                        return
            elif pos is not None and math.hypot(pos.x - goal_xy[0], pos.y - goal_xy[1]) <= skip_tol:
                node.get_logger().info(
                    f"[{self.name}] Base already at segment {self._seg_idx + 1} start; "
                    f"skipping transit."
                )
                self._reset_park_state()
                self._seg_phase = "park"
                return
            node.get_logger().info(
                f"[{self.name}] Transit to "
                f"{'partition' if arm_sweep else 'segment'} {self._seg_idx + 1}/"
                f"{len(self._segments)} "
                f"{'scan pose' if arm_sweep else 'start'} "
                f"({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) (no press)."
            )
            # Retract the column BEFORE the base moves, never after. A raised
            # column with the arm on top is a tall, top-heavy load; driving the
            # base like that is unsafe whatever the terrain. Deferred to here
            # rather than done in transit_clear so a SKIPPED transit costs no
            # column travel at all -- above, the skip paths return without ever
            # reaching this point.
            self._pending_transit = (goal_xy, goal_yaw)
            self.column_commanded = False
            self._seg_phase = "transit_column"
            return

        if self._seg_phase == "transit_column":
            self.set_activity(
                ctx,
                f"Retracting the column before moving to segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            if not self.column_commanded:
                node.get_logger().info(
                    f"[{self.name}] Lowering the column to "
                    f"{self.column.column_min_height_m:.3f}m before the base moves. "
                    f"(A no-op when it was never raised.)"
                )
                if not self.column.command(node, ctx, self.column.column_min_height_m):
                    ctx["error_triggered"] = True
                    return
                self.column_commanded = True
                return
            if not self.column.reached_target(node, ctx):
                if self.column.timed_out(node):
                    # Do NOT transit anyway: moving the base with the column up is
                    # the thing this phase exists to prevent.
                    node.get_logger().error(
                        f"[{self.name}] Column did not retract before the transit; "
                        f"refusing to move the base with it raised."
                    )
                    self.fail(ctx, "column did not retract before a base transit")
                return
            self.column.reset()
            self.column_commanded = False
            self._seg_phase = "transit_send"
            return

        if self._seg_phase == "transit_send":
            # Rebind: every phase is a separate tick, so nothing bound in
            # `transit` survives to here.
            arm_sweep = self._use_arm_sweep(ctx)
            if self._pending_transit is None:
                node.get_logger().error(
                    f"[{self.name}] transit_send with no pending goal; restarting "
                    f"the transit for partition {seg_no}."
                )
                self._seg_phase = "transit"
                return
            goal_xy, goal_yaw = self._pending_transit
            self._pending_transit = None
            if arm_sweep and bool(ctx.get("partition_transit_use_crawl", True)):
                # Nav2 cannot make this move. Both scan poses sit inside the
                # costmap inflation band (ARM_SWEEP_PLAN §9: DWB needs footprint +
                # inscribed = 1.3 m of clearance and the scan standoff is
                # necessarily less), so the global planner cannot even reach the
                # goal cell:
                #     GridBased: failed to create plan with tolerance 0.50
                #     No valid trajectories out of 6656! BaseObstacle/...
                # and the recovery behaviours drive the base backwards away from
                # the wall, undoing the scan pose.
                #
                # Between partitions the move is a pure strafe ALONG the wall at
                # constant standoff and constant heading, which the omnidirectional
                # base does directly. Same argument §9 already made for the
                # in-place rotation: DWB is being conservative rather than right.
                # It does mean the move is not obstacle-checked -- acceptable only
                # because reachable_wall_segments already established that a base
                # cell exists within arm reach along this stretch of wall.
                self._start_sweep_crawl(
                    ctx, goal_xy, yaw=goal_yaw,
                    speed=float(ctx.get("partition_transit_speed", 0.15)),
                    # Far looser than the sweep's own 0.05 m. The scan pose is
                    # accepted by NavigateToTarget at nav_pos_tolerance (0.30 m),
                    # so demanding 0.05 m here is stricter than the pose the FSM
                    # was happy with for partition 1 — and the arm re-measures its
                    # own standoff afterwards anyway.
                    tol=float(ctx.get("partition_transit_arrive_tol", 0.15)),
                    timeout_pad=float(ctx.get("partition_transit_timeout_pad_s", 60.0)),
                    what=f"Partition {seg_no} transit",
                )
            elif not self._send_base_goal(ctx, goal_xy, yaw=goal_yaw):
                return
            self._seg_phase = "transit_wait"
            return

        if self._seg_phase == "transit_wait":
            if self._nav_status is None:
                return
            # No-op for a Nav2 transit; stops the timer and zeroes /cmd_vel after a
            # crawl, including the timeout path.
            self._stop_sweep_crawl(ctx, publish_stop=True)
            if self._nav_status == GoalStatus.STATUS_SUCCEEDED:
                # Arm-sweep mode: the Nav2 goal WAS the final scan pose (position
                # and wall-facing heading), so there is nothing left to correct —
                # _run_parking exists only to square the chassis after a transit
                # that ignored heading. Skipping it also avoids rotating the base
                # away from the pose the partition geometry just established.
                if self._use_arm_sweep(ctx):
                    # The column was lowered for the transit, so it always has to
                    # be raised to this partition's scan height before sweeping --
                    # in both modes, not only when nesting.
                    self.column_commanded = False
                    self._seg_phase = "line_column"
                    return
                self._reset_park_state()
                self._seg_phase = "park"
            else:
                node.get_logger().warn(
                    f"[{self.name}] Transit to segment {self._seg_idx + 1} failed "
                    f"(status={self._nav_status}); skipping this segment."
                )
                self._seg_idx += 1
                self._seg_phase = "transit_clear"
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

        if self._seg_phase == "line_change":
            # Between heights at ONE base stop the base does not move, so the plate
            # only has to clear the wall enough for the COLUMN to travel -- much
            # less than the transit case, which exists to stop the plate being
            # dragged sideways while the whole robot slides (§8.2). Its own knob so
            # it can be tried at the scan standoff, where this retract and the
            # following arm_approach both collapse to no-ops.
            heights = self._line_heights(ctx)
            self.current_line_z = heights[self._line_idx]
            self.set_activity(
                ctx,
                f"Raising the column to height {self._line_idx + 1}/{len(heights)} "
                f"at wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_arm_to_clearance(
                ctx,
                float(ctx.get("scan_wall_line_change_plate_offset", 0.20)),
                retract_only=True,
            )
            if outcome == "wait":
                return
            if outcome == "sent":
                self._arm_goal_start = None
                self._seg_phase = "line_change_wait"
                return
            self._seg_phase = "line_column"
            return

        if self._seg_phase == "line_change_wait":
            if not self._arm_goal_settled(ctx):
                return
            self._arm_goal_start = None
            self._seg_phase = "line_column"
            return

        if self._seg_phase == "line_column":
            if self._nest_lines(ctx):
                self.current_line_z = self._line_heights(ctx)[self._line_idx]
            elif self.current_line_z is None:
                self.current_line_z = self._resolve_current_line_z(ctx)
            # The only motion between heights. The arm keeps its pose; §8.2
            # confirmed on hardware that the plate standoff is enough clearance for
            # the column to travel, so no unfolded_fsm re-pose is needed here.
            if not self.column_commanded:
                target_h = self._column_target_for_line(ctx, self.current_line_z)
                node.get_logger().info(
                    f"[{self.name}] Column to {target_h:.3f}m for line "
                    f"z={self.current_line_z:.3f}m; base stays at partition {seg_no}."
                )
                if not self.column.command(node, ctx, target_h):
                    ctx["error_triggered"] = True
                    return
                self.column_commanded = True
                return
            if not self.column.reached_target(node, ctx):
                if self.column.timed_out(node):
                    node.get_logger().error(
                        f"[{self.name}] Column did not reach the height for line "
                        f"{self._line_idx + 1}; skipping this height rather than "
                        f"scanning it at the wrong elevation."
                    )
                    self.column.reset()
                    self.column_commanded = False
                    self._sweep_result = (
                        False, "column_failed", "column did not reach the line height"
                    )
                    self._nav_status = -1
                    self._seg_phase = "sweep_wait"
                return
            self.column.reset()
            self.column_commanded = False
            self._seg_phase = "sweep_setup"
            return

        if self._seg_phase == "sweep_setup":
            self.set_activity(
                ctx,
                f"Preparing the sweep of wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # Nav2 route: cap the base to a crawl speed + relax the progress checker
            # NOW, so the async param change has propagated by the time the base
            # sweeps (the arm approach + stabilise wait below cover it). Skipped when
            # the /cmd_vel crawl is selected (it sets its own speed), and entirely
            # when the ARM sweeps — the base does not move at all then, so capping
            # its speed and relaxing Nav2's progress checker only leave the next
            # transit crawling for no reason (ARM_SWEEP_PLAN §7.C).
            if not self._use_arm_sweep(ctx) and not bool(ctx.get("sweep_use_crawl", False)):
                self._apply_sweep_speed(ctx)
            self._seg_phase = "arm_approach"
            return

        if self._seg_phase == "arm_approach":
            # The base is parked and will not move again until the sweep, so the arm
            # closes the remaining distance itself: measure the plate standoff and
            # travel along the plate's Z axis until only the approach standoff is
            # left for force_mode to press through. Runs BEFORE the lead-in and the
            # alignment controller — that node streams IK setpoints continuously, so
            # nothing else may command the arm while it runs.
            self.set_activity(
                ctx,
                f"Extending the arm toward wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # Normal distance only -- no lateral move here, in either mode.
            #
            # An earlier attempt sent the plate to a Cartesian pose in front of the
            # partition to save the executor the traverse. It cannot work: the
            # partition centre lies on the base's own centreline, and the arm
            # planner carries the robot's column as a 0.3 m static cylinder at the
            # arm_base origin. Commanding the plate there put wrist_3 0.222 m from
            # that axis and the planner refused the endpoint outright
            # ("Global request goal wrist_3 position is INSIDE ENVIRONMENT
            # OBSTACLE"). Retracting further from the wall makes it worse, not
            # better: at this base standoff, back is toward the column.
            #
            # So the arm only ever moves along the plate normal here -- the move
            # that has always worked -- and the lateral traverse to the partition
            # start is the executor's, in the lead_in phase below, where the plate
            # is already extended and well clear of the column.
            target = (self._approach_standoff(ctx) if self._use_arm_sweep(ctx)
                      else float(ctx.get("scan_wall_plate_offset", 0.20)))
            outcome = self._send_arm_to_clearance(ctx, target)
            if outcome == "wait":
                return
            if outcome == "sent":
                self._arm_goal_start = None
                self._seg_phase = "arm_approach_wait"
                return
            # "skip": the plate is already at the approach standoff (or no reading
            # could size the move). The lateral lead-in still has to run.
            self._seg_phase = "lead_in" if self._use_arm_sweep(ctx) else "press_prepare"
            return

        if self._seg_phase == "arm_approach_wait":
            if not self._arm_goal_settled(ctx):
                return
            self._arm_goal_start = None
            self._seg_phase = "lead_in" if self._use_arm_sweep(ctx) else "press_prepare"
            return

        if self._seg_phase == "lead_in":
            # Cross to the partition START before anything presses. The base parks
            # at the partition CENTRE and the FSM's approach can only move along
            # the plate normal, so this lateral move has to happen somewhere; doing
            # it here rather than inside the sweep goal is what stops the GPR wheel
            # being dragged half a partition sideways while pressed against the
            # wall.
            #
            # Same mechanism as before -- the executor's own traverse leg, planned
            # and validated by plan_sweep -- just issued as its own goal. It is NOT
            # sent through /arm/goal_pose: that route goes to the arm planner,
            # which models the column as a 0.3 m obstacle at the arm base and
            # refuses any lateral endpoint near the base centreline.
            self.set_activity(
                ctx,
                f"Moving the arm to the start of wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # The bridge still holds whatever the arm approach queued; let it
            # dispatch during the lead-in and it preempts the executor's goal.
            # Released again below, because press_prepare's alignment controller
            # reaches the arm THROUGH the bridge.
            self._set_trajectory_bridge_hold(ctx, True)
            if not self._send_sweep_goal(ctx, seg_start, seg_end, lead_in_only=True):
                self._set_trajectory_bridge_hold(ctx, False)
                # _send_sweep_goal already filled in _sweep_result; hand the
                # failure to sweep_wait, which owns the backoff and the skip.
                self._seg_phase = "sweep_wait"
                return
            self._seg_phase = "lead_in_wait"
            return

        if self._seg_phase == "lead_in_wait":
            if self._sweep_result is None:
                return
            succeeded, reason, detail = self._sweep_result
            self._sweep_result = None
            self._set_trajectory_bridge_hold(ctx, False)
            if not succeeded:
                node.get_logger().error(
                    f"[{self.name}] Lead-in to segment {self._seg_idx + 1} failed "
                    f"[{reason}]: {detail}; not pressing against the wall."
                )
                # Re-raised for sweep_wait: a partition whose lead-in cannot be
                # planned is exactly the "too long for the arm" case its backoff
                # (§7.B) exists for, and the teardown there is a no-op when
                # nothing was started.
                self._sweep_result = (succeeded, reason, detail)
                self._seg_phase = "sweep_wait"
                return
            node.get_logger().info(
                f"[{self.name}] Plate at the start of segment {self._seg_idx + 1}; "
                f"starting the alignment controller and the press."
            )
            self._seg_phase = "press_prepare"
            return

        if self._seg_phase == "press_prepare":
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
            # The reader has been up since the pre-approach; this is a no-op unless it
            # died. The alignment controller starts only now, with the arm already at
            # its approach pose and no other goal in flight.
            if not self._start_distance_sensors(ctx):
                return
            if not self._start_wall_alignment(ctx):
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
            #
            # NOT in arm-sweep mode. The plate is at the partition start by now
            # (the lead_in phase put it there), but the executor still has to
            # preempt whatever holds the arm and settle it before the sweep leg
            # begins, and a line started here would record that dead time as scan
            # data. It starts instead on the executor's "sweep" feedback phase, in
            # sweep_wait.
            if not self._use_arm_sweep(ctx):
                self._gpr_start_measurement_and_line(ctx)
                if ctx.get("error_triggered"):
                    return

            self.set_activity(
                ctx,
                f"Sweeping wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )

            if self._use_arm_sweep(ctx):
                # The ARM sweeps; the base does not move at all. Stop the alignment
                # controller's trajectory output first: it publishes to
                # planned_trajectory, and two publishers driving one arm controller
                # is not survivable (ARM_SWEEP_PLAN §3.3). The distance sensors stay
                # up — the executor's contact watchdog reads them.
                self._stop_arm_processes(ctx, keep_sensors=True)
                # Stop the trajectory BRIDGE from dispatching too. Killing the
                # alignment controller leaves its last trajectory queued in the
                # bridge, which re-sends it the moment the executor's preempt frees
                # a slot — and that goal then preempts the executor's own, which is
                # how the traverse leg came back CANCELED.
                self._set_trajectory_bridge_hold(ctx, True)
                # No sleep here. Killing a node does not stop the trajectory it
                # already handed to the controller — one was measured still
                # executing 67 s later — so a fixed wait is guesswork either way.
                # The executor preempts whatever is in flight and waits for the
                # joints to go quiet before it reads them.
                if not self._send_sweep_goal(ctx, seg_start, seg_end):
                    self._set_trajectory_bridge_hold(ctx, False)
                    return
                self._seg_phase = "sweep_wait"
                return

            # Legacy base-driven sweep. Nav2 route (default): slow Nav2 sweep,
            # speed already capped in sweep_setup. Alternative
            # (ctx["sweep_use_crawl"]): a /cmd_vel drag.
            goal_xy = base_standoff_goal(ctx, self.name, seg_end)
            node.get_logger().info(
                f"[{self.name}] Sweeping segment {self._seg_idx + 1} to "
                f"({goal_xy[0]:.2f}, {goal_xy[1]:.2f})."
            )
            if bool(ctx.get("sweep_use_crawl", False)):
                self._start_sweep_crawl(ctx, goal_xy)
            elif not self._send_base_goal(ctx, goal_xy):
                return
            # Plate is on the wall and the base is moving: start clocking the GPR
            # with one trigger per gpr_trigger_distance_m of plate travel.
            self._start_gpr_triggers(ctx, seg_start, seg_end)
            self._seg_phase = "sweep_wait"
            return

        if self._seg_phase == "sweep_wait":
            self.set_activity(
                ctx,
                f"Sweeping wall segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            arm_sweep = self._use_arm_sweep(ctx)
            reason = ""      # only set on the arm-sweep path; read by the backoff
            if arm_sweep:
                # The lead-in is done and the plate is now crossing the wall: this
                # is the moment the GPR line has to start (see press_settle) and,
                # for the same reason, the moment to start clocking it. Arming any
                # earlier would trigger through the lead-in traverse and the
                # executor's settle, recording plate travel that is not scan data;
                # this feedback is also why none of the three no-sweep paths into
                # sweep_wait (column failure, lead-in dispatch, lead-in failure)
                # can arm the sampler.
                if self._sweep_scanning and not self.gpr_line_active:
                    self._gpr_start_measurement_and_line(ctx)
                    if ctx.get("error_triggered"):
                        return
                    # Armed once per sweep, not once per tick: with the probe
                    # disabled (the default) gpr_line_active never latches, so
                    # this block is re-entered every tick for the whole sweep and
                    # a bare arm call would restart the counter each second.
                    if self._gpr_trigger_timer is None:
                        self._start_gpr_triggers(ctx, seg_start, seg_end)
                # The executor owns the sweep's own watchdogs and reports one
                # outcome; everything after it here is unchanged.
                if self._sweep_result is None:
                    return
                self._stop_gpr_triggers(ctx)   # the plate has stopped scanning
                succeeded, reason, detail = self._sweep_result
                self._sweep_result = None
                status = GoalStatus.STATUS_SUCCEEDED if succeeded else GoalStatus.STATUS_ABORTED
                if succeeded and reason:
                    # Success with a reason: the scan is good but a cleanup step
                    # failed (currently only the post-sweep retract, which
                    # transit_clear will redo from a measured distance).
                    node.get_logger().warn(
                        f"[{self.name}] Arm sweep of segment {self._seg_idx + 1} "
                        f"completed with a warning [{reason}]: {detail}"
                    )
                if not succeeded:
                    node.get_logger().error(
                        f"[{self.name}] Arm sweep of segment {self._seg_idx + 1} "
                        f"failed [{reason}]: {detail}"
                    )
            else:
                # The crawl timer sets self._nav_status on arrival/timeout.
                if self._nav_status is None:
                    return
                status = self._nav_status
                self._stop_sweep_crawl(ctx, publish_stop=True)   # ensure base is stopped
                self._stop_gpr_triggers(ctx)     # no triggers once the plate stops moving
                self._restore_sweep_speed(ctx)   # clear the slow-sweep cap for the next transit
            # Release hardware/process state first (safety), regardless of outcome.
            node.get_logger().info(
                f"[{self.name}] Segment sweep finished (status={status}). Stopping GPR, "
                f"force mode + arm processes..."
            )
            if arm_sweep:
                # The executor is done with the arm; the bridge owns it again for
                # the retract and the next transit's arm Z moves.
                self._set_trajectory_bridge_hold(ctx, False)
            self._gpr_stop_line_and_measurement(ctx)   # stop line + measurement before releasing the press
            self._stop_force_mode(ctx)      # release the press before the arm retracts
            # Keep the reader running: the next segment's transit_clear needs the
            # plate distance to retract the arm before the base moves. Only the
            # alignment controller has to go, so the retraction goal is not fighting
            # its IK stream.
            self._stop_arm_processes(ctx, keep_sensors=True)
            time.sleep(2)   # delay to avoid errors in the arm goals

            if status == GoalStatus.STATUS_SUCCEEDED:
                self._segments_ok += 1
                self._last_swept_point = seg_end
            else:
                # "Too long for the arm" is recoverable: re-cut this partition's
                # SEGMENT shorter and sweep it again, rather than dropping that
                # stretch of wall (ARM_SWEEP_PLAN §7.B). _shorten_failed_partition
                # rewinds _seg_idx itself when it succeeds.
                if arm_sweep and self._shorten_failed_partition(ctx, reason):
                    self._line_idx = 0
                    self._seg_phase = "transit_clear"
                    return
                node.get_logger().warn(
                    f"[{self.name}] Segment {self._seg_idx + 1} sweep did not succeed "
                    f"(status={status}); skipping to the next segment."
                )

            # Nesting (§8): sweep every height at this base stop before moving the
            # base. A height that fails moves to the NEXT HEIGHT, not away from the
            # partition -- §8.2's failure isolation: losing one line must not cost
            # the whole stretch of wall.
            if self._nest_lines(ctx):
                heights = self._line_heights(ctx)
                if self._line_idx + 1 < len(heights):
                    self._line_idx += 1
                    ctx["current_line_idx"] = self._line_idx
                    node.get_logger().info(
                        f"[{self.name}] Partition {self._seg_idx + 1}: on to height "
                        f"{self._line_idx + 1}/{len(heights)} "
                        f"(z={heights[self._line_idx]:.3f}m). Base stays parked."
                    )
                    self._seg_phase = "line_change"
                    return
                self._line_idx = 0
                ctx["current_line_idx"] = 0

            self._seg_idx += 1
            self._seg_phase = "transit_clear"
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
        lines = ctx.get("current_wall_scan_lines", [])
        if self._nest_lines(ctx):
            # Every height was already swept at every partition, inside the phase
            # machine. There is no next line to re-enter for: the state exits once
            # (§8.1), and the column retract + walls_left in _run_post_scan run at
            # the end of the WALL rather than at the end of a line.
            ctx["current_line_idx"] = len(lines)
            self.more_lines = False
        else:
            ctx["current_line_idx"] = ctx.get("current_line_idx", 0) + 1
            self.more_lines = ctx["current_line_idx"] < len(lines)
        node.get_logger().info(
            f"[{self.name}] "
            + (f"Wall done: {len(lines)} height(s) x {len(self._segments)} partition(s), "
               f"{self._segments_ok} sweep(s) succeeded."
               if self._nest_lines(ctx) else
               f"Line done ({self._segments_ok}/{len(self._segments)} segment(s) swept).")
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
    # ------------------------------------------------------------------
    # Arm-driven sweep: the SweepLine action (ARM_SWEEP_PLAN §7.B)
    # ------------------------------------------------------------------
    def _set_trajectory_bridge_hold(self, ctx, held):
        """Tell publisher_joint_trajectory_planned to stand down, or resume.

        The bridge forwards ``planned_trajectory`` to the same controller the
        executor drives, so while a sweep is running the two preempt each other's
        goals and the sweep comes back CANCELED. Killing wall_parallel_controller
        does not prevent it: its last trajectory is still queued in the bridge, and
        the bridge re-dispatches it as soon as a slot opens -- which is exactly
        what happened to the traverse leg (JTC status 5).

        Latched (transient_local) so the bridge picks the hold up even if it
        restarts mid-sweep.
        """
        node = ctx["node"]
        pub = ctx.get("_traj_hold_pub")
        if pub is None:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            pub = node.create_publisher(Bool, "trajectory_bridge/hold", qos)
            ctx["_traj_hold_pub"] = pub
        pub.publish(Bool(data=bool(held)))
        node.get_logger().info(
            f"[{self.name}] Trajectory bridge {'held for the arm sweep' if held else 'released'}."
        )

    def _send_sweep_goal(self, ctx, seg_start, seg_end, lead_in_only=False):
        """Hand one partition to wall_sweep_executor. The base does not move.

        With ``lead_in_only`` the executor runs the lateral lead-in and stops,
        leaving the plate at the partition start for force mode to press from
        (see the ``lead_in`` phase). The result comes back the same way, so the
        caller polls ``self._sweep_result`` either way.

        The result lands in ``self._sweep_result`` as
        ``(succeeded, reason, detail)``; ``None`` while the sweep is in flight,
        which is what ``sweep_wait`` polls.

        The endpoints go over in the MAP frame exactly as the partitioning
        produced them: ``normalize(end - start)`` is the authoritative sweep
        direction (§4.2), and re-deriving it from the arm's current pose would
        substitute where the plate happens to point for where the wall actually
        is. The executor transforms them into ``arm_base`` once — safe because
        the base is parked for the whole sweep.
        """
        node = ctx["node"]
        if self._sweep_client is None:
            self._sweep_client = ActionClient(node, SweepLine, "sweep_line")

        # Restart a dead executor rather than failing every remaining partition
        # of the line. It is started once per line in the pre-approach, so without
        # this a single crash silently costs the whole wall.
        proc = ctx.get("sweep_executor_proc")
        if proc is not None and proc.poll() is not None:
            node.get_logger().warn(
                f"[{self.name}] wall_sweep_executor exited (code {proc.returncode}); "
                f"restarting it before this sweep."
            )
            ctx["sweep_executor_proc"] = None
            self._start_sweep_executor(ctx)

        if not self._sweep_client.wait_for_server(timeout_sec=10.0):
            # Say WHY. A dead executor and a slow one look identical from the
            # action client, and the difference is the whole diagnosis: an
            # unparseable --ros-args (a `-p name:=` with an empty value, say)
            # kills the node before it can advertise, and the only symptom
            # upstream was this timeout.
            proc = ctx.get("sweep_executor_proc")
            if proc is None:
                why = "the executor was never started"
            elif proc.poll() is not None:
                why = (f"the executor process exited with code {proc.returncode} — "
                       f"check its stderr, most likely a bad --ros-args parameter")
            else:
                why = f"the executor (pid {proc.pid}) is running but has not advertised"
            node.get_logger().error(
                f"[{self.name}] wall_sweep_executor's 'sweep_line' action never came "
                f"up: {why}. Cannot sweep this partition with the arm."
            )
            self._sweep_result = (False, "executor_unavailable", why)
            return False

        # The partitions carry the wall's BASE scan-line height, which is only the
        # first line's. Stamp the height being scanned now, or every line after the
        # first sweeps at the wrong elevation.
        line_z = self.current_line_z
        if line_z is None:
            line_z = self._resolve_current_line_z(ctx)
        line_z = float(line_z)

        goal = SweepLine.Goal()
        goal.start = Point(x=float(seg_start[0]), y=float(seg_start[1]), z=line_z)
        goal.end = Point(x=float(seg_end[0]), y=float(seg_end[1]), z=line_z)
        goal.partition_index = int(self._seg_idx)
        goal.partition_count = int(len(self._segments))
        goal.frame_id = "map"
        goal.speed = float(ctx.get("sweep_speed_mps", 0.05))
        # Sim has no force_mode controller, so the sweep runs contact-free at the
        # plate offset (§11.2) and the contact watchdog must not fire.
        goal.press = not bool(ctx.get("sim", False))
        # A lead-in goal runs BEFORE force mode, so nothing is pressing yet. It
        # still carries press=True on the robot: that is what tells the executor
        # to traverse in the plate's current plane instead of sizing a plunge from
        # the range sensors, and the plane it is in is the approach standoff.
        goal.lead_in_only = bool(lead_in_only)

        node.get_logger().info(
            f"[{self.name}] Arm {'lead-in to' if lead_in_only else 'sweep of'} partition "
            f"{self._seg_idx + 1}/{len(self._segments)}: "
            f"({seg_start[0]:.2f}, {seg_start[1]:.2f}) -> "
            f"({seg_end[0]:.2f}, {seg_end[1]:.2f}) at {goal.speed:.3f} m/s "
            f"({'press' if goal.press else 'no press'}). Base stays parked."
        )
        self._sweep_result = None
        self._sweep_goal_handle = None
        self._sweep_scanning = False
        future = self._sweep_client.send_goal_async(
            goal, feedback_callback=self._on_sweep_feedback
        )
        future.add_done_callback(self._on_sweep_response)
        return True

    def _on_sweep_feedback(self, message):
        if message.feedback.phase == "sweep":
            self._sweep_scanning = True

    def _on_sweep_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self._sweep_result = (False, "executor_exception", str(e))
            return
        if not handle.accepted:
            self._sweep_result = (False, "goal_rejected", "executor rejected the sweep goal")
            return
        self._sweep_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_sweep_result)

    def _on_sweep_result(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self._sweep_result = (False, "executor_exception", str(e))
            return
        self._sweep_result = (bool(result.success), result.reason, result.detail)

    def _cancel_sweep_goal(self, ctx):
        """Cancel an in-flight arm sweep on the way out of the state.

        Killing the executor process alone is not enough: SIGINT during a
        FollowJointTrajectory goal leaves the controller running the rest of the
        trajectory, so the arm would keep sweeping after the FSM has moved on.
        Cancelling first lets the executor preempt the controller goal properly.
        """
        handle = self._sweep_goal_handle
        self._sweep_goal_handle = None
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
            ctx["node"].get_logger().info(f"[{self.name}] Cancelled the in-flight arm sweep.")
        except Exception as e:
            ctx["node"].get_logger().warn(f"[{self.name}] Sweep cancel failed: {e}")

    def _send_base_goal(self, ctx, goal_xy, yaw=None):
        """Send a NavigateToPose goal. The result lands in ``self._nav_status``
        (a GoalStatus value, or -1 on rejection/exception); ``None`` while in
        flight.

        Defaults to the fixed along-wall sweep heading used by the base-driven
        sweep. ``yaw`` overrides it with a per-goal heading — the arm-driven
        sweep parks facing INTO the wall instead (ARM_SWEEP_PLAN §7.A), so it
        passes the yaw from ``partition_scan_pose``.
        """
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
        if yaw is None:
            goal_msg.pose.pose.orientation.z = self._sweep_qz
            goal_msg.pose.pose.orientation.w = self._sweep_qw
        else:
            goal_msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
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

    # ------------------------------------------------------------------
    # Sweep crawl: slow constant-velocity drag over /cmd_vel
    # ------------------------------------------------------------------
    # Default drag speed found most stable for wall scanning (the arm's contact
    # controllers keep up). Override per run via ctx["sweep_crawl_speed"].
    SWEEP_CRAWL_SPEED_MS = 0.05
    SWEEP_CRAWL_RATE_HZ = 10.0        # /cmd_vel publish rate (base drivers stop on stale cmd)
    SWEEP_ARRIVE_TOL_M = 0.05         # stop when the base is within this of the goal
    SWEEP_KP_YAW = 0.9                # P-hold on the fixed sweep heading
    SWEEP_MAX_YAW_RATE = 0.3          # rad/s cap on the heading hold
    SWEEP_CRAWL_TIMEOUT_PAD_S = 20.0  # grace beyond nominal (dist / speed) before aborting

    # Base frames tried when reading the map-frame base pose (turret_footprint
    # first: it is the frame the omni Nav2 config drives). Mirrors NavigateToTarget.
    BASE_FRAMES = ("turret_footprint", "base_footprint", "base_link", "base", "chassis")

    def _base_xy_yaw_map(self, ctx):
        """Current base pose (x, y, yaw) in the map frame from TF, or None.

        The sweep segments are map-frame, so progress must be measured in map too
        (ctx["base_position"] is the odom frame and would be off by map->odom).
        """
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        primary = str(ctx.get("nav_base_frame", "turret_footprint"))
        frames = (primary,) + tuple(f for f in self.BASE_FRAMES if f != primary)
        for frame in frames:
            try:
                if not tf_buffer.can_transform(
                    "map", frame, rclpy.time.Time(), Duration(seconds=0.2)
                ):
                    continue
                tf = tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time(), Duration(seconds=0.5)
                )
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return float(t.x), float(t.y), yaw
        return None

    def _start_sweep_crawl(self, ctx, target_xy, yaw=None, speed=None, tol=None,
                           timeout_pad=None, what="Sweep crawl"):
        """Begin driving the base to ``target_xy`` (map frame) over /cmd_vel.

        Drives ``self._nav_status`` like a Nav2 goal: None while in flight,
        STATUS_SUCCEEDED on arrival, -1 on timeout/lost pose, so both the
        ``sweep_wait`` and ``transit_wait`` completion logic are reused unchanged.

        ``yaw`` overrides the heading held during the move; it defaults to the
        fixed along-wall sweep heading the legacy base sweep uses. The partition
        transit passes the scan-pose yaw instead, so the base arrives already
        square to the wall.
        """
        node = ctx["node"]
        if ctx.get("_cmd_vel_pub") is None:
            ctx["_cmd_vel_pub"] = node.create_publisher(Twist, "/cmd_vel", 10)
        self._stop_sweep_crawl(ctx)   # no duplicate timers
        self._sweep_crawl_target = (float(target_xy[0]), float(target_xy[1]))
        self._crawl_yaw = None if yaw is None else float(yaw)
        self._nav_status = None

        if speed is None:
            speed = float(ctx.get("sweep_crawl_speed", self.SWEEP_CRAWL_SPEED_MS))
        self._crawl_speed = float(speed)
        self._crawl_tol = tol
        self._crawl_started = time.time()
        pose = self._base_xy_yaw_map(ctx)
        dist = (
            math.hypot(self._sweep_crawl_target[0] - pose[0],
                       self._sweep_crawl_target[1] - pose[1])
            if pose else 0.0
        )
        self._crawl_distance = dist
        # The base tracks a /cmd_vel command far more slowly than it is asked to
        # (measured at roughly a seventh of the commanded speed on the omni base in
        # Gazebo), so the nominal dist/speed is not a usable estimate on its own.
        pad = self.SWEEP_CRAWL_TIMEOUT_PAD_S if timeout_pad is None else float(timeout_pad)
        self._sweep_crawl_deadline = time.time() + dist / max(speed, 1e-3) + pad
        node.get_logger().info(
            f"[{self.name}] {what} to ({self._sweep_crawl_target[0]:.2f}, "
            f"{self._sweep_crawl_target[1]:.2f}) at {speed:.3f} m/s ({dist:.2f} m)."
        )
        self._sweep_crawl_timer = node.create_timer(
            1.0 / self.SWEEP_CRAWL_RATE_HZ, lambda: self._sweep_crawl_tick(ctx)
        )

    def _stop_sweep_crawl(self, ctx, publish_stop=False):
        timer = getattr(self, "_sweep_crawl_timer", None)
        if timer is not None:
            timer.cancel()
            ctx["node"].destroy_timer(timer)
            self._sweep_crawl_timer = None
        if publish_stop and ctx.get("_cmd_vel_pub") is not None:
            ctx["_cmd_vel_pub"].publish(Twist())   # zero twist

    def _sweep_crawl_tick(self, ctx):
        """Timer callback: publish a constant-speed Twist toward the target and
        stop (set _nav_status) on arrival / timeout / lost pose."""
        if self._nav_status is not None:
            return
        node = ctx["node"]
        speed = getattr(self, "_crawl_speed", None) or float(
            ctx.get("sweep_crawl_speed", self.SWEEP_CRAWL_SPEED_MS)
        )
        tol = getattr(self, "_crawl_tol", None)
        if tol is None:
            tol = float(ctx.get("sweep_arrive_tol", self.SWEEP_ARRIVE_TOL_M))
        tol = float(tol)

        pose = self._base_xy_yaw_map(ctx)
        if pose is None:
            node.get_logger().error(f"[{self.name}] Lost base pose during sweep crawl.")
            self._stop_sweep_crawl(ctx, publish_stop=True)
            self._nav_status = -1
            return
        px, py, yaw = pose
        ex, ey = self._sweep_crawl_target[0] - px, self._sweep_crawl_target[1] - py
        dist = math.hypot(ex, ey)

        if dist <= tol:
            self._stop_sweep_crawl(ctx, publish_stop=True)
            node.get_logger().info(
                f"[{self.name}] Base crawl reached its target ({dist:.3f} m)."
            )
            self._nav_status = GoalStatus.STATUS_SUCCEEDED
            return
        if time.time() > self._sweep_crawl_deadline:
            self._stop_sweep_crawl(ctx, publish_stop=True)
            elapsed = max(time.time() - getattr(self, "_crawl_started", time.time()), 1e-3)
            covered = max(getattr(self, "_crawl_distance", 0.0) - dist, 0.0)
            node.get_logger().warn(
                f"[{self.name}] Base crawl timed out {dist:.3f} m from the target "
                f"(tolerance {tol:.3f} m). Covered {covered:.3f} m in {elapsed:.1f}s "
                f"= {covered / elapsed:.3f} m/s against {speed:.3f} m/s commanded — "
                f"if those differ a lot, /cmd_vel is being throttled or arbitrated "
                f"away before it reaches the base."
            )
            self._nav_status = -1
            return

        # Constant-magnitude velocity toward the target, rotated into the base
        # frame (Twist is body-frame), plus a P-hold on the fixed sweep heading so
        # the omni base strafes along the wall without turning.
        vx_w, vy_w = speed * ex / dist, speed * ey / dist
        cmd = Twist()
        cmd.linear.x = cos(yaw) * vx_w + sin(yaw) * vy_w
        cmd.linear.y = -sin(yaw) * vx_w + cos(yaw) * vy_w
        crawl_yaw = getattr(self, "_crawl_yaw", None)
        target_yaw = (crawl_yaw if crawl_yaw is not None
                      else 2.0 * atan2(self._sweep_qz, self._sweep_qw))
        dyaw = atan2(sin(target_yaw - yaw), cos(target_yaw - yaw))
        cmd.angular.z = max(-self.SWEEP_MAX_YAW_RATE,
                            min(self.SWEEP_MAX_YAW_RATE, self.SWEEP_KP_YAW * dyaw))
        ctx["_cmd_vel_pub"].publish(cmd)

    # ------------------------------------------------------------------
    # Nav2 slow-sweep: cap DWB speed + relax the progress checker
    # ------------------------------------------------------------------
    # Absolute base-speed cap during the sweep (0.0 => no limit). Found most
    # stable at 0.05 m/s; override via ctx["sweep_speed_limit"].
    SWEEP_SPEED_LIMIT_MS = 0.05
    # Relaxed progress checker so a sub-0.1 m/s sweep is not aborted as "no
    # progress" (must move SWEEP_PROGRESS_RADIUS_M within SWEEP_PROGRESS_TIME_S).
    SWEEP_PROGRESS_RADIUS_M = 0.10
    SWEEP_PROGRESS_TIME_S = 15.0
    # Restored after the sweep (the nav2_params_omni.yaml progress_checker defaults).
    DEFAULT_PROGRESS_RADIUS_M = 0.50
    DEFAULT_PROGRESS_TIME_S = 5.0

    def _publish_speed_limit(self, ctx, speed_ms):
        """Absolute base-speed cap for the Nav2 controller (0.0 => no limit).
        Latched so the controller_server picks it up even if it subscribed late."""
        node = ctx["node"]
        if self._speed_limit_pub is None:
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._speed_limit_pub = node.create_publisher(
                SpeedLimit, ctx.get("speed_limit_topic", "/speed_limit"), qos)
        msg = SpeedLimit()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.percentage = False
        msg.speed_limit = float(speed_ms)
        self._speed_limit_pub.publish(msg)

    def _set_progress_checker(self, ctx, radius_m, time_s):
        """Relax/restore the controller_server progress checker at runtime."""
        node = ctx["node"]
        srv = ctx.get("controller_set_param_service", "/controller_server/set_parameters")
        if self._controller_param_client is None:
            self._controller_param_client = node.create_client(SetParameters, srv)
        if not self._controller_param_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().warn(
                f"[{self.name}] '{srv}' unavailable; cannot adjust the progress checker "
                f"(the slow sweep may be aborted as 'no progress')."
            )
            return
        checker = ctx.get("progress_checker_name", "progress_checker")
        req = SetParameters.Request()
        req.parameters = [
            ParameterMsg(
                name=f"{checker}.required_movement_radius",
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(radius_m))),
            ParameterMsg(
                name=f"{checker}.movement_time_allowance",
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(time_s))),
        ]
        fut = self._controller_param_client.call_async(req)
        fut.add_done_callback(
            lambda f: self._log_param_result(ctx, f, f"progress checker r={radius_m}m t={time_s}s")
        )

    def _log_param_result(self, ctx, future, label):
        node = ctx["node"]
        try:
            results = future.result().results
            if bool(results) and all(r.successful for r in results):
                node.get_logger().info(f"[{self.name}] {label}: applied.")
            else:
                reason = results[0].reason if results else "no result"
                node.get_logger().warn(
                    f"[{self.name}] {label}: rejected ({reason}). If your Nav2 build does not "
                    f"reconfigure the progress checker at runtime, relax it in nav2_params_omni.yaml."
                )
        except Exception as e:
            node.get_logger().warn(f"[{self.name}] {label}: set_parameters call failed ({e}).")

    def _apply_sweep_speed(self, ctx):
        """Scope a slow base speed to the sweep: cap DWB + relax the progress
        checker. Applied in sweep_setup (the 10 s arm-stabilise wait lets the
        async param change propagate before the base moves)."""
        node = ctx["node"]
        speed = float(ctx.get("sweep_speed_limit", self.SWEEP_SPEED_LIMIT_MS))
        node.get_logger().info(f"[{self.name}] Slow sweep: capping base to {speed:.3f} m/s.")
        self._publish_speed_limit(ctx, speed)
        self._set_progress_checker(
            ctx,
            float(ctx.get("sweep_progress_radius", self.SWEEP_PROGRESS_RADIUS_M)),
            float(ctx.get("sweep_progress_time", self.SWEEP_PROGRESS_TIME_S)),
        )
        self._sweep_speed_applied = True

    def _restore_sweep_speed(self, ctx):
        """Undo _apply_sweep_speed: clear the speed cap and restore the progress
        checker. Idempotent no-op if nothing was applied."""
        if not self._sweep_speed_applied:
            return
        self._publish_speed_limit(ctx, 0.0)   # 0.0 => no limit
        self._set_progress_checker(
            ctx,
            float(ctx.get("progress_radius_default", self.DEFAULT_PROGRESS_RADIUS_M)),
            float(ctx.get("progress_time_default", self.DEFAULT_PROGRESS_TIME_S)),
        )
        self._sweep_speed_applied = False

    def _run_post_scan(self, ctx):
        """Retract the arm from the wall (re-send the pose); on the wall's last
        line also retract the column. Sets self.finished when complete."""
        node = ctx["node"]

        # Step 1: pull the arm back from the wall by re-sending the pose.
        if not self.retract_pose_sent:
            if not self.position_client.service_is_ready():
                node.get_logger().warn(f"[{self.name}] Waiting for /send_position service (retract)...")
                return
            pose_name = self._unfolded_pose_name(ctx)
            request = SendPosition.Request()
            request.position_name = pose_name
            ctx["execution_status"] = False
            ctx["planner_goal_failed"] = False
            self.retract_future = self.position_client.call_async(request)
            self.retract_pose_sent = True
            node.get_logger().info(
                f"[{self.name}] Re-sending {pose_name} pose to retract arm from wall."
            )
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
        ## Safety net: stop the base, clear any slow-sweep speed cap, stop any
        ## running GPR measurement, release force mode (real) and stop sensor +
        ## alignment nodes.
        self._stop_sweep_crawl(ctx, publish_stop=True)
        self._stop_gpr_triggers(ctx, log_summary=False)
        self._restore_sweep_speed(ctx)
        self._cancel_sweep_goal(ctx)
        self._set_trajectory_bridge_hold(ctx, False)   # never leave the arm stack muted
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
