from ..state import State
from ..utils.chassis_parking import ChassisParker
from ..utils.column_control import ColumnController
from ..utils.controller_ready import TrajectoryControllerGate
from ..utils.gpr_sweep import GprSweep
from ..utils.hyperspectral_sampler import HyperspectralSampler
from ..utils.costmap_utils import (
    COSTMAP_WAIT_TIMEOUT_S,
    base_standoff_goal,
    nav_bt_xml,
    plan_wall_partitions,
    reachable_wall_segments,
    publish_wall_segment_markers,
    wall_axes,
    wall_parallel_goal,
    world_frame,
)
from ..utils.wall_partitioning import next_backoff_length, sweep_line_order
from ..utils.wall_approach import unfolded_pose_name
from arm_control.action import SweepLine
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.action import GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from rclpy.duration import Duration
import rclpy.time
from arm_control.srv import SendPosition
try:
    # The plane fit wall_parallel_controller aligns the plate with (arm_control's
    # sensors package): the FSM reads the same tilt to know when it is done.
    from sensors.wall_alignment_estimator import fit_wall_normal
except ImportError:      # arm_control not on the path: fall back to waiting it out
    fit_wall_normal = None
from ur_msgs.srv import SetForceMode
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
import subprocess, os, signal
import math
from math import atan2, sin, cos
import time

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

# How much further off the wall than the sweep standoff (sweep_scan_standoff_m,
# 0.20 m) the arm's planned approach stops, AT MOST, in metres
# (ctx["sweep_approach_retract_m"]): the approach closes on the wall to 0.30 m
# and never pulls back toward it. At 0.20 m the plate's collision mesh (GPR
# included, 0.155 m in front of arm_plate_link) ends ~6.5 cm from the wall,
# inside the 5-10 cm the collision checker's dilated 5 cm octomap puts in front
# of it, so the planned approach was refused "sometimes" and the partition
# skipped (seen on the whole-body side, 76fa525). At 0.30 m the mesh is ~16.5 cm
# clear. Force mode closes the gap on the robot (its Z reach is sized from
# this); in simulation the executor's lead-in closes it on the way to the
# partition start.
#
# A ceiling, not a target: at the partition standoff (1.15 m) the scanning pose
# itself already leaves the plate ~0.25 m off the wall, and moving it back to
# 0.30 m pulls the wrist into the mast the arm planner keeps clear ("wrist_3
# position is INSIDE ENVIRONMENT OBSTACLE: Cylinder r=0.4 m", 2026-10-20). Nor
# does anything retract TO it: the retracts after a sweep go to the sweep
# standoff (_retract_standoff).
SWEEP_APPROACH_RETRACT_M = 0.10

# Where 'arm_tool0' sits above 'arm_base_link' in the scanning pose, in metres.
# A constant of the POSE, not of the moment: the column and the base move the
# arm MOUNT, never this offset -- which is exactly why the column target is
# derived from the mount and this number, and not from wherever the tool
# happens to be when the tick fires. FK over the URDF at unfolded_fsm's joints
# gives 0.5217 m; unfolded_front_fsm's tool sits 0.1 mm higher in
# position_sender_node's pose table (0.51959 vs 0.51949), so one value serves
# both of base placement's scanning poses.
SCAN_POSE_TOOL_Z_ABOVE_ARM_BASE = 0.5217

# How far the arm alone carries the tool off that nominal height while holding
# the scanning posture, in metres. Only height OUTSIDE this window is worth
# waking a 7-second, non-backdrivable column for; inside it the executor's
# lead-in leg puts the plate on the line height itself.
ARM_Z_WINDOW_M = 0.25

# Cross-polarized GPR pass (ctx["scan_wall_cross_pol"], on by default): every
# partition is swept twice at each height, once with the plate upright
# (co-polarized) and straight back over it with the plate rolled about the wall
# normal (cross-polarized), the GPR's two modes. Both sweeps happen at the same
# base stop, one after the other, before the column moves to the next height.
#
# The roll is ONE fixed value, and the pose the arm takes for it is one fixed
# named pose ('<scanning pose>_xpol' in arm_control's position_sender_node),
# never "the current pose + 90 deg": every partition then rolls the plate the
# same way, and the cable and collision check done once on that pose covers all
# of them. The plate is always pulled back off the wall (_roll_clearance) before
# it is rolled or un-rolled.
#
# -pi/2 because it puts the GPR ABOVE the plate centre (+8 cm), and the plate is
# lowered by that much so the antenna sweeps the same line as the upright pass.
CROSS_POL_PLATE_ROLL = -math.pi / 2
CROSS_POL_POSE_SUFFIX = "_xpol"

# The named pose the arm folds into before the base moves to a different
# reachable segment (between partitions of the SAME segment it stays unfolded,
# only re-centred).
FOLDED_POSE = "folded_fsm"

# Where the GPR touches the wall, plate x/y, m (force mode's task frame is the
# same point: _build_force_mode_request).
GPR_CONTACT_XY = (-0.08, 0.0)


def gpr_height_above_plate(roll, contact_xy=GPR_CONTACT_XY):
    """How far the GPR sits above the plate centre, m, with the plate square to
    the wall and rolled ``roll`` rad about the normal from +Y-up.

    0 for the upright plate (the GPR is beside the centre, along the line), so
    only the rolled pass needs a height correction. Upright, plate +X points
    along the wall and +Y up; a roll r about +Z turns them into (cos r, sin r)
    and (-sin r, cos r), so the GPR's vertical offset is x sin r + y cos r.
    """
    cx, cy = contact_xy
    return cx * math.sin(roll) + cy * math.cos(roll)


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
        # (kept false by default): the parker flips it true via /set_parameters just
        # for the maneuver, then false again. See utils/chassis_parking.py; knobs
        # scan_wall_park_base / _park_grace_s / _park_timeout_s.
        self._parker = ChassisParker(name, "scan_wall")

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
        #   press_prepare    alignment controller, then tare the FT sensor
        #   ft_zero_wait     wait for the tare, then start the force-mode press
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
        # Latched once the GPR line, its triggers and the hyperspectral sampler
        # have been armed for the sweep in flight. Its own flag rather than
        # "is the trigger timer running": with gpr_trigger_enabled off there is
        # no timer, and keying on it re-armed the samplers every tick -- which
        # reset the hyperspectral travel counter each second, so no capture
        # could ever accumulate its spacing.
        self._sweep_samplers_armed = False
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
        self._crawl_axes = None            # ((nx,ny),(tx,ty)) when axis-split, else None
        self._crawl_tol_normal = None      # standoff tolerance in axis-split mode
        self._crawl_max_lateral = None     # achievable strafe speed, m/s
        self._transit_retries = 0          # arrival-check retries of one transit
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

        # Tare for that sensor: the broadcaster reads a non-zero Z with nothing
        # touching the plate, so it is zeroed once per segment just before the
        # press starts (see _zero_ft_sensor).
        self._ft_zero_client = None
        self._ft_zero_future = None
        self._ft_zero_start = None

        # GPR (GP8800): the HTTP API flow, the plate-travel trigger sampler and
        # the line manifest, shared with the whole-body strategy. ScanWall only
        # decides WHEN each step happens: the measurement opens before the
        # press, the line starts once the plate is pressed, the triggers arm on
        # the executor's "sweep" feedback. See utils/gpr_sweep.py.
        self._gpr = GprSweep(name, self)
        self._ee_frame = None               # cached map->EE frame name

        # A named pose sent while the External Control program is still starting
        # is rejected ("Controller is not running."); hold it until a trajectory
        # controller reports active. Re-armed after every send.
        self._controller_gate = TrajectoryControllerGate()

        # Cross-polarized pass (CROSS_POL_PLATE_ROLL): True while the sweep in
        # progress is the rolled one, back over the partition just swept upright.
        self._xpol = False
        self._xpol_ok = 0            # rolled sweeps completed on this line
        # Plate-to-wall gap measured at the scanning pose, before the upright
        # pass's approach: how far back the arm demonstrably reaches here, which
        # bounds the retract before an in-place roll (_roll_clearance).
        self._scan_pose_gap = None
        # Non-blocking waits (were time.sleep on the FSM's only thread).
        self._align_wait_start = None    # press_align_wait: when it started
        self._align_stamp = None         # last /distance_sensors frame judged
        self._align_streak = 0           # consecutive frames within tolerance
        self._settle_until = None        # sweep_settle deadline
        self._settled_sweep = None       # _advance_after_sweep args held meanwhile
        # Arm folded for a transit to another reachable segment (FOLDED_POSE);
        # unfolded again once the base has arrived.
        self._arm_folded = False
        # Reachable segment (index into current_wall_segments) of the partition
        # the base last stood at; a transit to a different one folds the arm.
        self._last_source = None

        # Hyperspectral: sampled on the same plate travel as the GPR triggers,
        # but two orders of magnitude coarser (10 cm vs 0.5 cm) because one
        # sample is a service round-trip plus an integration time, not a pulse.
        # Off unless ctx["hyperspectral_enabled"], exactly like the GPR probe,
        # and it never aborts a line: a failed point is recorded and skipped.
        # Records RAW spectra only -- reflectance and material prediction run
        # afterwards, in SensorDataProcessing.
        self._hs = HyperspectralSampler(name)

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering scanning state.")
        self.started = False
        self.finished = False
        self.goal_sent = False
        self.waiting = False
        self.more_lines = False
        self._parker.reset()
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
        self._sweep_samplers_armed = False
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
        self._ft_zero_future = None
        self._ft_zero_start = None
        self._gpr.abort(ctx)          # close anything a previous entry left open
        self._hs.stop_timer(ctx)      # clear any stale timer, hyperspectral
        self._ee_frame = None
        self._controller_gate.reset()
        self._xpol = False
        self._xpol_ok = 0
        self._scan_pose_gap = None
        self._arm_folded = False
        self._last_source = None
        ctx["error_triggered"] = False

        self.column.reset()
        self.column.configure(node, ctx)

        # Binds the session directory and the raw record on first entry, and
        # re-attaches to them on every later line and wall -- the session spans
        # the whole mission. No-op when hyperspectral sampling is disabled.
        self._hs.configure(node, ctx)

        # Same check the sweep makes before arming the triggers, but here it
        # costs nothing: better to fail at the door than after the approach.
        ok, reason = self._gpr.bridge_ready(ctx)
        if not ok:
            node.get_logger().error(f"[{self.name}] {reason}; not starting the scan.")
            return self.fail(ctx, reason)

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
        # plate_roll: the controller's roll compensation rebuilds the plate
        # orientation from scratch each cycle, so it must be told the rolled
        # pass's roll or it snaps the plate back upright in one goal.
        return self._start_arm_procs(ctx, [
            ("wall_parallel_proc",
             ["ros2", "run", "arm_control", "wall_parallel_controller", "--ros-args",
              "-p", "control_rate:=2.0",
              "-p", f"plate_roll:={self._plate_roll()}"],
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
            # No retract leg after a sweep: from a partition end, a straight
            # pull-back with the plate's orientation held curls the arm into the
            # plate, and the executor checks no collisions (seen in simulation,
            # 2026-10-20). The FSM's own retracts are planned.
            "-p", f"approach_retract_m:={float(ctx.get('sweep_executor_retract_m', 0.0))}",
            "-p", f"max_traverse_m:={float(ctx.get('sweep_max_traverse_m', 1.10))}",
            # Diagnostic only. Once force mode has the plate on the wall the
            # executor sweeps from that contact pose rather than nudging it back
            # onto the commanded line height; this is how far off that line the
            # pressed plate may sit before the executor says so in the log.
            "-p", f"press_height_tolerance_m:="
                  f"{float(ctx.get('sweep_press_height_tolerance_m', 0.05))}",
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
                    # Also under _procs, so proc_utils.stop_all stops it when the
                    # FSM is interrupted mid-scan; in its own session it would
                    # otherwise outlive the FSM (and keep the sensors' serial port).
                    ctx.setdefault("_procs", {})[proc_key] = ctx[proc_key]
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

        ``sweep_approach_retract_m`` defaults to SWEEP_APPROACH_RETRACT_M (0.10),
        for an approach that stops at 0.30 m at most: clear of the band the
        dilated octomap puts in front of the wall, where the planned approach was
        refused. The arm_approach only ever closes on the wall to it, never
        backs off to it. Force mode's Z reach is sized from this distance
        (_build_force_mode_request), so the press still gets to the wall.

        Keyed off ``sweep_scan_standoff_m`` in arm-sweep mode so raising the sweep
        standoff carries the approach out with it; the legacy base-driven path
        keeps using the press offset it was tuned against.
        """
        base = (float(ctx.get("sweep_scan_standoff_m", 0.20))
                if self._use_arm_sweep(ctx)
                else float(ctx.get("scan_wall_plate_offset", 0.20)))
        return base + self._approach_retract(ctx)

    def _retract_standoff(self, ctx):
        """Plate-to-wall gap the arm retracts to after a sweep, m: the sweep
        standoff, which the scanning pose can always reach. See
        SWEEP_APPROACH_RETRACT_M for why not the approach's 0.30 m."""
        return float(ctx.get("sweep_scan_standoff_m", 0.20))

    @staticmethod
    def _approach_retract(ctx):
        """Approach margin beyond the sweep standoff, m (SWEEP_APPROACH_RETRACT_M)."""
        return float(ctx.get("sweep_approach_retract_m", SWEEP_APPROACH_RETRACT_M))

    def _send_arm_to_clearance(self, ctx, target, contact_default=False,
                               retract_only=False, advance_only=False):
        """Move the plate so it ends up ``target`` metres off the wall.

        Returns "sent" once a goal is out, "skip" when the move is unnecessary or
        cannot be sized, or "wait" while still waiting for a first sensor frame.

        ``retract_only`` refuses to move the plate TOWARD the wall. A move whose
        whole purpose is to clear the wall should never close on it, whatever the
        arithmetic says -- and the arithmetic can say so, because the standoff
        knobs are independent and drift apart when one is retuned.
        ``advance_only`` is the mirror image, for the approach: it never backs
        the plate off toward the robot.

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
        if advance_only and advance < 0.0:
            node.get_logger().info(
                f"[{self.name}] Plate already {d:.3f} m off the wall, closer than "
                f"the {target:.2f} m approach standoff; leaving it there rather "
                f"than pulling it {-advance:.3f} m back toward the robot."
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

        # A pose sent while the External Control program is still starting is
        # rejected with "Controller is not running." -- and the program takes
        # longer to come up with the pendant speed slider turned down. Hold the
        # request until a trajectory controller reports active.
        if not self._controller_gate.ready(node):
            return "wait"
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
        # Re-arm so the next named pose checks again: a sweep in between can
        # drop the program or hold the controller.
        self._controller_gate.reset()
        return "wait"

    def _named_pose_settled(self, ctx):
        """"done" / "failed" / "wait" for a named-pose move.

        Unlike ``_arm_goal_settled`` this reports failure instead of swallowing
        it. A Z-clearance nudge that does not happen is survivable; a fold, an
        unfold or a roll that does not happen is not -- the base would move with
        the arm out, or a partition would be swept folded, or the "rolled" pass
        would be recorded with the plate upright.
        """
        node = ctx["node"]
        timeout = float(ctx.get("scan_wall_named_pose_timeout_s", 180.0))
        now = time.time()
        if self._arm_goal_start is None:
            self._arm_goal_start = now
        if ctx.get("planner_goal_failed"):
            ctx["planner_goal_failed"] = False
            node.get_logger().error(f"[{self.name}] Planner could not reach the pose.")
            return "failed"
        if ctx.get("execution_status") is True:
            ctx["execution_status"] = False
            return "done"
        if now - self._arm_goal_start >= timeout:
            node.get_logger().error(
                f"[{self.name}] Arm pose did not complete within {timeout:.0f}s.")
            return "failed"
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
        # Sized from the approach that actually ran rather than fixed: a gap
        # wider than the limit leaves force_mode short of the wall, and
        # _wall_contact_ready would burn its timeout and sweep without contact.
        # (ARM_SWEEP_PLAN §7.C kept this a deliberate decision; the 0.30 m
        # approach, SWEEP_APPROACH_RETRACT_M, is that decision.)
        approach = (self._approach_standoff(ctx) if self._use_arm_sweep(ctx)
                    else float(ctx.get("scan_wall_plate_offset", 0.20)))
        press_reach = approach + 0.05
        req.deviation_limits = [0.1, 0.1, max(0.15, press_reach), 0.1, 0.1, 0.1]
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
    # FT sensor tare (real robot only): zero the TCP wrench before pressing
    # ------------------------------------------------------------------
    def _zero_ft_sensor(self, ctx):
        """Real robot only. Tare the TCP force/torque sensor before the press.

        The sensor carries a bias: with the arm parked and nothing touching the
        plate, force_torque_sensor_broadcaster still reports ~-5 N on Z — which
        is exactly the value _wall_contact_ready reads as contact, so without a
        tare press_settle falls through immediately and the segment is swept
        with the plate still in free air. The UR driver exposes the same zeroing
        the teach pendant does; call it here, with the arm settled at the
        approach pose and not yet pressing, so the offset it captures is the
        true no-load bias.

        Only issues the call — _ft_zero_ready polls the future — because the FSM
        ticks on the same single-threaded executor that has to deliver the
        response, so blocking on it here would deadlock.
        """
        self._ft_zero_future = None
        self._ft_zero_start = time.time()
        if bool(ctx.get("sim", False)) or not bool(ctx.get("scan_wall_zero_ftsensor", True)):
            return
        node = ctx["node"]
        if self._ft_zero_client is None:
            self._ft_zero_client = node.create_client(
                Trigger, "/io_and_status_controller/zero_ftsensor")
        if not self._ft_zero_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().warn(
                f"[{self.name}] zero_ftsensor service unavailable; pressing with an "
                f"UNTARED FT sensor. Its idle bias (~-5 N on Z) may be read as wall "
                f"contact, so the sweep can start before the plate touches the wall."
            )
            return
        node.get_logger().info(f"[{self.name}] Taring the TCP force/torque sensor.")
        self._ft_zero_future = self._ft_zero_client.call_async(Trigger.Request())

    def _ft_zero_ready(self, ctx):
        """Return True once the tare has answered (or there is nothing to wait
        for), False while the call is still in flight, so the caller can hand
        control back to the tick loop. A bounded timeout proceeds anyway rather
        than stalling the segment on an unresponsive driver."""
        if self._ft_zero_future is None:
            return True
        node = ctx["node"]
        timeout_s = float(ctx.get("scan_wall_zero_ftsensor_timeout_s", 10.0))
        if not self._ft_zero_future.done():
            if time.time() - self._ft_zero_start < timeout_s:
                return False
            self._ft_zero_future = None
            node.get_logger().warn(
                f"[{self.name}] zero_ftsensor did not answer in {timeout_s:.0f}s; "
                f"pressing with a possibly untared FT sensor."
            )
            return True
        future, self._ft_zero_future = self._ft_zero_future, None
        try:
            res = future.result()
            if getattr(res, "success", True):
                node.get_logger().info(f"[{self.name}] FT sensor tared (Fz reset to 0).")
            else:
                node.get_logger().warn(
                    f"[{self.name}] zero_ftsensor refused the tare: "
                    f"{getattr(res, 'message', '')}"
                )
        except Exception as e:
            node.get_logger().warn(f"[{self.name}] zero_ftsensor call failed: {e}")
        return True

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
    # Plate travel: the frame and direction both sensors measure along
    # ------------------------------------------------------------------
    def _plate_travel_frame(self, ctx):
        """Frame the plate travel is measured against, for the GPR triggers and
        the hyperspectral sampler alike.

        Explicit ``gpr_trigger_reference_frame`` wins. Otherwise it follows the
        sweep route, because the two move the plate with different bodies: an arm
        sweep runs with the base parked, so ``arm_base`` measures the plate off
        the joint states alone and keeps odometry and localisation drift out of
        the spacing entirely; a base-driven sweep moves the arm base itself,
        where only a world frame sees the travel at all.
        """
        ref = ctx.get("gpr_trigger_reference_frame")
        if ref:
            return str(ref)
        if self._use_arm_sweep(ctx):
            return str(ctx.get("arm_base_frame", "arm_base"))
        return self._world_frame(ctx)

    @staticmethod
    def _rotate_vec(q, v):
        """Rotate vector ``v`` by quaternion ``q`` (v + 2q_v x (q_v x v + w v))."""
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        tx = 2.0 * (y * v[2] - z * v[1])
        ty = 2.0 * (z * v[0] - x * v[2])
        tz = 2.0 * (x * v[1] - y * v[0])
        return (
            v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx),
        )

    def _axis_in_frame(self, ctx, ref_frame, axis_xy):
        """Express the map-frame sweep direction ``axis_xy`` in ``ref_frame`` as a
        3-D unit vector, or None if the rotation cannot be looked up.

        Only the rotation matters — a direction has no origin — and the base is
        parked for the whole arm sweep, so this is read once at arming and stays
        valid for the sweep. Doing it per sample would put the very TF chain this
        parameter exists to avoid back into every measurement.
        """
        world = self._world_frame(ctx)
        if ref_frame == world:
            return (axis_xy[0], axis_xy[1], 0.0)
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        try:
            if not tf_buffer.can_transform(
                ref_frame, world, rclpy.time.Time(), Duration(seconds=1.0)
            ):
                return None
            tf = tf_buffer.lookup_transform(
                ref_frame, world, rclpy.time.Time(), Duration(seconds=1.0))
        except Exception:
            return None
        vx, vy, vz = self._rotate_vec(
            tf.transform.rotation, (axis_xy[0], axis_xy[1], 0.0))
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm < 1e-6:
            return None
        return (vx / norm, vy / norm, vz / norm)

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

    # ------------------------------------------------------------------
    # Hyperspectral sampling
    # ------------------------------------------------------------------
    def _start_hyperspectral(self, ctx, seg_start, seg_end):
        """Arm the hyperspectral sampler for the segment about to be swept.

        Deliberately reuses the GPR trigger sampler's answers to the two
        questions both sensors have to get right, rather than deciding them
        again: WHICH FRAME the plate travel is measured in (an arm sweep parks
        the base, so arm_base keeps localisation drift out of the spacing; a
        base sweep only moves relative to the world) and WHICH DIRECTION counts
        as forward along the wall. Two sensors on one plate disagreeing about
        either would be a subtle, permanent misalignment between the GPR traces
        and the material samples taken at the same instant.

        The plate lookup is handed over as a closure over _lookup_plate_xyz for
        the same reason: one TF fallback chain and one cached EE frame, shared.
        """
        if not self._hs.enabled(ctx):
            return
        ref, axis = self._hyperspectral_frame_and_axis(ctx, seg_start, seg_end)
        self._hs.start_line(
            ctx, seg_start, seg_end,
            pose_fn=self._plate_pose_fn(ctx),
            ref=ref,
            axis=axis,
            world=self._world_frame(ctx),
            wall_index=ctx.get("current_wall_index"),
            line_idx=ctx.get("current_line_idx", 0),
            seg_idx=self._seg_idx,
        )

    def _begin_hyperspectral_segment(self, ctx, seg_start, seg_end):
        """Open the hyperspectral segment and take sample #0 at the sweep start.

        Called the moment the plate is pressed on the wall with its orientation
        corrected -- before the sweep goal is sent -- so the first spectrum is
        taken with the plate at rest exactly at d = 0, where the GPR's first
        trace also lands. The distance sampler itself is armed later, by
        _start_hyperspectral on the executor's "sweep" feedback, and its
        counters carry on from this point.
        """
        if not self._hs.enabled(ctx):
            return
        ref, _axis = self._hyperspectral_frame_and_axis(ctx, seg_start, seg_end)
        if self._hs.begin_segment(
            ctx, self._plate_pose_fn(ctx), ref=ref, world=self._world_frame(ctx),
            wall_index=ctx.get("current_wall_index"),
            line_idx=ctx.get("current_line_idx", 0),
            seg_idx=self._seg_idx,
        ):
            self._hs.capture_at_start(ctx)

    def _plate_pose_fn(self, ctx):
        """The plate lookup handed to the sampler: one TF fallback chain and
        one cached EE frame, shared with the GPR triggers."""
        return lambda frame, timeout: self._lookup_plate_xyz(ctx, frame, timeout_s=timeout)

    def _hyperspectral_frame_and_axis(self, ctx, seg_start, seg_end):
        """``(ref, axis)`` for the segment: the GPR trigger frame and the sweep
        direction expressed in it, falling back to the world frame."""
        ref = self._plate_travel_frame(ctx)
        axis_xy = self._sweep_axis(seg_start, seg_end)
        if axis_xy is None:
            sweep_yaw = 2.0 * atan2(self._sweep_qz, self._sweep_qw)
            axis_xy = (cos(sweep_yaw), sin(sweep_yaw))
        axis = self._axis_in_frame(ctx, ref, axis_xy)
        if axis is None:
            world = self._world_frame(ctx)
            ctx["node"].get_logger().warn(
                f"[{self.name}] hyperspectral: cannot express the sweep "
                f"direction in '{ref}'; measuring in {world} instead."
            )
            ref, axis = world, (axis_xy[0], axis_xy[1], 0.0)
        return ref, axis

    # ------------------------------------------------------------------
    # Column height from the map-frame line z
    # ------------------------------------------------------------------
    def _lookup_plate_xyz(self, ctx, ref_frame="map", timeout_s=1.0):
        """Return the end-effector (sensor-plate TCP) position expressed in
        ``ref_frame`` as ``(x, y, z)``, or None.

        ``map`` gives a WORLD pose: it moves when the base slides along the wall,
        which is what the base-driven sweep has to measure. An arm-mounted
        reference (``arm_base``) instead measures the plate the way ``_tool0_pose``
        does — pure forward kinematics off the joint states, with no odometry or
        localisation in the number at all. That is the accurate choice whenever
        the base is parked, and the wrong one whenever it is not: relative to
        ``arm_base`` a base-driven sweep barely moves the plate.

        The first frame of the fallback chain that resolves is cached in
        ``self._ee_frame``, so the high-rate trigger sampler does not re-probe
        eight frames per tick; pass ``timeout_s=0.0`` there so a momentary TF gap
        cannot block the executor. A cached frame that stops resolving against a
        different reference falls back to the full chain.
        """
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        tool = str(ctx.get("arm_tool_frame", "arm_tool0"))
        chain = [tool] + [
            f for f in (
                "arm_tool0", "arm_wrist_3_link", "arm_ee_link", "arm_flange",
                "tool0", "wrist_3_link", "ee_link", "flange",
            )
            if f != tool
        ]
        frames = ([self._ee_frame] + [f for f in chain if f != self._ee_frame]
                  if self._ee_frame is not None else chain)
        timeout = Duration(seconds=float(timeout_s))
        for frame in frames:
            try:
                if tf_buffer.can_transform(ref_frame, frame, rclpy.time.Time(), timeout):
                    tf = tf_buffer.lookup_transform(
                        ref_frame, frame, rclpy.time.Time(), timeout)
                    self._ee_frame = frame
                    t = tf.transform.translation
                    return (float(t.x), float(t.y), float(t.z))
            except Exception:
                continue
        return None

    def _lookup_arm_base_z(self, ctx):
        """Map-frame height of the arm MOUNT, or None if TF cannot answer yet.

        The mount, not the tool: 'arm_base_link' does not move when the arm
        folds, so this number is the same whatever pose the arm is holding when
        the tick happens to fire. That is the whole point -- the tool's height
        is a property of the arm's configuration, and the column must not be
        sized from it.

        No timeout, deliberately: tf2's Python wait busy-sleeps, and on this
        node's single-threaded executor that blocks the very callbacks that
        deliver transforms. Read what is in the buffer and let the caller retry
        on the next tick.
        """
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        frame = str(ctx.get("arm_root_frame", "arm_base_link"))
        try:
            tf = tf_buffer.lookup_transform(self._world_frame(ctx), frame, rclpy.time.Time())
        except Exception:
            return None
        return float(tf.transform.translation.z)

    def _column_target_for_line(self, ctx, line_z):
        """Column extension for the row at map-frame ``line_z``.

        None means "inputs not in yet, ask again next tick" -- never a guess. An
        assumed column height moves a non-backdrivable 7-second axis to the
        wrong place, and an assumed mount height decides the wrong thing about
        reach.

        The column is the OUTER loop: it exists for the height the arm cannot
        cover by itself. So the height the arm can cover is subtracted first,
        and only the excess becomes column travel; the executor's lead-in then
        puts the plate on the line. Sizing the column by the gap between the
        line and wherever the tool currently is (as this used to) hands it the
        arm's own workspace as well, and makes the answer depend on the pose
        the arm happened to hold when the tick fired.
        """
        node = ctx["node"]

        column_current = ctx.get("column_current_height")
        if column_current is None:
            node.get_logger().warn(
                f"[{self.name}] Column height unknown: no 'column_joint' in /joint_states yet.",
                throttle_duration_sec=5.0,
            )
            return None
        column_current = float(column_current)

        arm_base_z = self._lookup_arm_base_z(ctx)
        if arm_base_z is None:
            node.get_logger().warn(
                f"[{self.name}] Arm mount height unknown: no "
                f"{self._world_frame(ctx)} -> arm_base_link transform yet.",
                throttle_duration_sec=5.0,
            )
            return None

        tool_offset = float(ctx.get("scan_wall_scan_pose_tool_z", SCAN_POSE_TOOL_Z_ABOVE_ARM_BASE))
        window = abs(float(ctx.get("scan_wall_arm_z_window", ARM_Z_WINDOW_M)))

        # With the cross-polarized pass on, each height is swept at TWO plate
        # heights: on the line, then lowered by the GPR's offset. Size the column
        # for the band spanning both -- its middle, with the arm's window
        # narrowed by half the band -- so the arm reaches each without the
        # column moving between the two passes.
        heights = [float(line_z)]
        if self._cross_pol_enabled(ctx) and self._use_arm_sweep(ctx):
            heights.append(float(line_z) - gpr_height_above_plate(CROSS_POL_PLATE_ROLL))
        band = max(heights) - min(heights)
        row_z = min(heights) + band / 2.0
        window = max(0.0, window - band / 2.0)

        # Where the tool WILL be for this row, at the current column height,
        # once the arm is in the scanning pose. Not where it is now.
        tool_z = arm_base_z + tool_offset
        gap = row_z - tool_z
        sized_for = (f"line z={line_z:.3f}m" if band == 0.0 else
                     f"line z={line_z:.3f}m (both passes: {min(heights):.3f}-"
                     f"{max(heights):.3f}m, centre {row_z:.3f}m)")

        if abs(gap) <= window:
            node.get_logger().info(
                f"[{self.name}] Column stays at {column_current:.3f}m: {sized_for} is "
                f"{gap:+.3f}m from the scanning pose's tool height ({tool_z:.3f}m), inside the "
                f"arm's own +/-{window:.2f}m window."
            )
            return column_current

        excess = math.copysign(abs(gap) - window, gap)
        target = column_current + excess
        clamped = max(self.column.column_min_height_m, min(target, self.column.column_max_height_m))
        node.get_logger().info(
            f"[{self.name}] Column calc: {sized_for}, arm mount={arm_base_z:.3f}m, "
            f"scanning-pose tool={tool_z:.3f}m, gap={gap:+.3f}m beyond the +/-{window:.2f}m arm "
            f"window by {excess:+.3f}m, column_now={column_current:.3f}m -> target={target:.3f}m "
            f"(clamped {clamped:.3f}m)."
        )
        if abs(clamped - target) > 1e-6:
            node.get_logger().warn(
                f"[{self.name}] Column travel is capped at [{self.column.column_min_height_m:.2f}, "
                f"{self.column.column_max_height_m:.2f}]m, so line z={line_z:.3f}m stays "
                f"{abs(target - clamped):.3f}m outside the arm's window."
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

        # --- Phase 0: handed over folded (ArmUnfolding, arm-sweep wall scans).
        # Keep it folded: the first partition's transit unfolds it once the base
        # stands at the scan pose -- or right away, through the same strict
        # unfold, when no transit is needed. ---
        if (not self.pose_reached and not self.pose_sent
                and self._use_arm_sweep(ctx) and ctx.get("arm_folded") is True):
            self.current_line_z = self._resolve_current_line_z(ctx)
            self._arm_folded = True
            self.pose_reached = True
            node.get_logger().info(
                f"[{self.name}] Pre-approach for line z={self.current_line_z:.3f}m: "
                f"the arm comes folded and stays folded until the base is at "
                f"partition 1's scan pose."
            )

        # --- Phase 1: send the arm to the unfolded_fsm pose and wait for it. ---
        if not self.pose_reached:
            # Step 1a: send the named pose.
            if not self.pose_sent:
                if not self._controller_gate.ready(node):
                    return
                if not self.position_client.service_is_ready():
                    node.get_logger().warn(f"[{self.name}] Waiting for /send_position service...")
                    return
                self.current_line_z = self._resolve_current_line_z(ctx)
                pose_name = self._scan_pose_name(ctx)
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
                self._controller_gate.reset()
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
                self._note_arm_folded(ctx, False)
                node.get_logger().info(f"[{self.name}] Arm at the scanning pose.")
            else:
                if not self.preapproach_verbose:
                    node.get_logger().info(f"[{self.name}] Waiting for arm to reach pre-approach pose...")
                    self.preapproach_verbose = True
                return

        # --- Phase 2: arm motion finished — now move the column. ---
        if not self.column_commanded:
            target_h = self._column_target_for_line(ctx, self.current_line_z)
            if target_h is None:
                return                    # inputs not in yet; ask again next tick
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
        node.get_logger().info(
            f"[{self.name}] Pre-approach complete. "
            + ("Arm folded; it unfolds once the base is at partition 1's scan pose."
               if self._arm_folded else "Arm in scanning position.")
        )

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def run(self, ctx):
        node = ctx["node"]

        # Fetch GDS/GRF once per mission. Non-blocking and re-entrant: it walks
        # GET_GDS -> GET_GRF -> GET_MTI over the ticks of the pre-approach, so
        # the calibration is cached well before the first sweep -- and an
        # uncalibrated or unreachable camera is discovered while the arm is
        # still moving into place, not after a wall has been swept against a
        # calibration that does not exist.
        if self._hs.enabled(ctx):
            self._hs.calibration_ready(ctx)

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

    def _world_frame(self, ctx):
        """Fixed frame of the wall geometry and sweep goals (``scan_world_frame``)."""
        return world_frame(ctx)

    # ------------------------------------------------------------------
    # Cross-polarized pass (CROSS_POL_PLATE_ROLL)
    # ------------------------------------------------------------------
    @staticmethod
    def _cross_pol_enabled(ctx):
        """Sweep every partition a second time with the plate rolled. On by default."""
        return bool(ctx.get("scan_wall_cross_pol", True))

    @staticmethod
    def _roll_in_place_enabled(ctx):
        """Roll the plate where it is after the upright sweep (default), rather
        than through the named ``_xpol`` pose."""
        return bool(ctx.get("scan_wall_roll_in_place", True))

    def _plate_roll(self):
        """Roll about the wall normal the plate sweeps at on this pass, rad."""
        return CROSS_POL_PLATE_ROLL if self._xpol else 0.0

    def _plate_rise(self):
        """How far to raise the plate centre off the line on this pass, m, so the
        GPR (not the centre) sweeps the line height. 0 upright; -0.08 rolled."""
        return -gpr_height_above_plate(self._plate_roll())

    def _polarization(self, ctx):
        """The GPR line's polarization label, or None when lines are swept once."""
        if not (self._cross_pol_enabled(ctx) and self._use_arm_sweep(ctx)):
            return None
        return "xpol" if self._xpol else "copol"

    def _pass_label(self):
        return " (cross-polarized)" if self._xpol else ""

    def _scan_pose_name(self, ctx):
        """The named pose the arm scans from on this pass: the approach's
        unfolded pose, or its rolled twin for the cross-polarized pass
        (``scan_wall_xpol_pose`` names it explicitly, e.g. alongside a
        ``scan_wall_unfolded_pose`` override that has no ``_xpol`` twin)."""
        name = self._unfolded_pose_name(ctx)
        if not self._xpol:
            return name
        return str(ctx.get("scan_wall_xpol_pose") or name + CROSS_POL_POSE_SUFFIX)

    def _roll_clearance(self, ctx):
        """Plate-to-wall gap to retract to before rolling or un-rolling, m
        (default: the sweep standoff)."""
        return float(ctx.get("scan_wall_roll_clearance_m", self._retract_standoff(ctx)))

    def _roll_in_place_clearance(self, ctx):
        """Gap for the in-place roll, m: the approach standoff (0.30 m, clear of
        the band the dilated octomap puts in front of the wall, where a goal at
        0.20 m was refused "sometimes"), but never further back than the gap the
        scanning pose itself left at this partition -- behind that the wrist
        heads for the mast keep-out (at the robot's 1.15 m standoff the
        scanning pose leaves only ~0.25 m). Never less than _roll_clearance."""
        override = ctx.get("scan_wall_roll_in_place_clearance_m")
        if override is not None:
            return float(override)
        target = self._approach_standoff(ctx)
        if self._scan_pose_gap is not None:
            target = min(target, self._scan_pose_gap)
        return max(target, self._roll_clearance(ctx))

    def _send_arm_roll_goal(self, ctx, roll):
        """Roll the plate ``roll`` rad about its own normal, in place: a
        /arm/goal_pose at the tool's current position with its orientation
        rotated about tool0 Z. tool0 lies on the wrist_3 axis, so the planner's
        nearest IK solution is the same arm configuration with wrist_3 turned --
        nothing else moves. Planned and collision-checked like the Z moves, and
        reported on the same execution_status / planner_goal_failed pair.
        Returns True once published."""
        node = ctx["node"]
        pose = self._tool0_pose(ctx)
        if pose is None:
            node.get_logger().warn(f"[{self.name}] No arm TF available; cannot roll in place.")
            return False
        p, q = pose
        half = roll / 2.0
        rz_w, rz_z = math.cos(half), math.sin(half)
        # q * Rz(roll): rotate about the plate's OWN Z (body frame).
        rolled = Quaternion(
            x=q.x * rz_w + q.y * rz_z,
            y=q.y * rz_w - q.x * rz_z,
            z=q.z * rz_w + q.w * rz_z,
            w=q.w * rz_w - q.z * rz_z,
        )
        if self._arm_goal_pub is None:
            topic = str(ctx.get("arm_goal_pose_topic", "/arm/goal_pose"))
            self._arm_goal_pub = node.create_publisher(PoseStamped, topic, 10)
        msg = PoseStamped()
        msg.header.frame_id = str(ctx.get("arm_base_frame", "arm_base"))
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x, p.y, p.z
        msg.pose.orientation = rolled
        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False
        self._arm_goal_pub.publish(msg)
        node.get_logger().info(
            f"[{self.name}] Rolling the plate {math.degrees(roll):+.0f} deg in place at "
            f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) in {msg.header.frame_id}."
        )
        return True

    # ------------------------------------------------------------------
    # Swept memory (survives re-entry into this state)
    # ------------------------------------------------------------------
    # on_enter clears _segments and _seg_idx, so without this a state retry would
    # re-sweep ground already covered. The record lives in ctx for exactly that
    # reason. Keyed by (wall, line height): a different height is a genuinely
    # different scan line. The cross-polarized pass is keyed apart too: same
    # ground, but a sweep that has not necessarily happened yet.
    def _swept_key(self, ctx, z=None, xpol=None):
        if z is None:
            z = self.current_line_z
            if z is None:
                z = self._resolve_current_line_z(ctx)
        xpol = self._xpol if xpol is None else xpol
        key = (int(ctx.get("current_wall_index") or 0), round(float(z), 3))
        return key + ("xpol",) if xpol else key

    def _record_swept(self, ctx, seg_start, seg_end):
        store = ctx.setdefault("scan_wall_swept_segments", {})
        store.setdefault(self._swept_key(ctx), []).append(
            ((float(seg_start[0]), float(seg_start[1])),
             (float(seg_end[0]), float(seg_end[1]))))

    def _already_swept(self, ctx, seg_start, seg_end, z=None, xpol=None):
        """Has this stretch of wall already been swept at this height and pass?

        Matched with a tolerance and unordered: a re-entry re-plans against a
        costmap that has moved on, and may pick the opposite direction, which
        reverses the partition without changing the ground it covers.
        """
        if not ctx.get("scan_wall_resume", True):
            return False
        tol = float(ctx.get("scan_wall_resume_tolerance", 0.25))
        s = (float(seg_start[0]), float(seg_start[1]))
        e = (float(seg_end[0]), float(seg_end[1]))
        for done_s, done_e in ctx.get("scan_wall_swept_segments", {}).get(
                self._swept_key(ctx, z=z, xpol=xpol), []):
            for a, b in ((done_s, done_e), (done_e, done_s)):
                if (math.hypot(s[0] - a[0], s[1] - a[1]) <= tol
                        and math.hypot(e[0] - b[0], e[1] - b[1]) <= tol):
                    return True
        return False

    def _partition_fully_swept(self, ctx, seg_start, seg_end):
        """Every height (and both passes) of this partition already swept?
        Then there is no reason to drive the base to it at all."""
        if not ctx.get("scan_wall_resume", True):
            return False
        if self._nest_lines(ctx):
            heights = self._line_heights(ctx)
        else:
            z = self.current_line_z
            heights = [z if z is not None else self._resolve_current_line_z(ctx)]
        passes = (False, True) if self._polarization(ctx) else (False,)
        return all(self._already_swept(ctx, seg_start, seg_end, z=z, xpol=p)
                   for z in heights for p in passes)

    # ------------------------------------------------------------------
    # Folding between reachable segments
    # ------------------------------------------------------------------
    def _partition_source(self, ctx):
        """Index of the reachable segment the current partition was cut from."""
        sources = ctx.get("current_wall_partition_sources") or []
        return sources[self._seg_idx] if self._seg_idx < len(sources) else None

    def _fold_before_transit(self, ctx):
        """Fold the arm for the coming transit?

        Between partitions of the SAME reachable segment the base only strafes a
        partition length along clear wall, so the arm stays unfolded and is just
        re-centred. Moving to a DIFFERENT segment crosses whatever split the line
        -- an obstacle, a gap, a corner -- or comes from NavigateToTarget's
        approach pose, so the arm folds first and unfolds once the base has
        arrived. ``scan_wall_fold_between_segments`` False keeps the re-centre
        everywhere.
        """
        if not bool(ctx.get("scan_wall_fold_between_segments", True)):
            return False
        source = self._partition_source(ctx)
        return self._last_source is None or source is None or source != self._last_source

    def _partition_transit_plan(self, ctx):
        """How the base reaches the current partition's scan pose (arm sweep).

        Returns ``(kind, goal_xy, goal_yaw, yaw_err)`` with ``kind`` one of
        "parked" (scan_wall_assume_parked: never drive), "no_pose" (partition
        has no scan pose), "here" (already there) or "go".
        """
        if self._assume_parked(ctx):
            return "parked", None, None, None
        pose = (self._scan_poses or [None] * len(self._segments))[self._seg_idx]
        if pose is None:
            return "no_pose", None, None, None
        goal_xy, goal_yaw = (pose[0], pose[1]), pose[2]
        # The same test the base has to pass after a transit
        # (_transit_arrival_ok): standoff, along-wall position and heading,
        # each against its own tolerance. One 0.4 m radius used to accept
        # NavigateToTarget's pose 0.15 m too far from the wall (simulation,
        # 2026-10-20), and partition 1 was swept from there -- while the
        # standoff is exactly what sizes the arm's reach. Compared in the map
        # frame; ctx["base_position"] is odom and would be off by map->odom.
        err = self._scan_pose_error(ctx, pose)
        if err is not None:
            if err["ok"]:
                return "here", goal_xy, goal_yaw, err["yaw"]
            return "go", goal_xy, goal_yaw, None
        # No wall normal to split the error along: the plain radius test.
        here = self._base_xy_yaw_map(ctx)
        if here is not None:
            yaw_err = abs(atan2(sin(here[2] - goal_yaw), cos(here[2] - goal_yaw)))
            yaw_tol = float(
                ctx.get("partition_scan_yaw_tol", ctx.get("nav_yaw_tolerance", 0.25))
            )
            if (math.hypot(here[0] - goal_xy[0], here[1] - goal_xy[1])
                    <= float(ctx.get("segment_transit_skip_tol", 0.4))
                    and yaw_err <= yaw_tol):
                return "here", goal_xy, goal_yaw, yaw_err
        return "go", goal_xy, goal_yaw, None

    def _scan_pose_error(self, ctx, target):
        """How far the base is from scan pose ``target`` (x, y, yaw), split
        into the wall frame: ``{"normal", "along", "yaw", "ok", "why"}``, or
        None when the base pose or the wall normal is unknown.

        The standoff (wall-normal) sets the arm's reach and is held to
        partition_standoff_tol; the along-wall position only has to be close
        enough for the executor's lead-in and gets the loose tolerance. The
        heading is checked too -- a parked arm sweep is only symmetric about
        the base centreline if the base is square to the wall. The yaw
        tolerance defaults to NavigateToTarget's own convergence tolerance: no
        test can demand a heading more accurate than navigation delivers.
        """
        pose = self._base_xy_yaw_map(ctx)
        axes = wall_axes(ctx)
        if pose is None or target is None or axes is None:
            return None
        (nx, ny), (tx, ty) = axes
        ex, ey = target[0] - pose[0], target[1] - pose[1]
        e_n, e_t = ex * nx + ey * ny, ex * tx + ey * ty
        yaw_err = abs(atan2(sin(pose[2] - target[2]), cos(pose[2] - target[2])))
        tol_n = float(ctx.get("partition_standoff_tol", self.PARTITION_STANDOFF_TOL_M))
        tol_t = float(ctx.get("partition_transit_arrive_tol",
                              self.PARTITION_ALONG_WALL_TOL_M))
        yaw_tol = float(
            ctx.get("partition_scan_yaw_tol", ctx.get("nav_yaw_tolerance", 0.25))
        )
        why = ", ".join(
            part for part in (
                f"standoff off by {e_n:+.3f} m (> {tol_n:.2f})" if abs(e_n) > tol_n else "",
                f"along-wall off by {e_t:+.3f} m (> {tol_t:.2f})" if abs(e_t) > tol_t else "",
                f"heading off by {yaw_err:.3f} rad (> {yaw_tol:.2f})" if yaw_err > yaw_tol else "",
            ) if part
        )
        return {"normal": e_n, "along": e_t, "yaw": yaw_err, "ok": not why, "why": why}

    def _plate_aligned(self, ctx):
        """True once the plate is square to the wall, for press_align_wait.

        Square means the plane fitted to the six ranges -- the same fit
        wall_parallel_controller corrects with -- is within
        ``scan_wall_align_tol_deg`` (its 1 deg deadband) on
        ``scan_wall_align_frames`` (3) consecutive fresh frames, after at least
        ``scan_wall_align_min_s`` (2 s: the controller's start-up). After
        ``scan_wall_align_max_s`` (10 s, the old fixed sleep) it goes on
        regardless, as before; without the fit (arm_control not importable) it
        waits that long.
        """
        node = ctx["node"]
        elapsed = time.time() - self._align_wait_start
        min_s = float(ctx.get("scan_wall_align_min_s", 2.0))
        max_s = float(ctx.get("scan_wall_align_max_s", 10.0))
        if elapsed >= max_s:
            node.get_logger().warn(
                f"[{self.name}] Plate not confirmed square to the wall after "
                f"{max_s:.0f}s; pressing anyway, as before."
            )
            return True
        if fit_wall_normal is None:
            return False
        stamp = ctx.get("plate_distances_stamp")
        readings = ctx.get("plate_distances")
        if stamp is None or stamp == self._align_stamp or not readings or len(readings) != 6:
            return False                       # no new frame since the last look
        if time.time() - float(stamp) > PLATE_DISTANCE_MAX_AGE_S:
            return False
        self._align_stamp = stamp
        normal, _distance, _n = fit_wall_normal(readings)
        if normal is None:
            self._align_streak = 0
            return False
        tilt = math.degrees(math.acos(min(1.0, abs(float(normal[2])))))
        if tilt <= float(ctx.get("scan_wall_align_tol_deg", 1.0)):
            self._align_streak += 1
        else:
            self._align_streak = 0
        if elapsed >= min_s and self._align_streak >= int(ctx.get("scan_wall_align_frames", 3)):
            node.get_logger().info(
                f"[{self.name}] Plate square to the wall ({tilt:.2f} deg) after "
                f"{elapsed:.1f}s; pressing."
            )
            return True
        return False

    def _note_arm_folded(self, ctx, folded):
        """Record whether the arm is folded, here and in ctx (which ArmFolding
        and ArmUnfolding read and write too)."""
        self._arm_folded = bool(folded)
        ctx["arm_folded"] = bool(folded)

    def _at_partition(self, ctx):
        """The base stands at the current partition: unfold if the transit
        folded the arm, else raise the column for the sweep."""
        self._last_source = self._partition_source(ctx)
        # The column was lowered for a transit (or never moved for a skipped
        # one); line_column re-sizes it for this partition's height either way.
        self.column_commanded = False
        self._seg_phase = "transit_unfold" if self._arm_folded else "line_column"

    def _skip_partition(self, ctx, why):
        ctx["node"].get_logger().error(f"[{self.name}] {why}; skipping this partition.")
        self._line_idx = 0
        ctx["current_line_idx"] = 0
        self._seg_idx += 1
        self._seg_phase = "transit_clear"

    def _assume_parked(self, ctx):
        """True when the base is taken to be AT its scan pose already, by fiat.

        The bench-test knob behind ``fsm_node --wall-source in-front``: the
        operator parked the robot facing the wall by hand, and the wall data
        was synthesised from that pose, so there is nothing to drive to and no
        costmap to consult. Two things change:

        * ``_plan_line`` partitions the raw scan line instead of asking the
          costmap which stretches of it are reachable -- the base is where it
          is, and reachability is the operator's assertion.
        * the ``transit`` phase is skipped for every partition, so the base
          never moves during the state. Only sensible with a scan line short
          enough for ONE partition; further partitions would be swept from the
          same spot and rejected by the executor as out of reach.

        Arm-sweep mode only: the base-driven sweep IS base motion.
        """
        return self._use_arm_sweep(ctx) and bool(ctx.get("scan_wall_assume_parked", False))

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

        if arm_sweep and self._assume_parked(ctx):
            # No costmap stage: the whole line is the one reachable segment,
            # because the base is standing in front of it by construction.
            # Uncached (as every caller-supplied split is), which is fine --
            # nesting means one plan per wall anyway.
            whole = [(tuple(self._sweep_from), tuple(self._sweep_to))]
            plan = plan_wall_partitions(
                ctx, self.name, self._sweep_from, self._sweep_to, segments=whole
            )
            segments, poses = ([], None) if plan is None else plan
        elif arm_sweep:
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
            near_end, far_end = sweep_line_order(
                wall_data, prev_target_point, ctx.get("target_scan_start_end")
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
        # The cross-polarized pass runs straight back over the partition the
        # upright pass just swept: it starts where that one ended.
        if self._xpol:
            seg_start, seg_end = seg_end, seg_start
        seg_no, seg_total = self._seg_idx + 1, len(self._segments)
        pass_label = self._pass_label()

        if self._seg_phase == "transit_clear":
            # First phase of every segment's cycle, so this is where the transit
            # retry budget is refreshed -- a partition abandoned for some other
            # reason must not spend the next one's re-tries.
            self._transit_retries = 0
            self._xpol = False
            if self._use_arm_sweep(ctx) and self._partition_fully_swept(ctx, seg_start, seg_end):
                node.get_logger().info(
                    f"[{self.name}] Partition {seg_no}/{seg_total} already swept at every "
                    f"height on an earlier entry; skipping it."
                )
                self._seg_idx += 1
                return
            if self._arm_folded:
                # Folded for a transit that then failed: the plate is nowhere
                # near the wall and its ranges read nothing useful.
                self._seg_phase = "transit_recenter"
                return
            # Pull the plate back before ANY base motion. The arm has been stretched
            # out since the pre-approach, and the transit slides the whole robot
            # sideways along the wall — at the gap the previous sweep left (roughly
            # zero, the plate was pressed against the wall) that drags the plate
            # along the surface. Retract to scan_wall_transit_plate_offset first.
            self.set_activity(
                ctx,
                f"Retracting the arm clear of the wall before moving to segment "
                f"{seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # In arm-sweep mode back to the sweep standoff: the plate sits there
            # after a no-press sweep (simulation), so this is then a no-op, and
            # after a press it clears the wall by the gap the sweep was planned
            # at. Further back would pull the wrist into the mast keep-out.
            target = (self._retract_standoff(ctx) if self._use_arm_sweep(ctx)
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
            if self._arm_folded:
                # Folded already (handed over folded, or a transit that failed):
                # no re-centre, which would unfold it through a wait that
                # swallows failures. The base moves -- or not -- with the arm in,
                # and the arrival unfolds it through the strict check.
                self._seg_phase = "transit"
                return
            kind = self._partition_transit_plan(ctx)[0]
            if kind == "go" and self._fold_before_transit(ctx):
                # A different reachable segment: fold rather than re-centre.
                self._seg_phase = "transit_fold"
                return
            self.set_activity(
                ctx,
                f"Centring the arm before moving to segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_named_pose(ctx, self._scan_pose_name(ctx))
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

        if self._seg_phase == "transit_fold":
            self.set_activity(
                ctx,
                f"Folding the arm before moving to segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_named_pose(ctx, FOLDED_POSE)
            if outcome == "wait":
                return
            if outcome == "skip":
                return self._skip_partition(
                    ctx, f"Could not fold the arm before the transit to partition {seg_no}")
            self._arm_goal_start = None
            self._seg_phase = "transit_fold_wait"
            return

        if self._seg_phase == "transit_fold_wait":
            status = self._named_pose_settled(ctx)
            if status == "wait":
                return
            self._arm_goal_start = None
            if status == "failed":
                # Never move the base with the arm somewhere between out and
                # folded: that is the situation folding exists to prevent.
                return self._skip_partition(
                    ctx, f"The fold before the transit to partition {seg_no} did not complete")
            self._note_arm_folded(ctx, True)
            node.get_logger().info(
                f"[{self.name}] Arm folded: "
                + (f"the base moves from NavigateToTarget's pose to partition {seg_no}"
                   if self._last_source is None else
                   f"partition {seg_no} is on another reachable segment")
                + ", with the arm in."
            )
            self._seg_phase = "transit"
            return

        if self._seg_phase == "transit_unfold":
            self.set_activity(
                ctx,
                f"Unfolding the arm at segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_named_pose(ctx, self._scan_pose_name(ctx))
            if outcome == "wait":
                return
            if outcome == "skip":
                return self._skip_partition(
                    ctx, f"Could not unfold the arm at partition {seg_no}")
            self._arm_goal_start = None
            self._seg_phase = "transit_unfold_wait"
            return

        if self._seg_phase == "transit_unfold_wait":
            status = self._named_pose_settled(ctx)
            if status == "wait":
                return
            self._arm_goal_start = None
            if status == "failed":
                return self._skip_partition(
                    ctx, f"The unfold at partition {seg_no} did not complete; a folded "
                         f"arm cannot reach the wall")
            self._note_arm_folded(ctx, False)
            self.column_commanded = False
            self._seg_phase = "line_column"
            return

        if self._seg_phase == "transit":
            self.set_activity(
                ctx,
                f"Repositioning base to wall segment {seg_no}/{seg_total}{pass_label}",
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
                kind, goal_xy, goal_yaw, yaw_err = self._partition_transit_plan(ctx)
                if kind == "parked":
                    # The base is at its scan pose by fiat (bench run on a
                    # hand-parked robot); never drive it.
                    node.get_logger().info(
                        f"[{self.name}] scan_wall_assume_parked: treating the base's "
                        f"current pose as partition {seg_no}'s scan pose; no transit."
                    )
                    return self._at_partition(ctx)
                if kind == "no_pose":
                    node.get_logger().warn(
                        f"[{self.name}] Partition {seg_no} has no scan pose; skipping."
                    )
                    self._seg_idx += 1
                    self._seg_phase = "transit_clear"
                    return
                if kind == "here":
                    node.get_logger().info(
                        f"[{self.name}] Base already at partition {seg_no} scan pose "
                        f"(yaw err {yaw_err:.3f} rad); skipping transit."
                    )
                    return self._at_partition(ctx)
            else:
                goal_yaw = None
                goal_xy = base_standoff_goal(ctx, self.name, seg_start)
                goal_xy = wall_parallel_goal(
                    ctx, self.name, goal_xy, (pos.x, pos.y) if pos is not None else None
                )

            if (not arm_sweep and pos is not None
                    and math.hypot(pos.x - goal_xy[0], pos.y - goal_xy[1]) <= skip_tol):
                node.get_logger().info(
                    f"[{self.name}] Base already at segment {self._seg_idx + 1} start; "
                    f"skipping transit."
                )
                self._parker.reset()
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
                f"Retracting the column before moving to segment {seg_no}/{seg_total}{pass_label}",
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
                    # Split by axis: the along-wall position may sit loose (the
                    # arm's lead-in re-centres on the partition and precision here
                    # costs real time at the 0.033 m/s strafe limit), but the
                    # standoff decides how far the arm has to reach and is held to
                    # PARTITION_STANDOFF_TOL_M. One isotropic tolerance let the
                    # base satisfy the whole test on the wall-normal axis alone,
                    # which is how it kept parking at the wrong distance.
                    tol=float(ctx.get("partition_transit_arrive_tol",
                                      self.PARTITION_ALONG_WALL_TOL_M)),
                    tol_normal=float(ctx.get("partition_standoff_tol",
                                             self.PARTITION_STANDOFF_TOL_M)),
                    axis_split=True,
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
                # The parker exists only to square the chassis after a transit
                # that ignored heading. Skipping it also avoids rotating the base
                # away from the pose the partition geometry just established.
                if self._use_arm_sweep(ctx):
                    # ...but "the goal WAS the scan pose" only helps if the base
                    # actually got there, and until this check nothing verified
                    # that: the crawl reported success on its own position test
                    # and this branch took it at face value. NavigateToTarget has
                    # always re-read the pose before handing on; a partition
                    # transit ends at exactly the same kind of scan pose and
                    # deserves the same gate. A wrong standoff here is not
                    # cosmetic — it is what the arm has to span on every press.
                    if not self._transit_arrival_ok(ctx, seg_no):
                        return
                    # The column was lowered for the transit, so it always has to
                    # be raised to this partition's scan height before sweeping --
                    # in both modes, not only when nesting. A folded arm unfolds
                    # first.
                    return self._at_partition(ctx)
                self._parker.reset()
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
            # rotates. Driven by the parker's enable->request->settle->disable sequence.
            self.set_activity(
                ctx,
                f"Parking chassis to square it against wall segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            if self._parker.step(ctx):
                self._seg_phase = "sweep_setup"
            return

        if self._seg_phase == "line_change":
            # Between two heights at ONE base stop. The column must not travel
            # with the arm stretched out to the end of the partition (simulation,
            # 2026-10-23: raised 0.31 m with the plate 0.20 m off the wall), so:
            # retract off the wall, back into the scanning pose -- which also
            # turns a rolled plate upright again -- and only then the column.
            heights = self._line_heights(ctx)
            self.current_line_z = heights[self._line_idx]
            self.set_activity(
                ctx,
                f"Retracting the arm before height {self._line_idx + 1}/{len(heights)} "
                f"at wall segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            # Defaults to the sweep standoff, like every retract after a sweep.
            # This always follows a sweep, so a missing reading means the plate
            # is still pressed against the wall (its ranges below their valid
            # floor), exactly as in transit_clear -- not "unknown", which waited
            # 15 s and then moved on without retracting at all.
            outcome = self._send_arm_to_clearance(
                ctx,
                float(ctx.get("scan_wall_line_change_plate_offset",
                              self._retract_standoff(ctx))),
                contact_default=True,
                retract_only=True,
            )
            if outcome == "wait":
                return
            self._arm_goal_start = None
            self._seg_phase = "line_change_wait" if outcome == "sent" else "line_change_pose"
            return

        if self._seg_phase == "line_change_wait":
            if not self._arm_goal_settled(ctx):
                return
            self._arm_goal_start = None
            self._seg_phase = "line_change_pose"
            return

        if self._seg_phase == "line_change_pose":
            self.set_activity(
                ctx,
                f"Re-centring the arm before the column moves at segment {seg_no}/{seg_total}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_named_pose(ctx, self._scan_pose_name(ctx))
            if outcome == "wait":
                return
            if outcome == "skip":
                return self._skip_partition(
                    ctx, f"Could not re-centre the arm for the next height at partition {seg_no}")
            self._arm_goal_start = None
            self._seg_phase = "line_change_pose_wait"
            return

        if self._seg_phase == "line_change_pose_wait":
            status = self._named_pose_settled(ctx)
            if status == "wait":
                return
            self._arm_goal_start = None
            if status == "failed":
                # Never move the column with the arm somewhere out there.
                return self._skip_partition(
                    ctx, f"The arm did not reach the scanning pose for the next height "
                         f"at partition {seg_no}")
            self.column_commanded = False
            self._seg_phase = "line_column"
            return

        if self._seg_phase == "line_column":
            if self._nest_lines(ctx):
                self.current_line_z = self._line_heights(ctx)[self._line_idx]
            elif self.current_line_z is None:
                self.current_line_z = self._resolve_current_line_z(ctx)
            # The only base-stop motion between heights. Every route here leaves
            # the arm in the scanning pose first (line_change, a transit's
            # re-centre or unfold), never stretched out along the partition.
            if not self.column_commanded:
                target_h = self._column_target_for_line(ctx, self.current_line_z)
                if target_h is None:
                    return                # inputs not in yet; ask again next tick
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
                f"Preparing the sweep of wall segment {seg_no}/{seg_total}{pass_label}",
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
            if self._use_arm_sweep(ctx) and self._already_swept(ctx, seg_start, seg_end):
                node.get_logger().info(
                    f"[{self.name}] Partition {seg_no}{pass_label} at z="
                    f"{float(self.current_line_z or 0.0):.3f}m was already swept on an "
                    f"earlier entry; not sweeping it again."
                )
                return self._advance_after_sweep(
                    ctx, True, "", seg_start, seg_end, True, skipped=True)
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
                f"Extending the arm toward wall segment {seg_no}/{seg_total}{pass_label}",
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
            #
            # Toward the wall only, in arm-sweep mode: when the scanning pose
            # already leaves the plate closer than the approach standoff, it
            # stays there -- moving it back would pull the wrist into the mast.
            target = (self._approach_standoff(ctx) if self._use_arm_sweep(ctx)
                      else float(ctx.get("scan_wall_plate_offset", 0.20)))
            if not self._xpol:
                # The arm is at the scanning pose: remember how far back it
                # reaches here, for the in-place roll's retract.
                gap = self._plate_wall_distance(ctx)
                if gap is not None:
                    self._scan_pose_gap = gap
            outcome = self._send_arm_to_clearance(
                ctx, target, advance_only=self._use_arm_sweep(ctx))
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
                f"Moving the arm to the start of wall segment {seg_no}/{seg_total}{pass_label}",
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
            self._align_wait_start = time.time()
            self._align_stamp = None
            self._align_streak = 0
            self._seg_phase = "press_align_wait"
            return

        if self._seg_phase == "press_align_wait":
            # Let the alignment controller square the plate to the wall before
            # the tare and the press. Was a blind time.sleep(10) that froze the
            # whole node (timers, sensor callbacks, telemetry, restarts); now a
            # tick-driven wait that ends as soon as the plate IS square.
            if not self._plate_aligned(ctx):
                return
            # Tare the FT sensor now: the arm is settled at the approach pose and
            # nothing is pressing yet, so this is the one moment its reading is a
            # pure no-load bias. Must happen BEFORE force mode, since the contact
            # gate compares the raw Z force against the commanded press.
            self._zero_ft_sensor(ctx)
            self._seg_phase = "ft_zero_wait"
            return

        if self._seg_phase == "ft_zero_wait":
            if not self._ft_zero_ready(ctx):
                return
            # Drop the pre-tare reading so the contact gate cannot pass on a
            # wrench measured against the old bias; it waits for a fresh frame.
            ctx.pop("ft_wrench", None)

            # GPR: connect and open this segment's measurement NOW, while the
            # plate is still off the wall. The line itself starts only once the
            # plate is pressed and about to move, so nothing before that is
            # recorded as scan data -- and a probe/app problem surfaces here,
            # before the press, rather than with the arm already on the wall.
            err = self._gpr.open_measurement(
                ctx, ctx.get("current_wall_index"), int(ctx.get("current_line_idx", 0)),
                self._seg_idx, polarization=self._polarization(ctx),
                plate_roll_deg=math.degrees(self._plate_roll()),
            )
            if err:
                node.get_logger().error(f"[{self.name}] {err}; aborting scan.")
                self._gpr.abort(ctx)
                return self.fail(ctx, err)

            # Real robot: press the GPR against the wall for this segment sweep
            # (sim has no force_mode controller -> no-op).
            self._start_force_mode(ctx)
            if ctx.get("error_triggered"):
                return

            # Do NOT start the GPR line or move the base yet: force_mode is still
            # driving the plate toward the wall. Wait in press_settle until the TCP
            # FT sensor reports contact, so the line + sweep begin only once the
            # sensor plate is actually pressed against the wall.
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

            # The plate is on the wall, orientation corrected, nothing moving
            # yet: this is d = 0 of the sweep. Take the first hyperspectral
            # sample here, at rest, in BOTH sweep routes -- the distance
            # sampler armed later carries on from it.
            if not self._xpol:   # same spot, same camera: the rolled pass adds nothing
                self._begin_hyperspectral_segment(ctx, seg_start, seg_end)

            # Wheel is pressed against the wall. GPR: start the line now (the
            # measurement was opened before the press, in ft_zero_wait), and only
            # once the app has confirmed it does anything move laterally.
            #
            # In arm-sweep mode too: the executor preempts and settles the arm
            # for a second or two before its scan leg, and the plate stands still
            # for that -- the distance triggers clock no traces while it does.
            # Starting the line on the executor's "sweep" feedback instead (the
            # old way) left the first tick of every sweep unrecorded and, worse,
            # let the arm move before the line was known to be running.
            err = self._gpr.start_line(
                ctx, seg_start, seg_end, self._world_frame(ctx),
                sweep="arm" if self._use_arm_sweep(ctx) else "base",
            )
            if err:
                node.get_logger().error(f"[{self.name}] {err}; aborting scan.")
                self._gpr.abort(ctx)
                return self.fail(ctx, err)

            self.set_activity(
                ctx,
                f"Sweeping wall segment {seg_no}/{seg_total}{pass_label}",
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
            self._arm_gpr_triggers(ctx, seg_start, seg_end)
            self._start_hyperspectral(ctx, seg_start, seg_end)
            self._seg_phase = "sweep_wait"
            return

        if self._seg_phase == "sweep_wait":
            self.set_activity(
                ctx,
                f"Sweeping wall segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            arm_sweep = self._use_arm_sweep(ctx)
            reason = ""      # only set on the arm-sweep path; read by the backoff
            if arm_sweep:
                # The plate is now crossing the wall (the GPR line itself has
                # been running since press_settle): this is the moment to start
                # clocking it. Arming any earlier would trigger through the
                # executor's settle, recording plate travel that is not scan
                # data; this feedback is also why none of the three no-sweep
                # paths into sweep_wait (column failure, lead-in dispatch,
                # lead-in failure) can arm the sampler.
                # Armed once per sweep, not once per tick: the trigger timer
                # (absent when gpr_trigger_enabled is off) cannot stand in for
                # "already armed", so the flag is explicit.
                if self._sweep_scanning and not self._sweep_samplers_armed:
                    self._sweep_samplers_armed = True
                    self._arm_gpr_triggers(ctx, seg_start, seg_end)
                    if not self._xpol:
                        self._start_hyperspectral(ctx, seg_start, seg_end)
                # The executor owns the sweep's own watchdogs and reports one
                # outcome; everything after it here is unchanged.
                if self._sweep_result is None:
                    return
                # The plate has stopped scanning; the GPR line is stopped and
                # exported below, before the press is released.
                self._hs.stop_line(ctx)
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
                self._hs.stop_line(ctx)
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
            # GPR: stop the line, export it, stop the measurement -- before the
            # press is released, so the probe is still on the wall for the stop.
            # Disarms the triggers, seals and writes the line record.
            self._gpr.end_segment(ctx)
            self._stop_force_mode(ctx)      # release the press before the arm retracts
            # Keep the reader running: the next segment's transit_clear needs the
            # plate distance to retract the arm before the base moves. Only the
            # alignment controller has to go, so the retraction goal is not fighting
            # its IK stream.
            self._stop_arm_processes(ctx, keep_sensors=True)
            # A pause before the next arm goal "to avoid errors in the arm goals"
            # (force mode releasing, the alignment controller's last trajectory
            # draining). Was a time.sleep(2) on the FSM's only thread.
            self._settled_sweep = (status == GoalStatus.STATUS_SUCCEEDED, reason,
                                   seg_start, seg_end, arm_sweep)
            self._settle_until = time.time() + float(
                ctx.get("scan_wall_post_sweep_settle_s", 2.0))
            self._seg_phase = "sweep_settle"
            return

        if self._seg_phase == "sweep_settle":
            if time.time() < self._settle_until:
                return
            args, self._settled_sweep = self._settled_sweep, None
            return self._advance_after_sweep(ctx, *args)

        if self._seg_phase == "xpol_clear":
            # Off the wall before anything rolls. Under force mode the executor
            # finishes with the plate still against the wall, and the move into
            # the rolled pose would drag the GPR wheel across it.
            self.set_activity(
                ctx,
                f"Retracting the plate before rolling it for segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_arm_to_clearance(
                ctx, self._roll_clearance(ctx), contact_default=True, retract_only=True)
            if outcome == "wait":
                return
            self._arm_goal_start = None
            self._seg_phase = "xpol_clear_wait" if outcome == "sent" else "xpol_clear_far"
            return

        if self._seg_phase == "xpol_clear_wait":
            if not self._arm_goal_settled(ctx):
                return
            self._arm_goal_start = None
            self._seg_phase = "xpol_clear_far"
            return

        if self._seg_phase == "xpol_clear_far":
            # Off the wall by the sweep standoff already (a reach the arm always
            # has); now, best effort, further back for the in-place roll, so its
            # goal sits clear of the octomap band. A refused move leaves the
            # plate where the first retract put it.
            if not self._roll_in_place_enabled(ctx):
                self._seg_phase = "xpol_pose"
                return
            outcome = self._send_arm_to_clearance(
                ctx, self._roll_in_place_clearance(ctx), retract_only=True)
            if outcome == "wait":
                return
            self._arm_goal_start = None
            self._seg_phase = "xpol_clear_far_wait" if outcome == "sent" else "xpol_roll"
            return

        if self._seg_phase == "xpol_clear_far_wait":
            if not self._arm_goal_settled(ctx):
                return
            self._arm_goal_start = None
            self._seg_phase = "xpol_roll"
            return

        if self._seg_phase == "xpol_roll":
            # Roll where the plate is: it already stands at the end of the
            # partition, which is where the rolled sweep starts. Going through
            # the named pose instead re-centred the arm only for the lead-in to
            # cross half a partition back.
            self.set_activity(
                ctx,
                f"Rolling the sensor plate in place for segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            if not self._send_arm_roll_goal(ctx, CROSS_POL_PLATE_ROLL):
                self._seg_phase = "xpol_pose"      # through the named pose instead
                return
            self._arm_goal_start = None
            self._seg_phase = "xpol_roll_wait"
            return

        if self._seg_phase == "xpol_roll_wait":
            status = self._named_pose_settled(ctx)
            if status == "wait":
                return
            self._arm_goal_start = None
            if status == "failed":
                node.get_logger().warn(
                    f"[{self.name}] In-place roll refused or incomplete; rolling through "
                    f"'{self._scan_pose_name(ctx)}' instead."
                )
                self._seg_phase = "xpol_pose"
                return
            node.get_logger().info(
                f"[{self.name}] Plate rolled {math.degrees(CROSS_POL_PLATE_ROLL):+.0f} deg in "
                f"place; sweeping partition {seg_no} back cross-polarized, the plate "
                f"{abs(self._plate_rise()) * 100:.0f} cm lower so the GPR stays on the line."
            )
            self._seg_phase = "sweep_setup"
            return

        if self._seg_phase == "xpol_pose":
            self.set_activity(
                ctx,
                f"Rolling the sensor plate for segment {seg_no}/{seg_total}{pass_label}",
                progress_current=seg_no,
                progress_total=seg_total,
            )
            outcome = self._send_named_pose(ctx, self._scan_pose_name(ctx))
            if outcome == "wait":
                return
            if outcome == "skip":
                self._sweep_result = (False, "xpol_pose_refused",
                                      f"'{self._scan_pose_name(ctx)}' was refused")
                self._seg_phase = "sweep_wait"
                return
            self._arm_goal_start = None
            self._seg_phase = "xpol_pose_wait"
            return

        if self._seg_phase == "xpol_pose_wait":
            status = self._named_pose_settled(ctx)
            if status == "wait":
                return
            self._arm_goal_start = None
            if status == "failed":
                # Never sweep "cross-polarized" with the plate in an unknown roll.
                self._sweep_result = (False, "xpol_pose_failed",
                                      f"'{self._scan_pose_name(ctx)}' did not complete")
                self._seg_phase = "sweep_wait"
                return
            node.get_logger().info(
                f"[{self.name}] Plate rolled {math.degrees(CROSS_POL_PLATE_ROLL):+.0f} deg; "
                f"sweeping partition {seg_no} back cross-polarized, the plate "
                f"{abs(self._plate_rise()) * 100:.0f} cm lower so the GPR stays on the line."
            )
            # The column was sized for both passes' heights; straight to the sweep.
            self._seg_phase = "sweep_setup"
            return

    def _arm_gpr_triggers(self, ctx, seg_start, seg_end):
        """Start clocking the GPR: the plate's pose now is d = 0. The segment
        is given in the world frame; the travel is measured in the plate-travel
        frame (arm_base for an arm sweep, with the base parked)."""
        if self._use_arm_sweep(ctx):
            speed = float(ctx.get("sweep_speed_mps", 0.05))
        elif bool(ctx.get("sweep_use_crawl", False)):
            speed = float(ctx.get("sweep_crawl_speed", self.SWEEP_CRAWL_SPEED_MS))
        else:
            speed = float(ctx.get("sweep_speed_limit", self.SWEEP_SPEED_LIMIT_MS))
        speed = speed or self.SWEEP_SPEED_LIMIT_MS   # 0.0 means "uncapped"
        self._gpr.arm_triggers(
            ctx, self._world_frame(ctx), seg_start, seg_end, speed,
            measure_frame=self._plate_travel_frame(ctx),
        )

    def _advance_after_sweep(self, ctx, ok, reason, seg_start, seg_end, arm_sweep,
                             skipped=False):
        """Book the sweep that just ended (or was found already done) and pick
        what comes next: the rolled pass of this partition and height, the next
        height at this base stop, or the next partition."""
        node = ctx["node"]
        seg_no = self._seg_idx + 1
        # This sweep's executor feedback must not arm the samplers of whatever
        # reaches sweep_wait next without a sweep of its own (a refused roll, a
        # column that did not reach its height): only a new goal sets them.
        self._sweep_scanning = False
        self._sweep_samplers_armed = False
        if ok:
            if not skipped:
                self._record_swept(ctx, seg_start, seg_end)
            if self._xpol:
                self._xpol_ok += 1
            else:
                self._segments_ok += 1
                self._last_swept_point = seg_end
        elif not self._xpol:
            # "Too long for the arm" is recoverable: re-cut this partition's
            # SEGMENT shorter and sweep it again, rather than dropping that
            # stretch of wall (ARM_SWEEP_PLAN §7.B). _shorten_failed_partition
            # rewinds _seg_idx itself when it succeeds.
            if arm_sweep and self._shorten_failed_partition(ctx, reason):
                self._line_idx = 0
                self._seg_phase = "transit_clear"
                return
            node.get_logger().warn(
                f"[{self.name}] Segment {seg_no} sweep did not succeed; "
                f"skipping to the next segment."
            )
        else:
            # The upright data of this stretch stands; only the rolled pass is lost.
            node.get_logger().error(
                f"[{self.name}] Cross-polarized sweep of partition {seg_no} failed "
                f"[{reason}]; keeping its co-polarized sweep."
            )

        # Straight back over the same partition with the plate rolled, at the
        # same base stop and the same column height -- only after an upright
        # sweep that covered it: one the arm could not reach or finish is not
        # worth a second attempt.
        if (ok and not self._xpol and arm_sweep and self._polarization(ctx)):
            self._xpol = True
            if self._already_swept(ctx, seg_end, seg_start):
                node.get_logger().info(
                    f"[{self.name}] Partition {seg_no} cross-polarized sweep already done "
                    f"on an earlier entry."
                )
            else:
                node.get_logger().info(
                    f"[{self.name}] Partition {seg_no} co-polarized sweep covered; next, "
                    f"the same partition back with the plate rolled "
                    f"{math.degrees(CROSS_POL_PLATE_ROLL):+.0f} deg."
                )
                self._seg_phase = "xpol_clear"
                return
            # Nothing to roll for: on to the next height or partition.
            self._xpol = False
        else:
            self._xpol = False

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
                    f"[{self.name}] Partition {seg_no}: on to height "
                    f"{self._line_idx + 1}/{len(heights)} "
                    f"(z={heights[self._line_idx]:.3f}m). Base stays parked."
                )
                # Retract and re-centre (a rolled plate turns upright on the
                # way) before the column moves.
                self._seg_phase = "line_change"
                return
            self._line_idx = 0
            ctx["current_line_idx"] = 0

        # Next partition. transit_clear retracts the plate and the re-centre or
        # the fold that follows brings a rolled plate back upright.
        self._seg_idx += 1
        self._seg_phase = "transit_clear"

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
        ctx["target_scan_start_end"] = self._sweep_to

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
               f"{self._segments_ok} sweep(s) succeeded"
               + (f", {self._xpol_ok} cross-polarized." if self._polarization(ctx) else ".")
               if self._nest_lines(ctx) else
               f"Line done ({self._segments_ok}/{len(self._segments)} segment(s) swept"
               + (f", {self._xpol_ok} cross-polarized)." if self._polarization(ctx) else ")."))
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
        # The rolled pass puts the GPR above the plate centre; lower the plate by
        # that much so the antenna, not the centre, sweeps the line.
        line_z = float(line_z) + self._plate_rise()

        goal = SweepLine.Goal()
        goal.start = Point(x=float(seg_start[0]), y=float(seg_start[1]), z=line_z)
        goal.end = Point(x=float(seg_end[0]), y=float(seg_end[1]), z=line_z)
        goal.partition_index = int(self._seg_idx)
        goal.partition_count = int(len(self._segments))
        goal.frame_id = self._world_frame(ctx)
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
        self._sweep_samplers_armed = False
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

    # --- Platform anisotropy (the reason the partition transit is axis-split) ---
    #
    # The base is holonomic AT THE TURRET POINT, but only because a sideways move
    # of that point is produced by ROTATING THE CHASSIS: theta_chassis_dot =
    # y_chassis / d1, with d1 the turret's offset from the wheel axle. Two numbers
    # follow, and both matter here:
    #
    #   max strafe speed = d1 * max_angular_base = 0.167 * 0.2 = 0.033 m/s
    #   chassis wind-up  = 1 / d1 = 6.0 rad per metre strafed
    #
    # while motion along the turret's own +X (which is the WALL NORMAL at a scan
    # pose) is limited only by wheel speed, ~0.2 m/s. That is a 6:1 anisotropy
    # between the two axes of a partition transit, and driving a constant-magnitude
    # vector at the goal through it converges on the fast axis (the wall normal)
    # first -- so the base reached "within tolerance" largely by moving toward or
    # away from the wall, and stopped wherever that left it. Hence the standoff
    # being sometimes much closer and sometimes much further than commanded.
    #
    # Defaults mirror navi-wall's diffdrive_controllers.yaml; override through ctx
    # if the platform is retuned.
    BASE_CENTER_DISTANCE_M = 0.167    # d1: turret offset from the wheel axle
    BASE_MAX_ANGULAR_BASE = 0.2       # rad/s cap on chassis rotation
    # Arrival tolerances for a partition transit, split because the two axes carry
    # very different consequences. The standoff sets how far the arm has to reach
    # and how hard force mode presses, so it is held tight; the along-wall position
    # only has to be close enough for the executor's lead-in to find the partition
    # start, and buying precision there costs real time at 0.033 m/s.
    PARTITION_STANDOFF_TOL_M = 0.05
    PARTITION_ALONG_WALL_TOL_M = 0.15
    # Bounded re-tries of the whole transit when the arrival check fails.
    PARTITION_TRANSIT_MAX_RETRIES = 2

    def _max_lateral_speed(self, ctx):
        """Fastest the base can strafe along the wall, in m/s.

        ``d1 * max_angular_base``: a sideways move of the turret point is produced
        by rotating the chassis, so the chassis rate limit IS the strafe limit.
        Overridable wholesale with ``crawl_max_lateral_speed`` for a platform whose
        controller has been retuned.
        """
        direct = ctx.get("crawl_max_lateral_speed")
        if direct is not None:
            return max(float(direct), 1e-3)
        d1 = float(ctx.get("base_center_distance_m", self.BASE_CENTER_DISTANCE_M))
        rate = float(ctx.get("base_max_angular_base", self.BASE_MAX_ANGULAR_BASE))
        return max(abs(d1 * rate), 1e-3)
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
        world = self._world_frame(ctx)
        primary = str(ctx.get("nav_base_frame", "turret_footprint"))
        frames = (primary,) + tuple(f for f in self.BASE_FRAMES if f != primary)
        for frame in frames:
            try:
                if not tf_buffer.can_transform(
                    world, frame, rclpy.time.Time(), Duration(seconds=0.2)
                ):
                    continue
                tf = tf_buffer.lookup_transform(
                    world, frame, rclpy.time.Time(), Duration(seconds=0.5)
                )
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return float(t.x), float(t.y), yaw
        return None

    def _start_sweep_crawl(self, ctx, target_xy, yaw=None, speed=None, tol=None,
                           timeout_pad=None, what="Sweep crawl", axis_split=False,
                           tol_normal=None):
        """Begin driving the base to ``target_xy`` (map frame) over /cmd_vel.

        Drives ``self._nav_status`` like a Nav2 goal: None while in flight,
        STATUS_SUCCEEDED on arrival, -1 on timeout/lost pose, so both the
        ``sweep_wait`` and ``transit_wait`` completion logic are reused unchanged.

        ``yaw`` overrides the heading held during the move; it defaults to the
        fixed along-wall sweep heading the legacy base sweep uses. The partition
        transit passes the scan-pose yaw instead, so the base arrives already
        square to the wall.

        ``axis_split`` drives the wall-normal and along-wall axes as separate
        components with separate tolerances (``tol_normal`` for the standoff,
        ``tol`` for the along-wall position) instead of one vector at the goal.
        See BASE_CENTER_DISTANCE_M for why the two axes are not interchangeable
        on this base; the short version is that the standoff axis is ~6x faster
        than the strafe axis and matters ~3x more, so a single isotropic
        tolerance gets both wrong.
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
        # Axis-split mode: resolve the wall frame ONCE per move. The base is parked
        # for the whole crawl in the sense that matters here -- the wall does not
        # move -- so re-deriving these every tick would only add TF jitter to the
        # split. None falls back to the isotropic legacy behaviour, which is what
        # the base-driven sweep still wants (it drives ALONG the wall by design and
        # has no standoff of its own to hold).
        self._crawl_axes = wall_axes(ctx) if axis_split else None
        if axis_split and self._crawl_axes is None:
            node.get_logger().warn(
                f"[{self.name}] {what}: no wall normal, so the move cannot be split "
                f"into standoff and along-wall axes; driving it as one vector and "
                f"leaving the standoff to the arrival check."
            )
        self._crawl_max_lateral = self._max_lateral_speed(ctx)
        self._crawl_tol_normal = (
            float(ctx.get("partition_standoff_tol", self.PARTITION_STANDOFF_TOL_M))
            if tol_normal is None else float(tol_normal)
        )

        pose = self._base_xy_yaw_map(ctx)
        dist = (
            math.hypot(self._sweep_crawl_target[0] - pose[0],
                       self._sweep_crawl_target[1] - pose[1])
            if pose else 0.0
        )
        self._crawl_distance = dist
        pad = self.SWEEP_CRAWL_TIMEOUT_PAD_S if timeout_pad is None else float(timeout_pad)
        # Budget the two axes at the speeds they can ACTUALLY run. Charging the
        # whole distance at the commanded speed was ~4.5x optimistic for an
        # along-wall move (0.15 m/s asked, 0.033 m/s available) and survived only
        # on the pad -- which is why a long transit could run out of budget and be
        # dropped as a failed segment.
        if self._crawl_axes is not None and pose is not None:
            (nx, ny), (tx, ty) = self._crawl_axes
            ex = self._sweep_crawl_target[0] - pose[0]
            ey = self._sweep_crawl_target[1] - pose[1]
            d_n, d_t = abs(ex * nx + ey * ny), abs(ex * tx + ey * ty)
            budget = d_n / max(speed, 1e-3) + d_t / max(self._crawl_max_lateral, 1e-3)
            node.get_logger().info(
                f"[{self.name}] {what} to ({self._sweep_crawl_target[0]:.2f}, "
                f"{self._sweep_crawl_target[1]:.2f}): {d_n:.2f} m of standoff at "
                f"{speed:.3f} m/s + {d_t:.2f} m along the wall at "
                f"{self._crawl_max_lateral:.3f} m/s (the platform's strafe limit)."
            )
        else:
            budget = dist / max(speed, 1e-3)
            node.get_logger().info(
                f"[{self.name}] {what} to ({self._sweep_crawl_target[0]:.2f}, "
                f"{self._sweep_crawl_target[1]:.2f}) at {speed:.3f} m/s ({dist:.2f} m)."
            )
        self._sweep_crawl_deadline = time.time() + budget + pad
        self._sweep_crawl_timer = node.create_timer(
            1.0 / self.SWEEP_CRAWL_RATE_HZ, lambda: self._sweep_crawl_tick(ctx)
        )

    def _transit_arrival_ok(self, ctx, seg_no):
        """Verify the base actually reached this partition's scan pose.

        Returns True to carry on to the sweep. Returns False having already set
        the next phase: either a bounded re-try of the transit, or -- once those
        are spent -- skipping the partition, because sweeping from a standoff the
        arm cannot span is worse than not sweeping it.

        Checked in the WALL frame, not as one distance: the standoff (wall-normal)
        is what sets the arm's reach and is held to partition_standoff_tol, while
        the along-wall position only has to be close enough for the executor's
        lead-in and gets the loose tolerance. The heading is checked too -- a
        parked arm sweep is only symmetric about the base centreline if the base
        is square to the wall.
        """
        node = ctx["node"]
        target = (self._scan_poses or [None] * len(self._segments))[self._seg_idx]
        err = self._scan_pose_error(ctx, target)
        if err is None:
            node.get_logger().warn(
                f"[{self.name}] Cannot verify the partition {seg_no} scan pose "
                f"(no base pose, wall normal or scan pose); trusting the transit "
                f"and sweeping from here."
            )
            return True
        if err["ok"]:
            node.get_logger().info(
                f"[{self.name}] Partition {seg_no} scan pose verified: standoff "
                f"error {err['normal']:+.3f} m, along-wall {err['along']:+.3f} m, "
                f"yaw {err['yaw']:.3f} rad."
            )
            self._transit_retries = 0
            return True

        max_retries = int(ctx.get("partition_transit_max_retries",
                                  self.PARTITION_TRANSIT_MAX_RETRIES))
        # Say which axis failed: a standoff miss and an along-wall miss have
        # different causes and different consequences for the sweep.
        why = err["why"]
        if self._transit_retries < max_retries:
            self._transit_retries += 1
            node.get_logger().warn(
                f"[{self.name}] Partition {seg_no} scan pose not reached ({why}); "
                f"re-running the transit ({self._transit_retries}/{max_retries})."
            )
            # Back to `transit`, which re-reads the scan pose and re-decides
            # whether the move is even needed -- the skip test there will accept
            # the pose if a later measurement says it is fine after all.
            self._seg_phase = "transit"
            return False

        node.get_logger().error(
            f"[{self.name}] Partition {seg_no} scan pose still not reached after "
            f"{max_retries} re-tries ({why}); skipping this partition rather than "
            f"sweeping it from a pose the arm cannot span."
        )
        self._transit_retries = 0
        self._seg_idx += 1
        self._seg_phase = "transit_clear"
        return False

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

        # Split the error into standoff (wall normal) and along-wall (tangent)
        # components. They get different tolerances because they mean different
        # things: the standoff sets how far the arm has to reach and is the thing
        # that was coming out wrong, while the along-wall position only has to be
        # close enough for the arm's own lead-in to centre on the partition.
        axes = getattr(self, "_crawl_axes", None)
        if axes is not None:
            (nx, ny), (tx, ty) = axes
            e_n, e_t = ex * nx + ey * ny, ex * tx + ey * ty
            tol_n = getattr(self, "_crawl_tol_normal", tol)
            arrived = abs(e_n) <= tol_n and abs(e_t) <= tol
        else:
            e_n = e_t = None
            tol_n = tol
            arrived = dist <= tol

        if arrived:
            self._stop_sweep_crawl(ctx, publish_stop=True)
            if axes is not None:
                node.get_logger().info(
                    f"[{self.name}] Base crawl reached its target: standoff error "
                    f"{e_n:+.3f} m (tol {tol_n:.3f}), along-wall {e_t:+.3f} m "
                    f"(tol {tol:.3f})."
                )
            else:
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

        # Velocity toward the target in the map frame, then rotated into the base
        # frame below (Twist is body-frame), plus a P-hold on the heading.
        if axes is not None:
            # Per-axis speeds, so the along-wall component is commanded at what
            # the base can actually strafe rather than 4.5x that. Asking for more
            # does not go faster -- the controller scales the whole command down
            # to the feasible set -- it just makes the standoff component shrink
            # with it, which is how the fast axis ended up doing the converging.
            # Each axis is zeroed once ITS OWN tolerance is met, so the last part
            # of the move is along whichever axis is still out, not a diagonal.
            v_n = 0.0 if abs(e_n) <= tol_n else math.copysign(speed, e_n)
            v_t = (0.0 if abs(e_t) <= tol else
                   math.copysign(min(self._crawl_max_lateral, speed), e_t))
            # Do not overshoot on a tick: cap each component at the distance left.
            step = 1.0 / self.SWEEP_CRAWL_RATE_HZ
            v_n = math.copysign(min(abs(v_n), abs(e_n) / step), v_n) if v_n else 0.0
            v_t = math.copysign(min(abs(v_t), abs(e_t) / step), v_t) if v_t else 0.0
            vx_w, vy_w = v_n * nx + v_t * tx, v_n * ny + v_t * ty
        else:
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
            if not self._controller_gate.ready(node):
                return
            self._xpol = False
            pose_name = self._scan_pose_name(ctx)
            request = SendPosition.Request()
            request.position_name = pose_name
            ctx["execution_status"] = False
            ctx["planner_goal_failed"] = False
            self.retract_future = self.position_client.call_async(request)
            self._controller_gate.reset()
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
                self._note_arm_folded(ctx, False)
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
        self._hs.abort(ctx)   # disarm, close the open segment, persist metrics
        self._restore_sweep_speed(ctx)
        self._cancel_sweep_goal(ctx)
        self._set_trajectory_bridge_hold(ctx, False)   # never leave the arm stack muted
        self._gpr.abort(ctx)               # stop line, export, stop measurement, record
        self._stop_force_mode(ctx)
        self._stop_arm_processes(ctx)

    def reset_run(self, ctx):
        # The sampler keeps its session for the whole mission (configure() is
        # idempotent); a restarted run is a new mission with its own session.
        self._hs.new_mission()
        self._transit_retries = 0

    def check_transition(self, ctx):
        if self.finished and self.more_lines:
            return "ScanWall"        # same wall, next height (base stays put)
        if self.finished:
            return "SensorDataProcessing"
        if ctx.get("error_triggered"):
            return "Error"
        return None
