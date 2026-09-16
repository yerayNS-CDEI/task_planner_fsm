import argparse
import json
import math
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
import rclpy.time
from geometry_msgs.msg import Point, Pose, Quaternion, WrenchStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
import tf2_ros
from rclpy.duration import Duration
from std_msgs.msg import Bool, Float32MultiArray, String

from task_planner_fsm.machine import StateMachine, seed_wall_detection_ctx
from task_planner_fsm.sensors import paths as sensor_paths
from task_planner_fsm.states import (
    ArmFolding,
    ArmUnfolding,
    ComputeWallPoints,
    CreateMap,
    Error,
    Finished,
    GeometryReconstruction,
    HomePosition,
    Initialization,
    NavigateToTarget,
    ObjectID,
    WallLinesComputation,
    ScanWall,
    WallTargetSelection,
    SensorDataProcessing,
    SendDataToPokeye,
    ScanCeiling,
    ScanFloor,
    # AreasOfInterest,
    # BasePlacement,
    # ExhaustiveScan,
    # WallDiscretization,
)
from task_planner_fsm.states.proc_utils import (
    start_proc,
    stop_all,
    wait_stack_ready,
    wait_services_ready,
    COLLISION_READY_TIMEOUT_S,
    ROBOT_STACK_LAUNCH_SHUTDOWN_ARGS,
    STACK_READY_TIMEOUT_S,
)
from task_planner_fsm.telemetry import build_fsm_graph_payload, make_json_safe
from task_planner_fsm.utils.costmap_utils import (
    DEFAULT_NAV_BASE_FRAME,
    DEFAULT_PARTITION_BASE_STANDOFF,
    DEFAULT_SCAN_LINE_OFFSET,
    world_frame,
)
from task_planner_fsm.utils.wall_geometry import (
    build_wall_data,
    build_wall_in_front,
    left_scan_endpoint,
)

FSM_STATE_ORDER = [
    "Initialization",
    "CreateMap",
    "ObjectID",
    "WallLinesComputation",
    "GeometryReconstruction",
    "ComputeWallPoints",
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ScanWall",
    "SensorDataProcessing",
    "SendDataToPokeye",
    "ArmFolding",
    "ScanFloor",
    "ScanCeiling",
    # "AreasOfInterest",
    # "WallDiscretization",
    # "BasePlacement",
    # "ExhaustiveScan",
    "HomePosition",
    "Finished",
    "Error",
]

PHASE2_DEFAULT_INITIAL_STATES = {
    # "AreasOfInterest",
    # "WallDiscretization",
    # "BasePlacement",
    # "ExhaustiveScan",
    "ScanFloor",
    "HomePosition",
    "Finished",
}

PHASE3_DEFAULT_INITIAL_STATES = {
    "ScanCeiling",
    "HomePosition",
    "Finished",
}

WALL_DATA_REQUIRED_INITIAL_STATES = {
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    # "AreasOfInterest",
    # "WallDiscretization",
    # "BasePlacement",
    # "ExhaustiveScan",
    "HomePosition",
}

PHASE1_TARGET_REQUIRED_INITIAL_STATES = {
    "NavigateToTarget",
    "ArmUnfolding",
    "ScanWall",
}

PHASE2_BASE_REQUIRED_INITIAL_STATES = {
    "NavigateToTarget",
    "ArmUnfolding",
    "ScanWall",
    "ArmFolding",
    # "ExhaustiveScan",
}

NEEDS_SYNTHETIC_DISCRETIZATION_INITIAL_STATES = {
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    # "BasePlacement",
    # "ExhaustiveScan",
    "HomePosition",
}

NAV_CLIENT_BOOTSTRAP_STATES = {
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "ScanFloor",
    # "AreasOfInterest",
    # "WallDiscretization",
    # "BasePlacement",
    # "ExhaustiveScan",
    "HomePosition",
}

# States that work on what is already on disk and touch no hardware. Starting
# at one of these is an offline run: no robot stack is launched, nothing
# prompts for walls, and the run ends there (fsm_stop_after) instead of
# carrying on into ArmFolding / SendDataToPokeye. See
# _bootstrap_sensor_processing.
OFFLINE_INITIAL_STATES = {"SensorDataProcessing"}

# GeometryReconstruction is the first state of the wall-processing/scanning
# pipeline: it itself only needs navi_wall's detected_walls.yaml on disk, but it
# flows straight into ComputeWallPoints -> WallTargetSelection -> NavigateToTarget,
# which need the navigation + localization stack. In the normal flow that stack is
# already up (started by ObjectID), so when bootstrapping from GeometryReconstruction
# (or later) we start it here too.
NAV_SIM_REQUIRED_START_STATES = {
    s
    for s in FSM_STATE_ORDER[FSM_STATE_ORDER.index("GeometryReconstruction") :]
    if s not in {"Finished", "Error"} | OFFLINE_INITIAL_STATES
}


# Fallback walls, only used when navi_wall's detected_walls.yaml cannot be
# loaded during bootstrap. The default wall source is the YAML (see
# _load_bootstrap_walls / GeometryReconstruction._load_detected_walls_from_yaml).
PREDEFINED_WALLS = [
    ((4.0, 0.0, 2.0), (4.0, -3.0, 3.0)),
    ((9.0, 0.0, 0.19), (9.0, -4.5, 2.0)),
    ((10.0, -4.5, 0.2), (10.0, 0.0, 3.0)),
    ((4.0, 2.0, 0.2), (8.0, 2.0, 3.0)),
]

# Endpoint z used for YAML walls that don't carry a z_min (mirrors
# GeometryReconstruction.default_wall_z).
BOOTSTRAP_WALL_DEFAULT_Z = 0.0


class BootstrapError(RuntimeError):
    """The bootstrap could not assemble what the initial state needs.

    Raised instead of flagging ``error_triggered``, because the first state's
    ``on_enter`` clears that flag and would run anyway -- for a bench start
    that means unfolding the arm with no wall to sweep. Whatever the bootstrap
    launched has already been stopped when this is raised.
    """


# Where a bootstrap gets its walls from.
#   yaml      navi_wall's detected_walls.yaml, chosen interactively (the default)
#   in-front  ONE short wall synthesised from where the robot stands: the base is
#             taken to be parked at its scan pose already, facing the wall. For
#             bench runs of the scan states on a hand-placed robot -- no
#             navigation, no map-derived walls. See _wall_in_front_of_robot.
WALL_SOURCES = ("yaml", "in-front")

# Frames tried, in order, for the base pose an in-front wall is built from.
# Mirrors ScanWall.BASE_FRAMES: the first is Nav2's base frame, which is also
# what the partition scan pose is expressed in.
BOOTSTRAP_BASE_FRAMES = (
    DEFAULT_NAV_BASE_FRAME, "base_footprint", "base_link", "base", "chassis",
)


class RobotFSMNode(Node):
    def __init__(
        self,
        sim: bool = False,
        initial_state: str = "Initialization",
        scan_phase: Optional[int] = None,
        planner_backend: str = "legacy",
        wall_source: str = "yaml",
        launch_stack: bool = True,
        stop_after: Optional[str] = None,
    ):
        # Auto-declare any parameter passed as an override (e.g. via
        # `--ros-args -p create_map_sweep_axis:=perpendicular` or a launch file),
        # so per-state tuning knobs read from ctx below can be set without a
        # declare_parameter call for each one.
        super().__init__(
            "robot_fsm_node",
            automatically_declare_parameters_from_overrides=True,
        )
        self.initial_state = initial_state
        self._stdin_warned = False
        self.planner_backend = str(planner_backend).strip().lower()
        if wall_source not in WALL_SOURCES:
            raise ValueError(
                f"Invalid wall source '{wall_source}'. Valid options: {', '.join(WALL_SOURCES)}"
            )
        self.wall_source = wall_source
        # False: the robot stack (move_robot.launch.py) is already up in another
        # terminal; the bootstrap only waits for it instead of launching a second
        # copy on top of it.
        self.launch_stack = bool(launch_stack)
        self._stack_ensured = False

        # NOTE: Do NOT set use_sim_time=True here!
        # The FSM timer must run on wall time even in simulation mode,
        # otherwise it blocks waiting for /clock before simulation starts.
        # TF checks will still work correctly by using rclpy.time.Time() which
        # automatically gets the latest available transform regardless of time source.
        
        # TF2 buffer and listener for transform checks
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # FSM telemetry publishers
        current_qos = QoSProfile(depth=1)
        current_qos.reliability = ReliabilityPolicy.RELIABLE
        current_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transition_qos = QoSProfile(depth=20)
        transition_qos.reliability = ReliabilityPolicy.RELIABLE
        transition_qos.durability = DurabilityPolicy.VOLATILE
        event_qos = QoSProfile(depth=100)
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        event_qos.durability = DurabilityPolicy.VOLATILE

        self.fsm_current_pub = self.create_publisher(String, "/fsm/current_state", current_qos)
        self.fsm_transition_pub = self.create_publisher(String, "/fsm/transition", transition_qos)
        self.fsm_graph_pub = self.create_publisher(String, "/fsm/graph", current_qos)
        self.fsm_status_pub = self.create_publisher(String, "/fsm/status", current_qos)
        self.fsm_event_pub = self.create_publisher(String, "/fsm/event", event_qos)

        # Shared context for the FSM
        self.ctx = {
            "node": self,
            "start": False,
            "map_ready": False,
            "error_triggered": False,
            "last_state": None,
            "scan_phase": 1,
            "execution_status": False,
            "planner_goal_failed": False,
            "sim": bool(sim),
            "planner_backend": self.planner_backend,
            "_fsm_status": {},
            "publish_fsm_current": self.publish_fsm_current,
            "publish_fsm_transition": self.publish_fsm_transition,
            "publish_fsm_graph": self.publish_fsm_graph,
            "set_fsm_status": self.set_fsm_status,
            "publish_fsm_status": self.publish_fsm_status,
            "publish_fsm_event": self.publish_fsm_event,
            "tf_buffer": self.tf_buffer,
        }

        # Bridge externally-set ROS parameter overrides into ctx so per-state
        # tuning knobs (create_map_*, object_id_*, scan_*, ...) can be configured
        # at launch. setdefault keeps the explicit ctx values above authoritative
        # (a param cannot clobber e.g. "node"/"sim"); any other override lands in
        # ctx under its flat name and is picked up by the matching ctx.get(...).
        param_overrides = self.get_parameters_by_prefix("")
        for pname, param in param_overrides.items():
            if pname == "use_sim_time":
                continue
            self.ctx.setdefault(pname, param.value)
            self.get_logger().info(f"[FSM] ctx param override: {pname}={param.value!r}")
        # Last state to run; its onward transition goes to Finished instead
        # (StateMachine.step). The flag wins over a same-named param; unset,
        # the mission runs to its natural end.
        if stop_after:
            self.ctx["fsm_stop_after"] = stop_after

        # Wall-detection defaults MUST be seeded before the bootstrap: when the
        # FSM starts at a state past ObjectID, the bootstrap stands in for it and
        # launches the detector itself, which needs wall_detection_cmd to already
        # be in ctx. StateMachine seeds these too, but it is constructed after
        # the bootstrap runs.
        seed_wall_detection_ctx(self.ctx)

        # Build test context for non-default initial state.
        self._bootstrap_context_for_initial_state(initial_state, scan_phase)

        # FSM
        self.machine = StateMachine(
            [
                Initialization("Initialization"),
                CreateMap("CreateMap"),
                ObjectID("ObjectID"),
                WallLinesComputation("WallLinesComputation"),
                GeometryReconstruction("GeometryReconstruction"),
                ComputeWallPoints("ComputeWallPoints"),
                WallTargetSelection("WallTargetSelection"),
                NavigateToTarget("NavigateToTarget"),
                ArmUnfolding("ArmUnfolding"),
                ArmFolding("ArmFolding"),
                ScanWall("ScanWall"),
                ScanFloor("ScanFloor"),
                ScanCeiling("ScanCeiling"),
                SensorDataProcessing("SensorDataProcessing"),
                SendDataToPokeye("SendDataToPokeye"),
                # AreasOfInterest("AreasOfInterest"),
                # WallDiscretization("WallDiscretization"),
                # BasePlacement("BasePlacement"),
                # ExhaustiveScan("ExhaustiveScan"),
                HomePosition("HomePosition"),
                Finished("Finished"),
                Error("Error"),
            ],
            initial_state=initial_state,
            ctx=self.ctx,
        )
        self.publish_fsm_graph()

        # Subscriptions
        self.create_subscription(Bool, "/start_flag", self.start_callback, 10)
        self.create_subscription(Odometry, "/rtabmap/odom", self.odometry_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Bool, "/execution_status", self.execution_status_callback, 10)
        self.create_subscription(Bool, "/arm/execution_status", self.execution_status_callback, 10)
        self.create_subscription(Bool, "/planner/goal_failed", self.planner_goal_failed_callback, 10)
        self.create_subscription(Bool, "/map_done", self.mapping_callback, 10)
        # TCP force/torque sensor (ur_ros2_driver's force_torque_sensor_broadcaster).
        # ScanWall uses the wall-normal (arm_tool0 Z) force to detect when the GPR
        # wheel touches the wall before starting the measurement and the base sweep.
        # The broadcaster publishes best-effort, so subscribe best-effort (a reliable
        # subscription would drop every message).
        ft_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.ft_data_callback,
            ft_qos,
        )
        # Sensor-plate ranges from arm_control's arduino_sensors(_sim): six floats
        # in metres, [C/U1, A/U2, B/U3] ultrasonic then [S1, S2, S3] ToF. The
        # wall_parallel_controller consumes the same topic for its plane fit, but it
        # only logs the resulting mean and is not running between segments, so
        # ScanWall reads the raw frame instead: it drives the arm's Z approach to the
        # wall and the retraction that clears the plate before the base slides to the
        # next segment.
        self.create_subscription(
            Float32MultiArray, "/distance_sensors", self.distance_sensors_callback, 10
        )
        # Link status of the GPR trigger bridge (gpr_trigger_bridge node: JSON
        # once a second with alive/acked/lost counters for the ESP32 fake
        # encoder on the robot Wi-Fi). ScanWall refuses to sweep on a dead link
        # when gpr_trigger_bridge_required is set (real robot with the ESP32).
        self.create_subscription(
            String,
            str(self.ctx.get("gpr_trigger_bridge_status_topic", "/gpr_trigger_bridge/status")),
            self.gpr_trigger_bridge_status_callback,
            10,
        )
        # Nav2 global costmap (latched) so states can project scan goals onto the
        # nearest cell the base can actually occupy. See costmap_utils.
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.global_costmap_callback, costmap_qos
        )
        # Chassis-parking status from sim_controller (latched, transient_local so a
        # late subscriber gets the current value). True while a /sim_controller/park_now
        # maneuver runs, false when the chassis is aligned with the turret. ScanWall
        # parks the base to face the wall before the sweep and waits on this flag.
        parking_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/sim_controller/parking_active", self.parking_active_callback, parking_qos
        )

        # Timer
        self.timer = self.create_timer(1.0, self.machine.step)
        self.get_logger().info(f"[FSM] Simulation mode: {self.ctx['sim']}")
        self.get_logger().info(f"[FSM] Planner backend: {self.ctx['planner_backend']}")
        self.get_logger().info(
            f"[FSM] Initial state: {initial_state}, scan_phase={self.ctx.get('scan_phase')}"
        )

    def _safe_input(self, prompt: str) -> str:
        try:
            return input(prompt)
        except EOFError:
            if not self._stdin_warned:
                self.get_logger().warn(
                    "[FSM] STDIN not available for bootstrap prompts. Falling back to defaults."
                )
                self._stdin_warned = True
            return ""

    def _prompt_int(self, prompt: str, min_value: int, max_value: int, default: int) -> int:
        while True:
            raw = self._safe_input(f"{prompt} [{default}] ").strip()
            if not raw:
                return default
            try:
                value = int(raw)
            except ValueError:
                print(f"Invalid integer: '{raw}'")
                continue
            if value < min_value or value > max_value:
                print(f"Value must be in [{min_value}, {max_value}].")
                continue
            return value

    def _build_wall_data(
        self,
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        offset: float = 0.6,
        scan_lines_z: Optional[List[float]] = None,
        outward_normal: Optional[Tuple[float, float]] = None,
    ) -> Dict[str, Tuple]:
        return build_wall_data(
            p1, p2, offset=offset, scan_lines_z=scan_lines_z, outward_normal=outward_normal
        )

    def _prompt_walls_data(self) -> List[Dict[str, Tuple]]:
        available_walls = self._load_bootstrap_walls()

        # The selection number below (1..N) is the same 1-based order the RViz
        # 'detected_wall_labels' markers use ("W1", "W2", ...), so picking N here
        # scans the wall labelled "W{N}" in RViz. The YAML id is shown too for
        # cross-referencing detected_walls.yaml.
        print("[FSM Bootstrap] Available walls (number matches the green 'W#' label in RViz):")
        for i, (wall_id, (p1, p2), _n) in enumerate(available_walls, start=1):
            print(f"  {i}: W{i}  (yaml id={wall_id})  p1={p1}, p2={p2}")

        max_walls = len(available_walls)
        num_walls = self._prompt_int(
            f">> Number of walls to include for this run (1-{max_walls})",
            1,
            max_walls,
            max_walls,
        )

        default_indices = list(range(1, num_walls + 1))
        while True:
            raw = self._safe_input(
                f">> Wall indices to include (e.g. 1 3) {default_indices}: "
            ).strip()
            if not raw:
                selected_indices = default_indices
            else:
                try:
                    selected_indices = [int(tok) for tok in raw.replace(",", " ").split()]
                except ValueError:
                    print(f"Invalid index list: '{raw}'")
                    continue

            if len(selected_indices) != num_walls:
                print(
                    f"You requested {num_walls} wall(s), but provided {len(selected_indices)} indices."
                )
                continue
            if len(set(selected_indices)) != len(selected_indices):
                print("Duplicate wall indices are not allowed.")
                continue
            if any(idx < 1 or idx > max_walls for idx in selected_indices):
                print(f"All indices must be in [1, {max_walls}].")
                continue

            walls_data = []
            for idx in selected_indices:
                _wall_id, (p1, p2), outward_normal = available_walls[idx - 1]
                scan_lines_z = self._prompt_wall_lines(idx)
                walls_data.append(
                    self._build_wall_data(
                        p1, p2, scan_lines_z=scan_lines_z, outward_normal=outward_normal
                    )
                )
            return walls_data

    def _load_bootstrap_walls(self) -> List[Tuple[object, Tuple[Tuple, Tuple]]]:
        """Selectable walls for a direct (bootstrap) start, loaded from
        navi_wall's ``detected_walls.yaml`` as ``((x, y, z), (x, y, z))``
        map-frame endpoint pairs — the same source and shape GeometryReconstruction
        feeds into ComputeWallPoints. Falls back to ``PREDEFINED_WALLS`` only when
        the YAML is missing/empty so a demo run without navi_wall still works.
        """
        detections_dir = self._resolve_detections_dir()
        yaml_path = (
            os.path.join(detections_dir, "detected_walls.yaml")
            if detections_dir is not None
            else None
        )
        raw_walls = self._load_walls_from_yaml(yaml_path) if yaml_path else []
        if not raw_walls:
            self.get_logger().warn(
                f"[FSM Bootstrap] No walls from detected_walls.yaml "
                f"({yaml_path or 'navi_wall rgb_detections dir not found'}); "
                f"falling back to hardcoded PREDEFINED_WALLS."
            )
            return [(i, (p1, p2), None) for i, (p1, p2) in enumerate(PREDEFINED_WALLS)]

        walls = []
        for order, w in enumerate(raw_walls):
            z = float(w.get("z_min", BOOTSTRAP_WALL_DEFAULT_Z))
            p1, p2 = w["p1"], w["p2"]
            wall_id = w.get("id", order)
            walls.append(
                (
                    wall_id,
                    ((float(p1[0]), float(p1[1]), z), (float(p2[0]), float(p2[1]), z)),
                    w.get("normal"),
                )
            )
        self.get_logger().info(
            f"[FSM Bootstrap] Loaded {len(walls)} wall(s) from '{yaml_path}'."
        )
        return walls

    def _resolve_detections_dir(self) -> Optional[str]:
        """Locate the navi_wall ``rgb_detections`` directory.

        Order: explicit override param -> package share folder -> dev fallback to
        the source checkout under ``<ws>/src``. Returns the first existing
        directory, or None. Mirrors GeometryReconstruction._resolve_detections_dir.
        """
        candidates = []

        override = self.ctx.get("geometry_reconstruction_rgb_detections_dir")
        if override:
            candidates.append(os.path.expanduser(str(override)))

        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("navi_wall")
            candidates.append(os.path.join(share, "rgb_detections"))
            if os.sep + "install" + os.sep in share:
                ws = share.split(os.sep + "install" + os.sep, 1)[0]
                for pkg_dir in ("navi-wall", "navi_wall"):
                    candidates.append(os.path.join(ws, "src", pkg_dir, "rgb_detections"))
        except Exception as exc:
            self.get_logger().warn(
                f"[FSM Bootstrap] Could not resolve navi_wall share directory: {exc}"
            )

        for cand in candidates:
            if cand and os.path.isdir(cand):
                return cand
        return None

    def _load_walls_from_yaml(self, yaml_path: str) -> List[Dict]:
        """Parse navi_wall's ``detected_walls.yaml`` into ``{'p1','p2','z_min','z_max'}``
        entries. Mirrors GeometryReconstruction._load_walls_from_yaml.
        """
        if not yaml_path or not os.path.isfile(yaml_path):
            return []
        try:
            import yaml

            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(
                f"[FSM Bootstrap] Could not parse '{yaml_path}': {exc}"
            )
            return []

        walls = []
        for w in data.get("walls", []) or []:
            p1 = w.get("p1")
            p2 = w.get("p2")
            if not p1 or not p2:
                continue
            entry = {
                "id": w.get("id", len(walls)),
                "p1": (float(p1[0]), float(p1[1])),
                "p2": (float(p2[0]), float(p2[1])),
            }
            if w.get("z_min") is not None:
                entry["z_min"] = float(w["z_min"])
            if w.get("z_max") is not None:
                entry["z_max"] = float(w["z_max"])
            # Outward wall normal (RViz arrow) so scanning uses the interior side.
            n = w.get("normal")
            if n and len(n) >= 2:
                entry["normal"] = (float(n[0]), float(n[1]))
            walls.append(entry)
        return walls

    def _prompt_wall_lines(self, wall_idx: int) -> List[float]:
        """Prompt for the number of horizontal lines and their z heights.

        Returns a list sorted ascending (bottom first). Accepting the defaults
        (empty input) yields a single line, preserving single-pass behaviour.
        """
        num_lines = self._prompt_int(f">> Wall {wall_idx}: number of horizontal lines", 1, 20, 1)
        default_str = " ".join("0.0" for _ in range(num_lines))
        while True:
            raw = self._safe_input(
                f">> Wall {wall_idx}: z values for the {num_lines} line(s) "
                f"(e.g. 0.5 1.2 2.0) [{default_str}]: "
            ).strip()
            if not raw:
                return [0.0] * num_lines
            try:
                z_values = [float(tok) for tok in raw.replace(",", " ").split()]
            except ValueError:
                print(f"Invalid z value list: '{raw}'")
                continue
            if len(z_values) != num_lines:
                print(f"Requested {num_lines} line(s) but provided {len(z_values)} z value(s).")
                continue
            return sorted(z_values)

    def _make_pose(self, x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def _generate_mock_discretization(self, walls_data: List[Dict[str, Tuple]]) -> Dict[str, List]:
        results = {
            "wall_panels_centers": [],
            "wall_panels_vertices": [],
            "panel_cells_centers": [],
            "panel_cells_vertices": [],
        }

        for wall in walls_data:
            p1, p2 = wall["original"]
            x1, y1, z1 = p1
            x2, y2, z2 = p2

            z_low = min(z1, z2)
            z_high = max(z1, z2)
            if abs(z_high - z_low) < 0.2:
                z_high = z_low + 1.0

            # One quad panel per wall.
            v0 = self._make_pose(x1, y1, z_low)
            v1 = self._make_pose(x2, y2, z_low)
            v2 = self._make_pose(x2, y2, z_high)
            v3 = self._make_pose(x1, y1, z_high)
            results["wall_panels_vertices"].append([v0, v1, v2, v3])
            results["wall_panels_centers"].append(
                self._make_pose((x1 + x2) / 2.0, (y1 + y2) / 2.0, (z_low + z_high) / 2.0)
            )

            # Synthetic cell centers for exhaustive scan tests.
            panel_cells = []
            for z_ratio in (0.25, 0.60):
                for t in (0.2, 0.5, 0.8):
                    panel_cells.append(
                        self._make_pose(
                            x1 + t * (x2 - x1),
                            y1 + t * (y2 - y1),
                            z_low + z_ratio * (z_high - z_low),
                        )
                    )

            results["panel_cells_centers"].append(panel_cells)

        return results

    def _generate_mock_base_data(
        self, wall_discretization_results: Dict[str, List]
    ) -> Tuple[Dict[int, List[int]], Dict[int, Tuple[float, float]]]:
        wall_panels_vertices = wall_discretization_results.get("wall_panels_vertices", [])
        seen_xy_keys: Dict[Tuple[Tuple[float, float], ...], int] = {}
        base_to_panel_indices: Dict[int, List[int]] = {}
        optimal_base_results: Dict[int, Tuple[float, float]] = {}

        global_panel_idx = 0
        for wall_verts_flat in wall_panels_vertices:
            for i in range(0, len(wall_verts_flat), 4):
                panel = wall_verts_flat[i : i + 4]
                if len(panel) < 4:
                    continue

                xy_key = tuple(
                    sorted(
                        (
                            round(float(v.position.x), 1),
                            round(float(v.position.y), 1),
                        )
                        for v in panel
                    )
                )

                if xy_key not in seen_xy_keys:
                    col_rank = len(seen_xy_keys)
                    seen_xy_keys[xy_key] = col_rank
                    base_to_panel_indices[col_rank] = []

                    center_x = sum(float(v.position.x) for v in panel) / 4.0
                    center_y = sum(float(v.position.y) for v in panel) / 4.0
                    dx = float(panel[1].position.x) - float(panel[0].position.x)
                    dy = float(panel[1].position.y) - float(panel[0].position.y)
                    norm = math.hypot(dx, dy) or 1.0
                    nx = dy / norm
                    ny = -dx / norm
                    optimal_base_results[col_rank] = (
                        round(center_x + 0.9 * nx, 3),
                        round(center_y + 0.9 * ny, 3),
                    )
                else:
                    col_rank = seen_xy_keys[xy_key]

                base_to_panel_indices[col_rank].append(global_panel_idx)
                global_panel_idx += 1

        return base_to_panel_indices, optimal_base_results

    def _resolve_scan_phase(self, initial_state: str, scan_phase: Optional[int]) -> int:
        if scan_phase in (1, 2):
            return int(scan_phase)
        if initial_state in PHASE2_DEFAULT_INITIAL_STATES:
            return 2
        return 1

    def _closest_scan_line_to_point(
        self, walls_data: List[Dict[str, Tuple]], point_xy: Tuple[float, float]
    ) -> Optional[Dict[str, Tuple]]:
        min_dist = float("inf")
        selected_wall = None

        for wall in walls_data:
            scan_line = wall.get("scan_line")
            if not scan_line or len(scan_line) != 2:
                continue
            for pt in scan_line:
                dist = ((point_xy[0] - pt[0]) ** 2 + (point_xy[1] - pt[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    selected_wall = wall

        return selected_wall

    def _ensure_reference_target_from_walls(self, selected_base: Optional[Tuple[float, float]] = None):
        if self.ctx.get("target_scan_wall") and self.ctx.get("target_scan_point"):
            return

        walls_data = self.ctx.get("walls_data", [])
        if not walls_data:
            return

        selected_wall = None
        if selected_base is not None:
            selected_wall = self._closest_scan_line_to_point(walls_data, selected_base)
        if selected_wall is None:
            selected_wall = walls_data[0]

        selected_line = selected_wall.get("scan_line")
        if selected_line and len(selected_line) == 2:
            # Start from the LEFT scan endpoint so the base's left flank (sensor
            # plate side) faces the wall, matching WallTargetSelection.
            self.ctx["target_scan_wall"] = selected_line
            self.ctx["target_scan_point"] = left_scan_endpoint(
                selected_line, selected_wall.get("inward_normal")
            )

    def _prompt_phase1_target(self):
        walls_data = self.ctx.get("walls_data", [])
        if not walls_data:
            return

        print("[FSM Bootstrap] Select initial phase-1 target wall:")
        for i, wall in enumerate(walls_data, start=1):
            line = wall["scan_line"]
            print(f"  {i}: {line[0]} -> {line[1]}")

        wall_idx = self._prompt_int(">> Target wall index", 1, len(walls_data), 1) - 1
        endpoint = self._prompt_int(">> Target endpoint (1=start, 2=end)", 1, 2, 1) - 1
        self._set_phase1_target(wall_idx, walls_data[wall_idx]["scan_line"][endpoint])

    def _set_phase1_target(self, wall_idx: int, target_scan_point):
        """Point the phase-1 states at one wall and one of its scan-line ends."""
        walls_data = self.ctx.get("walls_data", [])
        target_scan_wall = walls_data[wall_idx]["scan_line"]
        self.ctx["current_wall_index"] = wall_idx
        self.ctx["target_scan_wall"] = target_scan_wall
        self.ctx["target_scan_point"] = target_scan_point
        self.ctx["current_wall_scan_lines"] = list(
            walls_data[wall_idx].get("scan_lines_z") or [target_scan_wall[0][2]]
        )
        self.ctx["current_line_idx"] = 0
        self.get_logger().info(
            f"[FSM] Phase-1 bootstrap target set: wall #{wall_idx}, point={target_scan_point}, "
            f"lines z={self.ctx['current_wall_scan_lines']}"
        )

    def _wait_for_base_pose(self, timeout_s: float):
        """Block until ``<world> -> <base frame>`` resolves, spinning the node so
        the TF listener actually receives anything. Returns ``(frame, (x, y, yaw))``
        or None on timeout. Only ever called from the bootstrap, before the FSM
        timer exists, so the spin services nothing but TF.

        The world frame is ``scan_world_frame`` -- ``map`` unless the run has
        opted out of localisation with ``odom``.
        """
        world = world_frame(self.ctx)
        primary = str(self.ctx.get("nav_base_frame", DEFAULT_NAV_BASE_FRAME))
        frames = (primary,) + tuple(f for f in BOOTSTRAP_BASE_FRAMES if f != primary)
        deadline = time.time() + float(timeout_s)
        announced = False
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            for frame in frames:
                try:
                    if not self.tf_buffer.can_transform(world, frame, rclpy.time.Time()):
                        continue
                    tf = self.tf_buffer.lookup_transform(world, frame, rclpy.time.Time())
                except Exception:
                    continue
                t, q = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                return frame, (float(t.x), float(t.y), yaw)
            if not announced:
                self.get_logger().info(
                    f"[FSM Bootstrap] Waiting (up to {timeout_s:.0f}s) for {world}->{primary} "
                    f"so the wall can be placed in front of the robot..."
                )
                announced = True
        return None

    def _bootstrap_wall_lines_z(self) -> List[float]:
        """Scan-line heights for an in-front wall: ``bootstrap_wall_lines_z``
        (a float or a list of floats, e.g. ``-p bootstrap_wall_lines_z:=[0.9,1.4]``)
        or, failing that, the same interactive prompt the YAML path uses.
        """
        raw = self.ctx.get("bootstrap_wall_lines_z")
        if raw is None:
            return self._prompt_wall_lines(1)
        values = [float(raw)] if isinstance(raw, (int, float)) else [float(v) for v in raw]
        if not values:
            return self._prompt_wall_lines(1)
        return sorted(values)

    def _wall_in_front_of_robot(self) -> Dict[str, Tuple]:
        """One wall placed where the robot is already looking (``--wall-source in-front``).

        The wall face is put ``partition_base_standoff_m`` ahead of the base along
        its heading, so the partition scan pose of the resulting line is the base
        pose itself and ScanWall has nothing to drive to. The line is
        ``bootstrap_wall_length_m`` long (default: one partition), centred on the
        robot. Which is also why ``scan_wall_assume_parked`` is switched on here:
        the geometry says the base is at its scan pose, and the knob stops the
        costmap from arguing.

        Needs the robot stack's TF, so the caller must have brought the stack up
        first. Raises on timeout rather than guessing a pose: a wall placed at the
        map origin would send the arm sweeping thin air.
        """
        timeout = float(self.ctx.get("bootstrap_tf_timeout_s", 120.0))
        found = self._wait_for_base_pose(timeout)
        if found is None:
            world = world_frame(self.ctx)
            raise RuntimeError(
                f"No {world}->base transform within {timeout:.0f}s; cannot place a wall "
                f"in front of the robot. Is the robot stack up"
                f"{' and localised' if world == 'map' else ''}?"
            )
        frame, (x, y, yaw) = found

        standoff = float(
            self.ctx.get("partition_base_standoff_m", DEFAULT_PARTITION_BASE_STANDOFF)
        )
        offset = float(self.ctx.get("wall_scan_line_offset_m", DEFAULT_SCAN_LINE_OFFSET))
        max_len = float(self.ctx.get("partition_max_length_m", 0.8))
        length = float(self.ctx.get("bootstrap_wall_length_m", max_len))
        lines_z = self._bootstrap_wall_lines_z()

        wall = build_wall_in_front(
            (x, y), yaw, standoff, length, offset=offset, scan_lines_z=lines_z
        )
        self.ctx.setdefault("scan_wall_assume_parked", True)
        if length > max_len + 1e-6:
            self.get_logger().warn(
                f"[FSM Bootstrap] bootstrap_wall_length_m {length:.2f} m exceeds one "
                f"partition ({max_len:.2f} m). The base will NOT move between "
                f"partitions (scan_wall_assume_parked); partitions beyond the arm's "
                f"reach are re-cut or skipped by the executor, not driven to."
            )
        line = wall["scan_line"]
        self.get_logger().info(
            f"[FSM Bootstrap] Wall placed in front of the robot: {frame} at "
            f"({x:.2f}, {y:.2f}, yaw {yaw:.2f} rad) in '{world_frame(self.ctx)}', "
            f"wall face assumed {standoff:.2f} m "
            f"ahead; scan line ({line[0][0]:.2f}, {line[0][1]:.2f}) -> "
            f"({line[1][0]:.2f}, {line[1][1]:.2f}), {length:.2f} m, heights {lines_z}. "
            f"The arm measures the true wall distance itself; only the heading and "
            f"the lateral placement are trusted from this pose."
        )
        return wall

    def _prompt_phase2_base(self):
        base_positions = self.ctx.get("optimal_base_results", {})
        if not base_positions:
            return

        completed = set(self.ctx.get("completed_base_indices", []))
        available = sorted(idx for idx in base_positions if idx not in completed)
        if not available:
            return

        print("[FSM Bootstrap] Available base indices:")
        for idx in available:
            print(f"  {idx}: {base_positions[idx]}")

        default_idx = available[0]
        while True:
            raw = self._safe_input(f">> Base index to start from [{default_idx}] ").strip()
            if not raw:
                selected_idx = default_idx
                break
            try:
                selected_idx = int(raw)
            except ValueError:
                print(f"Invalid base index: '{raw}'")
                continue
            if selected_idx not in available:
                print(f"Base index must be one of: {available}")
                continue
            break

        self.ctx["selected_base_idx"] = selected_idx
        self.ctx["selected_base"] = base_positions[selected_idx]
        self._ensure_reference_target_from_walls(selected_base=base_positions[selected_idx])
        self.get_logger().info(
            f"[FSM] Phase-2 bootstrap base set: idx={selected_idx}, pos={base_positions[selected_idx]}"
        )

    def _ensure_nav_sim_running(self):
        if self._stack_ensured:
            return
        self._stack_ensured = True
        p = self.ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            self.get_logger().info(f"[FSM] Navigation simulation already running (pid={p.pid}).")
            return

        sim_value = "true" if self.ctx.get("sim", False) else "false"
        planner_backend = str(self.ctx.get("planner_backend", "legacy")).strip().lower()
        try:
            if not self.launch_stack:
                self.get_logger().info(
                    "[FSM] --no-launch-stack: attaching to the robot stack already "
                    "running; waiting for it to be ready instead of launching it."
                )
            else:
                start_proc(
                    self.ctx,
                    "nav_sim",
                    [
                        "ros2",
                        "launch",
                        "navi_wall",
                        "move_robot.launch.py",
                        f"sim:={sim_value}",
                        "mode:=full",
                        "controller_type:=omni",
                        "database_name:=rtabmap_fsm",
                        "headless:=true",
                        f"use_sim_time:={sim_value}",
                        "hybrid_sim:=false",
                        f"planner_backend:={planner_backend}",
                        # This launch starts wall_detection_node itself, so it must
                        # write the very file GeometryReconstruction reads back.
                        f"wall_file_path:={self.ctx['geometry_reconstruction_wall_file_path']}",
                        # Push out launch's own SIGKILL deadline so ros2_control survives
                        # long enough to retract the column on shutdown.
                        *ROBOT_STACK_LAUNCH_SHUTDOWN_ARGS,
                    ],
                )
                time.sleep(2.0)
                self.get_logger().info("[FSM] Navigation + localization simulation started by bootstrap.")

            # Mirror ObjectID's readiness gates. When the FSM is started at a
            # non-initial state this bootstrap path replaces ObjectID, so without
            # these checks it would race ahead while the stack — in particular the
            # collision-checking service — is still coming up, and the planner
            # would silently plan without collision validation.
            # /clock is only published when sim time is running, so it must NOT
            # be required on the real robot (sim:=false) — otherwise
            # wait_stack_ready blocks on '/clock' forever and eventually times
            # out. Mirror ObjectID's conditional gating here.
            default_ready_topics = ["/tf", "/joint_states", "/rtabmap/odom"]
            if self.ctx.get("sim", False):
                default_ready_topics = ["/clock"] + default_ready_topics
            required_topics = self.ctx.get(
                "stack_ready_topics",
                default_ready_topics,
            )
            ready_timeout = float(self.ctx.get("stack_ready_timeout", STACK_READY_TIMEOUT_S))
            self.get_logger().info(
                f"[FSM] Waiting (up to {ready_timeout:.0f}s) for navigation + localization stack to become ready..."
            )
            if not wait_stack_ready(self.ctx, required_topics, timeout=ready_timeout):
                self.ctx["error_triggered"] = True
                self.get_logger().error(
                    "[FSM] Navigation + localization stack did not become ready during bootstrap."
                )
                return

            if planner_backend == "legacy":
                collision_services = self.ctx.get(
                    "collision_ready_services",
                    ["/collision/check_collision_pose"],
                )
                collision_timeout = float(
                    self.ctx.get("collision_ready_timeout", COLLISION_READY_TIMEOUT_S)
                )
                self.get_logger().info(
                    f"[FSM] Waiting (up to {collision_timeout:.0f}s) for collision checking service to come up..."
                )
                if not wait_services_ready(self.ctx, collision_services, timeout=collision_timeout):
                    self.ctx["error_triggered"] = True
                    self.get_logger().error(
                        "[FSM] Collision checking service did not come up during bootstrap; "
                        "the planner would run without collision validation."
                    )
                    return
        except Exception as exc:
            self.ctx["error_triggered"] = True
            self.get_logger().error(
                f"[FSM] Failed to start navigation + localization simulation during bootstrap: {exc}"
            )

    def _bootstrap_sensor_processing(self):
        """Point SensorDataProcessing at a recorded session and run it alone.

        The state reads everything from disk: the hyperspectral session
        (``hyperspectral_session_dir``), the GPR line manifest that shares its
        stamp, and whatever GP8800 exports sit in ``data/raw/gpr/incoming``.
        Unless a session is named explicitly (``-p hyperspectral_session_dir:=...``)
        the most recent one under ``data/raw/hyperspectral`` is taken, which is
        what "process what we just recorded" means after a bench run or a
        mission that was cut short. Results land in
        ``data/processed/session_<same stamp>/``.

        No robot: no stack is launched, and the run ends in Finished rather
        than folding an arm that was never unfolded. The legacy simulation mock
        is disabled so a real record is never replaced by the fake verdict.
        """
        explicit = self.ctx.get("hyperspectral_session_dir")
        if explicit:
            session = os.path.expanduser(str(explicit))
            if not os.path.isdir(session):
                self._abort_bootstrap(f"hyperspectral_session_dir '{session}' does not exist")
            self.get_logger().info(f"[FSM Bootstrap] Processing the session given: {session}")
        else:
            latest = sensor_paths.latest_raw_session_dir(self.ctx)
            if latest is None:
                self.get_logger().warn(
                    f"[FSM Bootstrap] No recorded hyperspectral session under "
                    f"{sensor_paths.raw_hyperspectral_root(self.ctx)}; only GPR exports "
                    f"in {sensor_paths.gpr_incoming_dir(self.ctx)} will be processed."
                )
            else:
                session = str(latest)
                self.ctx["hyperspectral_session_dir"] = session
                others = len(sensor_paths.raw_session_dirs(self.ctx)) - 1
                self.get_logger().info(
                    f"[FSM Bootstrap] Processing the latest recorded session: {session}"
                    + (f" ({others} older session(s) left alone; name one with "
                       f"-p hyperspectral_session_dir:=<dir> to process it instead)"
                       if others else "")
                )
        # Every wall in the record: there is no "wall just scanned" here.
        self.ctx.setdefault("current_wall_index", None)
        self.ctx.setdefault("sensor_processing_mock", False)
        self.ctx.setdefault("fsm_stop_after", "SensorDataProcessing")
        self.get_logger().info(
            f"[FSM Bootstrap] Offline run: no robot stack; the FSM finishes after "
            f"{self.ctx['fsm_stop_after']}."
        )

    def _abort_bootstrap(self, reason: str):
        """Stop whatever the bootstrap launched and refuse to start the machine."""
        self.get_logger().error(f"[FSM Bootstrap] {reason}; not starting the FSM.")
        try:
            stop_all(self.ctx)
        except Exception as exc:   # noqa: BLE001 - report, then still refuse
            self.get_logger().warn(f"[FSM Bootstrap] cleanup after the failure: {exc}")
        raise BootstrapError(reason)

    def _ensure_nav_client(self):
        if self.ctx.get("nav_client") is None:
            self.ctx["nav_client"] = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _bootstrap_context_for_initial_state(self, initial_state: str, scan_phase: Optional[int]):
        if initial_state not in FSM_STATE_ORDER:
            raise ValueError(
                f"Invalid initial state '{initial_state}'. Valid options: {', '.join(FSM_STATE_ORDER)}"
            )

        resolved_scan_phase = self._resolve_scan_phase(initial_state, scan_phase)
        self.ctx["scan_phase"] = resolved_scan_phase
        self.ctx["fsm_initial_state"] = initial_state

        # Defaults usually initialized in earlier states/subscriptions.
        self.ctx.setdefault("home_position", Point(x=0.0, y=0.0, z=0.0))
        self.ctx.setdefault("home_orientation", Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.ctx.setdefault("base_position", Point(x=0.0, y=0.0, z=0.0))
        self.ctx.setdefault("base_orientation", Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.ctx.setdefault("odom_received", False)
        self.ctx.setdefault("completed_base_indices", [])
        self.ctx.setdefault("base_recompute_retry_counts", {})
        self.ctx.setdefault("recompute_base_placement", False)
        self.ctx.setdefault("scan_done", False)
        self.ctx.setdefault("exhaustive_scan_done", False)
        self.ctx.setdefault("panels_left", 0)

        if initial_state == "Initialization":
            return

        self.get_logger().info(
            f"[FSM] Bootstrapping context for initial_state='{initial_state}', scan_phase={resolved_scan_phase}"
        )
        # Skip external start gate when starting from any non-initial state.
        self.ctx["start"] = True

        if initial_state in OFFLINE_INITIAL_STATES:
            self._bootstrap_sensor_processing()
            return

        in_front = self.wall_source == "in-front"
        if in_front and initial_state not in WALL_DATA_REQUIRED_INITIAL_STATES:
            raise ValueError(
                f"--wall-source in-front needs a wall-scanning initial state "
                f"({', '.join(sorted(WALL_DATA_REQUIRED_INITIAL_STATES))}), not '{initial_state}'."
            )
        if in_front and resolved_scan_phase != 1:
            raise ValueError("--wall-source in-front is a phase-1 (wall sweep) bootstrap.")

        if initial_state in WALL_DATA_REQUIRED_INITIAL_STATES:
            if in_front:
                # The wall is built from where the robot stands, and that pose
                # comes from the stack's TF -- so the stack goes up first here,
                # not at the end of the bootstrap as for a prompted wall.
                if initial_state in NAV_SIM_REQUIRED_START_STATES:
                    self._ensure_nav_sim_running()
                    if self.ctx.get("error_triggered"):
                        self._abort_bootstrap("the robot stack did not come up")
                try:
                    walls_data = [self._wall_in_front_of_robot()]
                except RuntimeError as exc:
                    self._abort_bootstrap(str(exc))
            else:
                walls_data = self._prompt_walls_data()
            self.ctx["walls_data"] = walls_data
            self.ctx["wall_inward_normals"] = [w["inward_normal"] for w in walls_data]
            self.ctx["wall_ee_rpy_deg"] = [w["ee_rpy_deg"] for w in walls_data]
            self.ctx["database_generated"] = bool(walls_data)
            if resolved_scan_phase == 1:
                self.ctx["walls_left"] = len(walls_data)
                self.ctx["scan_done"] = False
            else:
                self.ctx["walls_left"] = 0
                self.ctx["scan_done"] = True
            self.ctx.setdefault("aoi_data", walls_data)
            self.ctx.setdefault("object_id_data", walls_data)

        # For phase-2 starts that skip WallDiscretization/BasePlacement,
        # create synthetic context to keep following states runnable.
        needs_synthetic_discretization = initial_state in {
            "BasePlacement",
            "ExhaustiveScan",
            "HomePosition",
        } or (
            resolved_scan_phase == 2
            and initial_state in NEEDS_SYNTHETIC_DISCRETIZATION_INITIAL_STATES
        )
        if needs_synthetic_discretization:
            wall_discretization_results = self._generate_mock_discretization(
                self.ctx.get("walls_data", [])
            )
            self.ctx["wall_discretization_results"] = wall_discretization_results
            base_to_panel_indices, optimal_base_results = self._generate_mock_base_data(
                wall_discretization_results
            )
            self.ctx["base_to_panel_indices"] = base_to_panel_indices
            self.ctx["optimal_base_results"] = optimal_base_results
            completed = set(self.ctx.get("completed_base_indices", []))
            self.ctx["panels_left"] = max(0, len(optimal_base_results) - len(completed))

        if resolved_scan_phase == 1 and initial_state in PHASE1_TARGET_REQUIRED_INITIAL_STATES:
            if in_front:
                # One wall, swept from the robot's left end -- the same end
                # WallTargetSelection would pick.
                wall = self.ctx["walls_data"][0]
                self._set_phase1_target(
                    0, left_scan_endpoint(wall["scan_line"], wall.get("inward_normal"))
                )
            else:
                self._prompt_phase1_target()
        if resolved_scan_phase == 2:
            self.ctx["scan_done"] = True
            self.ctx.setdefault("aoi_data", self.ctx.get("walls_data", []))
            if initial_state in PHASE2_BASE_REQUIRED_INITIAL_STATES:
                self._prompt_phase2_base()
            self._ensure_reference_target_from_walls(self.ctx.get("selected_base"))

        if initial_state in NAV_SIM_REQUIRED_START_STATES:
            self._ensure_nav_sim_running()
        if initial_state in NAV_CLIENT_BOOTSTRAP_STATES:
            self._ensure_nav_client()

    def publish_fsm_current(self, state_name: str):
        msg = String()
        msg.data = state_name
        self.fsm_current_pub.publish(msg)

    def _publish_json(self, publisher, payload: Dict):
        msg = String()
        msg.data = json.dumps(make_json_safe(payload), sort_keys=True)
        publisher.publish(msg)

    def publish_fsm_transition(self, from_state: str, to_state: str, reason: str = ""):
        payload = {
            "from": from_state,
            "to": to_state,
            "reason": reason,
        }
        self._publish_json(self.fsm_transition_pub, payload)

    def publish_fsm_graph(self):
        payload = build_fsm_graph_payload(FSM_STATE_ORDER)
        payload["initial_state"] = self.initial_state
        self._publish_json(self.fsm_graph_pub, payload)

    def set_fsm_status(
        self,
        state_name: Optional[str] = None,
        *,
        phase: Optional[str] = None,
        summary: Optional[str] = None,
        data: Optional[Dict] = None,
        progress_current: Optional[int] = None,
        progress_total: Optional[int] = None,
        level: Optional[str] = None,
    ) -> Dict:
        snapshot = self.ctx.setdefault("_fsm_status", {})

        if state_name is None:
            if hasattr(self, "machine") and getattr(self.machine, "current_state", None):
                state_name = self.machine.current_state.name
            else:
                state_name = self.initial_state

        snapshot["state"] = state_name
        if phase is not None:
            snapshot["phase"] = phase
        if summary is not None:
            snapshot["summary"] = summary
        if level is not None:
            snapshot["level"] = level
        if data is not None:
            snapshot["data"] = make_json_safe(data)
        if progress_current is not None or progress_total is not None:
            snapshot["progress"] = {
                "current": make_json_safe(progress_current),
                "total": make_json_safe(progress_total),
            }

        return snapshot

    def publish_fsm_status(self, snapshot: Dict):
        self._publish_json(self.fsm_status_pub, snapshot)

    def publish_fsm_event(
        self,
        event_type: str,
        *,
        state_name: Optional[str] = None,
        summary: str = "",
        details: Optional[Dict] = None,
        level: str = "info",
    ):
        if state_name is None:
            if hasattr(self, "machine") and getattr(self.machine, "current_state", None):
                state_name = self.machine.current_state.name
            else:
                state_name = self.initial_state

        stamp = self.get_clock().now().nanoseconds / 1e9
        payload = {
            "state": state_name,
            "event": event_type,
            "summary": summary,
            "details": make_json_safe(details or {}),
            "level": level,
            "stamp": stamp,
        }
        self._publish_json(self.fsm_event_pub, payload)

    def start_callback(self, msg: Bool):
        self.ctx["start"] = msg.data
        self.get_logger().info(f"[ROS] /start_flag = {msg.data}")

    def odometry_callback(self, msg: Odometry):
        self.ctx["base_position"] = msg.pose.pose.position
        self.ctx["base_orientation"] = msg.pose.pose.orientation
        self.ctx["odom_received"] = True

    def global_costmap_callback(self, msg: OccupancyGrid):
        first = self.ctx.get("global_costmap") is None
        self.ctx["global_costmap"] = msg
        if first:
            info = msg.info
            self.get_logger().info(
                f"[FSM] Global costmap received: {info.width}x{info.height} @ "
                f"{info.resolution:.3f} m, origin=({info.origin.position.x:.2f}, "
                f"{info.origin.position.y:.2f})."
            )

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

        # Track column position in context for ExhaustiveScan state
        column_joint_name = "column_joint"
        try:
            idx = msg.name.index(column_joint_name)
            if idx < len(msg.position):
                self.ctx["column_current_height"] = float(msg.position[idx])
        except (ValueError, IndexError):
            # Column joint not in this message
            pass

    def execution_status_callback(self, msg):
        self.ctx["execution_status"] = msg.data

    def planner_goal_failed_callback(self, msg: Bool):
        self.ctx["planner_goal_failed"] = msg.data

    def ft_data_callback(self, msg: WrenchStamped):
        # Latest TCP wrench (tool0 frame). ScanWall reads ctx["ft_wrench"].force.z
        # to know when the GPR wheel is pressed against the wall.
        self.ctx["ft_wrench"] = msg.wrench

    def distance_sensors_callback(self, msg: Float32MultiArray):
        # Latest plate ranges + arrival time. ScanWall averages the valid ones to
        # decide how far the arm must travel along its Z axis to sit at the
        # commanded standoff from the wall; the timestamp lets it reject a frame
        # left over from a reader that has since been stopped.
        if len(msg.data) == 6:
            self.ctx["plate_distances"] = [float(v) for v in msg.data]
            self.ctx["plate_distances_stamp"] = time.time()

    def gpr_trigger_bridge_status_callback(self, msg: String):
        # Latest bridge snapshot + arrival time; the stamp lets ScanWall tell a
        # bridge that died from one that is merely reporting alive=false.
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(status, dict):
            self.ctx["gpr_trigger_bridge_status"] = status
            self.ctx["gpr_trigger_bridge_status_stamp"] = time.time()

    def mapping_callback(self, msg):
        self.ctx["map_ready"] = msg.data

    def parking_active_callback(self, msg: Bool):
        # Latched chassis-parking flag from sim_controller. ScanWall waits for the
        # active->inactive transition to know the base has aligned with the turret.
        self.ctx["parking_active"] = bool(msg.data)


def main(args=None):
    # Parse custom arguments before initializing rclpy
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--sim",
        type=str,
        default="false",
        choices=["true", "false"],
        help="Enable simulation mode (true/false).",
    )
    parser.add_argument(
        "--initial-state",
        type=str,
        default="Initialization",
        choices=FSM_STATE_ORDER,
        help="State where the FSM should start.",
    )
    parser.add_argument(
        "--scan-phase",
        type=int,
        default=None,
        choices=[1, 2],
        help="Force scan phase when bootstrapping from a non-default initial state.",
    )
    parser.add_argument(
        "--planner-backend",
        type=str,
        default="legacy",
        choices=["legacy", "moveit"],
        help="Planner backend to use when the FSM launches move_robot.launch.py.",
    )
    parser.add_argument(
        "--wall-source",
        type=str,
        default="yaml",
        choices=list(WALL_SOURCES),
        help="Where a bootstrapped wall-scanning start gets its wall: 'yaml' "
             "(pick from detected_walls.yaml, the default) or 'in-front' (one "
             "short wall synthesised from where the robot stands, base assumed "
             "parked at its scan pose; ctx knobs bootstrap_wall_length_m, "
             "bootstrap_wall_lines_z).",
    )
    parser.add_argument(
        "--no-launch-stack",
        action="store_true",
        help="Do not launch move_robot.launch.py during the bootstrap; the robot "
             "stack is already running. The bootstrap still waits for it to be ready.",
    )
    parser.add_argument(
        "--stop-after",
        type=str,
        default=None,
        choices=FSM_STATE_ORDER,
        help="Last state to run: its onward transition goes to Finished instead "
             "(bench runs that must not carry on to HomePosition).",
    )

    # Use sys.argv if args is None
    argv = args if args is not None else sys.argv[1:]
    parsed_args, remaining_args = parser.parse_known_args(argv)
    sim = parsed_args.sim.lower() == "true"

    # Initialize rclpy with remaining args (ROS-specific arguments)
    rclpy.init(args=remaining_args)

    try:
        node = RobotFSMNode(
            sim=sim,
            initial_state=parsed_args.initial_state,
            scan_phase=parsed_args.scan_phase,
            planner_backend=parsed_args.planner_backend,
            wall_source=parsed_args.wall_source,
            launch_stack=not parsed_args.no_launch_stack,
            stop_after=parsed_args.stop_after,
        )
    except BootstrapError as exc:
        # Already logged and cleaned up by the bootstrap; exit without a
        # traceback so the cause is the last line on the terminal.
        rclpy.shutdown()
        sys.exit(f"[FSM] bootstrap failed: {exc}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[FSM] Ctrl+C received, killing processes...")
        processes_to_kill = ["gz sim", "ign gazebo", "ruby.*gz", "gzserver", "gz-sim"]
        for process in processes_to_kill:
            subprocess.run(["pkill", "-9", "-f", process], timeout=2, stderr=subprocess.DEVNULL)
    finally:
        try:
            stop_all(node.ctx)
        except Exception as e:
            node.get_logger().warn(f"[FSM] stop_all falló: {e}")
        node.destroy_node()
        rclpy.shutdown()
