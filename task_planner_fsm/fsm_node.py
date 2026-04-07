import argparse
import json
import math
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
import tf2_ros
from rclpy.duration import Duration
from std_msgs.msg import Bool, String

from task_planner_fsm.machine import StateMachine
from task_planner_fsm.states import (
    AreasOfInterest,
    ArmFolding,
    ArmUnfolding,
    BasePlacement,
    ComputeWallPoints,
    CreateMap,
    Error,
    ExhaustiveScan,
    Finished,
    GeometryReconstruction,
    HomePosition,
    Initialization,
    NavigateToTarget,
    ObjectID,
    ScanWall,
    WallDiscretization,
    WallTargetSelection,
)
from task_planner_fsm.states.proc_utils import start_proc, stop_all

FSM_STATE_ORDER = [
    "Initialization",
    "CreateMap",
    "ObjectID",
    "GeometryReconstruction",
    "ComputeWallPoints",
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
    "Finished",
    "Error",
]

PHASE2_DEFAULT_INITIAL_STATES = {
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
    "Finished",
}

WALL_DATA_REQUIRED_INITIAL_STATES = {
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
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
    "ExhaustiveScan",
}

NEEDS_SYNTHETIC_DISCRETIZATION_INITIAL_STATES = {
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
}

NAV_CLIENT_BOOTSTRAP_STATES = {
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
}

NAV_SIM_REQUIRED_START_STATES = {
    s
    for s in FSM_STATE_ORDER[FSM_STATE_ORDER.index("ComputeWallPoints") :]
    if s not in {"Finished", "Error"}
}

PREDEFINED_WALLS = [
    ((3.0, 0.0, 2.0), (3.0, -3.0, 3.0)),
    ((9.0, 0.0, 0.19), (9.0, -4.5, 2.0)),
    ((10.0, 0.0, 0.2), (10.0, -4.5, 3.0)),
]


class RobotFSMNode(Node):
    def __init__(
        self,
        sim: bool = False,
        initial_state: str = "Initialization",
        scan_phase: Optional[int] = None,
        planner_backend: str = "legacy",
    ):
        super().__init__("robot_fsm_node")
        self.initial_state = initial_state
        self._stdin_warned = False
        self.planner_backend = str(planner_backend).strip().lower()

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

        self.fsm_current_pub = self.create_publisher(String, "/fsm/current_state", current_qos)
        self.fsm_transition_pub = self.create_publisher(String, "/fsm/transition", transition_qos)

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
            "publish_fsm_current": self.publish_fsm_current,
            "publish_fsm_transition": self.publish_fsm_transition,
            "tf_buffer": self.tf_buffer,
        }

        # Build test context for non-default initial state.
        self._bootstrap_context_for_initial_state(initial_state, scan_phase)

        # FSM
        self.machine = StateMachine(
            [
                Initialization("Initialization"),
                CreateMap("CreateMap"),
                ObjectID("ObjectID"),
                GeometryReconstruction("GeometryReconstruction"),
                ComputeWallPoints("ComputeWallPoints"),
                WallTargetSelection("WallTargetSelection"),
                NavigateToTarget("NavigateToTarget"),
                ArmUnfolding("ArmUnfolding"),
                ArmFolding("ArmFolding"),
                ScanWall("ScanWall"),
                AreasOfInterest("AreasOfInterest"),
                WallDiscretization("WallDiscretization"),
                BasePlacement("BasePlacement"),
                ExhaustiveScan("ExhaustiveScan"),
                HomePosition("HomePosition"),
                Finished("Finished"),
                Error("Error"),
            ],
            initial_state=initial_state,
            ctx=self.ctx,
        )

        # Subscriptions
        self.create_subscription(Bool, "/start_flag", self.start_callback, 10)
        self.create_subscription(Odometry, "/rtabmap/odom", self.odometry_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Bool, "/execution_status", self.execution_status_callback, 10)
        self.create_subscription(Bool, "/arm/execution_status", self.execution_status_callback, 10)
        self.create_subscription(Bool, "/planner/goal_failed", self.planner_goal_failed_callback, 10)
        self.create_subscription(Bool, "/map_done", self.mapping_callback, 10)

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
        self, p1: Tuple[float, float, float], p2: Tuple[float, float, float], offset: float = 0.6
    ) -> Dict[str, Tuple]:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length == 0:
            raise ValueError("Wall points must be different.")

        dx /= length
        dy /= length

        # Clock-wise normal for the exterior scan side.
        nx = dy
        ny = -dx

        scan_start = (
            p1[0] + offset * dx + nx * offset,
            p1[1] + offset * dy + ny * offset,
            p1[2],
        )
        scan_end = (
            p2[0] - offset * dx + nx * offset,
            p2[1] - offset * dy + ny * offset,
            p2[2],
        )

        return {
            "original": (tuple(p1), tuple(p2)),
            "scan_line": (scan_start, scan_end),
        }

    def _prompt_walls_data(self) -> List[Dict[str, Tuple]]:
        print("[FSM Bootstrap] Available predefined walls:")
        for i, (p1, p2) in enumerate(PREDEFINED_WALLS, start=1):
            print(f"  {i}: p1={p1}, p2={p2}")

        max_walls = len(PREDEFINED_WALLS)
        num_walls = self._prompt_int(
            ">> Number of walls to include for this run (1-3)", 1, max_walls, max_walls
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
                p1, p2 = PREDEFINED_WALLS[idx - 1]
                walls_data.append(self._build_wall_data(p1, p2))
            return walls_data

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
    ) -> Tuple[Optional[Tuple], Optional[Tuple]]:
        min_dist = float("inf")
        selected_line = None
        selected_point = None

        for wall in walls_data:
            scan_line = wall.get("scan_line")
            if not scan_line or len(scan_line) != 2:
                continue
            for pt in scan_line:
                dist = ((point_xy[0] - pt[0]) ** 2 + (point_xy[1] - pt[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    selected_line = scan_line
                    selected_point = pt

        return selected_line, selected_point

    def _ensure_reference_target_from_walls(self, selected_base: Optional[Tuple[float, float]] = None):
        if self.ctx.get("target_scan_wall") and self.ctx.get("target_scan_point"):
            return

        walls_data = self.ctx.get("walls_data", [])
        if not walls_data:
            return

        selected_line = None
        selected_point = None
        if selected_base is not None:
            selected_line, selected_point = self._closest_scan_line_to_point(walls_data, selected_base)

        if selected_line is None or selected_point is None:
            selected_line = walls_data[0].get("scan_line")
            if selected_line and len(selected_line) == 2:
                selected_point = selected_line[0]

        if selected_line and selected_point:
            self.ctx["target_scan_wall"] = selected_line
            self.ctx["target_scan_point"] = selected_point

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

        target_scan_wall = walls_data[wall_idx]["scan_line"]
        target_scan_point = target_scan_wall[endpoint]
        self.ctx["current_wall_index"] = wall_idx
        self.ctx["target_scan_wall"] = target_scan_wall
        self.ctx["target_scan_point"] = target_scan_point
        self.get_logger().info(
            f"[FSM] Phase-1 bootstrap target set: wall #{wall_idx}, point={target_scan_point}"
        )

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
        p = self.ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            self.get_logger().info(f"[FSM] Navigation simulation already running (pid={p.pid}).")
            return

        sim_value = "true" if self.ctx.get("sim", False) else "false"
        planner_backend = str(self.ctx.get("planner_backend", "legacy")).strip().lower()
        try:
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
                    "use_sim_time:=true",
                    "hybrid_sim:=true",
                    f"planner_backend:={planner_backend}",
                ],
            )
            time.sleep(2.0)
            self.get_logger().info("[FSM] Navigation + localization simulation started by bootstrap.")
        except Exception as exc:
            self.ctx["error_triggered"] = True
            self.get_logger().error(
                f"[FSM] Failed to start navigation + localization simulation during bootstrap: {exc}"
            )

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

        if initial_state in WALL_DATA_REQUIRED_INITIAL_STATES:
            walls_data = self._prompt_walls_data()
            self.ctx["walls_data"] = walls_data
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

    def publish_fsm_transition(self, from_state: str, to_state: str, reason: str = ""):
        payload = {
            "from": from_state,
            "to": to_state,
            "reason": reason,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.fsm_transition_pub.publish(msg)

    def start_callback(self, msg: Bool):
        self.ctx["start"] = msg.data
        self.get_logger().info(f"[ROS] /start_flag = {msg.data}")

    def odometry_callback(self, msg: Odometry):
        self.ctx["base_position"] = msg.pose.pose.position
        self.ctx["base_orientation"] = msg.pose.pose.orientation
        self.ctx["odom_received"] = True

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

    def mapping_callback(self, msg):
        self.ctx["map_ready"] = msg.data


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

    # Use sys.argv if args is None
    argv = args if args is not None else sys.argv[1:]
    parsed_args, remaining_args = parser.parse_known_args(argv)
    sim = parsed_args.sim.lower() == "true"

    # Initialize rclpy with remaining args (ROS-specific arguments)
    rclpy.init(args=remaining_args)

    node = RobotFSMNode(
        sim=sim,
        initial_state=parsed_args.initial_state,
        scan_phase=parsed_args.scan_phase,
        planner_backend=parsed_args.planner_backend,
    )
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
