import argparse
import json
import sys
from typing import Dict, Optional

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from task_planner_fsm.machine import StateMachine
from task_planner_fsm.states import (
    Initialization,
    ReceiveNav2Map,
    GetSemanticMap,
    WaitForData,
    TargetSelection,
    ManipulatorFolding,
    BasePlacementComputation,
    NavigateToTarget,
    ManipulatorReachability,
    NearbyPointSelection,
    ManipulatorUnfolding,
    DrillApproach,
    SuctionDrillStart,
    Drilling,
    TakeOutDrill,
    SuctionDrillStop,
    DrillRetract,
    SampleScanning,
    StoringToDatabase,
    HomePosition,
    Finished,
    Error,
)
from task_planner_fsm.states.proc_utils import stop_all
from task_planner_fsm.telemetry import build_fsm_graph_payload, make_json_safe

# Single, linear pokeye workflow. There are no scan phases: the FSM walks these
# states in order, looping back for each drill target, until HomePosition.
FSM_STATE_ORDER = [
    "Initialization",
    "ReceiveNav2Map",
    "GetSemanticMap",
    "WaitForData",
    "TargetSelection",
    "ManipulatorFolding",
    "BasePlacementComputation",
    "NavigateToTarget",
    "ManipulatorReachability",
    "NearbyPointSelection",
    "ManipulatorUnfolding",
    "DrillApproach",
    "SuctionDrillStart",
    "Drilling",
    "TakeOutDrill",
    "SuctionDrillStop",
    "DrillRetract",
    "SampleScanning",
    "StoringToDatabase",
    "HomePosition",
    "Finished",
    "Error",
]


class RobotFSMNode(Node):
    def __init__(self, sim: bool = False, initial_state: str = "Initialization"):
        # Auto-declare any parameter passed as an override (e.g. via
        # `--ros-args -p some_knob:=value` or a launch file), so per-state tuning
        # knobs read from ctx below can be set without a declare_parameter call.
        super().__init__(
            "robot_fsm_node",
            automatically_declare_parameters_from_overrides=True,
        )
        if initial_state not in FSM_STATE_ORDER:
            raise ValueError(
                f"Invalid initial state '{initial_state}'. "
                f"Valid options: {', '.join(FSM_STATE_ORDER)}"
            )
        self.initial_state = initial_state

        # NOTE: Do NOT set use_sim_time=True here!
        # The FSM timer must run on wall time even in simulation mode,
        # otherwise it blocks waiting for /clock before simulation starts.

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
        self.drill_markers_pub = self.create_publisher(
            MarkerArray, "/fsm/drill_points", current_qos
        )

        # TF buffer + listener, shared with states via ctx["tf_buffer"] (the key
        # coverage_driver already expects). Used by ManipulatorUnfolding to turn a
        # map-frame drill point into gantry (turret-frame) joint targets.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Shared context for the FSM. This is intentionally minimal for the first
        # workflow-only version; each state reads/writes its own flags via
        # ctx.get(...)/ctx[...] as needed.
        self.ctx = {
            "node": self,
            "start": False,
            "error_triggered": False,
            "last_state": None,
            "sim": bool(sim),
            "tf_buffer": self.tf_buffer,
            # TargetSelection still branches on scan_phase (legacy remnant). Phases
            # are no longer a concept in this FSM; default to 1 so it runs the
            # single-pass target-selection path.
            "scan_phase": 1,
            "base_position": None,
            "_fsm_status": {},
            "publish_fsm_current": self.publish_fsm_current,
            "publish_fsm_transition": self.publish_fsm_transition,
            "publish_fsm_graph": self.publish_fsm_graph,
            "set_fsm_status": self.set_fsm_status,
            "publish_fsm_status": self.publish_fsm_status,
            "publish_fsm_event": self.publish_fsm_event,
            "publish_drill_markers": self.publish_drill_markers,
        }

        # Bridge externally-set ROS parameter overrides into ctx so per-state
        # tuning knobs can be configured at launch. setdefault keeps the explicit
        # ctx values above authoritative (a param cannot clobber e.g. "node"/"sim").
        param_overrides = self.get_parameters_by_prefix("")
        for pname, param in param_overrides.items():
            if pname == "use_sim_time":
                continue
            self.ctx.setdefault(pname, param.value)
            self.get_logger().info(f"[FSM] ctx param override: {pname}={param.value!r}")

        # FSM
        self.machine = StateMachine(
            [
                Initialization("Initialization"),
                ReceiveNav2Map("ReceiveNav2Map"),
                GetSemanticMap("GetSemanticMap"),
                WaitForData("WaitForData"),
                TargetSelection("TargetSelection"),
                ManipulatorFolding("ManipulatorFolding"),
                BasePlacementComputation("BasePlacementComputation"),
                NavigateToTarget("NavigateToTarget"),
                ManipulatorReachability("ManipulatorReachability"),
                NearbyPointSelection("NearbyPointSelection"),
                ManipulatorUnfolding("ManipulatorUnfolding"),
                DrillApproach("DrillApproach"),
                SuctionDrillStart("SuctionDrillStart"),
                Drilling("Drilling"),
                TakeOutDrill("TakeOutDrill"),
                SuctionDrillStop("SuctionDrillStop"),
                DrillRetract("DrillRetract"),
                SampleScanning("SampleScanning"),
                StoringToDatabase("StoringToDatabase"),
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

        # Timer
        self.timer = self.create_timer(1.0, self.machine.step)
        self.get_logger().info(f"[FSM] Simulation mode: {self.ctx['sim']}")
        self.get_logger().info(f"[FSM] Initial state: {initial_state}")

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

    def publish_drill_markers(self, ctx: Dict) -> None:
        """Publish a MarkerArray to /fsm/drill_points showing all drill locations.

        Colors:
          - pending   : gray
          - current   : yellow (just selected)
          - completed : green
        """
        locations = ctx.get("drill_locations") or []
        if not locations:
            return

        map_frame = str(ctx.get("map_frame", "map"))
        current_idx = ctx.get("current_drill_index", -1)
        completed = set(ctx.get("completed_drill_indices") or [])
        stamp = self.get_clock().now().to_msg()

        array = MarkerArray()

        # Clear all previously published markers in these namespaces.
        for ns in ("drill_sphere", "drill_label"):
            delete = Marker()
            delete.header.frame_id = map_frame
            delete.header.stamp = stamp
            delete.ns = ns
            delete.action = Marker.DELETEALL
            array.markers.append(delete)

        for idx, loc in enumerate(locations):
            is_current = (idx == current_idx)
            is_done = (idx in completed) and not is_current

            if is_current:
                r, g, b, a = 1.0, 0.9, 0.0, 1.0   # yellow
            elif is_done:
                r, g, b, a = 0.2, 0.9, 0.2, 0.8   # green
            else:
                r, g, b, a = 0.6, 0.6, 0.6, 0.7   # gray

            sphere = Marker()
            sphere.header.frame_id = map_frame
            sphere.header.stamp = stamp
            sphere.ns = "drill_sphere"
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(loc[0])
            sphere.pose.position.y = float(loc[1])
            sphere.pose.position.z = float(loc[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, a
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = map_frame
            label.header.stamp = stamp
            label.ns = "drill_label"
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(loc[0])
            label.pose.position.y = float(loc[1])
            label.pose.position.z = float(loc[2]) + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = str(idx)
            array.markers.append(label)

        self.drill_markers_pub.publish(array)

    def start_callback(self, msg: Bool):
        self.ctx["start"] = msg.data
        self.get_logger().info(f"[ROS] /start_flag = {msg.data}")

    def odometry_callback(self, msg: Odometry):
        self.ctx["base_position"] = msg.pose.pose.position
        self.ctx["base_orientation"] = msg.pose.pose.orientation
        self.ctx["odom_received"] = True


def main(args=None):
    # Under `ros2 run` / `ros2 launch` stdout is a pipe, so Python block-buffers
    # it and plain print() output only appears once several KB have piled up.
    # Line buffering makes it show up as it happens, next to the logger output.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass  # not a regular text stream (e.g. already wrapped); nothing to do

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

    # Use sys.argv if args is None
    argv = args if args is not None else sys.argv[1:]
    parsed_args, remaining_args = parser.parse_known_args(argv)
    sim = parsed_args.sim.lower() == "true"

    # Initialize rclpy with remaining args (ROS-specific arguments)
    rclpy.init(args=remaining_args)

    node = RobotFSMNode(sim=sim, initial_state=parsed_args.initial_state)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[FSM] Ctrl+C received, shutting down...")
    finally:
        try:
            stop_all(node.ctx)
        except Exception as e:
            node.get_logger().warn(f"[FSM] stop_all failed: {e}")
        node.destroy_node()
        rclpy.shutdown()
