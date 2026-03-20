import argparse
import json
import sys
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros
from rclpy.duration import Duration

from task_planner_fsm.machine import StateMachine
from task_planner_fsm.states import Initialization, CreateMap, ObjectID, GeometryReconstruction, ComputeWallPoints, WallTargetSelection, NavigateToTarget
from task_planner_fsm.states import ArmUnfolding, ArmFolding, ScanWall, AreasOfInterest, WallDiscretization, BasePlacement, ExhaustiveScan, HomePosition, Finished, Error

from task_planner_fsm.states.proc_utils import stop_all

class RobotFSMNode(Node):
    def __init__(self, sim=False):
        super().__init__('robot_fsm_node')

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
            "publish_fsm_current": self.publish_fsm_current,
            "publish_fsm_transition": self.publish_fsm_transition,
            "tf_buffer": self.tf_buffer,
        }

        # FSM
        self.machine = StateMachine([
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
        ], initial_state="Initialization", ctx=self.ctx)

        # Subscriptions
        self.create_subscription(Bool, "/start_flag", self.start_callback, 10)
        self.create_subscription(Odometry, "/rtabmap/odom", self.odometry_callback, 10)        
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Bool, "/arm/execution_status", self.execution_status_callback, 10)
        self.create_subscription(Bool, "/planner/goal_failed", self.planner_goal_failed_callback, 10)
        self.create_subscription(Bool, "/map_done", self.mapping_callback, 10)       

        # Timer
        self.timer = self.create_timer(1.0, self.machine.step)
        self.get_logger().info(f"[FSM] Simulation mode: {self.ctx['sim']}")

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
    
    # Use sys.argv if args is None
    argv = args if args is not None else sys.argv[1:]
    parsed_args, remaining_args = parser.parse_known_args(argv)
    sim = parsed_args.sim.lower() == "true"
    
    # Initialize rclpy with remaining args (ROS-specific arguments)
    rclpy.init(args=remaining_args)

    node = RobotFSMNode(sim=sim)
    # rclpy.spin(node)   
    # node.destroy_node()
    # rclpy.shutdown()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[FSM] Ctrl+C received, killing processes...")
        processes_to_kill = ['gz sim', 'ign gazebo', 'ruby.*gz', 'gzserver', 'gz-sim']
        for process in processes_to_kill:
            subprocess.run(['pkill', '-9', '-f', process], timeout=2, stderr=subprocess.DEVNULL)
    finally:
        try:
            stop_all(node.ctx)
        except Exception as e:
            node.get_logger().warn(f"[FSM] stop_all falló: {e}")
        node.destroy_node()
        rclpy.shutdown()
