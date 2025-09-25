import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from task_planner_fsm.machine import StateMachine
from task_planner_fsm.states import Initialization, CreateMap, GeometryReconstruction, ComputeWallPoints, WallTargetSelection, NavigateToTarget
from task_planner_fsm.states import ArmUnfolding, ArmFolding, ScanWall, AreasOfInterest, WallDiscretization, BasePlacement, ExhaustiveScan, HomePosition, Finished, Error

class RobotFSMNode(Node):
    def __init__(self):
        super().__init__('robot_fsm_node')

        # Shared context for the FSM
        self.ctx = {
            "node": self,
            "start": False,
            "map_ready": False,
            "error_triggered": False,
            "last_state": None,
            "scan_phase": 1,
            "execution_status": False,            
        }

        # FSM
        self.machine = StateMachine([
            Initialization("Initialization"),
            CreateMap("CreateMap"),
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
        self.create_subscription(JointState, "/arm/joint_states", self.joint_state_callback, 10)        # if no namespace is needed, erase "arm/" in both
        self.create_subscription(Bool, "/arm/execution_status", self.execution_status_callback, 10)     # if no namespace is needed, erase "arm/" in both
        self.create_subscription(Bool, "/map_done", self.mapping_callback, 10)       

        # Action clients
        # self.ctx["nav_client"] = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # if not self.ctx["nav_client"].wait_for_server(timeout_sec=10.0):
        #     self.get_logger().error("NavigateToPose action server not available after 10 seconds.")
        #     self.ctx["error_triggered"] = True

        # self.ctx["manipulator_client"] = ActionClient(self, FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        # if not self.ctx["manipulator_client"].wait_for_server(timeout_sec=10.0):
        #     self.get_logger().error("ManipulatorControl action server not available after 10 seconds.")
        #     self.ctx["error_triggered"] = True

        # Timer
        self.timer = self.create_timer(1.0, self.machine.step)

    def start_callback(self, msg: Bool):
        self.ctx["start"] = msg.data
        self.get_logger().info(f"[ROS] /start_flag = {msg.data}")    

    def odometry_callback(self, msg: Odometry):
        self.ctx["base_position"] = msg.pose.pose.position
        self.ctx["base_orientation"] = msg.pose.pose.orientation
        self.ctx["odom_received"] = True

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def execution_status_callback(self, msg):
        self.ctx["execution_status"] = msg.data

    def mapping_callback(self, msg):
        self.ctx["map_ready"] = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = RobotFSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
