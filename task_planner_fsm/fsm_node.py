import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from task_planner_fsm.machine import StateMachine
from task_planner_fsm.states import Initialization, CreateMap, ComputeWallPoints, WallTargetSelection, NavigateToTarget, ScanWall, HomePosition, Finished, Error

class RobotFSMNode(Node):
    def __init__(self):
        super().__init__('robot_fsm_node')

        # Contexto compartido para FSM
        self.ctx = {
            "node": self,
            "start": False,
            "map_ready": False,
            "error_triggered": False,
            "last_state": None,
        }

        # FSM
        self.machine = StateMachine([
            Initialization("Initialization"),
            CreateMap("CreateMap"),
            ComputeWallPoints("ComputeWallPoints"),
            WallTargetSelection("WallTargetSelection"),
            NavigateToTarget("NavigateToTarget"),
            ScanWall("ScanWall"),
            HomePosition("HomePosition"),
            Finished("Finished"),
            Error("Error"),
        ], initial_state="Initialization", ctx=self.ctx)

        # Suscripción a orden externa
        self.subscription = self.create_subscription(
            Bool,
            "/start_flag",
            self.start_callback,
            10
        )

        # Action client
        self.ctx["nav_client"] = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        if not self.ctx["nav_client"].wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action server not available after 10 seconds.")
            self.ctx["error_triggered"] = True

        # Timer para avanzar la FSM
        self.timer = self.create_timer(1.0, self.machine.step)

    def start_callback(self, msg: Bool):
        self.ctx["start"] = msg.data
        self.get_logger().info(f"[ROS] /start_flag = {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotFSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
