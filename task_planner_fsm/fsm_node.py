import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from task_planner_fsm.machine import StateMachine
from task_planner_fsm.states import Initialization, CreateMap, Error

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
            Error("Error"),
        ], initial_state="Initialization", ctx=self.ctx)

        # Suscripción a orden externa
        self.subscription = self.create_subscription(
            Bool,
            "/start_flag",
            self.start_callback,
            10
        )

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
