from ..state import State
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.task import Future
from math import atan2, sin, cos
import math
from geometry_msgs.msg import Pose

class ArmUnfolding(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_sent = False
        self.movement_done = False
        self.future = None
        
    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering unfolding state.")
        node.publisher_ = node.create_publisher(Pose, '/goal_pose', 10)
        ctx["unfolding_success"] = False
        ctx["error_triggered"] = False
        # self.goal_sent = False
        # self.movement_done = False

    def run(self, ctx):
        node = ctx["node"]
        # arm_client: ActionClient = ctx["manipulator_client"]
        
        if not self.goal_sent:
            unfolded_safe_joint_state = ctx.get("unfolded_position")

            if not unfolded_safe_joint_state:
                node.get_logger().error(f"[{self.name}] Missing unfolded configuration in context.")
                ctx["error_triggered"] = True
                return
            
            if len(unfolded_safe_joint_state) != 6:
                node.get_logger().error(f"[{self.name}] Invalid unfolded_safe_joint_state: expected 6 values.")
                ctx["error_triggered"] = True
                return

            msg = Pose()
            msg.position.x = -0.25
            msg.position.y = -0.5
            msg.position.z = 0.6
            msg.orientation.x = 0.0
            msg.orientation.y = -0.70710678
            msg.orientation.z = 0.0
            msg.orientation.w = 0.70710678
            msg.publisher_.publish(msg)
            node.get_logger().info(f"[{self.name}] Sending unfolded configuration as a goal.")
            self.goal_sent = True
        
        else:
            if ctx["execution_status"] == False:
                node.get_logger().info(f"[{self.name}] Waiting to arrive to goal...")
            else:
                self.movement_done = True
                node.get_logger().info(f"[{self.name}] Goal reached.")

    def check_transition(self, ctx):
        if self.movement_done:
            return "ScanWall"
        if ctx.get("error_triggered"):
            return "Error"
        return None
