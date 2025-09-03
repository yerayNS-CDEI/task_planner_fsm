from ..state import State
from geometry_msgs.msg import Pose

class ArmUnfolding(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_sent = False
        self.movement_done = False
        self.future = None
        self.verbose = False
        
    def on_enter(self, ctx):
        self.goal_sent = False
        self.movement_done = False
        self.verbose = False
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering unfolding state.")
        node.publisher_ = node.create_publisher(Pose, '/arm/goal_pose', 10)     # if no namespace is needed, erase "arm/" in both
        ctx["unfolding_success"] = False
        ctx["error_triggered"] = False
        # ctx["unfolded_position_joints"] = (-1.594, -0.736, -1.974, -0.491, 1.472, 1.595)   # (x,y,z) = -0.189, -0.183, 0.944 / (x,y,z,w) = 0.548, 0.478, -0.511, 0.458
        # ctx["folded_position_joints"] = (-1.594, 0.000, -2.872, 0.613, 1.472, 1.595)       # (x,y,z) = -0.182, 0.085, 0.499 / (x,y,z,w) = 0.274, 0.200, -0.677, 0.653
        # ctx["unfolded_position"] = (-0.189, -0.183, 0.944, 0.548, 0.478, -0.511, 0.458)
        # ctx["folded_position"] = (-0.182, 0.085, 0.499, 0.274, 0.200, -0.677, 0.653)
        ctx["unfolded_position_joints"] = (0.0, -1.104, -2.034, 0.0, 1.57, 2.8)   # (x,y,z) = -0.189, -0.183, 0.944 / (x,y,z,w) = 0.548, 0.478, -0.511, 0.458
        ctx["folded_position_joints"] = (-1.594, 0.000, -2.872, 0.613, 1.472, 1.595)       # (x,y,z) = -0.182, 0.085, 0.499 / (x,y,z,w) = 0.274, 0.200, -0.677, 0.653
        ctx["unfolded_position"] = (0.411, -0.173, 0.850, 0.406, 0.577, 0.408, 0.580)
        ctx["folded_position"] = (0.0, -0.173, 0.517, 0.334, 0.476, 0.468, 0.666)

    def run(self, ctx):
        node = ctx["node"]
        # ###
        # ### Using Action Client
        # ###
        # arm_client: ActionClient = ctx["manipulator_client"]
        # ###
        # ###
        # ###

        ###
        ### Using /goal_pose and planner_node
        ###
        if not self.goal_sent:
            unfolded_safe_joint_state = ctx.get("unfolded_position_joints")

            if not unfolded_safe_joint_state:
                node.get_logger().error(f"[{self.name}] Missing unfolded configuration in context.")
                ctx["error_triggered"] = True
                return
            
            if len(unfolded_safe_joint_state) != 6:
                node.get_logger().error(f"[{self.name}] Invalid unfolded_safe_joint_state: expected 6 values.")
                ctx["error_triggered"] = True
                return

            unfolded_position = ctx.get("unfolded_position")
            msg = Pose()
            msg.position.x = unfolded_position[0]
            msg.position.y = unfolded_position[1]
            msg.position.z = unfolded_position[2]
            msg.orientation.x = unfolded_position[3]
            msg.orientation.y = unfolded_position[4]
            msg.orientation.z = unfolded_position[5]
            msg.orientation.w = unfolded_position[6]
            node.publisher_.publish(msg)
            node.get_logger().info(f"[{self.name}] Sending unfolded configuration as a goal.")
            self.goal_sent = True
        
        else:
            if ctx.get("execution_status") == False and not self.verbose:
                node.get_logger().info(f"[{self.name}] Waiting to arrive to goal...")
                self.verbose = True
            if ctx.get("execution_status") == True:
                self.movement_done = True
                node.get_logger().info(f"[{self.name}] Goal reached.")
        ###
        ###
        ###

        ## In this section the wall approximation algorithm should be activated (after ensuring the sensors readings are available)
        ## Then, the arm should approach the wall to the defined distance (NEEDS TO BE PARAMETRIZED)

    def check_transition(self, ctx):
        if self.movement_done and ctx.get("scan_phase") == 1:
            return "ScanWall"
        if self.movement_done and ctx.get("scan_phase") == 2:
            return "ExhaustiveScan"
        if ctx.get("error_triggered"):
            return "Error"
        return None
