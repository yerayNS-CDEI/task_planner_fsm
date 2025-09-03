from ..state import State
from geometry_msgs.msg import Pose
from collections import deque

class ArmFolding(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_pub = None
        self.movement_done = False
        self.future = None
        self.verbose = False
        self.waiting_for_arrival = False
        self.goals_queue = deque()
        
    def on_enter(self, ctx):
        self.movement_done = False
        self.verbose = False
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering folding state.")
        self.goal_pub = node.create_publisher(Pose, '/arm/goal_pose', 10)     # if no namespace is needed, erase "arm/" in both
        ctx["folding_success"] = False
        ctx["error_triggered"] = False
        ctx["folded_position"] = (0.254, -0.173, 0.401, 0.498, 0.710, 0.286, 0.407)
        ctx["folded_position_joints"] = (0.0, -0.859, -2.812, 0.0, 1.57, 2.80)       # (x,y,z) = -0.182, 0.085, 0.499 / (x,y,z,w) = 0.274, 0.200, -0.677, 0.653
        ctx["unfolded_position_joints"] = (0.0, -1.104, -2.034, 0.0, 1.57, 2.8)   # (x,y,z) = -0.189, -0.183, 0.944 / (x,y,z,w) = 0.548, 0.478, -0.511, 0.458
        ctx["unfolded_position"] = (0.411, -0.173, 0.850, 0.406, 0.577, 0.408, 0.580)

        self.goals_queue.clear()
        self.waiting_for_arrival = False

        unfolded_position = ctx.get("unfolded_position")
        unfolded_pose = Pose()
        unfolded_pose.position.x = unfolded_position[0]
        unfolded_pose.position.y = unfolded_position[1]
        unfolded_pose.position.z = unfolded_position[2]
        unfolded_pose.orientation.x = unfolded_position[3]
        unfolded_pose.orientation.y = unfolded_position[4]
        unfolded_pose.orientation.z = unfolded_position[5]
        unfolded_pose.orientation.w = unfolded_position[6]
        self.goals_queue.append(unfolded_pose)

        folded_position = ctx.get("folded_position")
        folded_pose = Pose()
        folded_pose.position.x = folded_position[0]
        folded_pose.position.y = folded_position[1]
        folded_pose.position.z = folded_position[2]
        folded_pose.orientation.x = folded_position[3]
        folded_pose.orientation.y = folded_position[4]
        folded_pose.orientation.z = folded_position[5]
        folded_pose.orientation.w = folded_position[6]
        self.goals_queue.append(folded_pose)
            
    def _send_next_goal(self, ctx):
        """Publica el siguiente goal de la cola y pone el sistema en espera de llegada."""
        node = ctx["node"]

        if not self.goals_queue:
            # No quedan objetivos → terminar
            self.movement_done = True
            return

        # Tomamos el siguiente sin perderlo para log; luego lo extraemos
        next_goal = self.goals_queue.popleft()

        # Importante para evitar arrastres: el planner debe poner esto a True cuando llegue
        ctx["execution_status"] = False

        # Publicamos el Pose directamente
        self.goal_pub.publish(next_goal)
        self.waiting_for_arrival = True
        node.get_logger().info(
            f"[{self.name}] Goal sent. "
            f"pos=({next_goal.position.x:.3f}, {next_goal.position.y:.3f}, {next_goal.position.z:.3f}). "
            f"orn=({next_goal.orientation.x:.3f}, {next_goal.orientation.y:.3f}, {next_goal.orientation.z:.3f}, {next_goal.orientation.w:.3f})"
        )

    def run(self, ctx):
        node = ctx["node"]

        ## In this section the wall approximation algorithm should be deactivated first
        ## Then, the arm should retreat from the wall to the defined distance (NEEDS TO BE PARAMETRIZED)

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
        if ctx.get("error_triggered") or self.movement_done:
            return
        
        if not self.waiting_for_arrival:
            self._send_next_goal(ctx)
            return
        
        exec_status = ctx.get("execution_status")
        if exec_status is True:
            # Llegamos: liberamos la espera y lanzamos el siguiente
            self.waiting_for_arrival = False
            node.get_logger().info(f"[{self.name}] Goal reached.")
            # Opcional: si tu planner no resetea execution_status, hazlo tú
            ctx["execution_status"] = False
        elif exec_status is False or exec_status is None:
            # Seguimos esperando
            node.get_logger().debug(f"[{self.name}] Waiting for arrival confirmation...")
        else:
            # Si tu pipeline usa otros estados/valores, puedes gestionarlos aquí
            pass

        ###
        ###
        ###

    def check_transition(self, ctx):
        # if self.movement_done and not ctx.get("scan_done"):
        #     return "WallTargetSelection"
        # if self.movement_done and ctx.get("scan_done"):
        #     if ctx.get("scan_phase") == 1:
        #         return "AreasOfInterest"
        #     if ctx.get("scan_phase") == 2:
        #         return "HomePosition"
        # if ctx.get("error_triggered"):
        #     return "Error"
        
        if ctx.get("scan_phase") == 1:
            if self.movement_done and not ctx.get("scan_done"):
                # print("ARM FOLDING - AF1")
                return "WallTargetSelection"
            if self.movement_done and ctx.get("scan_done"):
                # print("ARM FOLDING - AF2")
                return "AreasOfInterest"
        if ctx.get("scan_phase") == 2:
            if self.movement_done and not ctx.get("exhaustive_scan_done"):
                # print("ARM FOLDING - AF3")
                return "WallTargetSelection"
            if self.movement_done and ctx.get("exhaustive_scan_done"):
                # print("ARM FOLDING - AF4")
                return "HomePosition"
        if ctx.get("error_triggered"):
            return "Error"
        
        return None