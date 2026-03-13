from ..state import State
from geometry_msgs.msg import Point, Quaternion

class Initialization(State):
    def __init__(self, name):
        super().__init__(name)
        self.verbose = False

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Robot resting. Waiting external signal to begin...")
        
        # Home position is always map origin
        ctx["home_position"] = Point(x=0.0, y=0.0, z=0.0)
        ctx["home_orientation"] = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        node.get_logger().info(f"[{self.name}] Home pose set to map origin.")

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("start"):
            node.get_logger().info(f"[{self.name}] Signal received. Initializing FSM.")
        else:
            if not self.verbose:
                node.get_logger().info('[{}] Waiting for /start_flag to be true...\n\033[1;32mUse: ros2 topic pub /start_flag std_msgs/Bool "data: true" --once\033[0m'.format(self.name))
                self.verbose = True

    def check_transition(self, ctx):
        if ctx.get("start"):
            return "CreateMap"
        return None
