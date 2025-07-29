from ..state import State
from nav_msgs.msg import Odometry

class Initialization(State):
    def __init__(self, name):
        super().__init__(name)
        self.home_position = None
        self.home_orientation = None
        self.odom_received = False
        self.odom_saved = False

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Robot resting. Waiting external signal to begin...")

        node.create_subscription(Odometry, "/odometry/global", self.odometry_callback, 10)

    def odometry_callback(self, msg: Odometry):
        self.home_position = msg.pose.pose.position
        self.home_orientation = msg.pose.pose.orientation
        self.odom_received = True

    def run(self, ctx):
        node = ctx["node"]

        if self.odom_received and not self.odom_saved:
            ctx["home_position"] = self.home_position
            ctx["home_orientation"] = self.home_orientation
            node.get_logger().info(f"[{self.name}] Home pose saved: position={self.home_position}, orientation={self.home_orientation}")
            self.odom_received = False  # one time save
            self.odom_saved = True

        if ctx.get("start"):
            node.get_logger().info(f"[{self.name}] Signal received. Initializing FSM.")
        else:
            node.get_logger().info(f"[{self.name}] Waiting...")

    def check_transition(self, ctx):
        if ctx.get("start"):
            return "CreateMap"
        return None
