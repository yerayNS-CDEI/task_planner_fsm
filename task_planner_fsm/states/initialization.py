from ..state import State

class Initialization(State):
    def __init__(self, name):
        super().__init__(name)
        self.home_saved = False
        self.verbose = False

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Robot resting. Waiting external signal to begin...")

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("odom_received") and not self.home_saved:
            ctx["home_position"] = ctx.get("base_position")
            ctx["home_orientation"] = ctx.get("base_orientation")
            pos = ctx.get("home_position")
            orn = ctx.get("home_orientation")
            node.get_logger().info(f"[{self.name}] Home pose saved: position={pos}, orientation={orn}")
            self.odom_received = False  # one time save
            self.home_saved = True

        if ctx.get("start") and self.home_saved:
            node.get_logger().info(f"[{self.name}] Signal received. Initializing FSM.")
        else:
            if not self.verbose:
                node.get_logger().info(f"[{self.name}] Waiting...")
                self.verbose = True

    def check_transition(self, ctx):
        if ctx.get("start") and self.home_saved:
            return "CreateMap"
        return None
