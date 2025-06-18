from ..state import State

class Initialization(State):
    # def __init__(self, name):
    #     super().__init__(name)

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Robot resting. Waiting external signal to begin...")

    def run(self, ctx):
        node = ctx["node"]
        if ctx.get("start"):
            node.get_logger().info(f"[{self.name}] Signal received. Initializing FSM.")
        else:
            node.get_logger().info(f"[{self.name}] Waiting...")

    def check_transition(self, ctx):
        if ctx.get("start"):
            return "CreateMap"
        return None
