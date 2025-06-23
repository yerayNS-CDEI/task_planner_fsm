from ..state import State

class Finished(State):
    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering END state.")
        pass

    def run(self, ctx):
        print(f"[{self.name}] All tasks completed.")
        ctx["finished"] = True

    def check_transition(self, ctx):
        return None
