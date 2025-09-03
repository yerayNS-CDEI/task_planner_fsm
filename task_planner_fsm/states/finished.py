from ..state import State

class Finished(State):
    def __init__(self, name):
        super().__init__(name)
        self.verbose = False

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering END state.")
        pass

    def run(self, ctx):
        if not self.verbose:
            print(f"[{self.name}] All tasks completed.")
            self.verbose = True
            ctx["finished"] = True

    def check_transition(self, ctx):
        return None
