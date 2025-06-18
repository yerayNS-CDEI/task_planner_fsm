from ..state import State

class Finished(State):
    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        print(f"Advancing to state [{self.name}].")
        pass

    def run(self, ctx):
        print(f"[{self.name}] All tasks completed.")
        ctx["finished"] = True

    def check_transition(self, ctx):
        return None
