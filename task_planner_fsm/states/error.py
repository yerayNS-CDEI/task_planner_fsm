from ..state import State

class Error(State):
    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        prev = ctx.get("last_state", "unknown")
        print(f"[Error] An error occurred while executing the state {prev}")
        
        if prev == "NavigateToTarget":
            print("[Error] Navigation failed. The robot could not reach the target after several attempts..")
        elif prev == "ScanWall":
            print("[Error] Failed to analyze walls.")
        else:
            print("[Error] Unexpected error.")
        
        ctx["fatal"] = True

    def run(self, ctx):
        pass

    def check_transition(self, ctx):
        return None