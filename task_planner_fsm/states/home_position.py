import random
from ..state import State

class HomePosition(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False        # internal flag
        self.step_count = 0
        self.MAX_STEPS = 5
        
    def on_enter(self, ctx):
        print(f"Advancing to state [{self.name}].")
        self.started = False
        self.step_count = 0
        ctx["home"] = False
        ctx["error_triggered"] = False

    def run(self, ctx):
        if self.step_count >= self.MAX_STEPS:
            if not ctx.get("error_triggered"):
                print(f"[{self.name}] ERROR: The maximum time was exceeded.")
                ctx["error_triggered"] = True
            return
        
        if not self.started:
            print(f"[{self.name}] Starting navigation to HOME...")
            self.started = True

            # service call to move the base robot to target
            if not ctx["home"]:
                user_input = input(">> Target reached? ").strip().lower()
                if user_input == "yes":
                    ctx["home"] = True
                    print(f"[{self.name}] Navigation to target complete.")

        else:
            print(f"[{self.name}] Supervising navigation. Step {self.step_count + 1}...")
            self.step_count += 1

            if not ctx["home"]:
                user_input = input(">> Target reached? ").strip().lower()
                if user_input == "yes":
                    ctx["home"] = True
                    print(f"[{self.name}] Navigation to target complete.")

    def check_transition(self, ctx):
        if ctx.get("home"):
            return "Finished"
        if ctx.get("error_triggered"):
            return "Error"
        return None
