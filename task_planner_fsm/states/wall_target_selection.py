from ..state import State

class WallTargetSelection(State):     # necessari afegir un nou context per saber si hi ha scannable walls??
    def __init__(self, name):
        super().__init__(name)
        self.started = False    # internal flag
        self.step_count = 0
        self.MAX_STEPS = 5

    def on_enter(self, ctx):
        print(f"Advancing to state [{self.name}].")
        self.started = False
        self.step_count = 0
        ctx["target_selected"] = False
        ctx["error_triggered"] = False

    def run(self, ctx):
        if self.step_count >= self.MAX_STEPS:
            if not ctx.get("error_triggered"):
                print(f"[{self.name}] ERROR: The maximum time was exceeded.")
                ctx["error_triggered"] = True
            return
        
        if not self.started:
            print(f"[{self.name}] Selecting wall to scan...")
            self.started = True

            # service call to complete/create the database and compute all wall points
            if not ctx.get("target_selected"):
                user_input = input(">> Wall already selected? ").strip().lower()
                if user_input == "yes":
                    ctx["target_selected"] = True
                    print(f"[{self.name}] Wall selected.")

        else:
            print(f"[{self.name}] Supervising wall selection. Step {self.step_count + 1}...")
            self.step_count += 1

            if not ctx.get("target_selected"):
                user_input = input(">> Wall already selected? ").strip().lower()
                if user_input == "yes":
                    ctx["target_selected"] = True
                    print(f"[{self.name}] Wall selected.")

    def check_transition(self, ctx):
        if ctx.get("target_selected"):
            return "NavigateToTarget"
        if ctx.get("error_triggered"):
            return "Error"
        return None
