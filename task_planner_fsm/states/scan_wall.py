from ..state import State

class ScanWall(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False        # internal flag
        self.finished = False
        self.step_count = 0
        self.MAX_STEPS = 5
        
    def on_enter(self, ctx):
        print(f"Advancing to state [{self.name}].")
        self.started = False
        self.finished = False
        self.step_count = 0
        ctx["map_ready"] = False
        ctx["error_triggered"] = False

    def run(self, ctx):
        if self.step_count >= self.MAX_STEPS:
            if not ctx.get("error_triggered"):
                print(f"[{self.name}] ERROR: The maximum time was exceeded.")
                ctx["error_triggered"] = True
            return
        
        if not self.started:
            print(f"[{self.name}] Starting analysis of {ctx['walls_left']} walls...")
            self.started = True

            # service call to scan a wall
            if ctx["walls_left"] > 0:
                print(f"[{self.name}] Scanning wall {ctx['walls_left']}...")
                user_input = input(">> Wall already scanned? ").strip().lower()
                if user_input == "yes":
                    ctx["walls_left"] -= 1
                    self.finished = True
                    print(f"[{self.name}] Remaining walls to scan: {ctx['walls_left']}")
                if ctx["walls_left"] == 0:
                    ctx["scan_done"] = True

        else:
            print(f"[{self.name}] Supervising wall scanning. Step {self.step_count + 1}...")
            self.step_count += 1

            if ctx["walls_left"] > 0:
                print(f"[{self.name}] Scanning wall {ctx['walls_left']}...")
                user_input = input(">> Wall already scanned? ").strip().lower()
                if user_input == "yes":
                    ctx["walls_left"] -= 1
                    self.finished = True
                    print(f"[{self.name}] Remaining walls to scan: {ctx['walls_left']}")
                if ctx["walls_left"] == 0:
                    ctx["scan_done"] = True

    def check_transition(self, ctx):
        if self.finished and not ctx.get("scan_done"):
            return "WallTargetSelection"
        if ctx.get("scan_done"):
            return "HomePosition"
        if ctx.get("error_triggered"):
            return "Error"
        return None
