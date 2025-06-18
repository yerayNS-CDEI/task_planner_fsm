from ..state import State

class ComputeWallPoints(State):     # necessari afegir un nou context per saber si hi ha scannable walls??
    def __init__(self, name):
        super().__init__(name)
        self.started = False    # internal flag
        self.step_count = 0
        self.MAX_STEPS = 5

    def on_enter(self, ctx):
        print(f"Advancing to state [{self.name}].")
        self.started = False
        self.step_count = 0
        ctx["database_generated"] = False
        ctx["error_triggered"] = False

    def run(self, ctx):
        if self.step_count >= self.MAX_STEPS:
            if not ctx.get("error_triggered"):
                print(f"[{self.name}] ERROR: The maximum time was exceeded.")
                ctx["error_triggered"] = True
            return
        
        if not self.started:
            print(f"[{self.name}] Starting wall points computation...")
            self.started = True

            # service call to complete/create the database and compute all wall points
            if not ctx.get("database_generated"):
                user_input = input(">> Computing points finished? ").strip().lower()
                if user_input == "yes":
                    ctx["database_generated"] = True

                    wall_input = input(">> Number of walls to scan? ").strip()
                    try:
                        num_walls = int(wall_input)
                        if num_walls <= 0:
                            print(f"[{self.name}] The number must be greater than 0.")
                            ctx["database_generated"] = False
                        else:
                            ctx["walls_left"] = num_walls
                            print(f"[{self.name}] Points computed. {num_walls} walls will be scanned.")
                    except ValueError:
                        print(f"[{self.name}] Invalid input. Please enter an integer.")
                        ctx["database_generated"] = False

        else:
            print(f"[{self.name}] Supervising points computation. Step {self.step_count + 1}...")
            self.step_count += 1

            if not ctx.get("database_generated"):
                user_input = input(">> Computing points finished? ").strip().lower()
                if user_input == "yes":
                    ctx["database_generated"] = True

                    wall_input = input(">> Number of walls to scan? ").strip()
                    try:
                        num_walls = int(wall_input)
                        if num_walls <= 0:
                            print(f"[{self.name}] The number must be greater than 0.")
                            ctx["database_generated"] = False
                        else:
                            ctx["walls_left"] = num_walls
                            print(f"[{self.name}] Points computed. {num_walls} walls will be scanned.")
                    except ValueError:
                        print(f"[{self.name}] Invalid input. Please enter an integer.")
                        ctx["database_generated"] = False

    def check_transition(self, ctx):
        if ctx.get("database_generated") and ctx.get("walls_left", 0) > 0:
            return "WallTargetSelection"
        if ctx.get("error_triggered"):
            return "Error"
        return None
