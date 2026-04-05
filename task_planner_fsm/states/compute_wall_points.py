from ..state import State
from task_planner_fsm.utils.wall_utils import PREDEFINED_WALLS, build_wall_data

class ComputeWallPoints(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False
        self.step_count = 0
        self.MAX_STEPS = 5
        self.predefined_walls = PREDEFINED_WALLS

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Computing wall points...")
        self.started = False
        self.step_count = 0
        ctx["walls_data"] = []
        ctx["database_generated"] = False
        ctx["error_triggered"] = False
        ctx["walls_left"] = 0
    
    def run(self, ctx):
        node = ctx["node"]
        if self.step_count >= self.MAX_STEPS:
            if not ctx.get("error_triggered"):
                print(f"[{self.name}] ERROR: The maximum time was exceeded.")
                ctx["error_triggered"] = True
            return

        if not self.started:
            print(f"[{self.name}] Available predefined walls:")
            for i, (p1, p2) in enumerate(self.predefined_walls, 1):
                print(f"  {i}: p1={p1}, p2={p2}")

            try:
                max_walls = len(self.predefined_walls)
                # Optional context configuration:
                #   wall_indices: List[int] with 1-based wall indices (e.g. [1, 3]).
                # If missing, all predefined walls are selected by default.
                configured_indices = ctx.get("wall_indices", list(range(1, max_walls + 1)))
                if not isinstance(configured_indices, list) or not configured_indices:
                    raise ValueError("wall_indices must be a non-empty list of 1-based wall indices.")
                selected_indices = [int(idx) for idx in configured_indices]
                if len(set(selected_indices)) != len(selected_indices):
                    raise ValueError("Duplicate wall indices are not allowed.")
                num_walls = len(selected_indices)

                walls_data = []
                for idx in selected_indices:
                    if idx < 1 or idx > max_walls:
                        raise ValueError(f"Wall index {idx} out of range (1-{max_walls}).")
                    p1, p2 = self.predefined_walls[idx - 1]
                    walls_data.append(build_wall_data(p1, p2))

            except ValueError as e:
                print(f"[{self.name}] Invalid input: {e}")
                return

            self.started = True

            node.get_logger().info(f"[{self.name}] Selected scan lines:")
            for idx, wall in enumerate(walls_data, 1):
                s_start, s_end = wall["scan_line"]
                node.get_logger().info(f"  Wall {idx}: {s_start} -> {s_end}")

            ctx["database_generated"] = True
            ctx["walls_left"] = num_walls
            ctx["walls_data"] = walls_data
            node.get_logger().info(f"[{self.name}] {num_walls} predefined wall(s) loaded successfully.")

        else:
            node.get_logger().info(f"[{self.name}] Supervising points computation. Step {self.step_count + 1}...")
            self.step_count += 1

    def check_transition(self, ctx):
        if ctx.get("database_generated") and ctx.get("walls_left", 0) > 0:
            return "WallTargetSelection"
        if ctx.get("error_triggered"):
            return "Error"
        return None
