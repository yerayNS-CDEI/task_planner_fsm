from ..state import State
import math

class ComputeWallPoints(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False
        self.step_count = 0
        self.MAX_STEPS = 5

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Computing wall points...")
        self.started = False
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
            print(f"[{self.name}] Starting wall points computation...")
            self.started = True

            user_input = input(">> Database ready? (yes/no) ").strip().lower()
            if user_input == "yes":
                try:
                    num_walls = int(input(">> Number of walls to scan? ").strip())
                    if num_walls <= 0:
                        raise ValueError("Number must be greater than 0.")
                except ValueError as e:
                    print(f"[{self.name}] Invalid input: {e}")
                    return

                walls_data = []
                for i in range(num_walls):
                    print(f">> Enter points for wall {i+1} (format: x y z)")
                    try:
                        p1 = list(map(float, input(f"   - Start point: ").strip().split()))
                        p2 = list(map(float, input(f"   - End point: ").strip().split()))
                        if len(p1) != 3 or len(p2) != 3:
                            raise ValueError("Each point must have 3 coordinates.")

                        # Vector dirección en XY
                        dx = p2[0] - p1[0]
                        dy = p2[1] - p1[1]
                        length = math.hypot(dx, dy)
                        if length == 0:
                            raise ValueError("Wall points must be different.")
                        dx /= length
                        dy /= length

                        # Vector perpendicular normalizado
                        nx = -dy
                        ny = dx
                        offset = 0.4

                        # Puntos desplazados hacia fuera (scan exterior)
                        scan_start = (
                            p1[0] + offset * dx + nx * offset, 
                            p1[1] + offset * dy + ny * offset, 
                            p1[2]
                        )
                        scan_end   = (
                            p2[0] - offset * dx + nx * offset, 
                            p2[1] - offset * dy + ny * offset, 
                            p2[2]
                        )

                        # Guardar en lista de datos de muros
                        walls_data.append({
                            "original": (tuple(p1), tuple(p2)),
                            "scan_line": (scan_start, scan_end),
                        })

                    except ValueError as e:
                        print(f"[{self.name}] Invalid point format: {e}")
                        ctx["error_triggered"] = True
                        return

                node.get_logger().info(f"[{self.name}] All scan lines:")
                for idx, wall in enumerate(walls_data, 1):
                    s_start, s_end = wall["scan_line"]
                    node.get_logger().info(f"  Wall {idx}: {s_start} -> {s_end}")

                ctx["database_generated"] = True
                ctx["walls_left"] = num_walls
                ctx["walls_data"] = walls_data
                node.get_logger().info(f"[{self.name}] {num_walls} walls loaded successfully.")
            else:
                print(f"[{self.name}] Waiting for database confirmation...")

        else:
            node.get_logger().info(f"[{self.name}] Supervising points computation. Step {self.step_count + 1}...")
            self.step_count += 1

    def check_transition(self, ctx):
        if ctx.get("database_generated") and ctx.get("walls_left", 0) > 0:
            return "WallTargetSelection"
        if ctx.get("error_triggered"):
            return "Error"
        return None
