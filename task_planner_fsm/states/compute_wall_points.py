from ..state import State
from ..utils.wall_geometry import (
    ee_rpy_deg_from_inward_normal,
    inward_normal_from_wall_points,
)
import math

class ComputeWallPoints(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False
        self.step_count = 0
        self.MAX_STEPS = 5
        self.predefined_walls = [
            ((4.0, 0.0, 2.0), (4.0, -3.0, 3.0)),
            ((9.0, 0.0, 0.19), (9.0, -4.5, 2.0)),
            ((10.0, -4.5, 0.2), (10.0, 0.0, 3.0)),
        ]

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Computing wall points...")
        self.started = False
        self.step_count = 0
        ctx["walls_data"] = []
        ctx["database_generated"] = False
        ctx["error_triggered"] = False
        ctx["walls_left"] = 0
    
    def _build_wall_data(self, p1, p2, offset=0.6):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length == 0:
            raise ValueError("Wall points must be different.")
        dx /= length
        dy /= length

        # Vector perpendicular normalizado (CW rotation → interior/right face)
        nx = dy
        ny = -dx

        # Puntos desplazados hacia fuera (scan exterior)
        scan_start = (
            p1[0] + offset * dx + nx * offset,
            p1[1] + offset * dy + ny * offset,
            p1[2]
        )
        scan_end = (
            p2[0] - offset * dx + nx * offset,
            p2[1] - offset * dy + ny * offset,
            p2[2]
        )

        inward_normal = inward_normal_from_wall_points(p1, p2)
        ee_rpy_deg = ee_rpy_deg_from_inward_normal(inward_normal)

        return {
            "original": (tuple(p1), tuple(p2)),
            "scan_line": (scan_start, scan_end),
            "inward_normal": inward_normal,
            "ee_rpy_deg": ee_rpy_deg,
        }

    def _parse_indices(self, raw_text):
        tokens = raw_text.replace(",", " ").split()
        return [int(tok) for tok in tokens]

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
                num_walls = int(input(f">> Number of walls to scan? (1-{max_walls}) ").strip())
                if num_walls <= 0 or num_walls > max_walls:
                    raise ValueError(f"Number must be between 1 and {max_walls}.")

                indices_raw = input(">> Enter wall indices to scan (e.g. 1 3): ").strip()
                selected_indices = self._parse_indices(indices_raw)

                if len(selected_indices) != num_walls:
                    raise ValueError(
                        f"You requested {num_walls} wall(s), but provided {len(selected_indices)} indices."
                    )

                if len(set(selected_indices)) != len(selected_indices):
                    raise ValueError("Duplicate wall indices are not allowed.")

                walls_data = []
                for idx in selected_indices:
                    if idx < 1 or idx > max_walls:
                        raise ValueError(f"Wall index {idx} out of range (1-{max_walls}).")
                    p1, p2 = self.predefined_walls[idx - 1]
                    walls_data.append(self._build_wall_data(p1, p2))

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
            ctx["wall_inward_normals"] = [w["inward_normal"] for w in walls_data]
            ctx["wall_ee_rpy_deg"] = [w["ee_rpy_deg"] for w in walls_data]
            for idx, wall in enumerate(walls_data, 1):
                n = wall["inward_normal"]
                r, p, y = wall["ee_rpy_deg"]
                node.get_logger().info(
                    f"[{self.name}] Wall {idx} inward normal=({n[0]:.3f}, {n[1]:.3f}, {n[2]:.3f}) "
                    f"→ EE rpy_deg=({r:.1f}, {p:.1f}, {y:.1f})"
                )
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
