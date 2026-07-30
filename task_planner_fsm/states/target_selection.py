from ..state import State
import math

NEXT_STATE_OPTIONS = [
    "DrillApproach",
    "ManipulatorFolding",
    "Error",
]

class TargetSelection(State):
    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Selecting the nearest drilling target...")
        ctx["target_selected"] = False
        ctx["error_triggered"] = False
        self._user_choice = None
        # Indices of drilling locations already selected on previous visits.
        # Persist across revisits (setdefault, NOT reset) so the same location is
        # never chosen twice -- otherwise the FSM would keep picking the same
        # nearest point and loop forever.
        ctx.setdefault("completed_drill_indices", [])

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Selecting the nearest drilling target")

        # Robot's current position, from /rtabmap/odom (see fsm_node).
        base = ctx.get("base_position")
        if base is None:
            node.get_logger().warn(f"[{self.name}] Waiting for odometry (base_position)...")
            self.set_activity(ctx, "Waiting for odometry before picking a target")
            return

        locations = ctx.get("drill_locations", [])
        if not locations:
            node.get_logger().error(f"[{self.name}] No drill_locations found in context.")
            self.fail(ctx, "no drilling locations available to choose from")
            return

        completed = set(ctx.get("completed_drill_indices", []))
        robot_x = float(base.x)
        robot_y = float(base.y)

        # Pick the closest location (2D distance in the map plane) that has not
        # been selected yet.
        min_dist = float("inf")
        selected_idx = -1
        selected_point = None
        for idx, loc in enumerate(locations):
            if idx in completed:
                continue
            dist = math.hypot(robot_x - loc[0], robot_y - loc[1])
            if dist < min_dist:
                min_dist = dist
                selected_idx = idx
                selected_point = loc

        if selected_idx == -1:
            # Every location has been drilled; nothing left to select. Flag it so
            # the FSM stops instead of spinning on an empty selection forever.
            node.get_logger().warn(
                f"[{self.name}] All {len(locations)} drilling location(s) already "
                f"selected; none left to drill."
            )
            self.fail(
                ctx,
                f"all {len(locations)} drilling location(s) are already selected",
            )
            return

        # Remember this location so it is not selected again.
        completed.add(selected_idx)
        ctx["completed_drill_indices"] = sorted(completed)
        ctx["current_drill_index"] = selected_idx
        ctx["current_drill_target"] = selected_point
        ctx["target_selected"] = True

        remaining = len(locations) - len(completed)
        self.set_activity(
            ctx,
            f"Selected drilling point {len(completed)}/{len(locations)} "
            f"at ({selected_point[0]:.2f}, {selected_point[1]:.2f}, {selected_point[2]:.2f})",
            progress_current=len(completed),
            progress_total=len(locations),
        )
        node.get_logger().info(
            f"[{self.name}] Selected drilling location #{selected_idx} at "
            f"({selected_point[0]:.3f}, {selected_point[1]:.3f}, {selected_point[2]:.3f}) "
            f"| distance {min_dist:.3f} m | {remaining} location(s) remaining."
        )

        publish_markers = ctx.get("publish_drill_markers")
        if publish_markers:
            publish_markers(ctx)

    def check_transition(self, ctx):
        if not ctx.get("target_selected") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
