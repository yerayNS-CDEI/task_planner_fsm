from ..state import State, ask

NEXT_STATE_OPTIONS = [
    "TargetSelection",
    "ManipulatorFolding",
    "Error",
]

class WaitForData(State):
    def __init__(self, name):
        super().__init__(name)
        # Guards the one-shot interactive prompt so run() asks the operator only
        # once (run() is called every FSM tick).
        self.started = False

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(
            f"[{self.name}] Waiting for drilling data (operator input)."
        )
        ctx["data_ready"] = False
        ctx["error_triggered"] = False
        ctx["drill_locations"] = []
        ctx["num_drill_locations"] = 0
        # Set when oliwall reports it is done and asks for no further drilling.
        ctx["drilling_complete"] = False
        # A fresh batch renumbers the locations, so the indices TargetSelection
        # ticked off for the previous batch no longer refer to these points.
        ctx["completed_drill_indices"] = []
        self._user_choice = None
        self.started = False

    def _parse_floats(self, raw_text):
        tokens = raw_text.replace(",", " ").split()
        return [float(tok) for tok in tokens]

    def _prompt_drill_locations(self, node):
        """Prompt the operator for the number of drilling locations and each
        location's (x, y, z) coordinates.

        Returns the list of ``(x, y, z)`` tuples -- EMPTY when the operator enters
        0, meaning oliwall finished its wall without requesting more drilling, so
        pokeye has nothing left to do either. Raises ValueError on invalid input so
        the caller can report it and re-prompt on the next tick.
        """
        num_locations = int(
            ask(node, self.name, "Number of drilling locations? (>=1, or 0 if oliwall is finished)")
        )
        if num_locations < 0:
            raise ValueError("Number of drilling locations cannot be negative.")
        if num_locations == 0:
            return []

        locations = []
        for idx in range(1, num_locations + 1):
            coords = self._parse_floats(
                ask(node, self.name, f"Location {idx} coordinates (x y z):")
            )
            if len(coords) != 3:
                raise ValueError(
                    f"Location {idx}: expected 3 coordinates (x y z), got {len(coords)}."
                )
            locations.append(tuple(coords))
        return locations

    def run(self, ctx):
        node = ctx["node"]

        if self.started:
            self.set_activity(ctx, "Drilling locations received from oliwall")
            return

        # The prompt blocks run() on stdin for as long as the operator takes, so
        # push the description to the panel now rather than after run() returns.
        self.set_activity(
            ctx, "Waiting for the operator to enter the drilling locations", publish=True
        )

        try:
            locations = self._prompt_drill_locations(node)
        except ValueError as e:
            node.get_logger().warn(f"[{self.name}] Invalid input: {e}")
            return

        self.started = True
        ctx["drill_locations"] = locations
        ctx["num_drill_locations"] = len(locations)
        ctx["data_ready"] = True

        if not locations:
            # oliwall is done and needs no more holes: pokeye stops waiting, folds
            # the gantry and goes home instead of continuing into TargetSelection.
            ctx["drilling_complete"] = True
            node.get_logger().info(
                f"[{self.name}] oliwall reported no further drilling locations; "
                f"finishing the pokeye run."
            )
            return

        node.get_logger().info(
            f"[{self.name}] {len(locations)} drilling location(s) received:"
        )
        for idx, (x, y, z) in enumerate(locations, 1):
            node.get_logger().info(
                f"  Location {idx}: ({x:.3f}, {y:.3f}, {z:.3f})"
            )

    def check_transition(self, ctx):
        if not ctx.get("data_ready") and not ctx.get("error_triggered"):
            return None
        if ctx.get("drilling_complete") and not ctx.get("error_triggered"):
            # No drilling left to do -- route straight to folding (and from there
            # home) without asking the operator to pick a state.
            return "ManipulatorFolding"
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
