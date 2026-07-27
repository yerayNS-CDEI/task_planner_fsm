from ..state import State

NEXT_STATE_OPTIONS = [
    "TargetSelection",
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
        self._user_choice = None
        self.started = False

    def _parse_floats(self, raw_text):
        tokens = raw_text.replace(",", " ").split()
        return [float(tok) for tok in tokens]

    def _prompt_drill_locations(self):
        """Prompt the operator for the number of drilling locations and each
        location's (x, y, z) coordinates.

        Returns the list of ``(x, y, z)`` tuples. Raises ValueError on invalid
        input so the caller can report it and re-prompt on the next tick.
        """
        num_locations = int(input(">> Number of drilling locations? (>=1) ").strip())
        if num_locations < 1:
            raise ValueError("Number of drilling locations must be >= 1.")

        locations = []
        for idx in range(1, num_locations + 1):
            coords = self._parse_floats(
                input(f">> Location {idx} coordinates (x y z): ").strip()
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
            return

        try:
            locations = self._prompt_drill_locations()
        except ValueError as e:
            print(f"[{self.name}] Invalid input: {e}")
            return

        self.started = True
        ctx["drill_locations"] = locations
        ctx["num_drill_locations"] = len(locations)
        ctx["data_ready"] = True

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
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
