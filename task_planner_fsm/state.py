class State:
    def __init__(self, name):
        self.name = name
        # Cached choice for the interactive next-state picker used during manual
        # workflow testing (see select_next_state). Reset in on_enter so a state
        # revisited in a loop prompts again.
        self._user_choice = None

    def on_enter(self, ctx):
        pass

    def run(self, ctx):
        pass

    def check_transition(self, ctx):
        return None

    def on_exit(self, ctx):
        pass

    def select_next_state(self, ctx, options):
        """Prompt the user to pick the next state (manual workflow testing).

        Blocks on stdin, logs the numbered options and returns the chosen state
        name. The choice is cached in self._user_choice until the state is
        re-entered, so this can safely be called on every FSM tick. States must
        reset self._user_choice = None in on_enter.
        """
        if self._user_choice is not None:
            return self._user_choice

        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Select the next state:")
        for i, option in enumerate(options, 1):
            node.get_logger().info(f"  {i}. {option}")
        while True:
            raw = input(f"[{self.name}] Enter choice (1-{len(options)}): ").strip()
            if raw.isdigit() and 1 <= int(raw) <= len(options):
                self._user_choice = options[int(raw) - 1]
                node.get_logger().info(f"[{self.name}] Transitioning to '{self._user_choice}'")
                return self._user_choice
            print(f"  Invalid choice. Enter a number between 1 and {len(options)}.")
