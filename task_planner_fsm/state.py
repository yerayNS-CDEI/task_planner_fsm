import sys


def ask(node, state_name, question):
    """Ask the operator a question on stdin and return the stripped answer.

    The question is emitted through the ROS logger and the prompt marker through
    stderr -- deliberately NOT as the ``input()`` prompt. The FSM is normally run
    under ``ros2 run``/``ros2 launch``, where stdout is a pipe and therefore
    block-buffered, so a bare ``input()`` prompt (no trailing newline) is never
    flushed: the operator sees nothing and a state blocked on input looks like a
    hung FSM. Logger records and the flushed stderr marker always reach the
    console, next to the rest of the FSM output.
    """
    node.get_logger().info(f"[{state_name}] {question}")
    sys.stdout.flush()
    sys.stderr.write(f"[{state_name}] >> ")
    sys.stderr.flush()
    return input().strip()


class State:
    def __init__(self, name):
        self.name = name
        # Cached choice for the interactive next-state picker used during manual
        # workflow testing (see select_next_state). Reset in on_enter so a state
        # revisited in a loop prompts again.
        self._user_choice = None
        self._last_activity = None

    def on_enter(self, ctx):
        pass

    def run(self, ctx):
        pass

    def check_transition(self, ctx):
        return None

    def on_exit(self, ctx):
        pass

    # ------------------------------------------------------------------
    # Human-readable activity reporting (shown in the RViz FSM panel).
    # ------------------------------------------------------------------
    def set_activity(
        self,
        ctx,
        description,
        *,
        level="info",
        progress_current=None,
        progress_total=None,
        publish=False,
    ):
        """Describe, in plain language, what the robot is doing right now.

        The text lands in the ``summary`` field of ``/fsm/status`` and is shown
        as the "Summary" line of the RViz FSM panel, so it should read like a
        sentence a bystander could follow ("Drilling the target point",
        "Parking chassis", ...). Called every tick with the same string is fine
        -- it is a cheap idempotent republish. Safe no-op when the status hook
        is missing.

        The state machine publishes the status once per tick, *after* run()
        returns, so a description set right before a long blocking call inside
        run() would never reach the panel until the call finishes. Pass
        ``publish=True`` in that case to push the snapshot to the panel
        immediately (e.g. before a blocking stdin prompt).
        """
        # Republish even when unchanged: the machine's per-tick default would
        # otherwise reset the summary, and late panel subscribers need it too.
        self._last_activity = description
        set_status = ctx.get("set_fsm_status")
        if callable(set_status):
            set_status(
                self.name,
                phase="running",
                summary=description,
                level=level,
                progress_current=progress_current,
                progress_total=progress_total,
            )
        if publish:
            publish_status = ctx.get("publish_fsm_status")
            snapshot = ctx.get("_fsm_status")
            if callable(publish_status) and snapshot is not None:
                try:
                    publish_status(dict(snapshot))
                except Exception:
                    pass

    def fail(self, ctx, reason):
        """Flag a failure with a human-readable reason.

        Sets ``error_triggered`` (so ``check_transition`` routes to Error/retry)
        and stashes ``error_reason`` so the state machine can render
        "<State> failed due to <reason>" in the panel summary. Returns None so
        callers can ``return self.fail(...)`` in one line.
        """
        ctx["error_triggered"] = True
        ctx["error_reason"] = str(reason)
        return None

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
            raw = ask(node, self.name, f"Enter choice (1-{len(options)}):")
            if raw.isdigit() and 1 <= int(raw) <= len(options):
                self._user_choice = options[int(raw) - 1]
                node.get_logger().info(f"[{self.name}] Transitioning to '{self._user_choice}'")
                return self._user_choice
            node.get_logger().warn(
                f"[{self.name}] Invalid choice. Enter a number between 1 and {len(options)}."
            )
