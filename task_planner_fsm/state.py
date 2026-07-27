class State:
    def __init__(self, name):
        self.name = name
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
        sentence a bystander could follow ("Sweeping wall segment", "Parking
        chassis", ...). Called every tick with the same string is fine — it is a
        cheap idempotent republish. Safe no-op when the status hook is missing.

        The state machine publishes the status once per tick, *after* run()
        returns, so a description set right before a long blocking call inside
        run() would never reach the panel until the call finishes. Pass
        ``publish=True`` in that case to push the snapshot to the panel
        immediately (e.g. before a blocking GPR HTTP request).
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
