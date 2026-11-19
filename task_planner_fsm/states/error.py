from ..state import State

class Error(State):
    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        prev = ctx.get("last_state", "unknown")
        # The machine already logged the failure as an ERROR (and published it
        # on /fsm/event); this only says where the run stopped, at INFO so the
        # UI's error list does not count the same failure twice.
        summary = ctx.get("fsm_error_summary") or f"{prev} failed"
        node = ctx.get("node")
        message = f"[Error] Run stopped: {summary}. Send /fsm/restart to start again."
        if node is not None:
            node.get_logger().info(message)
        else:
            print(message)
        ctx["fatal"] = True

    def run(self, ctx):
        prev = ctx.get("last_state", "unknown")
        summary = ctx.get("fsm_error_summary") or f"{prev} failed; robot stopped"
        summary += " -- send /fsm/restart to start again"
        self.set_activity(ctx, summary, level="error")

    def check_transition(self, ctx):
        return None