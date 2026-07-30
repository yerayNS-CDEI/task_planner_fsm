from ..state import State
from example_interfaces.srv import SetBool

NEXT_STATE_OPTIONS = [
    "Drilling",
    "Error",
]

class SuctionDrillStart(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /start_suction_drill")
        ctx["start_suction_drill_success"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        self.client = node.create_client(SetBool, "/start_suction_drill")
        request = SetBool.Request()
        request.data = True

        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /start_suction_drill not available.")
            self.fail(ctx, "the /start_suction_drill service is unavailable")
            return

        self.future = self.client.call_async(request)

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Starting the dust suction and the drill motor")

        if self.future is None:
            node.get_logger().info(f"[{self.name}] Future is None.")
            return

        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Suction and drill start completed.")
                ctx["start_suction_drill_success"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving data.")
                self.fail(ctx, "the /start_suction_drill service reported a failure")
            self.future = None

    def check_transition(self, ctx):
        if not ctx.get("start_suction_drill_success") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
