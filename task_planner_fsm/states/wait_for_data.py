from ..state import State
from example_interfaces.srv import SetBool

NEXT_STATE_OPTIONS = [
    "TargetSelection",
    "Error",
]

class WaitForData(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /wait_for_data")
        ctx["data_ready"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        self.client = node.create_client(SetBool, "/wait_for_data")
        request = SetBool.Request()
        request.data = True

        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /wait_for_data not available.")
            ctx["error_triggered"] = True
            return

        self.future = self.client.call_async(request)

    def run(self, ctx):
        node = ctx["node"]

        if self.future is None:
            node.get_logger().info(f"[{self.name}] Future is None.")
            return

        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Data received correctly.")
                ctx["data_ready"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving data.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if not ctx.get("data_ready") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
