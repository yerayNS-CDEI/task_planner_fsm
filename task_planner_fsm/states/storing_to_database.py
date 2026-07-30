from ..state import State
from example_interfaces.srv import SetBool

NEXT_STATE_OPTIONS = [
    "TargetSelection",
    "WaitForData",
    "ManipulatorFolding",
    "Error",
]

class StoringToDatabase(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /storing_to_database")
        ctx["storing_to_database_success"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        self.client = node.create_client(SetBool, "/storing_to_database")
        request = SetBool.Request()
        request.data = True

        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /storing_to_database not available.")
            self.fail(ctx, "the /storing_to_database service is unavailable")
            return

        self.future = self.client.call_async(request)

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Storing the drilling results in the database")

        if self.future is None:
            node.get_logger().info(f"[{self.name}] Future is None.")
            return

        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Storing to database completed.")
                ctx["storing_to_database_success"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving data.")
                self.fail(ctx, "the /storing_to_database service reported a failure")
            self.future = None

    def check_transition(self, ctx):
        if not ctx.get("storing_to_database_success") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
