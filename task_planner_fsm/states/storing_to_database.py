from ..state import State
from example_interfaces.srv import SetBool

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

        self.client = node.create_client(SetBool, "/storing_to_database")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /storing_to_database not available.")
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
                node.get_logger().info(f"[{self.name}] Storing to database completed.")
                ctx["storing_to_database_success"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving data.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("storing_to_database_success"):
            if ctx.get("still_locations_to_drill"):
                return "TargetSelection"
            if ctx.get("all_locations_drilled") and not ctx.get("oliwall_finished"):
                return "WaitForData"
            if ctx.get("all_locations_drilled") and ctx.get("oliwall_finished"):
                return "ManipulatorFolding"
        if ctx.get("error_triggered"):
            return "Error"
        return None
