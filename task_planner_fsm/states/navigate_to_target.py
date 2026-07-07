from ..state import State
from example_interfaces.srv import SetBool

class NavigateToTarget(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /navigate_to_target")
        ctx["target_reached"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/navigate_to_target")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /navigate_to_target not available.")
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
                node.get_logger().info(f"[{self.name}] Navigate to target completed.")
                ctx["target_reached"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving data.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("target_reached"):
            return "ManipulatorReachability"
        if ctx.get("error_triggered"):
            return "Error"
        return None
