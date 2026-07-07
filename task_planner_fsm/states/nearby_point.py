from ..state import State
from example_interfaces.srv import SetBool

class NearbyPointSelection(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /nearby_point_selection")
        ctx["nearby_point_selected"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/nearby_point_selection")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /nearby_point_selection not available.")
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
                node.get_logger().info(f"[{self.name}] Nearby point selection completed.")
                ctx["nearby_point_selected"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving data.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("nearby_point_selected"):
            if ctx.get("no_path_found"):
                return "TargetSelection"
            else:
                return "ManipulatorUnfolding"
        if ctx.get("error_triggered"):
            return "Error"
        return None
