from ..state import State
from example_interfaces.srv import SetBool

class ReceiveNav2Map(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /receive_nav2_map")
        ctx["nav2_map_ready"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/receive_nav2_map")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /receive_nav2_map not available.")
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
                node.get_logger().info(f"[{self.name}] Nav2 map received correctly.")
                ctx["nav2_map_ready"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving Nav2 map.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("nav2_map_ready"):
            return "GetSemanticMap"
        if ctx.get("error_triggered"):
            return "Error"
        return None
