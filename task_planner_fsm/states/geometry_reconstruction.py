from ..state import State
from example_interfaces.srv import SetBool

class GeometryReconstruction(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"{self.name} Calling the service /start_geometry_reconstruction")
        ctx["reconstruction_ready"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/start_geometry_reconstruction")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /start_geometry_reconstruction not available.")
            ctx["error_triggered"] = True
            return
        
        self.future = self.client.call_async(request)

    def run(self, ctx):        
        if self.future is None:
            node.get_logger().info(f"[{self.name}] Future is None.")
            return
        
        node = ctx["node"]
        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Geometry reconstructed correctly.")
                ctx["reconstruction_ready"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while reconstructing geometry.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("reconstruction_ready"):
            return "ComputeWallPoints"
        if ctx.get("error_triggered"):
            return "Error"
        return None
