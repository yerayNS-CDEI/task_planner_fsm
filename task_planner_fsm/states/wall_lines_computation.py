from ..state import State
from example_interfaces.srv import SetBool

class WallLinesComputation(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /wall_lines_computation")
        ctx["interest_areas_ready"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/wall_lines_computation")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /wall_lines_computation not available.")
            ctx["error_triggered"] = True
            return
        
        self.future = self.client.call_async(request)

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Computing wall lines from the map")

        if self.future is None:
            node.get_logger().info(f"[{self.name}] Future is None.")
            return
        
        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Wall lines computed correctly.")
                ctx["wall_lines_located"] = True
                ctx["wall_lines_data"] = ctx.get("walls_data")
            else:
                node.get_logger().error(f"[{self.name}] Error while computing wall lines.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("wall_lines_located"):
            return "GeometryReconstruction"
        if ctx.get("error_triggered"):
            return "Error"
        return None
