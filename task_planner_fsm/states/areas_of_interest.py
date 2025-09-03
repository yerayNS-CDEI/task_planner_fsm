from ..state import State
from example_interfaces.srv import SetBool

class AreasOfInterest(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /compute_areas_of_interest")
        ctx["interest_areas_ready"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/compute_areas_of_interest")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /compute_areas_of_interest not available.")
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
                node.get_logger().info(f"[{self.name}] Areas of interest computed correctly.")
                ctx["interest_areas_ready"] = True
                ctx["aoi_data"] = ctx.get("walls_data")
                ctx["scan_phase"] = 2
            else:
                node.get_logger().error(f"[{self.name}] Error while computing areas.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("interest_areas_ready"):
            return "WallDiscretization"
        if ctx.get("error_triggered"):
            return "Error"
        return None
