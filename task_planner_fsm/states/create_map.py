from ..state import State
from example_interfaces.srv import SetBool

class CreateMap(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None
    #     self.started = False        # internal flag
    #     self.step_count = 0
    #     self.MAX_STEPS = 5

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info("[CreateMap] Calling the service /start_mapping")

        self.client = node.create_client(SetBool, "/start_mapping")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /start_mapping not available.")
            ctx["error_triggered"] = True
            return
        
        self.future = self.client.call_async(request)

    def run(self, ctx):
        if self.future is None:
            return
        
        node = ctx["node"]
        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Map generated correctly.")
                ctx["map_ready"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while generating map.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("map_ready"):
            return "ComputeWallPoints"
        if ctx.get("error_triggered"):
            return "Error"
        return None
