from ..state import State
from example_interfaces.srv import SetBool

class ObjectID(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None
        self.sim_value = None

    def on_enter(self, ctx):        
        node = ctx["node"]

        self.sim_value = "true" if ctx.get("sim", False) else "false"

        if self.sim_value == "true":
            node.get_logger().info(f"[{self.name}] Calling the service /object_id_sim")
            ctx["object_id_ready"] = False
            ctx["error_triggered"] = False

            self.client = node.create_client(SetBool, "/object_id_sim")
            request = SetBool.Request()
            request.data = True
            
            if not self.client.wait_for_service(timeout_sec=2.0):
                node.get_logger().error(f"[{self.name}] Service /object_id_sim not available.")
                ctx["error_triggered"] = True
                return
            
            self.future = self.client.call_async(request)

        else:
            #############################################
            ## SECTION FOR REAL ROBOT - NOT SIMULATION ##
            #############################################

            # Start camera, Yolo model, and object ID system here, and monitor completion
            node.get_logger().info(f"[{self.name}] Starting object ID system for real robot (not implemented in this example).")

    def run(self, ctx):     
        node = ctx["node"]

        if self.sim_value == "true":
            if self.future is None:
                node.get_logger().info(f"[{self.name}] Future is None.")
                return
            
            if self.future.done():
                result = self.future.result()
                if result and result.success:
                    node.get_logger().info(f"[{self.name}] Object ID completed correctly.")
                    ctx["object_id_ready"] = True
                    ctx["object_id_data"] = ctx.get("walls_data")
                    ctx["scan_phase"] = 2
                else:
                    node.get_logger().error(f"[{self.name}] Error while computing object ID.")
                    ctx["error_triggered"] = True
                self.future = None

        else:
            #############################################
            ## SECTION FOR REAL ROBOT - NOT SIMULATION ##
            #############################################

            # Start camera, Yolo model, and object ID system here, and set up future to monitor completion
            node.get_logger().info(f"[{self.name}] Object ID for real robot not implemented yet.")

    def check_transition(self, ctx):
        if ctx.get("object_id_ready"):
            return "GeometryReconstruction"
        if ctx.get("error_triggered"):
            return "Error"
        return None
