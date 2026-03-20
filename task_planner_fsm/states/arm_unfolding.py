from ..state import State
from arm_control.srv import SendPosition

class ArmUnfolding(State):
    def __init__(self, name):
        super().__init__(name)
        self.service_client = None
        self.goal_sent = False
        self.movement_done = False
        self.future = None
        self.verbose = False
        
    def on_enter(self, ctx):
        self.goal_sent = False
        self.movement_done = False
        self.verbose = False
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering unfolding state.")
        
        # Create service client for position_sender_node
        self.service_client = node.create_client(SendPosition, '/arm/send_position')
        
        ctx["unfolding_success"] = False
        ctx["error_triggered"] = False
        self.future = None

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("error_triggered") or self.movement_done:
            return
        
        # Send service request if not already sent
        if not self.goal_sent:
            # Wait for service to be available
            if not self.service_client.wait_for_service(timeout_sec=1.0):
                node.get_logger().warn(f"[{self.name}] Service /arm/send_position not available")
                ctx["error_triggered"] = True
                return
            
            # Create service request
            request = SendPosition.Request()
            request.position_name = 'unfolded_fsm'
            
            # Reset execution status before sending
            ctx["execution_status"] = False
            
            # Send async service call
            self.future = self.service_client.call_async(request)
            node.get_logger().info(f"[{self.name}] Sending service request for position: unfolded_fsm")
            self.goal_sent = True
            return
        
        # Check if service call is complete
        if self.future is not None and not self.future.done():
            return
        
        # Get service response (only once)
        if self.future is not None:
            try:
                response = self.future.result()
                if not response.success:
                    node.get_logger().error(f"[{self.name}] Service call failed: {response.message}")
                    ctx["error_triggered"] = True
                    return
                
                node.get_logger().info(f"[{self.name}] Service call successful: {response.message}")
                self.future = None  # Clear future after processing
                
            except Exception as e:
                node.get_logger().error(f"[{self.name}] Service call exception: {str(e)}")
                ctx["error_triggered"] = True
                return
        
        # Wait for execution to complete
        exec_status = ctx.get("execution_status")
        if exec_status is True:
            # Movement completed
            self.movement_done = True
            node.get_logger().info(f"[{self.name}] Position unfolded_fsm reached.")
        elif exec_status is False or exec_status is None:
            # Still waiting for arrival
            if not self.verbose:
                node.get_logger().info(f"[{self.name}] Waiting for arm to reach unfolded_fsm...")
                self.verbose = True

    def check_transition(self, ctx):
        if self.movement_done and ctx.get("scan_phase") == 1:
            return "ScanWall"
        if self.movement_done and ctx.get("scan_phase") == 2:
            return "ExhaustiveScan"
        if ctx.get("error_triggered"):
            return "Error"
        return None
