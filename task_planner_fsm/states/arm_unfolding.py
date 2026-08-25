from ..state import State
from arm_control.srv import SendPosition
from control.remote_DASHBOARD import send_dashboard_play_command

class ArmUnfolding(State):
    def __init__(self, name):
        super().__init__(name)
        self.service_client = None
        self.goal_sent = False
        self.movement_done = False
        self.future = None
        self.verbose = False
        self.dashboard_sent = False
        self.service_wait_deadline = None
        self.service_wait_timeout_s = 30.0

    def on_enter(self, ctx):
        self.goal_sent = False
        self.movement_done = False
        self.verbose = False
        self.dashboard_sent = False
        self.service_wait_deadline = None
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering unfolding state.")

        # Create service client for position_sender_node
        self.service_client = node.create_client(SendPosition, '/send_position')

        ctx["unfolding_success"] = False
        ctx["error_triggered"] = False
        self.future = None

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Unfolding the arm into the scanning pose")

        if ctx.get("error_triggered") or self.movement_done:
            return

        # Send dashboard command first (before service request)
        # Skip in Gazebo simulation — only needed for real robot
        if not self.dashboard_sent:
            if ctx.get("sim", False):
                node.get_logger().info(f"[{self.name}] Gazebo simulation: skipping dashboard play command.")
                self.dashboard_sent = True
            else:
                robot_ip = str(ctx.get("robot_ip", "192.168.1.102"))
                robot_port = int(ctx.get("robot_port", 29999))
                node.get_logger().info(
                    f"[{self.name}] Sending dashboard play command to UR robot at {robot_ip}:{robot_port}..."
                )
                success, message = send_dashboard_play_command(host=robot_ip, port=robot_port)
                if success:
                    node.get_logger().info(f"[{self.name}] Dashboard command successful: {message}")
                    self.dashboard_sent = True
                else:
                    node.get_logger().error(f"[{self.name}] Dashboard command failed: {message}")
                    ctx["error_triggered"] = True
                    return

        # ScanWall owns the unfold for wall scans, so don't do it here.
        # It folds the arm again immediately: the base still has to slide from
        # NavigateToTarget's coarse standoff to the first segment's scanning
        # standoff, and that transit runs folded. Unfolding here therefore buys
        # unfold -> fold -> unfold with nothing in between — measured at 87 s +
        # 89 s of wasted arm motion per scan line in simulation. Hand the arm
        # over folded instead and let ScanWall unfold once, after the base has
        # arrived. Floor and ceiling scans (phases 2 and 3) have no such transit
        # and still unfold here.
        #
        # The dashboard play command above has already run: it starts the UR
        # program on the real robot and has nothing to do with arm position.
        if ctx.get("scan_phase") == 1 and not ctx.get("arm_unfolding_force_unfold", False):
            node.get_logger().info(
                f"[{self.name}] Wall scan: leaving the arm folded — ScanWall unfolds "
                f"it once the base has reached the first segment."
            )
            ctx["unfolding_success"] = True
            self.movement_done = True
            return

        # Send service request if not already sent
        if not self.goal_sent:
            if not self.service_client.service_is_ready():
                if self.service_wait_deadline is None:
                    self.service_wait_deadline = (
                        node.get_clock().now().nanoseconds / 1e9 + self.service_wait_timeout_s
                    )
                    node.get_logger().warn(
                        f"[{self.name}] Waiting up to {self.service_wait_timeout_s:.0f}s "
                        f"for service /send_position..."
                    )
                elif node.get_clock().now().nanoseconds / 1e9 > self.service_wait_deadline:
                    node.get_logger().error(
                        f"[{self.name}] Service /send_position not available after "
                        f"{self.service_wait_timeout_s:.0f}s."
                    )
                    ctx["error_triggered"] = True
                    self.service_wait_deadline = None
                return  # retry next tick
            self.service_wait_deadline = None
            
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
        
        # Wait for execution to complete.  The planner publishes /execution_status
        # = True only on actual success; failures arrive on /planner/goal_failed.
        # Check the failure signal first so we don't hang waiting for a True that
        # will never come.
        if ctx.get("planner_goal_failed"):
            ctx["planner_goal_failed"] = False
            node.get_logger().error(
                f"[{self.name}] Planner reported failure while moving to unfolded_fsm."
            )
            ctx["error_triggered"] = True
            return

        exec_status = ctx.get("execution_status")
        if exec_status is True:
            # Movement completed successfully
            self.movement_done = True
            ctx["execution_status"] = False  # Reset for next state
            node.get_logger().info(f"[{self.name}] Position unfolded_fsm reached. Movement complete.")
        elif exec_status is False or exec_status is None:
            # Still waiting for arrival
            if not self.verbose:
                node.get_logger().info(f"[{self.name}] Waiting for arm to reach unfolded_fsm...")
                self.verbose = True

    def check_transition(self, ctx):
        if self.movement_done and ctx.get("scan_phase") == 1:
            return "ScanWall"
        if self.movement_done and ctx.get("scan_phase") == 2:
            return "FloorScan"
        if self.movement_done and ctx.get("scan_phase") == 3:
            return "CeilingScan"
        if ctx.get("error_triggered"):
            return "Error"
        return None
