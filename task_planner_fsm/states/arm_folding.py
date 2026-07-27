from ..state import State
from arm_control.srv import SendPosition
from control.remote_DASHBOARD import send_dashboard_play_command
from collections import deque

class ArmFolding(State):
    def __init__(self, name):
        super().__init__(name)
        self.service_client = None
        self.movement_done = False
        self.future = None
        self.verbose = False
        self.goals_queue = deque()
        self.current_goal = None
        self.dashboard_sent = False
        self.goals_sent = 0
        self.goals_completed = 0
        self.total_goals = 1
        self.service_wait_deadline = None
        self.service_wait_timeout_s = 30.0
        
    def on_enter(self, ctx):
        self.movement_done = False
        self.verbose = False
        self.dashboard_sent = False
        self.goals_sent = 0
        self.goals_completed = 0
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering folding state.")
        
        # Create service client for position_sender_node
        self.service_client = node.create_client(SendPosition, '/send_position')
        
        ctx["folding_success"] = False
        ctx["error_triggered"] = False

        # Define the sequence of movements: only fold
        self.goals_queue.clear()
        self.goals_queue.append('folded_fsm')
        
        self.current_goal = None
        self.future = None
        self.service_wait_deadline = None
        self.service_wait_timeout_s = 30.0
            
    def _send_next_goal(self, ctx):
        """Sends the next goal via service call."""
        node = ctx["node"]

        if not self.goals_queue:
            return

        # Check service availability before committing to this goal
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

        # Service ready — commit and send
        self.service_wait_deadline = None
        self.current_goal = self.goals_queue.popleft()
        self.goals_sent += 1

        request = SendPosition.Request()
        request.position_name = self.current_goal

        # Reset execution signals before sending so we wait for a FRESH result
        # rather than a stale True/failure left over from a previous state.
        ctx["execution_status"] = False
        ctx["planner_goal_failed"] = False

        self.future = self.service_client.call_async(request)
        node.get_logger().info(
            f"[{self.name}] Sending service request for position: {self.current_goal} "
            f"(goal {self.goals_sent}/{self.total_goals})"
        )

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Folding the arm back to the transport pose")

        if ctx.get("error_triggered") or self.movement_done:
            return

        # Send dashboard command first (before any service request)
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

        # If no future, send next goal
        if self.future is None:
            self._send_next_goal(ctx)
            self.verbose = False  # Reset verbose flag for new goal
            return
        
        # Check if service call is complete
        if not self.future.done():
            return
        
        # Get service response (only once per goal)
        if self.future is not None and not hasattr(self, '_service_response_processed'):
            try:
                response = self.future.result()
                if not response.success:
                    node.get_logger().error(f"[{self.name}] Service call failed: {response.message}")
                    ctx["error_triggered"] = True
                    return
                
                node.get_logger().info(f"[{self.name}] Service call successful: {response.message}")
                self._service_response_processed = True
                
            except Exception as e:
                node.get_logger().error(f"[{self.name}] Service call exception: {str(e)}")
                ctx["error_triggered"] = True
                return
        
        # Wait for execution to complete.  /execution_status only goes True on
        # actual success; failures arrive on /planner/goal_failed, so check that
        # signal first to avoid hanging on a True that will never come.
        if ctx.get("planner_goal_failed"):
            ctx["planner_goal_failed"] = False
            node.get_logger().error(
                f"[{self.name}] Planner reported failure while moving to {self.current_goal}."
            )
            ctx["error_triggered"] = True
            return

        exec_status = ctx.get("execution_status")
        if exec_status is True:
            # Movement completed successfully
            self.goals_completed += 1
            node.get_logger().info(
                f"[{self.name}] Position {self.current_goal} reached "
                f"(completed {self.goals_completed}/{self.total_goals})."
            )

            # Clear future and flags now that movement is complete
            self.future = None
            if hasattr(self, '_service_response_processed'):
                delattr(self, '_service_response_processed')
            self.verbose = False

            # Reset execution status for next goal
            ctx["execution_status"] = False

            # Check if all goals are completed
            if self.goals_completed >= self.total_goals:
                self.movement_done = True
                node.get_logger().info(f"[{self.name}] All folding movements completed.")
            # Note: do NOT call _send_next_goal() here, let next run() cycle handle it
        elif exec_status is False or exec_status is None:
            # Still waiting for arrival
            if not self.verbose:
                node.get_logger().info(f"[{self.name}] Waiting for arm to reach {self.current_goal}...")
                self.verbose = True

    def check_transition(self, ctx):        
        if self.movement_done and not ctx.get("scan_done"):
            return "WallTargetSelection"
        if self.movement_done and ctx.get("scan_done"):
            return "HomePosition"
        if ctx.get("error_triggered"):
            return "Error"
        
        return None
