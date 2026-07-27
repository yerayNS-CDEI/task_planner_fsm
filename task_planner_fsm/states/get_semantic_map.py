import socket
import time

from ..state import State
from example_interfaces.srv import SetBool
from task_planner_fsm.states.proc_utils import (
    start_proc,
    stop_proc,
    wait_stack_ready,
    wait_services_ready,
    kill_stale_stack,
    SIM_STACK_PATTERNS,
)

NEXT_STATE_OPTIONS = [
    "WaitForData",
    "Error",
]

class GetSemanticMap(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /get_semantic_map")
        ctx["semantic_map_ready"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

        self.client = node.create_client(SetBool, "/get_semantic_map")
        request = SetBool.Request()
        request.data = True

        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /get_semantic_map not available.")
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
                node.get_logger().info(f"[{self.name}] Semantic map received correctly.")
                # Only leave this state once the navigation/localization stack is
                # up (mirrors the legacy ObjectID, which started the nav stack and
                # left it running for the states that follow). Gating the exit here
                # -- rather than in on_exit -- lets a stack-startup failure abort to
                # Error instead of silently proceeding without a nav stack.
                if self._start_robot_stack(ctx):
                    ctx["semantic_map_ready"] = True
                else:
                    node.get_logger().error(f"[{self.name}] Navigation stack failed to start.")
                    ctx["error_triggered"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while receiving semantic map.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if not ctx.get("semantic_map_ready") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)

    # ------------------------------------------------------------------
    # Navigation / localization stack (robo_drill)
    # ------------------------------------------------------------------
    def _start_robot_stack(self, ctx):
        """Launch the robo_drill navigation/localization stack. Returns True when ready.

        Modeled on the legacy ObjectID._start_robot_stack, but launches
        ``robo_drill move_robot.launch.py`` (this FSM's stack) instead of the
        navi_wall one, so it only passes the arguments that launch declares.
        The stack is registered under the ``nav_sim`` process key and left
        running for the states that follow.
        """
        node = ctx["node"]
        requested_sim = bool(ctx.get("sim", False))
        sim_value = "true" if requested_sim else "false"

        p = ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            node.get_logger().info(f"[{self.name}] Navigation already running (pid={p.pid}).")
            return True

        try:
            node.get_logger().info(
                f"[{self.name}] Launching robo_drill navigation stack (sim:={sim_value})..."
            )
            # Free hardware resources held by any orphaned driver from a previous
            # run (notably the SICK UDP ports 2115/2116) before relaunching, so the
            # new stack's drivers can bind instead of failing with "Address already
            # in use" and silently breaking odometry/localization.
            kill_stale_stack(node=node)
            start_proc(
                ctx,
                "nav_sim",
                [
                    "ros2", "launch", "robo_drill", "move_robot.launch.py",
                    f"sim:={sim_value}", "mode:=full", "controller_type:=omni",
                    "database_name:=rtabmap_fsm", "headless:=true",
                ],
            )

            # Wait until the stack is actually producing data instead of a blind
            # sleep. Gate on topics known to exist in this system:
            #   /clock          -> sim time is running (SIMULATION ONLY)
            #   /tf             -> robot_state_publisher / TF is up (robot model)
            #   /joint_states   -> ros2_control controllers are active
            #   /rtabmap/odom   -> rtabmap odometry is initialized and receiving data
            #
            # /clock is only published when sim time is running, so it must NOT be
            # required on the real robot.
            default_ready_topics = ["/tf", "/joint_states", "/rtabmap/odom"]
            if requested_sim:
                default_ready_topics = ["/clock"] + default_ready_topics
            required_topics = ctx.get("stack_ready_topics", default_ready_topics)
            ready_timeout = float(ctx.get("stack_ready_timeout", 90.0))
            node.get_logger().info(
                f"[{self.name}] Waiting up to {ready_timeout:.0f}s for navigation + localization stack..."
            )
            if not wait_stack_ready(ctx, required_topics, timeout=ready_timeout):
                node.get_logger().error(f"[{self.name}] Navigation + localization stack did not become ready.")
                stop_proc(ctx, "nav_sim", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)
                ctx["error_triggered"] = True
                return False

            node.get_logger().info(f"[{self.name}] Navigation + localization stack is ready.")
            return True

        except Exception as e:
            node.get_logger().error(f"[{self.name}] Could not start navigation + localization process: {e}")
            ctx["error_triggered"] = True
            return False
