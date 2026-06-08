from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, time, socket
import rclpy.time
from rclpy.duration import Duration

from task_planner_fsm.states.proc_utils import start_proc

class SendDataToPokeye(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /send_data_to_pokeye")
        ctx["interest_areas_ready"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/send_data_to_pokeye")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /send_data_to_pokeye not available.")
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
                node.get_logger().info(f"[{self.name}] Data sent to Pokeye correctly.")
                ctx["data_sent"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while sending data to Pokeye.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("data_sent"):
            return "ArmFolding"
        if ctx.get("error_triggered"):
            return "Error"
        return None
