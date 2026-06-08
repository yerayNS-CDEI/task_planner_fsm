from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, time, socket
import rclpy.time
from rclpy.duration import Duration

from task_planner_fsm.states.proc_utils import start_proc

class SensorDataProcessing(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /sensor_data_processing")
        ctx["data_processed"] = False
        ctx["drilling_required"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/sensor_data_processing")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /sensor_data_processing not available.")
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
                node.get_logger().info(f"[{self.name}] Sensor data processed correctly.")
                ctx["data_processed"] = True
                ctx["drilling_required"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while processing sensor data.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        if not ctx.get("data_processed"):
            return None
        if ctx.get("drilling_required"):
            return "SendDataToPokeye"
        return "ArmFolding"
