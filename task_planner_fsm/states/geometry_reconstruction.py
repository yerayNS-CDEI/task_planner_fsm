from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, time

from task_planner_fsm.states.proc_utils import start_proc

class GeometryReconstruction(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

    def on_enter(self, ctx):
        time.sleep(5)
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /start_geometry_reconstruction")
        ctx["reconstruction_ready"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/start_geometry_reconstruction")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /start_geometry_reconstruction not available.")
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
                node.get_logger().info(f"[{self.name}] Geometry reconstructed correctly.")
                ctx["reconstruction_ready"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while reconstructing geometry.")
                ctx["error_triggered"] = True
            self.future = None

    def on_exit(self, ctx): 
        node = ctx["node"]
        ## Activacion de la simulación de navegación
        p = ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            node.get_logger().info(f"[{self.name}] Navigation already running (pid={p.pid}).")
        else:
            try:
                start_proc(
                    ctx, "nav_sim",
                    ["ros2", "launch", "navi_wall", "move_robot.launch.py",
                    "sim:=true", "world:=warehouse_v2", "database_name:=rtabmap",
                    "use_sim_time:=true"]
                )
                time.sleep(5)
                node.get_logger().info(f"[{self.name}] Navigation + localization simulation started.")
                time.sleep(2)
                start_proc(
                    ctx, "arm_sim",
                    ["ros2", "launch", "ur_arm_control", "general.launch.py", "arm_use_sim_time:=true"]
                )
                time.sleep(5)
                node.get_logger().info(f"[{self.name}] Arm manipulator simulation started.")
                
            except Exception as e:
                node.get_logger().error(
                    f"[{self.name}] Could not start the navigation + localization process: {e}"
                )
                ctx["error_triggered"] = True
                return

    def check_transition(self, ctx):
        if ctx.get("reconstruction_ready"):
            return "ComputeWallPoints"
        if ctx.get("error_triggered"):
            return "Error"
        return None
