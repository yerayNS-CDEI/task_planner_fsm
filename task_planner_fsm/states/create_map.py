from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, signal
import time

class CreateMap(State):
    def __init__(self, name):
        super().__init__(name)
        # self.client = None
        # self.future = None

    def on_enter(self, ctx):
        # node = ctx["node"]
        # node.get_logger().info(f"[{self.name}] Calling the service /start_mapping")
        # ctx["map_ready"] = False
        # ctx["error_triggered"] = False

        # self.client = node.create_client(SetBool, "/start_mapping")
        # request = SetBool.Request()
        # request.data = True
        
        # if not self.client.wait_for_service(timeout_sec=2.0):
        #     node.get_logger().error(f"[{self.name}] Service /start_mapping not available.")
        #     ctx["error_triggered"] = True
        #     return
        
        # self.future = self.client.call_async(request)

        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Starting the environment exploration.")
        ctx["map_ready"] = False
        ctx["error_triggered"] = False

        ## Activacion del mapping
        proc = ctx.get("mapping_proc")
        if proc and proc.poll() is None:
            node.get_logger().info(f"[{self.name}] Mapping already on going.")
        else:
            try:
                ctx["mapping_proc"] = subprocess.Popen(
                    ["ros2", "launch", "navi_wall", "global_exploration.launch.py"],
                    preexec_fn=os.setsid,  # crea su propio grupo de procesos (para matar como Ctrl+C)
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT
                )
                node.get_logger().info(
                    f"[{self.name}] Exploration process started (pid={ctx['mapping_proc'].pid})."
                )
            except Exception as e:
                node.get_logger().error(
                    f"[{self.name}] Could not start the exploration process: {e}"
                )
                ctx["error_triggered"] = True
                return

    def run(self, ctx):        
        # if self.future is None:
        #     node.get_logger().info(f"[{self.name}] Future is None.")
        #     return
        
        # node = ctx["node"]
        # if self.future.done():
        #     result = self.future.result()
        #     if result and result.success:
        #         node.get_logger().info(f"[{self.name}] Map generated correctly.")
        #         ctx["map_ready"] = True
        #     else:
        #         node.get_logger().error(f"[{self.name}] Error while generating map.")
        #         ctx["error_triggered"] = True
        #     self.future = None

        proc = ctx.get("mapping_proc")
        if proc and proc.poll() is not None:
            # Salida limpia del launch: todos los nodos han sido cerrados
            node = ctx["node"]
            node.get_logger().info(f"[{self.name}] Exploration finished. Launch exited with code {proc.returncode}.")
            ctx["map_ready"] = True
            ctx["mapping_proc"] = None  # libera el handle

    def on_exit(self, ctx):
        ## Desactivacion del mapping
        node = ctx["node"]
        proc = ctx.get("mapping_proc")
        if proc and proc.poll() is None:
            node.get_logger().info(f"[{self.name}] Stopping exploration...")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)  # equivalente a Ctrl+C
                proc.wait(timeout=5.0)
                time.sleep(2)   # 2 second delay to avoid errors in the goals
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                node.get_logger().warn(f"[{self.name}] Forced SIGKILL to exploration.")
        ctx["mapping_proc"] = None

    def check_transition(self, ctx):
        if ctx.get("map_ready"):
            return "GeometryReconstruction"
        if ctx.get("error_triggered"):
            return "Error"
        return None
