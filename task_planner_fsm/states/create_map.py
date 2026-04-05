from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, signal, shlex
import time

from task_planner_fsm.proc_utils import start_proc, stop_proc

class CreateMap(State):
    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        node = ctx["node"]
        cmd = ctx.get('mapping_cmd')
        if isinstance(cmd, str):
            cmd = shlex.split(cmd)
        start_proc(ctx, 'mapping', cmd)
        p = ctx.get('_procs', {}).get('mapping')
        node.get_logger().info(f"[{self.name}] Exploration process started (pid={p.pid if p else 'unknown'}).")
        ctx["map_ready"] = False
        ctx["error_triggered"] = False

    def run(self, ctx):    
  
        proc = ctx.get("_procs", {}).get("mapping")
        if proc:
            rc = proc.poll()
            if rc is not None:
                node = ctx["node"]
                node.get_logger().info(f"[{self.name}] Exploration finished. Launch exited with code {rc}.")
                ctx["map_ready"] = (rc == 0)
                ctx["error_triggered"] = (rc != 0)
                # Stop with Gazebo force-kill patterns as fallback
                gazebo_patterns = ['gz sim', 'ign gazebo', 'ruby.*gz', 'gzserver', 'gz-sim']
                stop_proc(ctx, "mapping", force_kill_patterns=gazebo_patterns)

    def on_exit(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Exiting state; ensuring exploration is stopped...")
        
        #### POPEN  TO LAUNCH PROCESSES
  
        # Force kill Gazebo IMMEDIATELY to prevent launch system from escalating
        node.get_logger().info(f"[{self.name}] Force killing Gazebo processes...")
        gazebo_patterns = ['gz sim', 'ign gazebo', 'ruby.*gz', 'gzserver', 'gz-sim']
        for pattern in gazebo_patterns:
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], timeout=2, stderr=subprocess.DEVNULL)
            except:
                pass
        
        # Now stop the launch process
        stop_proc(ctx, "mapping", timeout=5.0)
        time.sleep(1)  # Brief pause to ensure cleanup

    def check_transition(self, ctx):
        if ctx.get("map_ready"):
            return "ObjectID"
        if ctx.get("error_triggered"):
            return "Error"
        return None
