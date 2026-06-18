from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, signal, shlex
import time

from task_planner_fsm.states.proc_utils import (
    start_proc,
    stop_proc,
    wait_processes_gone,
    SIM_STACK_PATTERNS,
)

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
                # Graceful group shutdown; on_exit does the thorough wait.
                stop_proc(ctx, "mapping", force_kill_patterns=SIM_STACK_PATTERNS)

    def on_exit(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Exiting state; tearing down the mapping stack cleanly...")

        # 1) Graceful, whole-process-group shutdown of the mapping launch
        #    (SIGINT -> SIGTERM -> SIGKILL). This lets ros2 launch shut its
        #    children down cleanly so DDS participants deregister and rtabmap
        #    releases its sqlite database, instead of -9'ing Gazebo only and
        #    orphaning robot_state_publisher / controllers / rtabmap / rviz.
        stop_proc(ctx, "mapping", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)

        # 2) Wait until every node from the mapping launch is actually gone, so
        #    the next launch (ObjectID) does not collide with orphaned nodes or a
        #    locked rtabmap DB. Fixed sleeps are replaced by this real check.
        if not wait_processes_gone(SIM_STACK_PATTERNS, timeout=3.0, node=node):
            node.get_logger().warn(
                f"[{self.name}] Sim-stack processes still present after teardown; force-killing stragglers..."
            )
            for pattern in SIM_STACK_PATTERNS:
                try:
                    subprocess.run(['pkill', '-9', '-f', pattern], timeout=2, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
            wait_processes_gone(SIM_STACK_PATTERNS, timeout=10.0, node=node)

        # 3) Small settle so DDS discovery clears any stale endpoints before the
        #    new stack starts publishing.
        time.sleep(3)
        node.get_logger().info(f"[{self.name}] Mapping stack fully stopped.")

    def check_transition(self, ctx):
        if ctx.get("map_ready"):
            return "ObjectID"
        if ctx.get("error_triggered"):
            return "Error"
        return None
