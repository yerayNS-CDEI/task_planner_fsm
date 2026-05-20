from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, time, socket
import rclpy.time
from rclpy.duration import Duration

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
        requested_sim = bool(ctx.get("sim", False))
        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()
        if not requested_sim:
            node.get_logger().warn(f"[{self.name}] FSM was started with sim=false, forcing sim:=true for Gazebo navigation launch.")
        sim_value = "true"
        ## Activacion de la simulación de navegación
        p = ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            node.get_logger().info(f"[{self.name}] Navigation already running (pid={p.pid}).")
        else:
            try:
                # Check if UR robot is reachable before launching
                robot_ip = "192.168.1.102"
                robot_port = 29999  # UR Dashboard port
                node.get_logger().info(f"[{self.name}] Checking UR robot connectivity at {robot_ip}:{robot_port}...")

                robot_ready = False
                for attempt in range(5):
                    try:
                        sock = socket.create_connection((robot_ip, robot_port), timeout=3)
                        sock.close()
                        robot_ready = True
                        node.get_logger().info(f"[{self.name}] UR robot is reachable at {robot_ip}")
                        time.sleep(2)
                        break
                    except (socket.timeout, socket.error) as e:
                        node.get_logger().warn(
                            f"[{self.name}] UR robot not reachable (attempt {attempt+1}/5): {e}"
                        )
                        if attempt < 4:
                            time.sleep(3)

                if not robot_ready:
                    node.get_logger().error(
                        f"[{self.name}] UR robot not reachable at {robot_ip}:{robot_port} after 5 attempts. "
                        f"Ensure the robot is powered on and network is configured correctly."
                    )
                    # Continue anyway - the TF check will catch the issue

                # Launch navigation stack with Gazebo
                node.get_logger().info(f"[{self.name}] Launching navigation stack with Gazebo (hybrid_sim=false)...")
                start_proc(
                    ctx, "nav_sim",
                    ["ros2", "launch", "navi_wall", "move_robot.launch.py",
                    f"sim:={sim_value}", "mode:=full", "controller_type:=omni", "hybrid_sim:=false",
                    f"robot_ip:={robot_ip}", "database_name:=rtabmap_fsm", "headless:=true",
                    f"planner_backend:={planner_backend}",
                    "use_sim_time:=true"]
                )
                
                # Wait for processes to initialize
                node.get_logger().info(f"[{self.name}] Waiting 10 seconds for processes to initialize...")
                time.sleep(10)
                node.get_logger().info(f"[{self.name}] Navigation + localization simulation started.")
                                
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
