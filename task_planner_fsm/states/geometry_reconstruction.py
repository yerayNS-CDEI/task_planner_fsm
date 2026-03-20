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
        if not requested_sim:
            node.get_logger().warn(f"[{self.name}] FSM was started with sim=false, but hybrid_sim=true requires sim=true. Forcing sim:=true for move_robot.launch.py.")
        sim_value = "true"
        ## Activacion de la simulación de navegación
        p = ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            node.get_logger().info(f"[{self.name}] Navigation already running (pid={p.pid}).")
        else:
            try:
                # Check if URSim is reachable (hybrid mode requires URSim on 192.168.56.101)
                ursim_ip = "192.168.56.101"
                ursim_port = 29999  # UR Dashboard port
                node.get_logger().info(f"[{self.name}] Checking URSim connectivity at {ursim_ip}:{ursim_port}...")
                
                ursim_ready = False
                for attempt in range(5):  # Increased from 3 to 5 attempts
                    try:
                        sock = socket.create_connection((ursim_ip, ursim_port), timeout=3)  # Increased timeout
                        sock.close()
                        ursim_ready = True
                        node.get_logger().info(f"[{self.name}] URSim is reachable at {ursim_ip}")
                        # Give URSim extra time to stabilize after connection
                        time.sleep(2)
                        break
                    except (socket.timeout, socket.error) as e:
                        node.get_logger().warn(
                            f"[{self.name}] URSim not reachable (attempt {attempt+1}/5): {e}"
                        )
                        if attempt < 4:  # Don't sleep on last attempt
                            time.sleep(3)  # Wait longer between attempts
                
                if not ursim_ready:
                    node.get_logger().error(
                        f"[{self.name}] URSim not reachable at {ursim_ip}:{ursim_port} after 5 attempts. "
                        f"Ensure terminal 1 has: ros2 run ur_client_library start_ursim.sh -m ur10e -v 5.17.3"
                    )
                    # Continue anyway - the TF check will catch the issue
                
                # Launch navigation stack
                node.get_logger().info(f"[{self.name}] Launching navigation stack with hybrid_sim=true...")
                start_proc(
                    ctx, "nav_sim",
                    ["ros2", "launch", "navi_wall", "move_robot.launch.py",
                    f"sim:={sim_value}", "mode:=full", "controller_type:=omni", "hybrid_sim:=true", 
                    f"robot_ip:={ursim_ip}", "database_name:=rtabmap", "headless:=true",
                    "use_sim_time:=true"]
                )
                
                # Increased initial wait to allow processes to start
                node.get_logger().info(f"[{self.name}] Waiting 10 seconds for processes to initialize...")
                time.sleep(10)  # Increased from 7 to 10 seconds
                node.get_logger().info(f"[{self.name}] Navigation + localization simulation started.")
                
                # Wait for TF tree to be established (map -> arm_base)
                # This is critical in hybrid mode to ensure arm TFs are ready
                node.get_logger().info(f"[{self.name}] Waiting for TF tree (map -> arm_base) to be available...")
                tf_buffer = ctx.get("tf_buffer")
                if tf_buffer:
                    max_wait_time = 45.0  # Maximum 45 seconds (increased for URSim connection)
                    start_time = time.time()
                    tf_available = False
                    
                    # Intermediate frames to check for diagnostic purposes
                    check_frames = [
                        ('map', 'odom', 'Localizer'),
                        ('odom', 'base_footprint', 'Base odometry'),
                        ('base_footprint', 'column_link', 'Base robot_state_publisher'),
                        ('column_link', 'world', 'Static TF bridge'),
                        ('world', 'arm_base_link', 'Arm root frame'),
                        ('arm_base_link', 'arm_base', 'Arm robot_state_publisher'),
                        ('map', 'arm_base', 'Complete chain'),
                    ]
                    
                    last_report_time = start_time
                    while (time.time() - start_time) < max_wait_time:
                        # Check main transform
                        # Use rclpy.time.Time() to get latest available transform (works with both sim and real time)
                        try:
                            transform_available = tf_buffer.can_transform(
                                'map', 'arm_base', 
                                rclpy.time.Time(), 
                                timeout=Duration(seconds=0.5)
                            )
                            if transform_available:
                                tf_available = True
                                elapsed = time.time() - start_time
                                node.get_logger().info(
                                    f"[{self.name}] TF tree ready after {elapsed:.1f}s. Transform map->arm_base is available."
                                )
                                break
                        except Exception as e:
                            node.get_logger().debug(f"[{self.name}] TF check exception: {e}")
                        
                        # Report diagnostic every 5 seconds
                        current_time = time.time()
                        if current_time - last_report_time >= 5.0:
                            node.get_logger().info(
                                f"[{self.name}] Still waiting... ({current_time - start_time:.1f}s elapsed). Checking TF chain:"
                            )
                            for parent, child, description in check_frames:
                                try:
                                    # Use Time() for latest available, with timeout for proper checking
                                    available = tf_buffer.can_transform(
                                        parent, child, 
                                        rclpy.time.Time(), 
                                        timeout=Duration(seconds=0.1)
                                    )
                                    status = "✓" if available else "✗"
                                    node.get_logger().info(f"  {status} {parent} -> {child} ({description})")
                                except Exception as e:
                                    node.get_logger().info(f"  ✗ {parent} -> {child} ({description}) - Error: {e}")
                            last_report_time = current_time
                        
                        time.sleep(0.5)
                    
                    if not tf_available:
                        node.get_logger().error(
                            f"[{self.name}] TF tree (map->arm_base) not available after {max_wait_time}s!"
                        )
                        node.get_logger().error(f"[{self.name}] Final TF chain status:")
                        for parent, child, description in check_frames:
                            try:
                                # Use Time() for latest available transform
                                available = tf_buffer.can_transform(
                                    parent, child, 
                                    rclpy.time.Time(), 
                                    timeout=Duration(seconds=0.1)
                                )
                                status = "✓" if available else "✗"
                                node.get_logger().error(f"  {status} {parent} -> {child} ({description})")
                            except Exception as e:
                                node.get_logger().error(f"  ✗ {parent} -> {child} ({description}) - Error: {e}")
                        
                        node.get_logger().warn(
                            f"[{self.name}] Check: 1) URSim is running (terminal 1), "
                            "2) Arm driver connected, 3) Static TF publisher active. "
                            "Proceeding anyway, but arm WILL NOT appear in RViz correctly."
                        )
                else:
                    node.get_logger().warn(f"[{self.name}] TF buffer not available in context, skipping TF check.")
                                
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
