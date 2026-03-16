from ..state import State
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.action import GoalResponse, CancelResponse
from rclpy.task import Future
import subprocess, os, signal
from math import atan2, sin, cos
import time

class ScanWall(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False        # internal flag
        self.finished = False
        self.goal_sent = False
        self.waiting = False        
        
    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering scanning state.")
        self.started = False
        self.finished = False
        self.goal_sent = False
        self.waiting = False
        ctx["error_triggered"] = False

        ## Activacion de arduino_sensors_sim y align_ee_to_wall de arm_control.
        ## Launched here so they initialise during the scan line navigation movement.
        arm_procs = [
            ("arduino_sensors_proc", ["ros2", "run", "arm_control", "arduino_sensors_sim", "--ros-args", "-p", "autostart:=true"], "arduino_sensors_sim"),
            ("align_ee_proc",        ["ros2", "run", "arm_control", "align_ee_to_wall"],    "align_ee_to_wall"),
        ]
        for proc_key, cmd_args, proc_name in arm_procs:
            if ctx.get(proc_key) and ctx[proc_key].poll() is None:
                node.get_logger().info(f"[{self.name}] '{proc_name}' ya estaba activo (pid={ctx[proc_key].pid}).")
            else:
                try:
                    log_file = open(f"/tmp/{proc_name}.log", "w")
                    ctx[proc_key] = subprocess.Popen(
                        cmd_args,
                        preexec_fn=os.setsid,
                        stdout=log_file,
                        stderr=log_file
                    )
                    node.get_logger().info(
                        f"[{self.name}] '{proc_name}' activado (pid={ctx[proc_key].pid})."
                    )
                except Exception as e:
                    node.get_logger().error(
                        f"[{self.name}] No se pudo activar '{proc_name}': {e}"
                    )
                    ctx["error_triggered"] = True
                    return

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("walls_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No walls left to scan.")
            ctx["scan_done"] = True
            self.finished = True
            return
        
        if not self.started:
            node.get_logger().info(f"[{self.name}] Initiating wall scan maneuver...")
            self.started = True

            node.get_logger().info(f"[{self.name}] Waiting 10s for arm processes to stabilise...")
            time.sleep(10.0)

            wall_data = ctx.get("target_scan_wall", None)
            prev_target_point = ctx.get("target_scan_point", None)

            if not wall_data or not prev_target_point:
                node.get_logger().error(f"[{self.name}] Missing wall data or target point.")
                ctx["error_triggered"] = True
                return

            if all(abs(prev_target_point[i] - wall_data[0][i]) < 1e-6 for i in range(2)):
                target_point = wall_data[1]
            else:
                target_point = wall_data[0]
            other_point = prev_target_point

            dx = target_point[0] - other_point[0]
            dy = target_point[1] - other_point[1]
            yaw = atan2(dy, dx)
            qz = sin(yaw / 2.0)
            qw = cos(yaw / 2.0)

            # Construct PoseStamped goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_point[0]
            goal_msg.pose.pose.position.y = target_point[1]
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            nav_client = ctx.get("nav_client", None)
            if nav_client is None:
                node.get_logger().error(f"[{self.name}] Navigation client not found.")
                ctx["error_triggered"] = True
                return

            self._send_goal_future = nav_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            self.waiting = True

        elif self.waiting and self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                node.get_logger().error(f"[{self.name}] Goal was rejected.")
                ctx["error_triggered"] = True
                return
            node.get_logger().info(f"[{self.name}] Goal accepted. Waiting for result...")
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(lambda fut: self.result_callback(fut, ctx))

            self.waiting = False  # Dejar de esperar, ya está lanzado

    def goal_response_callback(self, future):
        pass

    def _stop_arm_processes(self, ctx):
        node = ctx["node"]
        for proc_key, proc_name in [
            ("arduino_sensors_proc", "arduino_sensors_sim"),
            ("align_ee_proc",        "align_ee_to_wall"),
        ]:
            proc = ctx.get(proc_key)
            if proc and proc.poll() is None:
                node.get_logger().info(f"[{self.name}] Deteniendo '{proc_name}'...")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    node.get_logger().warn(f"[{self.name}] Forzado SIGKILL a '{proc_name}'.")
            ctx[proc_key] = None

    def result_callback(self, future, ctx):
        node = ctx["node"]
        result = future.result().result
        status = future.result().status

        node.get_logger().info(f"[{self.name}] Scanning finished. Stopping arm processes for safety...")
        self._stop_arm_processes(ctx)
        time.sleep(2)   # delay to avoid errors in the arm goals
        self.finished = True
        ctx["walls_left"] -= 1
    
        if ctx.get("walls_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No walls left to scan.")
            ctx["scan_done"] = True
            return

    def on_exit(self, ctx):
        ## Desactivacion de arduino_sensors_sim y align_ee_to_wall (safety net)
        self._stop_arm_processes(ctx)

    def check_transition(self, ctx):
        if self.finished: # and not ctx.get("scan_done"):
            # return "WallTargetSelection"
            return "ArmFolding"
        # if ctx.get("scan_done"):
        #     return "HomePosition"
        if ctx.get("error_triggered"):
            return "Error"
        return None
