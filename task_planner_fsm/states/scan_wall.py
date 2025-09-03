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

        ## Activacion del nodo de sensors_distance_orientation_sim de ur_arm_control
        if ctx.get("sensors_proc") and ctx["sensors_proc"].poll() is None:
            node.get_logger().info(f"[{self.name}] 'sensors_distance_orientation_sim' ya estaba activo.")
        else:
            try:
                ctx["sensors_proc"] = subprocess.Popen(
                    ["ros2", "run", "ur_arm_control", "sensors_distance_orientation_sim",
                    #  "--ros-args", "-p", "rate:=20.0"
                     ],
                    preexec_fn=os.setsid,  # crea su propio grupo de procesos (para matar como Ctrl+C)
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT
                )
                node.get_logger().info(
                    f"[{self.name}] 'sensors_distance_orientation_sim' activado (pid={ctx['sensors_proc'].pid})."
                )
            except Exception as e:
                node.get_logger().error(
                    f"[{self.name}] No se pudo activar 'sensors_distance_orientation_sim': {e}"
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

    def result_callback(self, future, ctx):
        node = ctx["node"]
        result = future.result().result
        status = future.result().status

        node.get_logger().info(f"[{self.name}] Scanning finished.")
        self.finished = True
        ctx["walls_left"] -= 1
    
        if ctx.get("walls_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No walls left to scan.")
            ctx["scan_done"] = True
            # self.finished = True
            return

    def on_exit(self, ctx):
        ## Desactivacion del nodo de sensors_distance_orientation_sim de ur_arm_control
        node = ctx["node"]
        proc = ctx.get("sensors_proc")
        if proc and proc.poll() is None:
            node.get_logger().info(f"[{self.name}] Deteniendo 'sensors_distance_orientation_sim'...")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)  # equivalente a Ctrl+C
                proc.wait(timeout=5.0)
                time.sleep(2)   # 2 second delay to avoid errors in the arm goals
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                node.get_logger().warn(f"[{self.name}] Forzado SIGKILL a 'sensors_distance_orientation_sim'.")
        ctx["sensors_proc"] = None

    def check_transition(self, ctx):
        if self.finished: # and not ctx.get("scan_done"):
            # return "WallTargetSelection"
            return "ArmFolding"
        # if ctx.get("scan_done"):
        #     return "HomePosition"
        if ctx.get("error_triggered"):
            return "Error"
        return None
