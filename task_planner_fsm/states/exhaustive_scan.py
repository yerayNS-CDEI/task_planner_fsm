from ..state import State
from statistics import median
from typing import List, Tuple
from collections import deque
import numpy as np
import rclpy.time

from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

class ExhaustiveScan(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_pub = None
        self.goals_queue = deque()
        self.current_goal_idx = -1
        self.waiting_for_arrival = False
        self.movement_done = False
        self.tf_buffer = None
        self.tf_listener = None
        self.goals_initialized = False
        self._pending_panel_cells = None

    def _estimate_step_threshold(self, sorted_vals: List[float], resolution: float = None) -> float:
        if len(sorted_vals) < 2:
            return float('inf')
        diffs = [abs(sorted_vals[i+1] - sorted_vals[i]) for i in range(len(sorted_vals)-1)]
        if resolution and resolution > 0:
            return 0.6 * float(resolution)
        m = median(diffs) if diffs else 0.0
        return max(1e-6, 1.5 * m)

    def _serpentine_order_vertical_z(self, node, tf_buffer, panel_cells_centers: List[Pose], resolution: float = None) -> List[PoseStamped]:
        """
        Transforms each cell center from map to arm_base via TF2
        (same pattern as GoalRouter.transform_goal_to_arm_base), then
        orders in a serpentine pattern:
        - v = Z in arm_base frame (rows: highest Z first)
        - u = projection of (X,Y) onto principal wall direction (PCA 2D)
        Returns list of PoseStamped in arm_base frame.
        """
        if not panel_cells_centers:
            return []

        try:
            tf = tf_buffer.lookup_transform(
                'arm_base', 'map', rclpy.time.Time(), Duration(seconds=1.0)
            )
        except Exception as e:
            node.get_logger().error(f"[{self.name}] TF lookup map->arm_base failed: {e}")
            return []

        transformed: List[PoseStamped] = []
        now = node.get_clock().now().to_msg()
        for p in panel_cells_centers:
            ps_map = PoseStamped()
            ps_map.header.frame_id = 'map'
            ps_map.header.stamp = now
            ps_map.pose = p
            ps_arm = do_transform_pose_stamped(ps_map, tf)
            ps_arm.header.frame_id = 'arm_base'
            transformed.append(ps_arm)

        X = np.array([p.pose.position.x for p in transformed], dtype=float)
        Y = np.array([p.pose.position.y for p in transformed], dtype=float)
        Z = np.array([p.pose.position.z for p in transformed], dtype=float)

        XY = np.vstack([X, Y])
        XY_centered = XY - XY.mean(axis=1, keepdims=True)
        cov = np.cov(XY_centered)
        eigvals, eigvecs = np.linalg.eigh(cov)
        u_hat = eigvecs[:, np.argmax(eigvals)]
        u_hat = u_hat / np.linalg.norm(u_hat)

        U = XY_centered.T @ u_hat

        idx = np.argsort(-Z)
        U_sorted = U[idx]
        Z_sorted = Z[idx]
        poses_sorted = [transformed[i] for i in idx]

        thr = self._estimate_step_threshold(list(Z_sorted), resolution=resolution)

        rows_idx = [[0]]
        for i in range(1, len(Z_sorted)):
            if abs(Z_sorted[i] - Z_sorted[i-1]) > thr:
                rows_idx.append([i])
            else:
                rows_idx[-1].append(i)

        ordered = []
        for r, row in enumerate(rows_idx):
            row_items = [(U_sorted[i], poses_sorted[i]) for i in row]
            row_items.sort(key=lambda t: t[0])
            if r % 2 == 1:
                row_items.reverse()
            ordered.extend([p for _, p in row_items])

        return ordered
        
    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering exhaustive scanning state.")

        if self.tf_buffer is None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, node)

        self.goal_pub = node.create_publisher(PoseStamped, '/arm/goal_pose', 10)
        self.goals_queue.clear()
        self.current_goal_idx = -1
        self.waiting_for_arrival = False
        self.movement_done = False
        self.goals_initialized = False
        self._pending_panel_cells = None
        ctx["error_triggered"] = False

        res = ctx.get("wall_discretization_results")
        if not res:
            node.get_logger().error(f"[{self.name}] Missing 'wall_discretization_results' in ctx.")
            ctx["error_triggered"] = True
            return

        selected_base_idx = ctx.get("selected_base_idx")
        if selected_base_idx is None:
            node.get_logger().error(f"[{self.name}] Missing 'selected_base_idx' in ctx.")
            ctx["error_triggered"] = True
            return
        panel_cells_centers = res.get("panel_cells_centers", [])[selected_base_idx]
        if not panel_cells_centers:
            node.get_logger().error(f"[{self.name}] No 'panel_cells_centers' to scan.")
            ctx["error_triggered"] = True
            return

        # Store cells; actual TF lookup + ordering happens in run() once the TF buffer is populated
        self._pending_panel_cells = panel_cells_centers
        node.get_logger().info(f"[{self.name}] Stored {len(panel_cells_centers)} panel cells. Waiting for TF to be ready...")

    def _send_next_goal(self, ctx):
        """Publica el siguiente goal de la cola y pone el sistema en espera de llegada."""
        node = ctx["node"]

        if not self.goals_queue:
            # No quedan objetivos → terminar
            self.movement_done = True
            node.get_logger().info(f"[{self.name}] All scan goals completed.")
            ctx["panels_left"] -= 1
            if ctx.get("panels_left", 0) <= 0:
                node.get_logger().info(f"[{self.name}] No panels left to exhaustively scan.")
                ctx["exhaustive_scan_done"] = True
            return

        # Tomamos el siguiente sin perderlo para log; luego lo extraemos
        next_goal = self.goals_queue.popleft()
        self.current_goal_idx += 1

        # Importante para evitar arrastres: el planner debe poner esto a True cuando llegue
        ctx["execution_status"] = False

        # Publicamos el Pose directamente
        self.goal_pub.publish(next_goal)
        self.waiting_for_arrival = True
        node.get_logger().info(
            f"[{self.name}] Sent goal {self.current_goal_idx + 1} "
            f"(remaining: {len(self.goals_queue)}). "
            f"pos=({next_goal.pose.position.x:.3f}, {next_goal.pose.position.y:.3f}, {next_goal.pose.position.z:.3f}). "
            f"orn=({next_goal.pose.orientation.x:.3f}, {next_goal.pose.orientation.y:.3f}, {next_goal.pose.orientation.z:.3f}, {next_goal.pose.orientation.w:.3f})"
        )

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("panels_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No panels left to exhaustively scan.")
            ctx["exhaustive_scan_done"] = True
            return

        if ctx.get("error_triggered") or self.movement_done:
            return

        # Populate queue via TF lookup on first run() tick (buffer is ready by now)
        if not self.goals_initialized:
            if self._pending_panel_cells is None:
                return
            ordered_goals = self._serpentine_order_vertical_z(node, self.tf_buffer, self._pending_panel_cells, resolution=0.1)
            if not ordered_goals:
                # TF not ready yet — retry next tick silently
                node.get_logger().debug(f"[{self.name}] TF not ready yet, retrying...")
                return
            for p in ordered_goals:
                p.pose.orientation.x = -0.5
                p.pose.orientation.y = -0.5
                p.pose.orientation.z = -0.5
                p.pose.orientation.w = -0.5
                self.goals_queue.append(p)
            self.goals_initialized = True
            node.get_logger().info(f"[{self.name}] Queued {len(self.goals_queue)} scan goals (panel_cells_centers).")
            return  # send first goal on next tick

        if not self.waiting_for_arrival:
            self._send_next_goal(ctx)
            return
        
        exec_status = ctx.get("execution_status")
        if exec_status is True:
            # Llegamos: liberamos la espera y lanzamos el siguiente
            self.waiting_for_arrival = False
            node.get_logger().info(f"[{self.name}] Goal {self.current_goal_idx + 1} reached.")
            # Opcional: si tu planner no resetea execution_status, hazlo tú
            ctx["execution_status"] = False
        elif exec_status is False or exec_status is None:
            # Seguimos esperando
            node.get_logger().debug(f"[{self.name}] Waiting for arrival confirmation...")
        else:
            # Si tu pipeline usa otros estados/valores, puedes gestionarlos aquí
            pass

    def check_transition(self, ctx):
        if self.movement_done:
            return "ArmFolding"
        if ctx.get("error_triggered"):
            return "Error"
        return None