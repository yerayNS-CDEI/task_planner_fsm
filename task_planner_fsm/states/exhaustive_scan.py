from ..state import State
from statistics import median
from typing import List, Tuple
from collections import deque
import numpy as np

from geometry_msgs.msg import Pose

class ExhaustiveScan(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_pub = None
        self.goals_queue = deque()
        self.current_goal_idx = -1
        self.waiting_for_arrival = False
        self.movement_done = False
        
    def on_enter(self, ctx):
        # def _pose_to_tuple(p: Pose) -> Tuple[float, float, float, Pose]:
        #     return (float(p.position.x), float(p.position.y), float(p.position.z), p)

        # def _infer_grid_axes(points_xyz: List[Tuple[float, float, float]], axes_hint=None):
        #     """
        #     Devuelve dos índices de ejes (u_idx, v_idx) dentro de (x=0,y=1,z=2) que forman el plano de la pared.
        #     - Si axes_hint = ('x','y') o similar, se respeta.
        #     - Si no, asume que el eje con **menor varianza** es perpendicular a la pared y los otros dos forman el plano.
        #     Dentro del plano, tomamos u como el eje con **mayor varianza** (columnas) y v como el otro (filas).
        #     """
        #     if axes_hint:
        #         name_to_idx = {'x':0, 'y':1, 'z':2}
        #         u_idx = name_to_idx[axes_hint[0]]
        #         v_idx = name_to_idx[axes_hint[1]]
        #         return u_idx, v_idx

        #     # Calcular varianzas simples
        #     xs = [p[0] for p in points_xyz]
        #     ys = [p[1] for p in points_xyz]
        #     zs = [p[2] for p in points_xyz]
        #     def _var(arr):
        #         m = sum(arr)/len(arr)
        #         return sum((a-m)*(a-m) for a in arr)/max(1, (len(arr)-1))

        #     vars_ = [ _var(xs), _var(ys), _var(zs) ]
        #     # eje perpendicular = mínima varianza
        #     perp_idx = min(range(3), key=lambda i: vars_[i])
        #     plane_axes = [i for i in (0,1,2) if i != perp_idx]
        #     # Dentro del plano, u = mayor varianza, v = menor (así columnas varían más)
        #     u_idx = max(plane_axes, key=lambda i: vars_[i])
        #     v_idx = [i for i in plane_axes if i != u_idx][0]
        #     return u_idx, v_idx

        # def _estimate_step_threshold(sorted_vals: List[float], resolution: float = None) -> float:
        #     """
        #     Estima un umbral para decidir cortes de fila a partir de las diferencias entre valores consecutivos.
        #     Si 'resolution' está disponible (la de tu servicio), usa ~0.6*resolution como base.
        #     Si no, estima a partir de la mediana de las diferencias “pequeñas”.
        #     """
        #     if len(sorted_vals) < 2:
        #         return float('inf')
        #     diffs = [abs(sorted_vals[i+1] - sorted_vals[i]) for i in range(len(sorted_vals)-1)]

        #     if resolution and resolution > 0:
        #         return 0.6 * resolution  # tolerancia robusta

        #     # Estimación robusta: mediana de diffs y un factor
        #     m = median(diffs)
        #     # Si hay gaps grandes entre filas, m suele captar el paso pequeño dentro de fila
        #     # Multiplicamos por 1.5 para separar bien filas
        #     return max(1e-6, 1.5 * m)

        def _estimate_step_threshold(sorted_vals: List[float], resolution: float = None) -> float:
            if len(sorted_vals) < 2:
                return float('inf')
            diffs = [abs(sorted_vals[i+1] - sorted_vals[i]) for i in range(len(sorted_vals)-1)]
            if resolution and resolution > 0:
                return 0.6 * float(resolution)
            m = median(diffs) if diffs else 0.0
            return max(1e-6, 1.5 * m)

        # def serpentine_order(panel_cells_centers: List[Pose], resolution: float = None, axes_hint: Tuple[str,str]=None) -> List[Pose]:
        #     """
        #     Devuelve la lista de Poses en orden serpentina (fila1 L->R, fila2 R->L, ...).
        #     - resolution: si la conoces (p.ej. ctx['discretization_resolution']), mejora la detección de filas.
        #     - axes_hint: tu pista de ejes del plano, p.ej. ('x','y') o ('y','z').
        #     """
        #     if not panel_cells_centers:
        #         return []

        #     # Convertimos a tuplas para análisis
        #     xyzp = [ _pose_to_tuple(p) for p in panel_cells_centers ]  # (x,y,z,Pose)

        #     # 1) Elegir ejes del plano (u=columnas, v=filas)
        #     u_idx, v_idx = _infer_grid_axes([(x,y,z) for (x,y,z,_) in xyzp], axes_hint=axes_hint)

        #     # 2) Orden preliminar por v para agrupar por filas
        #     xyzp.sort(key=lambda t: t[v_idx])

        #     # 3) Cortes de fila por gaps en v
        #     v_vals = [t[v_idx] for t in xyzp]
        #     thr = _estimate_step_threshold(v_vals, resolution=resolution)

        #     rows: List[List[Tuple[float,float,float,Pose]]] = []
        #     current_row = [xyzp[0]]
        #     for i in range(1, len(xyzp)):
        #         if abs(v_vals[i] - v_vals[i-1]) > thr:
        #             rows.append(current_row)
        #             current_row = [xyzp[i]]
        #         else:
        #             current_row.append(xyzp[i])
        #     rows.append(current_row)
        #     # rows.reverse()

        #     # 4) Ordenar cada fila por u y alternar sentido
        #     ordered: List[Pose] = []
        #     for r_idx, row in enumerate(rows):
        #         row.sort(key=lambda t: t[u_idx])
        #         if r_idx % 2 == 1:
        #             row.reverse()  # serpentina: filas impares van al revés
        #         ordered.extend([t[3] for t in row])  # extraer Pose

        #     return ordered

        def serpentine_order_vertical_z(current_position, panel_cells_centers: List[Pose], resolution: float = None) -> List[Pose]:
            """
            Ordena en serpentina asumiendo:
            - v = Z (vertical, filas: de mayor Z a menor Z → “superior” primero)
            - u = proyección de (X,Y) sobre la dirección principal del muro (PCA 2D)
            Devuelve lista de Pose en orden S: fila1 izq→der, fila2 der→izq, ...
            """
            if not panel_cells_centers:
                return []

            X = np.array([p.position.x for p in panel_cells_centers], dtype=float)
            Y = np.array([p.position.y for p in panel_cells_centers], dtype=float)
            Z = np.array([p.position.z for p in panel_cells_centers], dtype=float)
            z_min = min(Z)
            # Z -= z_min
            # X -= current_position.x
            # Y -= current_position.y
            # print("Z: ", Z)
            # print("X: ", X)
            # print("Y: ", Y)            

            # PCA 2D en (X,Y) para hallar dirección principal û
            XY = np.vstack([X, Y])  # shape (2, N)
            XY_centered = XY - XY.mean(axis=1, keepdims=True)
            cov = np.cov(XY_centered)
            # eigenval/eigenvec de cov (2x2); elegimos el autovector de mayor autovalor
            eigvals, eigvecs = np.linalg.eigh(cov)
            u_hat = eigvecs[:, np.argmax(eigvals)]  # vector unitario en XY (columna)
            u_hat = u_hat / np.linalg.norm(u_hat)

            # Proyección escalar u = <(X,Y), û>
            U = XY_centered.T @ u_hat  # shape (N,)

            # Orden preliminar por Z descendente (fila superior primero)
            idx = np.argsort(-Z)
            U_sorted = U[idx]
            Z_sorted = Z[idx]
            poses_sorted = [panel_cells_centers[i] for i in idx]
            
            poses_sorted = []
            for i, element in enumerate(idx,0):
                # print("Panel cell centers BEFORE: ", panel_cells_centers[i])
                panel_cells_centers[i].position.x -= current_position.x
                panel_cells_centers[i].position.y -= current_position.y
                panel_cells_centers[i].position.z -= z_min
                # print("Panel cell centers AFTER: ", panel_cells_centers[i])
            poses_sorted = [panel_cells_centers[i] for i in idx]
            # print("Coordinates example: ",panel_cells_centers[i].position.x)
            # print("POSES SORTED: ",poses_sorted)

            # Estimar umbral para separar filas por Z
            thr = _estimate_step_threshold(list(Z_sorted), resolution=resolution)

            # Particionar en filas (por gaps en Z)
            rows_idx = [[0]]
            for i in range(1, len(Z_sorted)):
                if abs(Z_sorted[i] - Z_sorted[i-1]) > thr:
                    rows_idx.append([i])
                else:
                    rows_idx[-1].append(i)

            # Orden serpentina: por fila, ordenar por U asc y alternar sentido
            ordered = []
            for r, row in enumerate(rows_idx):
                # pares (U_val, Pose) de la fila
                row_items = [(U_sorted[i], poses_sorted[i]) for i in row]
                row_items.sort(key=lambda t: t[0])  # izq→der según u ascendente
                if r % 2 == 1:
                    row_items.reverse()  # serpentina: fila impar va al revés
                ordered.extend([p for _, p in row_items])

            return ordered

        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering exhaustive scanning state.")
        
        self.goal_pub = node.create_publisher(Pose, '/arm/goal_pose', 10)
        self.goals_queue.clear()
        self.current_goal_idx = -1
        self.waiting_for_arrival = False
        self.movement_done = False
        ctx["error_triggered"] = False

        res = ctx.get("wall_discretization_results")
        if not res:
            node.get_logger().error(f"[{self.name}] Missing 'wall_discretization_results' in ctx.")
            ctx["error_triggered"] = True
            return

        ## ARREGLAR PER TENIR VARIES ETAPES
        selected_base_idx = ctx.get("selected_base_idx")
        panel_cells_centers = res.get("panel_cells_centers", [])[selected_base_idx]
        # print(f"Panel cell centers of panel #{selected_base_idx}: ", panel_cells_centers)
        if not panel_cells_centers:
            node.get_logger().error(f"[{self.name}] No 'panel_cells_centers' to scan.")
            ctx["error_triggered"] = True
            return
        
        # (Opcional) lee resolución y ejes del ctx si los guardaste:
        # p.ej. ctx["discretization_resolution"] = 0.1, ctx["scan_axes_hint"] = ('x','y')
        # resolution = ctx.get("discretization_resolution", None)
        # axes_hint = ctx.get("scan_axes_hint", None)   # ejemplo: ('x','y') o ('y','z')

        # 1) Obtener lista ordenada en serpentina
        current_position = ctx.get("base_position")
        # print(f"Home position: ", ctx.get("home_position"))
        # print(f"Base position: ", current_position)
        ordered_goals = serpentine_order_vertical_z(current_position, panel_cells_centers, resolution=0.1)
        
        # (Opcional) orientación fija de escaneo:
        # if "scan_orientation" in ctx:
        #     q = ctx["scan_orientation"]
        #     for p in ordered_goals:
        #         p.orientation = q

        # 2) Encolar
        for p in ordered_goals:
            # # p.position.x -= current_position.x
            # # p.position.y -= current_position.y
            # # p.position.z -= 0.0
            # p.orientation.x = 0.0
            # p.orientation.y = -0.70710678
            # p.orientation.z = 0.0
            # p.orientation.w = 0.70710678
            p.orientation.x = -0.5
            p.orientation.y = -0.5
            p.orientation.z = -0.5
            p.orientation.w = -0.5
            self.goals_queue.append(p)

        node.get_logger().info(f"[{self.name}] Queued {len(self.goals_queue)} scan goals (panel_cells_centers).")

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
            f"pos=({next_goal.position.x:.3f}, {next_goal.position.y:.3f}, {next_goal.position.z:.3f}). "
            f"orn=({next_goal.orientation.x:.3f}, {next_goal.orientation.y:.3f}, {next_goal.orientation.z:.3f}, {next_goal.orientation.w:.3f})"
        )

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("panels_left", 0) <= 0:
            node.get_logger().info(f"[{self.name}] No panels left to exhaustively scan.")
            ctx["exhaustive_scan_done"] = True
            return

        if ctx.get("error_triggered") or self.movement_done:
            return
        
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