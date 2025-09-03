from ..state import State
from collections import deque
from robotic_arm_planner_interfaces.srv import OptimalBase
import numpy as np

class BasePlacement(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.pending_reqs = deque()
        self.current_future = None
        self.total = 0
        self.ok_count = 0

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Computing optimal base placement...")
        node.get_logger().info(f"[{self.name}] Calling the service /compute_optimal_base")
        
        ctx["optimal_bases_computed"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(OptimalBase, 'compute_optimal_base')
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /compute_optimal_base not available.")
            ctx["error_triggered"] = True
            return
        
        walls_results_panels = ctx["wall_discretization_results"]["wall_panels_vertices"]
        # print(panels_vertices)
        if not walls_results_panels:
            node.get_logger().error(f"[{self.name}] No wall_panels_vertices found in context.")
            ctx["error_triggered"] = True
            return
        
        self.pending_reqs.clear()

        def chunker(seq, size):
            return (seq[pos:pos + size] for pos in range(0, len(seq), size))
        
        for wall, wall_panels in enumerate(walls_results_panels,1):
            wall_vertices = chunker(wall_panels, 4)
            for idx, panel in enumerate(wall_vertices,1):
                list_vertices = []

                panel_base_z = 0.0
                zs_panel = [round(float(v.position.z), 1) for v in panel]
                z0 = min(zs_panel)
                z_shift = panel_base_z - z0  # mueve el panel para que z0 → panel_base_z

                for v in panel:
                    x = round(float(v.position.x), 1)
                    y = round(float(v.position.y), 1)
                    z = round(float(v.position.z), 1)
                    z = round(z + z_shift, 3)

                    list_vertices.extend([x,y,z,0.0,90.0,0.0])
                    node.get_logger().debug(f"[{self.name}] Wall {wall} panel {idx} z_raw={v.position.z:.3f} → z_adj={z:.3f} (base={panel_base_z}, z0={z0})")
                    # print([v.position.x,v.position.y,v.position.z,0.0,90.0,0.0])
                    # print(list_vertices)
                
                req = OptimalBase.Request()
                req.poses_ee_xyzrpy = list_vertices
                req.obstacle_rects = []
                req.obstacle_circles = []
                req.min_dist = 0.6
                req.round_decimals = 3
                req.grid_res = 0.1
                req.x_limits = [-10.0,10.0]
                req.y_limits = [-10.0,10.0]
                req.enable_simulator = False
                req.enable_robot_viz = False

                self.pending_reqs.append((idx*wall, req))

        self.total = len(self.pending_reqs)
        self.ok_count = 0
        self.current_future = None
        node.get_logger().info(f"[{self.name}] Queued {self.total} optimal-base-computation requests.")

        ctx["optimal_base_results"] = []

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("error_triggered") or ctx.get("optimal_bases_computed"):
            return
        
        if self.current_future is None and self.pending_reqs:
            idx, req = self.pending_reqs[0]
            node.get_logger().info(f"[{self.name}] Sending request {idx+1}/{self.total}...")
            self.current_future = self.client.call_async(req)
            return
        
        if self.current_future is not None and self.current_future.done():
            idx, _ = self.pending_reqs.popleft()
            try:
                result = self.current_future.result()
            except Exception as e:
                node.get_logger().error(f"[{self.name}] Request {idx+1}/{self.total} failed with exception: {e}")
                ctx["error_triggered"] = True
                self.current_future = None
                return

            if result and getattr(result, "success", False):
                self.ok_count += 1
                node.get_logger().info(f"[{self.name}] Request {idx+1}/{self.total} succeeded. ({self.ok_count} OK)")

                ctx["optimal_base_results"].append((round(result.base_x,3),round(result.base_y,3)))
            else:
                node.get_logger().error(f"[{self.name}] Request {idx+1}/{self.total} returned error: {getattr(result, 'message', '(no message)')}")
                ctx["error_triggered"] = True
                self.current_future = None
                return

            # Limpiamos future para permitir lanzar el siguiente en el próximo tick
            self.current_future = None

            # Si ya no quedan pendientes y todas fueron OK, marcamos listo
            if not self.pending_reqs and self.ok_count == self.total:
                ctx["optimal_base_results"] = list(dict.fromkeys(ctx.get("optimal_base_results")))
                print("Optimal bases: ",ctx.get("optimal_base_results"))
                node.get_logger().info(f"[{self.name}] All {self.total} optimal bases completed successfully.")
                ctx["optimal_bases_computed"] = True

    def check_transition(self, ctx):
        if ctx.get("optimal_bases_computed"):
            return "WallTargetSelection"
        if ctx.get("error_triggered"):
            return "Error"
        return None
