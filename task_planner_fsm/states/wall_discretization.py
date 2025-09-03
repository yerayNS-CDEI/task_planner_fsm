from ..state import State
from collections import deque
from robotic_arm_planner_interfaces.srv import ComputeWallDiscretization

class WallDiscretization(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.pending_reqs = deque()
        self.current_future = None
        self.total = 0
        self.ok_count = 0

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Computing wall discretization...")
        node.get_logger().info(f"{self.name} Calling the service /compute_wall_discretization")
        
        ctx["discretization_generated"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(ComputeWallDiscretization, 'compute_wall_discretization')
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /compute_wall_discretization not available.")
            ctx["error_triggered"] = True
            return
        
        aoi_data = ctx.get("aoi_data", [])
        print(aoi_data)
        if not aoi_data:
            node.get_logger().error(f"[{self.name}] No aoi_data found in context.")
            ctx["error_triggered"] = True
            return
        
        self.pending_reqs.clear()
        for idx, area in enumerate(aoi_data):
            start_point = area["original"][0]
            end_point = area["original"][1]

            req = ComputeWallDiscretization.Request()
            req.start_point.x = float(start_point[0])
            req.start_point.y = float(start_point[1])
            req.start_point.z = float(start_point[2])
            req.end_point.x   = float(end_point[0])
            req.end_point.y   = float(end_point[1])
            req.end_point.z   = float(end_point[2])
            req.robot_amplitude_range   = 1.3
            req.robot_height_range      = 1.3
            req.sensors_amplitude_range = 0.4
            req.sensors_height_range    = 0.4
            req.resolution              = 0.1
            req.target_i = 1
            req.target_j = 1

            self.pending_reqs.append((idx, req))

        self.total = len(self.pending_reqs)
        self.ok_count = 0
        self.current_future = None
        node.get_logger().info(f"[{self.name}] Queued {self.total} wall-discretization requests.")

        ctx["wall_discretization_results"] = {
            "wall_panels_centers": [],
            "wall_panels_vertices": [],     # needed for base placement
            "panel_cells_centers": [],      # needed for exhaustive scan
            "panel_cells_vertices": []
        }


    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("error_triggered") or ctx.get("discretization_generated"):
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

                res_store = ctx["wall_discretization_results"]
                res_store["wall_panels_centers"].extend(result.wall_panels_centers)
                res_store["wall_panels_vertices"].extend([result.wall_panels_vertices])
                res_store["panel_cells_centers"].extend([result.panel_cells_centers])
                res_store["panel_cells_vertices"].extend(result.panel_cells_vertices)
            else:
                node.get_logger().error(f"[{self.name}] Request {idx+1}/{self.total} returned error.")
                ctx["error_triggered"] = True
                self.current_future = None
                return

            # Limpiamos future para permitir lanzar el siguiente en el próximo tick
            self.current_future = None

            # Si ya no quedan pendientes y todas fueron OK, marcamos listo
            if not self.pending_reqs and self.ok_count == self.total:
                node.get_logger().info(f"[{self.name}] All {self.total} wall discretizations completed successfully.")
                ctx["discretization_generated"] = True
                res_store = ctx["wall_discretization_results"]
                print("Wall panel vertices: ",res_store.get("wall_panels_vertices"))
                print("Panel cells center: ",res_store.get("panel_cells_centers"))
                ctx["panels_left"] = len(res_store.get("panel_cells_centers"))

    def check_transition(self, ctx):
        if ctx.get("discretization_generated"):
            return "BasePlacement"
            # return "WallTargetSelection"
        if ctx.get("error_triggered"):
            return "Error"
        return None
