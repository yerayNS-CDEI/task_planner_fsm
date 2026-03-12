from ..state import State
from collections import deque
from arm_control.srv import ComputeWallDiscretization
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy

# Per-wall fill colours (r, g, b)
_WALL_COLORS = [
    (0.20, 0.45, 0.90),  # blue
    (0.90, 0.50, 0.10),  # orange
    (0.10, 0.80, 0.30),  # green
    (0.80, 0.10, 0.80),  # purple
    (0.90, 0.85, 0.10),  # yellow
]

class WallDiscretization(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.pending_reqs = deque()
        self.current_future = None
        self.total = 0
        self.ok_count = 0
        self.marker_pub = None

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Computing wall discretization...")
        node.get_logger().info(f"{self.name} Calling the service /compute_wall_discretization")

        if self.marker_pub is None:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.marker_pub = node.create_publisher(MarkerArray, '/wall_discretization/markers', qos)
        
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

                # Split flat panel_cells_centers using cells_per_panel from service response
                num_panels = len(result.wall_panels_vertices) // 4
                num_cells = len(result.panel_cells_centers)
                cells_per_panel = result.cells_per_panel
                
                node.get_logger().info(
                    f"[{self.name}] Wall {idx+1}: {num_panels} panel(s), {num_cells} cell(s) total, "
                    f"{cells_per_panel} cells/panel"
                )

                cells = list(result.panel_cells_centers)
                if num_panels > 0 and len(cells) > 0:
                    for panel_idx in range(num_panels):
                        start = panel_idx * cells_per_panel
                        end = (panel_idx + 1) * cells_per_panel
                        panel_cells = cells[start:end]
                        res_store["panel_cells_centers"].append(panel_cells)
                        node.get_logger().debug(
                            f"[{self.name}]     Panel {panel_idx}: {len(panel_cells)} cells"
                        )
                elif len(cells) > 0:
                    # Fallback: treat all cells as one panel
                    res_store["panel_cells_centers"].append(cells)
                    node.get_logger().warn(
                        f"[{self.name}]   → No panels detected, storing {len(cells)} cells as single group"
                    )

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
                ctx["panels_left"] = len(res_store.get("panel_cells_centers"))
                self._publish_markers(ctx, node)

    def _publish_markers(self, ctx, node):
        res = ctx["wall_discretization_results"]
        now = node.get_clock().now().to_msg()
        array = MarkerArray()
        mid = 0

        # Clear any markers from a previous run
        clr = Marker()
        clr.header.frame_id = 'map'
        clr.header.stamp = now
        clr.action = Marker.DELETEALL
        array.markers.append(clr)

        # ── Panel fills + outlines ────────────────────────────────────────────
        total_panels = 0
        for wall_idx, wall_verts_flat in enumerate(res["wall_panels_vertices"]):
            r, g, b = _WALL_COLORS[wall_idx % len(_WALL_COLORS)]
            panels = [wall_verts_flat[i:i+4] for i in range(0, len(wall_verts_flat), 4)]
            total_panels += len(panels)

            # Semi-transparent fill: one TRIANGLE_LIST per wall (2 triangles per quad)
            fill = Marker()
            fill.header.frame_id = 'map'
            fill.header.stamp = now
            fill.ns = 'panel_fill'
            fill.id = mid; mid += 1
            fill.type = Marker.TRIANGLE_LIST
            fill.action = Marker.ADD
            fill.scale.x = fill.scale.y = fill.scale.z = 1.0
            fill.color.r, fill.color.g, fill.color.b, fill.color.a = r, g, b, 0.35
            for panel in panels:
                for vi in [0, 1, 2, 0, 2, 3]:
                    v = panel[vi]
                    fill.points.append(Point(x=float(v.position.x),
                                             y=float(v.position.y),
                                             z=float(v.position.z)))
            array.markers.append(fill)

            # Solid outline: one LINE_STRIP per panel
            for panel in panels:
                ln = Marker()
                ln.header.frame_id = 'map'
                ln.header.stamp = now
                ln.ns = 'panel_outline'
                ln.id = mid; mid += 1
                ln.type = Marker.LINE_STRIP
                ln.action = Marker.ADD
                ln.scale.x = 0.02
                ln.color.r, ln.color.g, ln.color.b, ln.color.a = r, g, b, 1.0
                for v in panel:
                    ln.points.append(Point(x=float(v.position.x),
                                           y=float(v.position.y),
                                           z=float(v.position.z)))
                # Close the loop
                ln.points.append(Point(x=float(panel[0].position.x),
                                       y=float(panel[0].position.y),
                                       z=float(panel[0].position.z)))
                array.markers.append(ln)

        # ── Cell centres (SPHERE_LIST) ─────────────────────────────────────────
        cells_m = Marker()
        cells_m.header.frame_id = 'map'
        cells_m.header.stamp = now
        cells_m.ns = 'cell_centers'
        cells_m.id = mid; mid += 1
        cells_m.type = Marker.SPHERE_LIST
        cells_m.action = Marker.ADD
        cells_m.scale.x = cells_m.scale.y = cells_m.scale.z = 0.06
        cells_m.color.r, cells_m.color.g, cells_m.color.b, cells_m.color.a = 0.0, 1.0, 0.4, 0.9
        
        total_cells = 0
        for panel_idx, panel_cells in enumerate(res["panel_cells_centers"]):
            for c in panel_cells:
                cells_m.points.append(Point(x=float(c.position.x),
                                            y=float(c.position.y),
                                            z=float(c.position.z)))
                total_cells += 1
        
        if cells_m.points:
            array.markers.append(cells_m)

        self.marker_pub.publish(array)
        node.get_logger().info(
            f"[{self.name}] Published markers: {total_panels} panels, {total_cells} cells "
            f"({len(array.markers)} total markers) to /wall_discretization/markers."
        )

    def check_transition(self, ctx):
        if ctx.get("discretization_generated"):
            return "BasePlacement"
            # return "WallTargetSelection"
        if ctx.get("error_triggered"):
            return "Error"
        return None
