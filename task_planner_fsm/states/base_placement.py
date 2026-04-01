from ..state import State
from collections import deque
from arm_control.srv import OptimalBase
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros

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
        if ctx.get("recompute_base_placement"):
            node.get_logger().warn(f"[{self.name}] Recovery mode: recomputing base placement after planner stall.")
        ctx["recompute_base_placement"] = False
        ctx.setdefault("completed_base_indices", [])
        
        ctx["optimal_bases_computed"] = False
        ctx["error_triggered"] = False

        # Get TF transform from base_link to arm_base (static transform)
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            node.get_logger().error(f"[{self.name}] TF buffer not found in context.")
            ctx["error_triggered"] = True
            return
        
        try:
            # Look up the static transform: base_link -> arm_base
            # Use rclpy.time.Time() to get latest available transform (works with both sim and wall time)
            from rclpy.time import Time
            transform = tf_buffer.lookup_transform(
                "base_link", 
                "arm_base", 
                Time(),
                timeout=tf2_ros.Duration(seconds=2.0)
            )
            self.arm_base_offset_x = transform.transform.translation.x
            self.arm_base_offset_y = transform.transform.translation.y
            node.get_logger().info(
                f"[{self.name}] arm_base offset from base_link: "
                f"dx={self.arm_base_offset_x:.3f}, dy={self.arm_base_offset_y:.3f}"
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            node.get_logger().error(f"[{self.name}] Failed to get base_link -> arm_base transform: {e}")
            ctx["error_triggered"] = True
            return

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

        # xy_key → col_rank: maps each unique (x,y) footprint to a column index
        seen_xy_keys = {}
        # col_rank → [global_panel_idx, ...]: all panel_cells_centers indices per column
        base_to_panel_indices = {}
        global_panel_idx = 0

        for wall, wall_panels in enumerate(walls_results_panels,1):
            wall_vertices = chunker(wall_panels, 4)
            for idx, panel in enumerate(wall_vertices,1):
                xy_key = tuple(sorted(
                    (round(float(v.position.x), 1), round(float(v.position.y), 1))
                    for v in panel
                ))

                if xy_key not in seen_xy_keys:
                    col_rank = len(seen_xy_keys)
                    seen_xy_keys[xy_key] = col_rank
                    base_to_panel_indices[col_rank] = []

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

                    req = OptimalBase.Request()
                    req.poses_ee_xyzrpy = list_vertices
                    req.obstacle_rects = []
                    req.obstacle_circles = []
                    req.min_dist = 0.6  # Close wall distance enabled by pre-approach step preventing singularities
                    req.round_decimals = 3
                    req.grid_res = 0.1
                    req.x_limits = [-10.0,10.0]
                    req.y_limits = [-10.0,10.0]
                    req.enable_simulator = False
                    req.enable_robot_viz = False

                    # Log the actual request for debugging
                    node.get_logger().info(
                        f"[{self.name}] OptimalBase request #{col_rank}: "
                        f"min_dist={req.min_dist}, {len(list_vertices)//6} poses"
                    )
                    node.get_logger().info(
                        f"[{self.name}] poses_ee_xyzrpy sample (first 3): "
                        f"{list_vertices[:18] if len(list_vertices) >= 18 else list_vertices}"
                    )

                    self.pending_reqs.append((col_rank, req))
                else:
                    col_rank = seen_xy_keys[xy_key]
                    node.get_logger().debug(
                        f"[{self.name}] Wall {wall} panel {idx} → existing col_rank {col_rank}"
                    )

                # Track every panel (all rows) under its column
                base_to_panel_indices[col_rank].append(global_panel_idx)
                global_panel_idx += 1

        ctx["base_to_panel_indices"] = base_to_panel_indices

        self.total = len(self.pending_reqs)
        self.ok_count = 0
        self.current_future = None
        node.get_logger().info(f"[{self.name}] Queued {self.total} optimal-base-computation requests.")

        ctx["optimal_base_results"] = {}

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
            col_rank, _ = self.pending_reqs.popleft()
            try:
                result = self.current_future.result()
            except Exception as e:
                node.get_logger().error(f"[{self.name}] Column {col_rank} request failed with exception: {e}")
                ctx["error_triggered"] = True
                self.current_future = None
                return

            if result and getattr(result, "success", False):
                self.ok_count += 1
                node.get_logger().info(f"[{self.name}] Column {col_rank} request succeeded. ({self.ok_count} OK)")

                # Transform arm_base position to base_link position
                arm_base_x = result.base_x
                arm_base_y = result.base_y
                base_link_x = round(arm_base_x - self.arm_base_offset_x, 3)
                base_link_y = round(arm_base_y - self.arm_base_offset_y, 3)
                
                node.get_logger().info(
                    f"[{self.name}] Column {col_rank}: arm_base=({arm_base_x:.3f}, {arm_base_y:.3f}) "
                    f"→ base_link=({base_link_x:.3f}, {base_link_y:.3f})"
                )
                
                ctx["optimal_base_results"][col_rank] = (base_link_x, base_link_y)
            else:
                node.get_logger().error(f"[{self.name}] Column {col_rank} request returned error: {getattr(result, 'message', '(no message)')}")
                ctx["error_triggered"] = True
                self.current_future = None
                return

            # Limpiamos future para permitir lanzar el siguiente en el próximo tick
            self.current_future = None

            # Si ya no quedan pendientes y todas fueron OK, marcamos listo
            if not self.pending_reqs and self.ok_count == self.total:
                print("Optimal bases: ",ctx.get("optimal_base_results"))
                node.get_logger().info(f"[{self.name}] All {self.total} optimal bases completed successfully.")
                completed = set(ctx.get("completed_base_indices", []))
                ctx["panels_left"] = max(0, self.total - len(completed))
                ctx["optimal_bases_computed"] = True

    def check_transition(self, ctx):
        if ctx.get("optimal_bases_computed"):
            return "WallTargetSelection"
        if ctx.get("error_triggered"):
            return "Error"
        return None
