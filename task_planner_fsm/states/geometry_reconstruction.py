from ..state import State
from ..utils.map_wall_detection import detect_map_features, opencv_available
from example_interfaces.srv import SetBool

import math
import time

import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class GeometryReconstruction(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None

        # Map intake for wall detection.
        self.map_sub = None
        self.latest_map = None
        self.map_wait_start = None

        # Wall-marker publisher.
        self.marker_pub = None

        # Parameters (resolved in on_enter).
        self.map_topic = "/map"
        self.publish_markers = True
        self.map_timeout_s = 30.0
        self.default_wall_z = 0.0
        # Detection tunables (forwarded to detect_map_features).
        self.detect_kwargs = {}

    # ------------------------------------------------------------------
    def _declare_params(self, node):
        defaults = {
            "geometry_reconstruction_wall_map_topic": "/map",
            "geometry_reconstruction_publish_wall_markers": True,
            "geometry_reconstruction_map_timeout_s": 30.0,
            "geometry_reconstruction_wall_default_z": 0.0,
            # Debug: when non-empty, dump the exact occupancy grid the state runs
            # detection on (as <path>.npz) so the offline tuner can reproduce it.
            "geometry_reconstruction_debug_map_path": "",
            # --- detection tunables ---
            "geometry_reconstruction_wall_occupied_thresh": 65,
            "geometry_reconstruction_wall_open_radius_cells": 0,
            "geometry_reconstruction_wall_close_radius_cells": 2,
            "geometry_reconstruction_wall_noise_min_area_cells": 4,
            "geometry_reconstruction_column_max_extent_m": 1.0,
            "geometry_reconstruction_column_max_aspect": 2.5,
            "geometry_reconstruction_column_min_solidity": 0.4,
            "geometry_reconstruction_obstacle_max_extent_m": 2.0,
            "geometry_reconstruction_obstacle_max_unknown_frac": 0.15,
            "geometry_reconstruction_wall_max_thickness_m": 0.4,
            "geometry_reconstruction_wall_min_aspect": 2.0,
            "geometry_reconstruction_wall_rect_fill_min": 0.55,
            "geometry_reconstruction_wall_approx_epsilon_m": 0.4,
            "geometry_reconstruction_wall_min_length_m": 1.0,
            "geometry_reconstruction_wall_merge_angle_deg": 14.0,
            "geometry_reconstruction_wall_merge_dist_m": 0.5,
            "geometry_reconstruction_wall_merge_gap_m": 0.5,
            "geometry_reconstruction_wall_junction_split_tol_m": 0.35,
            "geometry_reconstruction_wall_junction_split_margin_m": 1.0,
            "geometry_reconstruction_wall_fit_band_m": 0.3,
            "geometry_reconstruction_embedded_col_min_protrusion_m": 0.3,
            "geometry_reconstruction_embedded_col_min_diameter_m": 0.4,
            "geometry_reconstruction_embedded_col_max_diameter_m": 1.5,
            "geometry_reconstruction_wall_inner_face_search_m": 1.0,
            "geometry_reconstruction_wall_corner_mend_tol_m": 0.6,
            "geometry_reconstruction_wall_corner_mend_max_extend_m": 0.8,
            "geometry_reconstruction_wall_corner_mend_min_angle_deg": 20.0,
        }
        for pname, default in defaults.items():
            if not node.has_parameter(pname):
                node.declare_parameter(pname, default)

        gp = lambda name: node.get_parameter(name).value
        self.map_topic = str(gp("geometry_reconstruction_wall_map_topic"))
        self.publish_markers = bool(gp("geometry_reconstruction_publish_wall_markers"))
        self.map_timeout_s = float(gp("geometry_reconstruction_map_timeout_s"))
        self.default_wall_z = float(gp("geometry_reconstruction_wall_default_z"))
        self.debug_map_path = str(gp("geometry_reconstruction_debug_map_path"))

        self.detect_kwargs = dict(
            occupied_thresh=int(gp("geometry_reconstruction_wall_occupied_thresh")),
            open_radius_cells=int(gp("geometry_reconstruction_wall_open_radius_cells")),
            close_radius_cells=int(gp("geometry_reconstruction_wall_close_radius_cells")),
            noise_min_area_cells=int(gp("geometry_reconstruction_wall_noise_min_area_cells")),
            column_max_extent_m=float(gp("geometry_reconstruction_column_max_extent_m")),
            column_max_aspect=float(gp("geometry_reconstruction_column_max_aspect")),
            column_min_solidity=float(gp("geometry_reconstruction_column_min_solidity")),
            obstacle_max_extent_m=float(gp("geometry_reconstruction_obstacle_max_extent_m")),
            obstacle_max_unknown_frac=float(gp("geometry_reconstruction_obstacle_max_unknown_frac")),
            wall_max_thickness_m=float(gp("geometry_reconstruction_wall_max_thickness_m")),
            wall_min_aspect=float(gp("geometry_reconstruction_wall_min_aspect")),
            wall_rect_fill_min=float(gp("geometry_reconstruction_wall_rect_fill_min")),
            approx_epsilon_m=float(gp("geometry_reconstruction_wall_approx_epsilon_m")),
            min_wall_length_m=float(gp("geometry_reconstruction_wall_min_length_m")),
            merge_angle_deg=float(gp("geometry_reconstruction_wall_merge_angle_deg")),
            merge_dist_m=float(gp("geometry_reconstruction_wall_merge_dist_m")),
            merge_gap_m=float(gp("geometry_reconstruction_wall_merge_gap_m")),
            junction_split_tol_m=float(gp("geometry_reconstruction_wall_junction_split_tol_m")),
            junction_split_margin_m=float(gp("geometry_reconstruction_wall_junction_split_margin_m")),
            wall_fit_band_m=float(gp("geometry_reconstruction_wall_fit_band_m")),
            embedded_col_min_protrusion_m=float(gp("geometry_reconstruction_embedded_col_min_protrusion_m")),
            embedded_col_min_diameter_m=float(gp("geometry_reconstruction_embedded_col_min_diameter_m")),
            embedded_col_max_diameter_m=float(gp("geometry_reconstruction_embedded_col_max_diameter_m")),
            inner_face_search_m=float(gp("geometry_reconstruction_wall_inner_face_search_m")),
            corner_mend_tol_m=float(gp("geometry_reconstruction_wall_corner_mend_tol_m")),
            corner_mend_max_extend_m=float(gp("geometry_reconstruction_wall_corner_mend_max_extend_m")),
            corner_mend_min_angle_deg=float(gp("geometry_reconstruction_wall_corner_mend_min_angle_deg")),
        )

    def on_enter(self, ctx):
        time.sleep(5)
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Calling the service /start_geometry_reconstruction")
        ctx["reconstruction_ready"] = False
        ctx["walls_detected"] = False
        ctx["error_triggered"] = False

        self._declare_params(node)
        self.latest_map = None
        self.map_wait_start = node.get_clock().now()

        # Latched-map QoS so we receive the last published OccupancyGrid.
        if self.map_sub is None:
            map_qos = QoSProfile(depth=1)
            map_qos.reliability = ReliabilityPolicy.RELIABLE
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.map_sub = node.create_subscription(
                OccupancyGrid, self.map_topic, self._map_callback, map_qos
            )

        if self.publish_markers and self.marker_pub is None:
            marker_qos = QoSProfile(depth=1)
            marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.marker_pub = node.create_publisher(
                MarkerArray, "/geometry_reconstruction/wall_markers", marker_qos
            )

        self.client = node.create_client(SetBool, "/start_geometry_reconstruction")
        request = SetBool.Request()
        request.data = True

        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /start_geometry_reconstruction not available.")
            ctx["error_triggered"] = True
            return

        self.future = self.client.call_async(request)

    def _map_callback(self, msg):
        self.latest_map = msg

    def run(self, ctx):
        node = ctx["node"]

        # ── Step 1: wait for the geometry-reconstruction service to finish. ──
        if not ctx.get("reconstruction_ready"):
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
            return

        # ── Step 2: detect map walls + columns and store them in ctx. ──
        if not ctx.get("walls_detected"):
            self._detect_and_store_features(ctx)

    def _detect_and_store_features(self, ctx):
        node = ctx["node"]

        if not opencv_available():
            node.get_logger().error(
                f"[{self.name}] OpenCV not available; cannot detect map features."
            )
            ctx["error_triggered"] = True
            return

        if self.latest_map is None:
            elapsed = (node.get_clock().now() - self.map_wait_start).nanoseconds / 1e9
            if elapsed > self.map_timeout_s:
                node.get_logger().error(
                    f"[{self.name}] No map received on '{self.map_topic}' after "
                    f"{self.map_timeout_s:.0f}s; cannot detect walls."
                )
                ctx["error_triggered"] = True
            else:
                node.get_logger().info(
                    f"[{self.name}] Waiting for map on '{self.map_topic}'...",
                    throttle_duration_sec=5.0,
                )
            return

        grid_msg = self.latest_map
        info = grid_msg.info
        if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
            node.get_logger().error(f"[{self.name}] Invalid map metadata.")
            ctx["error_triggered"] = True
            return

        grid = np.array(grid_msg.data, dtype=np.int16).reshape(info.height, info.width)
        origin = (info.origin.position.x, info.origin.position.y)
        resolution = float(info.resolution)

        if self.debug_map_path:
            try:
                np.savez(self.debug_map_path, grid=grid,
                         origin=np.array(origin, dtype=np.float64), resolution=resolution)
                node.get_logger().info(
                    f"[{self.name}] Dumped detection input grid to '{self.debug_map_path}.npz' "
                    f"(feed it to tune_wall_detection.py to reproduce exactly)."
                )
            except Exception as exc:
                node.get_logger().warn(f"[{self.name}] Failed to dump debug map: {exc}")

        features = detect_map_features(grid, origin, resolution, **self.detect_kwargs)
        segments = features["walls"]
        columns = features["columns"]

        if not segments:
            node.get_logger().error(
                f"[{self.name}] No walls detected in the map on '{self.map_topic}'."
            )
            ctx["error_triggered"] = True
            return

        # Store walls as 3D ((x, y, z), (x, y, z)) endpoint pairs in the map frame
        # so the data drops straight into the wall-selection list (same shape as
        # the predefined walls). The 2D map carries no height, so z defaults to a
        # parameter; the operator supplies real scan heights in ComputeWallPoints.
        z = self.default_wall_z
        detected_walls = [
            ((float(p0[0]), float(p0[1]), z), (float(p1[0]), float(p1[1]), z))
            for p0, p1 in segments
        ]
        ctx["detected_walls"] = detected_walls
        ctx["detected_columns"] = columns
        ctx["walls_detected"] = True

        node.get_logger().info(
            f"[{self.name}] Detected {len(detected_walls)} wall(s) and "
            f"{len(columns)} column(s) from the map:"
        )
        for idx, (p0, p1) in enumerate(detected_walls, 1):
            length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            node.get_logger().info(
                f"  Wall {idx}: ({p0[0]:.2f}, {p0[1]:.2f}) -> "
                f"({p1[0]:.2f}, {p1[1]:.2f})  |  {length:.2f} m"
            )
        for idx, col in enumerate(columns, 1):
            cx, cy = col["center"]
            node.get_logger().info(
                f"  Column {idx}: ({cx:.2f}, {cy:.2f})  |  r={col['radius_m']:.2f} m"
            )

        if self.publish_markers:
            self._publish_markers(node, detected_walls, columns)

    # ------------------------------------------------------------------
    # Markers
    # ------------------------------------------------------------------
    def _publish_markers(self, node, detected_walls, columns):
        now = node.get_clock().now().to_msg()
        array = MarkerArray()

        # Clear any markers from a previous run.
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = now
        clear.ns = "detected_walls"
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        # All wall segments as one red LINE_LIST.
        lines = Marker()
        lines.header.frame_id = "map"
        lines.header.stamp = now
        lines.ns = "detected_walls"
        lines.id = 0
        lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.scale.x = 0.05
        lines.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0)
        lines.pose.orientation.w = 1.0

        # Wall endpoints as a yellow SPHERE_LIST for clear visual verification.
        endpoints = Marker()
        endpoints.header.frame_id = "map"
        endpoints.header.stamp = now
        endpoints.ns = "detected_wall_endpoints"
        endpoints.id = 1
        endpoints.type = Marker.SPHERE_LIST
        endpoints.action = Marker.ADD
        endpoints.scale.x = endpoints.scale.y = endpoints.scale.z = 0.12
        endpoints.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=1.0)
        endpoints.pose.orientation.w = 1.0

        for idx, (p0, p1) in enumerate(detected_walls, 1):
            a = Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2]))
            b = Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))
            lines.points.append(a)
            lines.points.append(b)
            endpoints.points.append(a)
            endpoints.points.append(b)

            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = now
            text.ns = "detected_wall_labels"
            text.id = 100 + idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.3
            text.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0)  # green, readable on the map
            text.pose.position.x = float((p0[0] + p1[0]) / 2.0)
            text.pose.position.y = float((p0[1] + p1[1]) / 2.0)
            text.pose.position.z = float((p0[2] + p1[2]) / 2.0) + 0.3
            text.pose.orientation.w = 1.0
            text.text = f"W{idx}"
            array.markers.append(text)

        array.markers.append(lines)
        array.markers.append(endpoints)

        # Columns as blue CYLINDER markers (one per column) + labels.
        for idx, col in enumerate(columns, 1):
            cx, cy = col["center"]
            radius = max(float(col["radius_m"]), 0.05)
            z = self.default_wall_z

            cyl = Marker()
            cyl.header.frame_id = "map"
            cyl.header.stamp = now
            cyl.ns = "detected_columns"
            cyl.id = 200 + idx
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.scale.x = cyl.scale.y = 2.0 * radius
            cyl.scale.z = 0.1
            # free-standing columns blue, embedded (pilaster) orange, obstacles green
            if col.get("embedded"):
                cyl.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.7)
            elif col.get("obstacle"):
                cyl.color = ColorRGBA(r=0.1, g=0.7, b=0.1, a=0.7)
            else:
                cyl.color = ColorRGBA(r=0.2, g=0.45, b=0.95, a=0.7)
            cyl.pose.position.x = float(cx)
            cyl.pose.position.y = float(cy)
            cyl.pose.position.z = float(z)
            cyl.pose.orientation.w = 1.0
            array.markers.append(cyl)

            ctext = Marker()
            ctext.header.frame_id = "map"
            ctext.header.stamp = now
            ctext.ns = "detected_column_labels"
            ctext.id = 300 + idx
            ctext.type = Marker.TEXT_VIEW_FACING
            ctext.action = Marker.ADD
            ctext.scale.z = 0.25
            ctext.color = ColorRGBA(r=0.9, g=0.2, b=0.9, a=1.0)  # magenta, readable on the map
            ctext.pose.position.x = float(cx)
            ctext.pose.position.y = float(cy)
            ctext.pose.position.z = float(z) + 0.3
            ctext.pose.orientation.w = 1.0
            ctext.text = f"O{idx}"
            array.markers.append(ctext)

        self.marker_pub.publish(array)
        node.get_logger().info(
            f"[{self.name}] Published {len(detected_walls)} wall + {len(columns)} column "
            f"marker(s) on /geometry_reconstruction/wall_markers."
        )

    def check_transition(self, ctx):
        if ctx.get("reconstruction_ready") and ctx.get("walls_detected"):
            return "ComputeWallPoints"
        if ctx.get("error_triggered"):
            return "Error"
        return None
