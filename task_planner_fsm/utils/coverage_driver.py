"""Uniform free-cell coverage driver for FSM states.

This encapsulates the "cover the reachable free space with an ordered set of
waypoints and drive them" behaviour that :class:`CreateMap` uses to densify the
map with a second pass after exploration.

The geometry comes from :func:`utils.floor_coverage.build_uniform_coverage_plan`:
the reachable free space is tiled at a fixed spacing, every tile gets one
waypoint (so all free space is covered, nothing skipped), and the points are
ordered as a wall-aware geodesic TSP tour (nearest-neighbour + 2-opt) so the base
drives an efficient, non-revisiting route.

The driver is non-blocking and tick-based, matching the FSM's cooperative run
loop. A state owns one :class:`CoverageDriver`, builds a :class:`CoverageConfig`
from its OWN ``ctx`` keys, and:

    driver.ensure_io(ctx)      # once: subscriptions + markers + nav action client
    driver.begin(ctx)          # start a fresh plan-and-drive cycle
    status = driver.tick(ctx)  # each run(): -> PLANNING / NAVIGATING / DONE / FAILED

The driver never writes state-specific ``ctx`` keys; the owning state maps the
returned status onto its own ready/error flags and transitions.
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .floor_coverage import build_uniform_coverage_plan


@dataclass
class CoverageConfig:
    """Resolved configuration for one coverage pass (built from the owner's ctx)."""

    # Grid intake.
    costmap_topic: str = "/global_costmap/costmap"
    map_topic: str = "/map"
    allow_map_fallback: bool = True
    costmap_wait_timeout: float = 10.0

    # Waypoint spacing (tile size) for the uniform coverage grid.
    coverage_step_m: float = 1.5

    # Costs. ``max_traversable_cost`` is the permissive connectivity threshold for
    # the reachability flood-fill (keep narrow gaps connected); ``waypoint_max_cost``
    # (None -> same) is the strict placement threshold keeping points on genuinely
    # free cells. ``wall_clearance_m`` adds an optional standoff eroded from the
    # free/inflation seam. ``map_occupied_threshold`` is the raw-/map fallback.
    max_traversable_cost: int = 65      # costmap cost <= this is drivable
    map_occupied_threshold: int = 50    # raw /map fallback occupancy threshold
    waypoint_max_cost: Optional[int] = None
    wall_clearance_m: float = 0.4
    # Coarsening factor for the geodesic floods used to ORDER points (not the
    # driven path); higher = faster planning on a big map.
    geodesic_downsample: int = 1

    # Execution.
    dry_run: bool = False
    use_through_poses: bool = False

    # Visualization.
    marker_topic: str = "/coverage/markers"
    marker_namespace: str = "coverage"
    show_heading_arrows: bool = False


class CoverageDriver:
    # Status returned by tick().
    PLANNING = "planning"
    NAVIGATING = "navigating"
    DONE = "done"
    FAILED = "failed"

    def __init__(self, node, name: str, cfg: CoverageConfig, nav_client=None):
        self.node = node
        self.name = name
        self.cfg = cfg

        # I/O.
        self.costmap_sub = None
        self.map_sub = None
        self.marker_pub = None
        self.nav_client = nav_client
        self._owns_nav_client = nav_client is None

        self.latest_costmap: Optional[OccupancyGrid] = None
        self.latest_map: Optional[OccupancyGrid] = None

        # Cycle state.
        self._status = self.PLANNING
        self.grid_wait_start: Optional[float] = None
        self.generated_waypoints: List[PoseStamped] = []
        self.last_plan_debug: dict = {}

        # Sequential NavigateToPose execution.
        self.wp_idx = 0
        self.nav_goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self.nav_success_count = 0
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None

        # NavigateThroughPoses execution.
        self._last_poses_remaining = None

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def ensure_io(self, ctx, *, wait_for_server_timeout: float = 10.0) -> bool:
        """Create grid subscriptions, the marker publisher and the nav client.

        Returns False only if a non-dry-run action server is unavailable.
        """
        if self.costmap_sub is None:
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.VOLATILE
            self.costmap_sub = self.node.create_subscription(
                OccupancyGrid, self.cfg.costmap_topic, self._costmap_cb, qos
            )

        if self.map_sub is None:
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.map_sub = self.node.create_subscription(
                OccupancyGrid, self.cfg.map_topic, self._map_cb, qos
            )

        if self.marker_pub is None:
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.marker_pub = self.node.create_publisher(
                MarkerArray, self.cfg.marker_topic, qos
            )

        if self.cfg.use_through_poses:
            action_type, action_name = NavigateThroughPoses, "/navigate_through_poses"
        else:
            action_type, action_name = NavigateToPose, "/navigate_to_pose"

        if self.nav_client is None:
            self.nav_client = ActionClient(self.node, action_type, action_name)
            self._owns_nav_client = True

        if not self.cfg.dry_run:
            self.node.get_logger().info(f"[{self.name}] Waiting for {action_name} action server...")
            if not self.nav_client.wait_for_server(timeout_sec=wait_for_server_timeout):
                self.node.get_logger().error(f"[{self.name}] {action_name} action server not available.")
                return False
        return True

    def begin(self, ctx):
        """Start a fresh plan-and-drive cycle (call after ensure_io)."""
        self._status = self.PLANNING
        self.grid_wait_start = time.time()
        self.generated_waypoints = []
        self.last_plan_debug = {}
        self.wp_idx = 0
        self.nav_goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self.nav_success_count = 0
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._last_poses_remaining = None

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------
    def tick(self, ctx) -> str:
        if self._status in (self.DONE, self.FAILED):
            return self._status

        if self._status == self.PLANNING:
            grid_msg, grid_source = self._select_grid(ctx)
            if grid_msg is None:
                return self._status
            self._plan_and_execute(ctx, grid_msg, grid_source)
            return self._status

        if self._status == self.NAVIGATING and not self.cfg.use_through_poses:
            # Sequential mode is driven here; through-poses mode is fully
            # asynchronous via its result callback.
            self._run_sequential_navigation(ctx)
        return self._status

    # ------------------------------------------------------------------
    # Grid intake
    # ------------------------------------------------------------------
    def _costmap_cb(self, msg):
        self.latest_costmap = msg

    def _map_cb(self, msg):
        self.latest_map = msg

    def _select_grid(self, ctx) -> Tuple[Optional[OccupancyGrid], Optional[str]]:
        node = self.node
        if self.latest_costmap is not None:
            return self.latest_costmap, self.cfg.costmap_topic

        elapsed = time.time() - float(self.grid_wait_start or time.time())
        if elapsed < self.cfg.costmap_wait_timeout:
            if int(elapsed) in {0, 3, 6, 9}:
                node.get_logger().info(
                    f"[{self.name}] Waiting for inflated global costmap "
                    f"({elapsed:.1f}/{self.cfg.costmap_wait_timeout:.1f}s)..."
                )
            return None, None

        if self.cfg.allow_map_fallback and self.latest_map is not None:
            node.get_logger().warn(
                f"[{self.name}] Costmap not received after {self.cfg.costmap_wait_timeout:.1f}s. "
                f"Falling back to {self.cfg.map_topic} (no inflation)."
            )
            return self.latest_map, self.cfg.map_topic

        if int(elapsed) % 5 == 0:
            node.get_logger().warn(f"[{self.name}] Still waiting for costmap/map grid...")
        return None, None

    # ------------------------------------------------------------------
    # Planning + execution
    # ------------------------------------------------------------------
    def _plan_and_execute(self, ctx, grid_msg: OccupancyGrid, grid_source: str):
        node = self.node
        node.get_logger().info(f"[{self.name}] Planning coverage route from {grid_source}.")

        waypoints = self._compute_waypoints(ctx, grid_msg, grid_source)
        if not waypoints:
            node.get_logger().error(f"[{self.name}] Could not compute any valid coverage waypoints.")
            self._status = self.FAILED
            return

        self.generated_waypoints = waypoints
        self._publish_plan_markers(waypoints)

        node.get_logger().info(
            f"[{self.name}] Coverage plan: {len(waypoints)} waypoint(s) at "
            f"{self.last_plan_debug.get('step_m', 0.0):.2f} m spacing "
            f"(geodesic TSP order), clearance "
            f"{self.last_plan_debug.get('wall_clearance_m', 0.0):.2f} m."
        )

        if self.cfg.dry_run:
            node.get_logger().warn(
                f"[{self.name}] dry_run: markers published, skipping base motion. "
                f"Inspect {self.cfg.marker_topic} in RViz."
            )
            self._status = self.DONE
            return

        self.wp_idx = 0
        self.nav_goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self.nav_success_count = 0
        if self.cfg.use_through_poses:
            self._send_navigate_through_poses_goal(ctx, waypoints)
        self._status = self.NAVIGATING

    def _compute_waypoints(self, ctx, grid_msg: OccupancyGrid, grid_source: str) -> List[PoseStamped]:
        node = self.node
        info = grid_msg.info
        resolution = float(info.resolution)
        width = int(info.width)
        height = int(info.height)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            node.get_logger().error(f"[{self.name}] Invalid occupancy grid metadata.")
            return []

        grid = np.asarray(grid_msg.data, dtype=np.int16).reshape((height, width))
        origin = (float(info.origin.position.x), float(info.origin.position.y))

        # A raw /map (0/100/-1) fallback needs a low threshold; an inflated costmap
        # uses the permissive connectivity threshold directly.
        is_costmap = "costmap" in grid_source.lower()
        if is_costmap:
            max_cost = int(self.cfg.max_traversable_cost)
        else:
            max_cost = int(self.cfg.map_occupied_threshold) - 1

        start_cell = self._get_start_cell(ctx, info, origin, resolution)

        plan = build_uniform_coverage_plan(
            grid, origin, resolution, start_cell,
            step_m=float(self.cfg.coverage_step_m),
            max_cost=max_cost,
            waypoint_max_cost=self.cfg.waypoint_max_cost,
            inflation_m=float(self.cfg.wall_clearance_m),
            downsample=int(self.cfg.geodesic_downsample),
        )
        pts = plan.get("waypoints", [])
        if not pts:
            node.get_logger().error(
                f"[{self.name}] No coverage waypoints (grid empty or fully blocked)."
            )
            return []

        waypoints = [self._make_pose_stamped(x, y, float(yaw)) for (x, y, yaw) in pts]

        self.last_plan_debug = {
            "waypoint_count": len(waypoints),
            "step_m": float(self.cfg.coverage_step_m),
            "wall_clearance_m": float(self.cfg.wall_clearance_m),
            "max_traversable_cost": int(max_cost),
            "waypoint_max_cost": (int(max_cost) if self.cfg.waypoint_max_cost is None
                                  else int(self.cfg.waypoint_max_cost)),
        }
        return waypoints

    def _get_start_cell(self, ctx, info, origin, resolution) -> Tuple[int, int]:
        """Robot start cell (row, col), from odom/base_position, TF, or map centre."""
        base = ctx.get("base_position")
        if base is not None:
            c = int((float(base.x) - origin[0]) / resolution)
            r = int((float(base.y) - origin[1]) / resolution)
            return r, c

        xy = self._get_current_robot_xy(ctx)
        if xy is not None:
            c = int((xy[0] - origin[0]) / resolution)
            r = int((xy[1] - origin[1]) / resolution)
            return r, c

        return int(info.height) // 2, int(info.width) // 2

    # ------------------------------------------------------------------
    # Sequential NavigateToPose execution (default, robust)
    # ------------------------------------------------------------------
    def _run_sequential_navigation(self, ctx):
        node = self.node
        waypoints = self.generated_waypoints
        total = len(waypoints)

        if self.wp_idx >= total:
            node.get_logger().info(
                f"[{self.name}] Coverage route completed "
                f"({self.nav_success_count}/{total} waypoints reached)."
            )
            self._status = self.DONE
            return

        if not self.nav_goal_sent:
            self._send_nav_to_pose_goal(ctx, waypoints[self.wp_idx])
            return

        if self.nav_done:
            reached = self.wp_idx + 1
            if self.nav_failed:
                node.get_logger().warn(
                    f"[{self.name}] Coverage: waypoint {reached}/{total} not reached; skipping it."
                )
            else:
                self.nav_success_count += 1
                node.get_logger().info(
                    f"[{self.name}] Coverage: reached waypoint {reached}/{total}, "
                    f"{total - reached} remaining."
                )
            self.wp_idx += 1
            self.nav_goal_sent = False
            self.nav_done = False
            self.nav_failed = False

    def _send_nav_to_pose_goal(self, ctx, pose: PoseStamped):
        node = self.node
        if self.nav_client is None:
            node.get_logger().error(f"[{self.name}] NavigateToPose client is missing.")
            self._status = self.FAILED
            return

        pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_done = False
        self.nav_failed = False
        self.nav_goal_sent = True
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(
            lambda fut: self._nav_to_pose_response_cb(fut)
        )

    def _nav_to_pose_response_cb(self, future):
        node = self.node
        try:
            goal_handle = future.result()
        except Exception as e:
            node.get_logger().warn(f"[{self.name}] NavigateToPose goal request failed: {e}")
            self.nav_failed = True
            self.nav_done = True
            return

        if not goal_handle.accepted:
            node.get_logger().warn(f"[{self.name}] NavigateToPose goal rejected.")
            self.nav_failed = True
            self.nav_done = True
            return

        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda fut: self._nav_to_pose_result_cb(fut)
        )

    def _nav_to_pose_result_cb(self, future):
        node = self.node
        try:
            status = int(future.result().status)
        except Exception as e:
            node.get_logger().warn(f"[{self.name}] NavigateToPose result failed: {e}")
            self.nav_failed = True
            self.nav_done = True
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.nav_failed = True
        self.nav_done = True

    # ------------------------------------------------------------------
    # NavigateThroughPoses execution (opt-in)
    # ------------------------------------------------------------------
    def _send_navigate_through_poses_goal(self, ctx, waypoints: Sequence[PoseStamped]):
        node = self.node
        if self.nav_client is None:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses client is missing.")
            self._status = self.FAILED
            return

        now = node.get_clock().now().to_msg()
        for pose in waypoints:
            pose.header.stamp = now

        goal_msg = NavigateThroughPoses.Goal()
        if hasattr(goal_msg, "poses"):
            goal_msg.poses = list(waypoints)
        elif hasattr(goal_msg, "goals") and hasattr(goal_msg.goals, "poses"):
            goal_msg.goals.poses = list(waypoints)
        else:
            node.get_logger().error(
                f"[{self.name}] Unsupported NavigateThroughPoses goal layout in this Nav2 version."
            )
            self._status = self.FAILED
            return

        node.get_logger().info(f"[{self.name}] Sending NavigateThroughPoses goal with {len(waypoints)} poses.")
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback: self._through_poses_feedback_cb(feedback),
        )
        self._send_goal_future.add_done_callback(lambda fut: self._through_poses_response_cb(fut))

    def _through_poses_response_cb(self, future):
        node = self.node
        try:
            goal_handle = future.result()
        except Exception as e:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses goal request failed: {e}")
            self._status = self.FAILED
            return

        if not goal_handle.accepted:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses goal was rejected.")
            self._status = self.FAILED
            return

        node.get_logger().info(f"[{self.name}] NavigateThroughPoses goal accepted.")
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda fut: self._through_poses_result_cb(fut))

    def _through_poses_feedback_cb(self, feedback_msg):
        node = self.node
        feedback = feedback_msg.feedback
        poses_remaining = getattr(feedback, "number_of_poses_remaining", None)
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if poses_remaining is None:
            return

        # Feedback arrives continuously; only log when a waypoint is actually
        # reached, i.e. when the remaining-pose count drops.
        if self._last_poses_remaining is not None and poses_remaining >= self._last_poses_remaining:
            return
        self._last_poses_remaining = poses_remaining

        if distance_remaining is not None:
            node.get_logger().info(
                f"[{self.name}] Coverage: {poses_remaining} poses remaining, "
                f"{distance_remaining:.2f} m remaining."
            )
        else:
            node.get_logger().info(f"[{self.name}] Coverage: {poses_remaining} poses remaining.")

    def _through_poses_result_cb(self, future):
        node = self.node
        try:
            status = int(future.result().status)
        except Exception as e:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses result failed: {e}")
            self._status = self.FAILED
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().info(f"[{self.name}] Coverage route completed successfully.")
            self._status = self.DONE
        else:
            node.get_logger().error(f"[{self.name}] Coverage navigation failed with status {status}.")
            self._status = self.FAILED

    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------
    def clear_markers(self):
        if self.marker_pub is None:
            return
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _publish_plan_markers(self, waypoints: Sequence[PoseStamped]):
        node = self.node
        if self.marker_pub is None:
            return

        ns = self.cfg.marker_namespace
        markers = []
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.append(delete_marker)

        frame_id = "map"
        stamp = node.get_clock().now().to_msg()

        # Route line through the ordered waypoints.
        route_marker = Marker()
        route_marker.header.frame_id = frame_id
        route_marker.header.stamp = stamp
        route_marker.ns = f"{ns}_route"
        route_marker.id = 1
        route_marker.type = Marker.LINE_STRIP
        route_marker.action = Marker.ADD
        route_marker.scale.x = 0.05
        route_marker.color.r = 0.0
        route_marker.color.g = 1.0
        route_marker.color.b = 0.2
        route_marker.color.a = 0.9
        route_marker.pose.orientation.w = 1.0
        for pose in waypoints:
            route_marker.points.append(Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.05))
        markers.append(route_marker)

        # Waypoint spheres.
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = frame_id
        waypoint_marker.header.stamp = stamp
        waypoint_marker.ns = f"{ns}_waypoints"
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.16
        waypoint_marker.scale.y = 0.16
        waypoint_marker.scale.z = 0.16
        waypoint_marker.color.r = 0.1
        waypoint_marker.color.g = 0.8
        waypoint_marker.color.b = 1.0
        waypoint_marker.color.a = 0.95
        waypoint_marker.pose.orientation.w = 1.0
        for pose in waypoints:
            waypoint_marker.points.append(Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.08))
        markers.append(waypoint_marker)

        for idx, pose in enumerate(waypoints):
            if self.cfg.show_heading_arrows:
                yaw = self._yaw_from_quaternion(pose.pose.orientation)
                arrow = Marker()
                arrow.header.frame_id = frame_id
                arrow.header.stamp = stamp
                arrow.ns = f"{ns}_yaw"
                arrow.id = 1000 + idx
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.scale.x = 0.4
                arrow.scale.y = 0.06
                arrow.scale.z = 0.06
                arrow.color.r = 1.0
                arrow.color.g = 0.7
                arrow.color.b = 0.1
                arrow.color.a = 0.95
                arrow.pose.orientation.w = 1.0
                start = Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.12)
                end = Point(
                    x=pose.pose.position.x + 0.4 * math.cos(yaw),
                    y=pose.pose.position.y + 0.4 * math.sin(yaw),
                    z=0.12,
                )
                arrow.points = [start, end]
                markers.append(arrow)

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = stamp
            text.ns = f"{ns}_labels"
            text.id = 2000 + idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.pose.position.x
            text.pose.position.y = pose.pose.position.y
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.text = str(idx + 1)
            markers.append(text)

        self.marker_pub.publish(MarkerArray(markers=markers))
        node.get_logger().info(f"[{self.name}] Published coverage markers on {self.cfg.marker_topic}.")

    # ------------------------------------------------------------------
    # Geometry / transforms
    # ------------------------------------------------------------------
    def _get_current_robot_xy(self, ctx) -> Optional[Tuple[float, float]]:
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        # turret_footprint first: the omni Nav2 config drives that frame, so
        # progress/pose reads must match it rather than base_link (offset by the
        # turret mount). Keep in sync with NavigateToTarget.nav_base_frame.
        primary = str(ctx.get("nav_base_frame", "turret_footprint"))
        for base_frame in (primary, "base_footprint", "base_link", "base", "chassis"):
            try:
                if tf_buffer.can_transform("map", base_frame, rclpy.time.Time(), Duration(seconds=0.2)):
                    tf = tf_buffer.lookup_transform("map", base_frame, rclpy.time.Time(), Duration(seconds=0.5))
                    return float(tf.transform.translation.x), float(tf.transform.translation.y)
            except Exception:
                continue
        return None

    def _make_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        node = self.node
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
