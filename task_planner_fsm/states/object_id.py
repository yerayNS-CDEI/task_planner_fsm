"""ObjectID: drive a wall-following perimeter route over the space while YOLO runs.

This state plans a perimeter (wall-following) route over the already-inflated
nav2 global costmap: the free space is split into its natural connected
components -- each room, corridor or pocket -- and each is lapped once along its
boundary, with the base facing the room interior so a forward camera looks
across the open space. Components too small for a lap still get a single point,
so no free patch is skipped. The route is published as RViz markers for
debugging and executed with Nav2 (sequential NavigateToPose by default).

The geometry comes from ``utils/floor_coverage.build_perimeter_plan``. Two cost
thresholds decide where points go: ``object_id_max_traversable_cost`` is the
permissive *reachability* threshold (it may include the inflation band, so a
patch joined to the room only through inflation still counts as reachable), while
``object_id_waypoint_max_cost`` is the strict *placement* threshold -- set to 0
so every waypoint lands on genuine free space, just inside the free/inflation
seam rather than within the inflation band.

Config is read from ``ctx`` (all keys optional):
  object_id_perimeter_spacing_m   along-lap waypoint spacing in metres (default 2.5)
  object_id_perimeter_min_loop_m  components with a shorter boundary get a single
                                  point instead of a lap (default 1.5)
  object_id_perimeter_min_point_dist_m  minimum spacing between consecutive lap
                                  waypoints; prunes near-duplicate points that
                                  cause back-and-forth motion (default 0.5*spacing)
  object_id_max_traversable_cost  costmap cost <= this is drivable/reachable (default 65)
  object_id_waypoint_max_cost     cost <= this may host a waypoint; 0 keeps points
                                  in free space, off the inflation band (default 0)
  object_id_ensure_pocket_coverage  give every free patch >=1 point, incl. tiny
                                  ones below the min-loop length (default True)
  object_id_skip_start_waypoint   drop a leading waypoint coinciding with the
                                  robot's pose so it moves directly (default True)
  object_id_greedy_nearest        order the route as a pure greedy nearest-neighbour
                                  tour (always go to the closest point) instead of
                                  per-component boundary laps (default True)
  object_id_wall_clearance_m      OPTIONAL extra standoff on top of the costmap
                                  inflation, in metres (default 0)
  object_id_costmap_topic         default "/global_costmap/costmap"
  object_id_map_topic             raw-map fallback, default "/map"
  object_id_check_localization    gate planning on a good rtabmap localization
                                  fix before computing waypoints (default True)
  object_id_localization_topic    rtabmap localization pose w/ covariance,
                                  default "/rtabmap/localization_pose". Only
                                  published once rtabmap relocalizes against the
                                  loaded map, so this is a true map-localization
                                  fix (not just healthy odometry).
  object_id_localization_max_xy_std   max acceptable horizontal position std-dev
                                  in metres, sqrt(cov_xx+cov_yy) (default 0.25)
  object_id_localization_max_yaw_std  max acceptable yaw std-dev in radians,
                                  sqrt(cov_yawyaw) (default 0.20)
  object_id_localization_timeout  seconds to wait for a good fix before giving up
                                  (default 30.0)
  object_id_localization_max_age  ignore localization-pose samples older than
                                  this many seconds (default 30.0); rtabmap only
                                  emits one per relocalization, so keep this
                                  generous
  object_id_localization_required when True (default) a bad/absent fix aborts to
                                  Error; when False it only warns and proceeds
  object_id_dry_run               plan + markers only, skip base motion
  object_id_use_mock              legacy /object_id_sim service path
  object_id_yolo_cmd              optional command to launch YOLO
"""

import math
import shlex
import socket
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy.time
from action_msgs.msg import GoalStatus
from example_interfaces.srv import SetBool
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from ..state import State
from ..utils.floor_coverage import build_perimeter_plan
from task_planner_fsm.states.proc_utils import (
    start_proc,
    stop_proc,
    wait_stack_ready,
    wait_services_ready,
    kill_stale_stack,
    SIM_STACK_PATTERNS,
)


class ObjectID(State):
    def __init__(self, name):
        super().__init__(name)

        # Legacy/mock service fields.
        self.client = None
        self.future = None

        # Navigation/action fields.
        self.nav_client = None
        self.use_through_poses = False
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._last_poses_remaining = None

        # Sequential (NavigateToPose) execution state.
        self.wp_idx = 0
        self.nav_goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self.nav_success_count = 0

        # Grid + visualization.
        self.costmap_sub = None
        self.map_sub = None
        self.marker_pub = None
        self.latest_costmap = None
        self.latest_map = None

        # Localization-quality gate (rtabmap localization-pose covariance).
        self.localization_sub = None
        self.latest_localization_cov = None  # (cov_xx, cov_yy, cov_yawyaw, recv_time)
        self.latest_localization_xy = None   # (x, y, recv_time) in the map frame
        self.localization_wait_start = None
        self._pending_grid = None

        self.sim_value = None
        self.phase = "idle"
        self.grid_wait_start = None
        self.generated_waypoints: List[PoseStamped] = []
        self._last_plan_debug = {}

    # ------------------------------------------------------------------
    # FSM lifecycle
    # ------------------------------------------------------------------
    def on_enter(self, ctx):
        node = ctx["node"]

        ctx["object_id_ready"] = False
        ctx["error_triggered"] = False
        self.phase = "starting"
        self.generated_waypoints = []
        self._last_plan_debug = {}
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._last_poses_remaining = None
        self.wp_idx = 0
        self.nav_goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self.nav_success_count = 0
        self.latest_localization_cov = None
        self.latest_localization_xy = None
        self.localization_wait_start = None
        self._pending_grid = None

        if not self._start_robot_stack(ctx):
            self.phase = "error"
            return

        self.sim_value = "true" if ctx.get("sim", False) else "false"

        # Legacy mock path for quick FSM-only tests.
        if bool(ctx.get("object_id_use_mock", False)):
            self._start_mock_object_id(ctx)
            return

        self._ensure_grid_subscriptions(ctx)
        self._ensure_localization_subscriptions(ctx)
        self._ensure_marker_publisher(ctx)
        self._start_object_id_detection(ctx)
        self._start_optional_yolo_process(ctx)

        # Default to sequential NavigateToPose: it is robust to a single hard-to-
        # reach waypoint (it just retries/skips that one), whereas
        # NavigateThroughPoses fails the whole plan if any pose is unreachable,
        # which can leave the base stuck mid-route. Opt back in with
        # object_id_use_nav_through_poses:=True.
        self.use_through_poses = bool(ctx.get("object_id_use_nav_through_poses", False))
        if self.use_through_poses:
            action_type, action_name = NavigateThroughPoses, "/navigate_through_poses"
        else:
            action_type, action_name = NavigateToPose, "/navigate_to_pose"

        if self.nav_client is None:
            self.nav_client = ActionClient(node, action_type, action_name)

        if not bool(ctx.get("object_id_dry_run", False)):
            node.get_logger().info(f"[{self.name}] Waiting for {action_name} action server...")
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                node.get_logger().error(f"[{self.name}] {action_name} action server not available.")
                ctx["error_triggered"] = True
                self.phase = "error"
                return

        node.get_logger().info(
            f"[{self.name}] Coverage planner ready. Preferred grid: "
            f"{ctx.get('object_id_costmap_topic', '/global_costmap/costmap')} "
            f"(fallback {ctx.get('object_id_map_topic', '/map')})."
        )
        self.grid_wait_start = time.time()
        self.phase = "waiting_grid"

    def run(self, ctx):
        if self.phase == "mock_waiting":
            self._run_mock_object_id(ctx)
            return

        if self.phase in {"idle", "error", "done"}:
            return

        if self.phase == "waiting_grid":
            grid_msg, grid_source = self._select_grid_for_planning(ctx)
            if grid_msg is None:
                return
            # Hold the grid and gate planning on a good localization fix first.
            self._pending_grid = (grid_msg, grid_source)
            self.localization_wait_start = time.time()
            self.phase = "checking_localization"
            return

        if self.phase == "checking_localization":
            status = self._check_localization_quality(ctx)
            if status == "pending":
                return
            if status == "bad":
                ctx["error_triggered"] = True
                self.phase = "error"
                return
            grid_msg, grid_source = self._pending_grid
            self._pending_grid = None
            self._plan_and_execute(ctx, grid_msg, grid_source)
            return

        if self.phase == "navigating" and not self.use_through_poses:
            # Sequential mode is driven from run(); through-poses mode is fully
            # asynchronous via its result callback.
            self._run_sequential_navigation(ctx)

    def on_exit(self, ctx):
        if bool(ctx.get("object_id_clear_markers_on_exit", False)):
            self._clear_markers(ctx)
        # Stop the YOLO object-detection launch when leaving the state. The
        # navigation stack ("nav_sim") is intentionally left running for the
        # following states; only the object-ID process is torn down here. This
        # is a no-op in simulation (the process was never started).
        stop_proc(ctx, "object_id", timeout=10.0)
        if bool(ctx.get("object_id_stop_yolo_on_exit", False)):
            stop_proc(ctx, "object_id_yolo")

    def check_transition(self, ctx):
        if ctx.get("object_id_ready"):
            return "WallLinesComputation"
        if ctx.get("error_triggered"):
            return "Error"
        return None

    # ------------------------------------------------------------------
    # Planning + execution
    # ------------------------------------------------------------------
    def _plan_and_execute(self, ctx, grid_msg: OccupancyGrid, grid_source: str):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Planning coverage route from {grid_source}.")

        waypoints = self._compute_coverage_waypoints(ctx, grid_msg, grid_source)
        if not waypoints:
            node.get_logger().error(f"[{self.name}] Could not compute any valid coverage waypoints.")
            ctx["error_triggered"] = True
            self.phase = "error"
            return

        self.generated_waypoints = waypoints
        ctx["object_id_waypoints"] = waypoints
        ctx["object_id_plan_debug"] = self._last_plan_debug
        self._publish_plan_markers(ctx, waypoints)

        dbg = self._last_plan_debug
        node.get_logger().info(
            f"[{self.name}] Perimeter plan: {dbg.get('room_count', 1)} free patch(es) "
            f"-> {dbg.get('loop_count', 0)} lap(s) + {dbg.get('pocket_count', 0)} pocket point(s) "
            f"= {len(waypoints)} waypoint(s), spacing {dbg.get('spacing_m', 0.0):.2f} m, "
            f"clearance {dbg.get('wall_clearance_m', 0.0):.2f} m, "
            f"placement cost <= {dbg.get('waypoint_max_cost', 0)}, "
            f"reachable cost <= {dbg.get('max_traversable_cost', 0)}."
        )

        if bool(ctx.get("object_id_dry_run", False)):
            node.get_logger().warn(
                f"[{self.name}] dry_run: markers published, skipping base motion. "
                f"Inspect /object_id/coverage_markers in RViz."
            )
            ctx["object_id_ready"] = True
            self.phase = "done"
            return

        self.wp_idx = 0
        self.nav_goal_sent = False
        self.nav_done = False
        self.nav_failed = False
        self.nav_success_count = 0
        if self.use_through_poses:
            self._send_navigate_through_poses_goal(ctx, waypoints)
        self.phase = "navigating"

    def _compute_coverage_waypoints(
        self, ctx, grid_msg: OccupancyGrid, grid_source: str
    ) -> List[PoseStamped]:
        """Build the perimeter route as oriented PoseStamped waypoints."""
        node = ctx["node"]
        info = grid_msg.info
        resolution = float(info.resolution)
        width = int(info.width)
        height = int(info.height)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            node.get_logger().error(f"[{self.name}] Invalid occupancy grid metadata.")
            return []

        grid = np.asarray(grid_msg.data, dtype=np.int16).reshape((height, width))
        origin = (float(info.origin.position.x), float(info.origin.position.y))

        # Reachability (traversable) threshold: permissive so a patch joined to
        # the room only through inflation still counts as reachable. A raw /map
        # (0/100/-1) has no inflation gradient, so it uses a low threshold.
        is_costmap = "costmap" in grid_source.lower()
        if is_costmap:
            max_cost = int(ctx.get("object_id_max_traversable_cost", 99))
        else:
            max_cost = int(ctx.get("object_id_map_occupied_threshold", 50)) - 1

        # Placement threshold: where waypoints may actually land. Default 0 keeps
        # them on genuine free cells, just inside the free/inflation seam.
        waypoint_max_cost = int(ctx.get("object_id_waypoint_max_cost", 0))
        # Give every free patch at least one point, even tiny pockets.
        ensure_pockets = bool(ctx.get("object_id_ensure_pocket_coverage", True))
        # Along-lap waypoint spacing and the minimum boundary length for a lap.
        spacing_m = float(ctx.get("object_id_perimeter_spacing_m", 2.5))
        min_loop_m = float(ctx.get("object_id_perimeter_min_loop_m", 1.5))
        # Minimum spacing between consecutive lap waypoints (default 0.5*spacing):
        # prunes near-duplicate points that otherwise cause back-and-forth motion.
        _mpd = ctx.get("object_id_perimeter_min_point_dist_m", 1.0)
        min_point_dist_m = None if _mpd is None else float(_mpd)
        # Drop a leading waypoint that coincides with the robot's current pose so
        # it starts driving to the first real point instead of to where it stands.
        drop_start_waypoint = bool(ctx.get("object_id_skip_start_waypoint", True))
        # Order the whole route as a pure greedy nearest-neighbour tour (always go
        # to the closest point) instead of following per-component boundary laps.
        greedy_nearest = bool(ctx.get("object_id_greedy_nearest", True))
        # OPTIONAL extra standoff on top of the costmap inflation.
        inflation_m = float(ctx.get("object_id_wall_clearance_m", 0.1))

        start_cell = self._get_start_cell(ctx, info, origin, resolution)

        plan = build_perimeter_plan(
            grid, origin, resolution, start_cell,
            spacing_m=spacing_m,
            inflation_m=inflation_m,
            max_cost=max_cost,
            waypoint_max_cost=waypoint_max_cost,
            ensure_pocket_coverage=ensure_pockets,
            min_loop_len_m=min_loop_m,
            min_point_dist_m=min_point_dist_m,
            drop_start_waypoint=drop_start_waypoint,
            greedy_nearest=greedy_nearest,
        )
        pts = plan.get("waypoints", [])
        if not pts:
            node.get_logger().error(
                f"[{self.name}] No perimeter route generated (grid empty or fully blocked)."
            )
            return []

        # yaw already faces the room interior; use it directly.
        waypoints = [self._make_pose_stamped(ctx, x, y, yaw) for (x, y, yaw) in pts]

        self._last_plan_debug = {
            "angle": float(plan.get("angle", 0.0)),
            "loop_count": int(plan.get("loop_count", 0)),
            "pocket_count": int(plan.get("pocket_count", 0)),
            "room_count": int(plan.get("room_count", 1)),
            "waypoint_count": len(waypoints),
            "spacing_m": spacing_m,
            "min_point_dist_m": (0.5 * spacing_m if min_point_dist_m is None
                                 else min_point_dist_m),
            "wall_clearance_m": inflation_m,
            "max_traversable_cost": max_cost,
            "waypoint_max_cost": waypoint_max_cost,
        }
        return waypoints

    def _get_start_cell(self, ctx, info, origin, resolution) -> Tuple[int, int]:
        """Robot start cell (row, col).

        Prefer rtabmap's relocalized localization pose (the same fix the gate
        validated) so the route is planned from where the robot actually is in
        the map, then fall back to odom/base_position, TF, or the map centre.
        """
        node = ctx["node"]

        loc = self.latest_localization_xy
        if loc is not None:
            c = int((loc[0] - origin[0]) / resolution)
            r = int((loc[1] - origin[1]) / resolution)
            node.get_logger().info(
                f"[{self.name}] Planning start from relocalized pose "
                f"({loc[0]:.2f}, {loc[1]:.2f}) m."
            )
            return r, c

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
    # Stack and optional process setup
    # ------------------------------------------------------------------
    def _start_robot_stack(self, ctx):
        """Launch navigation/localization stack. Returns True when ready."""
        node = ctx["node"]
        requested_sim = bool(ctx.get("sim", False))
        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()
        # Honour the sim value from the context, same as CreateMap / machine.py
        # (which build mapping_cmd from ctx.get('sim')). No more forcing sim:=true.
        sim_value = "true" if requested_sim else "false"
        p = ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            node.get_logger().info(f"[{self.name}] Navigation already running (pid={p.pid}).")
            return True

        try:
            robot_ip = str(ctx.get("robot_ip", "192.168.1.102"))
            robot_port = int(ctx.get("robot_port", 29999))

            if not requested_sim:
                node.get_logger().info(f"[{self.name}] Checking UR robot connectivity at {robot_ip}:{robot_port}...")
                robot_ready = False
                for attempt in range(5):
                    try:
                        sock = socket.create_connection((robot_ip, robot_port), timeout=3)
                        sock.close()
                        robot_ready = True
                        node.get_logger().info(f"[{self.name}] UR robot is reachable at {robot_ip}")
                        time.sleep(2)
                        break
                    except (socket.timeout, socket.error) as e:
                        node.get_logger().warn(
                            f"[{self.name}] UR robot not reachable (attempt {attempt + 1}/5): {e}"
                        )
                        if attempt < 4:
                            time.sleep(3)

                if not robot_ready:
                    node.get_logger().error(
                        f"[{self.name}] UR robot not reachable at {robot_ip}:{robot_port} after 5 attempts."
                    )
                    ctx["error_triggered"] = True
                    return False

            node.get_logger().info(
                f"[{self.name}] Launching navigation stack with Gazebo/Nav2 (sim:={sim_value}, hybrid_sim:=false)..."
            )
            # Free hardware resources held by any orphaned driver from a previous
            # run (notably the SICK UDP ports 2115/2116) before relaunching, so the
            # new stack's drivers can bind instead of failing with "Address already
            # in use" and silently breaking odometry/localization.
            kill_stale_stack(node=node)
            start_proc(
                ctx,
                "nav_sim",
                [
                    "ros2", "launch", "navi_wall", "move_robot.launch.py",
                    f"sim:={sim_value}", "mode:=full", "controller_type:=omni", "hybrid_sim:=false",
                    f"robot_ip:={robot_ip}", "database_name:=rtabmap_fsm", "headless:=true",
                    f"planner_backend:={planner_backend}",
                    f"use_sim_time:={sim_value}",
                ],
            )

            # Wait until the stack is actually producing data instead of a blind
            # sleep. We gate on topics that are known to exist in this system and
            # that map directly to the symptoms we want to avoid:
            #   /clock          -> sim time is running (SIMULATION ONLY)
            #   /tf             -> robot_state_publisher / TF is up (robot model)
            #   /joint_states   -> ros2_control controllers are active
            #   /rtabmap/odom   -> rtabmap odometry is initialized and receiving data
            #
            # /clock is only published when sim time is running, so it must NOT be
            # required on the real robot — otherwise wait_stack_ready always times
            # out, _start_robot_stack returns False, and on_enter bails out before
            # ever launching the object-ID (YOLO) stack.
            default_ready_topics = ["/tf", "/joint_states", "/rtabmap/odom"]
            if requested_sim:
                default_ready_topics = ["/clock"] + default_ready_topics
            required_topics = ctx.get("stack_ready_topics", default_ready_topics)
            ready_timeout = float(ctx.get("stack_ready_timeout", 90.0))
            node.get_logger().info(
                f"[{self.name}] Waiting up to {ready_timeout:.0f}s for navigation + localization stack..."
            )
            if not wait_stack_ready(ctx, required_topics, timeout=ready_timeout):
                node.get_logger().error(f"[{self.name}] Navigation + localization stack did not become ready.")
                stop_proc(ctx, "nav_sim", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)
                ctx["error_triggered"] = True
                return False

            if planner_backend == "legacy":
                collision_services = ctx.get(
                    "collision_ready_services",
                    ["/collision/check_collision_pose"],
                )
                collision_timeout = float(ctx.get("collision_ready_timeout", 60.0))
                node.get_logger().info(
                    f"[{self.name}] Waiting up to {collision_timeout:.0f}s for collision checking service..."
                )
                if not wait_services_ready(ctx, collision_services, timeout=collision_timeout):
                    node.get_logger().error(f"[{self.name}] Collision checking service did not come up.")
                    stop_proc(ctx, "nav_sim", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)
                    ctx["error_triggered"] = True
                    return False

            node.get_logger().info(f"[{self.name}] Navigation + localization stack is ready.")
            return True

        except Exception as e:
            node.get_logger().error(f"[{self.name}] Could not start navigation + localization process: {e}")
            ctx["error_triggered"] = True
            return False

    def _start_object_id_detection(self, ctx):
        """Launch the YOLO object-detection stack (real robot only).

        The camera + YOLO model only run on the physical robot; in simulation
        there is no real camera feed, so the launch is skipped and only the
        space-coverage route is executed. The navigation stack ("nav_sim") is
        kept running for the following states; this process is torn down in
        ``on_exit``.
        """
        node = ctx["node"]
        if bool(ctx.get("sim", False)):
            node.get_logger().info(
                f"[{self.name}] Simulation: skipping YOLO object-detection launch "
                f"(running space coverage only)."
            )
            return

        proc = ctx.get("_procs", {}).get("object_id")
        if proc and proc.poll() is None:
            node.get_logger().info(f"[{self.name}] Object-ID detection already running (pid={proc.pid}).")
            return

        cmd = ctx.get(
            "object_id_detection_cmd",
            ["ros2", "launch", "navi_wall", "yolo_object_detection.launch.py"],
        )
        if isinstance(cmd, str):
            cmd = shlex.split(cmd)
        node.get_logger().info(
            f"[{self.name}] Starting YOLO object detection for real robot: {' '.join(cmd)}"
        )
        start_proc(ctx, "object_id", cmd)

    def _start_optional_yolo_process(self, ctx):
        node = ctx["node"]
        cmd = ctx.get("object_id_yolo_cmd")
        if not cmd:
            return
        proc = ctx.get("_procs", {}).get("object_id_yolo")
        if proc and proc.poll() is None:
            node.get_logger().info(f"[{self.name}] YOLO process already running (pid={proc.pid}).")
            return
        if isinstance(cmd, str):
            cmd = shlex.split(cmd)
        node.get_logger().info(f"[{self.name}] Starting YOLO process: {' '.join(cmd)}")
        start_proc(ctx, "object_id_yolo", cmd)

    # ------------------------------------------------------------------
    # Legacy mock mode
    # ------------------------------------------------------------------
    def _start_mock_object_id(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Mock mode: calling /object_id_sim")
        self.client = node.create_client(SetBool, "/object_id_sim")
        request = SetBool.Request()
        request.data = True
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /object_id_sim not available.")
            ctx["error_triggered"] = True
            self.phase = "error"
            return
        self.future = self.client.call_async(request)
        self.phase = "mock_waiting"

    def _run_mock_object_id(self, ctx):
        node = ctx["node"]
        if self.future is None:
            return
        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Mock ObjectID completed correctly.")
                ctx["object_id_ready"] = True
                ctx["object_id_data"] = ctx.get("walls_data")
                self.phase = "done"
            else:
                node.get_logger().error(f"[{self.name}] Error while computing mock ObjectID.")
                ctx["error_triggered"] = True
                self.phase = "error"
            self.future = None

    # ------------------------------------------------------------------
    # Grid subscriptions
    # ------------------------------------------------------------------
    def _ensure_grid_subscriptions(self, ctx):
        node = ctx["node"]
        costmap_topic = ctx.get("object_id_costmap_topic", "/global_costmap/costmap")
        map_topic = ctx.get("object_id_map_topic", "/map")

        if self.costmap_sub is None:
            costmap_qos = QoSProfile(depth=1)
            costmap_qos.reliability = ReliabilityPolicy.RELIABLE
            costmap_qos.durability = DurabilityPolicy.VOLATILE
            self.costmap_sub = node.create_subscription(
                OccupancyGrid, costmap_topic, self._costmap_callback, costmap_qos
            )

        if self.map_sub is None:
            map_qos = QoSProfile(depth=1)
            map_qos.reliability = ReliabilityPolicy.RELIABLE
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.map_sub = node.create_subscription(
                OccupancyGrid, map_topic, self._map_callback, map_qos
            )

    def _costmap_callback(self, msg):
        self.latest_costmap = msg

    def _map_callback(self, msg):
        self.latest_map = msg

    def _select_grid_for_planning(self, ctx) -> Tuple[Optional[OccupancyGrid], Optional[str]]:
        node = ctx["node"]
        costmap_timeout = float(ctx.get("object_id_costmap_wait_timeout", 20.0))
        allow_map_fallback = bool(ctx.get("object_id_allow_map_fallback", True))

        if self.latest_costmap is not None:
            return self.latest_costmap, str(ctx.get("object_id_costmap_topic", "/global_costmap/costmap"))

        elapsed = time.time() - float(self.grid_wait_start or time.time())
        if elapsed < costmap_timeout:
            if int(elapsed) in {0, 3, 6, 9}:
                node.get_logger().info(
                    f"[{self.name}] Waiting for inflated global costmap "
                    f"({elapsed:.1f}/{costmap_timeout:.1f}s)..."
                )
            return None, None

        if allow_map_fallback and self.latest_map is not None:
            node.get_logger().warn(
                f"[{self.name}] Costmap not received after {costmap_timeout:.1f}s. "
                f"Falling back to /map (no inflation; lanes stay further from walls)."
            )
            return self.latest_map, str(ctx.get("object_id_map_topic", "/map"))

        if int(elapsed) % 5 == 0:
            node.get_logger().warn(f"[{self.name}] Still waiting for costmap/map grid...")
        return None, None

    # ------------------------------------------------------------------
    # Localization-quality gate (rtabmap covariance)
    # ------------------------------------------------------------------
    def _ensure_localization_subscriptions(self, ctx):
        """Subscribe to rtabmap's localization pose.

        In localization mode (Mem/IncrementalMemory=false, set by move_robot's
        ``localization:=true``) rtabmap publishes ``/rtabmap/localization_pose``
        as a PoseWithCovarianceStamped each time it relocalizes against the
        loaded map. Its covariance is a direct measure of how confidently the
        robot is placed *in the map*, so receiving a fresh, low-covariance
        sample is exactly the "the robot has localized itself" guarantee we want
        before planning the coverage route.
        """
        node = ctx["node"]
        loc_topic = ctx.get("object_id_localization_topic", "/rtabmap/localization_pose")
        if self.localization_sub is None:
            self.localization_sub = node.create_subscription(
                PoseWithCovarianceStamped, loc_topic, self._localization_pose_callback, 10
            )

    def _localization_pose_callback(self, msg: PoseWithCovarianceStamped):
        now = time.time()
        # 6x6 row-major covariance; diagonal: xx=0, yy=7, yaw(=zz rot)=35.
        cov = msg.pose.covariance
        self.latest_localization_cov = (
            float(cov[0]), float(cov[7]), float(cov[35]), now
        )
        # Keep the relocalized map-frame position so planning can start from
        # where rtabmap actually placed the robot, not the older odom pose.
        p = msg.pose.pose.position
        self.latest_localization_xy = (float(p.x), float(p.y), now)

    def _check_localization_quality(self, ctx) -> str:
        """Decide whether the robot is localized well enough to start planning.

        Returns ``"ok"`` to proceed, ``"pending"`` to keep waiting on this tick,
        or ``"bad"`` to abort. Compares the position/yaw standard deviations
        derived from rtabmap's covariance against configurable thresholds.
        """
        node = ctx["node"]

        if not bool(ctx.get("object_id_check_localization", True)):
            return "ok"

        max_xy_std = float(ctx.get("object_id_localization_max_xy_std", 0.25))
        max_yaw_std = float(ctx.get("object_id_localization_max_yaw_std", 0.20))
        timeout = float(ctx.get("object_id_localization_timeout", 60.0))
        max_age = float(ctx.get("object_id_localization_max_age", 60.0))
        required = bool(ctx.get("object_id_localization_required", True))

        now = time.time()
        elapsed = now - float(self.localization_wait_start or now)
        sample = self.latest_localization_cov

        fresh = sample is not None and (now - sample[3]) <= max_age
        if fresh:
            cov_xx, cov_yy, cov_yawyaw, _ = sample
            xy_std = math.sqrt(max(cov_xx + cov_yy, 0.0))
            yaw_std = math.sqrt(max(cov_yawyaw, 0.0))
            if xy_std <= max_xy_std and yaw_std <= max_yaw_std:
                node.get_logger().info(
                    f"[{self.name}] Localization OK: "
                    f"xy_std={xy_std:.3f} m (<= {max_xy_std:.3f}), "
                    f"yaw_std={yaw_std:.3f} rad (<= {max_yaw_std:.3f})."
                )
                return "ok"
            if int(elapsed) % 5 == 0:
                node.get_logger().warn(
                    f"[{self.name}] Waiting for better localization: "
                    f"xy_std={xy_std:.3f}/{max_xy_std:.3f} m, "
                    f"yaw_std={yaw_std:.3f}/{max_yaw_std:.3f} rad "
                    f"({elapsed:.0f}/{timeout:.0f}s)."
                )
        elif int(elapsed) % 5 == 0:
            node.get_logger().warn(
                f"[{self.name}] Waiting for an rtabmap localization fix on "
                f"{ctx.get('object_id_localization_topic', '/rtabmap/localization_pose')} "
                f"({elapsed:.0f}/{timeout:.0f}s)..."
            )

        if elapsed < timeout:
            return "pending"

        # Timed out without a good fix.
        if required:
            node.get_logger().error(
                f"[{self.name}] Robot not confidently localized after {timeout:.0f}s; "
                f"aborting before planning the coverage route."
            )
            return "bad"
        node.get_logger().warn(
            f"[{self.name}] Localization not confirmed after {timeout:.0f}s, but "
            f"object_id_localization_required=False; proceeding anyway."
        )
        return "ok"

    # ------------------------------------------------------------------
    # Sequential NavigateToPose execution (default, robust)
    # ------------------------------------------------------------------
    def _run_sequential_navigation(self, ctx):
        node = ctx["node"]
        waypoints = self.generated_waypoints
        total = len(waypoints)

        if self.wp_idx >= total:
            node.get_logger().info(
                f"[{self.name}] Coverage route completed "
                f"({self.nav_success_count}/{total} waypoints reached)."
            )
            ctx["object_id_ready"] = True
            ctx["object_id_data"] = ctx.get("walls_data")
            self.phase = "done"
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
        node = ctx["node"]
        if self.nav_client is None:
            node.get_logger().error(f"[{self.name}] NavigateToPose client is missing.")
            ctx["error_triggered"] = True
            self.phase = "error"
            return

        pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_done = False
        self.nav_failed = False
        self.nav_goal_sent = True
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(
            lambda fut: self._nav_to_pose_response_callback(ctx, fut)
        )

    def _nav_to_pose_response_callback(self, ctx, future):
        node = ctx["node"]
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
            lambda fut: self._nav_to_pose_result_callback(ctx, fut)
        )

    def _nav_to_pose_result_callback(self, ctx, future):
        node = ctx["node"]
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
    # Nav2 action execution (legacy NavigateThroughPoses, opt-in)
    # ------------------------------------------------------------------
    def _send_navigate_through_poses_goal(self, ctx, waypoints: Sequence[PoseStamped]):
        node = ctx["node"]
        if self.nav_client is None:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses client is missing.")
            ctx["error_triggered"] = True
            self.phase = "error"
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
            ctx["error_triggered"] = True
            self.phase = "error"
            return

        node.get_logger().info(f"[{self.name}] Sending NavigateThroughPoses goal with {len(waypoints)} poses.")
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback: self._navigation_feedback_callback(ctx, feedback),
        )
        self._send_goal_future.add_done_callback(lambda fut: self._navigation_goal_response_callback(ctx, fut))

    def _navigation_goal_response_callback(self, ctx, future):
        node = ctx["node"]
        try:
            goal_handle = future.result()
        except Exception as e:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses goal request failed: {e}")
            ctx["error_triggered"] = True
            self.phase = "error"
            return

        if not goal_handle.accepted:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses goal was rejected.")
            ctx["error_triggered"] = True
            self.phase = "error"
            return

        node.get_logger().info(f"[{self.name}] NavigateThroughPoses goal accepted.")
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda fut: self._navigation_result_callback(ctx, fut))

    def _navigation_feedback_callback(self, ctx, feedback_msg):
        node = ctx["node"]
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
            node.get_logger().info(
                f"[{self.name}] Coverage: {poses_remaining} poses remaining."
            )

    def _navigation_result_callback(self, ctx, future):
        node = ctx["node"]
        try:
            result_msg = future.result()
            status = int(result_msg.status)
        except Exception as e:
            node.get_logger().error(f"[{self.name}] NavigateThroughPoses result failed: {e}")
            ctx["error_triggered"] = True
            self.phase = "error"
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().info(f"[{self.name}] Coverage route completed successfully.")
            ctx["object_id_ready"] = True
            ctx["object_id_data"] = ctx.get("walls_data")
            self.phase = "done"
        else:
            node.get_logger().error(f"[{self.name}] Coverage navigation failed with status {status}.")
            ctx["error_triggered"] = True
            self.phase = "error"

    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------
    def _ensure_marker_publisher(self, ctx):
        node = ctx["node"]
        if self.marker_pub is None:
            marker_qos = QoSProfile(depth=1)
            marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.marker_pub = node.create_publisher(
                MarkerArray, "/object_id/coverage_markers", marker_qos
            )

    def _clear_markers(self, ctx):
        if self.marker_pub is None:
            return
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _publish_plan_markers(self, ctx, waypoints: Sequence[PoseStamped]):
        node = ctx["node"]
        if self.marker_pub is None:
            return

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
        route_marker.ns = "object_id_route"
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
        waypoint_marker.ns = "object_id_waypoints"
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

        # Heading arrows (the yellow ones) show the base yaw at each goal. Off by
        # default; enable with object_id_show_heading_arrows:=True.
        show_arrows = bool(ctx.get("object_id_show_heading_arrows", False))

        # Orientation arrows and numeric labels.
        for idx, pose in enumerate(waypoints):
            if show_arrows:
                yaw = self._yaw_from_quaternion(pose.pose.orientation)
                arrow = Marker()
                arrow.header.frame_id = frame_id
                arrow.header.stamp = stamp
                arrow.ns = "object_id_yaw"
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
            text.ns = "object_id_labels"
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
        node.get_logger().info(
            f"[{self.name}] Published coverage markers on /object_id/coverage_markers."
        )

    # ------------------------------------------------------------------
    # Geometry / transforms
    # ------------------------------------------------------------------
    def _get_current_robot_xy(self, ctx) -> Optional[Tuple[float, float]]:
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        for base_frame in ("base_link", "base_footprint", "turret_footprint", "base", "chassis"):
            try:
                if tf_buffer.can_transform("map", base_frame, rclpy.time.Time(), Duration(seconds=0.2)):
                    tf = tf_buffer.lookup_transform("map", base_frame, rclpy.time.Time(), Duration(seconds=0.5))
                    return float(tf.transform.translation.x), float(tf.transform.translation.y)
            except Exception:
                continue
        return None

    def _make_pose_stamped(self, ctx, x: float, y: float, yaw: float) -> PoseStamped:
        node = ctx["node"]
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
    def _heading_to(x: float, y: float, target: Optional[Tuple[float, float]]) -> float:
        if target is None:
            return 0.0
        return math.atan2(target[1] - y, target[0] - x)

    def _yaw_from_quaternion(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
