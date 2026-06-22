"""ScanFloor: cover the floor with straight scan trajectories.

The omnidirectional base drives parallel "lines" in two perpendicular
directions aligned to the building walls (estimated from the nav2 occupancy
grid). Lines are spaced a configurable distance apart and clipped around
obstacles. Before traversing, the arm is parked in an ``under*`` pose so the
downward sensor faces the floor.

Phases (non-blocking, one transition per tick — mirrors ScanWall):
  PLAN       wait for /map, compute the coverage plan, publish RViz markers
  GO_TO_START navigate the base to the first segment's start (line-aligned heading)
  ARM_UNDER  send the arm to the under* pose and wait for it to settle
  TRAVERSE   drive each segment in order (dir_a then dir_b), serpentine
  DONE       set floor_scan_done

The turret heading is aligned with the scan line being driven so the downward
sensor stays parallel to the line: ``dir_a`` lines use the dominant wall angle
and ``dir_b`` lines its perpendicular. The heading therefore takes exactly two
values and only changes once, when the sweep switches from dir_a to dir_b; the
omni base still translates along each line without rotating mid-line.
"""

import math

import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from arm_control.srv import SendPosition

from ..state import State
from ..utils.floor_coverage import build_coverage_plan


class ScanFloor(State):
    # Phase constants.
    PLAN = "PLAN"
    GO_TO_START = "GO_TO_START"
    ARM_UNDER = "ARM_UNDER"
    TRAVERSE = "TRAVERSE"
    DONE = "DONE"

    def __init__(self, name):
        super().__init__(name)
        self.phase = self.PLAN
        self.finished = False

        # Map intake.
        self.map_sub = None
        self.latest_map = None
        self.map_wait_start = None

        # Planning output.
        self.segments = []          # flat ordered list of map-frame segments
        self.seg_idx = 0
        self.seg_substate = "to_end"  # "to_start" then "to_end" per segment
        self.segment_angles = []    # per-segment turret yaw, parallel to self.segments

        # Markers.
        self.marker_pub = None

        # Arm.
        self.position_client = None
        self.arm_pose = "under2"
        self.arm_pose_sent = False
        self.arm_pose_reached = False
        self.arm_future = None
        self.arm_verbose = False

        # Navigation.
        self._send_goal_future = None
        self._get_result_future = None
        self.nav_goal_sent = False
        self.nav_waiting = False
        self.nav_done = False

        # Parameters (resolved in on_enter).
        self.line_spacing_m = 3.0
        self.obstacle_inflation_m = 0.0
        self.min_segment_length_m = 0.5
        self.publish_markers = True
        # Default to the nav2 global costmap so lines respect the inflation layer.
        self.map_topic = "/global_costmap/costmap"
        # Max traversable cost (0 = stay fully outside the inflation layer).
        self.max_traversable_cost = 0

    # ------------------------------------------------------------------
    def _declare_params(self, node):
        defaults = {
            "scan_floor_line_spacing_m": 3.0,
            "scan_floor_arm_pose": "under2",
            "scan_floor_publish_markers": True,
            # Extra safety margin ON TOP of the costmap inflation (usually 0).
            "scan_floor_obstacle_inflation_m": 0.0,
            "scan_floor_min_segment_length_m": 0.5,
            "scan_floor_map_topic": "/global_costmap/costmap",
            # Cells with cost above this are untraversable; 0 keeps lines out of
            # the costmap inflation layer entirely.
            "scan_floor_max_traversable_cost": 0,
        }
        for pname, default in defaults.items():
            if not node.has_parameter(pname):
                node.declare_parameter(pname, default)

        self.line_spacing_m = float(node.get_parameter("scan_floor_line_spacing_m").value)
        self.arm_pose = str(node.get_parameter("scan_floor_arm_pose").value)
        self.publish_markers = bool(node.get_parameter("scan_floor_publish_markers").value)
        self.obstacle_inflation_m = float(node.get_parameter("scan_floor_obstacle_inflation_m").value)
        self.min_segment_length_m = float(node.get_parameter("scan_floor_min_segment_length_m").value)
        self.map_topic = str(node.get_parameter("scan_floor_map_topic").value)
        self.max_traversable_cost = int(node.get_parameter("scan_floor_max_traversable_cost").value)

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering floor-scanning state.")

        self.phase = self.PLAN
        self.finished = False
        self.latest_map = None
        self.map_wait_start = node.get_clock().now()
        self.segments = []
        self.seg_idx = 0
        self.seg_substate = "to_end"
        self.segment_angles = []
        self.arm_pose_sent = False
        self.arm_pose_reached = False
        self.arm_future = None
        self.arm_verbose = False
        self._send_goal_future = None
        self._get_result_future = None
        self.nav_goal_sent = False
        self.nav_waiting = False
        self.nav_done = False
        ctx["floor_scan_done"] = False
        ctx["error_triggered"] = False

        self._declare_params(node)

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
                MarkerArray, "/scan_floor/coverage_markers", marker_qos
            )

        if self.position_client is None:
            self.position_client = node.create_client(SendPosition, "/send_position")

        if ctx.get("nav_client") is None:
            node.get_logger().info(f"[{self.name}] Navigation client missing. Creating one.")
            ctx["nav_client"] = ActionClient(node, NavigateToPose, "/navigate_to_pose")
            if not ctx["nav_client"].wait_for_server(timeout_sec=10.0):
                node.get_logger().error(f"[{self.name}] NavigateToPose action server not available.")
                ctx["error_triggered"] = True

    def _map_callback(self, msg):
        self.latest_map = msg

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def run(self, ctx):
        if self.phase == self.PLAN:
            self._run_plan(ctx)
        elif self.phase == self.GO_TO_START:
            self._run_go_to_start(ctx)
        elif self.phase == self.ARM_UNDER:
            self._run_arm_under(ctx)
        elif self.phase == self.TRAVERSE:
            self._run_traverse(ctx)
        elif self.phase == self.DONE:
            ctx["floor_scan_done"] = True
            self.finished = True

    # ------------------------------------------------------------------
    # Phase PLAN
    # ------------------------------------------------------------------
    def _run_plan(self, ctx):
        node = ctx["node"]

        if self.latest_map is None:
            elapsed = (node.get_clock().now() - self.map_wait_start).nanoseconds / 1e9
            if elapsed > 30.0:
                node.get_logger().error(
                    f"[{self.name}] No map received on '{self.map_topic}' after 30s."
                )
                ctx["error_triggered"] = True
            else:
                node.get_logger().info(
                    f"[{self.name}] Waiting for map on '{self.map_topic}'...", throttle_duration_sec=5.0
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

        # Robot start cell from odometry (map frame). Falls back to map center.
        base = ctx.get("base_position")
        if base is not None:
            start_c = int((base.x - origin[0]) / resolution)
            start_r = int((base.y - origin[1]) / resolution)
        else:
            start_c, start_r = info.width // 2, info.height // 2

        plan = build_coverage_plan(
            grid, origin, resolution, (start_r, start_c),
            spacing_m=self.line_spacing_m,
            inflation_m=self.obstacle_inflation_m,
            min_len_m=self.min_segment_length_m,
            max_cost=self.max_traversable_cost,
        )
        self.segments = plan["segments"]
        self.seg_idx = 0

        start_dir = "dir-A" if plan.get("first_is_a", True) else "dir-B"
        node.get_logger().info(
            f"[{self.name}] Coverage plan: angle={math.degrees(plan['angle']):.1f} deg, "
            f"{len(plan['dir_a'])} line(s) dir-A + {len(plan['dir_b'])} dir-B "
            f"= {len(self.segments)} segment(s), spacing={self.line_spacing_m:.2f}m, "
            f"starting with {start_dir} (closest to robot)."
        )

        if not self.segments:
            node.get_logger().warn(
                f"[{self.name}] No coverage segments generated; nothing to scan."
            )
            self.phase = self.DONE
            return

        if self.publish_markers:
            self._publish_markers(node, plan)

        # Per-segment turret heading (parallel to each line). The plan already
        # encodes the execution order and the matching line angle for each
        # segment, so the heading follows whichever direction is swept first.
        self.segment_angles = list(plan["segment_angles"])

        self.phase = self.GO_TO_START

    # ------------------------------------------------------------------
    # Phase GO_TO_START
    # ------------------------------------------------------------------
    def _run_go_to_start(self, ctx):
        node = ctx["node"]
        start_point = self.segments[0][0]

        if not self.nav_goal_sent:
            node.get_logger().info(
                f"[{self.name}] Navigating to first scan line start "
                f"({start_point[0]:.2f}, {start_point[1]:.2f})."
            )
            self._send_nav_goal(ctx, start_point, self._heading_for_segment(0))
            return

        if self.nav_done:
            self.nav_goal_sent = False
            self.nav_waiting = False
            self.nav_done = False
            self.phase = self.ARM_UNDER

    # ------------------------------------------------------------------
    # Phase ARM_UNDER
    # ------------------------------------------------------------------
    def _run_arm_under(self, ctx):
        node = ctx["node"]

        if not self.arm_pose_reached:
            if not self.arm_pose_sent:
                if not self.position_client.service_is_ready():
                    node.get_logger().warn(f"[{self.name}] Waiting for /send_position service...")
                    return
                node.get_logger().info(f"[{self.name}] Sending arm to '{self.arm_pose}' pose.")
                request = SendPosition.Request()
                request.position_name = self.arm_pose
                ctx["execution_status"] = False
                ctx["planner_goal_failed"] = False
                self.arm_future = self.position_client.call_async(request)
                self.arm_pose_sent = True
                return

            if self.arm_future is not None:
                if not self.arm_future.done():
                    return
                try:
                    response = self.arm_future.result()
                    if not response.success:
                        node.get_logger().error(f"[{self.name}] Arm pose rejected: {response.message}")
                        ctx["error_triggered"] = True
                        return
                    node.get_logger().info(f"[{self.name}] Arm pose accepted: {response.message}")
                except Exception as e:
                    node.get_logger().error(f"[{self.name}] Arm service exception: {e}")
                    ctx["error_triggered"] = True
                    return
                self.arm_future = None

            if ctx.get("planner_goal_failed"):
                ctx["planner_goal_failed"] = False
                node.get_logger().error(f"[{self.name}] Planner reported failure reaching '{self.arm_pose}'.")
                ctx["error_triggered"] = True
                return

            if ctx.get("execution_status") is True:
                ctx["execution_status"] = False
                self.arm_pose_reached = True
                node.get_logger().info(f"[{self.name}] Arm at '{self.arm_pose}' pose; base may travel.")
            else:
                if not self.arm_verbose:
                    node.get_logger().info(f"[{self.name}] Waiting for arm to reach '{self.arm_pose}'...")
                    self.arm_verbose = True
                return

        self.phase = self.TRAVERSE

    # ------------------------------------------------------------------
    # Phase TRAVERSE
    # ------------------------------------------------------------------
    def _run_traverse(self, ctx):
        """Drive each segment with two nav goals: reposition to its start, then
        sweep to its end. The first segment's start was already reached in
        GO_TO_START, so it begins directly in the ``to_end`` substate.
        """
        node = ctx["node"]

        if self.seg_idx >= len(self.segments):
            node.get_logger().info(f"[{self.name}] All scan lines traversed.")
            self.phase = self.DONE
            return

        seg = self.segments[self.seg_idx]
        target_point = seg[0] if self.seg_substate == "to_start" else seg[1]
        heading = self._heading_for_segment(self.seg_idx)

        if not self.nav_goal_sent:
            if self.seg_substate == "to_start":
                node.get_logger().info(
                    f"[{self.name}] Repositioning to line {self.seg_idx + 1} start "
                    f"({target_point[0]:.2f}, {target_point[1]:.2f})."
                )
            else:
                node.get_logger().info(
                    f"[{self.name}] Scanning line {self.seg_idx + 1}/{len(self.segments)} "
                    f"-> ({target_point[0]:.2f}, {target_point[1]:.2f}), "
                    f"turret heading {math.degrees(heading):.1f} deg."
                )
                self._set_progress(ctx, self.seg_idx + 1, len(self.segments))
            self._send_nav_goal(ctx, target_point, heading)
            return

        if self.nav_done:
            self.nav_goal_sent = False
            self.nav_waiting = False
            self.nav_done = False
            if self.seg_substate == "to_start":
                self.seg_substate = "to_end"
            else:
                self.seg_idx += 1
                self.seg_substate = "to_start"

    # ------------------------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------------------------
    def _heading_for_segment(self, idx):
        """Turret yaw (rad) aligned with the scan line at ``idx``.

        Each of the two scan directions gets a single heading so the sensor runs
        parallel to the line. The angles come straight from the plan's
        ``segment_angles`` (execution order), so this is correct regardless of
        which direction is swept first. Returns 0.0 before planning has run.
        """
        if 0 <= idx < len(self.segment_angles):
            return self.segment_angles[idx]
        return 0.0

    def _send_nav_goal(self, ctx, point, heading):
        node = ctx["node"]
        nav_client = ctx.get("nav_client")
        if nav_client is None:
            node.get_logger().error(f"[{self.name}] Navigation client not found.")
            ctx["error_triggered"] = True
            return

        qz = math.sin(heading / 2.0)
        qw = math.cos(heading / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(point[0])
        goal_msg.pose.pose.position.y = float(point[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.nav_done = False
        self.nav_waiting = True
        self.nav_goal_sent = True
        self._send_goal_future = nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(
            lambda fut: self._goal_response_callback(fut, ctx)
        )

    def _goal_response_callback(self, future, ctx):
        node = ctx["node"]
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().error(f"[{self.name}] Navigation goal rejected.")
            ctx["error_triggered"] = True
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda fut: self._result_callback(fut, ctx)
        )
        self.nav_waiting = False

    def _result_callback(self, future, ctx):
        self.nav_done = True

    def _set_progress(self, ctx, current, total):
        set_status = ctx.get("set_fsm_status")
        if callable(set_status):
            set_status(
                self.name,
                phase="running",
                summary=f"Scanning floor line {current}/{total}",
                progress_current=current,
                progress_total=total,
            )

    # ------------------------------------------------------------------
    # Markers
    # ------------------------------------------------------------------
    def _publish_markers(self, node, plan):
        array = MarkerArray()

        # Clear previous markers.
        clear = Marker()
        clear.header.frame_id = "map"
        clear.ns = "scan_floor"
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        colors = {
            "dir_a": ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.9),   # blue
            "dir_b": ColorRGBA(r=1.0, g=0.5, b=0.1, a=0.9),   # orange
        }
        for mid, (key, color) in enumerate(colors.items()):
            segs = plan[key]
            if not segs:
                continue
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = node.get_clock().now().to_msg()
            line.ns = "scan_floor"
            line.id = mid
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.03
            line.color = color
            line.pose.orientation.w = 1.0
            for (x0, y0), (x1, y1) in segs:
                line.points.append(Point(x=float(x0), y=float(y0), z=0.02))
                line.points.append(Point(x=float(x1), y=float(y1), z=0.02))
            array.markers.append(line)

        self.marker_pub.publish(array)
        node.get_logger().info(
            f"[{self.name}] Published coverage markers on /scan_floor/coverage_markers."
        )

    # ------------------------------------------------------------------
    def check_transition(self, ctx):
        if self.finished and ctx.get("floor_scan_done"):
            return "SensorDataProcessing"
        if ctx.get("error_triggered"):
            return "Error"
        return None
