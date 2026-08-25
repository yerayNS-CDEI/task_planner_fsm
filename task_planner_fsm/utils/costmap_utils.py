"""Helpers to pick a collision-free Nav2 goal near a wall.

The scan geometry puts the desired point (~0.6 m off the wall) inside the Nav2
global costmap's inflated/lethal band, so the global planner rejects it. Rather
than hard-coding a standoff distance, we project the point outward (away from the
wall) to the *nearest* costmap cell the robot base can actually occupy, adapting
to whatever footprint/inflation the costmap is configured with.
"""

import math
import os
import time
from typing import Optional, Tuple

from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

# Topic the dynamic_footprint_publisher listens on to switch the Nav2 footprint
# between the arm-aware hull (True) and a base-only circle (False). Must match
# the publisher's enable_arm_topic param.
ARM_FOOTPRINT_TOPIC = "/dynamic_footprint/enable_arm_expansion"

# How long a state polls for the global costmap to cover a scan target before
# giving up and sending the goal anyway (with the fixed-standoff fallback).
COSTMAP_WAIT_TIMEOUT_S = 2.0

# Nearest-free search knobs, overridable per launch via ROS params of the same
# name (fsm_node bridges any param override into ctx).
# - base_goal_cost_threshold: highest costmap value the base may stand on. The
#   published /global_costmap/costmap uses the 0..100 occupancy scale where 99 is
#   the inscribed (definitely-in-collision) cost and 100 is lethal, so this must
#   stay <= 98; Nav2 plans fine through the inflation gradient below that.
# - base_goal_max_offset: how far (m) the goal may drift from the scan point.
#   The scan point IS the EE target (wall offset already baked in), so this caps
#   the base-center-to-EE-target distance — the arm's horizontal reach (UR10e
#   ~1.3 m). It bounds BOTH the goal search and reachable_wall_segments'
#   feasibility test: anything farther is out of arm reach, and sweeping from
#   there produces a scan that never touched the wall (seen at 1.5: the base
#   cruised 1.45-1.50 m off the scan line through an inflated stretch).
#   Corner-blocked wall ends are handled by segmentation trimming, not by
#   inflating this cap.
DEFAULT_COST_THRESHOLD = 90
DEFAULT_MAX_OFFSET = 1.3

# Minimum base-center distance from the wall FACE (m). Hard clamp applied on top
# of the nearest-free search: even when a cheaper (free) costmap cell exists
# closer to the wall, the base goal is pushed out to AT LEAST this distance. This
# is the reliable knob when the global costmap has little/no inflation gradient,
# so "free" starts right at the lethal band and base_goal_cost_threshold has no
# effect (the first cell below any threshold is already against the wall).
# Override per launch with the ``min_base_standoff`` ctx/ROS param; 0 disables
# the clamp (pure nearest-free). Always capped by arm reach: the enforced
# distance can never exceed scan_offset + base_goal_max_offset.
DEFAULT_MIN_STANDOFF = 0.75


def _search_knobs(ctx) -> Tuple[int, float]:
    """(cost_threshold, max_offset) for the nearest-free search, ctx-overridable.

    The threshold is clamped to 98: 99/100 are inscribed/lethal on the published
    occupancy scale and must never be accepted as standable.
    """
    return (
        min(98, int(ctx.get("base_goal_cost_threshold", DEFAULT_COST_THRESHOLD))),
        float(ctx.get("base_goal_max_offset", DEFAULT_MAX_OFFSET)),
    )


def _min_standoff(ctx) -> float:
    """Minimum base-to-wall-face distance (m) for the nearest-free clamp, ctx-overridable."""
    return max(0.0, float(ctx.get("min_base_standoff", DEFAULT_MIN_STANDOFF)))


def set_arm_footprint_enabled(ctx, enabled: bool) -> None:
    """Command the dynamic footprint publisher to include the arm in the Nav2
    footprint (``True``) or fall back to a base-only circular footprint (``False``).

    Parked at a wall the arm-inclusive hull crosses the wall, so Nav2's collision
    check refuses to move the base and the final approach / fine-correction /
    retries stall. Switching to the base circle on arrival lets the base finish
    its approach and the wall sweep move; the arm is folded during transit so the
    base circle is the true envelope there anyway.

    Latched (transient_local) so the publisher picks up the last command even if
    it (re)subscribes later. A no-op-safe fire-and-forget: if the publisher isn't
    running (use_dynamic_footprint:=false), nothing consumes the topic.

    The publisher/topic name is overridable with the ``arm_footprint_topic``
    ctx/ROS param (must match the publisher's ``enable_arm_topic``).
    """
    node = ctx["node"]
    pub = ctx.get("_arm_footprint_pub")
    if pub is None:
        topic = str(ctx.get("arm_footprint_topic", ARM_FOOTPRINT_TOPIC))
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = node.create_publisher(Bool, topic, qos)
        ctx["_arm_footprint_pub"] = pub
        ctx["_arm_footprint_state"] = None
    if ctx.get("_arm_footprint_state") == bool(enabled):
        return  # already in that state; avoid redundant chatter
    pub.publish(Bool(data=bool(enabled)))
    ctx["_arm_footprint_state"] = bool(enabled)
    node.get_logger().info(
        f"[footprint] Nav2 footprint -> "
        f"{'arm-aware (dynamic hull)' if enabled else 'base-only circle'}."
    )


def nav_bt_xml(ctx, state_name: str) -> str:
    """BT XML for every FSM NavigateToPose goal: the default recovery tree minus
    the base-reversing recoveries (BackOutFromObstacle and BackUp), so a wedged
    robot fails fast back to the FSM instead of reversing away from the wall.
    Override with the 'nav_bt_xml' ctx/ROS param (empty string = bt_navigator's
    default tree, backout included). Falls back to the default tree with a
    warning if the variant isn't installed.
    """
    override = ctx.get("nav_bt_xml")
    if override is not None:
        return str(override)
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory("navi_wall"),
            "config", "behavior_trees", "navigate_to_pose_no_backout.xml",
        )
        if os.path.isfile(path):
            return path
        reason = f"{path} not found"
    except Exception as e:
        reason = str(e)
    ctx["node"].get_logger().warn(
        f"[{state_name}] no-backout BT unavailable ({reason}); goals will use "
        f"the default BT (BackOutFromObstacle enabled)."
    )
    return ""


def publish_base_goal_markers(ctx, scan_xy, goal_xy, mode: str):
    """RViz markers for the last base-goal computation: scan point (red), base
    goal (green) and a text label with the search mode. Latched (transient_local)
    on /fsm/base_goal_markers — set the RViz MarkerArray display's durability to
    Transient Local to see markers published before RViz starts.
    """
    node = ctx["node"]
    pub = ctx.get("_base_goal_marker_pub")
    if pub is None:
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = node.create_publisher(MarkerArray, "/fsm/base_goal_markers", qos)
        ctx["_base_goal_marker_pub"] = pub

    stamp = node.get_clock().now().to_msg()

    def _marker(mid, mtype, x, y, z, r, g, b, scale):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns = "base_goal"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
        m.lifetime = Duration(seconds=0).to_msg()  # forever
        return m

    scan_m = _marker(0, Marker.SPHERE, scan_xy[0], scan_xy[1], 0.1, 1.0, 0.1, 0.1, 0.15)
    goal_m = _marker(1, Marker.SPHERE, goal_xy[0], goal_xy[1], 0.1, 0.1, 1.0, 0.1, 0.2)
    label = _marker(2, Marker.TEXT_VIEW_FACING, goal_xy[0], goal_xy[1], 0.5, 1.0, 1.0, 1.0, 0.25)
    label.text = f"base goal ({mode})"
    pub.publish(MarkerArray(markers=[scan_m, goal_m, label]))


def _cost_at(costmap, x: float, y: float) -> Optional[int]:
    """Occupancy value (0..100, -1 unknown) at world (x, y), or None if off-grid."""
    info = costmap.info
    res = info.resolution
    if res <= 0.0:
        return None
    col = int((x - info.origin.position.x) / res)
    row = int((y - info.origin.position.y) / res)
    if col < 0 or row < 0 or col >= info.width or row >= info.height:
        return None
    return costmap.data[row * info.width + col]


def _in_bounds(costmap, x: float, y: float) -> bool:
    info = costmap.info
    if info.resolution <= 0.0:
        return False
    col = int((x - info.origin.position.x) / info.resolution)
    row = int((y - info.origin.position.y) / info.resolution)
    return 0 <= col < info.width and 0 <= row < info.height


def nearest_free_along(
    costmap,
    x: float,
    y: float,
    dir_x: float,
    dir_y: float,
    max_dist: float = 2.5,
    step: float = 0.05,
    cost_threshold: int = 25,
    min_dist: float = 0.0,
) -> Optional[Tuple[float, float, float]]:
    """Walk from (x, y) along (dir_x, dir_y) and return the first known cell whose
    cost is below ``cost_threshold`` (i.e. the base can stand there).

    Returns ``(fx, fy, dist)`` or ``None`` if no free cell is found within
    ``max_dist`` (or the direction/costmap is unusable). ``dist`` is how far the
    point was pushed out. The walk starts at ``min_dist`` (default 0), so any cell
    closer than that is skipped even if free — used to enforce a minimum standoff.
    """
    norm = math.hypot(dir_x, dir_y)
    if costmap is None or norm < 1e-9:
        return None
    ux, uy = dir_x / norm, dir_y / norm

    d = max(0.0, min_dist)
    while d <= max_dist + 1e-9:
        px, py = x + ux * d, y + uy * d
        cost = _cost_at(costmap, px, py)
        # Known (>=0) and below threshold means a base pose is valid here.
        if cost is not None and 0 <= cost < cost_threshold:
            return (px, py, d)
        d += step
    return None


def await_costmap(ctx, state_name, wait_start, target_point):
    """Non-blocking readiness gate for base goals near a wall.

    Returns ``(decision, new_wait_start)`` where decision is 'ready' (costmap
    covers the target — send now), 'waiting' (retry next tick), or 'timeout'
    (COSTMAP_WAIT_TIMEOUT_S elapsed — send anyway with the fallback). Pass the
    returned ``new_wait_start`` back in on the next call.
    """
    node = ctx["node"]
    if costmap_goal_ready(ctx, target_point):
        if wait_start is not None:
            node.get_logger().info(f"[{state_name}] Global costmap ready; sending base goal.")
        return "ready", wait_start
    now = time.time()
    if wait_start is None:
        node.get_logger().info(
            f"[{state_name}] Waiting for the Nav2 global costmap to cover the target "
            f"(up to {COSTMAP_WAIT_TIMEOUT_S:.0f}s)..."
        )
        return "waiting", now
    if now - wait_start > COSTMAP_WAIT_TIMEOUT_S:
        node.get_logger().warn(
            f"[{state_name}] Costmap still not covering the target after "
            f"{COSTMAP_WAIT_TIMEOUT_S:.0f}s; sending goal with fixed-standoff fallback."
        )
        return "timeout", wait_start
    return "waiting", wait_start


def _outward_dir(ctx):
    """Unit exterior normal (-inward_normal) for the current wall, or None."""
    idx = ctx.get("current_wall_index")
    normals = ctx.get("wall_inward_normals", []) or []
    inward = normals[idx] if isinstance(idx, int) and 0 <= idx < len(normals) else None
    if inward is None:
        return None
    nx, ny = float(inward[0]), float(inward[1])
    norm = math.hypot(nx, ny)
    if norm < 1e-9:
        return None
    return (-nx / norm, -ny / norm)


def costmap_goal_ready(ctx, target_point) -> bool:
    """True when the global costmap is populated enough to place a base goal near
    ``target_point`` — i.e. a known, free cell exists along the exterior normal.

    Used as a non-blocking readiness gate: Nav2's global costmap starts as a small
    patch at the origin and grows, so a scan goal fired too early lands out of
    bounds/unknown and the planner fails. When this returns True the area is mapped
    with reachable free space and base_standoff_goal will find a nearest-free cell.
    Uses the same ctx knobs as base_standoff_goal so gate and goal agree.
    """
    costmap = ctx.get("global_costmap")
    if costmap is None:
        return False
    cost_threshold, max_offset = _search_knobs(ctx)
    tx, ty = float(target_point[0]), float(target_point[1])
    outward = _outward_dir(ctx)
    if outward is None:
        cost = _cost_at(costmap, tx, ty)
        return cost is not None and 0 <= cost < cost_threshold
    # Ready if a base cell exists straight out along the normal OR anywhere on the
    # interior half-plane (corner case) — matches base_standoff_goal's search.
    if nearest_free_along(costmap, tx, ty, outward[0], outward[1],
                          max_dist=max_offset, cost_threshold=cost_threshold) is not None:
        return True
    return nearest_free_2d(costmap, tx, ty, interior_dir=outward,
                           max_radius=max_offset, cost_threshold=cost_threshold) is not None


def nearest_free_2d(
    costmap,
    x: float,
    y: float,
    interior_dir=None,
    max_radius: float = 2.0,
    step: float = 0.05,
    cost_threshold: int = 50,
    min_radius: float = 0.0,
) -> Optional[Tuple[float, float, float]]:
    """Nearest costmap cell below ``cost_threshold`` in ANY direction (ring search).

    Handles wall corners where the straight along-normal search (``nearest_free_along``)
    runs parallel to a perpendicular wall and never escapes its inflation. When
    ``interior_dir`` is given, only cells in that half-plane are accepted so the base
    stays on the scanning (interior) side of the wall rather than jumping outside.
    The ring search starts at ``min_radius`` (default ``step``), so cells closer than
    that are skipped even if free — used to enforce a minimum standoff.
    Returns ``(fx, fy, dist)`` or None.
    """
    if costmap is None:
        return None
    r = max(step, min_radius)
    while r <= max_radius + 1e-9:
        n = max(8, int(2.0 * math.pi * r / step))
        for k in range(n):
            a = 2.0 * math.pi * k / n
            px, py = x + r * math.cos(a), y + r * math.sin(a)
            if interior_dir is not None:
                if (px - x) * interior_dir[0] + (py - y) * interior_dir[1] < -1e-6:
                    continue
            cost = _cost_at(costmap, px, py)
            if cost is not None and 0 <= cost < cost_threshold:
                return (px, py, r)
        r += step
    return None


# LETHAL only. On the published 0..100 occupancy scale 100 is an observed
# obstacle cell, while 99 is INSCRIBED and 98-and-below are the inflation
# gradient -- and both of those are derived from the robot's own radius, not from
# geometry. That distinction is the whole ball game here: with robot_radius
# 0.6 m the wall's inscribed ring reaches 0.6 m out, which is exactly where the
# scan line sits, so a probe keyed on 99 finds every point on every wall blocked
# and segmentation returns nothing at all.
BLOCKED_COST = 100
# How far back from the scan point, toward the wall, to look for something
# sticking out of the surface. The plate sweeps between the wall face and the
# scan line, so this covers its path without reaching the wall's own cells.
DEFAULT_PROBE_DEPTH = 0.35
DEFAULT_PROBE_STEP = 0.05
# How far to back a segment end off from whatever blocked it. The sensor plate
# is ~0.34 m across, so half of it plus a margin: feasibility says where the
# plate's CENTRE may sit, and sweeping to that limit puts the plate's edge into
# the obstacle. Override with ``wall_segment_obstacle_clearance``.
DEFAULT_OBSTACLE_CLEARANCE = 0.25


def _plate_path_clear(ctx, costmap, x: float, y: float) -> bool:
    """True when nothing sticks out of the wall into the plate's path at (x, y).

    The base-standing test below cannot answer this. ``nearest_free_along``
    starts its walk AT the scan point and returns the first free cell, so a
    column on the scan line just pushes the base's standing spot further out and
    the point still counts as reachable — which is how a swept segment ends up
    running straight through a column.

    Probes from the scan point back toward the wall (the volume the plate travels
    through) and rejects the point if any of it is occupied.
    """
    if not bool(ctx.get("wall_segment_check_plate_path", True)):
        return True
    outward = _outward_dir(ctx)
    if outward is None:
        return True
    step = float(ctx.get("wall_segment_probe_step", DEFAULT_PROBE_STEP))
    blocked_cost = int(ctx.get("wall_segment_blocked_cost", BLOCKED_COST))
    # Never probe as far as the wall face itself, whatever depth is configured:
    # the face is ``scan_offset`` away (that offset is baked into the scan line by
    # build_wall_data), and reading its lethal cells would reject the whole wall.
    # The margin also absorbs the disagreement between the mapped wall and the
    # detected one, which runs to tens of centimetres.
    scan_offset = float(ctx.get("wall_scan_offset", 0.6))
    margin = float(ctx.get("wall_segment_face_margin", 0.15))
    depth = min(float(ctx.get("wall_segment_probe_depth", DEFAULT_PROBE_DEPTH)),
                max(0.0, scan_offset - margin))
    d = 0.0
    while d <= depth + 1e-9:
        # Negative offset walks toward the wall, into the plate's corridor.
        cost = _cost_at(costmap, x - outward[0] * d, y - outward[1] * d)
        if cost is not None and cost >= blocked_cost:
            return False
        d += max(step, 1e-3)
    return True


def _point_feasible(ctx, costmap, x: float, y: float) -> bool:
    """True when scan point (x, y) can actually be scanned.

    Two independent conditions, and both are needed:

    * the plate's path to the wall is clear (``_plate_path_clear``) — a column
      bracketed to the wall blocks the scan even though the base could happily
      stand behind it;
    * the base can stand IN FRONT of the point — a free cell along the exterior
      normal within arm reach (base_goal_max_offset). Stricter on purpose than
      base_standoff_goal's fallbacks: a cell merely within radius but off to the
      side (nearest_free_2d) is useless mid-wall — the plate can't face the wall
      from there and whatever pushed the search sideways (a corner, a
      perpendicular wall) blocks the arm anyway.
    """
    cost_threshold, max_offset = _search_knobs(ctx)
    # Standing room is not the same as WORKING room. base_goal_max_offset is the
    # arm's raw reach (~1.3 m), but the arm also has to travel from its unfolded
    # pose to the scanning standoff, and ScanWall clamps that move
    # (scan_wall_arm_z_max_step). A base pushed far out along the normal — which
    # is what happens beside a column, where the inflation blocks everything
    # nearer — leaves an approach longer than the clamp allows, so the segment
    # is planned, transited to, and then abandoned at the approach. Better to
    # judge it unscannable here and let the segment start further along, where
    # the base can actually park close.
    max_offset = min(max_offset, float(ctx.get("wall_segment_max_base_offset", 0.45)))
    outward = _outward_dir(ctx)
    if outward is None:
        cost = _cost_at(costmap, x, y)
        return cost is not None and 0 <= cost < cost_threshold
    if not _plate_path_clear(ctx, costmap, x, y):
        return False
    return nearest_free_along(costmap, x, y, outward[0], outward[1],
                              max_dist=max_offset, cost_threshold=cost_threshold) is not None


def reachable_wall_segments(ctx, p1, p2):
    """Split the scan line p1 -> p2 into the sub-segments the arm can actually
    scan: portions where a base cell exists within ``base_goal_max_offset`` (arm
    reach) of the line. Obstacles that force the base out of that corridor (wall
    corners, perpendicular walls mid-wall) become gaps to skip, per the
    reachability-first scan policy: stay close enough to scan, skip what can't
    be reached rather than detouring away from the wall.

    Returns a list of ((sx, sy, z), (ex, ey, z)) tuples in p1->p2 order (z taken
    from p1), possibly empty (nothing reachable), or ``None`` when the costmap
    is not available yet (caller should fall back / wait).

    ctx knobs: ``wall_segment_sample_step`` (m between feasibility samples,
    default 0.25) and ``wall_segment_min_len`` (drop segments shorter than this,
    default 0.5 m — a sweep shorter than the base footprint isn't worth a
    press/release cycle).
    """
    costmap = ctx.get("global_costmap")
    if costmap is None:
        return None
    step = float(ctx.get("wall_segment_sample_step", 0.25))
    min_len = float(ctx.get("wall_segment_min_len", 0.5))
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    z = float(p1[2]) if len(p1) > 2 else 0.0
    length = math.hypot(x2 - x1, y2 - y1)
    if length < 1e-6:
        return []
    ux, uy = (x2 - x1) / length, (y2 - y1) / length

    # Sample feasibility along the line (endpoints included).
    n = max(2, int(math.ceil(length / step)) + 1)
    ts = [min(length, i * length / (n - 1)) for i in range(n)]
    feas = [_point_feasible(ctx, costmap, x1 + ux * t, y1 + uy * t) for t in ts]

    # Merge consecutive feasible samples into runs of sample INDICES, so we can
    # tell an end that ran into an obstacle from one that simply reached the end
    # of the line.
    runs = []
    run_start = None
    for i, ok in enumerate(feas):
        if ok and run_start is None:
            run_start = i
        elif not ok and run_start is not None:
            runs.append((run_start, i - 1))
            run_start = None
    if run_start is not None:
        runs.append((run_start, len(feas) - 1))

    # Back every obstacle-terminated end off by the plate's own half-width. The
    # feasibility test asks whether the plate can be CENTRED at a sample, so a
    # run ends at the last centre that clears the obstacle — with the plate's
    # near edge, half a width further on, already touching it. Sweeping to that
    # endpoint drives the plate edge-first into the column; backing the boundary
    # off is what turns "where the plate fits" into "where the plate can sweep".
    # Ends at the line's own extremities are left alone: nothing is there.
    clearance = float(ctx.get("wall_segment_obstacle_clearance",
                              DEFAULT_OBSTACLE_CLEARANCE))
    segments = []
    for i0, i1 in runs:
        a, b = ts[i0], ts[i1]
        if i0 > 0:
            a += clearance
        if i1 < len(ts) - 1:
            b -= clearance
        if (b - a) >= min_len:
            segments.append((a, b))

    return [
        ((x1 + ux * a, y1 + uy * a, z), (x1 + ux * b, y1 + uy * b, z))
        for a, b in segments
    ]


def publish_wall_segment_markers(ctx, segments):
    """Latched RViz markers for the reachable sweep segments (green bars slightly
    above the floor), in the ``wall_segments`` namespace. Gaps between bars are
    the skipped, unreachable portions. Reuses GeometryReconstruction's wall
    marker publisher (/geometry_reconstruction/wall_markers) so walls and their
    segments share one RViz display; creates it only when bootstrapping past
    that state. Set the display durability to Transient Local."""
    node = ctx["node"]
    pub = ctx.get("wall_marker_pub")
    if pub is None:
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = node.create_publisher(
            MarkerArray, "/geometry_reconstruction/wall_markers", qos
        )
        ctx["wall_marker_pub"] = pub

    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = node.get_clock().now().to_msg()
    m.ns = "wall_segments"
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.06
    m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.9
    m.lifetime = Duration(seconds=0).to_msg()
    from geometry_msgs.msg import Point
    for (sx, sy, _z1), (ex, ey, _z2) in segments:
        m.points.append(Point(x=float(sx), y=float(sy), z=0.15))
        m.points.append(Point(x=float(ex), y=float(ey), z=0.15))
    pub.publish(MarkerArray(markers=[m]))


def base_standoff_goal(
    ctx,
    state_name: str,
    target_point,
    default_standoff: float = 1.6,
    scan_offset: float = 0.6,
) -> Tuple[float, float]:
    """Turn a wall scan point into a base Nav2 goal the planner can reach.

    Strategy (nearest-free, adaptive): push the point outward along the wall's
    exterior normal (-inward_normal) to the nearest costmap cell the base can
    occupy. The search is capped at ``base_goal_max_offset`` so the goal never
    drifts beyond arm reach, and accepts cells up to ``base_goal_cost_threshold``
    (both ctx/ROS-param overridable; see module defaults). A hard ``min_base_standoff``
    clamp guarantees the goal is never closer than that distance to the wall face,
    even when a free cell exists closer (the reliable knob when the costmap has no
    inflation gradient). Fallbacks, in order: fixed ``base_wall_standoff`` when the
    costmap is unavailable/all-blocked within the cap; the raw scan point when the
    inward normal is missing.

    ``base_wall_standoff`` in ctx overrides ``default_standoff``.
    """
    node = ctx["node"]
    tx, ty = float(target_point[0]), float(target_point[1])

    outward = _outward_dir(ctx)
    if outward is None:
        node.get_logger().warn(
            f"[{state_name}] No inward normal for wall index {ctx.get('current_wall_index')}; "
            f"using raw scan point ({tx:.2f}, {ty:.2f}) (may be too close to the wall)."
        )
        publish_base_goal_markers(ctx, (tx, ty), (tx, ty), "raw scan point")
        return (tx, ty)
    ox, oy = outward

    costmap = ctx.get("global_costmap")
    cost_threshold, max_offset = _search_knobs(ctx)
    # Minimum push from the scan point so the base sits >= min_standoff off the
    # wall face (scan point is already scan_offset off it). Capped at arm reach
    # (max_offset): the base can never both clear min_standoff and stay within
    # reach if min_standoff > scan_offset + max_offset, so honour reach and warn.
    min_standoff = _min_standoff(ctx)
    min_push = max(0.0, min_standoff - scan_offset)
    if min_push > max_offset:
        node.get_logger().warn(
            f"[{state_name}] min_base_standoff {min_standoff:.2f} m exceeds arm reach "
            f"(scan_offset {scan_offset:.2f} + base_goal_max_offset {max_offset:.2f} = "
            f"{scan_offset + max_offset:.2f} m); clamping the standoff to arm reach."
        )
        min_push = max_offset
    # 1) Straight out along the interior normal (keeps the base squarely in front
    #    of the scan point). 2) If that's blocked (e.g. a corner with a
    #    perpendicular wall), search all directions on the interior half-plane.
    #    Both searches start at min_push so a closer free cell is never returned.
    hit = nearest_free_along(costmap, tx, ty, ox, oy,
                             max_dist=max_offset, cost_threshold=cost_threshold,
                             min_dist=min_push)
    mode = "along-normal"
    if hit is None:
        hit = nearest_free_2d(costmap, tx, ty, interior_dir=(ox, oy),
                              max_radius=max_offset, cost_threshold=cost_threshold,
                              min_radius=min_push)
        mode = "2D corner search"
    if hit is not None:
        gx, gy, dist = hit
        node.get_logger().info(
            f"[{state_name}] Nearest-free base goal ({mode}): scan point "
            f"({tx:.2f}, {ty:.2f}) -> ({gx:.2f}, {gy:.2f}), {dist:.2f} m into free space "
            f"(>= {min_standoff:.2f} m min standoff)."
        )
        publish_base_goal_markers(ctx, (tx, ty), (gx, gy), mode)
        return (gx, gy)

    # Fallback: fixed standoff. Diagnose *why* nearest-free failed so map-coverage
    # problems (the usual cause: /map not built out to this wall) are obvious.
    if costmap is None:
        reason = "no global costmap received yet"
    elif not _in_bounds(costmap, tx, ty):
        info = costmap.info
        x0, y0 = info.origin.position.x, info.origin.position.y
        x1 = x0 + info.width * info.resolution
        y1 = y0 + info.height * info.resolution
        reason = (
            f"target ({tx:.2f}, {ty:.2f}) is OUTSIDE the global costmap bounds "
            f"[{x0:.1f}, {x1:.1f}]x[{y0:.1f}, {y1:.1f}] — /map not built out to this wall"
        )
    else:
        reason = (
            f"no cell below cost {cost_threshold} within {max_offset:.2f} m "
            f"(base_goal_max_offset, the arm-reach cap) on the interior side"
        )

    # Honour the min-standoff clamp here too, but never push the base past arm
    # reach (scan_offset + max_offset), else the fixed fallback would place a goal
    # the arm can't scan from.
    base_standoff = float(ctx.get("base_wall_standoff", default_standoff))
    base_standoff = max(base_standoff, min_standoff)
    base_standoff = min(base_standoff, scan_offset + max_offset)
    extra = max(0.0, base_standoff - scan_offset)
    gx, gy = tx + ox * extra, ty + oy * extra
    node.get_logger().warn(
        f"[{state_name}] Nearest-free unavailable ({reason}); using fixed "
        f"{base_standoff:.2f} m standoff: ({tx:.2f}, {ty:.2f}) -> ({gx:.2f}, {gy:.2f})."
    )
    publish_base_goal_markers(ctx, (tx, ty), (gx, gy), "fixed-standoff fallback")
    return (gx, gy)


def wall_parallel_goal(ctx, state_name: str, goal_xy, current_xy) -> Tuple[float, float]:
    """Strip the wall-ward component of a base goal, keeping only the slide ALONG
    the wall.

    ScanWall moves between wall segments with the arm already stretched out in the
    unfolded_fsm pose, but Nav2 plans those moves against a base-only footprint
    (NavigateToTarget switches the arm-inclusive hull off on arrival, see
    set_arm_footprint_enabled), so a goal that is perfectly legal for the base can
    still drive the sensor plate into the wall. The fix is a division of labour:
    the base only ever slides parallel to the wall at whatever standoff it arrived
    with, and the distance to the wall belongs to the ARM, which measures it with
    the plate sensors and closes it along its own Z axis (ScanWall's arm_approach
    seg-phase).

    Goals that move AWAY from the wall are left alone — that direction can never
    pinch the arm. Projected goals are re-validated against the costmap and pushed
    further out along the exterior normal if they land somewhere the base cannot
    stand; when even that fails the projected goal is returned anyway, so Nav2
    rejects it and the caller skips the segment rather than driving into the wall.

    Returns ``goal_xy`` unchanged when the wall normal or the current pose is
    unknown. The RViz markers are re-published with the ORIGINAL goal in the red
    'scan point' slot and the projected one in the green goal slot, so the
    correction is visible on /fsm/base_goal_markers.
    """
    node = ctx["node"]
    outward = _outward_dir(ctx)
    if outward is None or current_xy is None:
        node.get_logger().warn(
            f"[{state_name}] Cannot make the base goal wall-parallel "
            f"({'no wall normal' if outward is None else 'no base pose'}); sending it "
            f"as-is — the stretched arm may close in on the wall."
        )
        return (float(goal_xy[0]), float(goal_xy[1]))

    ox, oy = outward
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    cx, cy = float(current_xy[0]), float(current_xy[1])

    # Displacement projected on the exterior normal: negative means the goal pulls
    # the base toward the wall face, which is the component we drop.
    normal_comp = (gx - cx) * ox + (gy - cy) * oy
    if normal_comp >= -1e-3:
        return (gx, gy)

    px, py = gx - ox * normal_comp, gy - oy * normal_comp

    costmap = ctx.get("global_costmap")
    cost_threshold, max_offset = _search_knobs(ctx)
    if costmap is not None:
        cost = _cost_at(costmap, px, py)
        if cost is None or not (0 <= cost < cost_threshold):
            hit = nearest_free_along(costmap, px, py, ox, oy,
                                     max_dist=max_offset, cost_threshold=cost_threshold)
            if hit is None:
                node.get_logger().warn(
                    f"[{state_name}] Wall-parallel goal ({px:.2f}, {py:.2f}) is not "
                    f"standable and nothing free lies further out within "
                    f"{max_offset:.2f} m; sending it anyway so Nav2 rejects it rather "
                    f"than the base closing in on the wall."
                )
            else:
                px, py, pushed = hit
                node.get_logger().info(
                    f"[{state_name}] Wall-parallel goal was blocked; pushed "
                    f"{pushed:.2f} m further out to ({px:.2f}, {py:.2f})."
                )

    node.get_logger().info(
        f"[{state_name}] Base goal made wall-parallel: ({gx:.2f}, {gy:.2f}) -> "
        f"({px:.2f}, {py:.2f}); dropped {abs(normal_comp):.2f} m of approach toward "
        f"the wall (the arm covers that itself)."
    )
    publish_base_goal_markers(ctx, (gx, gy), (px, py), "wall-parallel transit")
    return (px, py)
