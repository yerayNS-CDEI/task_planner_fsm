import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration

from ..state import State
from ..utils.gantry_kinematics import TIP_OFFSET_DEFAULT, gantry_ik
from ..utils.wall_geometry import (
    associate_point_to_wall,
    load_walls,
    resolve_detected_walls_path,
)

NEXT_STATE_OPTIONS = [
    "ManipulatorUnfolding",
    "TargetSelection",
    "Error",
]


class NearbyPointSelection(State):
    """Nudge the drill point to a drillable spot in the vicinity of the original.

    Entered when the operator's point cannot be drilled from where the base is
    parked (see ManipulatorReachability). The vicinity is a disc of radius
    ``nearby_point_radius_m`` centred on the ORIGINAL point and lying IN THE WALL
    PLANE -- offsets are taken along the wall direction and vertically, so the
    replacement point stays on the wall surface where drilling makes sense.

    Candidates are sampled on concentric rings (closest first) and each one is
    run through the gantry IK from the current base pose; the first one the
    gantry can actually reach wins. Points already tried for this target are
    skipped, so repeated visits keep exploring instead of returning the same
    spot, and the disc always stays centred on the operator's original point so
    successive nudges cannot drift away from it.

    Tuning knobs (context / ROS params):
      ``nearby_point_radius_m``      vicinity radius            (default 0.30 m)
      ``nearby_point_rings``         rings sampled in the disc  (default 6)
      ``nearby_point_spokes``        samples per ring           (default 12)
      ``nearby_point_min_step_m``    min gap from tried points  (default 0.05 m)
      ``nearby_point_min_z_m``       lowest acceptable height   (default 0.10 m)
    """

    def __init__(self, name):
        super().__init__(name)
        self.compute_start = None

    def on_enter(self, ctx):
        node = ctx["node"]
        ctx["nearby_point_selected"] = False
        ctx["nearby_point_found"] = False
        ctx["error_triggered"] = False
        self._user_choice = None
        self.compute_start = time.time()

        radius = float(ctx.get("nearby_point_radius_m", 0.30))
        target = ctx.get("current_drill_target")

        # Re-centre the search disc whenever the target did NOT come from a
        # previous visit here (i.e. TargetSelection picked a fresh point), so the
        # vicinity is always measured from the operator's original coordinates.
        picked = ctx.get("nearby_point_last_pick")
        if target is not None and (picked is None or tuple(target) != tuple(picked)):
            origin = tuple(float(c) for c in target)
            ctx["nearby_point_origin"] = origin
            ctx["nearby_points_tried"] = [origin]

        node.get_logger().info(
            f"[{self.name}] Looking for a drillable point within {radius:.2f} m of "
            f"the original drill point."
        )

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("nearby_point_selected") or ctx.get("error_triggered"):
            return

        origin = ctx.get("nearby_point_origin")
        if origin is None:
            node.get_logger().error(f"[{self.name}] No current_drill_target in context.")
            ctx["error_triggered"] = True
            return

        # In-wall-plane axes: the along-wall direction and the vertical.
        along, wall = self._along_wall_direction(ctx, origin)
        if along is None:
            self._wait_or_fail(
                ctx,
                waiting_msg="Waiting for the wall map / odometry to fix the along-wall direction...",
                fail_msg="Could not determine the along-wall direction (no wall map and no odometry).",
            )
            return

        # Same axes expressed in the turret frame, so candidates can be fed to the
        # gantry IK without a TF lookup each.
        turret = self._turret_basis(ctx, origin, along)
        if turret is None:
            if ctx.get("error_triggered"):  # hard failure (no tf_buffer), not a wait
                return
            self._wait_or_fail(
                ctx,
                waiting_msg="Waiting for TF to the gantry frame...",
                fail_msg="TF to the gantry frame never became available.",
            )
            return

        found = self._search(ctx, origin, along, wall, turret)
        ctx["nearby_point_selected"] = True

        if found is None:
            radius = float(ctx.get("nearby_point_radius_m", 0.30))
            ctx["nearby_point_found"] = False
            node.get_logger().warn(
                f"[{self.name}] No drillable point found within {radius:.2f} m of "
                f"({origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f}); giving up on this target."
            )
            return

        point, distance, result = found
        ctx["current_drill_target"] = point
        ctx["nearby_point_last_pick"] = point
        ctx["nearby_points_tried"] = list(ctx.get("nearby_points_tried", [])) + [point]
        ctx["nearby_point_found"] = True

        # Keep the stored batch (and the RViz markers built from it) in sync with
        # where the robot will actually drill.
        index = ctx.get("current_drill_index", -1)
        locations = list(ctx.get("drill_locations") or [])
        if 0 <= index < len(locations):
            locations[index] = point
            ctx["drill_locations"] = locations

        node.get_logger().info(
            f"[{self.name}] Drill point moved to ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) "
            f"-- {distance:.3f} m from the original, in the wall plane "
            f"(along-wall residual {result.along_wall_residual:.3f} m)."
        )

        publish_markers = ctx.get("publish_drill_markers")
        if publish_markers:
            publish_markers(ctx)

    # ── Geometry helpers ──────────────────────────────────────────────────
    def _along_wall_direction(self, ctx, origin):
        """Return ``(unit along-wall vector, wall dict or None)`` in the map frame.

        Prefers the endpoints of the wall the drill point belongs to. If the wall
        map is unusable, falls back to the base's +X axis -- BasePlacementComputation
        parks the robot parallel to the wall, so that axis runs along it. Returns
        ``(None, wall)`` while neither source is available yet.
        """
        wall = self._associate_wall(ctx, origin)
        if wall is not None:
            p1, p2 = wall["p1"], wall["p2"]
            ux, uy = float(p2[0]) - float(p1[0]), float(p2[1]) - float(p1[1])
            norm = math.hypot(ux, uy)
            if norm > 1e-6:
                return (ux / norm, uy / norm, 0.0), wall

        orientation = ctx.get("base_orientation")
        if orientation is None:
            return None, wall
        yaw = self._yaw_from_quaternion(orientation)
        return (math.cos(yaw), math.sin(yaw), 0.0), wall

    def _associate_wall(self, ctx, point):
        """Wall the drill point sits on, or ``None`` if the wall map is unusable.

        Unlike BasePlacementComputation this is best-effort: a missing wall only
        costs us the wall-plane basis and the z extent, both of which have
        fallbacks, so it warns instead of failing the state.
        """
        node = ctx["node"]
        yaml_path = resolve_detected_walls_path(ctx.get("detected_walls_yaml"))
        if yaml_path is None:
            node.get_logger().warn(
                f"[{self.name}] detected_walls.yaml not found; using the base heading "
                f"as the along-wall direction.",
                throttle_duration_sec=5.0,
            )
            return None
        try:
            return associate_point_to_wall(
                (float(point[0]), float(point[1]), float(point[2])), load_walls(yaml_path)
            )
        except (OSError, KeyError, ValueError, yaml.YAMLError) as e:
            node.get_logger().warn(
                f"[{self.name}] Wall map '{yaml_path}' unusable ({e}).",
                throttle_duration_sec=5.0,
            )
            return None

    def _turret_basis(self, ctx, origin, along):
        """Return ``(origin, along, up)`` in the turret frame, or ``None`` if TF isn't ready.

        The map->turret transform is rigid, so the axes are recovered by
        transforming the origin and the two unit-offset points and differencing.
        """
        node = ctx["node"]
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            node.get_logger().error(f"[{self.name}] No tf_buffer in context.")
            ctx["error_triggered"] = True
            return None

        map_frame = str(ctx.get("map_frame", "map"))
        gantry_frame = str(ctx.get("gantry_base_frame", "turret_link"))

        # NOTE: lookups must be non-blocking. fsm_node runs on a single-threaded
        # rclpy.spin(), so blocking here would starve the TransformListener.
        try:
            if not tf_buffer.can_transform(
                gantry_frame, map_frame, rclpy.time.Time(), Duration(seconds=0.0)
            ):
                return None
            p0 = self._to_turret(tf_buffer, map_frame, gantry_frame, origin)
            pu = self._to_turret(
                tf_buffer,
                map_frame,
                gantry_frame,
                (origin[0] + along[0], origin[1] + along[1], origin[2] + along[2]),
            )
            pz = self._to_turret(
                tf_buffer, map_frame, gantry_frame, (origin[0], origin[1], origin[2] + 1.0)
            )
        except Exception as e:  # tf2 raises several exception types
            node.get_logger().warn(
                f"[{self.name}] Waiting for TF {map_frame}->{gantry_frame} ({e})...",
                throttle_duration_sec=2.0,
            )
            return None

        along_t = tuple(pu[i] - p0[i] for i in range(3))
        up_t = tuple(pz[i] - p0[i] for i in range(3))
        return p0, along_t, up_t

    @staticmethod
    def _to_turret(tf_buffer, map_frame, gantry_frame, point):
        stamped = PointStamped()
        stamped.header.frame_id = map_frame
        stamped.point.x = float(point[0])
        stamped.point.y = float(point[1])
        stamped.point.z = float(point[2])
        out = tf_buffer.transform(stamped, gantry_frame, timeout=Duration(seconds=0.0))
        return (out.point.x, out.point.y, out.point.z)

    # ── Vicinity search ───────────────────────────────────────────────────
    @staticmethod
    def _candidate_offsets(radius, rings, spokes):
        """In-plane ``(along, up)`` offsets sampling the disc, closest ring first.

        Each ring is rotated by half a spoke step against the previous one so the
        samples do not all line up on the same spokes.
        """
        offsets = []
        for k in range(1, rings + 1):
            r = radius * k / rings
            for j in range(spokes):
                phi = 2.0 * math.pi * j / spokes + math.pi * k / spokes
                offsets.append((r * math.cos(phi), r * math.sin(phi)))
        return offsets

    def _search(self, ctx, origin, along, wall, turret):
        """Return ``(point, distance, ik_result)`` for the closest reachable
        candidate in the vicinity, or ``None`` if the whole disc is unusable."""
        p0_t, along_t, up_t = turret

        radius = float(ctx.get("nearby_point_radius_m", 0.30))
        rings = max(1, int(ctx.get("nearby_point_rings", 6)))
        spokes = max(1, int(ctx.get("nearby_point_spokes", 12)))
        min_step = float(ctx.get("nearby_point_min_step_m", 0.05))

        retract = float(ctx.get("drill_retract_m", 0.20))
        rotate_angle = float(ctx.get("gantry_rotate_angle_rad", 0.0))
        tip_offset = tuple(ctx.get("drill_tip_offset", TIP_OFFSET_DEFAULT))
        lateral_tol = float(ctx.get("gantry_lateral_tol_m", 0.05))
        along_tol = float(ctx.get("gantry_along_wall_tol_m", 0.50))

        tried = [tuple(float(c) for c in p) for p in ctx.get("nearby_points_tried", [])]

        for d_along, d_up in self._candidate_offsets(radius, rings, spokes):
            point = (
                origin[0] + along[0] * d_along,
                origin[1] + along[1] * d_along,
                origin[2] + d_up,
            )
            if not self._point_allowed(ctx, point, wall):
                continue
            if any(self._distance(point, other) < min_step for other in tried):
                continue

            # Same offsets applied in the turret frame; the tip is retracted from
            # the wall exactly as ManipulatorUnfolding will command it.
            tip_target = (
                p0_t[0] + along_t[0] * d_along + up_t[0] * d_up,
                p0_t[1] + along_t[1] * d_along + up_t[1] * d_up + retract,
                p0_t[2] + along_t[2] * d_along + up_t[2] * d_up,
            )
            result = gantry_ik(tip_target, rotate_angle, tip_offset, lateral_tol=lateral_tol)
            if not result.feasible or abs(result.along_wall_residual) > along_tol:
                continue
            return point, math.hypot(d_along, d_up), result

        return None

    @staticmethod
    def _point_allowed(ctx, point, wall):
        """Reject candidates below the minimum drilling height or off the wall panel."""
        if point[2] < float(ctx.get("nearby_point_min_z_m", 0.10)):
            return False
        if wall is None:
            return True
        margin = float(ctx.get("nearby_point_wall_z_margin_m", 0.05))
        z_min = float(wall.get("z_min", -math.inf)) + margin
        z_max = float(wall.get("z_max", math.inf)) - margin
        return z_min <= point[2] <= z_max

    @staticmethod
    def _distance(a, b):
        return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _wait_or_fail(self, ctx, *, waiting_msg, fail_msg):
        """Retry on the next tick until ``nearby_point_compute_timeout`` runs out."""
        node = ctx["node"]
        timeout = float(ctx.get("nearby_point_compute_timeout", 15.0))
        if time.time() - self.compute_start > timeout:
            node.get_logger().error(f"[{self.name}] {fail_msg} (after {timeout:.0f}s).")
            ctx["error_triggered"] = True
        else:
            node.get_logger().warn(f"[{self.name}] {waiting_msg}", throttle_duration_sec=2.0)

    def check_transition(self, ctx):
        if not ctx.get("nearby_point_selected") and not ctx.get("error_triggered"):
            return None
        if ctx.get("error_triggered"):
            return "Error"
        if not ctx.get("nearby_point_found"):
            # Nothing drillable in the vicinity: drop this target and let
            # TargetSelection move on to the next one (it is already ticked off).
            return "TargetSelection"
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
