import math

import yaml

from ..state import State
from ..utils.wall_geometry import (
    associate_point_to_wall,
    load_walls,
    resolve_detected_walls_path,
)
from geometry_msgs.msg import PoseStamped

NEXT_STATE_OPTIONS = [
    "NavigateToTarget",
    "Error",
]


class BasePlacementComputation(State):
    """Compute a ground base pose parked *parallel* to the wall.

    The PokEye gantry drills sideways: the turret is oriented so the wall lies to
    its RIGHT side (-Y), and the gantry lateral axis is the approach/retract
    direction (negative lateral = toward wall). So we park the base with its X
    axis running along the wall (wall on the robot's right, -Y side) at
    ``wall_standoff_m`` from the wall along the outward normal.
    The wall normal comes from the detected-wall map; if the point can't be
    associated to a wall we fall back to the robot->point direction.
    """

    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Computing parallel base placement for the drill target...")
        ctx["base_placement_computed"] = False
        ctx["error_triggered"] = False
        self._user_choice = None

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("base_placement_computed") or ctx.get("error_triggered"):
            return

        target = ctx.get("current_drill_target")
        if target is None:
            node.get_logger().error(f"[{self.name}] No current_drill_target in context.")
            ctx["error_triggered"] = True
            return

        base = ctx.get("base_position")
        if base is None:
            node.get_logger().warn(f"[{self.name}] Waiting for odometry (base_position)...")
            return

        wall_x, wall_y = float(target[0]), float(target[1])
        robot_x, robot_y = float(base.x), float(base.y)

        # Associate the drill point to a detected wall. We prefer the wall's two
        # base endpoints (p1, p2) and derive the along-wall direction from them
        # directly -- this mirrors the endpoint-based yaw used by the verified
        # oliwalls navigate.py (refactored_fsm branch) and is independent of the
        # sign convention stored in the wall's `normal` field.
        wall = self._associate_wall(ctx, target)
        # No wall associated (map missing/mismatched, or the point sits too far
        # from every wall plane): refuse to navigate rather than drill at a bogus
        # orientation derived from a robot-relative fallback normal.
        if wall is None:
            node.get_logger().error(
                f"[{self.name}] Drill point ({wall_x:.3f}, {wall_y:.3f}) could not be "
                f"associated to any detected wall; refusing to navigate. Check that "
                f"detected_walls.yaml matches the current map (set 'detected_walls_yaml' "
                f"to override)."
            )
            ctx["error_triggered"] = True
            return

        # Along-wall unit direction straight from the wall endpoints.
        p1, p2 = wall["p1"], wall["p2"]
        ux, uy = float(p2[0]) - float(p1[0]), float(p2[1]) - float(p1[1])
        u_len = math.hypot(ux, uy)
        if u_len < 1e-6:
            node.get_logger().error(
                f"[{self.name}] Associated wall {wall.get('id')} has degenerate endpoints."
            )
            ctx["error_triggered"] = True
            return
        ux, uy = ux / u_len, uy / u_len
        # Outward normal toward the robot, taken as the perpendicular of the
        # wall edge (self-consistent with the endpoints we orient along).
        nx, ny = -uy, ux
        if (robot_x - wall_x) * nx + (robot_y - wall_y) * ny < 0.0:
            nx, ny = -nx, -ny
        wall_desc = f"wall id={wall.get('id')} p1={tuple(round(float(c), 2) for c in p1)} p2={tuple(round(float(c), 2) for c in p2)}"

        standoff = float(ctx.get("wall_standoff_m", 0.5))
        # Shift the base along the wall so the turret (which sits forward of
        # base_footprint) lines up with the drill point. Tune to the base->turret
        # forward offset; unfolding reports the residual along-wall error.
        along_offset = float(ctx.get("base_along_wall_offset_m", 0.0))

        # Yaw so the wall ends up on the robot's RIGHT (-Y): robot +Y (left) =
        # outward normal toward the robot, so robot +X runs along the wall.
        #   robot +Y = (-sin θ, cos θ) = (nx, ny) → θ = atan2(-nx, ny)
        yaw = math.atan2(-nx, ny)
        along_x, along_y = math.cos(yaw), math.sin(yaw)

        goal_x = wall_x + nx * standoff + along_x * along_offset
        goal_y = wall_y + ny * standoff + along_y * along_offset

        pose = PoseStamped()
        pose.header.frame_id = str(ctx.get("map_frame", "map"))
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        ctx["base_goal_pose"] = pose
        ctx["base_goal"] = (goal_x, goal_y, yaw)
        ctx["base_placement_computed"] = True

        node.get_logger().info(
            f"[{self.name}] Drill point ({wall_x:.3f}, {wall_y:.3f}); {wall_desc}; "
            f"outward normal ({nx:.3f}, {ny:.3f}) -> parallel base goal "
            f"({goal_x:.3f}, {goal_y:.3f}, yaw {math.degrees(yaw):.1f} deg), "
            f"wall on robot's RIGHT (-Y), standoff {standoff:.2f} m."
        )

    def _associate_wall(self, ctx, target):
        """Associate the drill point to a detected wall.

        Returns the matched wall dict (with ``p1``/``p2`` endpoints and outward
        ``normal``), or ``None`` if the wall map is missing/unusable or the point
        sits too far from every wall plane. The caller derives orientation from
        the wall endpoints and treats ``None`` as a hard error.
        """
        node = ctx["node"]
        wall_x, wall_y = float(target[0]), float(target[1])

        yaml_path = resolve_detected_walls_path(ctx.get("detected_walls_yaml"))
        if yaml_path is None:
            node.get_logger().warn(
                f"[{self.name}] detected_walls.yaml not found (robo_drill share or <ws>/src). "
                f"Set 'detected_walls_yaml' to override."
            )
            return None

        try:
            walls = load_walls(yaml_path)
            wall = associate_point_to_wall((wall_x, wall_y, float(target[2])), walls)
        except (OSError, KeyError, ValueError, yaml.YAMLError) as e:
            node.get_logger().warn(f"[{self.name}] Wall map '{yaml_path}' unusable ({e}).")
            return None

        if wall is None:
            node.get_logger().warn(
                f"[{self.name}] Drill point ({wall_x:.3f}, {wall_y:.3f}) not associated to "
                f"any wall in '{yaml_path}'."
            )
        return wall

    def check_transition(self, ctx):
        if not ctx.get("base_placement_computed") and not ctx.get("error_triggered"):
            return None
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
