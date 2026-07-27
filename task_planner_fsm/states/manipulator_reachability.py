import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration

from ..state import State
from ..utils.gantry_kinematics import TIP_OFFSET_DEFAULT, fixed_tip_x

# Coarse base-frame sanity limits: is the drill point anywhere near the robot?
# These are intentionally loose; the precise check is the gantry-frame
# along-wall alignment below.
MAX_REACH_X_M = 1.2
MAX_REACH_Y_M = 1.2

NEXT_STATE_OPTIONS = [
    "ManipulatorUnfolding",
    "NearbyPointSelection",
    "Error",
]

class ManipulatorReachability(State):
    """Check whether the parked robot can reach the drill point with its gantry.

    Two checks are performed:

    1. **Coarse base-frame check** (fast, no TF required):
       The drill point must be within MAX_REACH_X/Y_M of the base_footprint in
       the robot body frame.  This catches gross navigation failures.

    2. **Fine gantry along-wall check** (requires TF to turret_link):
       The gantry has no joint in the turret-X (along-wall) direction — the
       drill tip's turret-X position is geometrically fixed at
       ``fixed_tip_x(tip_offset)``.  If the drill point's turret-X
       deviates from this expected value by more than ``gantry_along_wall_tol_m``
       the base parked at the wrong along-wall position and must be recomputed.
       The measured residual is fed back into ``base_along_wall_offset_m`` in
       context so the next ``BasePlacementComputation`` self-corrects.
    """

    def __init__(self, name):
        super().__init__(name)

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(
            f"[{self.name}] Checking manipulator reachability to the drill point..."
        )
        ctx["manipulator_reachability_computed"] = False
        ctx["error_triggered"] = False
        ctx["target_reachable"] = False
        self._user_choice = None
        self._tf_wait_start = None

    def run(self, ctx):
        node = ctx["node"]

        if ctx.get("manipulator_reachability_computed") or ctx.get("error_triggered"):
            return

        target = ctx.get("current_drill_target")
        if target is None:
            node.get_logger().error(f"[{self.name}] No current_drill_target in context.")
            ctx["error_triggered"] = True
            return

        # Robot's current pose (from /rtabmap/odom, see fsm_node).
        base = ctx.get("base_position")
        orientation = ctx.get("base_orientation")
        if base is None or orientation is None:
            node.get_logger().warn(f"[{self.name}] Waiting for odometry (base pose)...")
            return

        # ── 1. Coarse base-frame check ─────────────────────────────────────
        dx_map = float(target[0]) - float(base.x)
        dy_map = float(target[1]) - float(base.y)
        yaw = self._yaw_from_quaternion(orientation)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        dx_base = cos_y * dx_map + sin_y * dy_map
        dy_base = -sin_y * dx_map + cos_y * dy_map

        if abs(dx_base) > MAX_REACH_X_M or abs(dy_base) > MAX_REACH_Y_M:
            ctx["target_base_offset"] = (dx_base, dy_base)
            ctx["target_reachable"] = False
            ctx["manipulator_reachability_computed"] = True
            node.get_logger().warn(
                f"[{self.name}] Drill point OUT OF COARSE REACH (base-frame offset "
                f"x={dx_base:.3f} [max {MAX_REACH_X_M:.3f}], "
                f"y={dy_base:.3f} [max {MAX_REACH_Y_M:.3f}]); recomputing base placement."
            )
            return

        # ── 2. Fine gantry along-wall check (turret frame via TF) ──────────
        #
        # The gantry tip's X in turret frame is kinematically fixed:
        #   expected_tip_x = fixed_tip_x(tip_offset)
        # (the rotary stage maps the tip offset onto turret X independently of the
        # rotate angle and every other joint — see gantry_kinematics.py.)
        # If the drill point's turret-X deviates beyond the tolerance, the base
        # stopped at the wrong along-wall position; the residual is absorbed into
        # base_along_wall_offset_m so BasePlacementComputation self-corrects.
        tf_buffer = ctx.get("tf_buffer")
        gantry_frame = str(ctx.get("gantry_base_frame", "turret_link"))
        map_frame = str(ctx.get("map_frame", "map"))
        along_tol = float(ctx.get("gantry_along_wall_tol_m", 0.50))
        tf_timeout = float(ctx.get("reachability_tf_timeout_s", 10.0))

        along_wall_ok = True
        along_wall_residual = 0.0
        tf_checked = False

        if tf_buffer is not None:
            if self._tf_wait_start is None:
                self._tf_wait_start = time.time()

            try:
                if not tf_buffer.can_transform(
                    gantry_frame, map_frame, rclpy.time.Time(), Duration(seconds=0.0)
                ):
                    if time.time() - self._tf_wait_start > tf_timeout:
                        node.get_logger().warn(
                            f"[{self.name}] TF {map_frame}->{gantry_frame} not available "
                            f"after {tf_timeout:.0f} s; skipping along-wall check."
                        )
                        # along_wall_ok stays True — ManipulatorUnfolding will gate it.
                    else:
                        node.get_logger().warn(
                            f"[{self.name}] Waiting for TF {map_frame}->{gantry_frame}...",
                            throttle_duration_sec=2.0,
                        )
                        return  # retry next FSM tick
                else:
                    pt_map = PointStamped()
                    pt_map.header.frame_id = map_frame
                    pt_map.point.x = float(target[0])
                    pt_map.point.y = float(target[1])
                    pt_map.point.z = float(target[2])
                    pt_turret = tf_buffer.transform(
                        pt_map, gantry_frame, timeout=Duration(seconds=0.0)
                    )

                    tip_offset = ctx.get("drill_tip_offset", TIP_OFFSET_DEFAULT)
                    # The gantry pins the tip's turret-X regardless of rotate angle.
                    expected_tip_x = fixed_tip_x(tip_offset)
                    along_wall_residual = pt_turret.point.x - expected_tip_x
                    along_wall_ok = abs(along_wall_residual) <= along_tol
                    tf_checked = True

            except Exception as e:
                if time.time() - self._tf_wait_start > tf_timeout:
                    node.get_logger().warn(
                        f"[{self.name}] TF unavailable after {tf_timeout:.0f} s; "
                        f"skipping along-wall check."
                    )
                else:
                    node.get_logger().warn(
                        f"[{self.name}] Waiting for TF {map_frame}->{gantry_frame} ({e})...",
                        throttle_duration_sec=2.0,
                    )
                    return  # retry next FSM tick

        reachable = along_wall_ok  # coarse check already passed above

        ctx["target_base_offset"] = (dx_base, dy_base)
        ctx["target_reachable"] = reachable
        ctx["manipulator_reachability_computed"] = True

        if reachable:
            along_detail = (
                f", along-wall residual {along_wall_residual:.3f} m" if tf_checked else ""
            )
            node.get_logger().info(
                f"[{self.name}] Drill point reachable: base-frame offset "
                f"|x|={abs(dx_base):.3f}<={MAX_REACH_X_M:.3f} m, "
                f"|y|={abs(dy_base):.3f}<={MAX_REACH_Y_M:.3f} m{along_detail}."
            )
        else:
            # Absorb the residual into the along-wall offset so the next
            # BasePlacementComputation corrects the parking position automatically:
            #   residual < 0 → robot too far forward  → offset decreases
            #   residual > 0 → robot too far backward → offset increases
            old_offset = float(ctx.get("base_along_wall_offset_m", 0.0))
            new_offset = old_offset + along_wall_residual
            ctx["base_along_wall_offset_m"] = new_offset
            node.get_logger().warn(
                f"[{self.name}] Drill point mis-aligned along the wall by "
                f"{along_wall_residual:.3f} m (> {along_tol:.3f} m); "
                f"base_along_wall_offset_m adjusted {old_offset:.3f} -> {new_offset:.3f} m, "
                f"recomputing base placement."
            )

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def check_transition(self, ctx):
        if not ctx.get("manipulator_reachability_computed") and not ctx.get("error_triggered"):
            return None
        if ctx.get("error_triggered"):
            return "Error"
        # Out-of-reach is routed automatically back to base placement; the
        # reachable path keeps the manual next-state picker.
        if not ctx.get("target_reachable", False):
            return "BasePlacementComputation"
        return self.select_next_state(ctx, NEXT_STATE_OPTIONS)
