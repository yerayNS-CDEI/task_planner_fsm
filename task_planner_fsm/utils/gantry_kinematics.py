"""Inverse kinematics for the robo_drill "PokEye" Cartesian gantry.

The gantry chain (see robo_drill_description/description/pokeye_base.urdf) hangs
off ``turret_link`` (rigidly fixed to the mobile base, so it shares the base
yaw). After a redesign the stage order is now:

    turret_link
      -> stage1_lift_joint      (prismatic, +Z_turret)  origin (0.2905, 0, 0)
      -> stage2_lift_joint      (prismatic, +Z_turret)  origin (0.15,   0, 0.423)
      -> stage3_rotate_joint    (revolute,  about +X_t)  origin (0.08,   0, 0.10),  rpy(pi/2,0,pi/2)
      -> stage4_horizontal_joint(prismatic, slide)       origin (0, 0, 0.08),       rpy(-pi/2,-pi/2,0)
      -> drill_tip              (fixed)                  origin = configurable tool offset

IMPORTANT: the rotary stage (stage 3) is now UPSTREAM of the horizontal slide
(stage 4) — the two used to be the other way round. Because the slide hangs off
the rotary joint, the slide's travel direction rotates with ``rotate_angle``
instead of the rotation merely re-aiming a fixed tool offset.

Frames: the two lifts and the rotary origin are all rotationally aligned with
``turret_link`` (their intermediate rpy are zero), so the lift pair moves purely
along turret +Z and the rotary origin sits at a fixed turret-X. The rotary joint
spins about turret +X (the constant rpy(pi/2,0,pi/2) maps stage3's local +Z onto
turret +X). Spinning it rotates the slide axis in the turret Y–Z plane:

    tip_x = ROTATE_ORIGIN_X + STAGE4_ORIGIN_Z + tip_offset[0]        (constant)
    tip_y =  cos(theta) * (slide + tip_offset[1]) - sin(theta) * tip_offset[2]
    tip_z =  ROTATE_ORIGIN_Z0 + lift1 + lift2
           + sin(theta) * (slide + tip_offset[1]) + cos(theta) * tip_offset[2]

Two consequences drive the IK design:

* ``tip_x`` (along the wall) is fixed by geometry — it is returned as
  ``along_wall_residual`` for the caller to gate on (≈0 when the base is parked
  correctly), exactly as before.
* The rotary angle picks which turret axis the slide feeds along:
    - ``theta = 0``   -> slide feeds along turret ±Y  (WALL drilling: depth/retract).
    - ``theta = ±pi/2`` -> slide feeds along turret ±Z (CEILING/FLOOR drilling).
  ``rotate_angle`` is therefore chosen by the caller (normally one of 0, +pi/2,
  -pi/2) and passed in; this module does not search for it.

  In the vertical-feed modes (``theta`` near ±pi/2) the slide no longer moves the
  tip in turret-Y, so the tip's turret-Y coordinate becomes geometry-fixed too and
  is reported as ``cross_residual`` (another base-positioning residual, ≈0 in wall
  mode). There, height is provided by the lifts and the slide acts as the drill
  feed, so the redundant vertical DOF is resolved by keeping the slide retracted
  and only spilling onto it when the lifts saturate.

The returned ``joints`` are ordered to match the gantry_position_controller:
``(lift1, lift2, rotate, horizontal)`` — i.e. stage1, stage2, stage3, stage4.

IMPORTANT: the constants below mirror the URDF. If you edit the ``drill_tip_*``
values or the gantry joint origins/limits in pokeye_base.urdf, update them here
(or pass ``tip_offset`` explicitly).
"""

import math
from dataclasses import dataclass
from typing import Tuple

Vec3 = Tuple[float, float, float]

# Fixed joint-origin translations from the URDF (turret frame up to stage 3).
STAGE1_ORIGIN_X = 0.2905
STAGE2_ORIGIN_X = 0.15
STAGE3_ORIGIN_X = 0.08
STAGE2_ORIGIN_Z = 0.423
STAGE3_ORIGIN_Z = 0.10
STAGE4_ORIGIN_Z = 0.08   # stage4 joint origin's local +Z; maps to turret +X via R3.

# turret-X of the stage3 (rotary) origin. Constant: the lifts move only along +Z
# and the rotary joint is downstream, so nothing shifts this.
ROTATE_ORIGIN_X = STAGE1_ORIGIN_X + STAGE2_ORIGIN_X + STAGE3_ORIGIN_X  # 0.5205
# turret-Z of the rotary origin at zero lift.
ROTATE_ORIGIN_Z0 = STAGE2_ORIGIN_Z + STAGE3_ORIGIN_Z                   # 0.523

# Default drill_tip offset in the stage4_horizontal_link frame (matches URDF
# drill_tip_x/y/z; the tip's rpy only sets orientation, not this position).
TIP_OFFSET_DEFAULT: Vec3 = (0.05, -0.54, 0.0)

# Joint limits [lower, upper] from pokeye_base.urdf.
LIFT1_LIMITS = (-0.47, 0.40)
LIFT2_LIMITS = (0.0, 1.0)
HORIZONTAL_LIMITS = (-0.32, 0.32)
ROTATE_LIMITS = (-math.pi, math.pi)

# Achievable range of the combined lift1+lift2 travel.
LIFT_SUM_LIMITS = (LIFT1_LIMITS[0] + LIFT2_LIMITS[0], LIFT1_LIMITS[1] + LIFT2_LIMITS[1])

# |cos(theta)| below this is treated as a pure vertical-feed mode (theta = ±pi/2),
# where the slide can no longer position the tip in turret-Y. The intended discrete
# angles (0, ±pi/2) sit far from this threshold, so classification is unambiguous.
_COS_SINGULAR = 1e-3


@dataclass
class GantryIKResult:
    # (lift1, lift2, rotate, horizontal) — gantry_position_controller order.
    joints: Tuple[float, float, float, float]
    along_wall_residual: float  # tip target turret-X minus the fixed gantry X (m)
    cross_residual: float       # tip target turret-Y minus fixed Y (m); ~0 in wall mode
    feasible: bool
    reason: str


def fixed_tip_x(tip_offset: Vec3 = TIP_OFFSET_DEFAULT) -> float:
    """turret-X the drill tip is pinned to, independent of every joint value."""
    return ROTATE_ORIGIN_X + STAGE4_ORIGIN_Z + float(tip_offset[0])


def _within(value: float, limits: Tuple[float, float]) -> bool:
    return limits[0] - 1e-6 <= value <= limits[1] + 1e-6


def _clamp(value: float, limits: Tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def _distribute_lifts(total: float) -> Tuple[float, float, bool]:
    """Split a required lift1+lift2 sum across the two lift joints within limits.

    Prefers to keep lift1 at 0 and use lift2 (the longer-travel stage); spills
    the remainder onto lift1. Returns (lift1, lift2, ok).
    """
    lift2 = _clamp(total, LIFT2_LIMITS)
    lift1 = total - lift2
    if not _within(lift1, LIFT1_LIMITS):
        lift1 = _clamp(lift1, LIFT1_LIMITS)
        lift2 = total - lift1
    ok = _within(lift1, LIFT1_LIMITS) and _within(lift2, LIFT2_LIMITS)
    return lift1, lift2, ok


def gantry_ik(
    tip_target_turret: Vec3,
    rotate_angle: float,
    tip_offset: Vec3 = TIP_OFFSET_DEFAULT,
    lateral_tol: float = 0.0,
) -> GantryIKResult:
    """Solve gantry joints so ``drill_tip`` reaches ``tip_target_turret``.

    ``tip_target_turret`` is the desired tool-tip position in the ``turret_link``
    frame; ``rotate_angle`` is the commanded stage3 rotation (normally 0, +pi/2 or
    -pi/2 — chosen by the caller to pick wall vs. ceiling/floor drilling). The two
    lifts and the horizontal slide are solved from the target; the tip's turret-X
    is geometry-fixed and returned as ``along_wall_residual`` for the caller to gate
    on. In the vertical-feed modes the tip's turret-Y is also geometry-fixed and
    returned as ``cross_residual``.

    ``lateral_tol`` is an optional positioning tolerance [m] applied to the
    horizontal-slide limits. Values within this margin of a hard limit are accepted
    and clamped to the limit so the joint command stays within the hardware range.
    """
    tx, ty, tz = (float(tip_target_turret[0]),
                  float(tip_target_turret[1]),
                  float(tip_target_turret[2]))
    ox, oy, oz = float(tip_offset[0]), float(tip_offset[1]), float(tip_offset[2])

    c, s = math.cos(rotate_angle), math.sin(rotate_angle)

    # Along-wall (turret-X) residual: fixed by geometry regardless of the joints.
    along_wall_residual = tx - (ROTATE_ORIGIN_X + STAGE4_ORIGIN_Z + ox)

    # Contribution of the tool offset, rotated into the turret Y–Z plane.
    off_y = c * oy - s * oz
    off_z = s * oy + c * oz

    # In-plane target the slide + lifts must produce (offset removed):
    #   c * slide            = y_target
    #   s * slide + lift_sum = z_target
    y_target = ty - off_y
    z_target = tz - ROTATE_ORIGIN_Z0 - off_z

    if abs(c) >= _COS_SINGULAR:
        # WALL-type mode: the slide controls turret-Y, the lifts finish turret-Z.
        cross_residual = 0.0
        horizontal = y_target / c
        lift_sum = z_target - s * horizontal
        lift1, lift2, lifts_ok = _distribute_lifts(lift_sum)
    else:
        # Vertical-feed mode (theta ≈ ±pi/2): the slide feeds along turret-Z and can
        # no longer set turret-Y, so Y becomes a base-positioning residual. Height is
        # taken by the lifts, keeping the slide retracted; only if the lifts saturate
        # do we extend the slide (its travel adds s*slide to Z) to cover the shortfall.
        cross_residual = y_target
        # Height from the lifts (bounded to their combined travel); the slide feed
        # covers any remainder (its travel adds s*slide to Z, with s = ±1 here).
        lift_sum = _clamp(z_target, LIFT_SUM_LIMITS)
        horizontal = (z_target - lift_sum) / s
        lift1, lift2, lifts_ok = _distribute_lifts(lift_sum)

    reason = "ok"
    feasible = True

    tol_limits = (HORIZONTAL_LIMITS[0] - lateral_tol, HORIZONTAL_LIMITS[1] + lateral_tol)
    if not _within(horizontal, tol_limits):
        feasible = False
        reason = (f"horizontal {horizontal:.3f} out of {HORIZONTAL_LIMITS} "
                  f"(tol ±{lateral_tol:.3f})")
    else:
        # Clamp to the hard limits so the joint command is always valid even if the
        # IK landed just outside a limit within tolerance.
        horizontal = _clamp(horizontal, HORIZONTAL_LIMITS)

    if feasible and not lifts_ok:
        feasible = False
        reason = (f"lift sum {lift1 + lift2:.3f} not reachable by lifts "
                  f"{LIFT1_LIMITS}+{LIFT2_LIMITS}")
    if feasible and not _within(rotate_angle, ROTATE_LIMITS):
        feasible = False
        reason = f"rotate {rotate_angle:.3f} out of {ROTATE_LIMITS}"

    return GantryIKResult(
        joints=(lift1, lift2, rotate_angle, horizontal),
        along_wall_residual=along_wall_residual,
        cross_residual=cross_residual,
        feasible=feasible,
        reason=reason,
    )
