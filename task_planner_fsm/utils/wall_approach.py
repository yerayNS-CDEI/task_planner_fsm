"""How the robot approaches a wall: base heading and the matching arm pose.

Policy only — the geometry lives in ``costmap_utils.wall_facing_yaw``. Kept in
one place because the two choices below are really one decision, and splitting
them silently breaks the scan:

- **along the wall** (legacy, base-driven sweep): the base drives down the wall
  and the sensor plate rides on its LEFT flank, so it unfolds to
  ``unfolded_fsm``.
- **facing the wall** (arm-driven sweep): the base parks square to the wall so
  the arm can sweep symmetrically about its centreline, so the plate has to
  unfold to the FRONT — ``unfolded_front_fsm``.

Facing the wall while unfolding to the left-flank pose points the plate along
the wall instead of at it; the reverse aims it into open space. Every state that
picks a heading or an unfold pose must go through here.
"""

DEFAULT_ALONG_WALL_POSE = "unfolded_fsm"
DEFAULT_FACING_WALL_POSE = "unfolded_front_fsm"


def should_face_wall(ctx) -> bool:
    """True when the base should park square to the wall rather than along it.

    ``nav_face_wall`` decides when set. Otherwise it follows ``sweep_use_arm``,
    since the arm-driven sweep requires the square-on approach — so once that
    becomes the default, the heading follows automatically with no second knob
    to remember.

    The separate ``nav_face_wall`` exists so the wall-facing approach can be
    exercised on its own: ``sweep_use_arm`` also switches ScanWall into
    partition mode, whose sweep is only half-migrated until the arm sweep
    executor lands (ARM_SWEEP_PLAN §11.5 S5).
    """
    explicit = ctx.get("nav_face_wall")
    if explicit is not None:
        return bool(explicit)
    return bool(ctx.get("sweep_use_arm", True))


def unfolded_pose_name(ctx) -> str:
    """Named arm pose whose plate orientation matches the approach heading.

    Override with ``scan_wall_unfolded_pose`` to force a specific pose (bench
    testing a new one, or pinning the old behaviour).
    """
    override = ctx.get("scan_wall_unfolded_pose")
    if override:
        return str(override)
    return DEFAULT_FACING_WALL_POSE if should_face_wall(ctx) else DEFAULT_ALONG_WALL_POSE


def should_approach_partition(ctx) -> bool:
    """True when NavigateToTarget should drive straight to the FIRST partition's
    scan pose instead of to the wall's reachable start.

    Without this the base makes two trips: NavigateToTarget parks at the start of
    the wall, the arm unfolds, and ScanWall then transits to the centre of
    partition 1 -- with the arm already out, at a standoff where Nav2 barely
    manoeuvres (ARM_SWEEP_PLAN §9). Going straight to the partition centre makes
    ScanWall's first transit a no-op.

    Requires BOTH halves of the arm-sweep approach: ``sweep_use_arm`` (or
    ScanWall is not in partition mode and there are no partitions to aim at) and
    :func:`should_face_wall` (or the base would arrive at the partition centre on
    the along-wall heading, which is not the pose ScanWall's skip test accepts).
    ``nav_approach_partition`` forces it either way.
    """
    explicit = ctx.get("nav_approach_partition")
    if explicit is not None:
        return bool(explicit)
    return bool(ctx.get("sweep_use_arm", True)) and should_face_wall(ctx)
