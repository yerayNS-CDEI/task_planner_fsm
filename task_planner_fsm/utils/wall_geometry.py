"""Wall geometry helpers shared between FSM states."""

import math
from typing import Tuple

Point3 = Tuple[float, float, float]
Vec3 = Tuple[float, float, float]


def inward_normal_from_wall_points(p1: Point3, p2: Point3) -> Vec3:
    """Return the inward (scan-facing) wall normal as a unit vector in map frame.

    Convention: p1 is the bottom-left vertex, p2 is the top-right vertex of the
    wall. The exterior (scan) side lies on the clockwise perpendicular of the
    p1 -> p2 edge in the XY plane; the inward normal points the opposite way
    and is the direction the EE z-axis should look along while scanning.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        raise ValueError("Wall points must differ in the XY plane.")
    dx /= length
    dy /= length
    # CW perpendicular (dy, -dx) is the exterior side; flip for the inward normal.
    return (-dy, dx, 0.0)


def ee_rpy_deg_from_inward_normal(normal: Vec3) -> Vec3:
    """Return EE (roll, pitch, yaw) in degrees so its z-axis aligns with ``normal``.

    Roll is fixed at 0, preserving the original convention that keeps the EE
    x-axis pointing down for vertical walls. For a horizontal inward normal
    this reduces to pitch=90, yaw=atan2(ny, nx); the previous hard-coded
    (0, 90, 0) is recovered when the normal is (+1, 0, 0).
    """
    nx, ny, nz = normal
    pitch = math.degrees(math.atan2(math.hypot(nx, ny), nz))
    yaw = math.degrees(math.atan2(ny, nx))
    return (0.0, pitch, yaw)
