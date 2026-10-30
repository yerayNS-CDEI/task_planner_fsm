"""The base's pose in the map frame, from TF.

Walls, scan lines, Nav2 goals and sweep segments are all map-frame.
ctx["base_position"] comes from /rtabmap/odom, the odom frame, and is off from
them by the whole map->odom correction once rtabmap has relocalized. Anything
compared against map-frame geometry reads the pose here instead, and never
falls back to odom.
"""

from math import atan2

import rclpy.time
from rclpy.duration import Duration

# turret_footprint first: the frame the omni Nav2 config drives
# (nav2_params_omni robot_base_frame). Override with the nav_base_frame ctx/ROS
# param to match the launched Nav2 config.
BASE_FRAMES = ("turret_footprint", "base_footprint", "base_link", "base", "chassis")


def base_pose_map(ctx):
    """Current base pose ``(x, y, yaw)`` in the map frame, or None without TF."""
    tf_buffer = ctx.get("tf_buffer")
    if tf_buffer is None:
        return None
    primary = str(ctx.get("nav_base_frame", "turret_footprint"))
    frames = (primary,) + tuple(f for f in BASE_FRAMES if f != primary)
    for frame in frames:
        try:
            if not tf_buffer.can_transform(
                "map", frame, rclpy.time.Time(), Duration(seconds=0.2)
            ):
                continue
            tf = tf_buffer.lookup_transform(
                "map", frame, rclpy.time.Time(), Duration(seconds=0.5)
            )
        except Exception:
            continue
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), yaw
    return None
