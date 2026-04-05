import math
from typing import Dict, Tuple


PREDEFINED_WALLS = [
    ((3.0, 0.0, 2.0), (3.0, -3.0, 3.0)),
    ((9.0, 0.0, 0.19), (9.0, -4.5, 2.0)),
    ((10.0, 0.0, 0.2), (10.0, -4.5, 3.0)),
]


def build_wall_data(
    p1: Tuple[float, float, float],
    p2: Tuple[float, float, float],
    offset: float = 0.6,
) -> Dict[str, Tuple]:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length == 0:
        raise ValueError("Wall points must be different.")

    dx /= length
    dy /= length
    nx = dy
    ny = -dx

    scan_start = (
        p1[0] + offset * dx + nx * offset,
        p1[1] + offset * dy + ny * offset,
        p1[2],
    )
    scan_end = (
        p2[0] - offset * dx + nx * offset,
        p2[1] - offset * dy + ny * offset,
        p2[2],
    )

    return {
        "original": (tuple(p1), tuple(p2)),
        "scan_line": (scan_start, scan_end),
    }
