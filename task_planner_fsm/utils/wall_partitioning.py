"""Split reachable wall segments into arm-sweepable partitions.

Second stage of the two-stage split described in ARM_SWEEP_PLAN §7.A. The first
stage (``costmap_utils.reachable_wall_segments``) is obstacle-driven: it keeps
only the stretches of a scan line where a base cell exists within arm reach.
This module is purely geometric: it cuts each surviving segment into pieces
short enough for the arm to sweep from a single parked base pose.

Deliberately free of ROS imports so it can be unit-tested without a running
system.

Partitions are **equal length** within a segment rather than "max_len until the
remainder runs out". A trailing stub would park the base for a sweep of a few
centimetres, paying a full transit + approach + press cycle for almost no
coverage; spreading the length evenly costs nothing and avoids that.

Consecutive partitions **overlap** by ``overlap`` metres so the GPR line has no
gap at the seam where one sweep ends and the next begins.
"""

import math
from typing import List, Sequence, Tuple

Point3 = Tuple[float, float, float]
Segment = Tuple[Point3, Point3]

# Length comparisons are done on metre-scale values, so an absolute epsilon is
# both sufficient and easier to reason about than a relative one.
_EPS = 1e-9


def partition_count(length: float, max_len: float, overlap: float) -> int:
    """Number of equal, overlapping partitions needed to cover ``length``.

    ``n`` partitions of length ``p`` overlapping by ``overlap`` cover
    ``p + (n - 1) * (p - overlap)``. Setting that equal to ``length`` and
    requiring ``p <= max_len`` gives::

        n >= (length - overlap) / (max_len - overlap)

    so the smallest admissible ``n`` is the ceiling of that ratio (never < 1).
    """
    if max_len - overlap <= _EPS:
        raise ValueError(
            f"max_len ({max_len}) must exceed overlap ({overlap}); otherwise "
            f"each partition advances by zero and the split never terminates."
        )
    if length <= max_len + _EPS:
        return 1
    return max(1, math.ceil((length - overlap) / (max_len - overlap) - _EPS))


def partition_length(length: float, count: int, overlap: float) -> float:
    """Length of each of ``count`` equal partitions overlapping by ``overlap``.

    Inverse of :func:`partition_count`: solving
    ``count * p - (count - 1) * overlap == length`` for ``p``.
    """
    if count < 1:
        raise ValueError(f"count must be >= 1, got {count}")
    return (length + (count - 1) * overlap) / count


def partition_segment(
    p_start: Point3,
    p_end: Point3,
    max_len: float,
    overlap: float,
) -> List[Segment]:
    """Split one segment into equal, overlapping partitions of <= ``max_len``.

    Endpoints are interpolated in the XY plane; ``z`` is taken from ``p_start``
    and carried through unchanged, matching the convention of
    ``reachable_wall_segments`` (the scan line is horizontal, so both endpoints
    share a height).

    Returns partitions in ``p_start -> p_end`` order. A degenerate segment
    (zero length in XY) yields an empty list rather than a zero-length
    partition, so callers never park the base for a sweep that cannot move.
    """
    if overlap < 0.0:
        raise ValueError(f"overlap must be >= 0, got {overlap}")

    x1, y1 = float(p_start[0]), float(p_start[1])
    x2, y2 = float(p_end[0]), float(p_end[1])
    z = float(p_start[2]) if len(p_start) > 2 else 0.0

    length = math.hypot(x2 - x1, y2 - y1)
    if length <= _EPS:
        return []

    ux, uy = (x2 - x1) / length, (y2 - y1) / length

    count = partition_count(length, max_len, overlap)
    span = partition_length(length, count, overlap)
    stride = span - overlap

    partitions: List[Segment] = []
    for i in range(count):
        a = i * stride
        b = a + span
        # Clamp the final endpoint instead of trusting accumulated float error;
        # the last partition must land exactly on p_end so coverage is complete.
        if i == count - 1:
            b = length
        partitions.append((
            (x1 + ux * a, y1 + uy * a, z),
            (x1 + ux * b, y1 + uy * b, z),
        ))
    return partitions


def partition_segments(
    segments: Sequence[Segment],
    max_len: float,
    overlap: float,
) -> List[Segment]:
    """Apply :func:`partition_segment` to every segment, preserving order.

    The result is a drop-in replacement for the reachable-segment list the
    ``ScanWall`` phase machine already iterates over: each partition is simply
    "a segment" as far as that machine is concerned (ARM_SWEEP_PLAN §7.A).
    """
    partitions: List[Segment] = []
    for p_start, p_end in segments:
        partitions.extend(partition_segment(p_start, p_end, max_len, overlap))
    return partitions


def backoff_lengths(
    max_len: float,
    backoff: float,
    min_len: float,
) -> List[float]:
    """Descending partition-length candidates to try when validation fails.

    Per ARM_SWEEP_PLAN §7.B, a partition the arm cannot sweep is *not* halved:
    the maximum length is reduced by ``backoff`` and the affected reachable
    segment is re-partitioned as a whole, so equal lengths, full coverage and
    the configured overlap all survive the retry::

        0.80 m fails -> retry the segment at 0.60 m -> 0.40 m -> 0.20 m

    The list always contains ``max_len`` itself so callers can iterate over it
    directly. It stops at ``min_len``; below that the caller reports the region
    unreachable rather than looping forever.
    """
    if backoff <= _EPS:
        raise ValueError(f"backoff must be > 0, got {backoff}")
    if min_len <= _EPS:
        raise ValueError(f"min_len must be > 0, got {min_len}")

    lengths: List[float] = []
    current = float(max_len)
    while current >= min_len - _EPS:
        lengths.append(current)
        current -= backoff
    return lengths


def partition_centre_and_tangent(
    p_start: Point3,
    p_end: Point3,
) -> Tuple[Point3, Tuple[float, float, float]]:
    """Return ``(centre, tangent)`` for a partition, per ARM_SWEEP_PLAN §7.A.

    ``centre`` is where along the wall the base parks (before the standoff
    offset is applied along the wall normal); ``tangent`` is the authoritative
    arm sweep direction, unit length and horizontal.

    The plate-derived direction ``cross(z_plate, up)`` is deliberately *not*
    used here: §4.2 keeps it as a sanity check only, because it depends on the
    arm's current pose whereas this depends only on the partition geometry.
    """
    x1, y1 = float(p_start[0]), float(p_start[1])
    x2, y2 = float(p_end[0]), float(p_end[1])
    z = float(p_start[2]) if len(p_start) > 2 else 0.0

    length = math.hypot(x2 - x1, y2 - y1)
    if length <= _EPS:
        raise ValueError("Partition endpoints must differ in the XY plane.")

    centre = (0.5 * (x1 + x2), 0.5 * (y1 + y2), z)
    tangent = ((x2 - x1) / length, (y2 - y1) / length, 0.0)
    return centre, tangent


def sweep_line_order(wall_data, target_point) -> Tuple[Point3, Point3]:
    """Order a wall's two scan-line endpoints as ``(near, far)`` from ``target_point``.

    The serpentine rule: sweep away from the end the robot is already at, so
    consecutive line heights meet end to end instead of driving back down the
    wall between them.

    Shared by ``NavigateToTarget`` and ``ScanWall`` because both must derive the
    **same** partition ordering. If they disagreed, NavigateToTarget would
    approach the scan pose of what ScanWall considers the *last* partition, and
    ScanWall would immediately transit the whole wall to get back.

    ``target_point`` need not lie on the line -- it is usually a clamped
    reachable-segment endpoint -- so the nearer end is chosen by distance rather
    than by projecting onto the line.
    """
    d0 = math.hypot(target_point[0] - wall_data[0][0],
                    target_point[1] - wall_data[0][1])
    d1 = math.hypot(target_point[0] - wall_data[1][0],
                    target_point[1] - wall_data[1][1])
    return (wall_data[0], wall_data[1]) if d0 <= d1 else (wall_data[1], wall_data[0])


def next_backoff_length(current: float, backoff: float, min_len: float):
    """Next shorter maximum partition length to try, or ``None`` at the floor.

    The incremental form of :func:`backoff_lengths`, for a retry loop that learns
    one failure at a time rather than planning the whole ladder up front::

        0.80 m rejected -> 0.60 -> 0.40 -> 0.20 -> None (report unreachable)

    ARM_SWEEP_PLAN §7.B is explicit that a rejected partition must NOT simply be
    halved: the affected reachable segment is re-partitioned as a whole at the
    reduced maximum, so equal lengths, full coverage and the configured overlap
    all survive the retry.
    """
    if backoff <= _EPS:
        raise ValueError(f"backoff must be > 0, got {backoff}")
    nxt = float(current) - float(backoff)
    return nxt if nxt >= float(min_len) - _EPS else None
