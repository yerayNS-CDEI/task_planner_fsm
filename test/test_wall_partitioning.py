"""Unit tests for task_planner_fsm.utils.wall_partitioning.

Pure geometry — no ROS, no simulator. Run with:

    python3 -m pytest test/test_wall_partitioning.py -v
"""

import math

import pytest

from task_planner_fsm.utils.wall_partitioning import (
    backoff_lengths,
    partition_centre_and_tangent,
    partition_count,
    partition_length,
    partition_segment,
    partition_segments,
    next_backoff_length,
)

MAX_LEN = 0.8
OVERLAP = 0.05
TOL = 1e-9


def seg_length(segment):
    (x1, y1, _), (x2, y2, _) = segment
    return math.hypot(x2 - x1, y2 - y1)


def assert_covers(partitions, p_start, p_end, overlap):
    """Partitions must start at p_start, end at p_end, and leave no gap."""
    assert partitions
    assert partitions[0][0] == pytest.approx(p_start, abs=1e-9)
    assert partitions[-1][1] == pytest.approx(p_end, abs=1e-9)

    for previous, current in zip(partitions, partitions[1:]):
        # Next partition starts before the previous one ends, by exactly overlap.
        gap_start = seg_length((partitions[0][0], current[0]))
        prev_end = seg_length((partitions[0][0], previous[1]))
        assert prev_end - gap_start == pytest.approx(overlap, abs=1e-9)


# ---------------------------------------------------------------------------
# partition_count / partition_length
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "length, expected",
    [
        (0.5, 1),    # shorter than max_len -> single partition
        (0.8, 1),    # exactly max_len -> still one
        (0.81, 2),   # just over -> two
        (1.5, 2),
        (1.6, 3),
        (6.0, 8),
    ],
)
def test_partition_count(length, expected):
    assert partition_count(length, MAX_LEN, OVERLAP) == expected


def test_partition_count_never_below_one():
    assert partition_count(0.01, MAX_LEN, OVERLAP) == 1


def test_partition_count_rejects_overlap_ge_max_len():
    with pytest.raises(ValueError):
        partition_count(5.0, 0.05, 0.05)
    with pytest.raises(ValueError):
        partition_count(5.0, 0.04, 0.05)


def test_partition_length_inverts_the_cover_equation():
    length, count = 6.0, 8
    span = partition_length(length, count, OVERLAP)
    # n spans minus (n-1) overlaps must reconstruct the original length.
    assert count * span - (count - 1) * OVERLAP == pytest.approx(length)


def test_partition_length_respects_max_len():
    for length in (0.5, 0.81, 1.5, 2.4, 6.0, 13.7):
        count = partition_count(length, MAX_LEN, OVERLAP)
        assert partition_length(length, count, OVERLAP) <= MAX_LEN + TOL


# ---------------------------------------------------------------------------
# partition_segment
# ---------------------------------------------------------------------------

def test_single_partition_when_segment_fits():
    p_start, p_end = (0.0, 0.0, 1.2), (0.6, 0.0, 1.2)
    partitions = partition_segment(p_start, p_end, MAX_LEN, OVERLAP)
    assert len(partitions) == 1
    assert partitions[0] == (p_start, p_end)


def test_partitions_are_equal_length_no_stub():
    partitions = partition_segment((0.0, 0.0, 0.0), (2.5, 0.0, 0.0), MAX_LEN, OVERLAP)
    lengths = [seg_length(p) for p in partitions]
    assert len(partitions) > 1
    # The point of equal splitting: no short trailing partition.
    assert max(lengths) - min(lengths) == pytest.approx(0.0, abs=1e-9)
    assert max(lengths) <= MAX_LEN + TOL


def test_partitions_cover_the_segment_with_the_requested_overlap():
    p_start, p_end = (1.0, 2.0, 0.9), (1.0, 7.0, 0.9)
    partitions = partition_segment(p_start, p_end, MAX_LEN, OVERLAP)
    assert_covers(partitions, p_start, p_end, OVERLAP)


def test_zero_overlap_is_allowed():
    p_start, p_end = (0.0, 0.0, 0.0), (2.4, 0.0, 0.0)
    partitions = partition_segment(p_start, p_end, MAX_LEN, 0.0)
    assert_covers(partitions, p_start, p_end, 0.0)
    assert len(partitions) == 3
    assert all(seg_length(p) == pytest.approx(0.8) for p in partitions)


def test_z_is_carried_through_from_the_start_point():
    partitions = partition_segment((0.0, 0.0, 1.75), (3.0, 0.0, 1.75), MAX_LEN, OVERLAP)
    for start, end in partitions:
        assert start[2] == pytest.approx(1.75)
        assert end[2] == pytest.approx(1.75)


def test_diagonal_segment_partitions_stay_on_the_line():
    p_start, p_end = (0.0, 0.0, 0.5), (3.0, 4.0, 0.5)   # length 5.0
    partitions = partition_segment(p_start, p_end, MAX_LEN, OVERLAP)
    assert_covers(partitions, p_start, p_end, OVERLAP)
    for start, end in partitions:
        for x, y, _ in (start, end):
            # The line is y = (4/3) x; every endpoint must satisfy it.
            assert y == pytest.approx(4.0 * x / 3.0, abs=1e-9)


def test_degenerate_segment_yields_nothing():
    assert partition_segment((1.0, 1.0, 0.0), (1.0, 1.0, 0.0), MAX_LEN, OVERLAP) == []


def test_negative_overlap_rejected():
    with pytest.raises(ValueError):
        partition_segment((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), MAX_LEN, -0.01)


def test_reversed_segment_is_partitioned_in_the_given_order():
    forward = partition_segment((0.0, 0.0, 0.0), (2.5, 0.0, 0.0), MAX_LEN, OVERLAP)
    reverse = partition_segment((2.5, 0.0, 0.0), (0.0, 0.0, 0.0), MAX_LEN, OVERLAP)
    assert len(forward) == len(reverse)
    # Serpentine relies on direction being preserved, not normalised away.
    assert reverse[0][0][0] == pytest.approx(2.5)
    assert reverse[-1][1][0] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# partition_segments
# ---------------------------------------------------------------------------

def test_partition_segments_preserves_order_and_concatenates():
    segments = [
        ((0.0, 0.0, 1.0), (1.5, 0.0, 1.0)),
        ((3.0, 0.0, 1.0), (3.6, 0.0, 1.0)),
    ]
    partitions = partition_segments(segments, MAX_LEN, OVERLAP)
    assert len(partitions) == 2 + 1
    assert partitions[0][0] == pytest.approx((0.0, 0.0, 1.0))
    assert partitions[1][1] == pytest.approx((1.5, 0.0, 1.0))
    assert partitions[2] == ((3.0, 0.0, 1.0), (3.6, 0.0, 1.0))


def test_partition_segments_skips_degenerate_segments():
    segments = [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.6, 0.0, 0.0)),
    ]
    assert len(partition_segments(segments, MAX_LEN, OVERLAP)) == 1


def test_partition_segments_empty_input():
    assert partition_segments([], MAX_LEN, OVERLAP) == []


def test_plan_worked_example_six_metre_wall():
    """ARM_SWEEP_PLAN §6 quotes ~8 partitions for a 6 m wall at 0.8 m."""
    partitions = partition_segment((0.0, 0.0, 0.0), (6.0, 0.0, 0.0), MAX_LEN, OVERLAP)
    assert len(partitions) == 8


# ---------------------------------------------------------------------------
# backoff_lengths
# ---------------------------------------------------------------------------

def test_backoff_lengths_matches_the_plan_sequence():
    assert backoff_lengths(0.8, 0.2, 0.2) == pytest.approx([0.8, 0.6, 0.4, 0.2])


def test_backoff_includes_the_starting_length():
    assert backoff_lengths(0.8, 0.2, 0.2)[0] == pytest.approx(0.8)


def test_backoff_stops_at_min_len():
    assert min(backoff_lengths(0.8, 0.3, 0.25)) >= 0.25


def test_backoff_single_entry_when_min_equals_max():
    assert backoff_lengths(0.8, 0.2, 0.8) == pytest.approx([0.8])


def test_backoff_rejects_non_positive_steps():
    with pytest.raises(ValueError):
        backoff_lengths(0.8, 0.0, 0.2)
    with pytest.raises(ValueError):
        backoff_lengths(0.8, 0.2, 0.0)


def test_every_backoff_length_still_partitions_a_hard_segment():
    for max_len in backoff_lengths(0.8, 0.2, 0.2):
        partitions = partition_segment((0.0, 0.0, 0.0), (5.0, 0.0, 0.0), max_len, OVERLAP)
        assert all(seg_length(p) <= max_len + TOL for p in partitions)
        assert_covers(partitions, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0), OVERLAP)


# ---------------------------------------------------------------------------
# partition_centre_and_tangent
# ---------------------------------------------------------------------------

def test_centre_and_tangent_axis_aligned():
    centre, tangent = partition_centre_and_tangent((1.0, 2.0, 0.7), (3.0, 2.0, 0.7))
    assert centre == pytest.approx((2.0, 2.0, 0.7))
    assert tangent == pytest.approx((1.0, 0.0, 0.0))


def test_tangent_is_unit_length_and_horizontal():
    _, tangent = partition_centre_and_tangent((0.0, 0.0, 1.0), (3.0, 4.0, 1.0))
    assert math.hypot(tangent[0], tangent[1]) == pytest.approx(1.0)
    assert tangent[2] == pytest.approx(0.0)


def test_tangent_flips_with_endpoint_order():
    _, forward = partition_centre_and_tangent((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    _, reverse = partition_centre_and_tangent((2.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert forward == pytest.approx(tuple(-v for v in reverse))


def test_centre_and_tangent_rejects_degenerate_partition():
    with pytest.raises(ValueError):
        partition_centre_and_tangent((1.0, 1.0, 0.0), (1.0, 1.0, 0.0))


# ---------------------------------------------------------------------------
# next_backoff_length -- the incremental retry ladder (ARM_SWEEP_PLAN §7.B)
# ---------------------------------------------------------------------------

def test_the_ladder_steps_down_by_the_backoff():
    assert next_backoff_length(0.80, 0.20, 0.20) == pytest.approx(0.60)
    assert next_backoff_length(0.60, 0.20, 0.20) == pytest.approx(0.40)


def test_the_ladder_stops_at_the_floor_rather_than_looping():
    """Below partition_min_length_m the region is reported unreachable. Without
    this the retry would shorten forever, parking the base for sweeps of a few
    centimetres."""
    assert next_backoff_length(0.20, 0.20, 0.20) is None


def test_the_floor_itself_is_admissible():
    """0.40 -> 0.20 must be allowed; it is the last usable rung."""
    assert next_backoff_length(0.40, 0.20, 0.20) == pytest.approx(0.20)


def test_the_full_ladder_matches_backoff_lengths():
    """The incremental helper and the up-front list must agree, or a retry loop
    and a planned-ahead one would visit different lengths."""
    incremental, current = [0.80], 0.80
    while (current := next_backoff_length(current, 0.20, 0.20)) is not None:
        incremental.append(current)
    assert incremental == pytest.approx(backoff_lengths(0.80, 0.20, 0.20))


def test_a_zero_backoff_is_rejected():
    with pytest.raises(ValueError):
        next_backoff_length(0.8, 0.0, 0.2)


# ---------------------------------------------------------------------------
# Re-cutting a segment preserves coverage -- the point of the whole mechanism
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("max_len", [0.80, 0.60, 0.40, 0.20])
def test_recutting_shorter_still_covers_the_whole_segment(max_len):
    """The reason §7.B re-cuts the SEGMENT instead of halving the partition: at
    every rung of the ladder the stretch of wall stays fully covered, with the
    configured overlap intact."""
    p_start, p_end = (0.0, 0.0, 1.5), (2.6, 0.0, 1.5)
    partitions = partition_segment(p_start, p_end, max_len, OVERLAP)
    assert_covers(partitions, p_start, p_end, OVERLAP)
    assert all(seg_length(p) <= max_len + TOL for p in partitions)


def test_shorter_rungs_never_lose_coverage_relative_to_longer_ones():
    p_start, p_end = (0.0, 0.0, 1.5), (2.6, 0.0, 1.5)
    for max_len in (0.80, 0.60, 0.40, 0.20):
        partitions = partition_segment(p_start, p_end, max_len, OVERLAP)
        assert partitions[0][0] == pytest.approx(p_start)
        assert partitions[-1][1] == pytest.approx(p_end)


def test_recutting_yields_more_and_shorter_partitions_each_rung():
    p_start, p_end = (0.0, 0.0, 1.5), (2.6, 0.0, 1.5)
    counts = [len(partition_segment(p_start, p_end, m, OVERLAP))
              for m in (0.80, 0.60, 0.40, 0.20)]
    assert counts == sorted(counts)
    assert counts[0] < counts[-1]
