"""Tests for re-cutting a failed partition's segment (ARM_SWEEP_PLAN §7.B).

When the executor rejects a sweep because the arm could not reach along it, the
partition is NOT skipped and NOT halved: the whole reachable segment it came from
is re-cut at a shorter maximum length. Skipping — which is what happened before
this existed — silently drops that stretch of wall from the scan.

The splice is the fiddly part: the replacements have to land exactly where the
old partitions were, so the list stays in sweep order and the other segments are
untouched. That logic is exercised here directly, because driving it through
ScanWall needs a ROS graph.

Run with:

    python3 -m pytest test/test_partition_backoff_splice.py -v
"""

import pytest


def splice(segments, poses, sources, source, replacements, replacement_poses):
    """The splice from ScanWall._shorten_failed_partition, in isolation."""
    first = sources.index(source)
    last = len(sources) - 1 - sources[::-1].index(source)
    return (
        segments[:first] + list(replacements) + segments[last + 1:],
        poses[:first] + list(replacement_poses) + poses[last + 1:],
        sources[:first] + [source] * len(replacements) + sources[last + 1:],
        first,
    )


@pytest.fixture
def plan():
    """Three reachable segments cut into 6 partitions: A A B B B C."""
    segments = [f"p{i}" for i in range(6)]
    poses = [f"pose{i}" for i in range(6)]
    sources = [0, 0, 1, 1, 1, 2]
    return segments, poses, sources


def test_the_replacements_land_where_the_old_ones_were(plan):
    segments, poses, sources = plan
    new, _, new_sources, first = splice(
        segments, poses, sources, 1, ["b0", "b1", "b2", "b3"], ["q0", "q1", "q2", "q3"])
    assert new == ["p0", "p1", "b0", "b1", "b2", "b3", "p5"]
    assert new_sources == [0, 0, 1, 1, 1, 1, 2]
    assert first == 2


def test_neighbouring_segments_are_untouched(plan):
    """Only the failing segment is re-cut. Re-cutting the whole line would throw
    away partitions that already swept fine."""
    segments, poses, sources = plan
    new, new_poses, _, _ = splice(segments, poses, sources, 1, ["b0"], ["q0"])
    assert new[:2] == ["p0", "p1"]          # segment 0 intact
    assert new[-1:] == ["p5"]               # segment 2 intact
    assert new_poses[:2] == ["pose0", "pose1"]
    assert new_poses[-1:] == ["pose5"]


def test_poses_stay_aligned_with_partitions(plan):
    """They are indexed together everywhere downstream; a length mismatch would
    park the base using another partition's scan pose."""
    segments, poses, sources = plan
    new, new_poses, new_sources, _ = splice(
        segments, poses, sources, 1, ["b0", "b1", "b2"], ["q0", "q1", "q2"])
    assert len(new) == len(new_poses) == len(new_sources)


def test_the_retry_resumes_at_the_first_replacement(plan):
    """_seg_idx rewinds to `first`, so the re-cut stretch is swept from its start
    rather than from wherever the failure happened."""
    segments, poses, sources = plan
    _, _, _, first = splice(segments, poses, sources, 1, ["b0", "b1"], ["q0", "q1"])
    assert first == sources.index(1)


def test_recutting_the_first_segment_works(plan):
    segments, poses, sources = plan
    new, _, new_sources, first = splice(
        segments, poses, sources, 0, ["a0", "a1", "a2"], ["q0", "q1", "q2"])
    assert first == 0
    assert new == ["a0", "a1", "a2", "p2", "p3", "p4", "p5"]
    assert new_sources == [0, 0, 0, 1, 1, 1, 2]


def test_recutting_the_last_segment_works(plan):
    segments, poses, sources = plan
    new, _, new_sources, first = splice(
        segments, poses, sources, 2, ["c0", "c1"], ["q0", "q1"])
    assert first == 5
    assert new == ["p0", "p1", "p2", "p3", "p4", "c0", "c1"]
    assert new_sources == [0, 0, 1, 1, 1, 2, 2]


def test_a_single_partition_segment_is_replaced_in_place(plan):
    segments, poses, sources = plan
    new, _, _, _ = splice(segments, poses, sources, 2, ["c0"], ["q0"])
    assert new == ["p0", "p1", "p2", "p3", "p4", "c0"]


def test_sources_remain_non_decreasing_so_sweep_order_holds():
    """Partitions advance monotonically along the wall; a splice that broke that
    would make the base jump back and forth between segments."""
    segments = [f"p{i}" for i in range(6)]
    poses = [f"pose{i}" for i in range(6)]
    sources = [0, 0, 1, 1, 1, 2]
    _, _, new_sources, _ = splice(
        segments, poses, sources, 1, ["b0", "b1", "b2", "b3"], ["q"] * 4)
    assert new_sources == sorted(new_sources)
