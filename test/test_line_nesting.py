"""Tests for nesting line heights inside partitions (ARM_SWEEP_PLAN §8, Step 2).

Without nesting the base re-traverses every partition for every scan-line height.
§6's worked example -- 6 m wall, 3 reachable segments, 4 heights, 0.8 m
partitions -- is 32 base stops against 8 with nesting. The column is faster and
far more repeatable than repositioning the base, so trading base transits for
column moves is the whole point.

The phase machine itself needs a ROS graph, so what is tested here is the
decision logic and the geometry the nesting relies on: the height-independence
that makes one partitioning valid for every line (§3.6), the per-height
serpentine, and the arithmetic §6 claims.

Run with:

    python3 -m pytest test/test_line_nesting.py -v
"""

import math

import pytest

from task_planner_fsm.utils.wall_partitioning import partition_segment


# ---------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------

def nests(ctx):
    """Mirror of ScanWall._nest_lines."""
    return bool(ctx.get("nest_lines_in_partition", False)) and bool(
        ctx.get("sweep_use_arm", True)
    )


def test_nesting_is_off_by_default():
    """Step 2 restructures the state's control flow; §4.3 is explicit that it
    must not ride along with Step 1."""
    assert nests({}) is False


def test_nesting_requires_the_arm_sweep():
    """In the base-driven path the base IS the sweep, so there is nothing to
    nest -- the base has to move down the wall regardless."""
    assert nests({"nest_lines_in_partition": True, "sweep_use_arm": False}) is False
    assert nests({"nest_lines_in_partition": True, "sweep_use_arm": True}) is True


# ---------------------------------------------------------------------------
# Height independence -- what makes one partitioning valid for every line (§3.6)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("z", [0.8, 1.5, 2.2, 3.0])
def test_partition_boundaries_are_identical_at_every_height(z):
    """The guarantee nesting rests on: if the split differed per height, the
    partitions swept at one base stop would not line up and the GPR lines would
    not stitch."""
    at_z = partition_segment((0.0, 0.0, z), (2.6, 0.0, z), 0.8, 0.05)
    at_ref = partition_segment((0.0, 0.0, 1.5), (2.6, 0.0, 1.5), 0.8, 0.05)
    assert len(at_z) == len(at_ref)
    for (a, b), (ra, rb) in zip(at_z, at_ref):
        assert (a[0], a[1]) == pytest.approx((ra[0], ra[1]))
        assert (b[0], b[1]) == pytest.approx((rb[0], rb[1]))
        assert a[2] == pytest.approx(z) and b[2] == pytest.approx(z)


# ---------------------------------------------------------------------------
# Per-height serpentine (§8.2)
# ---------------------------------------------------------------------------

def sweep_ends(partition, line_index, serpentine=False):
    """Mirror of the direction choice in ScanWall._run_scan."""
    start, end = partition
    return (end, start) if serpentine and line_index % 2 == 1 else (start, end)


@pytest.mark.parametrize("line_index", range(5))
def test_every_height_sweeps_the_same_way_by_default(line_index):
    """One direction per partition, so consecutive GPR lines are acquired
    identically and nothing downstream has to know which way a line was
    recorded."""
    partition = ((0.0, 0.0, 1.5), (0.8, 0.0, 1.5))
    assert sweep_ends(partition, line_index) == (partition[0], partition[1])


def test_the_cost_of_a_constant_direction_is_a_return_traverse():
    """Each height ends at the far end and the next starts back at the near end,
    so the executor's traverse leg covers a full partition between heights. It
    has to fit inside sweep_max_traverse_m."""
    partition = ((0.0, 0.0, 1.5), (0.8, 0.0, 1.5))
    _, end = sweep_ends(partition, 0)
    next_start, _ = sweep_ends(partition, 1)
    ret = math.dist(end[:2], next_start[:2])
    assert ret == pytest.approx(0.8)
    assert ret < 1.10          # sweep_max_traverse_m


def test_serpentine_is_available_but_opt_in():
    """§8.2's alternating scheme: the arm finishes each height where the next
    begins, removing the return traverse. Off until a single-direction run has
    been checked -- a reversed line only shows up later, in stitching."""
    partition = ((0.0, 0.0, 1.5), (0.8, 0.0, 1.5))
    assert sweep_ends(partition, 1, serpentine=True) == (partition[1], partition[0])
    for line in range(5):
        _, end = sweep_ends(partition, line, serpentine=True)
        next_start, _ = sweep_ends(partition, line + 1, serpentine=True)
        assert end == next_start


def test_direction_is_per_height_never_per_partition():
    """Partitions advance monotonically along the wall in both modes; only the
    ARM ever reverses."""
    a = ((0.0, 0.0, 1.5), (0.8, 0.0, 1.5))
    b = ((0.75, 0.0, 1.5), (1.55, 0.0, 1.5))
    for serp in (False, True):
        assert sweep_ends(a, 0, serp)[0] == a[0]
        assert sweep_ends(b, 0, serp)[0] == b[0]


# ---------------------------------------------------------------------------
# The payoff §6 claims
# ---------------------------------------------------------------------------

def base_stops(partitions, heights, nested):
    return partitions if nested else partitions * heights


def column_travels(partitions, heights, nested):
    """Column moves that actually TRAVEL, i.e. height changes.

    Not simply `partitions * heights`: the column controller skips a command when
    it is already at the target ("Column already close to target ... skipping"),
    so commanding the same height again costs nothing. Nested, the column steps
    through the heights at each partition and then returns to the first for the
    next one.
    """
    if heights <= 1:
        return 0
    if nested:
        return partitions * (heights - 1) + (partitions - 1)
    return heights - 1


def test_the_worked_example_from_the_plan():
    """6 m wall, 3 reachable segments, 4 heights, ~8 partitions. §6's table."""
    assert base_stops(8, 4, nested=False) == 32
    assert base_stops(8, 4, nested=True) == 8
    assert column_travels(8, 4, nested=False) == 3
    assert column_travels(8, 4, nested=True) == 31


def test_nesting_trades_base_stops_for_column_moves():
    """The trade is only worth making because the column is the cheaper motion --
    it is what the whole of §6 argues."""
    for partitions, heights in ((8, 4), (19, 3), (5, 6)):
        assert base_stops(partitions, heights, True) < base_stops(partitions, heights, False)
        assert column_travels(partitions, heights, True) > column_travels(partitions, heights, False)


def test_nesting_is_a_no_op_for_a_single_height_wall():
    """Most of the risk of Step 2 is in walls with several heights; a one-line
    wall must behave exactly as before."""
    assert base_stops(19, 1, nested=True) == base_stops(19, 1, nested=False)
    assert column_travels(19, 1, nested=True) == column_travels(19, 1, nested=False) == 0
