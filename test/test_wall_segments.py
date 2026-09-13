"""Scan-line segmentation must skip what the plate cannot actually reach.

The failure this pins down: a column bracketed to the wall sat inside the first
"reachable" segment, so the sweep was planned straight through it. Feasibility
was asking only whether the BASE could find somewhere to stand along the outward
normal — and since ``nearest_free_along`` starts its walk at the scan point and
returns the first free cell, a column merely pushed that standing spot further
out and the point still counted as reachable.
"""

import numpy as np
import pytest
from scipy import ndimage

pytest.importorskip("rclpy")

from task_planner_fsm.utils.costmap_utils import reachable_wall_segments  # noqa: E402

RES = 0.05
ORIGIN = (0.0, 0.0)
SHAPE = (240, 240)          # 12 m x 12 m

# Geometry: a wall face along y = 5.0 (its cells span y in [5.0, 5.1]); the robot
# scans from below, so the exterior normal points -y and the scan line sits 0.6 m
# out at y = 4.4.
WALL_Y = 5.0
SCAN_Y = 4.4
OUTWARD = (0.0, -1.0)       # away from the wall, into the room
INWARD = (0.0, 1.0)


class FakeCostmap:
    """Just the fields costmap_utils reads off a nav_msgs/OccupancyGrid."""

    class _Info:
        pass

    class _Origin:
        pass

    def __init__(self, data):
        self.info = self._Info()
        self.info.resolution = RES
        self.info.width = data.shape[1]
        self.info.height = data.shape[0]
        self.info.origin = self._Origin()
        self.info.origin.position = self._Origin()
        self.info.origin.position.x = ORIGIN[0]
        self.info.origin.position.y = ORIGIN[1]
        self.data = [int(v) for v in data.flatten()]


def costmap(extra_obstacles=(), robot_radius=0.6, inflation_radius=1.0):
    """A Nav2-SHAPED costmap: lethal cells, inscribed ring, inflation tail.

    Modelling only the lethal cells would hide the bug this fixture exists to
    catch. Nav2 publishes 100 for an observed obstacle, 99 for everything within
    ``robot_radius`` of one (INSCRIBED) and a decaying tail out to
    ``inflation_radius`` — and the last two are functions of the ROBOT, not of
    the geometry. With a 0.6 m radius the wall's inscribed ring reaches exactly
    as far as the scan line, so any check keyed on 99 rejects every wall.
    """
    lethal = np.zeros(SHAPE, dtype=bool)

    def fill(x0, y0, x1, y1):
        c0, c1 = int(x0 / RES), int(x1 / RES)
        r0, r1 = int(y0 / RES), int(y1 / RES)
        lethal[r0:r1, c0:c1] = True

    fill(0.0, WALL_Y, 12.0, WALL_Y + 0.1)               # the wall itself
    for x0, y0, x1, y1 in extra_obstacles:
        fill(x0, y0, x1, y1)

    distance = ndimage.distance_transform_edt(~lethal, sampling=RES)
    data = np.zeros(SHAPE, dtype=int)
    tail = (distance > robot_radius) & (distance <= inflation_radius)
    data[tail] = np.clip(
        98 * (1.0 - (distance[tail] - robot_radius) / (inflation_radius - robot_radius)),
        1, 98).astype(int)
    data[distance <= robot_radius] = 99
    data[lethal] = 100
    return FakeCostmap(data)


def ctx_for(cmap, **overrides):
    ctx = {
        "global_costmap": cmap,
        "current_wall_index": 0,
        "wall_inward_normals": [INWARD],
        "wall_segment_sample_step": 0.1,
        "wall_segment_min_len": 0.3,
    }
    ctx.update(overrides)
    return ctx


def spans(segments):
    return [(round(s[0], 2), round(e[0], 2)) for s, e in segments]


def covers(segments, x):
    return any(s[0] - 1e-9 <= x <= e[0] + 1e-9 for s, e in segments)


def test_a_clear_wall_yields_one_segment():
    segments = reachable_wall_segments(
        ctx_for(costmap()), (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert len(segments) == 1
    assert spans(segments)[0] == pytest.approx((1.0, 9.0), abs=0.15)


def test_a_column_on_the_wall_splits_the_segment():
    """A 0.5 m column protruding 0.7 m — far enough to reach the scan line."""
    column = (4.8, WALL_Y - 0.7, 5.3, WALL_Y)
    segments = reachable_wall_segments(
        ctx_for(costmap([column])), (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))

    assert len(segments) >= 2, f"column did not split the line: {spans(segments)}"
    assert not covers(segments, 5.05), "a segment still runs through the column"
    # ... and the wall either side is still swept.
    assert covers(segments, 2.0) and covers(segments, 8.0)


def test_the_split_is_backed_off_by_the_plate_half_width():
    """Ending the sweep at the last feasible sample puts the plate's EDGE into
    the column: feasibility asks where the plate may be CENTRED. Both the
    segment before the column and the one after must stand clear of it."""
    column = (4.8, WALL_Y - 0.7, 5.3, WALL_Y)
    clearance = 0.25
    segments = reachable_wall_segments(
        ctx_for(costmap([column]), wall_segment_obstacle_clearance=clearance),
        (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))

    before = [(s, e) for s, e in segments if e[0] <= 4.8]
    after = [(s, e) for s, e in segments if s[0] >= 5.3]
    assert before and after, f"expected a segment either side: {spans(segments)}"

    # The plate sweeps to the segment end, so that end must clear the column by
    # at least the half-width we asked for.
    assert 4.8 - before[-1][1][0] >= clearance - 0.05
    assert after[0][0][0] - 5.3 >= clearance - 0.05


def test_a_column_that_only_reaches_the_plate_still_splits_it():
    """Protruding 0.45 m: short of the scan line at 0.6 m, but well into the
    corridor the plate sweeps through."""
    column = (4.8, WALL_Y - 0.45, 5.3, WALL_Y)
    segments = reachable_wall_segments(
        ctx_for(costmap([column])), (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert not covers(segments, 5.05), f"plate path is blocked but was swept: {spans(segments)}"


def test_the_wall_itself_never_blocks_its_own_scan():
    """The probe must not reach the wall's own cells, or nothing is scannable."""
    segments = reachable_wall_segments(
        ctx_for(costmap()), (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert segments and spans(segments)[0][1] - spans(segments)[0][0] > 7.0


def test_the_wall_own_inscribed_ring_does_not_block_its_scan():
    """The regression for a bug that made EVERY wall unscannable.

    The probe has to key on LETHAL alone. 99 (inscribed) means "within
    robot_radius of something", which for a 0.6 m radius reaches exactly as far
    out as the scan line — so a probe that treats 99 as geometry finds every scan
    point on every wall blocked, and the FSM reports that no portion of the wall
    is reachable.
    """
    cmap = costmap(robot_radius=0.6, inflation_radius=1.0)

    # Sanity: the fixture really does fill the plate's probe corridor — between
    # the scan line and the wall — with the wall's own inscribed ring.
    from task_planner_fsm.utils.costmap_utils import _cost_at
    assert _cost_at(cmap, 5.0, SCAN_Y + 0.2) == 99      # 0.4 m off the face
    assert _cost_at(cmap, 5.0, SCAN_Y + 0.3) == 99      # 0.3 m off the face

    segments = reachable_wall_segments(
        ctx_for(cmap), (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert segments, "the wall's own inscribed ring must not block its scan"
    assert spans(segments)[0] == pytest.approx((1.0, 9.0), abs=0.15)


def test_inflation_around_the_wall_does_not_block_the_scan():
    """A wall casting a fat inflation gradient must still be scannable."""
    segments = reachable_wall_segments(
        ctx_for(costmap(inflation_radius=1.6)), (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert segments, "inflation must not be mistaken for geometry"


def test_the_probe_never_reaches_the_wall_face_even_if_told_to():
    """A probe deeper than the scan offset would read the wall's own cells and
    reject every point, so the depth is clamped to the offset minus a margin."""
    ctx = ctx_for(costmap(), wall_segment_probe_depth=5.0, wall_scan_offset=0.6)
    segments = reachable_wall_segments(ctx, (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert segments, "an over-deep probe must be clamped, not allowed to hit the wall"


def test_a_shallower_scan_offset_shrinks_the_probe():
    """With the scan line only 0.3 m off the face, the probe must not run 0.35 m."""
    ctx = ctx_for(costmap(), wall_scan_offset=0.3)
    segments = reachable_wall_segments(ctx, (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert segments


def test_the_plate_path_check_can_be_turned_off():
    column = (4.8, WALL_Y - 0.7, 5.3, WALL_Y)
    # Relax the base-offset limit too, so this isolates the plate-path flag: the
    # column's inscribed ring would otherwise make the point unscannable on
    # standing-room grounds alone, and the flag would look broken.
    ctx = ctx_for(costmap([column]), wall_segment_check_plate_path=False,
                  wall_segment_max_base_offset=1.3)
    segments = reachable_wall_segments(ctx, (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    # The old behaviour, kept reachable for debugging: the base can stand beyond
    # the column, so the line is swept straight through it.
    assert covers(segments, 5.05)


def test_a_free_standing_obstacle_off_the_wall_still_blocks_the_base():
    """Something parked in the room, between base and wall, keeps its old
    meaning: no standing room within reach."""
    blocker = (4.5, 2.6, 5.6, 4.9)
    segments = reachable_wall_segments(
        ctx_for(costmap([blocker])), (1.0, SCAN_Y, 1.5), (9.0, SCAN_Y, 1.5))
    assert not covers(segments, 5.05)
