"""The column must be retracted whenever the base moves.

A raised column carries the arm on top of it: driving the base like that is a
tall, top-heavy load regardless of terrain. This was violated once nesting
landed — the base transited to the next partition with the column still at the
LAST height of the previous one, which for a tall wall means the full extension.

The phase machine needs a ROS graph to run, so what is pinned here is its
STRUCTURE, read out of the source: every path that commands base motion must
pass through the column retract first, and nothing may reach the base move
without it.

Run with:

    python3 -m pytest test/test_column_safety.py -v
"""

import os
import re

import pytest

SOURCE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "task_planner_fsm", "states", "scan_wall.py",
)


@pytest.fixture(scope="module")
def source():
    return open(SOURCE).read()


@pytest.fixture(scope="module")
def phases(source):
    """{phase: body} for every branch of the _seg_phase machine."""
    bodies, names = {}, re.findall(r'if self\._seg_phase == "(\w+)":', source)
    for name in names:
        start = source.index(f'if self._seg_phase == "{name}":')
        rest = source[start + 1:]
        nxt = re.search(r'\n        if self\._seg_phase == "\w+":', rest)
        bodies[name] = rest[:nxt.start()] if nxt else rest[:4000]
    return bodies


def targets_of(body):
    return set(re.findall(r'self\._seg_phase = "(\w+)"', body))


# ---------------------------------------------------------------------------
# The invariant
# ---------------------------------------------------------------------------

def test_transit_motion_happens_only_in_transit_send(phases):
    """Both ways a TRANSIT can be commanded -- the Nav2 goal and the /cmd_vel
    crawl -- live in one phase, so there is a single place to guard.

    `press_settle` also moves the base, but that is the legacy base-driven SWEEP
    (`sweep_use_arm=false`), where the base drives along the wall while scanning
    and the column is necessarily up. That mode's hazard is a different one and
    is not what this guard is about."""
    movers = {name for name, body in phases.items()
              if "_send_base_goal(ctx, goal_xy" in body or "_start_sweep_crawl(" in body}
    assert movers == {"transit_send", "press_settle"}
    assert "goal_xy, goal_yaw = self._pending_transit" in phases["transit_send"]


def test_the_column_is_retracted_in_the_phase_right_before_the_base_moves(phases):
    """transit_column -> transit_send, and nothing else reaches transit_send."""
    assert "transit_send" in targets_of(phases["transit_column"])
    reaching = {name for name, body in phases.items()
                if "transit_send" in targets_of(body)}
    assert reaching == {"transit_column"}


def test_transit_column_commands_the_minimum_height(phases):
    assert "self.column.column_min_height_m" in phases["transit_column"]


def test_a_column_that_will_not_retract_fails_rather_than_transiting(phases):
    """The failure mode this phase exists to prevent is moving the base with the
    column up, so a timeout must NOT fall through to the transit."""
    body = phases["transit_column"]
    timeout = body[body.index("timed_out"):]
    # The failure has to come BEFORE the phase's own advance, so the timeout
    # branch returns instead of falling through to the base move.
    assert "self.fail(ctx" in timeout
    assert timeout.index("self.fail(ctx") < timeout.index('self._seg_phase = "transit_send"')


def test_the_column_is_raised_again_after_the_base_arrives(phases):
    """Otherwise the sweep would run at the retracted height."""
    assert "line_column" in targets_of(phases["transit_wait"])
    assert "self.column.command(node, ctx, target_h)" in phases["line_column"]


def test_a_skipped_transit_costs_no_column_travel(phases):
    """When the base is already at the scan pose the column was never lowered, so
    the transit phase must reach line_column WITHOUT going through the retract."""
    transit = phases["transit"]
    skip = transit[:transit.index("Transit to")]
    assert "line_column" in targets_of(skip)
    assert "transit_column" not in targets_of(skip)


def test_the_column_retract_is_not_gated_on_nesting(phases):
    """The base moves between partitions in both modes; the hazard is identical."""
    body = phases["transit_column"]
    assert "_nest_lines" not in body
    # ...and the raise afterwards is reached unconditionally from transit_wait.
    arrival = phases["transit_wait"]
    assert "_nest_lines" not in arrival


def test_the_arm_is_folded_before_the_column_retract(phases):
    """Order matters: fold the arm in, then drop the column. Retracting with the
    arm still extended sideways swings it through a much wider arc.

    The fold is reached via `transit`, which then defers to `transit_column`, so
    the check is on the path rather than on a direct edge -- and `transit` is the
    only way in to the retract."""
    assert "transit" in targets_of(phases["transit_recenter_wait"])
    assert "transit" in targets_of(phases["transit_recenter"])
    reaching_retract = {name for name, body in phases.items()
                        if "transit_column" in targets_of(body)}
    assert reaching_retract == {"transit"}
