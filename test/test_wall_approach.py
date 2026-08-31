"""Unit tests for the wall-approach policy (heading + matching arm pose).

The two choices must never disagree: parking square to the wall while unfolding
the left-flank pose aims the sensor plate ALONG the wall instead of at it, and
the reverse aims it into open space. Every state picking a heading or an unfold
pose goes through these helpers, so this is where that invariant is pinned.

Run with:

    python3 -m pytest test/test_wall_approach.py -v
"""

import pytest

from task_planner_fsm.utils.wall_approach import (
    DEFAULT_ALONG_WALL_POSE,
    DEFAULT_FACING_WALL_POSE,
    should_face_wall,
    unfolded_pose_name,
)


# ---------------------------------------------------------------------------
# should_face_wall
# ---------------------------------------------------------------------------

def test_defaults_to_the_wall_facing_approach():
    """`sweep_use_arm` ships enabled, so the default heading must follow it.

    Pinned because the two are easy to drift apart: a default here that says
    "along the wall" while ScanWall partitions would park the base on the
    along-wall heading at a partition centre -- a pose neither sweep wants."""
    assert should_face_wall({}) is True


def test_follows_sweep_use_arm_when_nav_face_wall_is_unset():
    """The arm sweep requires the square-on approach, so enabling it must bring
    the heading along without a second knob to remember."""
    assert should_face_wall({"sweep_use_arm": True}) is True
    assert should_face_wall({"sweep_use_arm": False}) is False


def test_nav_face_wall_enables_facing_without_the_arm_sweep():
    """The whole point of the separate knob: exercise the wall-facing approach
    without switching ScanWall into its half-migrated partition sweep."""
    ctx = {"nav_face_wall": True}
    assert should_face_wall(ctx) is True
    assert ctx.get("sweep_use_arm") is None


def test_nav_face_wall_overrides_sweep_use_arm_in_both_directions():
    assert should_face_wall({"nav_face_wall": False, "sweep_use_arm": True}) is False
    assert should_face_wall({"nav_face_wall": True, "sweep_use_arm": False}) is True


def test_explicit_false_is_honoured_not_treated_as_unset():
    """Regression guard: `or`-style defaulting would let sweep_use_arm win here."""
    assert should_face_wall({"nav_face_wall": False}) is False


@pytest.mark.parametrize("value", [1, 0, "", "yes"])
def test_truthy_values_are_coerced(value):
    assert should_face_wall({"nav_face_wall": value}) is bool(value)


# ---------------------------------------------------------------------------
# unfolded_pose_name
# ---------------------------------------------------------------------------

def test_pose_matches_the_heading():
    assert unfolded_pose_name({}) == DEFAULT_FACING_WALL_POSE
    assert unfolded_pose_name({"nav_face_wall": True}) == DEFAULT_FACING_WALL_POSE
    assert unfolded_pose_name({"sweep_use_arm": True}) == DEFAULT_FACING_WALL_POSE
    assert unfolded_pose_name({"sweep_use_arm": False}) == DEFAULT_ALONG_WALL_POSE


@pytest.mark.parametrize("ctx", [
    {},
    {"sweep_use_arm": True},
    {"nav_face_wall": True},
    {"nav_face_wall": False, "sweep_use_arm": True},
])
def test_pose_and_heading_never_disagree(ctx):
    """The invariant this module exists for."""
    expected = DEFAULT_FACING_WALL_POSE if should_face_wall(ctx) else DEFAULT_ALONG_WALL_POSE
    assert unfolded_pose_name(ctx) == expected


def test_pose_override_wins_over_the_heading():
    ctx = {"nav_face_wall": True, "scan_wall_unfolded_pose": "some_bench_pose"}
    assert unfolded_pose_name(ctx) == "some_bench_pose"


def test_empty_pose_override_is_ignored():
    ctx = {"scan_wall_unfolded_pose": "", "nav_face_wall": False}
    assert unfolded_pose_name(ctx) == DEFAULT_ALONG_WALL_POSE


# ---------------------------------------------------------------------------
# The states must all go through these helpers, not hardcode a pose
# ---------------------------------------------------------------------------

def test_arm_unfolding_does_not_hardcode_a_pose():
    """Regression: ArmUnfolding used to request 'unfolded_fsm' unconditionally,
    so a wall-facing base still unfolded the left-flank pose."""
    src = open("task_planner_fsm/states/arm_unfolding.py").read()
    assert "unfolded_pose_name" in src
    assert "'unfolded_fsm'" not in src
    assert '"unfolded_fsm"' not in src


def test_scan_wall_uses_the_shared_helper():
    src = open("task_planner_fsm/states/scan_wall.py").read()
    assert "from ..utils.wall_approach import unfolded_pose_name" in src


def test_navigate_uses_the_shared_heading_decision():
    src = open("task_planner_fsm/states/navigate.py").read()
    assert "should_face_wall(ctx)" in src
