"""Self-collision barrier rows: the geometry, the Jacobians, and that they bite."""

import json

import numpy as np
import pytest

from task_planner_fsm.wbc.qp import SoftRows, Task, solve_velocity_qp
from task_planner_fsm.wbc.self_collision import (
    BODY, ROOT, SelfCollisionConfig, SelfCollisionModel, capsule_spheres,
    sphere_box, sphere_cylinder)

# A planar-ish 3-DOF arm: yaw at the root, then two pitch joints.
URDF = """<?xml version="1.0"?>
<robot name="arm">
  <link name="root"/><link name="a"/><link name="b"/><link name="c"/><link name="tip"/>
  <joint name="j1" type="revolute"><parent link="root"/><child link="a"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="2"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" velocity="2"/></joint>
  <joint name="j3" type="revolute"><parent link="b"/><child link="c"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" velocity="2"/></joint>
  <joint name="tool" type="fixed"><parent link="c"/><child link="tip"/>
    <origin xyz="0.4 0 0"/></joint>
</robot>
"""
JOINTS = ["j1", "j2", "j3"]


def _model(spheres, pairs=(), obstacles=(), **config):
    return SelfCollisionModel(URDF, "root", JOINTS, spheres=list(spheres), pairs=list(pairs),
                              obstacles=list(obstacles),
                              config=SelfCollisionConfig(**{"influence": 10.0, **config}))


# ----------------------------------------------------------------------
# Distances
# ----------------------------------------------------------------------
def test_sphere_box_outside_face_and_corner():
    d, n = sphere_box([0.5, 0.0, 0.0], 0.1, [0.0, 0.0, 0.0], [0.2, 0.2, 0.2])
    assert d == pytest.approx(0.2)
    assert n == pytest.approx([1.0, 0.0, 0.0])
    d, n = sphere_box([0.3, 0.3, 0.0], 0.0, [0.0, 0.0, 0.0], [0.2, 0.2, 0.2])
    assert d == pytest.approx(np.hypot(0.1, 0.1))
    assert n == pytest.approx([np.sqrt(0.5), np.sqrt(0.5), 0.0])


def test_sphere_box_inside_leaves_by_the_nearest_face():
    d, n = sphere_box([0.0, 0.0, 0.15], 0.02, [0.0, 0.0, 0.0], [0.2, 0.2, 0.2])
    assert d == pytest.approx(-0.05 - 0.02)
    assert n == pytest.approx([0.0, 0.0, 1.0])


def test_sphere_cylinder_side_cap_rim_and_inside():
    args = ([0.0, 0.0], 0.1, -1.0, 0.0)
    d, n = sphere_cylinder([0.3, 0.0, -0.5], 0.05, *args)          # beside it
    assert d == pytest.approx(0.15)
    assert n == pytest.approx([1.0, 0.0, 0.0])
    d, n = sphere_cylinder([0.0, 0.05, 0.3], 0.05, *args)          # above the cap
    assert d == pytest.approx(0.25)
    assert n == pytest.approx([0.0, 0.0, 1.0])
    d, n = sphere_cylinder([0.4, 0.0, 0.4], 0.0, *args)            # off the rim
    assert d == pytest.approx(0.5)
    assert n == pytest.approx([0.6, 0.0, 0.8])
    d, n = sphere_cylinder([0.08, 0.0, -0.5], 0.0, *args)          # inside, near the side
    assert d == pytest.approx(-0.02)
    assert n == pytest.approx([1.0, 0.0, 0.0])
    d, n = sphere_cylinder([0.0, 0.0, -0.01], 0.0, *args)          # inside, near the cap
    assert d == pytest.approx(-0.01)
    assert n == pytest.approx([0.0, 0.0, 1.0])


def test_capsules_are_covered_end_to_end():
    spheres = capsule_spheres("l", (0, 0, 0), (1, 0, 0), 0.1)
    xs = [c[0] for _, c, _ in spheres]
    assert xs[0] == 0.0 and xs[-1] == pytest.approx(1.0)
    assert max(np.diff(xs)) <= 0.1 + 1e-12


# ----------------------------------------------------------------------
# Rows
# ----------------------------------------------------------------------
def _distance_of(model, q, T_body=None):
    """The smallest signed clearance the model sees, straight from rows()."""
    return model.rows(q, T_body)[2]


@pytest.mark.parametrize("obstacle", [
    {"type": "cylinder", "frame": ROOT, "center": [0.0, 0.0], "radius": 0.1,
     "z_min": -1.0, "z_max": 0.0},
    {"type": "box", "frame": ROOT, "center": [0.6, 0.3, 0.0],
     "half_extents": [0.1, 0.1, 0.1]},
])
def test_an_obstacle_row_is_the_derivative_of_its_distance(obstacle):
    model = _model([("tip", (0.0, 0.0, 0.0), 0.05)], obstacles=[obstacle])
    q = np.array([0.4, 0.9, 0.7])
    A, _, d0, _ = model.rows(q)
    rng = np.random.default_rng(1)
    for _ in range(5):
        dq = rng.normal(size=3) * 1e-6
        assert _distance_of(model, q + dq) - d0 == pytest.approx(A[0] @ dq, abs=1e-11)


def test_a_pair_row_is_the_derivative_of_its_distance():
    model = _model([("tip", (0.0, 0.0, 0.0), 0.05), ("b", (0.1, 0.0, 0.0), 0.05)],
                   pairs=[(["tip"], ["b"])])
    q = np.array([0.2, 0.3, 2.4])     # folded back toward the first link
    A, _, d0, label = model.rows(q)
    assert label == "tip~b"
    dq = np.array([1e-6, -2e-6, 1.5e-6])
    assert _distance_of(model, q + dq) - d0 == pytest.approx(A[0] @ dq, abs=1e-11)


def test_the_barrier_permits_approach_far_away_and_demands_retreat_inside():
    model = _model([("tip", (0.0, 0.0, 0.0), 0.05)], obstacles=[
        {"type": "box", "frame": ROOT, "center": [0.0, 0.0, -0.5],
         "half_extents": [2.0, 2.0, 0.5]}], safety_margin=0.05, alpha=2.0)
    far = model.rows(np.array([0.0, -0.3, 0.0]))       # tip well above the floor
    assert far[1][0] < 0.0                            # may still close on it
    inside = model.rows(np.array([0.0, 0.6, 0.0]))    # tip pitched through it
    assert inside[2] < 0.0
    assert inside[1][0] > 0.0                         # must open the gap


def test_rows_outside_the_influence_are_dropped_and_the_tightest_are_kept():
    spheres = [("tip", (0.0, 0.0, 0.0), 0.05), ("c", (0.0, 0.0, 0.0), 0.05)]
    obstacle = {"type": "cylinder", "frame": ROOT, "center": [0.0, 0.0], "radius": 0.1,
                "z_min": -1.0, "z_max": 0.0}
    q = np.array([0.0, 0.0, 0.0])
    model = _model(spheres, obstacles=[obstacle], influence=0.05)
    A, lower, closest, _ = model.rows(q)
    assert A.shape == (0, 3) and np.isfinite(closest)
    model = _model(spheres, obstacles=[obstacle], max_rows=1)
    A, _, _, label = model.rows(q)
    assert A.shape == (1, 3) and label == "c~cylinder"


def test_body_obstacles_follow_the_pose_they_are_given_and_wait_for_one():
    body_box = {"name": "body", "type": "box", "frame": BODY, "center": [0.0, 0.0, 0.0],
                "half_extents": [0.05, 0.05, 0.05]}
    model = _model([("tip", (0.0, 0.0, 0.0), 0.0)], obstacles=[body_box])
    q = np.array([0.0, 0.0, 0.0])                     # tip at (0.9, 0, 0.2)
    assert model.rows(q)[2] == float("inf")          # no pose, no body rows
    T = np.eye(4)
    T[:3, 3] = [0.9, 0.0, 0.4]
    assert model.rows(q, T)[2] == pytest.approx(0.15)
    T[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    assert model.rows(q, T)[2] == pytest.approx(0.15)


def test_ignored_links_and_missing_links():
    obstacle = {"type": "cylinder", "frame": ROOT, "center": [0.0, 0.0], "radius": 0.1,
                "z_min": -1.0, "z_max": 0.0, "ignore": ["tip"]}
    model = _model([("tip", (0, 0, 0), 0.05), ("nowhere", (0, 0, 0), 0.05)],
                   obstacles=[obstacle])
    assert model.missing_links == ["nowhere"]
    assert model.rows(np.zeros(3))[0].shape == (0, 3)


def test_a_link_moved_by_joints_outside_the_qp_is_refused():
    with pytest.raises(ValueError):
        SelfCollisionModel(URDF, "root", ["j1", "j2"], spheres=[("tip", (0, 0, 0), 0.05)],
                           pairs=[], obstacles=[])


def test_the_json_override_replaces_only_what_it_names():
    text = json.dumps({"capsules": [["c", [0, 0, 0], [0.4, 0, 0], 0.05]],
                       "obstacles": [{"type": "cylinder", "frame": "root", "center": [0, 0],
                                      "radius": 0.1, "z_min": -1, "z_max": 0}]})
    model = SelfCollisionModel.from_json(text, URDF, "root", JOINTS)
    assert {link for link, _, _ in model.spheres} == {"c"}
    assert len(model.obstacles) == 1
    assert model.pairs == []     # the default pairs name UR links this arm lacks


# ----------------------------------------------------------------------
# In a QP
# ----------------------------------------------------------------------
def test_the_qp_stops_the_tip_at_the_margin_instead_of_driving_through():
    """A joint-space task that swings the tip straight down through a box: the
    rows let it approach, slow it, and stop it at the margin."""
    floor = {"type": "box", "frame": ROOT, "center": [0.0, 0.0, -0.45],
             "half_extents": [2.0, 2.0, 0.5]}           # top face at z = 0.05
    model = _model([("tip", (0.0, 0.0, 0.0), 0.05)], obstacles=[floor],
                   safety_margin=0.03, alpha=4.0, influence=0.3)
    q = np.array([0.0, -0.2, 0.0])
    dt = 0.02
    worst = float("inf")
    for _ in range(300):
        A, lower, closest, _ = model.rows(q)
        worst = min(worst, closest)
        soft = [SoftRows(A, lower, 1e3)] if len(lower) else None
        solution = solve_velocity_qp([Task(np.eye(3), np.array([0.0, 0.5, 0.0]), 1.0)],
                                     np.full(3, -1.0), np.full(3, 1.0), soft=soft)
        q = q + solution.u * dt
    assert worst > 0.0, f"tip went {-worst * 100:.1f} cm into the box"
    assert model.rows(q)[2] == pytest.approx(0.03, abs=0.01)   # parked at the margin
