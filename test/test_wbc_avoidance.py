"""The barrier has to stop the base without fighting the wall it is scanning.

Costmaps here are small hand-built grids, so each test states its geometry
outright: an obstacle strip, a wall to be swept, and where the base stands.
"""

import numpy as np
import pytest
from scipy import ndimage

from task_planner_fsm.wbc.avoidance import (
    AvoidanceConfig,
    ObstacleField,
    avoidance_rows,
    footprint_samples,
)
from task_planner_fsm.wbc.base_model import BaseLimits, box_bounds, constraint_rows
from task_planner_fsm.wbc.qp import SoftRows, Task, solve_velocity_qp

RES = 0.05
ORIGIN = (0.0, 0.0)
SHAPE = (120, 120)          # 6 m x 6 m


def grid(obstacles=()):
    """A free grid with rectangular obstacles given in world metres."""
    data = np.zeros(SHAPE, dtype=int)
    for x0, y0, x1, y1 in obstacles:
        c0, c1 = int(x0 / RES), int(x1 / RES)
        r0, r1 = int(y0 / RES), int(y1 / RES)
        data[r0:r1, c0:c1] = 100
    return data


def test_distance_and_gradient_point_away_from_the_obstacle():
    # A wall slab along x, spanning y in [4.0, 4.5].
    field = ObstacleField.from_grid(grid([(0.0, 4.0, 6.0, 4.5)]), RES, ORIGIN,
                                    AvoidanceConfig())
    distance, grad_x, grad_y = field.sample(3.0, 3.0)
    assert distance == pytest.approx(1.0, abs=0.1)      # 1 m below the slab
    assert grad_y < 0                                   # away from the wall is -y
    assert abs(grad_x) < 1e-6


def test_no_rows_when_everything_is_far_away():
    field = ObstacleField.from_grid(grid([(0.0, 5.5, 6.0, 6.0)]), RES, ORIGIN,
                                    AvoidanceConfig())
    A, lo, hi, closest = avoidance_rows(field, yaw=0.0, p_turret=(3.0, 1.0),
                                        config=AvoidanceConfig())
    assert A.shape[0] == 0 and closest > 1.0


def test_an_obstacle_inside_the_influence_radius_produces_rows():
    config = AvoidanceConfig()
    field = ObstacleField.from_grid(grid([(0.0, 4.0, 6.0, 4.5)]), RES, ORIGIN, config)
    # Base centre 0.8 m below the slab: the near footprint samples are ~0.35 m off.
    A, lo, hi, closest = avoidance_rows(field, 0.0, (3.0, 3.2), config)
    assert A.shape[0] > 0
    assert A.shape[1] == 3
    assert closest < config.influence
    assert np.all(np.isinf(hi))


def test_the_rows_bound_the_approach_speed_and_never_block_leaving():
    """A barrier is a speed limit, not a wall: closing is allowed, but only at
    ``alpha * (distance - margin)``, which goes to zero at the margin."""
    config = AvoidanceConfig(safety_margin=0.15, alpha=1.0)
    field = ObstacleField.from_grid(grid([(0.0, 4.0, 6.0, 4.5)]), RES, ORIGIN, config)
    # Nearest footprint sample sits ~0.35 m off the slab, so the allowance is
    # alpha * (0.35 - 0.15) = 0.2 m/s.
    A, lo, _, _ = avoidance_rows(field, 0.0, (3.0, 3.2), config)

    assert np.all(A @ np.array([0.0, 0.1, 0.0]) >= lo - 1e-9)   # slow approach: fine
    assert np.any(A @ np.array([0.0, 0.35, 0.0]) < lo - 1e-9)   # too fast: forbidden
    assert np.all(A @ np.array([0.0, -0.2, 0.0]) >= lo - 1e-9)  # leaving: always fine


def test_yawing_into_an_obstacle_is_constrained_too():
    """A point base would be blind to this: the turret axis never moves."""
    config = AvoidanceConfig(footprint_offset=(0.0, 0.0))
    field = ObstacleField.from_grid(grid([(0.0, 4.0, 6.0, 4.5)]), RES, ORIGIN, config)
    A, lo, _, _ = avoidance_rows(field, 0.0, (3.0, 3.2), config)
    # The wz column is non-zero somewhere: rotation does move footprint points.
    assert np.any(np.abs(A[:, 2]) > 1e-6)


def test_the_scanned_wall_is_masked_out_of_the_field():
    """The base sits ~1 m off the wall it is sweeping; the wall must not push it.

    Without the mask this is the failure mode that matters: the barrier drives
    the base away from the surface and the arm stretches to keep the standoff.
    """
    config = AvoidanceConfig()
    wall = (0.0, 4.0, 6.0, 4.5)
    segment = ((0.5, 4.0), (5.5, 4.0))          # the scan line along the wall face

    unmasked = ObstacleField.from_grid(grid([wall]), RES, ORIGIN, config)
    masked = ObstacleField.from_grid(grid([wall]), RES, ORIGIN, config,
                                     mask_segment=segment)

    A_unmasked, _, _, _ = avoidance_rows(unmasked, 0.0, (3.0, 3.2), config)
    A_masked, _, _, closest = avoidance_rows(masked, 0.0, (3.0, 3.2), config)

    assert A_unmasked.shape[0] > 0        # the wall would have pushed back
    assert A_masked.shape[0] == 0         # ... and does not, once masked
    assert not np.isfinite(closest) or closest > config.influence


def inflated_grid(obstacles=(), inflation_radius=1.0):
    """A Nav2-shaped costmap: LETHAL core, INSCRIBED ring, decaying tail.

    Nav2 publishes 100 for LETHAL, 99 for INSCRIBED ("a disc of robot_radius
    centred here touches something") and a decaying gradient out to
    ``inflation_radius``. Only the 100s are geometry.
    """
    lethal = grid(obstacles) >= 100
    distance = ndimage.distance_transform_edt(~lethal, sampling=RES)
    data = np.zeros(SHAPE, dtype=int)
    tail = (distance > 0) & (distance <= inflation_radius)
    data[tail] = np.clip(98 * (1.0 - distance[tail] / inflation_radius), 1, 98).astype(int)
    data[distance <= 0.6] = 99          # the inscribed ring (robot_radius 0.6)
    data[lethal] = 100
    return data


def test_inflation_is_not_geometry():
    """Counting the inflation halo as obstacle makes the robot permanently
    'in collision' with the wall it is scanning, and throttles the sweep."""
    config = AvoidanceConfig()
    data = inflated_grid([(0.0, 4.0, 6.0, 4.5)], inflation_radius=1.0)
    field = ObstacleField.from_grid(data, RES, ORIGIN, config)

    # 1 m below the slab: the true clearance is 1 m, and the halo reaches exactly
    # that far, so a threshold that counts inflation would report ~0.
    distance, _, grad_y = field.sample(3.0, 3.0)
    assert distance == pytest.approx(1.0, abs=0.1)
    assert grad_y < 0

    # Counting the halo measures the distance to the INFLATION boundary instead,
    # which is much nearer. (Worse on the real robot: Nav2's decay is
    # exponential with cost_scaling_factor 0.5, so cells stay above 50 nearly to
    # the full inflation radius, and the base reads as permanently in collision.)
    naive = ObstacleField.from_grid(data, RES, ORIGIN, AvoidanceConfig(obstacle_cost=50))
    assert naive.sample(3.0, 3.0)[0] < distance - 0.3


def test_the_wall_stays_masked_past_the_ends_of_a_short_segment():
    """The wall does not stop where the segment does.

    A short segment with a short mask leaves the same wall unmasked just beyond
    its ends, where the gradient points back ALONG the wall — so the barrier
    drives the base out of the masked window, unmasking more wall, and the sweep
    runs backwards. Seen in Gazebo before the corridor was lengthened.
    """
    config = AvoidanceConfig()
    wall = (0.0, 4.0, 6.0, 4.5)
    segment = ((3.0, 4.0), (3.6, 4.0))          # a 0.6 m segment on a 6 m wall
    field = ObstacleField.from_grid(grid([wall]), RES, ORIGIN, config,
                                    mask_segment=segment)

    # Anywhere the base can plausibly sit during that sweep — including well
    # past both ends — the wall must stay masked.
    for base_x in (2.0, 3.0, 3.6, 4.6):
        A, _, _, _ = avoidance_rows(field, 0.0, (base_x, 3.2), config)
        assert A.shape[0] == 0, f"wall reappeared as an obstacle at x={base_x}"

    short = AvoidanceConfig(mask_extension=0.5)
    short_field = ObstacleField.from_grid(grid([wall]), RES, ORIGIN, short,
                                          mask_segment=segment)
    A, _, _, _ = avoidance_rows(short_field, 0.0, (4.6, 3.2), short)
    assert A.shape[0] > 0, "the old short corridor is what caused the runaway"


def test_masking_the_wall_still_leaves_other_obstacles_visible():
    config = AvoidanceConfig()
    wall = (0.0, 4.0, 6.0, 4.5)
    pillar = (2.6, 2.6, 3.0, 3.0)          # something else, off the scan line
    segment = ((0.5, 4.0), (5.5, 4.0))
    field = ObstacleField.from_grid(grid([wall, pillar]), RES, ORIGIN, config,
                                    mask_segment=segment)
    A, _, _, closest = avoidance_rows(field, 0.0, (3.4, 3.4), config)
    assert A.shape[0] > 0 and closest < config.influence


def test_the_qp_slows_the_sweep_instead_of_driving_into_an_obstacle():
    """End to end: sweep task + actuator limits + barrier, solved together."""
    config = AvoidanceConfig(safety_margin=0.2, alpha=1.0)
    # Obstacle across the sweep direction (+x), 0.7 m ahead of the base centre.
    field = ObstacleField.from_grid(grid([(3.9, 0.0, 4.4, 6.0)]), RES, ORIGIN, config)
    yaw, p_turret = 0.0, (3.0, 3.0)

    n_arm = 6
    J = np.zeros((6, 3 + n_arm))
    J[0, 0] = 1.0                      # base vx -> world x (the sweep direction)
    J[1, 1] = 1.0
    J[5, 2] = 1.0
    target = np.array([0.03, 0.0, 0.0, 0.0, 0.0, 0.0])   # sweep at 0.03 m/s toward +x
    damping = np.concatenate((np.array([0.01, 0.01, 0.02]), np.full(n_arm, 0.1)))
    tasks = [Task(J, target, 1.0), Task(np.diag(damping), np.zeros(3 + n_arm), 1.0)]

    limits = BaseLimits()
    base_lo, base_hi = box_bounds(limits)
    A_act, act_lo, act_hi = constraint_rows(limits, phi=0.0)
    A_act = np.hstack((A_act, np.zeros((A_act.shape[0], n_arm))))
    A_avoid, avoid_lo, avoid_hi, closest = avoidance_rows(field, yaw, p_turret,
                                                          config, n_arm=n_arm)
    assert A_avoid.shape[0] > 0, "the obstacle should be within influence"

    solution = solve_velocity_qp(
        tasks,
        np.concatenate((base_lo, np.full(n_arm, -0.5))),
        np.concatenate((base_hi, np.full(n_arm, 0.5))),
        A_ineq=np.vstack((A_act, A_avoid)),
        ineq_lo=np.concatenate((act_lo, avoid_lo)),
        ineq_hi=np.concatenate((act_hi, avoid_hi)))

    assert solution.ok
    # Without the barrier the base would take the full 0.03 m/s toward the wall.
    assert solution.u[0] < 0.03
    assert np.all(A_avoid @ solution.u >= avoid_lo - 1e-6)


def test_a_base_already_inside_the_margin_is_told_to_retreat():
    config = AvoidanceConfig(safety_margin=0.4, alpha=1.0)
    field = ObstacleField.from_grid(grid([(0.0, 4.0, 6.0, 4.5)]), RES, ORIGIN, config)
    A, lo, _, _ = avoidance_rows(field, 0.0, (3.0, 3.62), config)   # ~0.1 m of clearance
    # Standing still is no longer allowed: some row demands motion.
    assert np.any(A @ np.zeros(3) < lo - 1e-9)
    # Retreating helps every row (the constraint value rises with retreat speed),
    # even though the rate demanded here exceeds what this base can strafe.
    assert np.all(A @ np.array([0.0, -0.1, 0.0]) > A @ np.zeros(3))


def test_an_impossible_retreat_bends_the_barrier_instead_of_killing_the_solve():
    """Deep inside the margin the barrier can demand 0.3 m/s of retreat, and this
    base strafes at 0.033. As a hard row that is infeasible and the robot stops
    where it is; as a soft row it retreats as fast as it can and reports how far
    short it fell."""
    config = AvoidanceConfig(safety_margin=0.4, alpha=1.0)
    field = ObstacleField.from_grid(grid([(0.0, 4.0, 6.0, 4.5)]), RES, ORIGIN, config)
    A, lo, _, _ = avoidance_rows(field, 0.0, (3.0, 3.62), config)

    limits = BaseLimits()
    base_lo, base_hi = box_bounds(limits)
    A_act, act_lo, act_hi = constraint_rows(limits, phi=0.0)
    tasks = [Task(np.eye(3), np.array([0.03, 0.0, 0.0]), 1.0),
             Task(np.diag([0.01, 0.01, 0.02]), np.zeros(3), 1.0)]

    hard = solve_velocity_qp(tasks, base_lo, base_hi, A_ineq=np.vstack((A_act, A)),
                             ineq_lo=np.concatenate((act_lo, lo)),
                             ineq_hi=np.concatenate((act_hi, np.full(len(lo), np.inf))))
    soft = solve_velocity_qp(tasks, base_lo, base_hi,
                             A_ineq=A_act, ineq_lo=act_lo, ineq_hi=act_hi,
                             soft=[SoftRows(A, lo, name="obstacle")])

    assert not hard.ok, "a hard barrier here is infeasible — that is the point"
    assert soft.ok and soft.slack > 0.0          # bent, and says by how much
    assert soft.u[1] < 0.0                       # ... while retreating from the wall
    assert np.all(soft.u >= base_lo - 1e-6) and np.all(soft.u <= base_hi + 1e-6)


def test_footprint_samples_ring_the_configured_radius():
    config = AvoidanceConfig(footprint_radius=0.45, footprint_offset=(0.1, 0.0),
                             n_samples=8)
    samples = footprint_samples(config)
    assert len(samples) == 8
    for point in samples:
        assert np.linalg.norm(point - np.array([0.1, 0.0])) == pytest.approx(0.45)


def test_unknown_cells_are_ignored_by_default_and_honoured_when_asked():
    data = np.zeros(SHAPE, dtype=int)
    data[int(4.0 / RES):int(4.5 / RES), :] = -1        # unknown strip
    permissive = ObstacleField.from_grid(data, RES, ORIGIN, AvoidanceConfig())
    strict = ObstacleField.from_grid(
        data, RES, ORIGIN, AvoidanceConfig(unknown_is_obstacle=True))
    assert permissive.sample(3.0, 3.0)[0] > 1.0
    assert strict.sample(3.0, 3.0)[0] == pytest.approx(1.0, abs=0.1)


def test_an_empty_costmap_produces_no_constraint():
    field = ObstacleField.from_grid(np.zeros(SHAPE, dtype=int), RES, ORIGIN,
                                    AvoidanceConfig())
    A, _, _, _ = avoidance_rows(field, 0.0, (3.0, 3.0), AvoidanceConfig())
    assert A.shape[0] == 0
