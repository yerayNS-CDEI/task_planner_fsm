"""The base's actuator limits must match what sim_controller actually enforces.

The QP is only as good as this model: if the constraint rows are wrong, the
controller silently rescales the command and the sweep speed drifts from what
was asked for. These tests pin the model to the formulas in
``sim_odometry.cpp`` (``InverseKine_relative_speed`` /
``applyClosedFormRotationalLimits``).
"""

import numpy as np
import pytest

from task_planner_fsm.wbc.base_model import (
    BaseLimits,
    box_bounds,
    constraint_rows,
    wheel_and_turret_rates,
)

# navi-wall/config/diffdrive_controllers.yaml, sim_controller block.
DEPLOYED = BaseLimits()


def test_sweep_speed_cap_is_set_by_the_chassis_yaw_rate():
    # Lateral motion yaws the chassis at v/d1, so 0.2 rad/s over d1 = 0.167 m.
    assert DEPLOYED.max_lateral_speed() == pytest.approx(0.167 * 0.2, rel=1e-9)
    # The 0.05 m/s the old Nav2 sweep asked for is ABOVE that: sim_controller
    # would have been rescaling it.
    assert DEPLOYED.max_lateral_speed() < 0.05


def test_the_speed_cap_depends_on_which_way_the_base_is_going():
    """Straight ahead is cheap; sideways has to yaw the chassis and is ~6x slower."""
    forward = DEPLOYED.max_speed_along(0.0, 0.0)
    sideways = DEPLOYED.max_speed_along(np.pi / 2, np.pi / 2)
    assert forward == pytest.approx(DEPLOYED.vx_max)
    assert sideways == pytest.approx(DEPLOYED.max_lateral_speed())
    assert forward > 5 * sideways
    # In between, the tighter of the two binds and the cap varies smoothly.
    diagonal = DEPLOYED.max_speed_along(np.pi / 4, np.pi / 4)
    assert sideways < diagonal < forward


def test_a_turned_turret_and_chassis_are_capped_separately():
    # Turret square to the travel direction but the chassis at 45 deg: the box
    # limit is slack, the chassis yaw limit is not.
    cap = DEPLOYED.max_speed_along(0.0, np.pi / 4)
    assert cap == pytest.approx(
        DEPLOYED.center_distance * DEPLOYED.w_chassis_max / np.sin(np.pi / 4))


def test_a_slow_turret_motor_can_bind_before_the_chassis():
    slow_turret = BaseLimits(w_turret_motor_max=0.1)
    assert slow_turret.max_lateral_speed() == pytest.approx(0.167 * 0.1, rel=1e-9)


def test_wheel_and_turret_rates_reproduce_the_controller_formulas():
    phi = 0.3
    vx, vy, wz = 0.05, 0.02, 0.01
    w_left, w_right, phi_dot, w_chassis = wheel_and_turret_rates(DEPLOYED, (vx, vy, wz), phi)

    d1, R, d2 = (DEPLOYED.center_distance, DEPLOYED.wheel_radius,
                 DEPLOYED.wheel_separation)
    x_c = np.cos(phi) * vx - np.sin(phi) * vy
    y_c = np.sin(phi) * vx + np.cos(phi) * vy
    assert w_left == pytest.approx(x_c / R - d2 * y_c / (2 * R * d1))
    assert w_right == pytest.approx(x_c / R + d2 * y_c / (2 * R * d1))
    assert phi_dot == pytest.approx(wz - y_c / d1)
    assert w_chassis == pytest.approx(y_c / d1)


def test_constraint_rows_bound_the_chassis_yaw_rate_at_the_lateral_cap():
    A, lo, hi = constraint_rows(DEPLOYED, phi=0.0)
    # With the chassis parked square (phi=0) the turret y axis IS the chassis
    # lateral axis, so the first row reads vy / d1.
    np.testing.assert_allclose(A[0], [0.0, 1.0 / DEPLOYED.center_distance, 0.0], atol=1e-12)
    cap = DEPLOYED.max_lateral_speed()
    assert A[0] @ np.array([0.0, cap, 0.0]) == pytest.approx(hi[0])
    assert lo[0] == -hi[0]


def test_constraint_rows_follow_the_turret_as_it_winds_up():
    # At phi = pi/2 the turret's +x is the chassis lateral axis, so it is vx that
    # costs chassis yaw rate — the reason phi is read every cycle.
    A, _, hi = constraint_rows(DEPLOYED, phi=np.pi / 2.0)
    np.testing.assert_allclose(A[0], [1.0 / DEPLOYED.center_distance, 0.0, 0.0], atol=1e-12)
    assert hi[0] == pytest.approx(DEPLOYED.w_chassis_max)


def test_a_twist_satisfying_the_rows_stays_inside_every_actuator_limit():
    limits = BaseLimits(wheel_speed_max=5.0)
    rng = np.random.default_rng(0)
    for _ in range(200):
        phi = rng.uniform(-np.pi, np.pi)
        A, lo, hi = constraint_rows(limits, phi)
        box_lo, box_hi = box_bounds(limits)
        u = rng.uniform(box_lo, box_hi)
        if np.any(A @ u < lo - 1e-12) or np.any(A @ u > hi + 1e-12):
            continue   # the QP would have rejected this one
        w_left, w_right, phi_dot, w_chassis = wheel_and_turret_rates(limits, u, phi)
        assert abs(w_chassis) <= limits.w_chassis_max + 1e-9
        assert abs(phi_dot) <= limits.w_turret_motor_max + 1e-9
        assert max(abs(w_left), abs(w_right)) <= limits.wheel_speed_max + 1e-9


def test_wheel_rows_are_omitted_when_no_wheel_limit_is_configured():
    A_without, _, _ = constraint_rows(BaseLimits(), phi=0.0)
    A_with, _, _ = constraint_rows(BaseLimits(wheel_speed_max=5.0), phi=0.0)
    assert A_without.shape[0] == 2      # chassis yaw + turret motor
    assert A_with.shape[0] == 4         # ... plus both wheels
