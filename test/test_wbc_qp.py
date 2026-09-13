"""The QP has to do three things: hit the task, respect the limits, split the work.

The third is the point of whole-body control — when the base runs out of
authority (the chassis yaw rate caps the along-wall speed), the arm must pick up
the remainder instead of the sweep silently slowing down.
"""

import numpy as np
import pytest

from task_planner_fsm.wbc.base_model import BaseLimits, box_bounds, constraint_rows
from task_planner_fsm.wbc.qp import (
    SoftRows, Task, joint_limit_bounds, solve_velocity_qp)

WIDE = 10.0


def test_an_unconstrained_solve_matches_the_pseudoinverse():
    rng = np.random.default_rng(1)
    J = rng.normal(size=(6, 9))
    target = rng.normal(size=6) * 0.05
    # A light damping task is what picks the minimum-norm point out of the
    # three-dimensional null space, so the solve should land on the pseudoinverse.
    solution = solve_velocity_qp(
        [Task(J, target, 1.0), Task(np.eye(9) * 1e-2, np.zeros(9), 1.0)],
        -np.full(9, WIDE), np.full(9, WIDE))
    assert solution.ok
    np.testing.assert_allclose(J @ solution.u, target, atol=1e-4)
    np.testing.assert_allclose(solution.u, np.linalg.pinv(J) @ target, atol=1e-3)


def test_the_box_is_never_exceeded():
    J = np.zeros((6, 9))
    J[0, 0] = 1.0
    solution = solve_velocity_qp(
        [Task(J, np.array([5.0, 0, 0, 0, 0, 0]), 1.0)],
        -np.full(9, 0.2), np.full(9, 0.2))
    assert solution.ok
    assert solution.u[0] == pytest.approx(0.2, abs=1e-6)
    assert np.all(solution.u <= 0.2 + 1e-9) and np.all(solution.u >= -0.2 - 1e-9)


def _sweep_problem(demand):
    """A 9-DOF toy where base ``vy`` and arm joint 1 both drive world +y."""
    J = np.zeros((6, 9))
    J[0, 0] = 1.0    # base vx -> world x
    J[1, 1] = 1.0    # base vy -> world y  (the along-wall axis)
    J[5, 2] = 1.0    # base wz -> yaw
    J[1, 3] = 1.0    # arm joint 1 -> world y
    target = np.array([0.0, demand, 0.0, 0.0, 0.0, 0.0])
    damping = np.concatenate((np.array([0.01, 0.01, 0.02]), np.full(6, 0.1)))
    tasks = [Task(J, target, 1.0), Task(np.diag(damping), np.zeros(9), 1.0)]

    limits = BaseLimits()
    base_lo, base_hi = box_bounds(limits)
    A, lo, hi = constraint_rows(limits, phi=0.0)
    A = np.hstack((A, np.zeros((A.shape[0], 6))))
    return limits, solve_velocity_qp(
        tasks,
        np.concatenate((base_lo, np.full(6, -0.5))),
        np.concatenate((base_hi, np.full(6, 0.5))),
        A_ineq=A, ineq_lo=lo, ineq_hi=hi)


def test_the_base_carries_a_sweep_it_can_hold():
    limits, solution = _sweep_problem(0.02)
    assert solution.ok
    assert solution.u[1] == pytest.approx(0.02, abs=2e-3)   # base does it all
    assert abs(solution.u[3]) < 1e-3                        # arm stays put


def test_the_arm_takes_over_what_the_chassis_yaw_rate_forbids():
    limits, solution = _sweep_problem(0.06)
    assert solution.ok
    cap = limits.max_lateral_speed()
    assert solution.u[1] == pytest.approx(cap, abs=1e-3)    # base pinned at its cap
    assert solution.u[3] > 0.02                             # arm covers the rest
    assert solution.u[1] + solution.u[3] == pytest.approx(0.06, abs=5e-3)


def test_joint_limit_bounds_stop_a_joint_at_its_stop():
    lower = np.array([-1.0, -1.0])
    upper = np.array([1.0, 1.0])
    # Second joint is already inside the margin at the upper stop.
    lo, hi = joint_limit_bounds(np.array([0.0, 0.95]), lower, upper, 0.5, margin=0.1, gain=1.0)
    assert hi[0] == pytest.approx(0.5) and lo[0] == pytest.approx(-0.5)
    assert hi[1] < 0.0        # only motion AWAY from the stop is allowed
    assert lo[1] == pytest.approx(-0.5)
    assert np.all(lo <= hi)


def test_joint_limit_bounds_taper_as_the_stop_approaches():
    lower, upper = np.array([-1.0]), np.array([1.0])
    far = joint_limit_bounds(np.array([0.0]), lower, upper, 0.5, margin=0.1)[1][0]
    near = joint_limit_bounds(np.array([0.7]), lower, upper, 0.5, margin=0.1)[1][0]
    assert far == pytest.approx(0.5)
    assert 0.0 < near < far


# ---------------------------------------------------------------------------
# Soft rows and their slacks
# ---------------------------------------------------------------------------
def test_a_soft_row_bends_instead_of_making_the_solve_infeasible():
    """A barrier deep inside its margin can demand a retreat faster than the
    actuators can deliver. Hard, that stops the robot dead; soft, it retreats as
    fast as it can and reports the shortfall."""
    # One DOF, boxed to +/-0.01, asked by a soft row to move at 0.5.
    solution = solve_velocity_qp(
        [Task(np.eye(1), np.zeros(1), 1.0)], np.full(1, -0.01), np.full(1, 0.01),
        soft=[SoftRows(np.eye(1), np.array([0.5]), 1e3, name="barrier")])

    assert solution.ok, "a soft row must never make the solve infeasible"
    assert solution.u[0] == pytest.approx(0.01, abs=1e-3), "retreat as fast as it can"
    assert solution.slack == pytest.approx(0.49, abs=1e-2), "and report the rest"


def test_one_group_bending_does_not_excuse_another():
    """The reason groups exist at all.

    With a single shared slack, an obstacle barrier that cannot be met hands
    every other soft row the same free violation. For the contact-force barrier
    that is precisely backwards: a barrier pushing the base at the wall is what
    drives the force up, so the moment it engages is the moment the force row
    must NOT be relaxed.
    """
    # Two DOF. u0 is asked for a retreat its box cannot deliver; u1 is asked by
    # the task to run at 1.0 and by a soft cap to stay under 0.02, so it is
    # leaning on that cap and would take any excuse to cross it.
    tasks = [Task(np.diag([1.0, 1.0]), np.array([0.0, 1.0]), 1.0)]
    lb, ub = np.array([-0.01, -1.0]), np.array([0.01, 1.0])
    impossible = SoftRows(np.array([[1.0, 0.0]]), np.array([0.5]), 1e3, name="obstacle")
    cap = SoftRows(np.array([[0.0, -1.0]]), np.array([-0.02]), 1e3, name="force")

    alone = solve_velocity_qp(tasks, lb, ub, soft=[cap])
    apart = solve_velocity_qp(tasks, lb, ub, soft=[impossible, cap])
    slacks = dict(zip(["obstacle", "force"], apart.slacks))

    assert apart.ok and alone.ok
    assert slacks["obstacle"] > 0.4, "the impossible row is the one that bends"
    # The cap gives up the same sliver either way — a soft row always trades a
    # little against the task — but NOT one bit more for the barrier's failure.
    assert slacks["force"] == pytest.approx(alone.slack, abs=1e-6)
    assert apart.u[1] == pytest.approx(alone.u[1], abs=1e-6)

    # And the same problem with both rows sharing ONE group, which is what the
    # QP used to do: now the cap inherits the barrier's whole shortfall.
    shared = solve_velocity_qp(
        tasks, lb, ub,
        soft=[SoftRows(np.vstack((impossible.jacobian, cap.jacobian)),
                       np.concatenate((impossible.lower, cap.lower)), 1e3)])

    assert shared.u[1] > 20.0 * apart.u[1], (
        f"sharing a slack lets the cap be crossed: {shared.u[1]:.3f} against "
        f"{apart.u[1]:.3f} with groups of their own")


def test_rows_inside_one_group_do_share_their_slack():
    """The other half of the contract, and why avoidance rows stay together: a
    retreat that satisfies the worst barrier satisfies the rest, so they should
    not each be billed for it."""
    group = SoftRows(np.array([[1.0], [1.0]]), np.array([0.5, 0.2]), 1e3)
    solution = solve_velocity_qp(
        [Task(np.eye(1), np.zeros(1), 1.0)], np.full(1, -0.01), np.full(1, 0.01),
        soft=[group])

    assert len(solution.slacks) == 1, "one slack for the group, not one per row"
    assert solution.slack == pytest.approx(0.49, abs=1e-2), "sized by the worst row"


def test_no_soft_rows_means_no_slack_variables_at_all():
    solution = solve_velocity_qp(
        [Task(np.eye(2), np.array([0.1, 0.2]), 1.0)],
        np.full(2, -WIDE), np.full(2, WIDE), soft=[])

    assert solution.u.shape == (2,), "the slack columns must not leak into u"
    assert solution.slack == 0.0
    assert solution.u == pytest.approx([0.1, 0.2], abs=1e-4)


def test_a_failed_solve_reports_itself_instead_of_returning_nonsense():
    # Infeasible box (lower above upper) — the caller must see it, not a NaN.
    solution = solve_velocity_qp(
        [Task(np.eye(3), np.ones(3), 1.0)], np.full(3, 1.0), np.full(3, -1.0))
    assert not solution.ok or np.all(np.isfinite(solution.u))
