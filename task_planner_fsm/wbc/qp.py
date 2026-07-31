"""The velocity QP that resolves one task twist across base + arm.

The controller wants a plate twist (sweep along the wall, hold the standoff,
keep the plate parallel); the robot has nine fast DOF to produce it. That is
three more than the task needs, so the mapping is not unique and the choice is
made by a quadratic program solved every cycle::

    min_u  sum_k w_k || J_k u - xdot_k ||^2
    s.t.   lb <= u <= ub                      (per-DOF speed / joint-limit box)
           c_lo <= A u <= c_hi                (base actuator limits, see base_model)

Every term is a weighted least-squares task, so secondary objectives (stay near
a posture, prefer the base over the arm for the along-wall motion) are just
extra rows with small weights rather than an explicit null-space projection.
That keeps one code path, degrades gracefully when the primary task is
infeasible, and — unlike a pseudoinverse — lets hard limits be *constraints*
instead of after-the-fact clipping.

Solved with OSQP when available. The fallback is a box-constrained least squares
(``scipy.optimize.lsq_linear``), which handles everything except the base
actuator rows; the caller is told via ``QPSolution.solver`` so it can warn.
"""

from dataclasses import dataclass

import numpy as np

try:  # optional: only needed for the constrained solve
    import osqp
    import scipy.sparse as sp
    _HAVE_OSQP = True
except ImportError:  # pragma: no cover - exercised only on hosts without osqp
    _HAVE_OSQP = False

from scipy.optimize import lsq_linear


@dataclass
class Task:
    """One weighted least-squares objective ``w * ||J u - target||^2``.

    ``weight`` may be a scalar or a per-row array (e.g. to weight the linear
    part of a twist differently from the angular part).
    """

    jacobian: np.ndarray
    target: np.ndarray
    weight: float = 1.0

    def rows(self):
        J = np.atleast_2d(np.asarray(self.jacobian, dtype=float))
        b = np.atleast_1d(np.asarray(self.target, dtype=float))
        w = np.asarray(self.weight, dtype=float)
        s = np.sqrt(np.broadcast_to(w, (J.shape[0],)))
        return J * s[:, None], b * s


@dataclass
class QPSolution:
    u: np.ndarray
    status: str
    solver: str
    task_residual: float

    @property
    def ok(self):
        return self.u is not None


def stack_tasks(tasks):
    """Fold the weighted tasks into one least-squares pair ``(A, b)``."""
    blocks = [task.rows() for task in tasks]
    return (np.vstack([A for A, _ in blocks]),
            np.concatenate([b for _, b in blocks]))


def joint_limit_bounds(q, lower, upper, qdot_max, margin=0.1, gain=1.0):
    """Velocity box that also keeps joint POSITIONS inside their limits.

    A plain speed box lets a joint drive straight into its stop; scaling the
    allowed velocity with the remaining travel (``gain * distance to the
    margin``) makes the QP slow down and stop there instead, and — if a joint
    somehow starts outside the margin — leaves only the velocity that brings it
    back. Both arrays are clipped to ``+/- qdot_max``.
    """
    q = np.asarray(q, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    qdot_max = np.broadcast_to(np.asarray(qdot_max, dtype=float), q.shape)
    hi = np.clip(gain * ((upper - margin) - q), -qdot_max, qdot_max)
    lo = np.clip(gain * ((lower + margin) - q), -qdot_max, qdot_max)
    return lo, hi


def solve_velocity_qp(tasks, lb, ub, A_ineq=None, ineq_lo=None, ineq_hi=None,
                      ridge=1e-8, osqp_settings=None):
    """Solve the stacked-task QP subject to a box and optional linear rows.

    Returns a :class:`QPSolution`; ``u`` is ``None`` only if every solver failed,
    which the caller must treat as "command zero velocity this cycle".
    """
    A, b = stack_tasks(tasks)
    lb = np.asarray(lb, dtype=float)
    ub = np.asarray(ub, dtype=float)

    if _HAVE_OSQP:
        solution = _solve_osqp(A, b, lb, ub, A_ineq, ineq_lo, ineq_hi, ridge, osqp_settings)
        if solution is not None:
            u, status = solution
            return QPSolution(u, status, "osqp", float(np.linalg.norm(A @ u - b)))

    # Fallback: box-constrained least squares. Exact for the box, blind to the
    # base actuator rows — the caller should say so out loud once.
    try:
        result = lsq_linear(A, b, bounds=(lb, ub), max_iter=50)
        u = np.asarray(result.x, dtype=float)
        solver = "lsq_linear" if A_ineq is None else "lsq_linear(box-only)"
        return QPSolution(u, "solved", solver, float(np.linalg.norm(A @ u - b)))
    except Exception as exc:  # pragma: no cover - numerical last resort
        return QPSolution(None, f"failed: {exc}", "none", float("inf"))


def _solve_osqp(A, b, lb, ub, A_ineq, ineq_lo, ineq_hi, ridge, settings):
    """OSQP on the normal equations; ``None`` when it cannot produce a solution."""
    n = A.shape[1]
    P = A.T @ A + ridge * np.eye(n)
    q = -A.T @ b

    rows = [np.eye(n)]
    lower = [lb]
    upper = [ub]
    if A_ineq is not None and len(np.atleast_2d(A_ineq)):
        rows.append(np.atleast_2d(np.asarray(A_ineq, dtype=float)))
        lower.append(np.atleast_1d(np.asarray(ineq_lo, dtype=float)))
        upper.append(np.atleast_1d(np.asarray(ineq_hi, dtype=float)))

    problem = osqp.OSQP()
    # Only settings that exist in both the 0.6.x and 1.x OSQP APIs (the polish
    # flag was renamed, so it is left at whatever the installed version defaults
    # to rather than pinned here).
    defaults = {"verbose": False, "eps_abs": 1e-6, "eps_rel": 1e-6, "max_iter": 4000}
    defaults.update(settings or {})
    try:
        problem.setup(P=sp.csc_matrix(np.triu(P)), q=q,
                      A=sp.csc_matrix(np.vstack(rows)),
                      l=np.concatenate(lower), u=np.concatenate(upper),
                      **defaults)
        result = problem.solve()
    except Exception:  # pragma: no cover - setup/solve blowups fall through
        return None
    status = str(result.info.status)
    if result.x is None or not np.all(np.isfinite(result.x)):
        return None
    if "solved" not in status:   # covers "solved inaccurate"
        return None
    # OSQP satisfies the box only to solver tolerance; clip so a command can
    # never leave the actuator envelope by a hair.
    return np.clip(np.asarray(result.x, dtype=float), lb, ub), status
