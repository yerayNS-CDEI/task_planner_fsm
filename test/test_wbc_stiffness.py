"""The contact stiffness estimate, and the force barrier row it sizes.

Pure numpy: the estimator is handed a commanded rate, a force and a period, and
returns N/m. What matters here is not that it is accurate — a single linear
stiffness cannot describe caster bars that bottom out onto concrete — but that
it is wrong in the SAFE direction and that it refuses to answer when it has not
been given enough to answer with.
"""

import numpy as np
import pytest

from task_planner_fsm.wbc.stiffness import ContactStiffness, force_limit_rows

DT = 0.02


def _press_into(stiffness, k_true, rate=5.0e-4, cycles=200, dt=DT, force0=0.0):
    """Advance the plate at ``rate`` into a spring of ``k_true`` N/m."""
    travel, forces = 0.0, []
    for _ in range(cycles):
        travel += rate * dt
        force = force0 + k_true * travel
        stiffness.update(rate, force, dt)
        forces.append(force)
    return np.array(forces)


def test_it_measures_the_slope_of_force_against_commanded_travel():
    stiffness = ContactStiffness(floor=100.0, ceiling=1.0e6)
    _press_into(stiffness, k_true=8000.0)
    assert stiffness.fitted
    assert stiffness.value == pytest.approx(8000.0, rel=0.05)


def test_an_offset_does_not_bias_the_slope():
    """The wheel is already loaded when the estimate starts — the press begins
    at whatever force the contact dwell confirmed. Only the SLOPE is wanted, so
    a constant offset must fall out of the fit rather than inflate it."""
    stiffness = ContactStiffness(floor=100.0, ceiling=1.0e6)
    _press_into(stiffness, k_true=8000.0, force0=4.0)
    assert stiffness.value == pytest.approx(8000.0, rel=0.05)


def test_it_stays_at_the_floor_until_the_plate_has_actually_moved():
    """A slope needs the regressor to have moved. A plate holding a steady force
    gives none, and fitting the noise there would hand the barrier a number with
    no measurement in it."""
    stiffness = ContactStiffness(floor=2000.0)
    for _ in range(500):
        stiffness.update(0.0, 5.0, DT)          # in contact, holding still

    assert not stiffness.fitted, "no travel, so nothing to fit"
    assert stiffness.value == 2000.0, "and the assumed floor is what it keeps"


def test_one_sample_cannot_define_a_line():
    stiffness = ContactStiffness(floor=2000.0, min_samples=8.0)
    stiffness.update(5.0e-4, 0.0, DT)
    stiffness.update(5.0e-4, 5.0, DT)
    assert not stiffness.fitted


def test_a_soft_wall_is_floored_because_being_wrong_low_costs_the_plate():
    """The asymmetry the whole module turns on. Overestimating K_e tightens the
    barrier and costs a slow press; underestimating it loosens the barrier and
    costs the plate. So a genuinely soft surface still gets the floor."""
    stiffness = ContactStiffness(floor=2000.0)
    _press_into(stiffness, k_true=500.0, rate=2.0e-3, cycles=400)
    assert stiffness.value == 2000.0, "a soft reading must not loosen the bound"


def test_a_stiffness_beyond_concrete_is_capped_so_it_cannot_stall_the_press():
    stiffness = ContactStiffness(floor=2000.0, ceiling=5.0e4)
    _press_into(stiffness, k_true=2.0e5, rate=1.0e-4, cycles=400)
    assert stiffness.fitted
    assert stiffness.value == 5.0e4


def test_force_falling_as_the_plate_advances_never_inverts_the_bound():
    """The wheel coming off a wall, or the arm not following the commanded rate.

    A negative slope is refused outright, so the estimate can only ever fall
    back toward the FLOOR — the value the barrier would have used knowing
    nothing. That is the guaranteed worst case for every way this fit can go
    wrong, and it is why the floor is a real number rather than zero.
    """
    stiffness = ContactStiffness(floor=2000.0, ceiling=1.0e6)
    _press_into(stiffness, k_true=8000.0)
    assert stiffness.value == pytest.approx(8000.0, rel=0.05)

    travel, worst = stiffness.travel, stiffness.value
    for _ in range(300):                        # advancing, force falling away
        travel += 5.0e-4 * DT
        stiffness.update(5.0e-4, max(0.0, 8.0 - 4000.0 * (travel - 0.002)), DT)
        worst = min(worst, stiffness.value)

    assert worst >= 2000.0, "a collapsing fit can only reach the floor"
    assert stiffness.value > 0.0, "and never a sign flip, which would invert it"


def test_it_follows_the_wall_changing_under_the_wheel():
    """Bars bottoming out onto concrete is a stiffness step, not a constant, so
    the estimate has to forget. tau is the whole mechanism."""
    stiffness = ContactStiffness(floor=100.0, ceiling=1.0e6, tau=1.0)
    _press_into(stiffness, k_true=1000.0, rate=1.0e-3, cycles=400)
    soft = stiffness.value

    travel, force = stiffness.travel, 1000.0 * stiffness.travel
    for _ in range(400):                        # same plate, ten times stiffer
        travel += 1.0e-4 * DT
        force += 10000.0 * 1.0e-4 * DT
        stiffness.update(1.0e-4, force, DT)

    assert soft == pytest.approx(1000.0, rel=0.1)
    assert stiffness.value == pytest.approx(10000.0, rel=0.2), "it must catch up"


# ---------------------------------------------------------------------------
# The barrier row
# ---------------------------------------------------------------------------
def test_the_cap_closes_as_the_force_approaches_the_limit():
    stiffness = ContactStiffness(floor=2000.0)
    caps = [stiffness.velocity_cap(f, 30.0, 1.0) for f in (0.0, 5.0, 20.0, 30.0)]

    assert caps == pytest.approx([0.015, 0.0125, 0.005, 0.0])
    assert all(a > b for a, b in zip(caps, caps[1:])), "monotone, so it binds late"


def test_past_the_limit_the_row_asks_for_a_retreat():
    """Not just 'stop': the barrier is h_dot >= -alpha h, so once h is negative
    the only motion it permits is back off the wall."""
    stiffness = ContactStiffness(floor=2000.0)
    assert stiffness.velocity_cap(35.0, 30.0, 1.0) < 0.0


def test_the_row_is_built_in_the_form_the_qp_soft_groups_take():
    """``A u + s >= lower``, so ``n^T J u <= cap`` has to arrive negated. Getting
    this backwards would build a barrier that forces the plate INTO the wall."""
    stiffness = ContactStiffness(floor=2000.0)
    normal_row = np.array([1.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    rows, lower = force_limit_rows(normal_row, 5.0, 30.0, 1.0, stiffness)

    assert rows.shape == (1, 9)
    assert rows[0] == pytest.approx(-normal_row)
    cap = stiffness.velocity_cap(5.0, 30.0, 1.0)
    assert lower[0] == pytest.approx(-cap)
    # A command that approaches at exactly the cap sits on the constraint; one
    # that approaches faster violates it; retreating always satisfies it.
    at_cap = np.zeros(9)
    at_cap[0] = cap
    assert float(rows @ at_cap) == pytest.approx(lower[0])
    assert float(rows @ (2.0 * at_cap)) < lower[0]
    assert float(rows @ (-at_cap)) > lower[0]


def test_the_row_spans_the_base_as_well_as_the_arm():
    """The part AdmittancePress structurally cannot do. It outputs an arm
    velocity, so a base being pushed at the wall by the avoidance barrier is
    invisible to it; this row is on the whole-body Jacobian, so base motion
    toward the wall spends the same headroom as arm motion does."""
    stiffness = ContactStiffness(floor=2000.0)
    normal_row = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rows, lower = force_limit_rows(normal_row, 5.0, 30.0, 1.0, stiffness)

    base_only = np.zeros(9)
    base_only[0] = stiffness.velocity_cap(5.0, 30.0, 1.0) * 2.0
    assert float(rows @ base_only) < lower[0], "base motion must violate it too"
