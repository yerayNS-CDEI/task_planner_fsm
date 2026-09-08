r"""How stiff the thing the plate is pressing on turns out to be.

The force row in the sweep's QP bounds how fast the plate may still close on the
wall, given how much force is left before the limit::

    n_hat^T J u  <=  alpha * (F_limit - F) / K_e

``K_e`` is the only term in that which is not measured: it converts a velocity
into a rate of change of force. Get it too low and the bound is slack and the
row does nothing; too high and the bound strangles a press that was going fine.
Neither failure is quiet, but neither is acceptable either, so it is estimated
online rather than guessed once.

**Regress force on COMMANDED TRAVEL, not on the sensed distance.** This is the
whole reason the module exists rather than being three lines in the sweep node.
The obvious estimator is a slope through the plate's own range readings against
the force they came with — on the robot, 0.8 N at 14.0 cm and 2.9 N at 13.6 cm,
which gives about 500 N/m. But the plane fit those distances come from has a
sigma of 4.2 mm, so that 4 mm baseline is ONE SIGMA of noise wide: the number is
an artefact of the fit, not a measurement of the wall. The whole penetration a
press makes is a fraction of a millimetre, three orders under the sensor, so no
amount of filtering rescues the sensed distance as a regressor here.

The commanded normal velocity has no such problem. It is a number this loop
chose, it is exact, and integrating it gives the displacement that produced the
force. What it does not capture is the robot failing to follow — speed scaling,
a protective stop, an arm lagging its setpoint — so the caller passes the rate
the SOLVER produced for the whole body, and the sweep's own lead clamp and
hardware watch are what notice when that stops being true.

**Conservative means HIGH.** Overestimating ``K_e`` tightens the bound, which
costs a slow press. Underestimating it loosens the bound, which costs the plate.
So the estimate is floored, not capped, at the value the row should assume when
it knows nothing — and a ceiling exists only to stop a noise-driven fit stalling
the press outright.

The floor is what makes every failure of this fit bounded, and that is worth
stating plainly, because there are several ways to under-read the slope and no
way to detect all of them from in here. The arm not following its setpoint
(speed scaling, a protective stop) accrues commanded travel that never happened.
The wheel leaving the surface makes the force fall while the travel rises. A
mixed window spanning both a rising and a falling force can average to a small
positive slope. In every one of those the estimate falls back toward the floor
and the barrier becomes as loose as its no-knowledge default — never looser, and
never inverted, since a non-positive slope is refused outright. Losing contact is
the benign case anyway: the force is low, so the bound has plenty of headroom in
it whatever ``K_e`` says.

Pure numpy, no ROS, no clock: the caller passes the measured ``dt``.
"""

import math

import numpy as np


class ContactStiffness:
    """Exponentially weighted least-squares slope of force against travel.

    ``update`` is called once per control cycle with the commanded normal rate
    (positive toward the wall), the measured force, and the measured period. It
    returns the current estimate in N/m, which is also available as ``value``.
    """

    def __init__(self, floor=2000.0, ceiling=5.0e4, tau=3.0,
                 min_travel_sigma=2.0e-5, min_samples=8.0):
        # What the row assumes before it has learned anything, and the value a
        # fit is never allowed to go below. 2000 N/m is deliberately stiffer
        # than the ~500 N/m the caster bars flexing would suggest: the bars
        # bottom out, and past that the plate is loading concrete at ~2e4, so a
        # single linear stiffness that has to cover both should sit toward the
        # stiff end. At 2000 with alpha = 1 the bound is 12.5 mm/s at the 5 N
        # target and 2.5 mm/s at 25 N, which binds late and hard — the shape
        # the row wants.
        self.floor = float(floor)
        # Only to stop a fit driven by noise from clamping the press to a stop.
        # Above concrete's own ~2e4, so it never binds on a real surface.
        self.ceiling = float(ceiling)
        # Forgetting time. Long enough to span a useful travel baseline, short
        # enough that the estimate follows the wall changing under the wheel.
        self.tau = float(tau)
        # The fit is a slope, so it needs the regressor to have MOVED. Below
        # this much spread in travel the slope is noise over noise, and the last
        # estimate is kept instead. 20 um: the press makes a quarter of a
        # millimetre for 5 N, so this is well inside a real press and well
        # outside a plate sitting still.
        self.min_travel_sigma = float(min_travel_sigma)
        # ...and enough samples that one pair cannot define a line.
        self.min_samples = float(min_samples)
        self.reset()

    def reset(self):
        """Forget everything. Call at the start of a segment, or on re-contact."""
        self.value = self.floor
        self.fitted = False      # whether ``value`` is a measurement or the floor
        self.travel = 0.0        # cumulative commanded normal travel, m
        self._w = 0.0            # exponentially weighted sample count
        self._x = 0.0            # ...and weighted sums of x, f, x^2, x*f
        self._f = 0.0
        self._xx = 0.0
        self._xf = 0.0

    # ------------------------------------------------------------------
    @property
    def travel_sigma(self):
        """Spread of the travel the current fit is looking at, m."""
        if self._w <= 0.0:
            return 0.0
        return math.sqrt(max(0.0, self._xx / self._w - (self._x / self._w) ** 2))

    def update(self, normal_rate, force, dt):
        """One cycle. ``normal_rate`` is positive toward the wall, in m/s."""
        dt = max(float(dt), 0.0)
        self.travel += float(normal_rate) * dt

        decay = math.exp(-dt / self.tau) if self.tau > 0.0 else 0.0
        x, f = self.travel, float(force)
        self._w = decay * self._w + 1.0
        self._x = decay * self._x + x
        self._f = decay * self._f + f
        self._xx = decay * self._xx + x * x
        self._xf = decay * self._xf + x * f

        if self._w >= self.min_samples and self.travel_sigma >= self.min_travel_sigma:
            mean_x, mean_f = self._x / self._w, self._f / self._w
            var = self._xx / self._w - mean_x * mean_x
            cov = self._xf / self._w - mean_x * mean_f
            slope = cov / var if var > 0.0 else 0.0
            # A negative slope means force FELL as the plate advanced, which is
            # not a wall — it is the wheel coming off one, or the sign of the
            # commanded rate disagreeing with what the arm did. Keep the last
            # good estimate rather than inverting the bound.
            if slope > 0.0:
                self.value = float(min(max(slope, self.floor), self.ceiling))
                self.fitted = True
        return self.value

    def velocity_cap(self, force, force_limit, alpha):
        """Largest approach rate the remaining force headroom allows, m/s.

        The control barrier ``h = F_limit - F`` with ``h_dot >= -alpha * h``,
        which is the same shape ``qp.joint_limit_bounds`` uses to stop a joint
        driving into its stop. Goes to zero as the force reaches the limit, and
        negative past it, which asks the solver to retreat.
        """
        return float(alpha * (float(force_limit) - float(force)) / self.value)


def force_limit_rows(normal_row, force, force_limit, alpha, stiffness):
    """One soft row bounding the whole body's approach rate along the normal.

    Returned in the ``A u + s >= lower`` form the QP's soft groups take, so the
    row is negated: ``-n^T J u >= -cap`` is ``n^T J u <= cap``.

    ``normal_row`` is expected to be ``n^T J`` on the WHOLE-BODY Jacobian, so
    base motion toward the wall spends the same headroom as arm motion does.
    That is not by itself new — the sweep's press task is already on the same
    row, at a weight high enough that the arm absorbs base motion without this
    — but a constraint and a weighted target fail differently, and it is the
    failure that this is for. See ``press_force_alpha`` in sweep_node.
    """
    row = -np.atleast_2d(np.asarray(normal_row, dtype=float))
    cap = stiffness.velocity_cap(force, force_limit, alpha)
    return row, np.array([-cap])
