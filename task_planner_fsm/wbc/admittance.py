r"""Pressing the GPR wheel against the wall with our own force loop.

UR's ``force_mode`` would do this in the robot's own controller, and it is the
better tool. We cannot use it: the driver's hardware component carries an
explicit compatibility table (``ur_robot_driver/src/hardware_interface.cpp``)::

    mode_compatibility_[HW_IF_POSITION][FORCE_MODE_GPIO] = false;
    mode_compatibility_[HW_IF_VELOCITY][FORCE_MODE_GPIO] = false;

``prepare_command_mode_switch`` enforces it, so force mode combines with the
passthrough trajectory controller and nothing else. The whole-body sweep streams
servoj setpoints through ``forward_position_controller``. Confirmed on hardware
2026-08-27: activating one deactivates the other. So the compliance has to
happen here, in the same QP cycle as everything else.

**What replaces what.** The sweep normally regulates the plate's DISTANCE to the
wall::

    v_normal = k_standoff * (distance - standoff)

That is right for a 20 cm scanning standoff and useless for a press: the GPR
wheel has to touch, and "touching" is a force, not a distance. So on the normal
axis only, this module substitutes::

    v_normal = gain * (target_force - measured_force)

Everything else about the sweep is untouched. The tangent still carries the
travel, the arm still holds the row height and the plate still stays parallel
from the same six ranges. Only the meaning of the normal axis changes.

**Sign convention, fixed here so nothing downstream has to think about it.**
Positive velocity is TOWARD the wall, matching ``m_hat`` in the sweep node.
Positive force means PRESSING INTO the wall. The UR reports the opposite on
tool0 Z (pressing reads negative), so the caller flips the sign at the ROS
boundary and this module only ever sees "how hard are we pushing".

**Why the gain is so small.** The loop is a velocity source against a stiff
environment::

    v = k (F_d - F),    F = K_e x    ->    e[n+1] = e[n] (1 - K_e k dt)

so it rings once ``k > 2 / (K_e dt)``. Concrete with a hard wheel is somewhere
around ``K_e ~ 2e4 N/m``, and ``dt`` is 20 ms, which puts the boundary at about
``5e-3 m/s/N``. Measured against that model: 5e-3 gives a 2 N limit cycle and
1e-2 gives 17 N and bounces off the surface entirely.

The default here is ``5e-5``, a hundred times under it. That is not timidity.
The model above has no servo lag in it, and the real path has 30-50 ms of it, so
the true boundary is LOWER than the measured one — by how much is not knowable
without pressing a real wall. Until something has, the gain stays where a wrong
estimate of ``K_e`` by an order of magnitude still leaves margin.

Two other things hold the loop together, and both are load-bearing rather than
cosmetic. The EMA on the force is stability margin, not just noise rejection:
at ``k = 1e-2`` the unfiltered loop rings at 17 N and the filtered one settles.
And ``v_max`` contains a badly tuned gain outright — at 200x the default it is
the clamp, not the gain, that keeps the press from banging.

A spring-loaded wheel mount would drop ``K_e`` by an order of magnitude and let
the gain rise with it. There is no such mount, so the loop stays conservative.
A 5 N error moves the plate about a quarter of a millimetre per second. That is
not slow for what it does: this regulates CONTACT, while the tangent carries the
scan.

**The distance sensors do not go away.** They stop being the setpoint and become
the safety envelope: the press may never drive the plate closer than
``min_distance`` to the sensed plane, whatever the force says. A force reading
that is wrong -- a failed tare, a snagged cable -- then cannot walk the arm into
the wall, because the thing that would stop it is a different sensor.

**The tare is done here, not by the driver.** The loop begins in ``TARE``: it
holds the normal axis still for ``tare_cycles`` and averages what the sensor
reports with nothing touching, then subtracts that from every later reading. An
untared TCP sensor reads several newtons of payload offset against a 5 N target,
which is the difference between leaning on the wall and never reaching it -- and
the second failure is silent, because the GPR records a clean-looking scan of
nothing.

UR's ``/io_and_status_controller/zero_ftsensor`` would also do this, and is not
called, for three reasons. It is a service, so it can be absent or slow, and a
press that starts before it lands would tare against a moving baseline. It
cannot be tested without hardware, whereas this can. And it corrects the sensor
globally while what the loop actually needs is the offset AT THIS POSE, which is
what gets measured. Calling it as well would do no harm; it would just be
belt-and-braces on something already guaranteed.

Distance, not force, is what says it is safe to tare. Taring in contact would
fold the contact force into the zero and the loop would then press until the
true force reached target-plus-contact. Force cannot detect that, because the
whole premise is that force is not yet trustworthy -- so the plate's own ranges
decide, and a tare attempted too close to the surface is a fault, not a warning.
"""

import numpy as np

TARE = "tare"          # holding still in free space, measuring the sensor's zero
SEEK = "seek"          # not touching yet; close the gap on distance
PRESS = "press"        # touching; regulate force


class AdmittancePress:
    """The normal axis during a press: a contact state machine and a force loop.

    One instance per sweep. ``update`` is called once per control cycle and
    returns the velocity to put on the normal axis, in place of the standoff
    law. It owns no ROS handles and no clock, so it can be tested against a
    simulated wall.
    """

    def __init__(self, target_force=5.0, gain=5.0e-5, v_max=0.005,
                 seek_speed=0.01, contact_force=1.0, release_force=0.5,
                 force_limit=25.0, min_distance=0.005, filter_alpha=0.2,
                 stall_cycles=100, tare_cycles=25, tare_min_distance=0.05):
        self.target_force = float(target_force)
        self.gain = float(gain)
        self.v_max = float(v_max)
        self.seek_speed = float(seek_speed)
        # Hysteresis, and it matters: a single threshold at the contact force
        # would flip state every time the filtered force crossed it, which on a
        # real surface is several times a second.
        self.contact_force = float(contact_force)
        self.release_force = float(release_force)
        self.force_limit = float(force_limit)
        self.min_distance = float(min_distance)
        self.filter_alpha = float(filter_alpha)
        self.stall_cycles = int(stall_cycles)
        # Tare: how many cycles to average the sensor's free-space reading over,
        # and how far off the wall the plate must be for that average to mean
        # anything. 25 cycles is half a second at 50 Hz.
        self.tare_cycles = int(tare_cycles)
        self.tare_min_distance = float(tare_min_distance)
        self.reset()

    def reset(self):
        self.state = TARE if self.tare_cycles > 0 else SEEK
        self.force = 0.0        # filtered, de-biased, positive = pressing in
        self.bias = 0.0         # the sensor's reading with nothing touching it
        self.fault = None       # set once; the caller decides what to do
        self._seeded = False
        self._stalled = 0
        self._tare_samples = []

    # ------------------------------------------------------------------
    @property
    def in_contact(self):
        return self.state == PRESS

    @property
    def stalled(self):
        """Sat at the envelope in SEEK without ever feeling the wall.

        Either the plate is not where the ranges say it is, or the force sensor
        is not reporting. Both mean the press is not happening and the sweep
        should not pretend otherwise.
        """
        return self._stalled >= self.stall_cycles

    def error(self):
        """Signed force error, N. Positive means we are pressing too softly."""
        return self.target_force - self.force

    # ------------------------------------------------------------------
    def update(self, raw_force, distance):
        """One cycle. Returns the normal-axis velocity, positive = toward the wall.

        ``raw_force`` is the measured press force in newtons, already flipped so
        that positive means pressing into the wall, and NOT yet corrected for the
        sensor's zero — that is measured here. ``distance`` is the plate's sensed
        gap to the surface, the same number the standoff law used.
        """
        # --- Tare -----------------------------------------------------------
        # An untared TCP sensor reads several newtons of payload offset, against
        # a 5 N target. That is not a small error: it is the difference between
        # leaning on the wall and never touching it, and the second one is
        # silent — the GPR records a clean-looking scan of nothing. So the zero
        # is measured here, in free space, before any of it is believed.
        #
        # Distance is the guard, not force. Taring while already in contact
        # would fold the contact force into the zero, and the loop would then
        # press until the TRUE force reached target-plus-contact. Force cannot
        # be used to detect that, because the whole point is that force is not
        # yet trustworthy — so the plate's own ranges decide.
        if self.state == TARE:
            if distance is not None and distance < self.tare_min_distance:
                self.fault = (
                    f"cannot tare the force sensor: the plate is {distance * 100:.1f} cm "
                    f"from the surface, inside the {self.tare_min_distance * 100:.0f} cm "
                    f"free-space margin the tare needs (is it already touching?)")
                return 0.0
            self._tare_samples.append(float(raw_force))
            if len(self._tare_samples) < self.tare_cycles:
                return 0.0                      # hold still while measuring
            self.bias = float(np.mean(self._tare_samples))
            self.state = SEEK
            return 0.0

        # Seed the filter rather than ramping from zero: starting at 0 N would
        # read as "no contact" for the first several cycles even when the plate
        # is already loaded, and SEEK would drive further into the wall.
        force = float(raw_force) - self.bias
        if not self._seeded:
            self.force = force
            self._seeded = True
        else:
            self.force += self.filter_alpha * (force - self.force)

        # Checked on the FILTERED force, so a single spike from the base thumping
        # over a floor joint cannot abort a good sweep, but a real overload --
        # which persists -- trips within a few cycles.
        if self.force > self.force_limit:
            self.fault = (f"press force {self.force:.1f} N exceeded the "
                          f"{self.force_limit:.1f} N limit")

        if self.state == SEEK and self.force >= self.contact_force:
            self.state = PRESS
            self._stalled = 0
        elif self.state == PRESS and self.force < self.release_force:
            # Contact lost: a hollow, a gap, the wheel riding over a lip. Go
            # back to closing the distance rather than commanding the full force
            # error, which out of contact is just "drive at the wall".
            self.state = SEEK

        if self.state == PRESS:
            # v_max bounds the FORCE loop only. It is sized for contact — a few
            # mm/s — and applying it to the approach as well would silently cap
            # seek_speed at it, so raising the seek speed would do nothing.
            v = float(np.clip(self.gain * self.error(), -self.v_max, self.v_max))
        else:
            v = self.seek_speed

        # The envelope. Approach is refused inside min_distance whatever the
        # force says; retreat is always allowed, so a press that has gone too
        # deep can still back out.
        if distance is not None and distance <= self.min_distance:
            v = min(v, 0.0)
            self._stalled += 1 if self.state == SEEK else 0
        else:
            self._stalled = 0

        return float(v)
