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

**Why the approach is SCHEDULED and not a constant speed.** This is what the
2026-09-07 field run broke on, and the reasoning is worth keeping.

Closing on the wall at a fixed ``seek_speed`` makes the peak contact force a
function of how fast the control loop happens to be running. The wheel keeps
approaching until the loop NOTICES contact, so the penetration is
``v_approach * detection_latency`` and the force is ``K_e`` times that. The
latency is at best one control period plus the servo lag, so at 0.01 m/s into
``K_e ~ 2e4`` a single 100 ms cycle is 1 mm of penetration and 20 N — spent
before the loop gets one chance to react. Driving the REAL class against a
simulated wall at that stiffness, with 40 ms of servo lag:

    loop rate     peak true force     what the filtered force reported
       50 Hz          16.1 N                    5.3 N
       17 Hz          17.8 N                    5.3 N
       10 Hz          40.0 N                    3.9 N
        8 Hz          50.0 N                    3.4 N

The second column is the nastier half: the EMA hides the overload from the very
limit check meant to catch it, which is why the field log jumped from +2.0 N to
32.1 N with nothing in between. On that run the loop was at 8-17 Hz because the
machine was at load 36 on 12 cores, and the sweep node cannot do anything about
that. So the press has to be safe at WHATEVER rate it gets, rather than assume
the 50 Hz it asks for.

The fix is to make the approach speed decay with the remaining gap::

    v_approach <= approach_gain * (gap to the plate's stop)

Then the penetration during the detection latency goes to zero as the gap does,
whatever the latency is, and the peak force stops depending on the loop rate.
This is the same trick ``qp.py:joint_limit_bounds`` already uses to stop a joint
driving into its stop; it is only being applied to one more axis.

**The gap has to be estimated PESSIMISTICALLY.** Scheduling on the raw range is
not enough, and this is the part that is easy to get wrong. The plane fit has
4.2 mm sigma, and a single positive noise excursion authorises full approach
speed at exactly the wrong moment. Worst of 20 noise seeds at 10 Hz, at the
defaults below:

    gap estimate                          nominal    with +10 mm bias
    raw distance                           38.8 N        40.0 N
    filtered distance, no margin           38.7 N        40.0 N
    filtered distance, minus 3 sigma        5.2 N        25.0 N

Read the middle row before touching any of this: at ``approach_gain = 3``, the
FILTER ON ITS OWN BUYS NOTHING. A 4 mm gap authorises 12 mm/s, which is over
``seek_speed`` anyway, so without the margin the schedule does not begin to bind
until the gap is inside the noise — by which point it is too late whether the
estimate is filtered or not. It is the MARGIN that moves the binding point out
to where there is still room to decelerate; the filter is there to stop the
margin-corrected estimate jittering once it does bind. Raising the gain and
dropping the margin looks like the same trade and is not.

The error is deliberately one-sided: under-reading the gap only makes the
approach slower, while over-reading it is what drives the wheel into the wall.
The +10 mm column is that asymmetry priced — a distance sensor reading 1 cm long
costs 25 N, which still clears the 30 N limit but not by much, so a plate offset
calibration is worth more here than any gain in this file.
``approach_min_speed`` is the floor that keeps a pessimistic estimate from
stalling the approach short of the wall forever.

**Every time constant here is in SECONDS, not cycles.** The old code counted
cycles — a 0.2 EMA coefficient, 25 tare cycles, 100 stall cycles — all sized at
50 Hz. At the 10 Hz the robot actually achieved, the force filter's lag went
from 0.1 s to 0.5 s and the tare quietly took 2.5 s instead of 0.5. ``update``
therefore takes the measured ``dt`` and converts: ``alpha = 1 - exp(-dt/tau)``.
On its own that does NOT fix the peak force — checked, still 40 N at 10 Hz,
because the dominant latency at low rate is the sample interval itself, not the
filter — but it makes the reported force honest so the limit fires on time, and
it stops the tare duration drifting with the machine's load.

**Contact is a sustained load, not a threshold crossing.** The second field run
(2026-09-07) found the wall on noise about 0.4 s after the tare, with the wheel
still 10 cm off it. ``touched`` latches by design — the sweep gates its travel on
it — so the base then swept 1.1 m of wall with the GPR scanning air, and every
other failure on that run followed from it: the base travelled into the obstacle
influence radius, the avoidance barrier engaged, and from then on the barrier was
pushing the base off the wall faster than the arm was closing on it.

The threshold was 1.0 N against a sensor whose de-biased noise measures sigma
0.95 N. That is 1.1 sigma, and an approach gets several hundred independent
tries at it. No threshold alone is enough at any level worth using, which is why
there is also a dwell — over 400 simulated 45 s approaches at 10 Hz:

    contact_force    dwell 0     0.10 s     0.20 s
        1.0 N         100%        100%        99%
        2.0 N         100%         36%       0.8%
        3.0 N         100%          0%         0%

Note the first column: with no dwell, EVERY threshold tested latches on noise
eventually. Time over the threshold is what separates contact from noise; the
threshold only sets how long that takes.

**The distance sensors do not go away.** They stop being the setpoint and become
the safety envelope: the press may never drive the plate closer than
``min_distance`` to the sensed plane, whatever the force says. A force reading
that is wrong -- a failed tare, a snagged cable -- then cannot walk the arm into
the wall, because the thing that would stop it is a different sensor. They are
now also what SCHEDULES the approach, which is a second, softer use of the same
measurement: the envelope is the hard floor, the schedule is what stops the
plate arriving at it fast.

**The tare is done here, not by the driver.** The loop begins in ``TARE``: it
holds the normal axis still for ``tare_seconds`` and averages what the sensor
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

import math

import numpy as np

TARE = "tare"          # holding still in free space, measuring the sensor's zero
SEEK = "seek"          # not touching yet; close the gap on distance
PRESS = "press"        # touching; regulate force


class AdmittancePress:
    """The normal axis during a press: a contact state machine and a force loop.

    One instance per sweep. ``update`` is called once per control cycle with the
    measured cycle period and returns the velocity to put on the normal axis, in
    place of the standoff law. It owns no ROS handles and no clock — the caller
    passes ``dt`` — so it can be tested against a simulated wall at any rate.
    """

    def __init__(self, target_force=5.0, gain=5.0e-5, v_max=0.005,
                 seek_speed=0.01, contact_force=3.0, release_force=1.5,
                 contact_dwell=0.15,
                 force_limit=30.0, min_distance=0.005, filter_tau=0.1,
                 stall_seconds=2.0, tare_seconds=0.5, tare_min_distance=0.05,
                 contact_distance=0.13, approach_gain=3.0,
                 approach_margin=0.0126, approach_min_speed=0.0008,
                 distance_tau=0.15, force_limit_dwell=0.06):
        self.target_force = float(target_force)
        self.gain = float(gain)
        self.v_max = float(v_max)
        self.seek_speed = float(seek_speed)
        # Hysteresis, and it matters: a single threshold at the contact force
        # would flip state every time the filtered force crossed it, which on a
        # real surface is several times a second.
        #
        # 3.0 N, not the 1.0 it was. Measured off the 2026-09-07 log, the
        # de-biased force sensor has sigma 0.95 N and swings -2.6 to +2.4 N with
        # nothing touching, so a 1.0 N threshold is 1.1 sigma — it is inside the
        # noise, not above it. That mattered more after the filter was put on a
        # proper time base: a fixed alpha of 0.2 smoothed the same signal to
        # sigma 0.32 N whatever the rate, while tau = 0.1 s gives 0.64 N at
        # 10 Hz, and the threshold that had been ~3 sigma became ~1.5. On the
        # second field run contact latched about 0.4 s after the tare with the
        # wheel still 10 cm off the wall, and because ``touched`` latches, the
        # base then swept 1.1 m of wall scanning air.
        self.contact_force = float(contact_force)
        self.release_force = float(release_force)
        # Contact must PERSIST for this long before it counts. A threshold alone
        # cannot separate contact from noise at any level worth using, because
        # the noise gets several hundred independent tries over an approach:
        # simulated over 400 approaches of 45 s at 10 Hz, 1.0 N latches
        # spuriously 100% of the time at every dwell, 2.0 N needs 0.2 s to get
        # down to 0.8%, and 3.0 N with 0.1 s never latches once. Both halves are
        # load-bearing — this is the one that survives someone deciding the
        # threshold is too conservative.
        self.contact_dwell = float(contact_dwell)
        self.force_limit = float(force_limit)
        self.min_distance = float(min_distance)
        # Time constant of the force EMA, in SECONDS. Converted to a per-cycle
        # coefficient against the measured dt, so the lag is 0.1 s whether the
        # loop is at 50 Hz or 10.
        self.filter_tau = float(filter_tau)
        # Both in seconds, for the same reason. 2.0 s and 0.5 s are what the old
        # 100 and 25 cycle counts meant at the 50 Hz they were sized at.
        self.stall_seconds = float(stall_seconds)
        self.tare_seconds = float(tare_seconds)
        self.tare_min_distance = float(tare_min_distance)
        # --- the approach schedule ---------------------------------------
        # Where the plate STOPS: the range reading when the wheel and the four
        # caster bars are all riding the wall. The plate cannot physically get
        # closer, so this — not zero — is what the approach is closing on and
        # what the gap is measured from.
        self.contact_distance = float(contact_distance)
        # 1/s. Bounded above by seek_speed, below by approach_min_speed. Raising
        # it buys back approach time and costs nothing in peak force (measured:
        # 1.2 -> 4.0 moves the peak by 0.1 N and the time to contact by 3 s); it
        # is the MARGIN below that buys the safety, not this.
        self.approach_gain = float(approach_gain)
        # Subtracted from the filtered distance before the gap is taken: 3x the
        # 4.2 mm sigma of the plane fit. This is the one-sided error the module
        # docstring argues for — see the bias table there.
        self.approach_margin = float(approach_margin)
        # The floor. A pessimistic gap estimate would otherwise creep to a halt
        # just short of the wall and the press would never happen.
        self.approach_min_speed = float(approach_min_speed)
        # Time constant of the EMA on the sensed distance, seconds. Only the
        # SCHEDULE reads the filtered distance; the min_distance envelope still
        # reads the raw one, because a hard safety floor must not be lagged.
        self.distance_tau = float(distance_tau)
        # How long the RAW force may sit above the limit before it is a fault.
        # The filtered check keeps its spike immunity; this one catches the case
        # the field run hit, where the filter was lagging so far behind that the
        # wheel was at 40 N while the filtered force read 3.9 and never tripped.
        self.force_limit_dwell = float(force_limit_dwell)
        self.reset()

    def reset(self):
        self.state = TARE if self.tare_seconds > 0.0 else SEEK
        self.force = 0.0        # filtered, de-biased, positive = pressing in
        self.raw = 0.0          # de-biased but UNfiltered, for the limit check
        self.bias = 0.0         # the sensor's reading with nothing touching it
        self.fault = None       # set once; the caller decides what to do
        self.distance = None    # filtered sensed distance, for the schedule
        self.approach_speed = 0.0   # what the schedule last allowed, for logging
        self._seeded = False
        self._stalled = 0.0     # seconds sat at the envelope in SEEK
        self._over_limit = 0.0  # seconds the raw force has been over the limit
        self._contact_held = 0.0    # seconds the force has been over contact_force
        self._tare_samples = []
        self._tare_elapsed = 0.0
        # Whether the wheel has EVER reached the wall in this segment. Latching,
        # and deliberately so: it is what the sweep gates its travel on, and the
        # thing it must not do is follow the contact state back down. ``state``
        # legitimately drops back to SEEK over a hollow or a lip, several times
        # in a sweep, and a base that stopped and restarted on each of those
        # would scrub the wheel instead of rolling it.
        self.touched = False

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
        return self._stalled >= self.stall_seconds

    def error(self):
        """Signed force error, N. Positive means we are pressing too softly."""
        return self.target_force - self.force

    # ------------------------------------------------------------------
    def _alpha(self, dt, tau):
        """Per-cycle EMA coefficient for a time constant in seconds.

        The whole point of the 2026-09-07 change: a fixed coefficient means the
        lag follows the machine's load. ``tau <= 0`` disables the filter.
        """
        if tau <= 0.0:
            return 1.0
        return 1.0 - math.exp(-max(dt, 0.0) / tau)

    def _approach_cap(self, distance):
        """Largest approach speed the sensed gap allows, m/s.

        ``distance`` is the FILTERED reading; the margin is then subtracted from
        it, so both halves of the pessimism are applied here and the caller
        cannot forget one. Returns ``approach_min_speed`` when there is no
        distance at all — with nothing to schedule against, crawling is the only
        honest option, and the min_distance envelope is still underneath.
        """
        if distance is None:
            return self.approach_min_speed
        gap = (distance - self.approach_margin) - self.contact_distance
        return max(self.approach_min_speed, self.approach_gain * max(0.0, gap))

    # ------------------------------------------------------------------
    def update(self, raw_force, distance, dt):
        """One cycle. Returns the normal-axis velocity, positive = toward the wall.

        ``raw_force`` is the measured press force in newtons, already flipped so
        that positive means pressing into the wall, and NOT yet corrected for the
        sensor's zero — that is measured here. ``distance`` is the plate's sensed
        gap to the surface, the same number the standoff law used. ``dt`` is the
        MEASURED seconds since the last call: every time constant in here is in
        seconds and is converted against it, so a loop running slow gets the same
        behaviour in wall-clock terms rather than a quietly stretched one.
        """
        dt = max(float(dt), 0.0)

        # The filtered distance the SCHEDULE runs on, maintained in every state
        # so that it is already settled by the time TARE hands over to SEEK.
        # The envelope and the tare guard below deliberately keep reading the
        # raw value: a hard floor must not lag.
        if distance is not None:
            if self.distance is None:
                self.distance = float(distance)
            else:
                self.distance += self._alpha(dt, self.distance_tau) * (
                    float(distance) - self.distance)

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
            self._tare_elapsed += dt
            # Elapsed TIME, not a sample count — but still require a sample, so
            # that a single very late first cycle cannot tare off one reading.
            if self._tare_elapsed < self.tare_seconds or len(self._tare_samples) < 2:
                return 0.0                      # hold still while measuring
            self.bias = float(np.mean(self._tare_samples))
            self.state = SEEK
            return 0.0

        # Seed the filter rather than ramping from zero: starting at 0 N would
        # read as "no contact" for the first several cycles even when the plate
        # is already loaded, and SEEK would drive further into the wall.
        force = float(raw_force) - self.bias
        self.raw = force
        seeding = not self._seeded
        if seeding:
            # Seeded at the raw reading, which is right for the force loop and
            # wrong for contact detection: this one value has had no filtering
            # at all, so on its own it is just a sample of the noise. The
            # contact test below skips this cycle for that reason. The dwell
            # would cover it too, but only while someone leaves the dwell alone.
            self.force = force
            self._seeded = True
        else:
            self.force += self._alpha(dt, self.filter_tau) * (force - self.force)

        # Two limit checks, because one is not enough and they fail differently.
        #
        # The FILTERED one is the original: a single spike from the base thumping
        # over a floor joint cannot abort a good sweep, and a real overload --
        # which persists -- trips within a fraction of a second.
        #
        # The RAW one exists because the filter is a lag, and a lag on a safety
        # check is a hole. On the 2026-09-07 run the loop was slow enough that
        # the wheel reached 40 N while the filtered force still read under 4 and
        # the sweep sailed past its own limit. Debounced in TIME rather than in
        # cycles, so it keeps the spike immunity at 50 Hz (three samples) and
        # still fires within one cycle at 10 Hz, which is exactly when the
        # filtered check is least trustworthy.
        if self.force > self.force_limit:
            self.fault = (f"press force {self.force:.1f} N exceeded the "
                          f"{self.force_limit:.1f} N limit")
        if force > self.force_limit:
            self._over_limit += dt
            if self._over_limit >= self.force_limit_dwell and not self.fault:
                self.fault = (
                    f"press force {force:.1f} N exceeded the "
                    f"{self.force_limit:.1f} N limit for {self._over_limit * 1000:.0f} ms "
                    f"(unfiltered; the filtered force was still reading "
                    f"{self.force:.1f} N)")
        else:
            self._over_limit = 0.0

        if self.state == SEEK:
            # Held over the threshold, in TIME, and never on the seeding cycle.
            # Anything short of the dwell resets it: contact is a sustained load,
            # and noise that happens to cross once is not one.
            if self.force >= self.contact_force and not seeding:
                self._contact_held += dt
            else:
                self._contact_held = 0.0
            if self._contact_held >= self.contact_dwell:
                self.state = PRESS
                self.touched = True
                self._stalled = 0.0
        elif self.state == PRESS and self.force < self.release_force:
            # Contact lost: a hollow, a gap, the wheel riding over a lip. Go
            # back to closing the distance rather than commanding the full force
            # error, which out of contact is just "drive at the wall".
            self.state = SEEK
            self._contact_held = 0.0

        if self.state == PRESS:
            # v_max bounds the FORCE loop only. It is sized for contact — a few
            # mm/s — and applying it to the approach as well would silently cap
            # seek_speed at it, so raising the seek speed would do nothing.
            v = float(np.clip(self.gain * self.error(), -self.v_max, self.v_max))
            self.approach_speed = 0.0
        else:
            if self._contact_held > 0.0:
                # Candidate contact: the force is over the threshold but has not
                # held for the dwell yet. STOP while confirming, rather than
                # carrying on into the wall for another dwell's worth of travel.
                #
                # This is not a detail. The dwell is detection latency, and
                # detection latency times approach speed is penetration — the
                # exact quantity the schedule exists to bound. Confirming while
                # still moving put 33 N on the wheel with a 10 mm distance bias,
                # against 25 N before the dwell existed; confirming while stopped
                # costs nothing, because there is no travel to pay for it with.
                # A noise spike therefore costs a 0.15 s pause in the approach
                # and nothing else, which is a price worth paying every time.
                self.approach_speed = 0.0
            else:
                # The scheduled approach. seek_speed is a CEILING rather than the
                # commanded speed: far from the wall the schedule is slack and
                # the approach runs at seek_speed, and it takes over only over
                # the last few centimetres, which is the only stretch where it
                # matters.
                self.approach_speed = min(self.seek_speed,
                                          self._approach_cap(self.distance))
            v = self.approach_speed

        # The envelope. Approach is refused inside min_distance whatever the
        # force says; retreat is always allowed, so a press that has gone too
        # deep can still back out. Reads the RAW distance, not the filtered one:
        # this is the hard floor and it must not be a cycle behind.
        if distance is not None and distance <= self.min_distance:
            v = min(v, 0.0)
            self._stalled += dt if self.state == SEEK else 0.0
        else:
            self._stalled = 0.0

        return float(v)
