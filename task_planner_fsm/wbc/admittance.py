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

**Re-contact is not a first approach.** The schedule above is pessimistic
because on a first approach the wall's position is known only through a plane
fit with 4 mm of sigma and a possible bias. After a contact, it is known far
better: the plate was pressing on it a moment ago, and the ranges at that
moment say where it was. A wheel that comes off — the base's chassis kicks
when it starts (measured 0.7 deg of uncommanded yaw on 2026-09-21, ~12 mm at
the plate), a hollow, a lip — is then a wheel a known distance from a known
wall, and closing that at the first-approach floor is what turned a 12 mm
excursion into a 30 s crawl on the 12:50 run. So for ``recontact_memory``
seconds after a release the approach is scheduled on the gap to the
REMEMBERED wall instead, with a higher ceiling (``recontact_speed``) and its
own gain. The impact bound is the same arithmetic as before — latency times
speed times stiffness — and the numbers below assume the caster contact this
plate actually rides on, ~2 kN/m measured (see the stiffness floor in the
sweep node): 5 mm/s over a 100 ms cycle is 1 N. The memory expires because a
wall that is gone for longer than that may genuinely be a different wall.

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

The threshold and the dwell below stop the latch firing on NOISE. They do nothing
about a wall that recedes after a genuine contact, which is the same failure
arriving later, so ``touched`` now only ARMS the base: the travel speed is scaled
by a filtered ``in_contact`` (``press_travel_tau``) and closes again on its own.

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

**And contact has to be PLAUSIBLE on distance, because the dwell is sized for
noise and the sensor also sees inertia.** The 2026-09-14 field run latched
``touched`` 5.9 s after the tare with the plate at 20.8 cm — 6.5 cm short of
where the wheel can physically touch — on a filtered +5.3 N that lasted three
cycles. That is not the 0.95 N sigma the table above is built on; it is the
GPR's mass on an arm being jerked at 4-18 Hz, and a 6-sigma transient will beat
any dwell that is sized against Gaussian noise. So the latch also asks the
ranges: a force reading with the plate more than ``contact_window`` beyond
``contact_distance`` is refused outright, however long it lasts, for the same
reason the tare trusts distance over force — the ranges can say where the wheel
is, and a force sensor cannot. The cost of that false latch was the whole run:
the gate opened 16 s before real contact, so when load did arrive the base set
off at 20 mm/s during SEEK, before the force loop was in charge, and drove the
wheel from 3.6 N to 36 N in three cycles of a starved loop.

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
                 contact_distance=0.1375, approach_gain=0.3,
                 approach_margin=0.0126, approach_min_speed=0.0008,
                 distance_tau=0.15, force_limit_dwell=0.06,
                 contact_window=0.03, recontact_speed=0.005,
                 recontact_gain=2.0, recontact_memory=5.0,
                 soft_limit=15.0, retreat_v_max=0.02, soft_limit_seconds=3.0):
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
        # ...and across at least this many separate readings, whichever is the
        # stronger demand at the rate the loop is achieving. A time-only dwell
        # inverts the old cycle-counting bug rather than fixing it: at 5 Hz one
        # period is 0.2 s, so a 0.15 s dwell is satisfied by a SINGLE sample and
        # stops being a dwell at all — measured, 5 spurious latches in 200
        # approaches. Two readings cannot both be noise nearly as easily as one.
        self.contact_samples = 2
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
        # Where the plate STOPS: the range reading when the GPR face and wheel
        # are on the wall. The plate cannot physically get closer, so this —
        # not zero — is what the approach is closing on and what the gap is
        # measured from. The GPR contact point is 15.0 cm in front of the
        # sensor plane (the pendant TCP, confirmed against FK on 2026-09-17),
        # so calibrated ranges read 0.150 there. The node passes that in; the
        # 0.1375 default is what the same stop read through the uncalibrated
        # sensors on 2026-09-14, kept so the existing tests stay meaningful.
        self.contact_distance = float(contact_distance)
        # How far beyond the stop the ranges may read while a force is still
        # believed to be contact. The wheel first loads about 7 mm outside the
        # stop and the plane fit has 4.2 mm sigma, so 3 cm is over 5 sigma clear
        # of a real contact — and 3.5 cm short of the 20.8 cm the 2026-09-14
        # transient latched at. See the module docstring.
        self.contact_window = float(contact_window)
        # 1/s, and the single most important number in this file. It sets where
        # the approach STARTS to slow:
        #
        #     bind point = contact_distance + approach_margin + seek_speed / gain
        #
        # and the distance between that and the wall is all the room the loop has
        # to decelerate in. At 3.0 that room was 4.9 mm, which at 7 Hz is three
        # cycles, and the 2026-09-07 run drove through it at the full 10 mm/s and
        # put 30.5 N on the wheel.
        #
        # It was 3.0 because it was tuned against an assumed contact distance of
        # 0.13 m, which sits 12.6 mm BELOW the schedule's asymptote — so in that
        # model the plate was always already crawling when it arrived and the gain
        # made almost no difference ("1.2 -> 4.0 moves the peak by 0.1 N"). The
        # robot then measured contact at 14.1 cm, essentially AT the asymptote,
        # where the gain is the only thing that matters. Peak force with contact
        # in the right place, worst of 12 seeds:
        #
        #     gain    15 Hz   10 Hz    7 Hz    5 Hz
        #      3.0     24.0    31.4    42.0    66.2
        #      1.0      8.7    15.5    18.3    50.2
        #      0.5      6.9     8.7    13.0    12.8
        #      0.3      6.5     9.0    11.2    11.2
        #
        # 0.3 gives 35 mm of braking room and holds ~11 N down to 5 Hz, for 17 s
        # to contact from a 22 cm start against a 45 s timeout. Raising it back
        # trades that room away, and the room is the whole mechanism.
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
        # --- re-contact --------------------------------------------------
        # Ceiling on the approach while closing on a wall the press was on a
        # moment ago; 1/s gain on the gap to it; and how long that knowledge
        # is trusted after the release. See the module docstring.
        #
        # The remembered distance is where the plate sat AT TARGET FORCE, so
        # it includes the contact's compression, F_target / K_e. The wheel
        # therefore meets the wall with that much gap still showing and lands
        # at gain * F_target / K_e — and the force that builds over one cycle
        # of latency is K_e times that times dt = gain * F_target * dt, the
        # same on any surface. At 2/s and 10 Hz that is 1 N; at 5 Hz, 2 N.
        self.recontact_speed = float(recontact_speed)
        self.recontact_gain = float(recontact_gain)
        self.recontact_memory = float(recontact_memory)
        # Scales ``gain`` from outside, per cycle, without touching the tunable
        # itself: the sweep node raises it when the measured contact is softer
        # than the ~2e4 N/m the gain was sized against.
        self.gain_scale = 1.0
        # --- the soft limit ------------------------------------------------
        # Between the target and the hard limit there used to be nothing: a
        # contact the base drove from 5 N to 30 N in a second was a FAULT, and
        # the sweep retreated, failed and the FSM retried — nine failures in
        # ten on the real wall were this. Above ``soft_limit`` the loop now
        # REACTS instead: the retreat clamp opens to ``retreat_v_max`` (only
        # the approach has an impact hazard; backing off has none, and 30 N on
        # a ~2 kN/m contact is 15 mm to unload — 3 s at the approach clamp,
        # under a second at this one), and the caller stops the base and the
        # plate's rotation the same cycle (``overloaded``). The hard limit is
        # then for what the reaction cannot handle: a force still climbing
        # through it, or one that sits above the soft limit for longer than
        # ``soft_limit_seconds`` — something is holding the plate in, and
        # backing off is not working.
        self.soft_limit = float(soft_limit)
        self.retreat_v_max = float(retreat_v_max)
        self.soft_limit_seconds = float(soft_limit_seconds)
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
        self._over_soft = 0.0   # seconds the filtered force has been over the soft limit
        self._contact_held = 0.0    # seconds the force has been over contact_force
        self._contact_n = 0         # ...and how many readings in a row
        self._loaded = False        # over contact_force THIS cycle, plausible or not
        self._tare_samples = []
        self._tare_elapsed = 0.0
        # Where the wall WAS: the filtered range while the press was on it, and
        # how long ago the press came off. None until there has been a press.
        self.wall_distance = None
        self._since_release = None
        # Whether the wheel has EVER reached the wall in this segment. Latching,
        # and deliberately so: it is what ARMS the sweep's travel, and an arming
        # test must not follow the contact state back down. It carries the dwell
        # and the sample count below, which is the only test here that the
        # sensor's noise cannot talk its way past.
        #
        # It is no longer what SETS the travel speed. The sweep scales that by a
        # filtered version of ``in_contact`` instead (``press_travel_tau`` in
        # sweep_node), so a wall that recedes after a genuine contact slows the
        # base rather than being ignored for the rest of the segment. The old
        # objection to reading the live state there — that stopping and
        # restarting a loaded wheel scrubs it — is answered by the filter, not by
        # the latch.
        self.touched = False

    # ------------------------------------------------------------------
    @property
    def in_contact(self):
        return self.state == PRESS

    @property
    def loaded(self):
        """Over ``contact_force`` on the LAST update, plausible or not, latched
        or not — the earliest sign there is of the wheel meeting something.
        True throughout PRESS. The sweep uses it to stop rotating the plate
        the moment it touches, without waiting for the dwell that arms the
        base."""
        return self.state == PRESS or self._loaded

    @property
    def overloaded(self):
        """Filtered force over ``soft_limit``: back off hard, and the caller
        stops whatever is doing the loading."""
        return self.force > self.soft_limit

    @property
    def recontacting(self):
        """In SEEK, closing on a wall the press was on within ``recontact_memory``."""
        return (self.state == SEEK and self._since_release is not None
                and self._since_release < self.recontact_memory
                and self.wall_distance is not None)

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
        if self.recontacting:
            # No margin: the wall position is the press's own filtered range
            # from a moment ago, not a plane fit against unknown bias, and
            # the same filter is on both sides of the subtraction.
            gap = distance - self.wall_distance
            return min(self.recontact_speed,
                       max(self.approach_min_speed, self.recontact_gain * max(0.0, gap)))
        gap = (distance - self.approach_margin) - self.contact_distance
        return max(self.approach_min_speed, self.approach_gain * max(0.0, gap))

    # ------------------------------------------------------------------
    def update(self, raw_force, distance, dt, quiet=True):
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
            # Only a QUIET arm is worth taring against. The F/T sensor carries
            # the GPR's mass, and an arm that is moving — the sweep's opening
            # alignment can swing the plate 15 deg at 0.5 rad/s — puts inertial
            # load on it that is not bias. On 2026-09-18 the tare landed in
            # exactly that swing and read +4.2 N; every force of the run was
            # then 4 N low, contact latch and 30 N limit included. The caller
            # says when the arm is still; until then the window does not run.
            if not quiet:
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
        if self.force > self.soft_limit:
            self._over_soft += dt
            if (self.soft_limit_seconds > 0.0 and self._over_soft >= self.soft_limit_seconds
                    and not self.fault):
                self.fault = (
                    f"press force {self.force:.1f} N has been over the "
                    f"{self.soft_limit:.0f} N soft limit for {self._over_soft:.1f} s "
                    f"and backing off at {self.retreat_v_max * 1000:.0f} mm/s is not "
                    f"relieving it — something is holding the plate in")
        else:
            self._over_soft = 0.0

        if self.state == SEEK:
            # Held over the threshold, in TIME, and never on the seeding cycle.
            # Anything short of the dwell resets it: contact is a sustained load,
            # and noise that happens to cross once is not one.
            #
            # And never with the plate where the wheel cannot be touching. The
            # dwell is sized against sensor noise, and a transient from the
            # arm's own motion is neither noise nor contact — it beat the dwell
            # at 20.8 cm on 2026-09-14. The FILTERED distance, the same one the
            # schedule reads: at the 0.8 mm/s the plate arrives at, its lag is
            # a tenth of a millimetre. With no distance at all there is nothing
            # to check against and force is the only sensor left, so it is
            # believed — the sweep's own data-age strike covers that case.
            #
            # The guard is on the LATCH only. A load the ranges call impossible
            # still halts the approach below, because if the ranges are ever
            # the sensor that is wrong, that is precisely the moment to stop
            # pushing — the halt costs nothing and the alternative is driving
            # at the schedule's speed into a wall the ranges deny. What such a
            # load may not do is arm the base.
            self._loaded = self.force >= self.contact_force and not seeding
            plausible = (self.distance is None or
                         self.distance <= self.contact_distance + self.contact_window)
            if self._loaded and plausible:
                self._contact_held += dt
                self._contact_n += 1
            else:
                self._contact_held = 0.0
                self._contact_n = 0
            if self._since_release is not None:
                self._since_release += dt
            if (self._contact_held >= self.contact_dwell
                    and self._contact_n >= self.contact_samples):
                self.state = PRESS
                self.touched = True
                self._stalled = 0.0
                self._since_release = None
        elif self.state == PRESS and self.force < self.release_force:
            # Contact lost: a hollow, a gap, the wheel riding over a lip. Go
            # back to closing the distance rather than commanding the full force
            # error, which out of contact is just "drive at the wall" — but
            # closing it on the wall the press was just on, which is known.
            self.state = SEEK
            self._contact_held = 0.0
            self._contact_n = 0
            self._loaded = False
            self._since_release = 0.0
        if self.state == PRESS and self.distance is not None and force >= self.contact_force:
            # Remember where the wall is while it is UNDER LOAD — the raw force
            # says so, not the state, because the state lags the force filter
            # by a cycle or two and on the cycle the wheel is kicked off the
            # range has already jumped while the state still says PRESS. A
            # memory taken then is the excursion, not the wall. Blended over
            # a few cycles for the same reason.
            if self.wall_distance is None:
                self.wall_distance = self.distance
            else:
                self.wall_distance += self._alpha(dt, 0.3) * (self.distance - self.wall_distance)

        if self.state == PRESS:
            # v_max bounds the FORCE loop only. It is sized for contact — a few
            # mm/s — and applying it to the approach as well would silently cap
            # seek_speed at it, so raising the seek speed would do nothing.
            # Asymmetric clamp: v_max bounds the APPROACH, which is where the
            # impact hazard is; the retreat may go as fast as retreat_v_max.
            v = float(np.clip(self.gain * self.gain_scale * self.error(),
                              -self.retreat_v_max, self.v_max))
            if self.force > self.soft_limit:
                # The reaction: over the soft limit the retreat is at least
                # the approach clamp, and grows with the EXCESS to reach
                # retreat_v_max at the hard limit. Starting from v_max rather
                # than from zero is what makes it fast where it matters — on
                # a ~2 kN/m contact a 35 N shove is under the soft limit in
                # ~1 s this way, ~2 s ramped from nothing — and the gain law
                # alone (5 mm/s at 25 N, x5 on the soft contact) would take
                # a second the base does not give it.
                span = max(self.force_limit - self.soft_limit, 1.0)
                excess = min(1.0, (self.force - self.soft_limit) / span)
                reaction = self.v_max + excess * (self.retreat_v_max - self.v_max)
                v = min(v, -min(reaction, self.retreat_v_max))
            self.approach_speed = 0.0
        else:
            if self._loaded:
                # Candidate contact: the force is over the threshold but has not
                # held for the dwell yet — or the ranges say it cannot be
                # contact at all. STOP either way while it lasts, rather than
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
