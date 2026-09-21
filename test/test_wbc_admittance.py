"""The press loop on its own, against an analytic wall.

No ROS and no robot: ``AdmittancePress`` is given a force and a distance each
cycle and returns a velocity, so the contact state machine and the force loop can
be checked directly — including the thing that matters most and is hardest to see
on hardware, which is whether the loop settles or rings.

**Every time constant in the press is in SECONDS and ``update`` takes the
measured period.** That is the 2026-09-07 rework: a fixed EMA coefficient and
cycle counts meant the filter lag and the tare duration followed the machine's
load. So tests say ``filter_tau=NO_FILTER`` rather than ``filter_alpha=1.0``, and
pass ``DT`` to every update. There is deliberately no default for ``dt`` — a
caller that forgets it should fail loudly rather than quietly assume 50 Hz.
"""

import math

import numpy as np
import pytest

from task_planner_fsm.wbc.admittance import PRESS, SEEK, TARE, AdmittancePress

# Concrete with a hard wheel. The press only has to travel a quarter of a
# millimetre to make 5 N, which is exactly why the gain has to be small.
WALL_STIFFNESS = 2.0e4      # N/m
DT = 0.02                   # 50 Hz, the sweep's control period

# tau = 0 disables the EMA, which is what the old ``filter_alpha=1.0`` meant.
NO_FILTER = 0.0
# ...and the time constant that reproduces the old fixed alpha of 0.2 at 50 Hz,
# for the tests that are about the filter rather than about the loop under it.
FAST_FILTER = -DT / math.log(1.0 - 0.2)


def _tare_seconds(cycles):
    """A tare that completes on exactly ``cycles`` updates of ``DT``.

    Half a period short of the round number, because ``_tare_elapsed`` is a
    running sum of floats: ten additions of 0.02 make 0.19999999999999998, and a
    test asserting the state on a particular cycle should not turn on that.
    """
    return (cycles - 0.5) * DT


def _press(**kw):
    """A press with the tare already done, unless a test asks for one.

    Three fixture-shaped defaults. The tare holds the axis still and returns zero,
    which is right in a real run and only noise in a test aimed at the force loop
    (the tare has its own tests below).

    The other two are about the APPROACH SCHEDULE rather than the force loop.
    ``contact_distance`` is where the plate physically STOPS, and it has to match
    ``_run_against_wall``'s ``wheel``: on the robot it is ~13 cm of GPR body and
    caster bars standing off the plate face, while in this fixture the wheel
    protrudes 3 cm and nothing else touches. ``approach_margin`` is zero because
    this wall is analytic — the margin exists to absorb 4.2 mm of plane-fit noise
    and there is none here. Left at their real defaults the schedule would spend
    every test crawling the last 1.3 cm at the 0.8 mm/s floor.
    """
    kw.setdefault("tare_seconds", 0.0)
    kw.setdefault("contact_distance", 0.03)
    kw.setdefault("approach_margin", 0.0)
    return AdmittancePress(**kw)


def _into_contact(press, force=5.0, distance=0.03, dt=DT):
    """Drive the press into PRESS, returning the number of cycles it took.

    Contact is deliberately NOT a single threshold crossing: the force must be
    held over ``contact_dwell`` AND across ``contact_samples`` readings, and the
    seeding cycle never counts. So no test can reach PRESS in one update, and
    every test that wants to start "in contact" has to get there through here.
    """
    for cycle in range(1, 200):
        press.update(force, distance, dt)
        if press.state == PRESS:
            return cycle
    raise AssertionError(
        f"never reached contact: {press.force:.1f} N in state {press.state}")


def test_it_seeks_the_wall_while_nothing_is_touching():
    press = _press(seek_speed=0.01)
    v = press.update(raw_force=0.0, distance=0.20, dt=DT)
    assert press.state == SEEK
    assert v == pytest.approx(0.01), "positive is toward the wall"


def test_contact_switches_it_from_distance_to_force():
    press = _press(filter_tau=NO_FILTER)
    press.update(0.0, 0.20, DT)
    assert press.state == SEEK
    _into_contact(press)
    assert press.in_contact


def test_contact_needs_the_load_to_last_rather_than_just_to_cross():
    """The second field run's failure, in miniature.

    The de-biased sensor measures sigma 0.95 N and swings +/-2.5 N with nothing
    touching, and an approach gives that noise several hundred independent tries
    at any threshold. So a crossing is not contact: the load has to persist, and
    across more than one reading. One spike must leave the press exactly where it
    was — including the counter, or a spike every dwell would accumulate.
    """
    press = _press(filter_tau=NO_FILTER, contact_dwell=0.15)
    press.update(0.0, 0.20, DT)                  # seed the filter clear of the wall
    for _ in range(3):
        press.update(9.0, 0.03, DT)              # a burst, well over the threshold
        press.update(0.0, 0.03, DT)              # ...that does not last
        assert press.state == SEEK, "a crossing that does not persist is noise"
        assert not press.touched

    assert _into_contact(press) > 1, "and contact can never be one update"


def test_a_load_with_the_wheel_nowhere_near_the_wall_is_not_contact():
    """The 2026-09-14 field run's failure, in miniature.

    The dwell is sized against sensor noise. What beat it on the robot was not
    noise: a +5.3 N transient from the GPR's mass on a jerked arm, held for
    three cycles, with the plate at 20.8 cm and the wheel 6.5 cm from anything
    it could touch. The ranges knew that; the force sensor could not. So a
    sustained load outside ``contact_window`` of the stop is refused however
    long it lasts, and it must leave no trace in the dwell counter either — a
    transient every dwell must not accumulate into a latch.

    It still HALTS the approach while it lasts. If the ranges are ever the
    sensor that is wrong, a load with the plate "far away" is exactly the
    moment to stop pushing; the halt is free, and what the guard refuses is
    only the arming of the base.
    """
    press = _press(filter_tau=NO_FILTER, contact_distance=0.03,
                   contact_window=0.03, distance_tau=NO_FILTER)
    press.update(0.0, 0.20, DT)                  # seed the filter clear of the wall
    for _ in range(50):                          # a whole second of "contact"
        v = press.update(9.0, 0.10, DT)          # ...with the plate 7 cm out
        assert v == 0.0, "an impossible load still halts the approach"
    assert press.state == SEEK, "a load the ranges say is impossible is not contact"
    assert not press.touched, "and it must not arm the base's travel"
    assert press._contact_held == 0.0, "nor leave the dwell part-way to a latch"

    # The same load with the ranges agreeing is contact, as before.
    assert _into_contact(press, force=9.0, distance=0.05) > 1


def test_the_distance_guard_is_a_window_not_a_point():
    """The wheel loads ~7 mm outside the stop and the plane fit has 4.2 mm of
    sigma, so contact has to be believed a little outside ``contact_distance``
    — the guard is a window above the stop, and the edge of it is inclusive of
    a real, slightly-long reading."""
    press = _press(filter_tau=NO_FILTER, contact_distance=0.03,
                   contact_window=0.03, distance_tau=NO_FILTER)
    press.update(0.0, 0.20, DT)
    assert _into_contact(press, force=9.0, distance=0.059) > 1, \
        "inside the window: contact"
    press = _press(filter_tau=NO_FILTER, contact_distance=0.03,
                   contact_window=0.03, distance_tau=NO_FILTER)
    press.update(0.0, 0.20, DT)
    with pytest.raises(AssertionError):
        _into_contact(press, force=9.0, distance=0.061)  # just outside: never


def test_without_ranges_force_is_the_only_sensor_and_is_believed():
    """No distance at all means nothing to check against. Refusing contact
    there would make a press with a dead range topic seek forever; the sweep's
    data-age strike is what covers that, not this guard."""
    press = _press(filter_tau=NO_FILTER, contact_window=0.03)
    press.update(0.0, None, DT)
    assert _into_contact(press, force=9.0, distance=None) > 1


def test_it_pushes_harder_when_light_and_backs_off_when_heavy():
    press = _press(target_force=5.0, gain=1e-4, filter_tau=NO_FILTER)
    _into_contact(press)
    light = press.update(2.0, 0.03, DT)          # 3 N short
    heavy = press.update(9.0, 0.03, DT)          # 4 N over
    assert light > 0.0, "under the target, press further in"
    assert heavy < 0.0, "over the target, ease out"


def test_losing_contact_does_not_immediately_flap_back():
    """One threshold would change state several times a second on a real wall."""
    press = _press(contact_force=3.0, release_force=1.5, filter_tau=NO_FILTER)
    _into_contact(press)
    press.update(2.0, 0.03, DT)                  # below contact, above release
    assert press.state == PRESS, "the gap between the thresholds is the point"
    press.update(1.0, 0.03, DT)
    assert press.state == SEEK


@pytest.mark.xfail(reason="release debounce was never implemented: PRESS -> SEEK "
                          "is immediate on force < release_force, and the only "
                          "thing smoothing it is the force EMA. See the "
                          "docstring — the motivation is a real field "
                          "observation, so this records intent, not a defect "
                          "introduced here.",
                   strict=True)
def test_a_brief_dip_below_release_does_not_drop_the_contact():
    """Hysteresis handles a force sitting near ONE threshold. It does nothing
    when the force sits BETWEEN the two, which is where the first hardware run
    put it: a press reaching ~1 N against thresholds of 0.5 and 1.0, flapping
    several times a sweep. Each flip is a 50x step in commanded normal velocity,
    felt at the wheel as a knock — and the GPR needs the wheel rolling, not
    bouncing. So a loss has to persist before it counts.
    """
    press = _press(contact_force=3.0, release_force=1.5, filter_tau=NO_FILTER,
                   release_dwell=5 * DT)
    _into_contact(press)

    for _ in range(4):
        press.update(0.1, 0.03, DT)              # four cycles of nothing
        assert press.in_contact, "a dip this short is noise, not a hollow"
    press.update(5.0, 0.03, DT)                  # ... and the wall comes back

    for _ in range(4):
        press.update(0.1, 0.03, DT)
    assert press.in_contact, "the counter must reset when contact returns"
    press.update(0.1, 0.03, DT)
    assert press.state == SEEK, "a loss that persists is a real one"


def test_contact_ever_made_is_remembered():
    """The sweep ARMS its travel on this: a segment that never touched is a
    recording of air, and calling it swept means nobody knows to come back.

    It latches, and stays latched through a loss. What the sweep does about a
    loss after that is the travel authority's job, not this flag's — see
    ``press_travel_tau`` in sweep_node.
    """
    press = _press(filter_tau=NO_FILTER)
    assert not press.touched
    press.update(0.2, 0.20, DT)
    assert not press.touched
    _into_contact(press)
    assert press.touched
    for _ in range(20):
        press.update(0.0, 0.20, DT)              # and stays true after a loss
    assert press.touched and not press.in_contact


def test_an_overload_is_reported_rather_than_absorbed():
    press = _press(force_limit=25.0, filter_tau=NO_FILTER)
    press.update(5.0, 0.03, DT)
    assert press.fault is None
    press.update(40.0, 0.03, DT)
    assert press.fault is not None and "25.0 N limit" in press.fault


def test_a_single_spike_does_not_trip_the_limit():
    """The filtered check exists so the base thumping over a floor joint cannot
    end a good sweep, and a real overload — which persists — trips within a
    fraction of a second."""
    press = _press(force_limit=25.0, filter_tau=FAST_FILTER)
    for _ in range(20):
        press.update(5.0, 0.03, DT)
    press.update(60.0, 0.03, DT)                 # one bad frame
    assert press.fault is None
    for _ in range(20):
        press.update(60.0, 0.03, DT)             # ... but it keeps coming
    assert press.fault is not None


def test_a_lagging_filter_cannot_hide_a_real_overload():
    """The hole that spike immunity opens, and the reason for the RAW check.

    On the 2026-09-07 run the loop was slow enough that the wheel reached 40 N
    while the filtered force still read under 4 and the sweep sailed past its own
    limit. Debounced in TIME rather than in cycles, so the same 60 ms means three
    samples at 50 Hz and fires within one cycle at 10 — which is exactly when the
    filtered check is least trustworthy.
    """
    press = _press(force_limit=25.0, filter_tau=1.0, force_limit_dwell=0.06)
    for _ in range(5):
        press.update(0.0, 0.03, DT)
    for _ in range(4):                           # 80 ms over the limit, raw
        press.update(60.0, 0.03, DT)

    assert press.force < 25.0, "the filter is still well behind, as on the robot"
    assert press.fault is not None, "and the raw check has to catch it anyway"
    assert "unfiltered" in press.fault


def test_the_distance_envelope_refuses_to_approach_but_allows_retreat():
    """A wrong force reading must not be able to walk the arm into the wall."""
    press = _press(min_distance=0.005, target_force=5.0, gain=1e-4,
                   filter_tau=NO_FILTER, seek_speed=0.01)
    _into_contact(press)
    # Way over the target and hard against the envelope: backing out is allowed.
    assert press.update(30.0, 0.004, DT) < 0.0, "retreat must always be possible"
    # ...and with the wheel unloaded, the envelope refuses the approach that the
    # distance schedule would otherwise authorise.
    assert press.update(0.0, 0.004, DT) <= 0.0, "no approach inside the envelope"


def test_sitting_at_the_envelope_without_contact_is_a_stall():
    """Either the plate is not where the ranges say, or the sensor is not
    reporting. Both mean the press is not happening."""
    press = _press(min_distance=0.005, stall_seconds=0.20)
    for _ in range(9):                           # 0.18 s
        press.update(0.0, 0.004, DT)
    assert not press.stalled
    for _ in range(2):                           # 0.22 s: clear of the float sum
        press.update(0.0, 0.004, DT)
    assert press.stalled


def test_contact_that_arrives_clears_the_stall_count():
    press = _press(min_distance=0.005, stall_seconds=0.20, filter_tau=NO_FILTER)
    for _ in range(9):
        press.update(0.0, 0.004, DT)
    press.update(5.0, 0.03, DT)
    assert not press.stalled


def test_the_filter_is_seeded_rather_than_ramped_from_zero():
    """Starting at 0 N would read as 'no contact' for the first cycles even with
    the plate already loaded, and SEEK would drive further into the wall."""
    press = _press(filter_tau=FAST_FILTER)
    press.update(8.0, 0.03, DT)
    assert press.force == pytest.approx(8.0), "seeded at the reading, not at zero"
    # ...but the seeding value has had no filtering at all, so on its own it is
    # just one sample of the noise, and it must not be allowed to latch contact.
    assert press.state == SEEK, "one unfiltered sample is not a contact"
    _into_contact(press, force=8.0)


# ---------------------------------------------------------------------------
# The approach schedule
# ---------------------------------------------------------------------------
def test_the_approach_slows_as_the_gap_closes():
    """Closing at a fixed speed makes the peak contact force a function of the
    loop rate, because the wheel keeps approaching until the loop NOTICES. The
    schedule makes the penetration during that detection latency go to zero as
    the gap does, whatever the latency is."""
    press = _press(seek_speed=0.01, approach_gain=0.3)
    far = press.update(0.0, 0.20, DT)
    press.reset()                                # no tare here, so this is SEEK
    for _ in range(40):                          # let the distance EMA settle
        near = press.update(0.0, 0.035, DT)

    assert far == pytest.approx(0.01), "far from the wall, seek_speed is the cap"
    assert near < far / 5.0, f"close in, the schedule should bind: {near:.4f} m/s"
    assert near > 0.0, "but it must never stall short of the wall"


def test_a_candidate_contact_stops_the_approach_while_it_is_confirmed():
    """The dwell is detection latency, and latency times approach speed is
    penetration — the exact quantity the schedule exists to bound. Confirming
    while still moving put 33 N on the wheel; confirming while stopped costs
    nothing, because there is no travel to pay for it with."""
    press = _press(filter_tau=NO_FILTER, seek_speed=0.01)
    press.update(0.0, 0.20, DT)
    assert press.approach_speed > 0.0

    press.update(9.0, 0.20, DT)                  # over the threshold, not yet held
    assert press.state == SEEK, "not contact until it has lasted"
    assert press.approach_speed == 0.0, "and the approach stops while it is judged"


# ---------------------------------------------------------------------------
# The tare
# ---------------------------------------------------------------------------
def test_it_measures_the_sensor_zero_before_believing_any_of_it():
    """An untared TCP sensor reads several newtons of payload offset against a
    5 N target — the difference between leaning on the wall and never reaching
    it, and the second one is silent."""
    press = _press(tare_seconds=_tare_seconds(10), tare_min_distance=0.05,
                   filter_tau=NO_FILTER)
    for cycle in range(10):
        assert press.update(raw_force=3.0, distance=0.20, dt=DT) == 0.0, "holds still"
        # The last sample is the one that completes it, so the state has already
        # moved on by the time that call returns.
        assert press.state == (SEEK if cycle == 9 else TARE)

    assert press.bias == pytest.approx(3.0)
    assert press.state == SEEK
    # The same 3 N now reads as nothing, which is what it is.
    press.update(3.0, 0.20, DT)
    assert press.force == pytest.approx(0.0)
    assert press.state == SEEK, "the offset must not look like contact"


def test_the_tare_is_timed_in_seconds_not_in_cycles():
    """What the 2026-09-07 rework was for. The tare used to be a sample count
    sized at 50 Hz, so at the 10 Hz the robot achieved it quietly took 2.5 s
    instead of 0.5 — and every other constant in the file drifted with it."""
    slow = _press(tare_seconds=0.5, tare_min_distance=0.05)
    for _ in range(5):                           # 5 cycles at 10 Hz is 0.5 s
        slow.update(3.0, 0.20, 0.1)
    assert slow.state == SEEK, "five slow cycles are a whole tare"

    fast = _press(tare_seconds=0.5, tare_min_distance=0.05)
    for _ in range(5):                           # 5 cycles at 50 Hz is 0.1 s
        fast.update(3.0, 0.20, DT)
    assert fast.state == TARE, "five fast ones are not"


def test_a_real_press_is_measured_from_the_tared_zero():
    press = _press(tare_seconds=_tare_seconds(5), filter_tau=NO_FILTER)
    for _ in range(5):
        press.update(3.0, 0.20, DT)
    _into_contact(press, force=8.0)              # 3 N of offset + 5 N of press
    assert press.force == pytest.approx(5.0)
    assert press.in_contact


def test_taring_against_something_it_may_be_touching_is_a_fault():
    """Taring in contact folds the press into the zero, and the loop then pushes
    until the TRUE force is target-plus-contact. Distance decides, because force
    is precisely what is not yet trustworthy.

    A fault rather than a warning: the failure it prevents is silent, and the
    sweep has no way to notice it later.
    """
    press = _press(tare_seconds=0.2, tare_min_distance=0.05, seek_speed=0.01)
    v = press.update(raw_force=0.0, distance=0.01, dt=DT)
    assert press.fault is not None
    assert "free-space margin" in press.fault
    assert v == 0.0, "and it commands nothing while faulted"


@pytest.mark.xfail(reason="backing off from a too-close tare was never "
                          "implemented — the module faults instead, and has "
                          "since its first commit. The docstring records a real "
                          "field regression, so this is a design question for "
                          "the FSM's approach tolerance, not a defect here.",
                   strict=True)
def test_it_backs_away_rather_than_taring_against_something_it_may_be_touching():
    """Backing off rather than failing, because the sweep does not choose where
    it starts: the FSM accepts its approach anywhere within a 15 cm tolerance, so
    a plate beginning inside this margin is ordinary. Making it fatal regressed a
    working sweep — a legitimate low approach ended the segment on cycle one.
    """
    press = _press(tare_seconds=0.2, tare_min_distance=0.05, seek_speed=0.01)
    v = press.update(raw_force=0.0, distance=0.01, dt=DT)
    assert press.fault is None, "too close to tare is recoverable, not fatal"
    assert press.backing_off
    assert v < 0.0, "negative is away from the wall"
    assert press.state == TARE


@pytest.mark.xfail(reason="depends on the same unimplemented back-off: with no "
                          "back-off there is no partial tare to discard.",
                   strict=True)
def test_backing_off_far_enough_lets_the_tare_run():
    press = _press(tare_seconds=_tare_seconds(5), tare_min_distance=0.05,
                   seek_speed=0.01, filter_tau=NO_FILTER)
    gap = 0.01
    # 4 cm of back-off at 0.01 m/s is 200 cycles on its own, plus the tare.
    for _ in range(400):
        gap -= press.update(3.0, gap, DT) * DT   # positive v closes on the wall
        if press.state != TARE:
            break

    assert not press.backing_off
    assert press.state == SEEK
    assert gap >= 0.05
    assert press.bias == pytest.approx(3.0), "and it tared where it was safe to"


@pytest.mark.xfail(reason="same unimplemented back-off: the drift is a fault, so "
                          "the tare never resumes to have partial samples.",
                   strict=True)
def test_a_partial_tare_is_discarded_if_the_plate_drifts_too_close():
    """Half a tare taken at a safe distance and half taken in contact would
    average to a zero that is neither."""
    press = _press(tare_seconds=_tare_seconds(10), tare_min_distance=0.05)
    for _ in range(5):
        press.update(2.0, 0.20, DT)
    press.update(2.0, 0.01, DT)                  # drifted inside the margin
    assert press.state == TARE and press.backing_off
    for _ in range(9):
        press.update(8.0, 0.20, DT)
    press.update(8.0, 0.20, DT)
    assert press.bias == pytest.approx(8.0), "the early samples must be dropped"


def test_the_tare_can_be_skipped_for_a_sensor_something_else_has_zeroed():
    press = _press(tare_seconds=0.0)
    assert press.state == SEEK
    assert press.bias == 0.0


# ---------------------------------------------------------------------------
# Closed loop against a compliant wall
# ---------------------------------------------------------------------------
def _run_against_wall(press, gap=0.05, wheel=0.03, stiffness=WALL_STIFFNESS,
                      cycles=1500, dt=DT):
    """Integrate the press against a spring wall, returning the force history.

    ``gap`` is the plate's distance to the surface and ``wheel`` how far the GPR
    wheel protrudes from the plate, so contact begins at ``gap == wheel`` and the
    force is the spring load on the overlap. ``wheel`` is therefore the same
    quantity as ``contact_distance``, which ``_press`` defaults to match.
    """
    history = []
    for _ in range(cycles):
        force = max(0.0, stiffness * (wheel - gap))
        v = press.update(force, gap, dt)
        gap -= v * dt                    # positive v closes on the wall
        history.append(force)
    return np.array(history)


def test_the_loop_settles_on_the_target_force_without_ringing():
    """The one thing that cannot be checked on paper. A velocity source against a
    stiff environment is only stable while gain * stiffness stays under what the
    servo lag allows; too high and it rings, then chatters on the surface."""
    press = _press(target_force=5.0, gain=5.0e-5, v_max=0.005,
                   seek_speed=0.01, filter_tau=FAST_FILTER)
    force = _run_against_wall(press)

    settled = force[-200:]
    assert press.in_contact
    assert settled.mean() == pytest.approx(5.0, abs=0.5), "should hold the target"
    assert settled.ptp() < 0.5, f"settled band is {settled.ptp():.2f} N — ringing"
    # It approached from below and never slammed: no excursion far past target.
    assert force.max() < 2.0 * press.target_force


def test_the_scheduled_approach_does_not_slam_whatever_the_loop_rate():
    """The 2026-09-07 failure, and the reason the schedule exists at all.

    At a constant seek speed the peak force is ``K_e * v * detection_latency``,
    so it grows as the loop slows: driven at a fixed 10 mm/s this model reaches
    tens of newtons at 10 Hz. Scheduled on the gap it stays flat, because the
    speed at the moment of contact goes to zero with the gap regardless of how
    long the loop takes to notice.
    """
    peaks = {}
    for rate in (50.0, 10.0, 5.0):
        press = _press(target_force=5.0, gain=5.0e-5, v_max=0.005,
                       seek_speed=0.01, filter_tau=FAST_FILTER)
        dt = 1.0 / rate
        peaks[rate] = _run_against_wall(
            press, cycles=int(60.0 * rate), dt=dt).max()

    assert max(peaks.values()) < 15.0, f"peak force by rate: {peaks}"


# The three demonstrations below run a deliberately wild gain against the
# wall to SHOW ringing. The press now caps its retreat at the compression it
# believes is there to unload (stiffness_hint), which is a deadbeat step for
# a wall at exactly that stiffness and would hide the ringing they exist to
# show — so they pass a hint of 1 N/m, which never binds.
def test_a_gain_that_is_too_high_for_the_wall_is_visibly_unstable():
    """The guard rail for anyone tempted to turn the gain up on hardware.

    With the filter out of the way the loop is a plain discrete integrator
    against a spring, ``e[n+1] = e[n] (1 - K_e k dt)``, so it rings once
    ``k > 2 / (K_e dt)`` — here 5e-3 m/s/N. Measured: 5e-3 gives a 2 N limit
    cycle and 1e-2 gives 17 N and loses contact altogether.

    The default is 5e-5, a hundred times under that, and deliberately so. This
    model has no servo lag in it, so the real boundary is LOWER than the one
    measured here, not higher.
    """
    tame = _press(stiffness_hint=1.0, target_force=5.0, gain=5.0e-5, v_max=0.05,
                  filter_tau=NO_FILTER, force_limit=1e9)
    wild = _press(stiffness_hint=1.0, target_force=5.0, gain=1.0e-2, v_max=0.05,
                  filter_tau=NO_FILTER, force_limit=1e9)

    tame_force = _run_against_wall(tame)[-300:]
    wild_force = _run_against_wall(wild)[-300:]

    assert tame_force.ptp() < 0.5
    assert wild_force.ptp() > 10.0, f"expected ringing, got {wild_force.ptp():.2f} N"


def test_the_force_filter_is_part_of_what_keeps_the_loop_stable():
    """Not just noise rejection — it buys real stability margin.

    Same gain, same wall: unfiltered it rings, filtered it settles. Worth knowing
    before anyone shortens filter_tau to make the press feel more responsive.
    """
    unfiltered = _run_against_wall(
        _press(stiffness_hint=1.0, target_force=5.0, gain=1.0e-2, v_max=0.05,
               filter_tau=NO_FILTER, force_limit=1e9))[-300:]
    filtered = _run_against_wall(
        _press(stiffness_hint=1.0, target_force=5.0, gain=1.0e-2, v_max=0.05,
               filter_tau=FAST_FILTER, force_limit=1e9))[-300:]

    assert unfiltered.ptp() > 10.0
    assert filtered.ptp() < 0.5


def test_the_velocity_clamp_bounds_a_gain_that_is_far_too_high():
    """Defence in depth. The clamp does not make a bad gain stable — it makes it
    BOUNDED, which is the property that matters when a tuning is wrong.

    At 200x the default gain with the filter out, the loop saturates v_max every
    cycle and settles into a limit cycle one clamped step wide: 4 to 6 N about a
    5 N target, instead of the 17 N excursion it reaches unclamped. Contact is
    never lost, so the GPR keeps its wheel on the wall the whole time.
    """
    # Symmetric clamps here: the shipped retreat clamp is wider than the
    # approach one (retreat_v_max, for the soft-limit reaction), and this test
    # is about what ONE clamped step is worth, not about that asymmetry.
    wild = _run_against_wall(
        _press(stiffness_hint=1.0, target_force=5.0, gain=1.0e-2, v_max=0.05, retreat_v_max=0.05,
               filter_tau=NO_FILTER, force_limit=1e9, soft_limit=1e9))[-300:]
    clamped = _run_against_wall(
        _press(stiffness_hint=1.0, target_force=5.0, gain=1.0e-2, v_max=0.005, retreat_v_max=0.005,
               filter_tau=NO_FILTER, force_limit=1e9, soft_limit=1e9))[-300:]

    # One clamped step against this wall is v_max * dt * K_e = 2 N of swing.
    assert clamped.ptp() == pytest.approx(2.0, abs=0.2)
    assert clamped.ptp() < wild.ptp() / 5.0
    assert clamped.min() > 0.0, "a bounded wobble must not break contact"


def test_a_biased_sensor_still_settles_on_the_target_end_to_end():
    """Tare, seek, press — with the sensor reading 4 N of payload offset the
    whole time. Without the tare this lands 4 N light, which is the silent
    failure: a GPR recording a clean-looking scan of nothing."""
    press = _press(target_force=5.0, gain=5.0e-5, v_max=0.005,
                   seek_speed=0.01, filter_tau=FAST_FILTER,
                   tare_seconds=_tare_seconds(25), tare_min_distance=0.05)
    gap, wheel, offset = 0.08, 0.03, 4.0
    true_force = []
    for _ in range(2000):
        f = max(0.0, WALL_STIFFNESS * (wheel - gap))
        gap -= press.update(f + offset, gap, DT) * DT
        true_force.append(f)

    assert press.bias == pytest.approx(offset, abs=0.1)
    assert np.array(true_force[-200:]).mean() == pytest.approx(5.0, abs=0.5)


def test_the_press_recovers_when_the_wall_falls_away():
    """A hollow or a lip: contact is lost, and the loop must close the gap again
    rather than sit there commanding a force error into thin air."""
    press = _press(target_force=5.0, gain=5.0e-5, v_max=0.005,
                   seek_speed=0.01, filter_tau=FAST_FILTER)
    _run_against_wall(press)
    assert press.in_contact

    # The surface steps back 2 cm. Nothing is touching any more.
    gap, wheel, recovered = 0.05, 0.03, False
    for _ in range(3000):
        force = max(0.0, WALL_STIFFNESS * (wheel - gap))
        gap -= press.update(force, gap, DT) * DT
        if press.in_contact and force > 4.0:
            recovered = True
            break
    assert recovered, "the press should find the surface again"


def _kicked_off_the_wall(press, kick=0.012, stiffness=2.0e3, dt=0.1, cycles=600):
    """After a settled press, the wall steps back ``kick`` metres — the base's
    chassis yawing under the plate, as measured on 2026-09-21 — and the loop
    runs on at ``dt`` against a ``stiffness`` wall. Returns (seconds to be
    back in contact, peak force on the way back, force history)."""
    gap, wheel = press.distance, 0.03
    gap += kick
    forces, back, released = [], None, False
    for cycle in range(cycles):
        force = max(0.0, stiffness * (wheel - gap))
        gap -= press.update(force, gap, dt) * dt
        forces.append(force)
        # The state lags the force filter by a cycle or two, so wait for the
        # release before looking for the re-contact.
        released = released or not press.in_contact
        if back is None and released and press.in_contact:
            back = cycle * dt
    return back, max(forces), np.array(forces)


def test_a_wheel_kicked_off_a_known_wall_is_back_on_it_in_seconds_not_half_a_minute():
    """The 12:50 run on 2026-09-21: 12 mm off, and the first-approach floor of
    0.8 mm/s took 30 s to win it back. After a contact the wall's position is
    known, so the re-contact closes on it at up to recontact_speed. At the
    caster contact's ~2 kN/m and a 10 Hz loop the landing stays gentle."""
    fast = _press(target_force=5.0, gain=5.0e-5, v_max=0.005, seek_speed=0.01,
                  approach_min_speed=0.0008, filter_tau=0.1,
                  recontact_speed=0.005, recontact_gain=2.0, recontact_memory=5.0)
    _run_against_wall(fast, stiffness=2.0e3, dt=0.1, cycles=400)
    assert fast.in_contact and fast.wall_distance is not None
    back, peak, _ = _kicked_off_the_wall(fast)
    # Fast down to recontact_margin from the remembered wall, then the
    # approach floor for the last few millimetres: ~6 s, against 30.
    assert back is not None and back < 7.0, f"re-contact took {back} s"
    assert peak < 12.0, f"re-contact landed at {peak:.1f} N"
    # Regaining the last newton of TARGET from there is the press law's job,
    # and at 5e-5 m/s/N on a 2 kN/m contact that is the slow part (0.5 mm at
    # 0.05 mm/s): the sweep node scales the gain to the measured stiffness
    # for exactly that reason — see gain_scale and test_wbc_sweep_node.

    slow = _press(target_force=5.0, gain=5.0e-5, v_max=0.005, seek_speed=0.01,
                  approach_min_speed=0.0008, filter_tau=0.1,
                  recontact_memory=0.0)          # the old behaviour: no memory
    _run_against_wall(slow, stiffness=2.0e3, dt=0.1, cycles=400)
    back_slow, _, _ = _kicked_off_the_wall(slow)
    assert back_slow is None or back_slow > 1.5 * back, (
        f"without the memory it should be the crawl: {back_slow} s vs {back} s")


def test_the_remembered_wall_is_forgotten_after_the_memory_expires():
    """A wall that has been gone for longer than recontact_memory may be a
    different wall, and the pessimistic first-approach schedule is the right
    one for a wall whose position is not known."""
    press = _press(target_force=5.0, gain=5.0e-5, v_max=0.005, seek_speed=0.01,
                   approach_min_speed=0.0008, recontact_speed=0.005,
                   recontact_memory=1.0)
    _run_against_wall(press, stiffness=2.0e3, dt=0.1, cycles=400)
    # Off the wall, and held off: the surface has gone away for good.
    gap = press.distance + 0.05
    speeds = [press.update(0.0, gap, 0.1) for _ in range(30)]
    assert press.recontacting is False
    # During the memory the approach ran at the re-contact ceiling; after it,
    # at whatever the first-approach schedule allows for a 5 cm gap (capped by
    # seek_speed), which is a different number — the point is that it changed.
    assert max(speeds[:8]) == pytest.approx(0.005, abs=1e-9)
    assert speeds[-1] != pytest.approx(0.005, abs=1e-9)


def test_over_the_soft_limit_the_press_backs_off_fast_and_does_not_fault():
    """The reaction layer. A contact the base drives from 5 to 25 N is not a
    fault any more: the retreat clamp opens to retreat_v_max and the force
    comes down in a fraction of a second, with the wheel still on the wall."""
    press = _press(target_force=5.0, gain=2.5e-4, v_max=0.005, retreat_v_max=0.02,
                   filter_tau=0.1, soft_limit=15.0, force_limit=45.0)
    _run_against_wall(press, stiffness=2.0e3, dt=0.1, cycles=400)
    assert press.in_contact
    # The wall lurches 15 mm toward the plate in one cycle: 30 N more — the
    # 2026-09-21 16:58 overload, which was a fault at the 30 N limit.
    gap, wheel = press.distance - 0.015, 0.03
    forces, retreat = [], []
    for _ in range(30):
        force = max(0.0, 2.0e3 * (wheel - gap))
        v = press.update(force, gap, 0.1)
        gap -= v * 0.1
        forces.append(force); retreat.append(-v)
    assert max(forces) > 30.0
    assert max(retreat) > 0.008, "the retreat should open well past the approach clamp"
    assert press.fault is None
    assert forces[-1] < 15.0 and press.in_contact, "relieved, and still touching"
    assert max(forces[1:]) - min(forces[1:]) > 10.0


def test_a_force_that_will_not_come_down_is_a_fault_after_the_soft_dwell():
    """Backing off at 20 mm/s and the force not moving means something is
    holding the plate in — a snagged caster, a base still pushing. The hard
    limit is for a force still climbing; this is for one that is stuck."""
    press = _press(target_force=5.0, filter_tau=0.0, soft_limit=15.0,
                   soft_limit_seconds=0.95, force_limit=45.0)
    _into_contact(press)
    for _ in range(9):
        press.update(20.0, 0.03, 0.1)
    assert press.fault is None
    press.update(20.0, 0.03, 0.1)
    assert press.fault and "soft limit" in press.fault


def test_re_contact_on_a_stiff_corner_with_a_slow_loop_does_not_slam():
    """The 2026-09-21 18:39 bounce: a corner at ~25 kN/m (a caster bottomed
    out — not the 2 kN/m of a caster on its bar), the loop at 5 Hz, and the
    re-contact landing at 5 mm/s made 39 N, 89 times. The last millimetres
    to the remembered wall are now taken at the approach floor, and the
    retreat cannot unload past target in one cycle at the believed
    stiffness, so the wheel is neither slammed nor thrown off."""
    press = _press(target_force=5.0, gain=5.0e-5, v_max=0.005, seek_speed=0.01,
                   approach_min_speed=0.0008, filter_tau=0.1,
                   recontact_speed=0.005, recontact_gain=2.0, recontact_memory=10.0,
                   recontact_margin=0.003, stiffness_hint=2.0e4,
                   soft_limit=15.0, force_limit=45.0)
    _run_against_wall(press, stiffness=2.5e4, dt=0.2, cycles=300)
    assert press.in_contact
    back, peak, forces = _kicked_off_the_wall(press, kick=0.012, stiffness=2.5e4, dt=0.2, cycles=200)
    assert back is not None, "it should find the wall again"
    assert peak < 15.0, f"landed at {peak:.1f} N on the stiff corner"
    # And having landed, it STAYS: no bounce back off the wall.
    after = forces[int(back / 0.2) + 5:]
    assert min(after) > 1.5, "the wheel came off again after landing"
    assert press.fault is None
