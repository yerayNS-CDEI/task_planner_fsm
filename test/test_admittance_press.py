"""The press against a simulated stiff wall, at the rates the robot achieves.

The property under test is RATE INDEPENDENCE, and it is here because the sweep
failed on 2026-09-07 for want of it: the control loop was at 8-17 Hz instead of
50 (the machine was at load 36 on 12 cores), and a constant-speed approach turned
that into 32 N on the GPR wheel against a 30 N limit. The loop rate is not the
sweep node's to control, so the press has to be safe at whatever rate it gets.

The wall model is the one wbc/admittance.py sizes its own gain against:
``K_e ~ 2e4 N/m`` for concrete with a hard wheel, 30-50 ms of servo lag, and the
plate bottoming out at 0.13 m because the caster bars reach the wall before the
plate face does. The range noise is the 4.2 mm sigma the plane fit measures.
"""

import numpy as np
import pytest

from task_planner_fsm.wbc.admittance import SEEK, AdmittancePress

K_E = 2.0e4          # N/m, stiffness of the wall as seen through the wheel
BOTTOM = 0.13        # m, sensed range when the wheel and casters ride the wall
SERVO_LAG = 0.04     # s, the 30-50 ms the module docstring credits the arm with
SIGMA = 0.0042       # m, sigma of the six-range plane fit


SIGMA_F = 0.95       # N, sigma of the de-biased force sensor, measured on hardware


def press_against_wall(rate, d0=0.155, bias=0.0, sigma=SIGMA, seed=0,
                       horizon=90.0, force_sigma=SIGMA_F, **kwargs):
    """Run one approach-and-press. Returns (peak true force, time to contact, fault)."""
    rng = np.random.default_rng(seed)
    dt = 1.0 / rate
    press = AdmittancePress(**kwargs)
    distance = d0
    # The arm does not execute a velocity the instant it is commanded.
    pipeline = [0.0] * max(1, int(round(SERVO_LAG / dt)))
    peak, touched_at = 0.0, None

    for step in range(int(horizon / dt)):
        force = K_E * max(0.0, BOTTOM - distance)
        peak = max(peak, force)
        if touched_at is None and press.touched:
            touched_at = step * dt
        v = press.update(force + rng.normal(0.0, force_sigma),
                         distance + bias + rng.normal(0.0, sigma), dt)
        if press.fault:
            return peak, touched_at, press.fault
        pipeline.append(v)
        distance -= pipeline.pop(0) * dt
    return peak, touched_at, None


# The rates the loop actually ran at in the field, plus the 50 it asks for.
RATES = (50, 25, 17, 10, 8)


@pytest.mark.parametrize("rate", RATES)
def test_peak_force_does_not_depend_on_loop_rate(rate):
    """The whole point. A slow loop must not press harder.

    The bound is deliberately well under the 30 N limit rather than at it: the
    limit is the abort, and a press that only passes by tripping its own abort
    has not worked.
    """
    peaks = [press_against_wall(rate, seed=s)[0] for s in range(20)]
    assert max(peaks) < 15.0, (
        f"at {rate} Hz the wheel reached {max(peaks):.1f} N")


@pytest.mark.parametrize("rate", RATES)
def test_the_press_actually_reaches_the_wall(rate):
    """Safety by never touching is the silent failure this must not become.

    A press that never lands records a clean-looking GPR scan of nothing, which
    is worse than an abort because nobody notices.
    """
    for seed in range(5):
        peak, touched_at, fault = press_against_wall(rate, seed=seed)
        assert fault is None, fault
        assert touched_at is not None, f"never reached the wall at {rate} Hz"
        assert touched_at < 45.0, (
            f"contact at {touched_at:.1f}s exceeds press_contact_timeout")


def test_constant_approach_is_what_used_to_break_it():
    """Pin the regression, so the schedule cannot be quietly disabled.

    Only the SCHEDULE is neutralised here (huge gain, no margin, no distance
    filter); the force filter keeps its proper time base. That isolates the one
    change under test, and reproduces the field failure on its own — safe at the
    50 Hz the loop asks for, well over the 30 N limit at the 10 Hz it got:

        50 Hz  16.6 N        17 Hz  23.3 N        8 Hz  50.0 N
        25 Hz  17.2 N        10 Hz  40.0 N

    If this ever starts passing at 10 Hz the wall model has drifted, and every
    number quoted in wbc/admittance.py stops meaning anything.
    """
    unscheduled = dict(approach_gain=1e6, approach_margin=0.0, distance_tau=0.0)
    fast = max(press_against_wall(50, seed=s, **unscheduled)[0] for s in range(20))
    slow = max(press_against_wall(10, seed=s, **unscheduled)[0] for s in range(20))
    assert fast < 30.0, f"the old law was survivable at 50 Hz, got {fast:.1f} N"
    assert slow > 30.0, (
        f"the old law should exceed the limit at 10 Hz, got {slow:.1f} N")


@pytest.mark.parametrize("bias_mm", (-10, -5, 0, 5))
def test_survives_a_miscalibrated_distance_sensor(bias_mm):
    """A plate offset that misreads the gap must not cost much force.

    +10 mm is deliberately NOT in this list — see the test below. Reading LONG
    is the dangerous direction, because the schedule then believes there is more
    room than there is.
    """
    peak = max(press_against_wall(10, bias=bias_mm / 1000.0, seed=s)[0]
               for s in range(20))
    assert peak < 15.0, f"{bias_mm:+d} mm of bias reached {peak:.1f} N"


def test_a_badly_miscalibrated_sensor_either_holds_or_aborts():
    """+10 mm of bias is close to the limit, and the honest bound is behavioural.

    Measured at 28.4 N worst-of-20 against a 30 N abort — real margin, but not
    much of it, and it moves with the wall model. Asserting a number here would
    be pinning a coincidence. What must be true is that the press never sits
    over its own limit without faulting: it either stays under, or it aborts.
    The fix for this case is calibrating the plate offset, not tuning the gain.
    """
    for seed in range(20):
        peak, _, fault = press_against_wall(10, bias=0.010, seed=seed)
        assert peak < 30.0 or fault is not None, (
            f"seed {seed}: reached {peak:.1f} N without faulting")


def test_raw_force_limit_fires_when_the_filter_lags():
    """The hole that filtering a safety check opens.

    A step well over the limit must be caught even though the EMA is nowhere
    near it yet — this is the check that was missing when the field log went
    from +2.0 N to 32.1 N.
    """
    press = AdmittancePress(tare_seconds=0.0, force_limit=30.0,
                            force_limit_dwell=0.06)
    press.update(0.0, 0.20, 0.02)            # seed the filter out of contact
    # 50 N is chosen to sit in the gap the dwell exists for: the raw force is
    # over the limit immediately, while the EMA only reaches 22.6 N by the third
    # sample, which is where the 60 ms dwell fires.
    for _ in range(3):
        press.update(50.0, 0.14, 0.02)
    assert press.fault is not None
    assert "unfiltered" in press.fault, press.fault
    assert press.force < 30.0, (
        "the filtered force should still be under the limit — that is the point")


def test_a_single_spike_does_not_abort_a_good_sweep():
    """The immunity the dwell must not cost. One bad sample is not an overload."""
    press = AdmittancePress(tare_seconds=0.0, force_limit=30.0,
                            force_limit_dwell=0.06)
    press.update(5.0, 0.14, 0.02)
    # One 20 ms sample, a third of the dwell. Not arbitrarily large: at tau=0.1
    # the EMA passes ~18% of a step straight through, so a spike beyond ~140 N
    # trips the FILTERED check on its own and no dwell can save it. That is the
    # filter's own limit, not the dwell's, and it predates this change.
    press.update(60.0, 0.14, 0.02)
    for _ in range(10):
        press.update(5.0, 0.14, 0.02)
    assert press.fault is None


@pytest.mark.parametrize("rate", RATES)
def test_tare_takes_the_same_time_at_every_rate(rate):
    """It used to be a cycle count, so it silently stretched to 2.5 s at 10 Hz."""
    press = AdmittancePress(tare_seconds=0.5)
    dt = 1.0 / rate
    elapsed = 0.0
    while press.state != SEEK and elapsed < 5.0:
        press.update(6.6, 0.20, dt)
        elapsed += dt
    assert press.state == SEEK
    assert 0.5 <= elapsed <= 0.5 + 2 * dt, f"tare took {elapsed:.2f}s at {rate} Hz"
    assert press.bias == pytest.approx(6.6, abs=1e-6)


def test_approach_is_capped_by_seek_speed_far_from_the_wall():
    """The schedule is a ceiling near the wall, not a brake everywhere."""
    press = AdmittancePress(tare_seconds=0.0, seek_speed=0.01)
    v = press.update(0.0, 0.40, 0.02)        # 27 cm of gap
    assert v == pytest.approx(0.01)


def test_approach_crawls_but_never_stops_at_the_stop():
    """A pessimistic gap must not stall the approach short of the wall."""
    press = AdmittancePress(tare_seconds=0.0)
    for _ in range(50):
        v = press.update(0.0, BOTTOM, 0.02)  # sitting exactly at the stop
    assert v >= press.approach_min_speed > 0.0


def test_envelope_still_refuses_to_go_deeper():
    """The hard floor is unchanged and still reads the RAW distance."""
    press = AdmittancePress(tare_seconds=0.0, min_distance=0.115)
    v = press.update(0.0, 0.10, 0.02)        # inside the envelope
    assert v <= 0.0


# ---------------------------------------------------------------------------
# Contact detection. The 2026-09-07 run found the wall on noise 0.4 s after the
# tare, with the wheel 10 cm off it, and because `touched` latches the base then
# swept 1.1 m scanning air. Every other failure on that run followed from it.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("rate", RATES)
def test_noise_alone_never_latches_contact(rate):
    """Free space, nothing but sensor noise, for a whole approach's worth of it.

    The old 1.0 N threshold latches 100% of the time here at every rate.
    """
    dt = 1.0 / rate
    latched = 0
    for trial in range(200):
        rng = np.random.default_rng(trial)
        press = AdmittancePress(tare_seconds=0.0)
        for _ in range(int(45.0 / dt)):
            press.update(rng.normal(0.0, SIGMA_F), 0.20, dt)
            if press.touched:
                latched += 1
                break
    assert latched == 0, f"{latched}/200 approaches latched on noise at {rate} Hz"


def test_the_seeding_sample_cannot_latch_contact():
    """The first SEEK cycle seeds the filter at one RAW reading.

    That value has had no filtering at all, so on its own it is a sample of the
    noise. It must not be able to declare contact, even though it is far over
    the threshold.
    """
    press = AdmittancePress(tare_seconds=0.0)
    press.update(50.0, 0.20, 0.02)       # seeds the filter at 50 N
    assert press.force == pytest.approx(50.0)
    assert not press.touched


def test_the_approach_stops_while_contact_is_being_confirmed():
    """The dwell must not be paid for in penetration.

    Confirming while still approaching is just detection latency by another
    name, and latency times speed is how hard the wheel hits.
    """
    press = AdmittancePress(tare_seconds=0.0, contact_dwell=0.15)
    press.update(0.0, 0.20, 0.02)                    # seed, far out
    assert press.update(0.0, 0.20, 0.02) > 0.0       # approaching normally
    # 50 N, not 10: the threshold is on the FILTERED force, and at tau=0.1 one
    # 10 N sample only moves it to 1.8 N — under the 3 N threshold, so it would
    # not halt anything. The filter's lag is part of the confirmation time.
    v = press.update(50.0, 0.20, 0.02)
    assert v == 0.0, "the approach must halt while the dwell runs"
    assert not press.touched, "one sample is not contact"


def test_contact_is_confirmed_when_the_load_persists():
    """The other half: a real load must still be recognised, and promptly."""
    press = AdmittancePress(tare_seconds=0.0, contact_dwell=0.15)
    press.update(0.0, 0.20, 0.02)
    held = 0.0
    while not press.touched and held < 1.0:
        press.update(50.0, 0.20, 0.02)
        held += 0.02
    assert press.touched
    # The dwell plus the filter's own lag in reaching the threshold. Bounded so
    # that a much slower confirmation — which is penetration on a real wall —
    # fails here rather than in the field.
    assert held <= 0.25, f"took {held:.2f}s to confirm a 50 N load"


def test_a_noise_spike_only_costs_a_pause():
    """A spike halts the approach; it must not stop it for good."""
    press = AdmittancePress(tare_seconds=0.0)
    press.update(0.0, 0.20, 0.02)
    assert press.update(50.0, 0.20, 0.02) == 0.0     # spike: approach halts
    for _ in range(10):
        v = press.update(0.0, 0.20, 0.02)            # noise passes
    assert v > 0.0, "the approach never resumed after a spike"
    assert not press.touched
