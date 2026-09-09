"""Hyperspectral sweep sampling and post-processing.

Covers the two halves of the collect/process split independently, plus the
seam between them: the sampler records raw spectra and coverage counters during
a sweep, and the processing pass turns that record into reflectance without ever
touching the camera.
"""

import json
import types

import numpy as np
import pytest

from arm_control.srv import HyperspectralCommand

from task_planner_fsm.utils import hyperspectral_processing as hp
from task_planner_fsm.utils.hyperspectral_sampler import HyperspectralSampler


# ----------------------------------------------------------------------
# Doubles
# ----------------------------------------------------------------------
class _Logger:
    def info(self, msg, **kwargs):
        pass

    def warn(self, msg, **kwargs):
        pass

    def error(self, msg, **kwargs):
        pass


class _Future:
    def __init__(self):
        self._callback = None
        self._result = None
        self._done = False

    def add_done_callback(self, callback):
        self._callback = callback

    def done(self):
        return self._done

    def result(self):
        return self._result

    def settle(self, result):
        """Complete the future and run the sampler's done-callback."""
        self._result = result
        self._done = True
        if self._callback is not None:
            self._callback(self)


class _Client:
    def __init__(self):
        self.calls = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = _Future()
        self.calls.append((request, future))
        return future


class _Node:
    def __init__(self, client):
        self._client = client
        self._logger = _Logger()
        self.timers = []

    def create_client(self, *args, **kwargs):
        return self._client

    def create_timer(self, period, callback):
        timer = types.SimpleNamespace(cancel=lambda: None, callback=callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        pass

    def get_logger(self):
        return self._logger


def _spectra_response(value, ok=True, length=hp.SPECTRUM_LENGTH, status=0):
    response = HyperspectralCommand.Response()
    response.vis_ok = ok
    response.nir_ok = ok
    response.vis_spectrum = [value] * length
    response.nir_spectrum = [value] * length
    response.vis_status = status
    response.nir_status = status
    response.message = "OK" if ok else "ERROR|sensor timeout"
    return response


def _ok_response():
    """A good capture: raw counts midway between the dark and white references."""
    return _spectra_response(600)


def _mti_response():
    response = HyperspectralCommand.Response()
    response.vis_ok = True
    response.nir_ok = True
    response.message = "150000|140000"
    return response


@pytest.fixture
def rig(tmp_path):
    """A configured, calibrated sampler with a mock camera behind it."""
    client = _Client()
    node = _Node(client)
    ctx = {
        "node": node,
        "hyperspectral_enabled": True,
        "hyperspectral_output_dir": str(tmp_path),
        "hyperspectral_sample_spacing_m": 0.10,
        "hyperspectral_min_sample_period_s": 0.0,   # distance-only in tests
    }
    sampler = HyperspectralSampler("test")
    assert sampler.configure(node, ctx)
    # GET_GDS -> GET_GRF -> GET_MTI, dark current 100, white reference 1100.
    for response in (_spectra_response(100), _spectra_response(1100), _mti_response()):
        sampler.calibration_ready(ctx)
        client.calls[-1][1].settle(response)
    assert sampler.calibration_ready(ctx)
    return types.SimpleNamespace(
        sampler=sampler, ctx=ctx, node=node, client=client)


def _sweep(rig, distance_m, step_m=0.02, respond=_ok_response,
           wall_index=0, line_idx=0, seg_idx=0):
    """Drive a straight sweep along +x and answer every capture."""
    plate = [0.0, 0.0, 1.0]
    rig.sampler.start_line(
        rig.ctx, (0.0, 0.0), (distance_m, 0.0),
        pose_fn=lambda frame, timeout: tuple(plate),
        ref="map", axis=(1.0, 0.0, 0.0),
        wall_index=wall_index, line_idx=line_idx, seg_idx=seg_idx,
    )
    tick = rig.node.timers[-1].callback
    seen = 0
    for i in range(1, int(round(distance_m / step_m)) + 1):
        plate[0] = i * step_m
        tick()
        while seen < len(rig.client.calls):
            request, future = rig.client.calls[seen]
            seen += 1
            if request.command == "GSM" and not future.done() and respond is not None:
                future.settle(respond())
    return tick


# ----------------------------------------------------------------------
# Reflectance
# ----------------------------------------------------------------------
def test_reflectance_matches_the_official_formula():
    gds = np.full(hp.SPECTRUM_LENGTH, 100.0)
    grf = np.full(hp.SPECTRUM_LENGTH, 1100.0)
    values, ok, _ = hp.reflectance(np.full(hp.SPECTRUM_LENGTH, 600.0), gds, grf)
    assert ok
    # (600 - 100) / (1100 - 100)
    assert values == pytest.approx(0.5)


def test_reflectance_keeps_negatives_and_values_above_one():
    """Neither is an error: thermal-noise negatives have to reach the
    StandardScaler intact, and dry materials genuinely read above 1.0 because
    the white reference absorbs IR."""
    gds = np.full(hp.SPECTRUM_LENGTH, 100.0)
    grf = np.full(hp.SPECTRUM_LENGTH, 1100.0)
    below, ok, _ = hp.reflectance(np.full(hp.SPECTRUM_LENGTH, 50.0), gds, grf)
    assert ok and (below < 0).all()
    above, ok, _ = hp.reflectance(np.full(hp.SPECTRUM_LENGTH, 1600.0), gds, grf)
    assert ok and (above > 1.0).all()


def test_reflectance_rejects_a_white_reference_that_collapsed_onto_the_dark():
    """The 'lid on for GRF' mistake from the English requirements doc. Without
    this check it surfaces later as an underexposure warning and the operator
    goes hunting for a hardware fault."""
    gds = np.full(hp.SPECTRUM_LENGTH, 100.0)
    _, ok, reason = hp.reflectance(np.full(hp.SPECTRUM_LENGTH, 600.0), gds, gds)
    assert not ok and "white reference" in reason


# ----------------------------------------------------------------------
# Stability gate
# ----------------------------------------------------------------------
def test_stability_accepts_a_clean_spectrum():
    flat = np.full(hp.SPECTRUM_LENGTH, 0.5)
    ok, reason = hp.check_stability(flat, flat)
    assert ok and reason == ""


@pytest.mark.parametrize("vis, nir, expected", [
    (np.random.default_rng(0).normal(0.5, 0.5, hp.SPECTRUM_LENGTH),
     np.full(hp.SPECTRUM_LENGTH, 0.5), "noise"),
    (np.full(hp.SPECTRUM_LENGTH, 0.001),
     np.full(hp.SPECTRUM_LENGTH, 0.5), "underexposed"),
    (np.full(hp.SPECTRUM_LENGTH, 5.0),
     np.full(hp.SPECTRUM_LENGTH, 0.5), "saturation"),
])
def test_stability_names_the_criterion_it_failed(vis, nir, expected):
    ok, reason = hp.check_stability(vis, nir)
    assert not ok and expected in reason


def test_ml_status_strings_are_rejections_not_materials():
    """inspection_manager filters only on the ERROR prefix, so a low-confidence
    rejection ends up in its CSV as if it were a material name."""
    assert hp.is_ml_rejection("REBUTJAT_BAIXA_CONFIANCA")
    assert hp.is_ml_rejection("ERROR: model not loaded")
    assert not hp.is_ml_rejection("Cartro")


# ----------------------------------------------------------------------
# Distance triggering
# ----------------------------------------------------------------------
def test_captures_fire_on_plate_travel_not_on_ticks(rig):
    _sweep(rig, distance_m=1.0)
    rig.sampler.stop_line(rig.ctx)
    segment = rig.sampler.metrics.segments[0]
    # 1.0 m at 0.10 m spacing, with float accumulation costing the last one.
    assert segment["collection"][hp.OK] == 9
    assert segment["travel_m"] == pytest.approx(1.0)


def test_tf_jitter_with_the_robot_parked_never_fires_a_capture(rig):
    """Raw path length would accumulate jitter as phantom travel and sample a
    stationary robot forever; the signed projection onto the sweep axis makes
    perpendicular noise drop out and along-axis noise average to zero."""
    plate = [0.0, 0.0, 1.0]
    rig.sampler.start_line(
        rig.ctx, (0.0, 0.0), (1.0, 0.0),
        pose_fn=lambda frame, timeout: tuple(plate),
        ref="map", axis=(1.0, 0.0, 0.0),
        wall_index=0, line_idx=0, seg_idx=0,
    )
    tick = rig.node.timers[-1].callback
    before = len(rig.client.calls)
    rng = np.random.default_rng(1)
    for _ in range(500):
        jitter = rng.normal(0.0, 0.002, 3)
        plate[0], plate[1], plate[2] = jitter[0], jitter[1], 1.0 + jitter[2]
        tick()
    assert len(rig.client.calls) == before


def test_a_retreat_costs_at_most_one_spacing(rig):
    """Without the clamp, a 1 m Nav2 recovery would leave a 1 m dead zone with
    no samples while the sweep re-covers that ground."""
    plate = [0.0, 0.0, 1.0]
    rig.sampler.start_line(
        rig.ctx, (0.0, 0.0), (1.0, 0.0),
        pose_fn=lambda frame, timeout: tuple(plate),
        ref="map", axis=(1.0, 0.0, 0.0),
        wall_index=0, line_idx=0, seg_idx=0,
    )
    tick = rig.node.timers[-1].callback
    seen = [0]

    def drive(positions):
        """Step the plate through ``positions``, answering every capture."""
        for x in positions:
            plate[0] = x
            tick()
            while seen[0] < len(rig.client.calls):
                request, future = rig.client.calls[seen[0]]
                seen[0] += 1
                if request.command == "GSM" and not future.done():
                    future.settle(_ok_response())

    drive([i * 0.02 for i in range(1, 51)])          # advance 1.0 m
    before = len(rig.client.calls)
    drive([1.0 - i * 0.02 for i in range(1, 51)])    # retreat all the way back
    assert len(rig.client.calls) == before, "a retreat must not capture"
    drive([i * 0.02 for i in range(1, 16)])          # re-advance 0.30 m
    # One spacing of hysteresis, then sampling resumes on the fixed grid.
    assert len(rig.client.calls) > before


# ----------------------------------------------------------------------
# Failure and skip accounting
# ----------------------------------------------------------------------
def test_a_sensor_failure_is_recorded_rather_than_dropped(rig):
    _sweep(rig, distance_m=0.5, respond=lambda: _spectra_response(0, ok=False))
    rig.sampler.stop_line(rig.ctx)
    counters = rig.sampler.metrics.segments[0]["collection"]
    assert counters[hp.FAILED_SENSOR] == 4
    assert counters[hp.OK] == 0


def test_a_truncated_frame_is_recorded_as_a_length_failure(rig):
    _sweep(rig, distance_m=0.5, respond=lambda: _spectra_response(600, length=128))
    rig.sampler.stop_line(rig.ctx)
    assert rig.sampler.metrics.segments[0]["collection"][hp.FAILED_LENGTH] == 4


def test_captures_still_in_flight_are_skipped_not_queued(rig):
    """One capture at a time. A camera slower than the spacing must produce
    gaps in the record, not a growing backlog of stale requests."""
    _sweep(rig, distance_m=1.0, respond=None)   # never answer
    rig.sampler.stop_line(rig.ctx)
    segment = rig.sampler.metrics.segments[0]
    assert segment["collection"][hp.SKIPPED_BUSY] == segment["triggered"] - 1


def test_the_minimum_sample_period_rate_limits_a_fast_sweep(rig):
    rig.ctx["hyperspectral_min_sample_period_s"] = 999.0
    _sweep(rig, distance_m=1.0)
    rig.sampler.stop_line(rig.ctx)
    counters = rig.sampler.metrics.segments[0]["collection"]
    assert counters[hp.OK] == 1
    assert counters[hp.SKIPPED_RATE] == 8


def test_a_capture_landing_after_its_segment_closed_still_counts(rig):
    """The sampler lets a capture dispatched at the end of a sweep land, because
    the plate was on the wall when it fired. It must be credited to the segment
    it was taken in, not dropped and not charged to the next one."""
    _sweep(rig, distance_m=0.3, respond=None)
    pending = rig.client.calls[-1][1]
    rig.sampler.stop_line(rig.ctx)
    assert rig.sampler.metrics.segments[0]["collection"][hp.OK] == 0
    pending.settle(_spectra_response(600))
    assert rig.sampler.metrics.segments[0]["collection"][hp.OK] == 1


# ----------------------------------------------------------------------
# Collect -> process seam
# ----------------------------------------------------------------------
def test_the_recorded_sweep_processes_into_reflectance_and_metrics(rig):
    _sweep(rig, distance_m=1.0, wall_index=0)
    rig.sampler.stop_line(rig.ctx)
    _sweep(rig, distance_m=0.5, wall_index=1,
           respond=lambda: _spectra_response(0, ok=False))
    rig.sampler.stop_line(rig.ctx)
    rig.sampler.abort(rig.ctx)

    result = hp.process_session(
        rig.ctx["hyperspectral_session_dir"],
        predict_fn=lambda vis, nir: ("Guix", 0.91),
    )
    totals = result["metrics"].totals()
    assert totals[hp.OK] == 9
    assert totals[hp.FAILED_SENSOR] == 4
    assert totals[hp.ACCEPTED] == 9
    assert totals["acceptance_rate"] == 1.0

    walls = result["metrics"].walls()
    assert walls["0"][hp.ACCEPTED] == 9
    assert walls["1"]["capture_failed"] == 4
    assert walls["1"]["captured"] == 0
    assert json.load(open(result["metrics_json"]))["orphan_segments"] == 0


def test_rejected_samples_stay_in_the_report(rig):
    """Rejections are the coverage evidence. Dropping them -- what the CLI does
    -- makes a wall the camera struggled on indistinguishable from a wall
    nobody scanned."""
    _sweep(rig, distance_m=0.5, respond=lambda: _spectra_response(60000))
    rig.sampler.stop_line(rig.ctx)
    rig.sampler.abort(rig.ctx)

    result = hp.process_session(rig.ctx["hyperspectral_session_dir"])
    totals = result["metrics"].totals()
    assert totals[hp.REJECTED_STABILITY] == 4
    assert totals[hp.ACCEPTED] == 0

    import csv
    rows = list(csv.DictReader(open(result["reflectance_csv"])))
    assert len(rows) == 4
    assert all(row["Status"] == hp.REJECTED_STABILITY for row in rows)
    assert all("saturation" in row["Reason"] for row in rows)


def test_processing_is_idempotent(rig):
    """Re-running with a retrained model must replace the verdicts, not append
    a second set of counts to the same segments."""
    _sweep(rig, distance_m=1.0)
    rig.sampler.stop_line(rig.ctx)
    rig.sampler.abort(rig.ctx)
    session = rig.ctx["hyperspectral_session_dir"]

    first = hp.process_session(session, predict_fn=lambda v, n: ("Guix", 0.9))
    assert first["metrics"].totals()[hp.ACCEPTED] == 9
    second = hp.process_session(session, predict_fn=lambda v, n: ("Guix", 0.9))
    assert second["metrics"].totals()[hp.ACCEPTED] == 9
    assert second["metrics"].totals()["triggered"] == 9

    # A model that now rejects everything moves the samples, it does not add to them.
    third = hp.process_session(
        session, predict_fn=lambda v, n: ("REBUTJAT_BAIXA_CONFIANCA", 0.3))
    totals = third["metrics"].totals()
    assert totals[hp.REJECTED_ML] == 9 and totals[hp.ACCEPTED] == 0


def test_sampling_without_a_calibration_is_refused(tmp_path):
    """Spectra recorded with no GDS/GRF could never become reflectance. Skip the
    segment loudly rather than fill the record with unusable samples."""
    client = _Client()
    node = _Node(client)
    ctx = {
        "node": node,
        "hyperspectral_enabled": True,
        "hyperspectral_output_dir": str(tmp_path),
    }
    sampler = HyperspectralSampler("test")
    sampler.configure(node, ctx)
    armed = sampler.start_line(
        ctx, (0.0, 0.0), (1.0, 0.0),
        pose_fn=lambda frame, timeout: (0.0, 0.0, 1.0),
        ref="map", axis=(1.0, 0.0, 0.0),
    )
    assert armed is False
    assert node.timers == []


def test_sampling_is_off_by_default():
    """Mirrors gpr_enabled: the hardware is not on the robot for every mission."""
    assert HyperspectralSampler.enabled({}) is False
