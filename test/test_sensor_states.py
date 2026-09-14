"""The two FSM states around the sensor pipelines: SensorDataProcessing driving
the real classifier off the tick, and SendDataToPokeye handing the targets to
the (fake) POKEYE service."""

import importlib
import json
import time

import numpy as np
import pytest

from task_planner_fsm.sensors import paths
from task_planner_fsm.states.send_data_to_pokeye import SendDataToPokeye
from task_planner_fsm.states.sensor_data_processing import SensorDataProcessing
from task_planner_fsm.utils import hyperspectral_processing as hp


# ----------------------------------------------------------------------
# Doubles
# ----------------------------------------------------------------------
class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg):
        self.lines.append((level, msg))

    def info(self, msg, **kwargs):
        self._log("info", msg)

    def warn(self, msg, **kwargs):
        self._log("warn", msg)

    def error(self, msg, **kwargs):
        self._log("error", msg)

    def debug(self, msg, **kwargs):
        self._log("debug", msg)


class _Future:
    def __init__(self):
        self._result = None
        self._done = False

    def done(self):
        return self._done

    def result(self):
        return self._result

    def settle(self, result):
        self._result = result
        self._done = True


class _Client:
    def __init__(self, ready=True, srv_name="/send_data_to_pokeye"):
        self.calls = []
        self.ready = ready
        self.srv_name = srv_name

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = _Future()
        self.calls.append((request, future))
        return future

    def remove_pending_request(self, future):
        pass


class _Node:
    def __init__(self, client=None):
        self.client = client or _Client()
        self.logger = _Logger()

    def create_client(self, srv_type, name):
        self.client.srv_type = srv_type
        return self.client

    def get_logger(self):
        return self.logger


def _has(module):
    try:
        importlib.import_module(module)
    except ImportError:
        return False
    return True


def _synthetic_session(data_dir, n_per_line=40, lines=2, bad=(0.5, 0.7)):
    """A raw session with smooth spectra everywhere except a noisy patch."""
    ctx = {"sensor_data_dir": str(data_dir), "sensor_session_id": "20260101_000000"}
    session = hp.new_session_dir(ctx)
    rng = np.random.default_rng(1)
    gds = np.full(hp.SPECTRUM_LENGTH, 100.0)
    grf = np.full(hp.SPECTRUM_LENGTH, 1100.0)
    hp.save_calibration(session, gds, gds, grf, grf, {})
    recorder = hp.RawRecorder(session).open()
    metrics = hp.SweepMetrics()
    wl_v, wl_n = hp.vis_wavelengths(), hp.nir_wavelengths()
    for line in range(lines):
        metrics.begin_segment(2, line, 0, 0.02)
        for i in range(n_per_line):
            x = 0.02 * i
            r_v = 0.55 + 0.1 * np.sin(wl_v / 80.0)
            r_n = 0.7 - 0.0002 * (wl_n - 1000.0)
            if bad[0] <= x <= bad[1]:
                r_v = r_v + rng.normal(0, 0.3, hp.SPECTRUM_LENGTH)
                r_n = r_n + rng.normal(0, 0.3, hp.SPECTRUM_LENGTH)
            vis = (gds + r_v * (grf - gds)).clip(0, 65535).astype(int)
            nir = (gds + r_n * (grf - gds)).clip(0, 65535).astype(int)
            metrics.record_trigger()
            metrics.record_collection(hp.OK, 2, line, 0)
            recorder.write(hp.OK, 2, line, 0, i, travel_m=x,
                           pose=[x, 0.0, 1.0 + 0.4 * line], frame="arm_base",
                           vis=vis, nir=nir, pose_map=[5.0 + x, 3.0, 1.0 + 0.4 * line])
        metrics.end_segment(travel_m=0.02 * n_per_line)
    recorder.close()
    metrics.save(session)
    return session


def _tick_until(state, ctx, phase, limit_s=60.0):
    deadline = time.monotonic() + limit_s
    while state._phase != phase:
        state.run(ctx)
        if time.monotonic() > deadline:
            raise AssertionError(f"still in {state._phase!r} after {limit_s} s")
        time.sleep(0.01)


# ----------------------------------------------------------------------
# SensorDataProcessing with the real classifier
# ----------------------------------------------------------------------
@pytest.mark.skipif(not (_has("xgboost") and paths.hsi_model_path().is_file()),
                    reason="needs xgboost and models/hsi/classifier.joblib")
def test_processing_state_classifies_decides_and_clusters_off_the_tick(tmp_path):
    session = _synthetic_session(tmp_path)
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = {
        "node": node, "sim": False,
        "hyperspectral_session_dir": session,
        "hyperspectral_batch_size": 50,
        "sensor_data_dir": str(tmp_path),
        "current_wall_index": 2,
        "gpr_processing_enabled": False,
    }
    state.on_enter(ctx)
    _tick_until(state, ctx, "hsi_classify")

    # The classifier runs in a thread: the first tick starts it and returns
    # immediately, later ticks poll. Whatever it costs, no single run() pays it.
    t0 = time.monotonic()
    state.run(ctx)
    assert time.monotonic() - t0 < 0.5
    assert state._job is not None and state._phase == "hsi_classify"
    _tick_until(state, ctx, "gpr")

    samples = ctx["hsi_samples"]
    assert len(samples) == 80
    assert {s["status"] for s in samples} <= {"detected", "low_confidence", "quality_rejected"}
    assert sum(s["status"] == "quality_rejected" for s in samples) >= 10
    assert samples[0]["pose_map"] == [5.0, 3.0, 1.0]

    _tick_until(state, ctx, "done")
    assert ctx["data_processed"] is True
    assert ctx["drilling_required"] is True
    targets = ctx["pokeye_targets"]
    # One noisy patch per line, 40 cm apart vertically: one target each.
    assert len(targets) == 2
    assert all(t["wall_index"] == 2 for t in targets)
    assert all(t["reason"] == "HSI_QUALITY_REJECTED" for t in targets)
    assert all(5.5 <= t["position"][0] <= 5.7 for t in targets)
    assert state.check_transition(ctx) == "SendDataToPokeye"

    processed = tmp_path / "processed" / "session_20260101_000000"
    assert (processed / "hsi" / "samples.csv").is_file()
    assert (processed / "pokeye" / "targets.json").is_file()
    assert ctx["sensor_results_dir"] == str(processed)


def test_processing_state_skips_classification_when_the_model_is_missing(tmp_path):
    session = _synthetic_session(tmp_path, n_per_line=5, lines=1, bad=(9, 9))
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = {"node": node, "hyperspectral_session_dir": session,
           "sensor_data_dir": str(tmp_path), "hsi_model_path": str(tmp_path / "no.joblib"),
           "gpr_processing_enabled": False, "current_wall_index": 2}
    state.on_enter(ctx)
    _tick_until(state, ctx, "done")
    assert ctx["drilling_required"] is False
    assert any("classifier not found" in msg for level, msg in node.logger.lines if level == "error")


def test_gpr_phase_waits_for_exports_then_gives_up(tmp_path, monkeypatch):
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = {"node": node, "sensor_data_dir": str(tmp_path), "gpr_wait_timeout_s": 0.05}
    state.on_enter(ctx)
    state.run(ctx)                      # no session -> gpr
    assert state._phase == "gpr"
    state.run(ctx)
    assert state._phase == "gpr"        # waiting
    time.sleep(0.06)
    state.run(ctx)
    assert state._phase == "decision"
    assert any("no new GPR exports" in msg for _, msg in node.logger.lines)


# ----------------------------------------------------------------------
# SendDataToPokeye
# ----------------------------------------------------------------------
def _targets():
    return [
        {"target_id": "w02_c01", "wall_index": 2, "frame_id": "map",
         "position": [5.6, 3.0, 1.0], "n_samples": 11, "reason": "HSI_QUALITY_REJECTED",
         "reasons": {"HSI_QUALITY_REJECTED": 11}, "requested_action": "MATERIAL_IDENTIFICATION",
         "line_idx": 0, "seg_idx": 0, "sample_seqs": list(range(1, 12)),
         "hsi_evidence": {"mean_confidence": None, "statuses": {"quality_rejected": 11}}},
        {"target_id": "w02_c02", "wall_index": 2, "frame_id": "map",
         "position": [5.6, 3.0, 1.4], "n_samples": 4, "reason": "HSI_LOW_CONFIDENCE",
         "reasons": {"HSI_LOW_CONFIDENCE": 4}, "requested_action": "MATERIAL_IDENTIFICATION",
         "line_idx": 1, "seg_idx": 0, "sample_seqs": [50, 51, 52, 53],
         "hsi_evidence": {"mean_confidence": 0.61, "statuses": {"low_confidence": 4}}},
    ]


def _pokeye_ctx(tmp_path, node):
    return {"node": node, "sensor_data_dir": str(tmp_path), "sensor_session_id": "s1",
            "current_wall_index": 2, "pokeye_targets": _targets(),
            "hsi_result_json": "/x/hsi_result.json"}


def test_send_data_to_pokeye_builds_the_request_and_waits_for_the_ack(tmp_path):
    from arm_control.srv import SendPokeyeTargets

    node = _Node()
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    state.on_enter(ctx)

    request_json = tmp_path / "processed" / "session_s1" / "pokeye" / "pokeye_request.json"
    assert request_json.is_file()
    with open(request_json) as handle:
        payload = json.load(handle)
    assert payload["n_targets"] == 2 and payload["wall_index"] == 2
    assert payload["targets"][0]["target_id"] == "w02_c01"
    assert payload["sources"]["hsi_result_json"] == "/x/hsi_result.json"

    state.run(ctx)
    assert node.client.srv_type is SendPokeyeTargets
    [(request, future)] = node.client.calls
    assert request.session_id == "s1" and request.wall_index == 2 and request.frame_id == "map"
    assert list(request.target_ids) == ["w02_c01", "w02_c02"]
    assert list(request.reasons) == ["HSI_QUALITY_REJECTED", "HSI_LOW_CONFIDENCE"]
    assert list(request.sample_counts) == [11, 4]
    assert (request.positions[1].x, request.positions[1].y, request.positions[1].z) == (5.6, 3.0, 1.4)
    assert request.request_json_path == str(request_json)
    assert state.check_transition(ctx) is None       # not until Pokeye answers

    state.run(ctx)                                    # still pending
    assert state.check_transition(ctx) is None
    response = SendPokeyeTargets.Response()
    response.success = True
    response.accepted_count = 2
    response.message = "ok"
    future.settle(response)
    state.run(ctx)
    assert ctx["data_sent"] is True and ctx["pokeye_accepted_count"] == 2
    assert state.check_transition(ctx) == "ArmFolding"


def test_send_data_to_pokeye_fails_when_pokeye_refuses(tmp_path):
    from arm_control.srv import SendPokeyeTargets

    node = _Node()
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    state.on_enter(ctx)
    state.run(ctx)
    [(_, future)] = node.client.calls
    response = SendPokeyeTargets.Response()
    response.success = False
    response.message = "drill busy"
    future.settle(response)
    state.run(ctx)
    assert ctx["error_triggered"] is True
    assert "drill busy" in ctx["error_reason"]
    assert state.check_transition(ctx) == "Error"


def test_send_data_to_pokeye_times_out_on_a_missing_service(tmp_path):
    node = _Node(_Client(ready=False))
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    ctx["pokeye_service_timeout_s"] = 0.05
    state.on_enter(ctx)
    state.run(ctx)
    assert node.client.calls == []
    assert not ctx.get("error_triggered")
    time.sleep(0.06)
    state.run(ctx)
    assert ctx["error_triggered"] is True
    assert "not available" in ctx["error_reason"]


def test_send_data_to_pokeye_with_no_targets_moves_on(tmp_path):
    node = _Node()
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    ctx["pokeye_targets"] = []
    state.on_enter(ctx)
    state.run(ctx)
    assert node.client.calls == []
    assert state.check_transition(ctx) == "ArmFolding"
