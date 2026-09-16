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
        # No GPR at all, so no wall stretch is drillable under the project rule;
        # this test is about the classify -> cluster chain, screened separately.
        "pokeye_require_gpr_coverage": False,
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


# ----------------------------------------------------------------------
# The GPR NO_DRILL constraint, through both states
# ----------------------------------------------------------------------
def _scan_entry(key, line, x_m=None):
    """One processed GPR scan, with a hyperbola at ``x_m`` along it if given."""
    from task_planner_fsm.sensors import gpr, no_drill

    detections = ([{"id": "H001", "type": "hyperbola",
                    "position": {"x_m": x_m, "x_cm": x_m * 100},
                    "depth": {"depth_cm": 5.0},
                    "robustness": {"confidence_mean": 0.9}}] if x_m is not None else [])
    constraints = no_drill.constraints_from_result(
        {"hyperbola_detected": bool(detections),
         "n_valid_detections": len(detections), "detections": detections},
        to_map=gpr.map_transform(line), source_key=key)
    return {"key": key, "line": line, "associated": line is not None,
            "no_drill": constraints}


def _write_gpr_summary(tmp_path, entries):
    from task_planner_fsm.sensors import gpr

    out_dir = tmp_path / "processed" / "session_20260101_000000" / "gpr"
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / gpr.SUMMARY_FILENAME, "w") as handle:
        json.dump({"entries": entries}, handle)


def _scanned_line(wall=2):
    return {"key": "w02_l00_s00", "wall_index": wall,
            "seg_start": [5.0, 3.0, 1.0], "seg_end": [7.0, 3.0, 1.0]}


def _gpr_summary_with_a_hyperbola(tmp_path, x_m=0.6, wall=2, located=True):
    """A processed GPR record placing one hyperbola at (5.6, 3.0, 1.0) in map.

    ``located=False`` adds a second, orphan export whose hyperbolae cannot be
    placed, on top of a line that was scanned properly -- the only situation
    where the unplaceable case is still interesting, since a wall with no
    scanned line at all is already undrillable.
    """
    line = _scanned_line(wall)
    entries = [_scan_entry("w02_l00_s00", line, x_m if located else None)]
    if not located:
        entries.append(_scan_entry("orphan", None, x_m))
    _write_gpr_summary(tmp_path, entries)


def _decision_ctx(tmp_path, node, **extra):
    ctx = {"node": node, "sim": False, "sensor_data_dir": str(tmp_path),
           "sensor_session_id": "20260101_000000", "current_wall_index": 2}
    ctx.update(extra)
    return ctx


def _hsi_double(x0=5.55, n=6, wall=2):
    """Flagged samples in a patch, as the classifier would have left them."""
    return {"confidence_threshold": 0.8, "samples": [
        {"seq": i, "wall_index": wall, "line_idx": 0, "seg_idx": 0, "frame": "map",
         "pose": [x0 + 0.02 * i, 3.0, 1.0], "pose_map": [x0 + 0.02 * i, 3.0, 1.0],
         "status": "low_confidence", "detected": False, "material": None,
         "confidence": 0.4, "reason": "below threshold"} for i in range(n)]}


def test_the_decision_phase_drops_a_target_sitting_on_a_hyperbola(tmp_path):
    """HSI asks for a hole where GPR saw a reflector; no hole is requested, and
    the wall is not sent to POKEYE at all because it was its only target."""
    _gpr_summary_with_a_hyperbola(tmp_path)
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = _decision_ctx(tmp_path, node)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)

    assert ctx["data_processed"] is True
    assert ctx["drilling_required"] is False
    assert ctx["pokeye_targets"] == []
    [blocked] = ctx["pokeye_blocked_targets"]
    assert blocked["blocked_by"]["source_detection_id"] == "H001"
    assert len(ctx["pokeye_no_drill_zones"]) == 1
    assert ctx["pokeye_no_drill_zones"][0]["position"] == [5.6, 3.0, 1.0]
    assert state.check_transition(ctx) == "ArmFolding"
    assert any("NO_DRILL" in msg for level, msg in node.logger.lines if level == "warn")


def test_a_hyperbola_elsewhere_on_the_wall_does_not_block_the_target(tmp_path):
    _gpr_summary_with_a_hyperbola(tmp_path, x_m=1.8)         # map x = 6.8, far away
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = _decision_ctx(tmp_path, node)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)

    assert ctx["drilling_required"] is True
    assert len(ctx["pokeye_targets"]) == 1
    assert ctx["pokeye_blocked_targets"] == []
    assert len(ctx["pokeye_no_drill_zones"]) == 1
    assert state.check_transition(ctx) == "SendDataToPokeye"


def test_the_exclusion_radius_is_the_projects_to_set(tmp_path):
    """The placeholder blocks it; a tolerance the project narrows does not."""
    _gpr_summary_with_a_hyperbola(tmp_path, x_m=0.70)        # map x = 5.70, 0.10 m away
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")

    ctx = _decision_ctx(tmp_path, node)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)
    assert ctx["pokeye_targets"] == []
    assert any("placeholder" in msg for level, msg in node.logger.lines if level == "warn")

    ctx = _decision_ctx(tmp_path, node, pokeye_no_drill_tolerance_m=0.05)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)
    assert len(ctx["pokeye_targets"]) == 1
    assert ctx["pokeye_no_drill_stats"]["tolerance_m"] == 0.05


def test_an_unplaceable_hyperbola_is_reported_and_can_veto_the_wall(tmp_path):
    """The line was scanned, so the target is covered; a second export carries
    hyperbolae we cannot place anywhere."""
    _gpr_summary_with_a_hyperbola(tmp_path, located=False)
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")

    ctx = _decision_ctx(tmp_path, node)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)
    assert len(ctx["pokeye_targets"]) == 1               # reported, not enforced
    assert ctx["pokeye_no_drill_stats"]["n_unlocated"] == 1
    assert any("could not be placed" in msg for level, msg in node.logger.lines if level == "warn")

    ctx = _decision_ctx(tmp_path, node, pokeye_no_drill_block_on_unlocated=True)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)
    assert ctx["pokeye_targets"] == []
    assert ctx["pokeye_blocked_targets"][0]["blocked_by"]["reason"] == \
        "GPR_NO_DRILL_POSITION_NOT_LOCATED"


def test_the_request_carries_the_zones_pokeye_must_respect_on_its_own_drills(tmp_path):
    """The targets were screened already; the zones travel anyway, because a
    RANDOM drill POKEYE chooses itself has to avoid the same hyperbolae."""
    node = _Node()
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    ctx["pokeye_no_drill_zones"] = [{
        "zone_id": "w02_l00_s00_H001", "source_key": "w02_l00_s00",
        "source_detection_id": "H001", "reason": "GPR_HYPERBOLA_DETECTED",
        "instruction": "NO_DRILL", "frame_id": "map", "position": [6.8, 3.0, 1.0],
        "radius_m": 0.15, "wall_index": 2, "depth_cm_approx": 5.0}]
    ctx["pokeye_no_drill_stats"] = {"tolerance_m": 0.15, "n_unlocated": 1}
    ctx["pokeye_blocked_targets"] = [{"target_id": "w02_c09"}]
    state.on_enter(ctx)

    with open(ctx["pokeye_request_json"]) as handle:
        payload = json.load(handle)
    constraints = payload["drilling_constraints"]
    assert constraints["n_no_drill_zones"] == 1
    assert constraints["coordinate_frame"] == "map"
    assert constraints["targets_already_screened"] is True
    assert constraints["exclusion_tolerance_m"] == 0.15
    assert constraints["exclusion_tolerance_source"] == "fsm_placeholder_pending_project_approval"
    assert constraints["n_unlocated"] == 1
    assert payload["blocked_targets"][0]["target_id"] == "w02_c09"

    state.run(ctx)
    [(request, _)] = node.client.calls
    assert list(request.no_drill_ids) == ["w02_l00_s00_H001"]
    assert (request.no_drill_positions[0].x, request.no_drill_positions[0].y,
            request.no_drill_positions[0].z) == (6.8, 3.0, 1.0)
    assert request.no_drill_radii[0] == pytest.approx(0.15)
    assert request.n_no_drill_unlocated == 1
    # Still two targets: the constraint is not a filter POKEYE re-applies here.
    assert len(request.positions) == 2


def test_a_request_with_no_hyperbolae_carries_an_empty_constraint_block(tmp_path):
    node = _Node()
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    state.on_enter(ctx)
    state.run(ctx)
    [(request, _)] = node.client.calls
    assert list(request.no_drill_ids) == [] and request.n_no_drill_unlocated == 0
    with open(ctx["pokeye_request_json"]) as handle:
        assert json.load(handle)["drilling_constraints"]["n_no_drill_zones"] == 0


def test_a_wall_no_gpr_line_scanned_yields_no_targets(tmp_path):
    """The project rule: a place nobody has looked behind is not drillable,
    however sure HSI is that something is wrong with it."""
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = _decision_ctx(tmp_path, node)          # no gpr_summary.json at all
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)

    assert ctx["data_processed"] is True
    assert ctx["drilling_required"] is False
    assert ctx["pokeye_targets"] == []
    assert ctx["pokeye_scanned_lines"] == []
    assert ctx["pokeye_blocked_targets"][0]["blocked_by"]["reason"] == \
        "NOT_ON_A_SCANNED_GPR_LINE"
    assert any("nothing on it may be drilled" in msg
               for level, msg in node.logger.lines if level == "warn")
    assert state.check_transition(ctx) == "ArmFolding"


def test_a_scanned_line_makes_its_own_stretch_drillable(tmp_path):
    """The same samples, once the line under them has been scanned and read."""
    _write_gpr_summary(tmp_path, [_scan_entry("w02_l00_s00", _scanned_line())])
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = _decision_ctx(tmp_path, node)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)

    assert ctx["drilling_required"] is True
    assert len(ctx["pokeye_targets"]) == 1
    assert [ln["line_id"] for ln in ctx["pokeye_scanned_lines"]] == ["w02_l00_s00"]
    assert ctx["pokeye_no_drill_zones"] == []          # clean B-scan, nothing forbidden


def test_a_scan_that_was_swept_but_never_analysed_does_not_count_as_scanned(tmp_path):
    """The export arrived and the line is known, but the hyperbola pipeline
    produced nothing readable — so the wall behind it is still unknown."""
    entry = _scan_entry("w02_l00_s00", _scanned_line())
    entry["no_drill"]["constraint_valid"] = False
    _write_gpr_summary(tmp_path, [entry])
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = _decision_ctx(tmp_path, node)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)

    assert ctx["pokeye_targets"] == []
    assert ctx["pokeye_scanned_lines"] == []
    assert ctx["pokeye_no_drill_stats"]["n_not_analysed"] == 1


def test_a_target_off_the_scanned_line_is_refused_even_on_a_scanned_wall(tmp_path):
    """The line was swept at z = 1.0; these samples sit a metre above it."""
    _write_gpr_summary(tmp_path, [_scan_entry("w02_l00_s00", _scanned_line())])
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = _decision_ctx(tmp_path, node)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    for sample in state._hsi["samples"]:
        sample["pose_map"] = [sample["pose_map"][0], 3.0, 2.0]
    state._phase = "decision"
    state.run(ctx)

    assert ctx["pokeye_targets"] == []
    assert ctx["pokeye_blocked_targets"][0]["blocked_by"]["reason"] == \
        "NOT_ON_A_SCANNED_GPR_LINE"
    assert len(ctx["pokeye_scanned_lines"]) == 1      # the wall was scanned, just not there


def test_bring_up_can_opt_out_of_the_coverage_rule(tmp_path):
    node = _Node()
    state = SensorDataProcessing("SensorDataProcessing")
    ctx = _decision_ctx(tmp_path, node, pokeye_require_gpr_coverage=False)
    state.on_enter(ctx)
    state._hsi = _hsi_double()
    state._phase = "decision"
    state.run(ctx)

    assert len(ctx["pokeye_targets"]) == 1
    assert any("pokeye_require_gpr_coverage is off" in msg
               for level, msg in node.logger.lines if level == "warn")


def test_the_request_carries_the_region_pokeye_may_drill_at_random(tmp_path):
    node = _Node()
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    ctx["pokeye_scanned_lines"] = [{
        "line_id": "w02_l00_s00", "wall_index": 2, "frame_id": "map",
        "seg_start": [5.0, 3.0, 1.0], "seg_end": [7.0, 3.0, 1.0], "n_hyperbolae": 0}]
    ctx["pokeye_no_drill_stats"] = {"tolerance_m": 0.15, "line_tolerance_m": 0.25,
                                    "n_unlocated": 0}
    state.on_enter(ctx)

    with open(ctx["pokeye_request_json"]) as handle:
        region = json.load(handle)["drillable_region"]
    assert region["enforced"] is True
    assert region["n_scanned_lines"] == 1
    assert region["line_tolerance_m"] == 0.25
    assert region["scanned_lines"][0]["line_id"] == "w02_l00_s00"

    state.run(ctx)
    [(request, _)] = node.client.calls
    assert list(request.scanned_line_ids) == ["w02_l00_s00"]
    assert (request.scanned_line_starts[0].x, request.scanned_line_starts[0].z) == (5.0, 1.0)
    assert (request.scanned_line_ends[0].x, request.scanned_line_ends[0].z) == (7.0, 1.0)
    assert request.scanned_line_tolerance == pytest.approx(0.25)


def test_a_request_with_no_scanned_lines_tells_pokeye_not_to_drill_at_random(tmp_path):
    node = _Node()
    state = SendDataToPokeye("SendDataToPokeye")
    ctx = _pokeye_ctx(tmp_path, node)
    state.on_enter(ctx)
    state.run(ctx)
    [(request, _)] = node.client.calls
    assert list(request.scanned_line_ids) == []
    with open(ctx["pokeye_request_json"]) as handle:
        assert json.load(handle)["drillable_region"]["n_scanned_lines"] == 0
