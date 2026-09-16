"""GPR post-processing plumbing: the line manifest ScanWall writes, matching
exported scans to lines, and putting a scan-local x back on the wall. The
pipelines themselves are not run here (they need torch/obspy and a real SEGY)."""

import importlib.util
import json
import os
import time
from pathlib import Path

import pytest

from task_planner_fsm.sensors import gpr, manifest, paths


# ----------------------------------------------------------------------
# paths / session
# ----------------------------------------------------------------------
def test_session_id_is_adopted_from_the_hyperspectral_session(tmp_path):
    ctx = {"sensor_data_dir": str(tmp_path),
           "hyperspectral_session_dir": "/somewhere/session_20260914_153000"}
    assert paths.session_id(ctx) == "20260914_153000"
    assert ctx["sensor_session_id"] == "20260914_153000"
    assert paths.processed_dir(ctx) == tmp_path / "processed" / "session_20260914_153000"
    assert paths.gpr_manifest_path(ctx) == (
        tmp_path / "raw" / "gpr" / "session_20260914_153000" / "gpr_lines.jsonl")
    assert paths.gpr_incoming_dir(ctx) == tmp_path / "raw" / "gpr" / "incoming"


def test_session_id_is_minted_once_and_cached(tmp_path):
    ctx = {"sensor_data_dir": str(tmp_path)}
    first = paths.session_id(ctx)
    assert paths.session_id(ctx) == first
    assert len(first) == len("20260914_153000")


def test_model_paths_follow_the_overrides(tmp_path):
    assert paths.gpr_weights_path({}) == paths.PACKAGE_ROOT / "models" / "gpr" / "best.pt"
    assert paths.hsi_model_path({"sensor_models_dir": str(tmp_path)}) == tmp_path / "hsi" / "classifier.joblib"
    assert paths.gpr_weights_path({"gpr_weights_path": "~/w.pt"}) == Path(os.path.expanduser("~/w.pt"))


# ----------------------------------------------------------------------
# manifest
# ----------------------------------------------------------------------
def test_manifest_rows_round_trip_with_a_stable_key(tmp_path):
    ctx = {"sensor_data_dir": str(tmp_path), "sensor_session_id": "s1"}
    path = manifest.append_gpr_line(ctx, {
        "wall_index": 2, "line_idx": 1, "seg_idx": 0,
        "seg_start": [0.0, 0.0, 1.0], "seg_end": [2.0, 0.0, 1.0],
        "t_start_epoch": 100.0, "t_stop_epoch": 130.0,
    })
    manifest.append_gpr_line(ctx, {"wall_index": 2, "line_idx": 1, "seg_idx": 1})
    rows = manifest.read_gpr_lines(path)
    assert [r["key"] for r in rows] == ["w02_l01_s00", "w02_l01_s01"]
    assert rows[0]["seg_end"] == [2.0, 0.0, 1.0]
    assert "t_written" in rows[0]
    assert manifest.line_key(None, 0, 0) == "wxx_l00_s00"
    assert manifest.read_gpr_lines(tmp_path / "nope.jsonl") == []


# ----------------------------------------------------------------------
# matching
# ----------------------------------------------------------------------
def _touch_scan(folder, stem, mtime, sidecar=True):
    folder.mkdir(parents=True, exist_ok=True)
    sgy = folder / f"{stem}.sgy"
    sgy.write_bytes(b"segy")
    os.utime(sgy, (mtime, mtime))
    if sidecar:
        (folder / f"{stem}.csv").write_text("meta")
    return sgy


def test_only_scans_with_a_sidecar_are_found_oldest_first(tmp_path):
    _touch_scan(tmp_path, "b", 200)
    _touch_scan(tmp_path, "a", 100)
    _touch_scan(tmp_path, "orphan", 150, sidecar=False)
    found = gpr.find_scan_files(tmp_path)
    assert [f["stem"] for f in found] == ["a", "b"]
    assert gpr.find_scan_files(tmp_path / "missing") == []


def test_scans_match_lines_by_name_first_then_by_time(tmp_path):
    lines = [
        {"key": "w02_l00_s00", "measurement_name": "scan_wall line 1 seg 1", "t_start_epoch": 100},
        {"key": "w02_l01_s00", "measurement_name": "scan_wall line 2 seg 1", "t_start_epoch": 200},
        {"key": "w02_l02_s00", "measurement_name": "scan_wall line 3 seg 1", "t_start_epoch": 300},
    ]
    files = [
        {"stem": "export_W02_L01_S00", "mtime": 50, "sgy": "x"},      # name wins over time
        {"stem": "GP8800_0007", "mtime": 250, "sgy": "y"},            # after line 2 started
        {"stem": "GP8800_0008", "mtime": 320, "sgy": "z"},            # after line 3
        {"stem": "GP8800_0009", "mtime": 330, "sgy": "w"},            # nothing left
    ]
    pairs = dict((f["stem"], (ln or {}).get("key")) for f, ln in gpr.match_files_to_lines(files, lines))
    assert pairs["export_W02_L01_S00"] == "w02_l01_s00"
    assert pairs["GP8800_0007"] == "w02_l00_s00"     # line 2 already taken by name; latest free start <= 250 is line 1
    assert pairs["GP8800_0008"] == "w02_l02_s00"
    assert pairs["GP8800_0009"] is None              # every line is taken


def test_a_scan_older_than_every_line_is_unassociated():
    pairs = gpr.match_files_to_lines([{"stem": "old", "mtime": 10, "sgy": "o"}],
                                     [{"key": "k", "t_start_epoch": 100}])
    assert pairs == [({"stem": "old", "mtime": 10, "sgy": "o"}, None)]


# ----------------------------------------------------------------------
# geo-referencing
# ----------------------------------------------------------------------
def test_scan_x_is_placed_along_the_segment():
    assert gpr.scan_to_map(0.5, [0, 0, 1.2], [2, 0, 1.2]) == [0.5, 0.0, 1.2]
    # Direction from the segment, not the axes; z is the segment's.
    assert gpr.scan_to_map(1.0, [0, 0, 0.8], [0, -3, 0.8]) == [0.0, -1.0, 0.8]
    # Degenerate segment: stay at the start rather than divide by zero.
    assert gpr.scan_to_map(1.0, [1, 1, 1], [1, 1, 1]) == [1.0, 1.0, 1.0]


def test_hyperbolae_are_compacted_and_geo_referenced():
    vendor_result = {
        "hyperbola_detected": True, "n_valid_detections": 1,
        "calibration": {"scan_distance_m": 2.0},
        "detections": [{
            "id": "H001", "position": {"x_m": 0.8, "x_relative": 40.0},
            "depth": {"depth_cm": 6.1},
            "robustness": {"gain_support": 1, "confidence_max": 0.91, "confidence_mean": 0.91},
        }],
    }
    line = {"seg_start": [1.0, 2.0, 1.0], "seg_end": [3.0, 2.0, 1.0]}
    out = gpr._compact_hyperbolae(vendor_result, line)
    assert out["n"] == 1 and out["detected"] is True
    assert out["detections"][0]["position_map"] == [1.8, 2.0, 1.0]
    assert out["detections"][0]["depth_cm"] == 6.1
    # Without a line there is nothing to place it with.
    assert "position_map" not in gpr._compact_hyperbolae(vendor_result, None)["detections"][0]


def test_map_transform_is_none_when_the_line_has_no_geometry():
    """One rule for whether a scan can be placed at all, shared by the compact
    detections and the NO_DRILL constraints."""
    assert gpr.map_transform(None) is None
    assert gpr.map_transform({"key": "w02_l00_s00"}) is None
    assert gpr.map_transform({"seg_start": [0, 0, 1], "seg_end": None}) is None
    to_map = gpr.map_transform({"seg_start": [1.0, 0.0, 1.0], "seg_end": [3.0, 0.0, 1.0]})
    assert to_map(0.5) == [1.5, 0.0, 1.0]


def test_the_summary_is_read_back_across_passes(tmp_path):
    """The NO_DRILL constraints are read from the session's whole GPR record,
    not just the pass that happened to run last."""
    assert gpr.load_summary(tmp_path) is None
    with open(tmp_path / gpr.SUMMARY_FILENAME, "w") as handle:
        json.dump({"entries": []}, handle)
    assert gpr.load_summary(tmp_path) is None          # nothing processed yet
    with open(tmp_path / gpr.SUMMARY_FILENAME, "w") as handle:
        json.dump({"entries": [{"key": "w02_l00_s00"}]}, handle)
    assert gpr.load_summary(tmp_path)["entries"][0]["key"] == "w02_l00_s00"


def test_an_entry_reports_the_no_drill_positions_it_produced():
    entry = {"key": "w02_l00_s00", "associated": True, "errors": {},
             "hyperbolae": {"n": 2}, "lines": {"n": 1},
             "no_drill": {"n_no_drill_positions": 2, "n_located": 1}}
    text = gpr.describe_entry(entry)
    assert "2 hyperbolae" in text
    assert "2 NO_DRILL (1 placed on the wall)" in text
    entry["no_drill"] = {"n_no_drill_positions": 0, "n_located": 0}
    assert "NO_DRILL" not in gpr.describe_entry(entry)


def test_pending_files_ignores_what_the_registry_already_has(tmp_path):
    incoming, out = tmp_path / "in", tmp_path / "out"
    a = _touch_scan(incoming, "a", 100)
    _touch_scan(incoming, "b", 200)
    out.mkdir()
    with open(out / gpr.REGISTRY_FILENAME, "w") as handle:
        json.dump({f"{a}@100": {"key": "a", "associated": False}}, handle)
    assert [f["stem"] for f in gpr.pending_files(incoming, out)] == ["b"]


def test_process_incoming_needs_the_segy_reader(tmp_path):
    """Without obspy nothing can be read: one clear error, not a traceback per file."""
    if importlib.util.find_spec("obspy") is not None:
        pytest.skip("obspy installed; the error path is not reachable")
    from task_planner_fsm.sensors import VendorUnavailable
    _touch_scan(tmp_path / "in", "a", time.time())
    with pytest.raises(VendorUnavailable, match="obspy"):
        gpr.process_incoming(tmp_path / "in", tmp_path / "m.jsonl", tmp_path / "out",
                             tmp_path / "w.pt")


# ----------------------------------------------------------------------
# ScanWall writes the manifest
# ----------------------------------------------------------------------
class _FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(("info", msg))

    def warn(self, msg, **kw):
        self.lines.append(("warn", msg))

    def error(self, msg, **kw):
        self.lines.append(("error", msg))


class _FakeNode:
    def __init__(self):
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger

    def destroy_timer(self, timer):
        pass


def test_scan_wall_records_one_manifest_row_per_swept_segment(tmp_path):
    """Opened when the triggers arm (once per segment sweep), closed when they
    stop; the row carries the map-frame segment and the probe's measurement name."""
    from task_planner_fsm.states.scan_wall import ScanWall

    state = ScanWall("ScanWall")
    ctx = {"node": _FakeNode(), "sensor_data_dir": str(tmp_path), "sensor_session_id": "s9",
           "current_wall_index": 4, "current_line_idx": 1}
    state._seg_idx = 2
    state.gpr_line_active = True
    state._gpr_trigger_count = 37
    state._gpr_trigger_travel = 0.1845
    state._open_gpr_line_record(ctx, (1.0, 2.0, 1.1), (1.0, 4.5, 1.1))
    assert state._gpr_line_record["measurement_name"] == "scan_wall line 2 seg 3"
    state._stop_gpr_triggers(ctx, log_summary=False)      # closes the record
    state._stop_gpr_triggers(ctx, log_summary=False)      # idempotent: no second row

    rows = manifest.read_gpr_lines(paths.gpr_manifest_path(ctx))
    assert len(rows) == 1
    row = rows[0]
    assert row["key"] == "w04_l01_s02"
    assert row["seg_start"] == [1.0, 2.0, 1.1] and row["seg_end"] == [1.0, 4.5, 1.1]
    assert row["frame"] == "map"
    assert row["probe_active"] is True
    assert row["trigger_count"] == 37 and row["travel_m"] == 0.1845
    assert row["t_stop_epoch"] >= row["t_start_epoch"]
    assert not any(level == "warn" for level, _ in ctx["node"].logger.lines)
