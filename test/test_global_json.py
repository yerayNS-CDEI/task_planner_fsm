"""The global JSON for Tecnalia: the sensor team's builder, with the robot's
fields filled in -- one element per wall, map coordinates, HSI timestamps."""

import json

import pytest

from task_planner_fsm.sensors import global_json


def _write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def _scan_folder(root, key, scan_id, objects=((0.5, 4.0, 0.9),), length=2.0):
    """A per-scan folder as gpr.process_scan leaves it: the acquisition next
    to ``hyperbola/`` and ``line/`` with the files the builder reads."""
    folder = root / "gpr" / key
    _write(folder / "hyperbola" / "gpr_result.json",
           {"status": "ok", "scan_id": scan_id, "calibration": {"scan_distance_m": length}})
    _write(folder / "hyperbola" / "gpr_data.json", {"schemaVersion": 2, "elements": [{
        "velMns": 0.12, "velSigmaMns": 0.006,
        "objects": [{"xM": x, "d": d, "c": c} for x, d, c in objects]}]})
    _write(folder / "line" / "line_result.json", {"lines": [
        {"depth_cm_approx": 0.0, "near_time_zero_reference": True},
        {"depth_cm_approx": 6.0, "thickness_cm_approx": 1.0}]})
    (folder / f"{scan_id}.sgy").write_bytes(b"")
    return folder


def _entry(folder, key, wall, start=(1.0, 2.0, 0.5), end=(3.0, 2.0, 0.5)):
    line = None if wall is None else {
        "key": key, "wall_index": wall, "line_idx": 0, "seg_idx": 0,
        "seg_start": list(start), "seg_end": list(end)}
    return {"key": key, "out_dir": str(folder), "line": line}


def _hsi(root, walls):
    """A classified session: hsi_result.json in the vendor's shape, and the
    adapter's joined samples in the same order."""
    hsi_dir = root / "hsi"
    vendor, samples = [], []
    for i, wall in enumerate(walls):
        vendor.append({"sample_index": i, "source_row": i + 1, "material": "gypsum",
                       "confidence": 0.97, "status": "detected", "detected": True})
        samples.append({"seq": i + 1, "wall_index": wall, "reflectance_row": i + 3,
                        "timestamp": f"2026-11-04T10:00:0{i}Z",
                        "pose_map": [float(i), 1.0, 1.5], "status": "detected"})
    _write(hsi_dir / "hsi_result.json", {"input_file": "input.csv", "samples": vendor})
    (hsi_dir / "input.csv").write_text("")
    (hsi_dir / "reflectance.csv").write_text("")
    return {"samples": samples, "hsi_result_json": str(hsi_dir / "hsi_result.json"),
            "reflectance_csv": str(hsi_dir / "reflectance.csv")}


def _build(tmp_path, entries=(), hsi=None):
    out = tmp_path / global_json.FILENAME
    stats = global_json.build_document(out, gpr_results={"entries": list(entries)}, hsi=hsi)
    return json.loads(out.read_text()), stats


def _xyz(point):
    return [point[a]["value"] for a in ("x", "y", "z")]


def test_one_element_per_wall_with_the_scans_and_spectra_of_that_wall(tmp_path):
    a = _scan_folder(tmp_path, "w00_l00_s00", "scanA")
    b = _scan_folder(tmp_path, "w01_l00_s00", "scanB")
    doc, stats = _build(tmp_path, [_entry(a, "w00_l00_s00", 0), _entry(b, "w01_l00_s00", 1)],
                        _hsi(tmp_path, walls=[0, 0, 1]))
    assert list(doc) == ["wall_00", "wall_01"]
    assert stats["elements"]["wall_00"] == {
        "continuous_gpr": 1, "discrete_gpr": 1, "hyperspectral_measurements": 2}
    assert [m["id"] for m in doc["wall_01"][0]["hyperspectral_camera"]["measurements"]] == ["hsc-003"]


def test_bscan_ends_and_hyperbolae_are_placed_on_the_map(tmp_path):
    folder = _scan_folder(tmp_path, "w00_l00_s00", "scanA", objects=((0.5, 4.0, 0.9),))
    doc, _ = _build(tmp_path, [_entry(folder, "w00_l00_s00", 0)])
    [block] = doc["wall_00"]
    [continuous] = block["continuous_gpr"]
    assert _xyz(continuous["bscan_coordinates"]["beginning"]) == [1.0, 2.0, 0.5]
    assert _xyz(continuous["bscan_coordinates"]["end"]) == [3.0, 2.0, 0.5]     # 2 m along x
    assert continuous["data_pointer"] == "gpr/w00_l00_s00/scanA.sgy"
    [discrete] = block["discrete_gpr"]
    assert _xyz(discrete["point_coordinates"]) == [1.5, 2.0, 0.5]
    assert discrete["depth"]["value"] == pytest.approx(0.04)
    # The builder's own work is untouched: one layer above the 6 cm interface.
    assert [layer["thickness"]["value"] for layer in continuous["wall_layers"]] == [0.06]


def test_a_scan_tied_to_no_line_is_kept_without_coordinates(tmp_path):
    folder = _scan_folder(tmp_path, "orphan", "scanX")
    doc, _ = _build(tmp_path, [_entry(folder, "orphan", None)])
    [block] = doc[global_json.UNASSOCIATED]
    assert _xyz(block["continuous_gpr"][0]["bscan_coordinates"]["beginning"]) == [None] * 3
    assert _xyz(block["discrete_gpr"][0]["point_coordinates"]) == [None] * 3


def test_spectra_get_their_timestamp_pose_and_reflectance_row(tmp_path):
    doc, _ = _build(tmp_path, hsi=_hsi(tmp_path, walls=[2, 2]))
    hsi_block = doc["wall_02"][0]["hyperspectral_camera"]
    assert hsi_block["data_pointer_csv"] == "hsi/reflectance.csv"
    first = hsi_block["measurements"][0]
    assert first["timestamp"] == "2026-11-04T10:00:00Z"
    assert _xyz(first["point_coordinates"]) == [0.0, 1.0, 1.5]
    assert first["row"] == 3                       # reflectance.csv, not input.csv
    assert first["status"] == "detected" and first["material"] == "gypsum"


def test_a_sample_list_that_disagrees_with_the_result_attaches_nothing(tmp_path):
    """Poses are joined by order; if the two lists differ, no pose is better
    than the wrong one."""
    hsi = _hsi(tmp_path, walls=[2, 2])
    hsi["samples"] = hsi["samples"][:1]
    doc, _ = _build(tmp_path, hsi=hsi)
    assert doc["wall_02"][0]["hyperspectral_camera"]["measurements"] == []


def test_a_rescanned_key_appears_once(tmp_path):
    folder = _scan_folder(tmp_path, "w00_l00_s00", "scanA")
    entry = _entry(folder, "w00_l00_s00", 0)
    doc, _ = _build(tmp_path, [entry, dict(entry)])
    assert len(doc["wall_00"][0]["continuous_gpr"]) == 1


def test_the_file_is_the_builders_compact_layout(tmp_path):
    folder = _scan_folder(tmp_path, "w00_l00_s00", "scanA")
    _build(tmp_path, [_entry(folder, "w00_l00_s00", 0)])
    text = (tmp_path / global_json.FILENAME).read_text()
    assert '"bscan_length": {"value": 2.0, "error": null, "unit": "m"}' in text
