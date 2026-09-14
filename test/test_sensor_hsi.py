"""The HSI adapter: reflectance.csv -> the classifier's input, and the join of
its verdicts back onto the sweep's samples. The classifier itself only runs when
its stack is installed (see the last test)."""

import csv
import importlib
import json

import numpy as np
import pytest

from task_planner_fsm.sensors import VendorUnavailable, hsi, paths
from task_planner_fsm.utils import hyperspectral_processing as hp


def _write_reflectance(path, rows):
    """A reflectance.csv in SessionProcessor's format with the given samples.

    ``rows``: ``(seq, wall, line, seg, status, pose, pose_map, level)``; the
    spectrum is flat at ``level`` so a row can be told apart after the rewrite.
    """
    header = hp.SessionProcessor._header()
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for seq, wall, line, seg, status, pose, pose_map, level in rows:
            pose = pose or [None, None, None]
            pose_map = pose_map or [None, None, None]
            row = [seq, "2026-09-14T10:00:00Z", wall, line, seg, seq, 0.02 * seq, "arm_base",
                   *pose, *pose_map, status, "", "", ""]
            row += [f"{level:.6f}"] * (2 * hp.SPECTRUM_LENGTH)
            writer.writerow(row)


def test_reflectance_csv_is_rewritten_with_numeric_wavelength_headers(tmp_path):
    src = tmp_path / hp.REFLECTANCE_FILENAME
    _write_reflectance(src, [
        (1, 2, 0, 0, hp.ACCEPTED, [0.1, 0.2, 1.0], [5.1, 3.2, 1.0], 0.5),
        (2, 2, 0, 0, hp.REJECTED_CALIBRATION, [0.2, 0.2, 1.0], None, 0.0),
        (3, 2, 0, 1, hp.REJECTED_STABILITY, [0.3, 0.2, 1.0], [5.3, 3.2, 1.0], 0.7),
    ])
    metadata = hsi.build_input_csv(src, tmp_path / "out" / "input.csv")

    with open(tmp_path / "out" / "input.csv", newline="") as handle:
        rows = list(csv.reader(handle))
    header, body = rows[0], rows[1:]
    assert header[0] == "Counter"
    wavelengths = np.array([float(h) for h in header[1:]])
    assert len(wavelengths) == 512
    # The sensor export's grid: VIS 325.3-792.6, NIR 991-1707, in order.
    assert wavelengths[0] == pytest.approx(325.3, abs=0.05)
    assert wavelengths[255] == pytest.approx(792.6, abs=0.05)
    assert wavelengths[256] == pytest.approx(991.0, abs=0.05)
    assert wavelengths[-1] == pytest.approx(1707.0, abs=0.05)
    assert np.all(np.diff(wavelengths) > 0)

    # The calibration-rejected row is left out; the stability-rejected one goes
    # through (the vendor's quality filter is the authority now).
    assert [r[0] for r in body] == ["1", "3"]
    assert body[0][1] == "0.500000" and body[1][1] == "0.700000"
    assert [m["seq"] for m in metadata] == [1, 3]
    assert metadata[0]["pose_map"] == [5.1, 3.2, 1.0]
    assert metadata[0]["pose"] == [0.1, 0.2, 1.0]
    assert metadata[0]["frame"] == "arm_base"
    assert metadata[1]["fsm_status"] == hp.REJECTED_STABILITY
    assert metadata[1]["seg_idx"] == 1


def test_results_are_joined_by_row_order_and_checked_against_counter():
    metadata = [{"seq": 4, "wall_index": 1}, {"seq": 9, "wall_index": 1}]
    result = {"samples": [
        {"sample_index": 0, "detected": True, "material": "gypsum", "confidence": 0.97,
         "status": "detected", "reason": None, "metadata": {"Counter": 4}},
        {"sample_index": 1, "detected": False, "material": None, "confidence": 0.61,
         "status": "low_confidence", "reason": "below threshold", "metadata": {"Counter": 9}},
    ]}
    samples = hsi.join_results(metadata, result)
    assert samples[0]["material"] == "gypsum" and samples[0]["seq"] == 4
    assert samples[1]["status"] == "low_confidence" and samples[1]["detected"] is False
    by_wall = hsi.summarize_by_wall(samples)
    assert by_wall[1] == {"n": 2, "status": {"detected": 1, "low_confidence": 1},
                          "material": {"gypsum": 1}}
    assert "gypsum x1" in hsi.describe_wall(1, by_wall[1])


def test_a_misaligned_join_is_refused_not_silently_wrong():
    metadata = [{"seq": 4}]
    result = {"samples": [{"sample_index": 0, "status": "detected", "detected": True,
                           "material": "brick", "confidence": 0.9, "metadata": {"Counter": 5}}]}
    with pytest.raises(ValueError, match="join by row order"):
        hsi.join_results(metadata, result)


def test_a_row_without_a_verdict_is_marked_missing():
    samples = hsi.join_results([{"seq": 1}, {"seq": 2}], {"samples": [
        {"sample_index": 0, "status": "detected", "detected": True, "material": "brick",
         "confidence": 0.9, "metadata": {"Counter": 1}}]})
    assert samples[1]["status"] == "missing"


def test_runtime_config_points_at_our_model_and_threshold(tmp_path):
    model = tmp_path / "classifier.joblib"
    model.write_bytes(b"x")
    cfg_path = hsi.write_runtime_config(tmp_path / "cfg.json", model, 0.85)
    with open(cfg_path) as handle:
        cfg = json.load(handle)
    assert cfg["model_path"] == str(model.resolve())
    assert cfg["confidence_threshold"] == 0.85
    assert cfg["keep_columns"] == ["Counter"]
    # The vendor's other keys survive.
    assert "output_root" in cfg


def test_classify_session_reports_missing_inputs_clearly(tmp_path):
    with pytest.raises(FileNotFoundError, match="reflectance"):
        hsi.classify_session(tmp_path, tmp_path / "out", tmp_path / "m.joblib")
    _write_reflectance(tmp_path / hp.REFLECTANCE_FILENAME,
                       [(1, 0, 0, 0, hp.ACCEPTED, None, None, 0.5)])
    with pytest.raises(FileNotFoundError, match="classifier"):
        hsi.classify_session(tmp_path, tmp_path / "out", tmp_path / "m.joblib")


def _has(module):
    try:
        importlib.import_module(module)
    except ImportError:
        return False
    return True


@pytest.mark.skipif(not (_has("xgboost") and paths.hsi_model_path().is_file()),
                    reason="needs xgboost and models/hsi/classifier.joblib")
def test_the_real_classifier_runs_end_to_end_on_a_synthetic_session(tmp_path):
    """Smoke test of the vendored pipeline over our CSV shape. Flat spectra are
    not a material, so the verdicts are expected to be rejections -- the point
    is that every row comes back, joined to its metadata."""
    _write_reflectance(tmp_path / hp.REFLECTANCE_FILENAME, [
        (1, 0, 0, 0, hp.ACCEPTED, [0.1, 0, 1], [1, 2, 1], 0.5),
        (2, 0, 0, 0, hp.ACCEPTED, [0.2, 0, 1], [1.1, 2, 1], 0.6),
    ])
    result = hsi.classify_session(tmp_path, tmp_path / "out", paths.hsi_model_path())
    assert result["n_classified"] == 2
    assert [s["seq"] for s in result["samples"]] == [1, 2]
    assert all(s["status"] in ("detected", "low_confidence", "quality_rejected")
               for s in result["samples"])
    assert (tmp_path / "out" / "hsi_result.json").is_file()
    assert (tmp_path / "out" / hsi.SAMPLES_FILENAME).is_file()


@pytest.mark.skipif(_has("xgboost"), reason="only meaningful without xgboost")
def test_a_missing_classifier_stack_is_one_clear_error(tmp_path):
    _write_reflectance(tmp_path / hp.REFLECTANCE_FILENAME,
                       [(1, 0, 0, 0, hp.ACCEPTED, None, None, 0.5)])
    model = tmp_path / "classifier.joblib"
    model.write_bytes(b"x")
    with pytest.raises(VendorUnavailable, match="xgboost"):
        hsi.classify_session(tmp_path, tmp_path / "out", model)
