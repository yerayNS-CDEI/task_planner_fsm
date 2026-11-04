from __future__ import annotations

import importlib.util
import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from .figures import save_material_overview, save_spectrum_images
from .input_adapter import prepare_input_for_benjamin, resolve_input

PROJECT_ROOT = Path(__file__).resolve().parents[1]
BENJAMIN_DIR = PROJECT_ROOT / "benjamin_original"

ROBOT_JSON = "hsi_result.json"
OVERVIEW_PNG = "hsi_material_overview.png"
SUMMARY_CSV = "hsi_classification_summary.csv"
SPECTRA_DIR = "spectra"
# Files written by the v1 integration; removed so the output folder only holds current deliverables.
LEGACY_OUTPUTS = ("classification_results.csv", "summary.json", "input_diagnostics.json",
                  "run_manifest.json")

_benjamin = None


def _load_benjamin():
    """Import Benjamin's unchanged classifier.py under a unique module name."""
    global _benjamin
    if _benjamin is None:
        spec = importlib.util.spec_from_file_location("benjamin_hsi_classifier",
                                                      BENJAMIN_DIR / "classifier.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _benjamin = module
    return _benjamin


def _json_value(v: Any):
    if v is None:
        return None
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.floating):
        return None if np.isnan(v) else float(v)
    try:
        if pd.isna(v):
            return None
    except Exception:
        pass
    return v


def load_config(config_path: str | Path | None = None) -> dict:
    path = Path(config_path) if config_path else Path(__file__).with_name("config.json")
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def run_hyperspectral_pipeline(
    input_path: str | Path,
    output_dir: str | Path | None = None,
    config_path: str | Path | None = None,
) -> dict:
    """Run the DISCOVER HSI material classifier (Benjamin v2, 1D-CNN).

    Accepted input: a robot acquisition session (folder or its reflectance.csv),
    the text-based .xls export of the LENZ software, or a CSV with spectra.
    The returned dict is the robot-internal result (consumed by the POKEYE decision)
    and is also written as hsi_result.json.
    """
    cfg = load_config(config_path)
    input_path, input_name = resolve_input(input_path)

    model_path = PROJECT_ROOT / cfg["model_path"]
    if not model_path.exists():
        raise FileNotFoundError(model_path)

    if output_dir is None:
        output_dir = PROJECT_ROOT / cfg.get("output_root", "../test_output") / input_name
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bj = _load_benjamin()
    device = cfg.get("torch_device", "cpu")
    if device != "auto":
        bj.TORCH_DEVICE = bj.torch.device(device)

    threshold = float(cfg.get("confidence_threshold", 0.8))
    keep_columns = cfg.get("keep_columns", [])

    prepared, source_rows, input_summary = prepare_input_for_benjamin(input_path)
    bundle = bj.load_bundle(model_path)
    an = bj.Analysis(prepared, bundle, threshold)

    samples = []
    for i in range(len(an.valid)):
        confidence = _json_value(an.conf[i])
        quality = an.quality[i]
        if not an.valid[i]:
            detected, material, status, reason = False, None, "quality_rejected", str(an.reasons[i])
        elif an.conf[i] < threshold:
            detected, material, status = False, None, "low_confidence"
            reason = f"confidence below threshold {threshold:.3f}"
        else:
            detected, material, status, reason = True, str(an.material[i]), "detected", None

        metadata = {}
        for col in keep_columns:
            if col in prepared.columns:
                value = _json_value(prepared.at[i, col])
                if value not in (None, ""):
                    metadata[col] = value

        samples.append({
            "sample_index": i,
            "source_row": int(source_rows[i]),
            "detected": detected,
            "material": material,
            "confidence": confidence,
            "status": status,
            "reason": reason,
            "spectrum_quality": round(float(quality["score"]), 1),
            "spectrum_quality_grade": quality["grade"],
            "metadata": metadata,
        })

    # Robot-internal result (POKEYE input). The global JSON for Tecnalia is built
    # from this file by FULL PIPELINE/Global JSON.
    robot_result = {
        "sensor": "hyperspectral",
        "input_file": input_path.name,
        "classifier": {
            "author": "Benjamin",
            "version": "v2",
            "pipeline_version": bundle.get("pipeline_version"),
            "feature_set": bundle.get("feat"),
            "classes": [str(c) for c in bundle["le"].classes_],
        },
        "confidence_threshold": threshold,
        "input_summary": input_summary,
        "n_samples": len(samples),
        "n_detected": sum(int(s["detected"]) for s in samples),
        "samples": samples,
    }
    if len(samples) == 1:
        robot_result.update({k: samples[0][k] for k in
                             ("detected", "material", "confidence", "status", "reason")})

    # ── Deliverables
    for name in LEGACY_OUTPUTS:
        (output_dir / name).unlink(missing_ok=True)

    (output_dir / ROBOT_JSON).write_text(
        json.dumps(robot_result, indent=2, ensure_ascii=False), encoding="utf-8")

    summary = an.summary()
    summary.insert(1, "source_row", source_rows)
    summary.to_csv(output_dir / SUMMARY_CSV, index=False, encoding="utf-8-sig")

    raw_copy = output_dir / input_path.name
    if raw_copy.resolve() != input_path:
        shutil.copy2(input_path, raw_copy)

    out_cfg = cfg.get("outputs", {})
    dpi = int(out_cfg.get("image_dpi", 100))
    save_material_overview(bj, an, output_dir / OVERVIEW_PNG, dpi)
    spectra_dir = output_dir / SPECTRA_DIR
    if spectra_dir.exists():
        shutil.rmtree(spectra_dir)
    if out_cfg.get("save_spectrum_images", True):
        save_spectrum_images(bj, an, spectra_dir, dpi)

    return robot_result
