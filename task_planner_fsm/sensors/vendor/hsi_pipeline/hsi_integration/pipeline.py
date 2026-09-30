from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from .input_adapter import prepare_input_for_benjamin

PROJECT_ROOT = Path(__file__).resolve().parents[1]
BENJAMIN_DIR = PROJECT_ROOT / "benjamin_original"
if str(BENJAMIN_DIR) not in sys.path:
    sys.path.insert(0, str(BENJAMIN_DIR))

from predict import predecir  # noqa: E402


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
    """Run the DISCOVER HSI material classifier.

    Accepted input includes Benjamin-style CSV files and the real text-based .xls
    export produced by the spectrometer software. The returned dict is deliberately
    transport-agnostic so the software team can map it onto its ROS2 interface.
    """
    cfg = load_config(config_path)
    input_path = Path(input_path).resolve()
    if not input_path.exists():
        raise FileNotFoundError(input_path)

    model_path = PROJECT_ROOT / cfg["model_path"]
    if not model_path.exists():
        raise FileNotFoundError(model_path)

    if output_dir is None:
        output_dir = PROJECT_ROOT / cfg.get("output_root", "outputs") / input_path.stem
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    prepared, diagnostics = prepare_input_for_benjamin(input_path, model_path)
    diagnostics_path = output_dir / "input_diagnostics.json"
    diagnostics_path.write_text(json.dumps(diagnostics, indent=2, ensure_ascii=False), encoding="utf-8")

    detailed_csv = output_dir / "classification_results.csv"
    threshold = float(cfg.get("confidence_threshold", 0.8))
    keep_columns = cfg.get("keep_columns", [])

    # Benjamin's original function currently accepts a CSV path. The adapted file
    # is therefore written only to a temporary directory and is not an operational output.
    with tempfile.TemporaryDirectory(prefix="hsi_prepared_") as tmp:
        prepared_csv = Path(tmp) / "prepared_input.csv"
        prepared.to_csv(prepared_csv, index=False)
        df_results = predecir(
            csv_path=str(prepared_csv),
            model_path=str(model_path),
            confianza_minima=threshold,
            output_path=str(detailed_csv),
            columnas_a_conservar=keep_columns,
        )

    samples = []
    for idx, row in df_results.reset_index(drop=True).iterrows():
        quality = str(row.get("filtro_calidad", ""))
        final_prediction = str(row.get("prediccion_final", ""))
        confidence = _json_value(row.get("confianza"))

        if quality != "OK":
            detected = False
            material = None
            status = "quality_rejected"
            reason = quality
        elif final_prediction == "no se ha podido detectar":
            detected = False
            material = None
            status = "low_confidence"
            reason = f"confidence below threshold {threshold:.3f}"
        else:
            detected = True
            material = final_prediction
            status = "detected"
            reason = None

        metadata = {}
        for col in keep_columns:
            if col in row.index:
                value = _json_value(row[col])
                if value is not None:
                    metadata[col] = value

        samples.append({
            "sample_index": int(idx),
            "detected": detected,
            "material": material,
            "confidence": confidence,
            "status": status,
            "reason": reason,
            "metadata": metadata,
        })

    # Compact structure intended to be reused by the ROS2 integration layer.
    ros2_result = {
        "sensor": "hyperspectral",
        "input_file": input_path.name,
        "confidence_threshold": threshold,
        "n_samples": len(samples),
        "n_detected": sum(int(s["detected"]) for s in samples),
        "samples": samples,
    }
    if len(samples) == 1:
        ros2_result.update({
            "detected": samples[0]["detected"],
            "material": samples[0]["material"],
            "confidence": samples[0]["confidence"],
            "status": samples[0]["status"],
            "reason": samples[0]["reason"],
        })

    result_json = output_dir / "hsi_result.json"
    result_json.write_text(json.dumps(ros2_result, indent=2, ensure_ascii=False), encoding="utf-8")

    status_counts = pd.Series([s["status"] for s in samples]).value_counts().to_dict() if samples else {}
    detected_materials = pd.Series(
        [s["material"] for s in samples if s["detected"]], dtype="object"
    ).value_counts().to_dict() if any(s["detected"] for s in samples) else {}
    summary = {
        "n_samples": len(samples),
        "n_detected": ros2_result["n_detected"],
        "status_counts": {str(k): int(v) for k, v in status_counts.items()},
        "detected_material_counts": {str(k): int(v) for k, v in detected_materials.items()},
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    run_manifest = {
        "input_file": str(input_path),
        # DISCOVER FSM integration: the model may live outside this project
        # (task_planner_fsm/models/hsi); record it as given in that case.
        "model": (str(model_path.relative_to(PROJECT_ROOT))
                  if model_path.is_relative_to(PROJECT_ROOT) else str(model_path)),
        "confidence_threshold": threshold,
        "input_adapter": {
            "detected_format": diagnostics["detected_format"],
            "source_band_count": diagnostics["source_band_count"],
            "model_band_count": diagnostics["model_band_count"],
            "excluded_non_sample_rows": diagnostics["excluded_non_sample_rows"],
        },
        "outputs": {
            "detailed_csv": detailed_csv.name,
            "ros2_compatible_json": result_json.name,
            "summary": "summary.json",
            "input_diagnostics": diagnostics_path.name,
        },
    }
    (output_dir / "run_manifest.json").write_text(
        json.dumps(run_manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    return ros2_result
