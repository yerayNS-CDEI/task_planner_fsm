from __future__ import annotations

import copy
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pandas as pd
import yaml

from .utils import load_module_from_path, load_yaml

_OBJECT_BASE_COLS = [
    "element", "x_m", "apex_px", "twt_ns", "v_m_ns", "eps_r",
    "depth_cm", "confidence", "arc_hw_m",
]
_SUMMARY_COLS = [
    "element", "type", "n_objects", "length_m", "area_m2", "bar_pitch_cm",
    "bar_pitch_sd_cm", "n_layers", "layer_depths_cm", "cover_min_cm",
    "interpretation", "recoverable_steel_kg", "value_eur", "co2_avoided_kg",
]
_PASSPORT_COLS = [
    "element", "type", "interpretation", "n_objects", "n_layers",
    "bar_pitch_cm", "cover_min_cm", "area_m2", "recoverable_steel_kg",
    "value_eur", "co2_avoided_kg",
]


def consensus_to_tahzeeb_objects(accepted: pd.DataFrame, element_id: str) -> pd.DataFrame:
    """Convert cross-gain consensus rows to the object schema expected by Tahzeeb 03/04.

    Confidence is the mean model confidence across the gains supporting that physical
    candidate. Extra consensus columns are retained for audit; Tahzeeb's scripts ignore them.
    """
    if accepted.empty:
        return pd.DataFrame(columns=_OBJECT_BASE_COLS)
    rows: list[dict[str, Any]] = []
    for r in accepted.to_dict(orient="records"):
        rows.append({
            "element": element_id,
            "x_m": r["x_m"],
            "apex_px": r["apex_px"],
            "twt_ns": r["twt_ns"],
            "v_m_ns": r["v_m_ns"],
            "eps_r": r["eps_r_assumed"],
            "depth_cm": r["depth_cm_assumed"],
            "confidence": r["confidence_mean"],
            "arc_hw_m": r["arc_hw_m"],
            "detection_id": r["detection_id"],
            "gain_support": r["support_count"],
            "supporting_gains_db": r["gains_db"],
            "confidence_max": r["confidence_max"],
            "x_std_m": r["x_std_m"],
            "depth_std_cm": r["depth_std_cm"],
        })
    return pd.DataFrame(rows)


def _build_runtime_config(
    base_cfg: dict[str, Any],
    integration_cfg: dict[str, Any],
    element_id: str,
    scan_distance_m: float,
    processed_time_window_ns: float,
    image_width_px: int,
    image_height_px: int,
    reference_bscan: Path,
    output_dir: Path,
) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    cfg.setdefault("io", {})
    cfg.setdefault("calibration", {})
    cfg.setdefault("quantify", {})

    cfg["io"]["results_dir"] = str(output_dir)
    cfg["io"]["bscan_images"] = {element_id: str(reference_bscan)}
    # These site-specific paths in Tahzeeb's supplied config are irrelevant to the
    # robot run and may not exist on another machine. 04_export only uses them when
    # explicitly asked to embed images, which this integration does not do.
    cfg["io"].pop("element_photos", None)
    cfg["io"].pop("element_drawings", None)
    cfg["io"]["site_plan"] = None

    cal = cfg["calibration"]
    cal["scan_distance_m"] = {element_id: float(scan_distance_m)}
    cal["time_window_ns"] = float(processed_time_window_ns)
    cal["image_height_px"] = int(image_height_px)
    cal["px_per_m"] = float(image_width_px) / float(scan_distance_m)
    cal["epsilon_r"] = float(integration_cfg["postprocessing"]["epsilon_r_assumed"])

    explicit_type = integration_cfg.get("tahzeeb_final_export", {}).get("element_type_override")
    if explicit_type:
        overrides = cfg["quantify"].setdefault("element_overrides", {}) or {}
        overrides[element_id] = str(explicit_type)
        cfg["quantify"]["element_overrides"] = overrides
    return cfg


def _run(cmd: list[str], cwd: Path, log_path: Path) -> None:
    proc = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True)
    log_path.write_text((proc.stdout or "") + ("\n--- STDERR ---\n" + proc.stderr if proc.stderr else ""), encoding="utf-8")
    if proc.returncode != 0:
        raise RuntimeError(f"Tahzeeb command failed ({' '.join(cmd)}). See {log_path}")


def run_tahzeeb_final_exports(
    accepted: pd.DataFrame,
    run_dir: Path,
    tahzeeb_dir: Path,
    tahzeeb_config_path: Path,
    integration_cfg: dict[str, Any],
    element_id: str,
    scan_distance_m: float,
    processed_time_window_ns: float,
    image_width_px: int,
    image_height_px: int,
    reference_bscan: Path,
) -> dict[str, str]:
    """Create Tahzeeb/MIP compatibility outputs from FINAL cross-gain detections.

    Runs the supplied 03_quantify.py and 04_export.py unchanged. Their material/recovery
    interpretation is a downstream compatibility layer and is not used as the robot-facing
    GPR result. 05_visualize.py can also be run on the same consensus object set.
    """
    run_dir.mkdir(parents=True, exist_ok=True)
    out_cfg = integration_cfg.get("tahzeeb_final_export", {})
    if not bool(out_cfg.get("enabled", True)):
        return {}

    objects = consensus_to_tahzeeb_objects(accepted, element_id)
    objects_path = run_dir / "gpr_consensus_objects.csv"
    objects.to_csv(objects_path, index=False)
    summary_path = run_dir / "element_summary.csv"
    passport_path = run_dir / "material_passport.csv"
    json_path = run_dir / "gpr_data.json"

    # Tahzeeb's 03_quantify assumes at least one object and errors on an empty dataframe.
    # Emit the valid empty downstream schema without changing his source code.
    if objects.empty:
        pd.DataFrame(columns=_SUMMARY_COLS).to_csv(summary_path, index=False)
        pd.DataFrame(columns=_PASSPORT_COLS).to_csv(passport_path, index=False)
        json_path.write_text(json.dumps({"schemaVersion": 2, "elements": []}, indent=2), encoding="utf-8")
        return {
            "tahzeeb_objects": objects_path.name,
            "mip_json": json_path.name,
            "element_summary": summary_path.name,
            "material_passport": passport_path.name,
        }

    base = load_yaml(tahzeeb_config_path)
    runtime = _build_runtime_config(
        base, integration_cfg, element_id, scan_distance_m,
        processed_time_window_ns, image_width_px, image_height_px, reference_bscan, run_dir,
    )
    runtime_path = run_dir / "tahzeeb_runtime_final.yaml"
    runtime_path.write_text(yaml.safe_dump(runtime, sort_keys=False, allow_unicode=True), encoding="utf-8")

    # Match Tahzeeb's own run_postprocessing.py: keep the Step-3 dataframe in
    # memory when calling Step 4. This preserves layer_depths_cm as a string even
    # for a single layer (the standalone 04_export CLI re-reads the CSV and pandas
    # may coerce e.g. "10.3" to float, losing layerDepthsCm in the JSON).
    step3 = load_module_from_path("tahzeeb_03_quantify_final", tahzeeb_dir / "03_quantify.py", tahzeeb_dir)
    step4 = load_module_from_path("tahzeeb_04_export_final", tahzeeb_dir / "04_export.py", tahzeeb_dir)
    summ = step3.quantify(runtime, objects.copy())
    summ.to_csv(summary_path, index=False)
    viewer = step4.build_viewer_json(objects.copy(), summ.copy(), runtime)
    json_path.write_text(json.dumps(viewer, indent=2), encoding="utf-8")
    summ[_PASSPORT_COLS].to_csv(passport_path, index=False)
    (run_dir / "tahzeeb_03_04_note.txt").write_text(
        "Generated by Tahzeeb 03_quantify.quantify + 04_export.build_viewer_json, "
        "following the in-memory sequence used by run_postprocessing.py.\n",
        encoding="utf-8",
    )

    files = {
        "tahzeeb_objects": objects_path.name,
        "mip_json": json_path.name,
        "element_summary": summary_path.name,
        "material_passport": passport_path.name,
        "tahzeeb_runtime_config": runtime_path.name,
    }

    if bool(out_cfg.get("run_visualization", True)):
        viz_dir = run_dir / "tahzeeb_viz"
        _run([
            sys.executable, str(tahzeeb_dir / "05_visualize.py"),
            "--config", str(runtime_path), "--objects", str(objects_path),
            "--outdir", str(viz_dir),
        ], tahzeeb_dir, run_dir / "tahzeeb_05_visualize.log")
        files["tahzeeb_visualization_dir"] = viz_dir.name
    return files
