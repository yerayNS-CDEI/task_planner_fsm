from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import pandas as pd

from .utils import load_module_from_path, load_yaml

_OBJECT_BASE_COLS = [
    "element", "x_m", "apex_px", "twt_ns", "v_m_ns", "eps_r",
    "depth_cm", "confidence", "arc_hw_m",
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
    """Create only the final Tahzeeb-compatible JSON deliverable.

    Tahzeeb 03_quantify.py and 04_export.py are still used, but their auxiliary CSV/YAML
    artefacts are no longer persisted unless the workflow is expanded again later.
    """
    run_dir.mkdir(parents=True, exist_ok=True)
    out_cfg = integration_cfg.get("tahzeeb_final_export", {})
    if not bool(out_cfg.get("enabled", True)):
        return {}

    objects = consensus_to_tahzeeb_objects(accepted, element_id)
    json_path = run_dir / "gpr_data.json"

    if objects.empty:
        json_path.write_text(json.dumps({"schemaVersion": 2, "elements": []}, indent=2), encoding="utf-8")
        return {"mip_json": json_path.name}

    base = load_yaml(tahzeeb_config_path)
    runtime = _build_runtime_config(
        base,
        integration_cfg,
        element_id,
        scan_distance_m,
        processed_time_window_ns,
        image_width_px,
        image_height_px,
        reference_bscan,
        run_dir,
    )

    # Match Tahzeeb's own run_postprocessing.py sequence by keeping step-3 output in memory.
    step3 = load_module_from_path("tahzeeb_03_quantify_final", tahzeeb_dir / "03_quantify.py", tahzeeb_dir)
    step4 = load_module_from_path("tahzeeb_04_export_final", tahzeeb_dir / "04_export.py", tahzeeb_dir)
    summ = step3.quantify(runtime, objects.copy())
    viewer = step4.build_viewer_json(objects.copy(), summ.copy(), runtime)
    json_path.write_text(json.dumps(viewer, indent=2), encoding="utf-8")

    return {"mip_json": json_path.name}
