from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pandas as pd

from .utils import json_dump, load_module_from_path, load_yaml

_OBJECT_COLUMNS = [
    "element", "x_m", "apex_px", "twt_ns", "v_m_ns", "eps_r",
    "depth_cm", "confidence", "arc_hw_m",
]


def _runtime_config(
    base_cfg: dict[str, Any],
    detections_csv: Path,
    masks_json: Path | None,
    crop_manifest: Path,
    post_dir: Path,
    element_id: str,
    scan_distance_m: float,
    processed_time_window_ns: float,
    tile_info: dict[str, Any],
    integration_cfg: dict[str, Any],
) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    for required in ("io", "calibration", "consolidate", "tiling", "dedup"):
        if required not in cfg:
            raise KeyError(f"Tahzeeb config.yaml is missing required section: {required}")

    cfg["io"]["detections_csv"] = str(detections_csv)
    cfg["io"]["crop_manifest"] = str(crop_manifest)
    cfg["io"]["masks_file"] = str(masks_json) if masks_json is not None else None
    cfg["io"]["results_dir"] = str(post_dir)

    cal = cfg["calibration"]
    cal["px_per_m"] = float(tile_info["source_w"]) / float(scan_distance_m)
    cal["scan_distance_m"] = {element_id: float(scan_distance_m)}
    cal["time_window_ns"] = float(processed_time_window_ns)
    # Tiler coordinates are in post-vscale working pixels.
    cal["image_height_px"] = int(tile_info["work_h"])
    cal["epsilon_r"] = float(integration_cfg["postprocessing"]["epsilon_r_assumed"])

    # Keep all Tahzeeb thresholds/calibrations that we were not explicitly asked
    # to change, including apex_frac, pulse_offset_samples and dedup parameters.
    cfg["tiling"]["tile_size"] = int(tile_info["tile_size"])
    return cfg


def run_postprocessing_one_gain(
    gain_db: float,
    detections_csv: Path,
    masks_json: Path | None,
    crop_manifest: Path,
    output_dir: Path,
    tahzeeb_dir: Path,
    tahzeeb_config_path: Path,
    element_id: str,
    scan_distance_m: float,
    processed_time_window_ns: float,
    tile_info: dict[str, Any],
    integration_cfg: dict[str, Any],
) -> dict[str, Any]:
    """Run Tahzeeb 01 -> 02 -> 02b, unchanged, for one gain only."""
    if not tahzeeb_config_path.exists():
        raise FileNotFoundError(
            f"Tahzeeb post-processing config is missing: {tahzeeb_config_path}. "
            "The supplied archive references config.yaml but did not include it."
        )
    base_cfg = load_yaml(tahzeeb_config_path)
    runtime_cfg = _runtime_config(
        base_cfg, detections_csv, masks_json, crop_manifest, output_dir,
        element_id, scan_distance_m, processed_time_window_ns, tile_info, integration_cfg,
    )

    step1 = load_module_from_path("tahzeeb_01_consolidate", tahzeeb_dir / "01_consolidate.py", tahzeeb_dir)
    step2 = load_module_from_path("tahzeeb_02_hyperbola_fit", tahzeeb_dir / "02_hyperbola_fit.py", tahzeeb_dir)
    step2b = load_module_from_path("tahzeeb_02b_dedup", tahzeeb_dir / "02b_dedup.py", tahzeeb_dir)

    output_dir.mkdir(parents=True, exist_ok=True)
    cons = step1.consolidate(runtime_cfg)
    cons.to_csv(output_dir / "consolidated.csv", index=False)

    if cons.empty:
        objects = pd.DataFrame(columns=_OBJECT_COLUMNS)
        kept = objects.copy()
        removed = objects.assign(reason=pd.Series(dtype=str))
    else:
        objects = step2.compute(runtime_cfg, cons.copy())
        kept, removed = step2b.run(runtime_cfg, objects.copy())
    objects.to_csv(output_dir / "objects.csv", index=False)
    kept.to_csv(output_dir / "objects_dedup.csv", index=False)
    removed.to_csv(output_dir / "objects_dedup_removed.csv", index=False)

    if not kept.empty:
        kept = kept.copy()
        kept["gain_db"] = float(gain_db)
    if not removed.empty:
        removed = removed.copy()
        removed["gain_db"] = float(gain_db)

    audit = {
        "gain_db": float(gain_db),
        "raw_detections_csv": str(detections_csv.name),
        "n_consolidated": int(len(cons)),
        "n_objects_before_dedup": int(len(objects)),
        "n_objects_after_dedup": int(len(kept)),
        "n_removed_by_dedup": int(len(removed)),
        "calibration": {
            "scan_distance_m": float(scan_distance_m),
            "time_window_ns_after_time_zero": float(processed_time_window_ns),
            "image_height_px_working": int(tile_info["work_h"]),
            "px_per_m": float(tile_info["source_w"]) / float(scan_distance_m),
            "epsilon_r_assumed": float(integration_cfg["postprocessing"]["epsilon_r_assumed"]),
            "epsilon_r_basis": integration_cfg["postprocessing"].get("epsilon_r_basis"),
            "pulse_offset_samples_from_tahzeeb_config": runtime_cfg["calibration"].get("pulse_offset_samples"),
        },
    }
    json_dump(audit, output_dir / "postprocess_audit.json")
    return {"kept": kept, "removed": removed, "audit": audit}
