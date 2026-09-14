from __future__ import annotations

from pathlib import Path
from typing import Any

import pandas as pd

from .consensus import build_consensus
from .inference_adapter import run_maskrcnn_inference, run_maskrcnn_inference_all_gains
from .outputs import build_software_result, draw_final_visualization, write_output_files
from .postprocess_adapter import run_postprocessing_one_gain
from .preprocess import preprocess_segy
from .tahzeeb_export_adapter import run_tahzeeb_final_exports
from .tiler_adapter import tile_one_gain
from .utils import json_dump, load_yaml, portable_scan_stem, resolve_from


def run_gpr_pipeline(
    input_sgy: str | Path,
    config_path: str | Path | None = None,
    output_dir: str | Path | None = None,
    weights_path: str | Path | None = None,
    tahzeeb_config_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run GPRTools -> Tahzeeb per-gain processing -> cross-gain consensus.

    The returned dict / gpr_result.json is the intended robot/software interface.
    Tahzeeb's gpr_data.json is generated separately as an MIP compatibility output.
    ROS2 transport is deliberately outside this function.
    """
    input_sgy = Path(input_sgy).resolve()
    if not input_sgy.exists():
        raise FileNotFoundError(input_sgy)
    default_cfg = Path(__file__).resolve().parent / "config.yaml"
    config_path = Path(config_path).resolve() if config_path else default_cfg
    cfg = load_yaml(config_path)

    tahzeeb_dir = resolve_from(config_path, cfg["paths"]["tahzeeb_dir"])
    weights = Path(weights_path).resolve() if weights_path else resolve_from(config_path, cfg["paths"]["weights"])
    tz_cfg = Path(tahzeeb_config_path).resolve() if tahzeeb_config_path else resolve_from(config_path, cfg["paths"]["tahzeeb_config"])
    if not weights.exists():
        raise FileNotFoundError(f"Missing Tahzeeb Mask R-CNN ResNet101 weights: {weights}")
    if not tz_cfg.exists():
        raise FileNotFoundError(f"Missing Tahzeeb post-processing config.yaml: {tz_cfg}")

    output_root = Path(output_dir).resolve() if output_dir else resolve_from(config_path, cfg["paths"]["output_dir"])
    run_dir = output_root / portable_scan_stem(input_sgy)
    run_dir.mkdir(parents=True, exist_ok=True)

    # 1) GPRTools preprocessing -> one full B-scan per gain.
    manifest = preprocess_segy(input_sgy, run_dir, cfg)
    scan_id = manifest["model_stem"]
    element_id = manifest["element_id"]

    # 2) Tahzeeb v9 tiler, independently for each gain.
    tile_infos: dict[str, dict[str, Any]] = {}
    entries_by_token: dict[str, dict[str, Any]] = {}
    for entry in manifest["images"]:
        token = str(entry["gain_token"])
        entries_by_token[token] = entry
        image_path = run_dir / entry["image_path"]
        tile_infos[token] = tile_one_gain(
            image_path=image_path,
            output_dir=run_dir / "02_tiling" / token,
            tahzeeb_dir=tahzeeb_dir,
            cfg=cfg,
        )

    # 3) Model inference. Prefer one model load across all gains; detections are split
    # back into isolated per-gain CSV/JSON before Tahzeeb 01_consolidate.
    if bool(cfg.get("inference", {}).get("batch_all_gains", True)):
        inference_by_token = run_maskrcnn_inference_all_gains(
            tile_infos=tile_infos,
            output_root=run_dir / "03_inference",
            weights_path=weights,
            tahzeeb_dir=tahzeeb_dir,
            cfg=cfg,
        )
    else:
        inference_by_token = {}
        for token, tile_info in tile_infos.items():
            inference_by_token[token] = run_maskrcnn_inference(
                tiles_dir=Path(tile_info["tiles_dir"]),
                output_dir=run_dir / "03_inference" / token,
                weights_path=weights,
                tahzeeb_dir=tahzeeb_dir,
                cfg=cfg,
            )

    # 4) Tahzeeb 01 -> 02 -> 02b independently per gain.
    all_kept: list[pd.DataFrame] = []
    all_removed: list[pd.DataFrame] = []
    gain_summaries: list[dict[str, Any]] = []
    for entry in manifest["images"]:
        gain_db = float(entry["gain_db"])
        token = str(entry["gain_token"])
        tile_info = tile_infos[token]
        inference = inference_by_token[token]
        post = run_postprocessing_one_gain(
            gain_db=gain_db,
            detections_csv=Path(inference["detections_csv"]),
            masks_json=Path(inference["masks_json"]) if inference.get("masks_json") else None,
            crop_manifest=Path(tile_info["crop_manifest"]),
            output_dir=run_dir / "04_postprocessing" / token,
            tahzeeb_dir=tahzeeb_dir,
            tahzeeb_config_path=tz_cfg,
            element_id=element_id,
            scan_distance_m=float(entry["scan_distance_m"]),
            processed_time_window_ns=float(entry["processed_time_window_ns"]),
            tile_info=tile_info,
            integration_cfg=cfg,
        )
        if not post["kept"].empty:
            all_kept.append(post["kept"])
        if not post["removed"].empty:
            all_removed.append(post["removed"])
        gain_summaries.append({
            "gain_db": gain_db,
            "n_tiles": int(tile_info["n_tiles"]),
            "n_raw_model_detections": len(inference["detections"]),
            "n_consolidated": post["audit"]["n_consolidated"],
            "n_after_tahzeeb_dedup": post["audit"]["n_objects_after_dedup"],
            "n_removed_by_tahzeeb_dedup": post["audit"]["n_removed_by_dedup"],
        })

    # 5) Cross-gain consensus on already physical/de-duplicated Tahzeeb objects.
    kept_all = pd.concat(all_kept, ignore_index=True) if all_kept else pd.DataFrame()
    removed_all = pd.concat(all_removed, ignore_index=True) if all_removed else pd.DataFrame()
    gain_summary = pd.DataFrame(gain_summaries)
    consensus_dir = run_dir / "05_consensus"
    consensus_dir.mkdir(parents=True, exist_ok=True)
    kept_all.to_csv(consensus_dir / "per_gain_objects.csv", index=False)

    accepted, audit, consensus_stats = build_consensus(
        kept_all,
        all_gains=[float(v) for v in manifest["gains_db"]],
        scan_distance_m=float(manifest["scan_distance_m"]),
        cfg=cfg,
    )
    accepted.to_csv(consensus_dir / "accepted_detections.csv", index=False)
    audit.to_csv(consensus_dir / "consensus_audit.csv", index=False)

    # 6) Robot-facing output.
    result = build_software_result(scan_id, accepted, consensus_stats, manifest, cfg)
    output_files = write_output_files(run_dir, result, accepted, audit, gain_summary, removed_all, cfg)

    if bool(cfg.get("outputs", {}).get("save_final_image", True)):
        p = run_dir / "gpr_final_detections.png"
        draw_final_visualization(run_dir, manifest, accepted, cfg, p)
        output_files["final_image"] = p.name

    # 7) Tahzeeb/MIP compatibility chain: 03_quantify -> 04_export (+05 visualization)
    # runs only after consensus, never per gain.
    desired_gain = float(cfg.get("outputs", {}).get("visualization_gain_db", 25))
    ref_entry = min(manifest["images"], key=lambda e: abs(float(e["gain_db"]) - desired_gain))
    ref_token = str(ref_entry["gain_token"])
    ref_tile = tile_infos[ref_token]
    ref_bscan = run_dir / ref_entry["image_path"]
    tahzeeb_files = run_tahzeeb_final_exports(
        accepted=accepted,
        run_dir=run_dir,
        tahzeeb_dir=tahzeeb_dir,
        tahzeeb_config_path=tz_cfg,
        integration_cfg=cfg,
        element_id=element_id,
        scan_distance_m=float(manifest["scan_distance_m"]),
        processed_time_window_ns=float(ref_entry["processed_time_window_ns"]),
        image_width_px=int(ref_tile["source_w"]),
        image_height_px=int(ref_tile["work_h"]),
        reference_bscan=ref_bscan,
    )
    output_files.update(tahzeeb_files)

    result["files"] = output_files
    result["compatibility"] = {
        "mip_json": output_files.get("mip_json"),
        "note": (
            "gpr_data.json is generated with Tahzeeb 03_quantify/04_export for MIP compatibility. "
            "Its element/material/recovery interpretation is downstream logic and is not the robot-facing GPR sensor conclusion."
        ),
    }
    if bool(cfg.get("outputs", {}).get("save_json", True)):
        json_dump(result, run_dir / "gpr_result.json")
    return result
