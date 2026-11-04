from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any

import pandas as pd

from .consensus import build_consensus
from .figures import draw_final_detections, draw_segmentation_masks, load_global_masks
from .inference_adapter import run_maskrcnn_inference, run_maskrcnn_inference_all_gains
from .outputs import build_software_result
from .postprocess_adapter import run_postprocessing_one_gain
from .preprocess import preprocess_segy, render_raw_bscan
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

    `output_dir` is the exact folder that receives the deliverables. When omitted it is
    `<paths.output_dir>/<scan>/hyperbola` (by default GPR/test_output/<scan>/hyperbola).
    The returned dict is the in-memory robot/software interface (also saved as gpr_result.json).
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

    if output_dir:
        run_dir = Path(output_dir).resolve()
    else:
        run_dir = resolve_from(config_path, cfg["paths"]["output_dir"]) / portable_scan_stem(input_sgy) / "hyperbola"
    run_dir.mkdir(parents=True, exist_ok=True)

    # A previous unsuccessful run may have left old crop manifests, tiles,
    # and inference results. Remove only pipeline-owned intermediate folders
    # for THIS scan before starting, otherwise the tiler can find multiple
    # crop_manifest_*.csv files and abort.
    # Keep final deliverables (gpr_data.json and the three PNG figures),
    # and never touch other scans or arbitrary files in the outputs folder.
    intermediate_dirs = (
        "01_preprocessed",
        "02_tiling",
        "03_inference",
        "04_postprocessing",
        "05_consensus",
        "tahzeeb_viz",
        "_workspace",
    )
    for dirname in intermediate_dirs:
        path = run_dir / dirname
        if path.is_dir():
            shutil.rmtree(path)
    (run_dir / "preprocess_manifest.json").unlink(missing_ok=True)

    keep_workspace = bool(cfg.get("outputs", {}).get("keep_workspace", False))
    # Process in the scan folder, as in the original Windows/network-share workflow.
    workspace_root = run_dir

    try:
        # 1) GPRTools preprocessing -> one full B-scan per gain.
        manifest = preprocess_segy(input_sgy, workspace_root, cfg)
        scan_id = manifest["model_stem"]
        element_id = manifest["element_id"]

        # 2) Tahzeeb v9 tiler, independently for each gain.
        tile_infos: dict[str, dict[str, Any]] = {}
        entries_by_token: dict[str, dict[str, Any]] = {}
        for entry in manifest["images"]:
            token = str(entry["gain_token"])
            entries_by_token[token] = entry
            image_path = workspace_root / entry["image_path"]
            tile_infos[token] = tile_one_gain(
                image_path=image_path,
                output_dir=workspace_root / "02_tiling" / token,
                tahzeeb_dir=tahzeeb_dir,
                cfg=cfg,
            )

        # 3) Model inference.
        if bool(cfg.get("inference", {}).get("batch_all_gains", True)):
            inference_by_token = run_maskrcnn_inference_all_gains(
                tile_infos=tile_infos,
                output_root=workspace_root / "03_inference",
                weights_path=weights,
                tahzeeb_dir=tahzeeb_dir,
                cfg=cfg,
            )
        else:
            inference_by_token = {}
            for token, tile_info in tile_infos.items():
                inference_by_token[token] = run_maskrcnn_inference(
                    tiles_dir=Path(tile_info["tiles_dir"]),
                    output_dir=workspace_root / "03_inference" / token,
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
                output_dir=workspace_root / "04_postprocessing" / token,
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

        # 5) Cross-gain consensus.
        kept_all = pd.concat(all_kept, ignore_index=True) if all_kept else pd.DataFrame()
        removed_all = pd.concat(all_removed, ignore_index=True) if all_removed else pd.DataFrame()
        gain_summary = pd.DataFrame(gain_summaries)

        accepted, audit, consensus_stats = build_consensus(
            kept_all,
            all_gains=[float(v) for v in manifest["gains_db"]],
            scan_distance_m=float(manifest["scan_distance_m"]),
            cfg=cfg,
        )

        # 6) In-memory robot-facing result (not persisted to disk unless requested later).
        result = build_software_result(scan_id, accepted, consensus_stats, manifest, cfg)
        output_files: dict[str, str] = {}

        # 6b) Figures, on the B-scan of the visualization gain.
        out_cfg = cfg.get("outputs", {})
        desired_gain = float(out_cfg.get("visualization_gain_db", 25))
        ref_entry = min(manifest["images"], key=lambda e: abs(float(e["gain_db"]) - desired_gain))
        ref_token = str(ref_entry["gain_token"])
        ref_tile = tile_infos[ref_token]
        ref_bscan = workspace_root / ref_entry["image_path"]
        ref_inference = inference_by_token[ref_token]
        masks = load_global_masks(ref_inference.get("masks_json"), Path(ref_tile["crop_manifest"]),
                                  ref_inference["detections"])
        title = f"{input_sgy.name}  ·  gain {float(ref_entry['gain_db']):g} dB"

        if bool(out_cfg.get("save_raw_image", True)):
            p = run_dir / "gpr_bscan_raw.png"
            render_raw_bscan(input_sgy, p, cfg)
            output_files["raw_bscan_image"] = p.name
        if bool(out_cfg.get("save_processed_image", True)):
            # Exactly the B-scan the model sees (GPRTools filters), nothing drawn on it.
            p = run_dir / "gpr_bscan_processed.png"
            shutil.copyfile(ref_bscan, p)
            output_files["processed_bscan_image"] = p.name
        if bool(out_cfg.get("save_mask_image", True)):
            p = run_dir / "gpr_segmentation_masks.png"
            draw_segmentation_masks(ref_bscan, masks, p, title)
            output_files["mask_image"] = p.name
        if bool(out_cfg.get("save_final_image", True)):
            p = run_dir / "gpr_final_detections.png"
            draw_final_detections(ref_bscan, masks, accepted, float(manifest["scan_distance_m"]),
                                  float(ref_entry["processed_time_window_ns"]), p, title)
            output_files["final_image"] = p.name

        # 6c) Unmodified acquisition files (Screening Eagle / GP8800 .sgy + sidecar .csv).
        if bool(out_cfg.get("copy_input_files", True)):
            for key, src in (("input_sgy", input_sgy), ("input_csv", input_sgy.with_suffix(".csv"))):
                dst = run_dir / src.name
                if dst.resolve() != src.resolve():
                    shutil.copy2(src, dst)
                output_files[key] = dst.name

        # 7) Tahzeeb/MIP compatibility chain: 03_quantify -> 04_export.
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

        # Only report the deliverables that are kept on disk.
        result["files"] = {
            key: value
            for key, value in output_files.items()
            if key in {"raw_bscan_image", "processed_bscan_image", "mask_image", "final_image", "mip_json",
                       "input_sgy", "input_csv"}
        }
        # The standardized project JSON is a separate deliverable, not defined yet:
        # it is NOT generated. gpr_data.json (Tahzeeb/MIP) is the internal JSON.
        # Remove legacy files from previous versions only after both deliverables exist.
        # Do not remove unrelated files placed by the user in this folder.
        if not keep_workspace:
            legacy_files = (
                "gpr_detections.csv", "gpr_results.xlsx",
                "gpr_consensus_audit.csv", "gpr_gain_summary.csv",
                "gpr_tahzeeb_removed.csv", "gpr_consensus_objects.csv",
                "element_summary.csv", "material_passport.csv",
                "tahzeeb_runtime_final.yaml", "tahzeeb_03_04_note.txt",
                "tahzeeb_05_visualize.log", "preprocess_manifest.json",
                "gpr_bscan.png",
            )
            legacy_dirs = (
                "01_preprocessed", "02_tiling", "03_inference",
                "04_postprocessing", "05_consensus", "tahzeeb_viz", "_workspace",
            )
            for filename in legacy_files:
                (run_dir / filename).unlink(missing_ok=True)
            for dirname in legacy_dirs:
                legacy_path = run_dir / dirname
                if legacy_path.is_dir():
                    shutil.rmtree(legacy_path)

        result["compatibility"] = {
            "mip_json": output_files.get("mip_json"),
            "note": (
                "gpr_data.json is generated with Tahzeeb 03_quantify/04_export for MIP compatibility. "
                "Its element/material/recovery interpretation is downstream logic and is not the robot-facing GPR sensor conclusion."
            ),
        }
        # Internal robot/POKEYE result. The standardized project JSON is a separate,
        # still undefined deliverable and is NOT generated here.
        if bool(out_cfg.get("save_robot_json", True)):
            result["files"]["robot_json"] = "gpr_result.json"
            json_dump(result, run_dir / "gpr_result.json")
        return result
    except Exception:
        # Keep intermediate files on failed runs for troubleshooting. Never mask the real error.
        raise
