from __future__ import annotations

import csv
import json
import shutil
from pathlib import Path
from typing import Any

from .utils import load_module_from_path

_DET_FIELDS = ["image", "class", "confidence", "x1", "y1", "x2", "y2", "inference_ms"]


def _write_detections(path: Path, detections: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(detections[0].keys()) if detections else _DET_FIELDS
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        if detections:
            writer.writerows(detections)


def _validate_inference_config(weights_path: Path, cfg: dict[str, Any]) -> dict[str, Any]:
    if not weights_path.exists():
        raise FileNotFoundError(f"Tahzeeb model weights are missing: {weights_path}")
    inf = cfg.get("inference", {})
    if str(inf.get("model_type", "maskrcnn")).lower() != "maskrcnn":
        raise ValueError("This DISCOVER integration is intentionally fixed to Tahzeeb's Mask R-CNN model")
    if str(inf.get("backbone", "resnet101")).lower() != "resnet101":
        raise ValueError("Tahzeeb confirmed maskrcnn_resnet101; inference.backbone must be resnet101")
    return inf


def run_maskrcnn_inference(
    tiles_dir: Path,
    output_dir: Path,
    weights_path: Path,
    tahzeeb_dir: Path,
    cfg: dict[str, Any],
) -> dict[str, Any]:
    """Run Tahzeeb's unmodified Mask R-CNN inference function for one tile folder."""
    inf = _validate_inference_config(weights_path, cfg)
    module = load_module_from_path("tahzeeb_inference_gpr", tahzeeb_dir / "inference_gpr.py")
    output_dir.mkdir(parents=True, exist_ok=True)
    detections = module.run_maskrcnn_inference(
        weights=weights_path,
        source=tiles_dir,
        output=output_dir,
        backbone="resnet101",
        conf_thresh=float(inf.get("confidence_threshold", 0.25)),
        device_str=inf.get("device"),
        save_masks=bool(inf.get("save_masks", True)),
    )
    det_csv = output_dir / "detections.csv"
    _write_detections(det_csv, detections)
    masks_path = output_dir / "masks.json"
    if bool(inf.get("save_masks", True)) and not masks_path.exists():
        raise RuntimeError("Mask R-CNN inference did not produce masks.json; ridge-based fitting cannot be reproduced")
    return {
        "detections": detections,
        "detections_csv": det_csv,
        "masks_json": masks_path if masks_path.exists() else None,
    }


def run_maskrcnn_inference_all_gains(
    tile_infos: dict[str, dict[str, Any]],
    output_root: Path,
    weights_path: Path,
    tahzeeb_dir: Path,
    cfg: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    """Run Tahzeeb inference once across all gain tiles, then split outputs by gain.

    The original inference function instantiates and loads the ~241 MB ResNet101 model on
    every call. Loading it ten times adds substantial demo latency. This adapter leaves
    Tahzeeb's source untouched: it flattens the already-created tiles into one temporary
    folder, calls his function once, then splits detections/masks back into isolated per-gain
    files before any post-processing. Thus 01_consolidate never sees multiple gains together.
    """
    inf = _validate_inference_config(weights_path, cfg)
    module = load_module_from_path("tahzeeb_inference_gpr_all", tahzeeb_dir / "inference_gpr.py")
    output_root.mkdir(parents=True, exist_ok=True)
    combined_tiles = output_root / "_all_tiles"
    combined_out = output_root / "_all_inference"
    if combined_tiles.exists():
        shutil.rmtree(combined_tiles)
    if combined_out.exists():
        shutil.rmtree(combined_out)
    combined_tiles.mkdir(parents=True, exist_ok=True)
    combined_out.mkdir(parents=True, exist_ok=True)

    gain_tile_names: dict[str, set[str]] = {}
    for token, info in tile_infos.items():
        names: set[str] = set()
        for src in sorted(Path(info["tiles_dir"]).glob("*.png")):
            dst = combined_tiles / src.name
            if dst.exists():
                raise RuntimeError(f"Tile filename collision across gains: {src.name}")
            shutil.copy2(src, dst)
            names.add(src.name)
        gain_tile_names[token] = names

    detections = module.run_maskrcnn_inference(
        weights=weights_path,
        source=combined_tiles,
        output=combined_out,
        backbone="resnet101",
        conf_thresh=float(inf.get("confidence_threshold", 0.25)),
        device_str=inf.get("device"),
        save_masks=bool(inf.get("save_masks", True)),
    )
    _write_detections(combined_out / "detections.csv", detections)

    masks: list[dict[str, Any]] = []
    combined_masks = combined_out / "masks.json"
    if bool(inf.get("save_masks", True)):
        if not combined_masks.exists():
            raise RuntimeError("Mask R-CNN inference did not produce masks.json")
        masks = json.loads(combined_masks.read_text(encoding="utf-8"))

    out: dict[str, dict[str, Any]] = {}
    for token, names in gain_tile_names.items():
        gain_dir = output_root / token
        gain_dir.mkdir(parents=True, exist_ok=True)
        det_subset = [d for d in detections if str(d.get("image", "")) in names]
        mask_subset = [m for m in masks if str(m.get("image", "")) in names]
        det_csv = gain_dir / "detections.csv"
        _write_detections(det_csv, det_subset)
        masks_path = gain_dir / "masks.json"
        if bool(inf.get("save_masks", True)):
            masks_path.write_text(json.dumps(mask_subset), encoding="utf-8")
        out[token] = {
            "detections": det_subset,
            "detections_csv": det_csv,
            "masks_json": masks_path if bool(inf.get("save_masks", True)) else None,
            "combined_inference_dir": combined_out,
        }
    return out
