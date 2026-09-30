from __future__ import annotations

from pathlib import Path
from typing import Any
import sys

import numpy as np
from PIL import Image

# GPRTools is shared at the parent project level:
# GPR_DISCOVER_PIPELINE_v4/GPRTools
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from GPRTools.edits.background_removal import background_removal
from GPRTools.edits.linear_gain import linear_gain_db
from GPRTools.edits.time_zero import time_zero
from GPRTools.info.segy_info import extract_metadata_segy_single
from GPRTools.loaders.segy_loader import SegyLoader
from GPRTools.plotters.plot_bscan import plot_bscan

from .utils import gain_token, json_dump, portable_scan_stem, scan_element_id


def preprocess_segy(
    sgy_path: Path,
    output_dir: Path,
    cfg: dict[str, Any],
) -> dict[str, Any]:
    """Create one axis-free grayscale full B-scan per configured gain using GPRTools."""
    if sgy_path.suffix.lower() not in {".sgy", ".segy"}:
        raise ValueError(f"Expected .sgy/.segy, got {sgy_path}")

    sidecar = sgy_path.with_suffix(".csv")
    if not sidecar.exists():
        raise FileNotFoundError(
            f"GP8800 sidecar CSV not found: {sidecar}. It is required for scan distance and time calibration."
        )

    plot_cfg = cfg["preprocessing"].get("plot", {})
    render_upsample = plot_cfg.get("render_upsample")
    if render_upsample is None:
        raise RuntimeError(
            "preprocessing.plot.render_upsample is not set. Ask Tahzeeb how the full B-scan images used "
            "for Mask R-CNN training/inference were rendered (especially vertical resolution/scale), "
            "then set this value explicitly."
        )

    raw = SegyLoader(str(sgy_path)).load_matrix()
    if raw.ndim != 2:
        raise ValueError(f"SEGY loader returned {raw.shape}; expected a 2D B-scan")

    meta = extract_metadata_segy_single(str(sgy_path))
    scan_distance_m = meta.get("trace_distance")
    corrected_time_window_ns = meta.get("time_window")
    if scan_distance_m is None or corrected_time_window_ns is None:
        raise ValueError("GPRTools could not recover scan distance and corrected time window from the sidecar CSV")

    expected = cfg.get("metadata", {}).get("expected_corrected_time_window_ns")
    if expected is not None and abs(float(corrected_time_window_ns) - float(expected)) > 1e-6:
        raise ValueError(
            f"Corrected GP8800 time window is {corrected_time_window_ns} ns, expected {expected} ns. "
            "Depth calibration is intentionally blocked until this is resolved."
        )

    pp = cfg["preprocessing"]
    tz_cfg = pp.get("time_zero", {})
    order = list(pp.get("edits_order", ["time_zero", "background_removal", "gain"]))
    gains = [float(v) for v in pp.get("gains_db", [])]
    if not gains:
        raise ValueError("preprocessing.gains_db is empty")

    model_stem = portable_scan_stem(sgy_path)
    element_id = scan_element_id(model_stem)
    images_dir = output_dir / "01_preprocessed"
    images_dir.mkdir(parents=True, exist_ok=True)

    entries: list[dict[str, Any]] = []
    tz_offsets: set[int] = set()
    for gain_db in gains:
        matrix = np.asarray(raw, dtype=np.float64).copy()
        applied: list[str] = []
        tz_offset = 0
        for step in order:
            if step == "time_zero":
                tz, matrix = time_zero(
                    matrix,
                    method=tz_cfg.get("method", 2),
                    threshold=float(tz_cfg.get("threshold", 0.2)),
                    start_sample=int(tz_cfg.get("start_sample", 0)),
                    backup_samples=int(tz_cfg.get("backup_samples", 0)),
                )
                tz_offset += int(tz)
                applied.append(f"time_zero({tz})")
            elif step == "background_removal":
                matrix = background_removal(matrix)
                applied.append("background_removal")
            elif step == "gain":
                matrix = linear_gain_db(matrix, gain_db)
                applied.append(f"gain({gain_db:g}dB)")
            else:
                raise ValueError(f"Unsupported preprocessing step: {step}")
        tz_offsets.add(tz_offset)

        image_name = f"{model_stem}_gain_{gain_token(gain_db)}.png"
        image_path = images_dir / image_name
        plot_bscan(
            self=None,
            M=matrix,
            applied_edits=applied,
            base_name=image_name,
            base_dir=str(images_dir),
            save=True,
            plot_scale=float(plot_cfg.get("plot_scale", 2.0)),
            aspect_ratio=plot_cfg.get("aspect_ratio"),
            plot_colormap=str(plot_cfg.get("colormap", "gray")),
            plot_axes=bool(plot_cfg.get("axes", False)),
            overwrite=True,
            loader_type="SEGY",
            src_file_path=str(sgy_path),
            render_upsample=float(render_upsample),
        )
        with Image.open(image_path) as im:
            width_px, height_px = im.size

        dt_raw_ns = float(corrected_time_window_ns) / float(raw.shape[0])
        processed_tw_ns = dt_raw_ns * float(matrix.shape[0])
        entries.append({
            "gain_db": gain_db,
            "gain_token": gain_token(gain_db),
            "image": image_name,
            "image_path": str(image_path.relative_to(output_dir)),
            "width_px": int(width_px),
            "height_px": int(height_px),
            "matrix_samples": int(matrix.shape[0]),
            "matrix_traces": int(matrix.shape[1]),
            "time_zero_samples": int(tz_offset),
            "processed_time_window_ns": float(processed_tw_ns),
            "scan_distance_m": float(scan_distance_m),
            "applied_edits": applied,
        })

    if len(tz_offsets) != 1:
        raise RuntimeError(f"Time-zero changed across gains unexpectedly: {sorted(tz_offsets)}")

    manifest = {
        "input_sgy": sgy_path.name,
        "sidecar_csv": sidecar.name,
        "model_stem": model_stem,
        "element_id": element_id,
        "raw_shape": [int(raw.shape[0]), int(raw.shape[1])],
        "scan_distance_m": float(scan_distance_m),
        "corrected_time_window_ns": float(corrected_time_window_ns),
        "time_window_note": cfg.get("metadata", {}).get("time_window_note"),
        "gains_db": gains,
        "images": entries,
    }
    json_dump(manifest, output_dir / "preprocess_manifest.json")
    return manifest
