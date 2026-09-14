from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

# Shared GPRTools lives one level above Line_Segmentation.
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


def _gain_token(gain_db: float) -> str:
    sign = "p" if gain_db >= 0 else "m"
    value = f"{abs(gain_db):g}".replace(".", "d")
    return f"{sign}{value}dB"


def prepare_gain_only_bscan(
    sgy_path: Path,
    output_dir: Path,
    cfg: dict[str, Any],
) -> dict[str, Any]:
    """Create the line-detector input and an independent time-zero reference.

    Important design choice:
      * detection branch: raw SEGY -> gain only -> full untrimmed B-scan PNG
      * calibration branch: raw SEGY -> estimate time-zero index only

    The time-zero operation is never applied to the detector image. This avoids
    losing a shallow horizontal reflector before Berta's detector has seen it.
    """
    try:
        from GPRTools.edits.linear_gain import linear_gain_db
        from GPRTools.edits.time_zero import time_zero
        from GPRTools.info.segy_info import extract_metadata_segy_single
        from GPRTools.loaders.segy_loader import SegyLoader
        from GPRTools.plotters.plot_bscan import plot_bscan
    except ModuleNotFoundError as exc:
        if exc.name == "obspy":
            raise ModuleNotFoundError(
                "GPRTools SEGY loading requires 'obspy'. Install Line_Segmentation/requirements.txt first."
            ) from exc
        raise

    sgy_path = Path(sgy_path).resolve()
    if sgy_path.suffix.lower() not in {".sgy", ".segy"}:
        raise ValueError(f"Expected .sgy/.segy, got: {sgy_path}")
    if not sgy_path.exists():
        raise FileNotFoundError(sgy_path)

    sidecar = sgy_path.with_suffix(".csv")
    if not sidecar.exists():
        raise FileNotFoundError(
            f"GP8800 sidecar CSV not found: {sidecar}. It is required for the 12 ns calibration."
        )

    raw = np.asarray(SegyLoader(str(sgy_path)).load_matrix(), dtype=np.float64)
    if raw.ndim != 2:
        raise ValueError(f"SEGY loader returned shape {raw.shape}; expected a 2D B-scan")

    meta = extract_metadata_segy_single(str(sgy_path))
    corrected_tw_ns = meta.get("time_window")
    scan_distance_m = meta.get("trace_distance")
    if corrected_tw_ns is None:
        raise ValueError("GPRTools could not recover the corrected time window from the GP8800 CSV")

    expected_tw = cfg.get("calibration", {}).get("expected_corrected_time_window_ns")
    if expected_tw is not None and abs(float(corrected_tw_ns) - float(expected_tw)) > 1e-6:
        raise ValueError(
            f"Corrected time window is {corrected_tw_ns} ns, expected {expected_tw} ns. "
            "Depth conversion is blocked until this is resolved."
        )

    gain_db = float(cfg.get("preprocessing", {}).get("gain_db", 40.0))

    # Detection branch: gain ONLY. No time-zero crop, no background removal.
    gain_matrix = linear_gain_db(raw.copy(), gain_db)

    # Calibration branch: estimate time-zero on RAW data and keep only the index.
    tz_cfg = cfg.get("time_zero_reference", {})
    tz_sample, _ = time_zero(
        raw.copy(),
        method=tz_cfg.get("method", 2),
        threshold=float(tz_cfg.get("threshold", 0.2)),
        start_sample=int(tz_cfg.get("start_sample", 0)),
        backup_samples=int(tz_cfg.get("backup_samples", 0)),
    )

    images_dir = output_dir / "01_gain_only"
    images_dir.mkdir(parents=True, exist_ok=True)
    token = _gain_token(gain_db)
    image_name = f"{sgy_path.stem}_gain_{token}.png"
    image_path = images_dir / image_name
    plot_cfg = cfg.get("preprocessing", {}).get("plot", {})

    plot_bscan(
        self=None,
        M=gain_matrix,
        applied_edits=[f"gain({gain_db:g}dB)"],
        base_name=image_name,
        base_dir=str(images_dir),
        save=True,
        plot_scale=float(plot_cfg.get("plot_scale", 8.0)),
        aspect_ratio=plot_cfg.get("aspect_ratio"),
        plot_colormap=str(plot_cfg.get("colormap", "gray")),
        plot_axes=bool(plot_cfg.get("axes", False)),
        overwrite=True,
        loader_type="SEGY",
        src_file_path=str(sgy_path),
        render_upsample=float(plot_cfg.get("render_upsample", 3.0)),
    )

    with Image.open(image_path) as im:
        width_px, height_px = im.size

    n_samples, n_traces = raw.shape
    dt_ns = float(corrected_tw_ns) / float(n_samples)
    tz_rel = (float(tz_sample) / float(max(n_samples - 1, 1))) * 100.0

    return {
        "input_sgy": sgy_path.name,
        "sidecar_csv": sidecar.name,
        "gain_db": gain_db,
        "gain_only_image": image_path,
        "gain_only_image_relative": str(image_path.relative_to(output_dir)),
        "image_width_px": int(width_px),
        "image_height_px": int(height_px),
        "raw_n_samples": int(n_samples),
        "raw_n_traces": int(n_traces),
        "corrected_time_window_ns": float(corrected_tw_ns),
        "dt_ns_per_sample": float(dt_ns),
        "scan_distance_m": None if scan_distance_m is None else float(scan_distance_m),
        "time_zero_sample": int(tz_sample),
        "time_zero_relative": float(tz_rel),
        "detection_preprocessing": [f"gain({gain_db:g}dB)"],
        "time_zero_applied_to_detection_image": False,
        "background_removal_applied": False,
    }
