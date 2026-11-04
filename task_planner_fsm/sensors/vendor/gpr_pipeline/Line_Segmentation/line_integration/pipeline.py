from __future__ import annotations

import importlib.util
import json
import math
import re
import shutil
from pathlib import Path
from typing import Any

import yaml

from .figures import draw_depth_figure
from .preprocess import prepare_gain_only_bscan, render_raw_bscan


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _load_berta_module(project_root: Path):
    src = project_root / "berta_original" / "line_segmentation.py"
    if not src.exists():
        raise FileNotFoundError(f"Berta original code not found: {src}")
    spec = importlib.util.spec_from_file_location("berta_line_segmentation", src)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot import Berta code from: {src}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def portable_scan_stem(path: str | Path) -> str:
    """Scan folder name. Must match Hyperbola_Segmentation's gpr_integration.utils.portable_scan_stem
    so that both branches write into the same GPR/test_output/<scan>/ folder."""
    stem = re.sub(r"[^A-Za-z0-9_-]+", "_", Path(path).stem).strip("_") or "gpr_scan"
    if not re.search(r"_20\d{6}", stem):
        stem += "_20260101"
    return stem


def _json_dump(data: Any, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _finite_or_none(value: Any) -> float | None:
    if value in (None, ""):
        return None
    v = float(value)
    return v if math.isfinite(v) else None


def _physical_calibration(prep: dict[str, Any], cfg: dict[str, Any]) -> dict[str, float | str]:
    cal = cfg.get("calibration", {})
    eps_r = float(cal.get("epsilon_r_assumed", 6.0))
    c_m_ns = float(cal.get("c_m_per_ns", 0.299792458))
    velocity = c_m_ns / math.sqrt(eps_r)
    dt_ns = float(prep["dt_ns_per_sample"])
    depth_cm_per_sample = velocity * dt_ns * 0.5 * 100.0
    return {
        "epsilon_r_assumed": eps_r,
        "epsilon_r_basis": str(cal.get("epsilon_r_basis", "assumed")),
        "velocity_m_ns": velocity,
        "dt_ns_per_sample": dt_ns,
        "depth_cm_per_sample": depth_cm_per_sample,
        "time_zero_depth_tolerance_samples": float(cal.get("time_zero_depth_tolerance_samples", 1.0)),
    }


def _augment_line_physics(
    line: dict[str, Any],
    index: int,
    prep: dict[str, Any],
    phys: dict[str, Any],
) -> dict[str, Any]:
    n_samples = int(prep["raw_n_samples"])
    tz_sample = float(prep["time_zero_sample"])
    rel = float(line["y_relativa"])

    # Berta coordinates are 0..100 over the full, untrimmed B-scan. The rendered
    # image is a uniform mapping of the raw sample axis, therefore the relative
    # coordinate can be mapped directly back to the raw sample axis.
    sample = (rel / 100.0) * float(max(n_samples - 1, 1))
    delta_samples = sample - tz_sample
    twt_ns = delta_samples * float(phys["dt_ns_per_sample"])
    signed_depth_cm = delta_samples * float(phys["depth_cm_per_sample"])

    tolerance_samples = float(phys.get("time_zero_depth_tolerance_samples", 1.0))
    before_t0 = delta_samples < -tolerance_samples
    near_t0 = abs(delta_samples) <= tolerance_samples
    depth_cm = None if before_t0 else max(0.0, signed_depth_cm)

    thickness_samples = (
        float(line["grosor_relativo"]) / 100.0 * float(max(n_samples - 1, 1))
    )
    thickness_cm = thickness_samples * float(phys["depth_cm_per_sample"])

    dist_rel = _finite_or_none(line.get("distancia_siguiente_relativa"))
    if dist_rel is None:
        dist_samples = None
        dist_cm = None
    else:
        dist_samples = dist_rel / 100.0 * float(max(n_samples - 1, 1))
        dist_cm = dist_samples * float(phys["depth_cm_per_sample"])

    # Bounds are also translated for auditability. A bound before time-zero gets
    # a signed coordinate; it is not silently clamped to zero.
    top_sample = float(line["region_y0"]) / float(max(int(prep["image_height_px"]) - 1, 1)) * float(max(n_samples - 1, 1))
    bottom_sample = float(line["region_y1"]) / float(max(int(prep["image_height_px"]) - 1, 1)) * float(max(n_samples - 1, 1))

    return {
        "id": f"L{index:03d}",
        "position_relative": round(rel, 6),
        "position_sample_raw": round(sample, 3),
        "offset_from_time_zero_samples": round(delta_samples, 3),
        "twt_from_time_zero_ns": round(twt_ns, 6),
        "depth_cm_approx": None if depth_cm is None else round(depth_cm, 4),
        "position_from_time_zero_cm_signed": round(signed_depth_cm, 4),
        "before_time_zero_reference": bool(before_t0),
        "near_time_zero_reference": bool(near_t0),
        "thickness_relative": round(float(line["grosor_relativo"]), 6),
        "thickness_cm_approx": round(thickness_cm, 4),
        "distance_to_next_relative": None if dist_rel is None else round(dist_rel, 6),
        "distance_to_next_cm_approx": None if dist_cm is None else round(dist_cm, 4),
        "confidence": round(float(line["confianza"]), 6),
        "angle_deg": round(float(line["angulo"]), 6),
        "support": round(float(line["soporte"]), 6),
        "strength": round(float(line["fuerza"]), 6),
        "position_px_rendered": int(line["y"]),
        "thickness_px_rendered": int(line["grosor_px"]),
        "region_top_from_time_zero_cm_signed": round((top_sample - tz_sample) * float(phys["depth_cm_per_sample"]), 4),
        "region_bottom_from_time_zero_cm_signed": round((bottom_sample - tz_sample) * float(phys["depth_cm_per_sample"]), 4),
    }


def run_line_pipeline(
    input_sgy: str | Path,
    output_dir: str | Path | None = None,
    config_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run Task 1b from raw GP8800 SEGY to robot-facing line results.

    Detection deliberately uses a gain-only, untrimmed B-scan. Time-zero is
    estimated separately and merged AFTER detection to obtain approximate cm.
    ROS2 transport is outside this function; the returned dict is serializable.

    `output_dir` is the exact folder that receives the deliverables. When omitted it is
    `<outputs.output_dir>/<scan>/line` (by default GPR/test_output/<scan>/line).
    """
    input_sgy = Path(input_sgy).expanduser().resolve()
    project_root = Path(__file__).resolve().parents[1]
    cfg_path = Path(config_path).expanduser().resolve() if config_path else project_root / "config.yaml"
    cfg = _load_yaml(cfg_path)

    if output_dir is None:
        output_root = project_root / cfg.get("outputs", {}).get("output_dir", "../../test_output")
        output_dir = (output_root / portable_scan_stem(input_sgy) / "line").resolve()
    else:
        output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    prep = prepare_gain_only_bscan(input_sgy, output_dir, cfg)

    berta = _load_berta_module(project_root)
    image_path = Path(prep["gain_only_image"])
    lines, gray = berta.detectar_lineas_horizontales(image_path)
    height, width = gray.shape
    lines = berta.añadir_metricas_relativas(lines, height)

    phys = _physical_calibration(prep, cfg)
    result_lines = [
        _augment_line_physics(line, i, prep, phys)
        for i, line in enumerate(lines, start=1)
    ]

    result = {
        "detected": bool(lines),
        "n_lines": len(lines),
        "lines": result_lines,
        "source": {
            "sgy": input_sgy.name,
            "sidecar_csv": prep["sidecar_csv"],
            "gain_only_bscan": "line_bscan_processed.png",
            "gain_db": prep["gain_db"],
            "raw_n_samples": prep["raw_n_samples"],
            "raw_n_traces": prep["raw_n_traces"],
            "corrected_time_window_ns": prep["corrected_time_window_ns"],
            "scan_distance_m": prep["scan_distance_m"],
        },
        "time_zero_reference": {
            "sample": prep["time_zero_sample"],
            "relative_0_100": round(float(prep["time_zero_relative"]), 6),
            "applied_before_detection": False,
            "strategy": "detect on full gain-only B-scan; estimate time-zero independently on raw data; merge coordinates afterwards",
        },
        "depth_calibration": {
            "epsilon_r_assumed": phys["epsilon_r_assumed"],
            "epsilon_r_basis": phys["epsilon_r_basis"],
            "velocity_m_ns": round(float(phys["velocity_m_ns"]), 8),
            "dt_ns_per_sample": round(float(phys["dt_ns_per_sample"]), 8),
            "depth_cm_per_sample": round(float(phys["depth_cm_per_sample"]), 8),
            "time_zero_depth_tolerance_samples": float(phys["time_zero_depth_tolerance_samples"]),
            "formula": "depth = (c/sqrt(epsilon_r)) * two_way_time / 2",
            "warning": "Approximate depth under assumed epsilon_r=6. Line-band thickness is a radar-image/time extent, not a validated physical object thickness.",
        },
        "preprocessing": {
            "detector_image": "gain only",
            "background_removal": False,
            "time_zero_crop": False,
        },
    }
    # Deliverables: raw and processed B-scans, depth figure and the internal robot/POKEYE JSON.
    # The global JSON for Tecnalia is built from line_result.json by FULL PIPELINE/Global JSON.
    out_cfg = cfg.get("outputs", {})
    files: dict[str, str] = {}
    if bool(out_cfg.get("save_raw_image", True)):
        render_raw_bscan(input_sgy, prep["raw_matrix"], output_dir / "line_bscan_raw.png", cfg)
        files["raw_bscan_image"] = "line_bscan_raw.png"
    if bool(out_cfg.get("save_processed_image", True)):
        # Exactly the image Berta's detector sees (gain only), nothing drawn on it.
        shutil.copyfile(image_path, output_dir / "line_bscan_processed.png")
        files["processed_bscan_image"] = "line_bscan_processed.png"
    if bool(out_cfg.get("save_depth_figure", True)):
        draw_depth_figure(prep, lines, result, float(phys["velocity_m_ns"]), output_dir / "line_bscan_depth.png")
        files["depth_figure"] = "line_bscan_depth.png"
    files["robot_json"] = "line_result.json"
    result["files"] = files
    _json_dump(result, output_dir / "line_result.json")

    # Keep processing files only on failure; delete them after successful completion.
    # Preprocessing created 01_gain_only in the original output folder; no temporary
    # random directory or network move is used (important for Windows shares).
    for folder in ("01_gain_only", "02_line_detection", "03_time_zero_merge"):
        directory = output_dir / folder
        if directory.is_dir():
            shutil.rmtree(directory)
    for obsolete in ("preprocessing_manifest.json", "run_manifest.json",
                     "results_finales.csv", "resultados_lineas.csv", "resultados_resumen.csv",
                     "line_final_detections.png"):
        (output_dir / obsolete).unlink(missing_ok=True)
    return result
