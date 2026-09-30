from __future__ import annotations

import csv
import importlib.util
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import yaml

from .preprocess import prepare_gain_only_bscan


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


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _json_dump(data: Any, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _finite_or_none(value: Any) -> float | None:
    if value in (None, ""):
        return None
    v = float(value)
    return v if math.isfinite(v) else None


def _physical_calibration(prep: dict[str, Any], cfg: dict[str, Any]) -> dict[str, float | str | bool]:
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
        "clamp_before_time_zero": bool(cal.get("clamp_before_time_zero", True)),
    }


def _sample_to_original_relative(sample: float, n_samples: int) -> float:
    return 100.0 * sample / float(max(n_samples - 1, 1))


def _sample_to_timezero_relative(sample: float, tz_sample: float, n_samples: int) -> float:
    """0..100 coordinate over the physically relevant part of the B-scan.

    time-zero -> 0
    last raw sample -> 100
    """
    remaining = float(max((n_samples - 1) - tz_sample, 1.0))
    return 100.0 * (sample - tz_sample) / remaining


def _augment_line_physics(
    line: dict[str, Any],
    index: int,
    prep: dict[str, Any],
    phys: dict[str, Any],
) -> dict[str, Any]:
    n_samples = int(prep["raw_n_samples"])
    tz_sample = float(prep["time_zero_sample"])

    # Berta's coordinate is 0..100 over the COMPLETE, untrimmed B-scan.
    original_rel = float(line["y_relativa"])
    original_sample = original_rel / 100.0 * float(max(n_samples - 1, 1))
    original_delta = original_sample - tz_sample

    # Time-zero has priority. If Berta places a line before time-zero, the robot
    # coordinate is snapped to time-zero instead of reporting negative depth.
    clamp = bool(phys.get("clamp_before_time_zero", True))
    if clamp and original_delta < 0.0:
        corrected_sample = tz_sample
        snapped = True
    else:
        corrected_sample = original_sample
        snapped = False

    corrected_delta = corrected_sample - tz_sample
    corrected_rel_t0 = _sample_to_timezero_relative(corrected_sample, tz_sample, n_samples)
    corrected_rel_t0 = float(max(0.0, min(100.0, corrected_rel_t0)))

    twt_ns = corrected_delta * float(phys["dt_ns_per_sample"])
    depth_cm = corrected_delta * float(phys["depth_cm_per_sample"])

    # Detected band thickness is kept from Berta's original image extent. It is
    # converted to a depth-equivalent span but is NOT a validated physical thickness.
    thickness_samples = (
        float(line["grosor_relativo"]) / 100.0 * float(max(n_samples - 1, 1))
    )
    thickness_cm = thickness_samples * float(phys["depth_cm_per_sample"])

    # Keep region bounds both in original coordinates and corrected relative-to-T0
    # coordinates. Bounds before T0 are also clipped to T0 for robot-facing values.
    image_h = max(int(prep["image_height_px"]) - 1, 1)
    top_sample_original = float(line["region_y0"]) / float(image_h) * float(max(n_samples - 1, 1))
    bottom_sample_original = float(line["region_y1"]) / float(image_h) * float(max(n_samples - 1, 1))
    top_sample_corrected = max(top_sample_original, tz_sample) if clamp else top_sample_original
    bottom_sample_corrected = max(bottom_sample_original, tz_sample) if clamp else bottom_sample_original

    return {
        "id": f"L{index:03d}",

        # Robot-facing coordinate system: time-zero is exactly 0.
        "position_relative": round(corrected_rel_t0, 6),
        "position_sample": round(corrected_sample, 3),
        "offset_from_time_zero_samples": round(corrected_delta, 3),
        "twt_from_time_zero_ns": round(twt_ns, 6),
        "depth_cm_approx": round(max(0.0, depth_cm), 4),

        # Audit trail: original Berta result before T0 correction.
        "position_relative_original": round(original_rel, 6),
        "position_sample_original": round(original_sample, 3),
        "offset_from_time_zero_samples_original": round(original_delta, 3),
        "snapped_to_time_zero": bool(snapped),

        "thickness_relative_original": round(float(line["grosor_relativo"]), 6),
        "thickness_cm_approx": round(thickness_cm, 4),
        "distance_to_next_relative": None,  # filled after all positions are corrected
        "distance_to_next_cm_approx": None,  # filled after all positions are corrected
        "confidence": round(float(line["confianza"]), 6),
        "angle_deg": round(float(line["angulo"]), 6),
        "support": round(float(line["soporte"]), 6),
        "strength": round(float(line["fuerza"]), 6),
        "position_px_rendered_original": int(line["y"]),
        "thickness_px_rendered": int(line["grosor_px"]),
        "region_top_relative_from_time_zero": round(
            max(0.0, min(100.0, _sample_to_timezero_relative(top_sample_corrected, tz_sample, n_samples))), 6
        ),
        "region_bottom_relative_from_time_zero": round(
            max(0.0, min(100.0, _sample_to_timezero_relative(bottom_sample_corrected, tz_sample, n_samples))), 6
        ),
        "region_top_cm_approx": round(max(0.0, (top_sample_corrected - tz_sample) * float(phys["depth_cm_per_sample"])), 4),
        "region_bottom_cm_approx": round(max(0.0, (bottom_sample_corrected - tz_sample) * float(phys["depth_cm_per_sample"])), 4),
    }


def _fill_corrected_distances(
    result_lines: list[dict[str, Any]],
    prep: dict[str, Any],
    phys: dict[str, Any],
) -> None:
    """Distances are recomputed AFTER the T0 snap, never copied from Berta."""
    n_samples = int(prep["raw_n_samples"])
    tz_sample = float(prep["time_zero_sample"])
    remaining = float(max((n_samples - 1) - tz_sample, 1.0))

    for i, item in enumerate(result_lines):
        if i == len(result_lines) - 1:
            item["distance_to_next_relative"] = None
            item["distance_to_next_cm_approx"] = None
            continue
        nxt = result_lines[i + 1]
        dist_samples = max(0.0, float(nxt["position_sample"]) - float(item["position_sample"]))
        item["distance_to_next_relative"] = round(100.0 * dist_samples / remaining, 6)
        item["distance_to_next_cm_approx"] = round(
            dist_samples * float(phys["depth_cm_per_sample"]), 4
        )


def _draw_time_zero_overlay(
    annotated_bgr,
    prep: dict[str, Any],
    result_lines: list[dict[str, Any]],
):
    img = annotated_bgr.copy()
    h, w = img.shape[:2]
    tz_y = int(round(float(prep["time_zero_relative"]) / 100.0 * float(max(h - 1, 1))))
    tz_y = max(0, min(h - 1, tz_y))

    # Time-zero reference.
    cv2.line(img, (0, tz_y), (w - 1, tz_y), (255, 255, 0), 2)
    cv2.putText(
        img,
        f"TIME ZERO sample={prep['time_zero_sample']} -> robot y=0",
        (8, max(20, tz_y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 0),
        1,
        cv2.LINE_AA,
    )

    # Robot-facing corrected line centres. When a line was before T0 it is drawn
    # at T0 in this merged view, while Berta's original red/green annotation remains
    # visible underneath for auditability.
    for item in result_lines:
        corrected_raw_rel = _sample_to_original_relative(
            float(item["position_sample"]), int(prep["raw_n_samples"])
        )
        y = int(round(corrected_raw_rel / 100.0 * float(max(h - 1, 1))))
        y = max(0, min(h - 1, y))
        cv2.line(img, (0, y), (w - 1, y), (0, 255, 255), 1)
        suffix = " SNAP->T0" if item["snapped_to_time_zero"] else ""
        label = f"{item['id']} robot={item['position_relative']:.1f}% ~{item['depth_cm_approx']:.1f} cm{suffix}"
        cv2.putText(
            img,
            label,
            (8, max(18, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return img


def run_line_pipeline(
    input_sgy: str | Path,
    output_dir: str | Path | None = None,
    config_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run Task 1b from raw GP8800 SEGY to robot-facing line results.

    Detection uses a gain-only, untrimmed B-scan. Time-zero is estimated
    independently on raw data. AFTER detection, coordinates are re-based so that
    time-zero is y=0. Any detected line before time-zero is snapped to time-zero.
    """
    input_sgy = Path(input_sgy).expanduser().resolve()
    project_root = Path(__file__).resolve().parents[1]
    cfg_path = Path(config_path).expanduser().resolve() if config_path else project_root / "config.yaml"
    cfg = _load_yaml(cfg_path)

    if output_dir is None:
        output_root = project_root / cfg.get("outputs", {}).get("output_dir", "outputs")
        output_dir = output_root / input_sgy.stem
    else:
        output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    prep = prepare_gain_only_bscan(input_sgy, output_dir, cfg)
    prep_json = {k: (str(v) if isinstance(v, Path) else v) for k, v in prep.items()}
    _json_dump(prep_json, output_dir / "preprocessing_manifest.json")

    berta = _load_berta_module(project_root)
    image_path = Path(prep["gain_only_image"])
    lines, gray = berta.detectar_lineas_horizontales(image_path)
    height, width = gray.shape
    lines = berta.añadir_metricas_relativas(lines, height)

    detection_dir = output_dir / "02_line_detection"
    detection_dir.mkdir(parents=True, exist_ok=True)
    annotated = berta.dibujar_resultado(gray, lines)
    annotated_path = detection_dir / f"{image_path.stem}_detectado.png"
    if bool(cfg.get("outputs", {}).get("save_annotated_image", True)):
        berta.guardar_png(annotated_path, annotated)

    phys = _physical_calibration(prep, cfg)
    result_lines = [
        _augment_line_physics(line, i, prep, phys)
        for i, line in enumerate(lines, start=1)
    ]
    _fill_corrected_distances(result_lines, prep, phys)

    if bool(cfg.get("outputs", {}).get("save_time_zero_overlay", True)):
        merged_dir = output_dir / "03_time_zero_merge"
        merged_dir.mkdir(parents=True, exist_ok=True)
        merged = _draw_time_zero_overlay(annotated, prep, result_lines)
        berta.guardar_png(merged_dir / f"{image_path.stem}_lines_plus_timezero.png", merged)

    # Robot-facing compact fields are now ALL expressed relative to time-zero.
    # Original Berta positions are retained separately for audit/debugging.
    pos_corrected = ";".join(f"{item['position_relative']:.6f}" for item in result_lines)
    pos_original = ";".join(f"{item['position_relative_original']:.6f}" for item in result_lines)
    thickness_original = ";".join(f"{line['grosor_relativo']:.6f}" for line in lines)
    distance_corrected = ";".join(
        f"{item['distance_to_next_relative']:.6f}"
        for item in result_lines if item["distance_to_next_relative"] is not None
    )
    depth_cm = ";".join(f"{item['depth_cm_approx']:.6f}" for item in result_lines)
    thickness_cm = ";".join(f"{item['thickness_cm_approx']:.6f}" for item in result_lines)
    distance_cm = ";".join(
        f"{item['distance_to_next_cm_approx']:.6f}"
        for item in result_lines if item["distance_to_next_cm_approx"] is not None
    )
    snapped = ";".join("1" if item["snapped_to_time_zero"] else "0" for item in result_lines)

    final_row = {
        "imagen": image_path.name,
        "posicion_lineas_relativa": pos_corrected,
        "grosor_lineas_relativo_original": thickness_original,
        "distancia_entre_lineas_relativa": distance_corrected,
        "profundidad_lineas_cm_aprox": depth_cm,
        "grosor_lineas_cm_aprox": thickness_cm,
        "distancia_entre_lineas_cm_aprox": distance_cm,
        "linea_ajustada_a_timezero": snapped,
        "posicion_lineas_relativa_original_berta": pos_original,
        "time_zero_sample": prep["time_zero_sample"],
        "time_zero_relativo_original": round(float(prep["time_zero_relative"]), 6),
        "epsilon_r_supuesta": phys["epsilon_r_assumed"],
    }
    _write_csv(output_dir / "results_finales.csv", list(final_row.keys()), [final_row])

    detailed_rows: list[dict[str, Any]] = []
    for i, (line, item) in enumerate(zip(lines, result_lines), start=1):
        detailed_rows.append({
            "imagen": image_path.name,
            "numero_linea": i,
            "y_original_berta": line["y"],
            "y_relativa_original_berta": item["position_relative_original"],
            "position_sample_original": item["position_sample_original"],
            "offset_from_time_zero_samples_original": item["offset_from_time_zero_samples_original"],
            "snapped_to_time_zero": item["snapped_to_time_zero"],
            "position_sample_corrected": item["position_sample"],
            "position_relative_from_time_zero": item["position_relative"],
            "offset_from_time_zero_samples": item["offset_from_time_zero_samples"],
            "twt_from_time_zero_ns": item["twt_from_time_zero_ns"],
            "depth_cm_approx": item["depth_cm_approx"],
            "grosor_px": line["grosor_px"],
            "grosor_relativo_original": line["grosor_relativo"],
            "thickness_cm_approx": item["thickness_cm_approx"],
            "distance_to_next_relative_from_time_zero": item["distance_to_next_relative"],
            "distance_to_next_cm_approx": item["distance_to_next_cm_approx"],
            "confianza": line["confianza"],
            "angulo": line["angulo"],
            "soporte": line["soporte"],
            "fuerza": line["fuerza"],
        })

    detailed_fields = list(detailed_rows[0].keys()) if detailed_rows else [
        "imagen", "numero_linea", "y_original_berta", "y_relativa_original_berta",
        "position_sample_original", "offset_from_time_zero_samples_original",
        "snapped_to_time_zero", "position_sample_corrected",
        "position_relative_from_time_zero", "offset_from_time_zero_samples",
        "twt_from_time_zero_ns", "depth_cm_approx", "grosor_px",
        "grosor_relativo_original", "thickness_cm_approx",
        "distance_to_next_relative_from_time_zero", "distance_to_next_cm_approx",
        "confianza", "angulo", "soporte", "fuerza",
    ]
    _write_csv(output_dir / "resultados_lineas.csv", detailed_fields, detailed_rows)

    principal = berta.obtener_linea_principal(lines)
    summary_row = {
        "imagen": image_path.name,
        "ancho": width,
        "alto": height,
        "gain_db": prep["gain_db"],
        "num_lineas": len(lines),
        "estado": "DETECTADA" if lines else "NO_DETECTADA",
        "time_zero_sample": prep["time_zero_sample"],
        "time_zero_relativo_original": prep["time_zero_relative"],
        "time_window_ns_corregida": prep["corrected_time_window_ns"],
        "epsilon_r_supuesta": phys["epsilon_r_assumed"],
        "n_lineas_ajustadas_a_timezero": sum(1 for item in result_lines if item["snapped_to_time_zero"]),
        "confianza_principal": "" if principal is None else principal["confianza"],
    }
    _write_csv(output_dir / "resultados_resumen.csv", list(summary_row.keys()), [summary_row])

    result = {
        "detected": bool(lines),
        "n_lines": len(lines),
        "lines": result_lines,
        "source": {
            "sgy": input_sgy.name,
            "sidecar_csv": prep["sidecar_csv"],
            "gain_only_bscan": prep["gain_only_image_relative"],
            "gain_db": prep["gain_db"],
            "raw_n_samples": prep["raw_n_samples"],
            "raw_n_traces": prep["raw_n_traces"],
            "corrected_time_window_ns": prep["corrected_time_window_ns"],
            "scan_distance_m": prep["scan_distance_m"],
        },
        "time_zero_reference": {
            "sample": prep["time_zero_sample"],
            "relative_original_0_100": round(float(prep["time_zero_relative"]), 6),
            "robot_relative_coordinate": 0.0,
            "applied_before_detection": False,
            "has_priority_over_line_detector": True,
            "negative_coordinate_policy": "snap detected line to time-zero",
            "strategy": "detect first on full gain-only B-scan; estimate time-zero independently; rebase all robot coordinates so T0=0; clamp any pre-T0 detection to T0",
        },
        "depth_calibration": {
            "epsilon_r_assumed": phys["epsilon_r_assumed"],
            "epsilon_r_basis": phys["epsilon_r_basis"],
            "velocity_m_ns": round(float(phys["velocity_m_ns"]), 8),
            "dt_ns_per_sample": round(float(phys["dt_ns_per_sample"]), 8),
            "depth_cm_per_sample": round(float(phys["depth_cm_per_sample"]), 8),
            "formula": "depth = (c/sqrt(epsilon_r)) * two_way_time / 2",
            "warning": "Approximate depth under assumed epsilon_r=6. Line-band thickness is a radar-image/time extent, not a validated physical object thickness.",
        },
        "coordinate_system": {
            "robot_position_relative": "0..100 from time-zero to last raw sample",
            "time_zero": 0.0,
            "pre_time_zero_detections": "clamped to 0",
            "original_berta_coordinate_retained_for_audit": True,
        },
        "preprocessing": {
            "detector_image": "gain only",
            "background_removal": False,
            "time_zero_crop": False,
        },
    }
    _json_dump(result, output_dir / "line_result.json")

    manifest = {
        "pipeline": "DISCOVER_GPR_Line_Segmentation_v4",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "input": str(input_sgy),
        "output_dir": str(output_dir),
        "shared_gprtools": str(project_root.parent / "GPRTools"),
        "original_algorithm": "berta_original/line_segmentation.py",
        "berta_algorithm_modified": False,
        "integration_changes": [
            "raw SEGY input through shared GPRTools",
            "gain-only detector image",
            "independent time-zero reference without pre-cropping",
            "post-detection coordinate re-basing with time-zero = 0",
            "pre-time-zero line detections snapped to time-zero",
            "approximate-cm conversion with epsilon_r=6",
        ],
    }
    _json_dump(manifest, output_dir / "run_manifest.json")
    return result
