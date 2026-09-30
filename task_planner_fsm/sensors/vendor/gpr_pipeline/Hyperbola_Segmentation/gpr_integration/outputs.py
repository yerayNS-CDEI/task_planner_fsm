from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

from .utils import json_dump


def _clean(v: Any) -> Any:
    if isinstance(v, np.generic):
        v = v.item()
    if isinstance(v, float) and (np.isnan(v) or np.isinf(v)):
        return None
    return v


def dataframe_records(df: pd.DataFrame) -> list[dict[str, Any]]:
    return [{k: _clean(v) for k, v in row.items()} for row in df.to_dict(orient="records")]


def build_software_result(
    scan_id: str,
    accepted: pd.DataFrame,
    consensus_stats: dict[str, Any],
    manifest: dict[str, Any],
    cfg: dict[str, Any],
) -> dict[str, Any]:
    detections = []
    basis = cfg["postprocessing"].get("epsilon_r_basis")
    for r in accepted.to_dict(orient="records"):
        detections.append({
            "id": r["detection_id"],
            "type": "hyperbola",
            "position": {
                "x_m": _clean(r["x_m"]),
                "x_cm": _clean(r["position_cm"]),
                "x_relative": _clean(r["x_relative"]),
            },
            "depth": {
                "depth_cm": _clean(r["depth_cm_assumed"]),
                "epsilon_r_assumed": _clean(r["eps_r_assumed"]),
                "basis": basis,
                "is_measured_permittivity": False,
            },
            "geometry": {
                "apex_px": _clean(r["apex_px"]),
                "twt_ns": _clean(r["twt_ns"]),
                "velocity_m_ns": _clean(r["v_m_ns"]),
                "arc_half_width_m": _clean(r["arc_hw_m"]),
            },
            "robustness": {
                "gain_support": int(r["support_count"]),
                "gain_support_ratio": _clean(r["support_ratio"]),
                "supporting_gains_db": [float(v) for v in str(r["gains_db"]).split(",") if v != ""],
                "confidence_mean": _clean(r["confidence_mean"]),
                "confidence_max": _clean(r["confidence_max"]),
                "x_std_m": _clean(r["x_std_m"]),
                "depth_std_cm": _clean(r["depth_std_cm"]),
            },
        })
    return {
        "status": "ok",
        "scan_id": scan_id,
        "hyperbola_detected": bool(len(detections)),
        "n_valid_detections": int(len(detections)),
        "calibration": {
            "scan_distance_m": manifest["scan_distance_m"],
            "corrected_raw_time_window_ns": manifest["corrected_time_window_ns"],
            "time_window_note": manifest.get("time_window_note"),
            "epsilon_r_assumed": float(cfg["postprocessing"]["epsilon_r_assumed"]),
            "epsilon_r_basis": basis,
        },
        "consensus": consensus_stats,
        "detections": detections,
        "files": {},
    }


def write_output_files(
    run_dir: Path,
    result: dict[str, Any],
    accepted: pd.DataFrame,
    audit: pd.DataFrame,
    gain_summary: pd.DataFrame,
    removed_all: pd.DataFrame,
    cfg: dict[str, Any],
) -> dict[str, str]:
    paths: dict[str, str] = {}
    ocfg = cfg.get("outputs", {})
    if bool(ocfg.get("save_json", True)):
        p = run_dir / "gpr_result.json"
        json_dump(result, p)
        paths["json"] = p.name
    if bool(ocfg.get("save_csv", True)):
        p = run_dir / "gpr_detections.csv"
        accepted.to_csv(p, index=False)
        audit.to_csv(run_dir / "gpr_consensus_audit.csv", index=False)
        gain_summary.to_csv(run_dir / "gpr_gain_summary.csv", index=False)
        if not removed_all.empty:
            removed_all.to_csv(run_dir / "gpr_tahzeeb_removed.csv", index=False)
        paths["csv"] = p.name
    if bool(ocfg.get("save_excel", True)):
        p = run_dir / "gpr_results.xlsx"
        summary = pd.DataFrame([{
            "scan_id": result["scan_id"],
            "hyperbola_detected": result["hyperbola_detected"],
            "n_valid_detections": result["n_valid_detections"],
            "n_gains": result["consensus"]["n_gains"],
            "min_gain_support": result["consensus"]["min_gain_support"],
            "epsilon_r_assumed": result["calibration"]["epsilon_r_assumed"],
            "corrected_raw_time_window_ns": result["calibration"]["corrected_raw_time_window_ns"],
        }])
        with pd.ExcelWriter(p, engine="openpyxl") as writer:
            summary.to_excel(writer, sheet_name="summary", index=False)
            accepted.to_excel(writer, sheet_name="detections", index=False)
            audit.to_excel(writer, sheet_name="consensus_audit", index=False)
            gain_summary.to_excel(writer, sheet_name="gain_summary", index=False)
            if not removed_all.empty:
                removed_all.to_excel(writer, sheet_name="tahzeeb_removed", index=False)
        paths["excel"] = p.name
    return paths


def _pick_reference_image(manifest: dict[str, Any], desired_gain: float, run_dir: Path) -> Path:
    entry = min(manifest["images"], key=lambda e: abs(float(e["gain_db"]) - desired_gain))
    return run_dir / entry["image_path"]


def draw_final_visualization(
    run_dir: Path,
    manifest: dict[str, Any],
    accepted: pd.DataFrame,
    cfg: dict[str, Any],
    output_path: Path,
) -> None:
    desired = float(cfg.get("outputs", {}).get("visualization_gain_db", 25))
    base = _pick_reference_image(manifest, desired, run_dir)
    img = Image.open(base).convert("RGB")
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()
    width, height = img.size
    scan_distance = float(manifest["scan_distance_m"])
    # All gains use the same time-zero. Use the selected B-scan's calibrated time window.
    ref = min(manifest["images"], key=lambda e: abs(float(e["gain_db"]) - desired))
    tw = float(ref["processed_time_window_ns"])

    if accepted.empty:
        draw.rectangle([8, 8, 175, 26], fill="white")
        draw.text((12, 11), "No valid hyperbolas", fill="black", font=font)
        img.save(output_path)
        return

    for r in accepted.itertuples(index=False):
        x0_m = float(r.x_m)
        t0 = float(r.twt_ns)
        v = max(float(r.v_m_ns), 1e-6)
        hw = max(float(r.arc_hw_m), 0.02)
        xs = np.linspace(max(0.0, x0_m - hw), min(scan_distance, x0_m + hw), 80)
        ts = np.sqrt(np.maximum(t0, 1e-6) ** 2 + (2.0 * (xs - x0_m) / v) ** 2)
        pts = []
        for xm, tn in zip(xs, ts):
            px = int(round(xm / scan_distance * (width - 1)))
            py = int(round(tn / tw * (height - 1)))
            if 0 <= px < width and 0 <= py < height:
                pts.append((px, py))
        if len(pts) >= 2:
            draw.line(pts, fill="red", width=3)
        ax = int(round(x0_m / scan_distance * (width - 1)))
        ay = int(round(t0 / tw * (height - 1)))
        draw.ellipse([ax - 5, ay - 5, ax + 5, ay + 5], fill="yellow", outline="black")
        label = f"{r.detection_id} x={x0_m:.2f}m d~{float(r.depth_cm_assumed):.1f}cm {int(r.support_count)}/{len(manifest['gains_db'])} gains"
        ty = max(0, ay - 17)
        box_w = min(width - ax, max(120, 6 * len(label)))
        draw.rectangle([ax, ty, ax + box_w, ty + 14], fill="white")
        draw.text((ax + 2, ty + 2), label, fill="black", font=font)
    img.save(output_path)
