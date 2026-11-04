from __future__ import annotations

from typing import Any

import numpy as np
import pandas as pd


def _clean(v: Any) -> Any:
    if isinstance(v, np.generic):
        v = v.item()
    if isinstance(v, float) and (np.isnan(v) or np.isinf(v)):
        return None
    return v


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
