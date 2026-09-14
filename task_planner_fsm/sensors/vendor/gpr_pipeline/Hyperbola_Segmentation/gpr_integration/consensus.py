from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pandas as pd


@dataclass
class Cluster:
    members: list[dict[str, Any]] = field(default_factory=list)

    @property
    def gains(self) -> set[float]:
        return {float(m["gain_db"]) for m in self.members}

    @property
    def x_m(self) -> float:
        return float(np.median([float(m["x_m"]) for m in self.members]))

    @property
    def depth_cm(self) -> float:
        return float(np.median([float(m["depth_cm"]) for m in self.members]))


def _compatible(row: dict[str, Any], cluster: Cluster, x_tol_m: float, d_tol_cm: float) -> tuple[bool, float]:
    dx = abs(float(row["x_m"]) - cluster.x_m) / max(x_tol_m, 1e-9)
    dd = abs(float(row["depth_cm"]) - cluster.depth_cm) / max(d_tol_cm, 1e-9)
    score = math.sqrt(dx * dx + dd * dd)
    return score <= 1.0, score


def _median(members: list[dict[str, Any]], key: str) -> float | None:
    vals = [float(m[key]) for m in members if key in m and pd.notna(m[key])]
    return float(np.median(vals)) if vals else None


def build_consensus(
    per_gain_objects: pd.DataFrame,
    all_gains: list[float],
    scan_distance_m: float,
    cfg: dict[str, Any],
) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, Any]]:
    ccfg = cfg.get("consensus", {})
    x_tol = float(ccfg.get("x_tolerance_m", 0.06))
    d_tol = float(ccfg.get("depth_tolerance_cm", 4.0))
    min_support = int(ccfg.get("min_gain_support", 3))

    if per_gain_objects.empty:
        cols = ["detection_id", "accepted", "support_count", "support_ratio", "gains_db",
                "x_m", "position_cm", "x_relative", "depth_cm_assumed", "twt_ns",
                "apex_px", "v_m_ns", "eps_r_assumed", "arc_hw_m", "confidence_mean",
                "confidence_max", "x_std_m", "depth_std_cm"]
        empty = pd.DataFrame(columns=cols)
        return empty.copy(), empty, {
            "n_gains": len(all_gains), "min_gain_support": min_support,
            "candidate_clusters": 0, "accepted_clusters": 0,
            "x_tolerance_m": x_tol, "depth_tolerance_cm": d_tol,
        }

    rows = per_gain_objects.to_dict(orient="records")
    rows.sort(key=lambda r: float(r.get("confidence", 0.0)), reverse=True)
    clusters: list[Cluster] = []
    for row in rows:
        candidates: list[tuple[float, Cluster]] = []
        for cluster in clusters:
            ok, score = _compatible(row, cluster, x_tol, d_tol)
            if ok and float(row["gain_db"]) not in cluster.gains:
                candidates.append((score, cluster))
        if candidates:
            candidates.sort(key=lambda t: t[0])
            candidates[0][1].members.append(row)
        else:
            clusters.append(Cluster([row]))

    all_rows: list[dict[str, Any]] = []
    n_gains = max(len(all_gains), 1)
    for idx, cluster in enumerate(sorted(clusters, key=lambda c: c.x_m), start=1):
        m = cluster.members
        support = len(cluster.gains)
        confs = [float(r.get("confidence", 0.0)) for r in m]
        xs = [float(r["x_m"]) for r in m]
        ds = [float(r["depth_cm"]) for r in m]
        gains = sorted(cluster.gains)
        x_m = float(np.median(xs))
        depth = float(np.median(ds))
        row = {
            "detection_id": f"H{idx:03d}",
            "accepted": bool(support >= min_support),
            "support_count": int(support),
            "support_ratio": round(float(support / n_gains), 4),
            "gains_db": ",".join(f"{g:g}" for g in gains),
            "gain_min_db": float(min(gains)),
            "gain_max_db": float(max(gains)),
            "gain_span_db": float(max(gains) - min(gains)),
            "x_m": round(x_m, 4),
            "position_cm": round(x_m * 100.0, 1),
            "x_relative": round(x_m / float(scan_distance_m), 5),
            "depth_cm_assumed": round(depth, 2),
            "twt_ns": round(float(_median(m, "twt_ns") or 0.0), 4),
            "apex_px": round(float(_median(m, "apex_px") or 0.0), 2),
            "v_m_ns": round(float(_median(m, "v_m_ns") or 0.0), 5),
            "eps_r_assumed": round(float(_median(m, "eps_r") or 0.0), 3),
            "arc_hw_m": round(float(_median(m, "arc_hw_m") or 0.0), 4),
            "confidence_mean": round(float(np.mean(confs)), 4),
            "confidence_max": round(float(np.max(confs)), 4),
            "x_std_m": round(float(np.std(xs)), 4),
            "depth_std_cm": round(float(np.std(ds)), 3),
        }
        all_rows.append(row)

    audit = pd.DataFrame(all_rows)
    accepted = audit[audit["accepted"]].reset_index(drop=True) if not audit.empty else audit.copy()
    stats = {
        "n_gains": len(all_gains),
        "min_gain_support": min_support,
        "candidate_clusters": int(len(audit)),
        "accepted_clusters": int(len(accepted)),
        "x_tolerance_m": x_tol,
        "depth_tolerance_cm": d_tol,
        "interpretation": "gain support measures robustness to preprocessing gain; gains are not independent acquisitions",
    }
    return accepted, audit, stats
