"""Step 2b - Duplicate-hyperbola removal (apex-space).

Tile-overlap NMS (step 1) only merges boxes that overlap heavily *in pixels*.
It does NOT merge two detections that describe the **same physical hyperbola**
with differently-shaped boxes/masks, nor does it drop spurious deep multiples.
This step works in physical apex space: it clusters objects whose apex is close
in distance AND depth, keeps the highest-confidence member of each cluster, and
optionally drops reflectors deeper than a physical limit (ringing / multiples).

Input : objects.csv  (element, x_m, depth_cm, confidence, ...)
Output: objects_dedup.csv  (+ a *_removed.csv audit of what was dropped)

Usage:
    python 02b_dedup.py --config config.yaml --in results/<ts>/objects.csv \
        --out results/<ts>/objects_dedup.csv
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from gpr_pp_utils import load_config


def remove_multiples(g: pd.DataFrame, x_tol: float,
                     ratio_min: float = 1.4, ratio_max: float = 2.6,
                     min_primary_conf: float = 0.70) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Drop first-order multiple reflections.

    A detection B is flagged as a multiple of A when:
      - A is shallower than B
      - |x_A - x_B| < x_tol (same horizontal position)
      - depth_B / depth_A ∈ [ratio_min, ratio_max]   (centred on 2×)
      - confidence_A >= min_primary_conf (A is a reliable primary reflector)

    The 2× depth ratio arises from the surface-reflector-surface round-trip path
    that produces first-order ringing beneath a strong primary.
    """
    g = g.sort_values("depth_cm").reset_index(drop=True)
    depths = g["depth_cm"].to_numpy()
    xs = g["x_m"].to_numpy()
    confs = g["confidence"].to_numpy()
    keep = np.ones(len(g), dtype=bool)

    for j in range(len(g)):
        if not keep[j]:
            continue
        for i in range(len(g)):
            if i == j or not keep[i]:
                continue
            if depths[i] >= depths[j]:
                continue  # A must be shallower than B
            if abs(xs[i] - xs[j]) > x_tol:
                continue
            r = depths[j] / max(depths[i], 0.5)
            if ratio_min <= r <= ratio_max and confs[i] >= min_primary_conf:
                keep[j] = False  # B is a ringing multiple of high-confidence primary A
                break

    kept_idx = g.index[keep]
    rem_idx = g.index[~keep]
    return g.loc[kept_idx], g.loc[rem_idx]


def dedup_element(g: pd.DataFrame, x_tol: float, d_tol: float) -> pd.DataFrame:
    """Greedy apex clustering: keep highest-confidence object per cluster."""
    g = g.sort_values("confidence", ascending=False)
    keep = np.ones(len(g), dtype=bool)
    xs = g["x_m"].to_numpy()
    ds = g["depth_cm"].to_numpy()
    for i in range(len(g)):
        if not keep[i]:
            continue
        for j in range(i + 1, len(g)):
            if not keep[j]:
                continue
            if abs(xs[i] - xs[j]) <= x_tol and abs(ds[i] - ds[j]) <= d_tol:
                keep[j] = False  # j is a lower-confidence duplicate of i
    return g[keep]  # original index preserved


def run(cfg: dict, obj: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    d = cfg.get("dedup", {})
    x_tol = float(d.get("x_tol_m", 0.06))
    d_tol = float(d.get("depth_tol_cm", 4.0))
    max_depth = float(d.get("max_depth_cm", 40.0))
    per_el_max = d.get("max_depth_cm_by_element") or {}

    do_mult = bool(d.get("multiple_removal", True))
    mult_x_tol = float(d.get("multiple_x_tol_m", 0.15))
    mult_ratio_min = float(d.get("multiple_ratio_min", 1.4))
    mult_ratio_max = float(d.get("multiple_ratio_max", 2.6))
    mult_min_conf  = float(d.get("multiple_min_primary_conf", 0.70))

    el_overrides = (cfg.get("quantify", {}).get("element_overrides") or {})
    _SKIP_MULTIPLE_MATS = {"partition_pladur", "partition"}

    kept, removed = [], []
    for el, g in obj.groupby("element"):
        # 1) drop physically implausible deep reflectors (ringing / multiples)
        el_max = float(per_el_max.get(el, max_depth))
        deep = g[g["depth_cm"] > el_max]
        g2 = g[g["depth_cm"] <= el_max]
        if len(deep):
            deep = deep.copy(); deep["reason"] = f"depth>{el_max}cm (multiple/clutter)"
            removed.append(deep)
        # 2) first-order multiple reflection removal (2× depth at same x)
        # Skipped for Pladur partitions: shallow genuine studs at 4 cm are falsely
        # flagged as multiples of 2 cm primaries because their depth ratio triggers
        # the 1.4–2.6 window.  Pladur does not produce ringing multiples.
        mat = el_overrides.get(el, "default")
        if do_mult and len(g2) and mat not in _SKIP_MULTIPLE_MATS:
            g2, mult_rem = remove_multiples(g2, mult_x_tol,
                                            ratio_min=mult_ratio_min,
                                            ratio_max=mult_ratio_max,
                                            min_primary_conf=mult_min_conf)
            if len(mult_rem):
                mult_rem = mult_rem.copy()
                mult_rem["reason"] = "ringing multiple (~2x depth of primary)"
                removed.append(mult_rem)
        # 3) apex-space duplicate merge
        kg = dedup_element(g2, x_tol, d_tol)
        dropped = g2.loc[~g2.index.isin(kg.index)].copy()
        if len(dropped):
            dropped["reason"] = "duplicate hyperbola (apex within tol)"
            removed.append(dropped)
        kept.append(kg.sort_values("x_m"))
    kept_df = pd.concat(kept, ignore_index=True) if kept else obj
    removed_df = pd.concat(removed, ignore_index=True) if removed else obj.iloc[0:0].assign(reason=[])
    return kept_df, removed_df


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cfg = load_config(args.config)
    obj = pd.read_csv(args.inp)
    kept, removed = run(cfg, obj)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    kept.to_csv(args.out, index=False)
    rem_path = args.out.replace(".csv", "_removed.csv")
    removed.to_csv(rem_path, index=False)
    print(f"[step2b] dedup: {len(obj)} -> {len(kept)} objects ({len(removed)} removed)")
    for el, g in kept.groupby("element"):
        n0 = int((obj['element'] == el).sum())
        print(f"[step2b]   {el:42s} {n0:3d} -> {len(g):3d}")
    print(f"[step2b] kept    -> {args.out}")
    print(f"[step2b] removed -> {rem_path}")


if __name__ == "__main__":
    main()
