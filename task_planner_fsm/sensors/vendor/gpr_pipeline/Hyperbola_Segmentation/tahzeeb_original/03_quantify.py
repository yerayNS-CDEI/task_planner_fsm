"""Step 3 - Per-element quantification for recovery planning.

For each element it derives: object count, bar pitch (mean +/- sd), cover-depth
statistics, reinforcement-layer count (1 or 2 via a depth-gap split), and the
recoverable steel mass, indicative value and avoided CO2.

Input : objects.csv  (from step 2)
Output: element_summary.csv

Usage:
    python 03_quantify.py --config config.yaml --in results/<ts>/objects.csv \
                          --out results/<ts>/element_summary.csv
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

from gpr_pp_utils import load_config


def split_layers(depths_cm: np.ndarray, gap_cm: float,
                 min_count: int = 1) -> tuple[int, list[float]]:
    """Split a 1-D set of depths into 1 or 2 layers.

    Tries candidate split gaps in descending order (largest first) and picks
    the first one that is >= gap_cm AND leaves at least min_count objects in
    BOTH clusters.  This prevents a lone outlier at the extreme from capturing
    the largest gap while the physically meaningful second cluster (e.g. rear
    rebar) is buried in the next-largest gap.

    Returns ``(n_layers, layer_means_cm)``.
    """
    if len(depths_cm) < 4:
        return 1, [float(np.median(depths_cm))] if len(depths_cm) else [0.0]
    s = np.sort(depths_cm)
    gaps = np.diff(s)
    for gi in np.argsort(gaps)[::-1]:   # descending by gap size
        if gaps[gi] < gap_cm:
            break                       # all remaining gaps are smaller
        lo, hi = s[:gi + 1], s[gi + 1:]
        if len(lo) >= min_count and len(hi) >= min_count:
            return 2, [float(np.mean(lo)), float(np.mean(hi))]
    return 1, [float(np.median(s))]


def element_type_from_name(name: str) -> str:
    n = name.lower()
    if "floor" in n or "slab" in n:
        return "floor_slab"
    if "partition" in n:
        return "partition"
    if "column" in n:
        return "column"
    if "beam" in n:
        return "beam"
    if "roof" in n:
        return "roof_slab"
    if "found" in n:
        return "foundation"
    return "wall"


def material_interpretation(etype: str, n_layers: int, n_obj: int) -> str:
    if n_obj == 0:
        return "no clear reflectors"
    if etype in ("floor_slab", "ground_slab", "roof_slab"):
        return "RCC slab - steel mesh (rebar)"
    if etype in ("column", "beam"):
        return "RCC structural support - rebar"
    if etype in ("partition", "partition_pladur"):
        return "Pladur/cardboard partition - equidistant hyperbolae are vertical studs/supports (non-structural)"
    if etype == "brick_wall":
        return "brick wall - hyperbolae from courses/ties/services (verify; no structural rebar)"
    if etype == "retaining_wall":
        return "RC retaining wall (cast against earth) - rebar" + (" (two layers)" if n_layers == 2 else "")
    return "reinforced concrete - rebar" + (" (two layers)" if n_layers == 2 else "")


def quantify(cfg: dict, obj: pd.DataFrame) -> pd.DataFrame:
    q = cfg["quantify"]
    econ = cfg["economics"]
    d_mm = float(q["bar_diameter_mm"])
    rho = float(q["steel_density"])
    area_bar = np.pi * (d_mm / 1000.0 / 2.0) ** 2  # m^2 cross-section
    heights = q["element_height_m"]

    _FORCE_ONE_LAYER = {"partition_pladur", "partition", "brick_wall"}
    _STRUCTURAL_RC   = {"floor_slab", "ground_slab", "wall", "retaining_wall"}
    min_lc = int(q.get("min_layer_count", 5))

    rows = []
    for el, g in obj.groupby("element"):
        xs = np.sort(g["x_m"].to_numpy())
        depths = g["depth_cm"].to_numpy()
        n = len(g)
        spans = np.diff(xs)
        spans = spans[(spans > 0.05) & (spans < 0.6)]  # plausible bar pitch only
        pitch_m = float(np.mean(spans)) if len(spans) else float("nan")
        pitch_sd = float(np.std(spans)) if len(spans) else float("nan")
        overrides = (q.get("element_overrides") or {})
        etype = overrides.get(el, element_type_from_name(el))
        if etype in _FORCE_ONE_LAYER:
            n_layers, layer_means = 1, ([float(np.median(depths))] if n else [0.0])
        else:
            # Structural RC requires min_layer_count objects in each cluster;
            # non-structural non-forced elements get only 1 layer.
            mc = min_lc if etype in _STRUCTURAL_RC else (n + 1)
            n_layers, layer_means = split_layers(depths, float(q["layer_gap_cm"]),
                                                 min_count=mc)
        height_m = float(heights.get(etype, heights["default"]))
        length_m = float(xs.max() - xs.min()) if n > 1 else 0.0
        area_m2 = max(length_m, 0.0) * height_m

        # recovery logic depends on element type
        steel_kg = 0.0; value_eur = 0.0; co2_kg = 0.0
        steel_types = ("floor_slab", "ground_slab", "wall", "retaining_wall",
                       "column", "beam", "roof_slab")
        if etype in steel_types and not np.isnan(pitch_m) and pitch_m > 0 and area_m2 > 0:
            bars_per_m = 1.0 / pitch_m
            length_total_m = n_layers * 2.0 * bars_per_m * area_m2
            steel_kg = length_total_m * area_bar * rho
            value_eur = steel_kg * float(econ["steel_eur_per_kg"])
            co2_kg = steel_kg * float(econ["co2_kg_per_kg_steel"])
        elif etype == "brick_wall":
            value_eur = area_m2 * float(econ["brick_eur_per_m2"])  # salvage bricks, no rebar
        # partition_pladur / partition: vertical studs/services -> no structural steel

        rows.append({
            "element": el,
            "type": etype,
            "n_objects": n,
            "length_m": round(length_m, 2),
            "area_m2": round(area_m2, 1),
            "bar_pitch_cm": round(pitch_m * 100, 1) if not np.isnan(pitch_m) else None,
            "bar_pitch_sd_cm": round(pitch_sd * 100, 1) if not np.isnan(pitch_sd) else None,
            "n_layers": n_layers,
            "layer_depths_cm": ";".join(f"{v:.1f}" for v in layer_means),
            "cover_min_cm": round(float(np.min(depths)), 1) if n else None,
            "interpretation": material_interpretation(etype, n_layers, n),
            "recoverable_steel_kg": round(steel_kg, 1),
            "value_eur": round(value_eur, 0),
            "co2_avoided_kg": round(co2_kg, 1),
        })
    return pd.DataFrame(rows).sort_values("element").reset_index(drop=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cfg = load_config(args.config)
    obj = pd.read_csv(args.inp)
    summ = quantify(cfg, obj)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    summ.to_csv(args.out, index=False)
    tot_steel = summ["recoverable_steel_kg"].sum()
    tot_val = summ["value_eur"].sum()
    print(f"[step3] elements quantified : {len(summ)}")
    for _, r in summ.iterrows():
        print(f"[step3]   {r['element']:42s} n={r['n_objects']:3d} "
              f"pitch={r['bar_pitch_cm']} cm layers={r['n_layers']} "
              f"steel={r['recoverable_steel_kg']} kg")
    print(f"[step3] TOTAL recoverable steel: {tot_steel:.0f} kg  value: EUR {tot_val:.0f}")
    print(f"[step3] written -> {args.out}")


if __name__ == "__main__":
    main()
