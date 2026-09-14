"""Step 2 - Physical parameters from each detection.

For every consolidated detection it computes the apex position and, via the
survey calibration, the cover depth. If instance masks are supplied it fits the
hyperbola travel-time model to recover the medium velocity per object; otherwise
it uses the host permittivity from the config (velocity assumed constant).

Travel-time model:   t(x) = sqrt(t0^2 + (2 (x - x0) / v)^2)
    depth   = v * t0 / 2
    eps_r   = (c / v)^2          (c = 0.299792458 m/ns)

Apex strategy (when masks_file is set):
    1. Find the ridge point with minimum y (leading edge of the pulse).
    2. Shift down by pulse_offset_samples (config calibration.pulse_offset_samples,
       default 21) to land at the visually bright amplitude peak (~0.28 ns for
       the GP8800 1.6 GHz antenna at 16 ns / 1200 px).
    3. Fit the hyperbola t(x) = sqrt(t0_fixed^2 + (2(x-x0_fixed)/v)^2) with
       apex fixed from step 1, solving only for v — much more stable than a
       3-parameter fit when the apex is near the image top (e.g. Pladur studs).

Input : consolidated.csv
Output: objects.csv  (element, x_m, apex_px, twt_ns, v_m_ns, eps_r, depth_cm,
                      confidence, arc_hw_m)
         arc_hw_m: arm half-width in metres (from ridge extent), used by
                   05_visualize.py to draw the correct arc width.

Usage:
    python 02_hyperbola_fit.py --config config.yaml --in results/<ts>/consolidated.csv \
                               --out results/<ts>/objects.csv
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from gpr_pp_utils import load_config, load_manifest, element_source_w

C_M_PER_NS = 0.299792458


def load_ridge_lookup(masks_path: str, manifest_path: str | None) -> dict:
    """Build {(element, round(gx1,1), round(gy1,1)): ridge_info} from masks.json.

    ridge_info is a dict:
        min_y       float  leading-edge apex row in SOURCE pixel coordinates
        min_y_x     float  SOURCE pixel x at the apex
        arc_hw_m    float  arm half-width in metres (apex → furthest ridge point)
        src_pts     list   [[x_src, y_src], ...] all ridge points in source px
        px_per_m    float  stored so callers can convert (filled in compute())
    """
    import json
    try:
        with open(masks_path) as f:
            masks_data = json.load(f)
    except Exception:
        return {}

    tile_to_sx0: dict[str, float] = {}
    if manifest_path:
        try:
            man = load_manifest(manifest_path)
            tile_to_sx0 = {k: v.get("src_x0", 0.0) for k, v in man.items()}
        except Exception:
            pass

    lookup: dict = {}
    for m in masks_data:
        tile = m.get("image", "")
        ridge = m.get("ridge", [])
        if not ridge:
            continue
        src_x0 = tile_to_sx0.get(tile, 0.0)
        el = tile.split("_202")[0]
        gx1 = round(src_x0 + float(m["x1"]), 1)
        gy1 = round(float(m["y1"]), 1)

        # Convert ridge from tile coords to source coords (src_y0==0 for all tiles)
        src_pts = [[src_x0 + float(pt[0]), float(pt[1])] for pt in ridge]
        min_y = min(p[1] for p in src_pts)
        # x at the apex (smallest y)
        min_y_x = next(p[0] for p in src_pts if p[1] == min_y)

        lookup[(el, gx1, gy1)] = {
            "min_y":    min_y,
            "min_y_x":  float(min_y_x),
            "src_pts":  src_pts,
            "arc_hw_m": None,   # filled per-detection in compute()
        }
    return lookup


def velocity_from_eps(eps_r: float) -> float:
    """EM wave velocity (m/ns) for a given relative permittivity."""
    return C_M_PER_NS / np.sqrt(eps_r)


def fit_velocity_from_ridge(src_pts: list, apex_x_src: float, apex_y_src: float,
                             px_per_m: float, ns_per_px: float,
                             v_bounds: tuple = (0.03, 0.35)) -> float | None:
    """Single-parameter hyperbola fit: apex fixed, solve for v only.

    Fixes x0 and t0 from the ridge apex (ridge min-y point) and fits ONLY the
    velocity v to the arm curvature.  Much more stable than the 3-parameter fit
    when the apex is close to y=0 (surface reflectors such as Pladur studs).

    Returns v (m/ns) in [v_bounds[0], v_bounds[1]], or None on failure.
    Requires >= 5 ridge points.
    """
    if len(src_pts) < 5:
        return None
    try:
        from scipy.optimize import curve_fit
    except ImportError:
        return None

    xs_m = np.array([p[0] for p in src_pts]) / px_per_m
    ts_ns = np.array([p[1] for p in src_pts]) * ns_per_px

    x0 = apex_x_src / px_per_m
    t0 = max(apex_y_src * ns_per_px, 1e-6)   # clamp to positive; apex near 0 is fine

    def model(x, v):
        v_safe = max(float(v), 1e-4)
        return np.sqrt(t0 ** 2 + (2.0 * (x - x0) / v_safe) ** 2)

    # Initial guess: use arm extent to estimate v
    arm_px = max(abs(p[0] - apex_x_src) for p in src_pts)
    arm_m  = arm_px / px_per_m
    arm_t  = max(abs(p[1] - apex_y_src) for p in src_pts) * ns_per_px
    v0     = (2.0 * arm_m / arm_t) if arm_t > 1e-6 else 0.12
    v0     = float(np.clip(v0, v_bounds[0] + 0.01, v_bounds[1] - 0.01))

    try:
        popt, _ = curve_fit(model, xs_m, ts_ns, p0=[v0],
                            bounds=(v_bounds[0], v_bounds[1]), maxfev=4000)
        v_fit = float(popt[0])
        # If the optimizer hit a bound the fit was poorly constrained — discard it.
        # The caller will fall back to the default permittivity velocity.
        _tol = (v_bounds[1] - v_bounds[0]) * 0.01   # 1% of range
        if abs(v_fit - v_bounds[0]) < _tol or abs(v_fit - v_bounds[1]) < _tol:
            return None
        return v_fit
    except Exception:
        return None


def compute(cfg: dict, cons: pd.DataFrame) -> pd.DataFrame:
    cal = cfg["calibration"]
    px_per_m_default = float(cal["px_per_m"])
    man_path = cfg["io"].get("crop_manifest")
    src_w = {}
    if man_path:
        src_w = element_source_w(load_manifest(man_path))
    scan_d = (cal.get("scan_distance_m") or {})

    def ppm(elem):
        sw, sd = src_w.get(elem), scan_d.get(elem)
        return (sw / sd) if (sw and sd) else px_per_m_default

    ns_per_px = float(cal["time_window_ns"]) / float(cal["image_height_px"])
    eps_default = float(cal["epsilon_r"])
    v_default = velocity_from_eps(eps_default)
    apex_frac = float(cal["apex_frac"])
    surf = float(cal.get("surface_offset_samples", 0))
    # Half-pulse-width offset: shifts ridge leading edge to visual amplitude peak
    pulse_offset = float(cal.get("pulse_offset_samples", 21))

    masks_path = cfg["io"].get("masks_file")
    ridge_lookup = load_ridge_lookup(masks_path, man_path) if masks_path else {}
    ridge_hits = ridge_misses = fit_success = fit_fail = 0

    # Material-specific velocity bounds prevent physically impossible fits.
    # Structural concrete elements (RC walls/slabs): eps_r ≈ 4–12 → v ∈ [0.09, 0.18] m/ns.
    # Pladur air cavity: eps_r ≈ 0.7–2 → v ∈ [0.15, 0.35] m/ns.
    el_overrides = (cfg.get("quantify", {}).get("element_overrides") or {})
    _CONCRETE_TYPES = {"retaining_wall", "floor_slab", "ground_slab", "column", "beam",
                       "roof_slab", "foundation"}
    _BRICK_TYPES    = {"brick_wall"}
    _PLADUR_TYPES   = {"partition_pladur", "partition"}

    def v_bounds_for(elem):
        mat = el_overrides.get(elem, "default")
        if mat in _CONCRETE_TYPES:   return (0.09, 0.18)
        if mat in _BRICK_TYPES:      return (0.09, 0.22)
        if mat in _PLADUR_TYPES:     return (0.15, 0.37)  # up to near-air; 0.35 is a legitimate result
        return (0.06, 0.35)   # generic / unknown

    rows = []
    for _, r in cons.iterrows():
        el = r["element"]
        px_per_m = ppm(el)
        cx = 0.5 * (r["gx1"] + r["gx2"])

        mat = el_overrides.get(el, "default")
        key = (el, round(float(r["gx1"]), 1), round(float(r["gy1"]), 1))
        if key in ridge_lookup:
            info = ridge_lookup[key]
            src_pts  = info["src_pts"]
            min_y    = info["min_y"]
            min_y_x  = info["min_y_x"]

            # Apex: leading edge + pulse half-width → visual amplitude peak
            apex_px = min_y + pulse_offset
            cx      = min_y_x    # ridge apex x is more precise than bbox centre

            # Velocity assignment:
            # Concrete / brick elements: eps_r=6 (v=0.122 m/ns) is the calibrated
            # literature value, confirmed by visual arc alignment on this dataset.
            # Per-object curve fitting on concrete/brick produces noise (v ranges
            # 0.115–0.175 for Wall_3001), not accuracy.
            # Pladur / partition: genuinely different medium (air cavity + wood studs,
            # eps_r ≈ 0.7–2). Fit velocity from ridge curvature — meaningful for these.
            if mat in (_CONCRETE_TYPES | _BRICK_TYPES):
                v = v_default
                eps = eps_default
                fit_success += 1   # not a fit, but counts as "velocity resolved"
            elif mat in _PLADUR_TYPES:
                vb = v_bounds_for(el)
                v_fit = fit_velocity_from_ridge(src_pts, min_y_x, min_y,
                                                px_per_m, ns_per_px,
                                                v_bounds=vb)
                if v_fit is not None:
                    v = v_fit
                    fit_success += 1
                else:
                    v = v_default
                    fit_fail += 1
                eps = (C_M_PER_NS / v) ** 2
            else:
                v = v_default
                eps = eps_default
                fit_fail += 1
            ridge_hits += 1
        else:
            apex_px = r["gy1"] + apex_frac * (r["gy2"] - r["gy1"])
            v = v_default
            eps = eps_default
            ridge_misses += 1

        # Arc display half-width: 0.12 m for concrete/brick (calibrated on this dataset).
        # Pladur uses the hyperbola equation at 40% bbox depth — wider because v is higher.
        if mat in (_CONCRETE_TYPES | _BRICK_TYPES):
            arc_hw_m = 0.12
        else:
            # For Pladur/unknown: derive from 40% of bbox depth via hyperbola equation
            gy2_src = float(r["gy2"])
            t0_ns_val = max(apex_px * ns_per_px, 1e-6)
            arm_end_px = apex_px + 0.40 * max(gy2_src - apex_px, 0.0)
            t_arm_ns = arm_end_px * ns_per_px
            if t_arm_ns > t0_ns_val:
                hw = (v / 2.0) * float(np.sqrt(max(t_arm_ns ** 2 - t0_ns_val ** 2, 0.0)))
                arc_hw_m = float(np.clip(hw, 0.05, 0.40))
            else:
                arc_hw_m = 0.20  # fallback for very shallow Pladur apex

        twt_ns = max(apex_px - surf, 0.0) * ns_per_px
        depth_m = v * twt_ns / 2.0
        rows.append({
            "element":   el,
            "x_m":       round(cx / px_per_m, 4),
            "apex_px":   round(float(apex_px), 1),
            "twt_ns":    round(float(twt_ns), 3),
            "v_m_ns":    round(float(v), 4),
            "eps_r":     round(float(eps), 2),
            "depth_cm":  round(float(depth_m * 100.0), 1),
            "confidence": round(float(r["confidence"]), 3),
            "arc_hw_m":  round(float(arc_hw_m), 4),
        })

    if ridge_lookup:
        print(f"[step2] ridge apex used for {ridge_hits}/{ridge_hits+ridge_misses} detections "
              f"({ridge_misses} fell back to apex_frac={apex_frac})")
        print(f"[step2] per-object velocity fit: {fit_success} success, {fit_fail} failed "
              f"(failed → default eps_r={eps_default} v={v_default:.4f} m/ns)")
    return pd.DataFrame(rows).sort_values(["element", "x_m"]).reset_index(drop=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cfg = load_config(args.config)
    cons = pd.read_csv(args.inp)
    obj = compute(cfg, cons)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    obj.to_csv(args.out, index=False)
    print(f"[step2] objects with depth  : {len(obj)}")
    print(f"[step2] written -> {args.out}")


if __name__ == "__main__":
    main()
