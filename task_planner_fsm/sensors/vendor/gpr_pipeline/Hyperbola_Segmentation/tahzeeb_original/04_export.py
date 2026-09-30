"""Step 4 - Export deliverables.

Writes:
* ``gpr_data.json``       - per-element objects (x, depth, confidence) for the
                            digital-twin viewer / material-map platforms.
* ``material_passport.csv`` - one audit row per element (type, interpretation,
                            quantities, value, CO2) for the pre-demolition audit.

Input : objects.csv, element_summary.csv
Output: gpr_data.json, material_passport.csv

Usage:
    python 04_export.py --config config.yaml --objects results/<ts>/objects.csv \
        --summary results/<ts>/element_summary.csv --outdir results/<ts>/
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from gpr_pp_utils import load_config

# Schema v2 (2026-07-03): adds measured-velocity provenance. Per object: "v" (its own
# Step-2 velocity, m/ns) + "vFit" (False when the object fell back to the calibrated
# eps_r=6 constant instead of a ridge-curvature fit). Per element: "velMns" (median of
# the per-object velocities, preferring fitted ones), "velSigmaMns" (MAD-based robust
# sigma, floored at 0.005 m/ns) and "velSource". All v1 fields are unchanged.
SCHEMA_VERSION = 2
VEL_SIGMA_FLOOR_MNS = 0.005


def _num(v):
    """Return float(v) or None for NaN/blank — keeps the JSON clean for the platform."""
    try:
        return None if pd.isna(v) else float(v)
    except Exception:
        return None


def element_velocity_stats(vels: np.ndarray, fitted: np.ndarray) -> tuple[float, float]:
    """Median + robust sigma (MAD x 1.4826, floor 0.005 m/ns) of an element's velocities.

    Uses only the fitted velocities when any exist (Pladur elements); otherwise all
    objects share the calibrated constant, so the median is that constant and sigma
    collapses to the floor.
    """
    v = vels[fitted] if fitted.any() else vels
    med = float(np.median(v))
    sigma = float(1.4826 * np.median(np.abs(v - med)))
    return med, max(sigma, VEL_SIGMA_FLOOR_MNS)


def build_viewer_json(obj: pd.DataFrame, summ: pd.DataFrame, cfg: dict) -> dict:
    scan_dist_map = cfg.get("calibration", {}).get("scan_distance_m", {})
    eps_default = float(cfg.get("calibration", {}).get("epsilon_r", 6.0))
    has_vel = "v_m_ns" in obj.columns and "eps_r" in obj.columns
    elements = []
    smap = {r["element"]: r for _, r in summ.iterrows()}
    for el, g in obj.groupby("element"):
        xs = g["x_m"]
        x0 = float(xs.min())
        span_m = float(scan_dist_map.get(el, float(xs.max()) - x0)) if scan_dist_map.get(el) else float(xs.max()) - x0
        span_m = span_m or 1.0
        # Each object carries normalised x (0-1) AND the physical horizontal distance
        # xM in metres, the depth (cm) and the detection confidence.
        objs = []
        for r in g.itertuples():
            o = {"x": round(min(max(float(r.x_m) / span_m, 0.0), 1.0), 4),
                 "xM": round(float(r.x_m), 3),
                 "d": float(r.depth_cm),
                 "c": float(r.confidence)}
            if has_vel:
                # vFit False = the object carries the calibrated eps_r=6 constant
                # (concrete/brick by design, or a Pladur ridge fit that failed).
                o["v"] = round(float(r.v_m_ns), 4)
                o["vFit"] = bool(abs(float(r.eps_r) - eps_default) > 1e-6)
            objs.append(o)
        s = smap.get(el)
        layer_depths = []
        if s is not None and isinstance(s["layer_depths_cm"], str):
            layer_depths = [float(x) for x in str(s["layer_depths_cm"]).split(";") if x]
        # Geometry + material-intelligence quantities travel together in one record.
        rec = {
            "id": el,
            "name": el,
            "type": (s["type"] if s is not None else "wall"),
            "spanM": round(span_m, 2),
            "nObjects": int(len(g)),
            "layers": int(s["n_layers"]) if s is not None else 1,
            "layerDepthsCm": layer_depths,
            "coverMinCm": (_num(s["cover_min_cm"]) if s is not None else None),
            "spacingCm": (None if s is None or pd.isna(s["bar_pitch_cm"]) else float(s["bar_pitch_cm"])),
            "spacingSdCm": (_num(s["bar_pitch_sd_cm"]) if s is not None else None),
            "recoverableSteelKg": (_num(s["recoverable_steel_kg"]) if s is not None else None),
            "valueEur": (_num(s["value_eur"]) if s is not None else None),
            "co2AvoidedKg": (_num(s["co2_avoided_kg"]) if s is not None else None),
            "interpretation": (s["interpretation"] if s is not None else ""),
            "objects": objs,
        }
        if has_vel:
            vel_med, vel_sigma = element_velocity_stats(
                g["v_m_ns"].to_numpy(dtype=float),
                (g["eps_r"].to_numpy(dtype=float) - eps_default).__abs__() > 1e-6)
            rec["velMns"] = round(vel_med, 4)
            rec["velSigmaMns"] = round(vel_sigma, 4)
            rec["velSource"] = "step2_hyperbola_fit"
        elements.append(rec)
    return {"schemaVersion": SCHEMA_VERSION, "elements": elements}


def _img_data_uri(path, max_px: int = 1600) -> str | None:
    """Read an image, downscale to max_px on the long side, return a base64 JPEG data URI.

    Matches the platform's own upload handling (1600 px, JPEG 85). Returns None if the file
    is missing or Pillow is unavailable.
    """
    try:
        import base64
        import io as _io
        from PIL import Image
    except Exception:
        return None
    p = Path(path)
    if not p.exists():
        return None
    im = Image.open(p).convert("RGB")
    w, h = im.size
    sc = min(1.0, max_px / max(w, h))
    if sc < 1.0:
        im = im.resize((max(1, int(w * sc)), max(1, int(h * sc))))
    buf = _io.BytesIO()
    im.save(buf, "JPEG", quality=85)
    return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def build_images(cfg: dict, element_ids: list[str], bscan_dir: str | None) -> dict:
    """Build the platform `images` block (+ `plan`) so JSON import loads them automatically.

    Keys the platform reads: `<id>` = site photo, `bscan_<id>` = GPR B-scan, top-level `plan`.
    B-scans come from <bscan_dir>/<id>_clean_bscan.png; photos/drawings/plan from config.io.
    Anything missing is simply skipped (manual upload still works in the platform).
    """
    io_cfg = cfg.get("io", {}) or {}
    photos = io_cfg.get("element_photos", {}) or {}
    drawings = io_cfg.get("element_drawings", {}) or {}
    images: dict[str, str] = {}
    for el in element_ids:
        if bscan_dir:
            uri = _img_data_uri(str(Path(bscan_dir) / f"{el}_clean_bscan.png"))
            if uri:
                images["bscan_" + el] = uri
        if photos.get(el):
            uri = _img_data_uri(photos[el])
            if uri:
                images[el] = uri
        if drawings.get(el):
            uri = _img_data_uri(drawings[el], max_px=2000)
            if uri:
                images["dwg_" + el] = uri      # platform key for the per-element drawing
    out: dict = {"images": images}
    if io_cfg.get("site_plan"):
        puri = _img_data_uri(io_cfg["site_plan"], max_px=2400)
        if puri:
            out["plan"] = puri
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--objects", required=True)
    ap.add_argument("--summary", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--embed-images", action="store_true",
                    help="embed B-scan/photo/plan as base64 in gpr_data.json for one-click "
                         "platform import (JSON grows by a few MB)")
    ap.add_argument("--bscan-dir", default=None,
                    help="folder with <element>_clean_bscan.png to embed (from make_bscan_images.py)")
    ap.add_argument("--json-name", default="gpr_data.json",
                    help="output JSON filename inside --outdir (e.g. gpr_data_v2.json to "
                         "re-export without overwriting the original)")
    args = ap.parse_args()
    cfg = load_config(args.config)
    obj = pd.read_csv(args.objects)
    summ = pd.read_csv(args.summary)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    viewer = build_viewer_json(obj, summ, cfg)
    if args.embed_images:
        block = build_images(cfg, [e["id"] for e in viewer["elements"]], args.bscan_dir)
        viewer.update(block)
        print(f"[step4] embedded {len(block.get('images', {}))} images"
              + (" + site plan" if "plan" in block else ""))
    (outdir / args.json_name).write_text(json.dumps(viewer, indent=2), encoding="utf-8")

    passport = summ[[
        "element", "type", "interpretation", "n_objects", "n_layers",
        "bar_pitch_cm", "cover_min_cm", "area_m2",
        "recoverable_steel_kg", "value_eur", "co2_avoided_kg",
    ]].copy()
    passport.to_csv(outdir / "material_passport.csv", index=False)

    print(f"[step4] viewer json -> {outdir / args.json_name} ({len(viewer['elements'])} elements)")
    print(f"[step4] passport    -> {outdir / 'material_passport.csv'}")


if __name__ == "__main__":
    main()
