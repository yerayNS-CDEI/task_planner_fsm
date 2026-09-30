"""Step 5 - Visualisation of the post-processing for progress review & figures.

For every element it renders a panel showing, on the element's B-scan (if a path
is given in config.io.bscan_images, else a dark canvas):
  * kept hyperbola apices (depth-coloured dots) and their fitted t(x) arcs,
  * removed duplicates / deep multiples (faint grey), if the *_removed.csv exists.

Arc parameters are taken per-object from objects_dedup.csv:
  - v_m_ns:   per-object fitted velocity (from 02_hyperbola_fit ridge fitting)
  - arc_hw_m: arm half-width in metres (from ridge extent stored in step 2)
  - twt_ns:   apex TWT including the pulse-offset correction (visual peak)

Outputs one PNG per element plus a combined figure, suitable for the manuscript.

Input : objects_dedup.csv (+ optional *_removed.csv), config
Output: viz/<element>.png, viz/all_elements.png

Usage:
    python 05_visualize.py --config config.yaml --objects results/<ts>/objects_dedup.csv \
        --outdir results/<ts>/viz
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import cm

from gpr_pp_utils import load_config

C = 0.299792458

# Default arc half-width when arc_hw_m is absent from the CSV (legacy runs)
_DEFAULT_ARC_HW_M = 0.12


def fitted_arc(x0_m: float, t0_ns: float, v: float,
               half_w_m: float = _DEFAULT_ARC_HW_M, n: int = 60):
    """Return (x_m, t_ns) curve of t(x)=sqrt(t0^2+(2(x-x0)/v)^2)."""
    xs = np.linspace(x0_m - half_w_m, x0_m + half_w_m, n)
    ts = np.sqrt(np.maximum(t0_ns, 1e-6) ** 2 + (2.0 * (xs - x0_m) / max(v, 1e-4)) ** 2)
    return xs, ts


def _arc_hw(r) -> float:
    """Get arc half-width for a dedup row; fall back to default if column absent."""
    try:
        hw = float(r.arc_hw_m)
        return hw if hw > 0.01 else _DEFAULT_ARC_HW_M
    except AttributeError:
        return _DEFAULT_ARC_HW_M


def draw_element(ax, el, g, removed, cfg, bscan_path):
    cal = cfg["calibration"]
    ns_per_px = float(cal["time_window_ns"]) / float(cal["image_height_px"])
    tw = float(cal["time_window_ns"])
    # Use configured scan distance so the B-scan image is displayed at the correct
    # physical scale regardless of where the last detection falls.
    scan_d = (cal.get("scan_distance_m") or {})
    x_max_det = float(g["x_m"].max()) if len(g) else 1.0
    span = float(scan_d.get(el, x_max_det))

    img = None
    if bscan_path and Path(bscan_path).exists():
        try:
            from PIL import Image
            raw = Image.open(bscan_path).convert("L")
            img = np.asarray(raw)
        except Exception:
            img = None
    if img is not None:
        img_h = img.shape[0]
        tile_rows = min(640, img_h)
        tw_display = tile_rows * ns_per_px
        ax.imshow(img[:tile_rows, :], extent=[0, span, tw_display, 0],
                  aspect="auto", cmap="gray",
                  vmin=np.percentile(img, 1), vmax=np.percentile(img, 99))
    else:
        ax.set_facecolor("#0b1320")
        tw_display = tw

    # removed (faint)
    if removed is not None and len(removed):
        rg = removed[removed["element"] == el]
        if "twt_ns" in rg.columns and len(rg):
            ax.scatter(rg["x_m"], rg["twt_ns"],
                       s=14, facecolors="none", edgecolors="#888", alpha=.5, label="removed")

    # kept apices + fitted arcs, coloured by depth
    if len(g):
        dmax = max(8.0, g["depth_cm"].max())
        for r in g.itertuples():
            col = cm.viridis(min(r.depth_cm / dmax, 1.0))
            # Use per-object velocity and arc half-width (from ridge fitting in step 2).
            # v_m_ns encodes the correct arm curvature for each material type:
            #   - concrete/RC (eps_r≈6): v≈0.122 m/ns  → steep narrow arms
            #   - Pladur partition (air cavity): v≈0.30-0.35 m/ns → shallow wide arms
            # arc_hw_m is the lateral extent of the detected ridge, not a fixed constant.
            xs, ts = fitted_arc(r.x_m, r.twt_ns, float(r.v_m_ns), half_w_m=_arc_hw(r))
            ax.plot(xs, ts, "-", color=col, lw=1.6, alpha=.9)
            ax.plot(r.x_m, r.twt_ns, "o", color=col, ms=4, mec="white", mew=.4)

    y_lim = tw_display if img is not None else tw
    ax.set_xlim(0, span); ax.set_ylim(y_lim, 0)
    ax.set_xlabel("distance (m)"); ax.set_ylabel("two-way time (ns)")
    ax.set_title(f"{el}  ·  {len(g)} hyperbolae (fitted)", fontsize=10, fontweight="bold")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--objects", required=True)
    ap.add_argument("--outdir", required=True)
    args = ap.parse_args()
    cfg = load_config(args.config)
    obj = pd.read_csv(args.objects)
    rem_path = args.objects.replace(".csv", "_removed.csv")
    removed = pd.read_csv(rem_path) if Path(rem_path).exists() else None
    bscans = (cfg.get("io", {}) or {}).get("bscan_images", {}) or {}
    out = Path(args.outdir); out.mkdir(parents=True, exist_ok=True)

    els = list(obj.groupby("element"))
    for el, g in els:
        fig, ax = plt.subplots(figsize=(11, 3.4), dpi=150)
        draw_element(ax, el, g, removed, cfg, bscans.get(el))
        fig.tight_layout(); fig.savefig(out / f"{el}.png", dpi=150); plt.close(fig)

    n = len(els)
    fig, axes = plt.subplots(n, 1, figsize=(11, 3.0 * n), dpi=150)
    if n == 1:
        axes = [axes]
    for ax, (el, g) in zip(axes, els):
        draw_element(ax, el, g, removed, cfg, bscans.get(el))
    fig.suptitle("Post-processing: detected, de-duplicated and fitted hyperbolae",
                 fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(out / "all_elements.png", dpi=150); plt.close(fig)
    print(f"[step5] wrote {n+1} figures -> {out}")


if __name__ == "__main__":
    main()
