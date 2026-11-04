from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure

UNDETERMINED = "UNDETERMINED"
NOT_EVALUABLE = "NOT EVALUABLE"
GROUP_GREY = "#7f8c8d"


def save_spectrum_images(bj, an, out_dir: Path, dpi: int) -> int:
    """One PNG per spectrum using Benjamin's own figure (`draw_result`), unchanged."""
    out_dir.mkdir(parents=True, exist_ok=True)
    n = len(an.valid)
    width = max(5, len(str(n)))
    fig = Figure(figsize=(11, 6.6), dpi=dpi)
    FigureCanvasAgg(fig)
    for i in range(n):
        bj.draw_result(fig, an, i)
        fig.savefig(out_dir / f"spectrum_{i + 1:0{width}d}.png", dpi=dpi)
    return n


def _segment_lines(an, idx: np.ndarray) -> list[np.ndarray]:
    """Polylines for the spectra in `idx`, one per sensor segment (the gap stays empty)."""
    lines = []
    for i in idx:
        y = an.X_raw[i].astype(float)
        y = np.where(y == 0, np.nan, y)
        for s, e in an.segments:
            pts = np.column_stack([an.wl[s:e], y[s:e]])
            pts = pts[np.isfinite(pts[:, 1])]
            if len(pts) > 1:
                lines.append(pts)
    return lines


def save_material_overview(bj, an, path: Path, dpi: int) -> None:
    """Summary figure: one panel per classification result, all its spectra overlaid.

    Panels follow Benjamin's verdict: each material, then UNDETERMINED (below the
    confidence threshold) and NOT EVALUABLE (rejected by the quality filter).
    """
    verdicts = np.array([an.verdict(i)[0] for i in range(len(an.valid))], dtype=object)
    materials = [v for v in dict.fromkeys(verdicts) if v not in (UNDETERMINED, NOT_EVALUABLE)]
    materials.sort(key=lambda v: (-(verdicts == v).sum(), v))
    groups = materials + [v for v in (UNDETERMINED, NOT_EVALUABLE) if (verdicts == v).any()]

    ncols = min(3, max(1, len(groups)))
    nrows = max(1, math.ceil(len(groups) / ncols))
    fig = Figure(figsize=(5.2 * ncols, 3.6 * nrows + 0.8), dpi=dpi)
    FigureCanvasAgg(fig)
    fig.patch.set_facecolor("white")
    n_total = len(verdicts)
    fig.suptitle(f"Hyperspectral classification - {n_total} spectra "
                 f"(confidence threshold {an.min_conf:.0%})", fontsize=13, fontweight="bold")

    if not groups:
        fig.text(0.5, 0.5, "No spectra", ha="center", va="center", fontsize=14)
        fig.savefig(path, dpi=dpi)
        return

    names = ["VIS", "NIR"] + [f"S{k}" for k in range(3, 10)]
    for k, group in enumerate(groups):
        ax = fig.add_subplot(nrows, ncols, k + 1)
        idx = np.flatnonzero(verdicts == group)
        color = bj.CLASS_COLORS.get(group.lower(), GROUP_GREY)
        if bj._luminance(color) > 0.55:
            color = bj._darken(color, 0.6)

        lines = _segment_lines(an, idx)
        alpha = float(np.clip(6.0 / math.sqrt(max(len(idx), 1)), 0.04, 0.9))
        ax.add_collection(LineCollection(lines, colors=color, linewidths=0.8, alpha=alpha))

        ys = np.concatenate([l[:, 1] for l in lines]) if lines else np.array([0.0, 1.0])
        lo, hi = np.percentile(ys, [0.5, 99.5])
        lo, hi = min(0.0, lo), max(1.0, hi)
        pad = 0.06 * (hi - lo)
        ax.set_ylim(max(lo - pad, -0.6), min(hi + pad, 2.6))
        ax.set_xlim(an.cfg["WL_MIN"], an.cfg["WL_MAX"])

        for (s0, e0), (s1, _) in zip(an.segments[:-1], an.segments[1:]):
            a, b = an.wl[e0 - 1], an.wl[s1]
            ax.axvspan(a, b, facecolor="#ecf0f1", edgecolor="#b2bec3", hatch="//", lw=0, zorder=0)
        for j, (s, e) in enumerate(an.segments):
            ax.text((an.wl[s] + an.wl[e - 1]) / 2, 0.98, names[j], transform=ax.get_xaxis_transform(),
                    ha="center", va="top", fontsize=8, fontweight="bold", color=GROUP_GREY)

        title = f"{group}  (n = {len(idx)})"
        conf = an.conf[idx]
        if np.isfinite(conf).any():
            title += f"  ·  mean conf {np.nanmean(conf):.0%}"
        ax.set_title(title, fontsize=10, fontweight="bold", color=color, loc="left")
        ax.grid(alpha=.25)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        if k % ncols == 0:
            ax.set_ylabel("Measured reflectance", fontsize=8)
        if k >= len(groups) - ncols:
            ax.set_xlabel("Wavelength (nm)", fontsize=8)
        ax.tick_params(labelsize=7)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(path, dpi=dpi)
