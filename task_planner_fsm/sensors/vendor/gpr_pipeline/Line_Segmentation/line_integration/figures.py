"""Client-facing depth figure for the horizontal-line branch.

Layout and colours follow Tahzeeb's v2 bscan_figure.result_figure (GPRiQ / MIP-R standard):
B-scan with depth below the front face on the left axis and two-way time on the right,
the averaged trace beside it with every detected line, and the result table below.
No text is written on the radargram itself; labels sit in the side panels and the legend.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.lines as mlines  # noqa: E402
import matplotlib.patches as mp  # noqa: E402
import matplotlib.patheffects as pe  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.ticker import FuncFormatter, MultipleLocator  # noqa: E402
from scipy.signal import hilbert  # noqa: E402

STD = {"front": "#7b61ff", "line": "#ff8a00", "excl": "#e62828", "band_alpha": 0.18}
INK, MUTED = "#1F2A33", "#5A6773"
HALO = [pe.withStroke(linewidth=3.0, foreground="black", alpha=0.45)]
FIG_W, FIG_H = 13.0, 8.4


def _segments(prep: dict[str, Any], berta_lines: list[dict], result_lines: list[dict]) -> list[dict]:
    """Berta's rendered-pixel segments -> metres along the scan and raw two-way time [ns]."""
    W, H = int(prep["image_width_px"]), int(prep["image_height_px"])
    n = int(prep["raw_n_samples"])
    dt, L = float(prep["dt_ns_per_sample"]), float(prep["scan_distance_m"])
    px_to_t = lambda y: float(y) / max(H - 1, 1) * (n - 1) * dt  # noqa: E731
    px_to_x = lambda x: float(x) / max(W - 1, 1) * L  # noqa: E731
    out = []
    for b, r in zip(berta_lines, result_lines):
        out.append({
            "id": r["id"], "x0": px_to_x(b["x1"]), "x1": px_to_x(b["x2"]),
            "t0": px_to_t(b["y1"]), "t1": px_to_t(b["y2"]),
            "half_thick": 0.5 * px_to_t(b["grosor_px"]),
            "before_t0": bool(r["before_time_zero_reference"]), "r": r,
        })
    return out


def _time_window(segs: list[dict], T0: float, t_end: float) -> float:
    """Deepest line plus half the depth to it (at least 2 ns), never past the record."""
    deep = max([T0] + [max(s["t0"], s["t1"]) + s["half_thick"] for s in segs])
    return float(min(t_end, deep + max(0.5 * (deep - T0), 2.0)))


def _depth_ticks(sec, span_cm: float) -> None:
    step = 1 if span_cm <= 12 else (2 if span_cm <= 25 else (5 if span_cm <= 60 else 10))
    sec.yaxis.set_major_locator(MultipleLocator(step))
    sec.yaxis.set_minor_locator(MultipleLocator(step / 5))
    sec.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}" if v >= -1e-9 else ""))  # no depth in the air


def _fmt(v: Any, nd: int = 1) -> str:
    return "-" if v is None else f"{float(v):.{nd}f}"


def draw_depth_figure(prep: dict[str, Any], berta_lines: list[dict], result: dict[str, Any],
                      velocity_m_ns: float, output_path: Path) -> None:
    M = np.asarray(prep["gain_matrix"], float)
    n, nt = M.shape
    dt, L = float(prep["dt_ns_per_sample"]), float(prep["scan_distance_m"])
    T0 = float(prep["time_zero_sample"]) * dt
    t = np.arange(n) * dt
    t2d = lambda tt: (np.asarray(tt, float) - T0) * velocity_m_ns / 2 * 100  # noqa: E731  ns -> cm
    d2t = lambda d: T0 + np.asarray(d, float) / 100 * 2 / velocity_m_ns  # noqa: E731

    segs = _segments(prep, berta_lines, result["lines"])
    tmax = _time_window(segs, T0, n * dt)
    i0, i1 = int(T0 / dt), max(int(T0 / dt) + 2, int(tmax / dt))
    clip = float(np.percentile(np.abs(M[i0:i1]), 99)) or 1.0

    fig = plt.figure(figsize=(FIG_W, FIG_H))
    gs = fig.add_gridspec(2, 2, width_ratios=[3.3, 1], height_ratios=[2.4, 1], hspace=0.16, wspace=0.09,
                          left=0.065, right=0.95, top=0.87, bottom=0.05)
    ax = fig.add_subplot(gs[0, 0])
    ae = fig.add_subplot(gs[0, 1], sharey=ax)
    at = fig.add_subplot(gs[1, 0])
    ac = fig.add_subplot(gs[1, 1])

    # --- B-scan (gain only, the detector's input) with the front face and the detected lines
    ax.imshow(M, aspect="auto", cmap="gray", vmin=-clip, vmax=clip, interpolation="nearest",
              extent=[0, L, n * dt, 0])
    ax.set_ylim(tmax, max(0.0, T0 - 0.5))
    ax.set_xlim(0, L)
    ax.axhline(T0, color=STD["front"], lw=1.3, zorder=5)
    for s in segs:
        col = STD["excl"] if s["before_t0"] else STD["line"]
        xs, ts = [s["x0"], s["x1"]], np.array([s["t0"], s["t1"]])
        ax.fill_between(xs, ts - s["half_thick"], ts + s["half_thick"], color=col, alpha=STD["band_alpha"], lw=0,
                        zorder=3)
        ax.plot(xs, ts, color=col, lw=1.9, ls="--" if s["before_t0"] else "-", zorder=6, path_effects=HALO)
    ax.tick_params(labelleft=False, left=False)
    ax.set_xlabel("distance along the line [m]")
    dep = ax.secondary_yaxis("left", functions=(t2d, d2t))
    _depth_ticks(dep, float(t2d(tmax) - t2d(max(0.0, T0 - 0.5))))
    dep.set_ylabel("depth below the front face [cm]")

    # --- averaged trace with every detected line (Tahzeeb: 'candidate echoes')
    env = np.abs(hilbert(M, axis=0)).mean(axis=1)
    ae.plot(20 * np.log10(env / env.max() + 1e-6), t, color=INK, lw=1)
    ae.set_xlim(-40, 30)
    ae.axhline(T0, color=STD["front"], lw=1.3)
    for s in segs:
        r = s["r"]
        tm = 0.5 * (s["t0"] + s["t1"])
        col = STD["excl"] if s["before_t0"] else STD["line"]
        ae.axhline(tm, xmax=0.6, color=col, lw=2.0)
        label = f"{r['id']}  {_fmt(r['depth_cm_approx'])} cm" if r["depth_cm_approx"] is not None else \
            f"{r['id']}  above t0"
        ae.text(3, tm, label, va="center", fontsize=8, color=INK, fontweight="bold")
    ae.set_xlabel("average echo strength [dB]")
    ae.set_title("detected lines\nid, depth", fontsize=9, loc="left")
    ae.yaxis.tick_right()
    ae.yaxis.set_label_position("right")
    ae.set_ylabel("two-way time from time-zero [ns]")
    step = 0.5 if tmax - T0 <= 4 else 1.0
    ae.set_yticks(T0 + np.arange(0.0, tmax - T0 + 1e-9, step))
    ae.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v - T0:.1f}"))

    # --- result table
    at.axis("off")
    cols = ["line", "depth [cm]", "band [cm]", "to next [cm]", "TWT [ns]", "extent [m]", "confidence"]
    rows = [[r["id"], _fmt(r["depth_cm_approx"]), _fmt(r["thickness_cm_approx"]),
             _fmt(r["distance_to_next_cm_approx"]), _fmt(r["twt_from_time_zero_ns"], 2),
             f"{s['x0']:.2f} - {s['x1']:.2f}", _fmt(r["confidence"], 2)]
            for s, r in ((s, s["r"]) for s in segs)]
    if rows:
        shown = rows[:8]
        tb = at.table(cellText=shown, colLabels=cols, loc="upper left", cellLoc="center", colLoc="center")
        tb.auto_set_font_size(False)
        tb.set_fontsize(8.5)
        tb.scale(1, 1.25)
        for (i, _), cell in tb.get_celld().items():
            cell.set_edgecolor("#D5DBE0")
            if i == 0:
                cell.set_facecolor("#EEF2F5")
                cell.set_text_props(fontweight="bold", color=INK)
        if len(rows) > len(shown):
            at.text(0, -0.02, f"+ {len(rows) - len(shown)} more lines in line_result.json", fontsize=8,
                    color=MUTED, transform=at.transAxes, va="top")
    else:
        at.text(0.0, 0.8, "No horizontal line detected", fontsize=12, color=INK, transform=at.transAxes)

    # --- result card
    ac.axis("off")
    cal = result["depth_calibration"]
    card = [("LINES", f"{result['n_lines']}"),
            ("time-zero", f"{T0:.3f} ns (sample {prep['time_zero_sample']})"),
            ("wave speed", f"{velocity_m_ns:.3f} m/ns (er {cal['epsilon_r_assumed']:g}, assumed)"),
            ("detector input", f"gain {float(prep['gain_db']):g} dB only"),
            ("record", f"{n * dt:.1f} ns, {nt} traces"),
            ("scan length", f"{L:.2f} m")]
    y = 0.98
    for k, v in card:
        big = k == "LINES"
        ac.text(0.04, y, k, fontsize=8.5, color=MUTED, va="top", transform=ac.transAxes)
        ac.text(0.42, y, v, fontsize=12 if big else 8.5, fontweight="bold" if big else None,
                color=STD["line"] if big else INK, va="top", transform=ac.transAxes)
        y -= 0.15 if big else 0.12

    handles = [mlines.Line2D([], [], color=STD["front"], lw=1.3, label="front face (time-zero, zero depth)"),
               mlines.Line2D([], [], color=STD["line"], lw=1.9, path_effects=HALO, label="detected horizontal line"),
               mp.Patch(color=STD["line"], alpha=0.35, label="line band (radar-image extent)")]
    if any(s["before_t0"] for s in segs):
        handles.append(mlines.Line2D([], [], color=STD["excl"], lw=1.9, ls="--", label="line above time-zero"))
    fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.055, 0.955), ncol=4, fontsize=8.5, frameon=False)
    fig.suptitle(prep["input_sgy"], x=0.065, y=0.99, ha="left", fontsize=10, color=MUTED)
    fig.text(0.95, 0.99, f"horizontal lines  ·  depth assumes er = {cal['epsilon_r_assumed']:g}", ha="right",
             va="top", fontsize=10, color=MUTED)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
