"""
Construction-material classifier from VIS-NIR spectral signatures.

LOCAL application (desktop window, no browser):
    python classifier.py                        -> opens the window and asks for a CSV
    python classifier.py samples.csv            -> opens the window with that CSV
    python classifier.py samples.csv --row 5 --png row5.png    -> saves one figure (no window)
    python classifier.py samples.csv --out-csv results.csv     -> saves only the summary table

For every spectrum it shows: the measured spectrum (VIS | GAP | NIR, gap left empty, not
interpolated), the detected material and a 0-100 spectral quality index.

Requirements: numpy pandas scipy scikit-learn joblib torch matplotlib  (tkinter ships with Python).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
import torch
from scipy.signal import medfilt, savgol_filter
from torch import nn

RANDOM_STATE = 42
TORCH_DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ════════════════════════════════════════════════════════════════════════════
# 1. Model classes (must stay compatible with classifier.joblib)
# ════════════════════════════════════════════════════════════════════════════
class _SegmentedCNN(nn.Module):
    def __init__(self, segments, n_cls, n_filters, kernel_size, n_blocks, dropout):
        super().__init__()
        self.segments = [tuple(s) for s in segments]
        self.branches = nn.ModuleList()
        n_feat = 0
        for s, e in self.segments:
            layers, c_in, length = [], 1, e - s
            for _ in range(n_blocks):
                c_out = n_filters * (2 ** (len(layers) // 4))
                layers += [
                    nn.Conv1d(c_in, c_out, kernel_size, padding=kernel_size // 2),
                    nn.BatchNorm1d(c_out),
                    nn.ReLU(),
                    nn.MaxPool1d(2),
                ]
                c_in, length = c_out, length // 2
            layers.append(nn.Flatten())
            self.branches.append(nn.Sequential(*layers))
            n_feat += c_in * length
        self.head = nn.Sequential(nn.Dropout(dropout), nn.Linear(n_feat, n_cls))

    def forward(self, x):
        z = [branch(x[:, :, start:end])
             for branch, (start, end) in zip(self.branches, self.segments)]
        return self.head(torch.cat(z, dim=1))


class CNN1DClassifier:
    """Sklearn-compatible wrapper, kept so the joblib bundle can be loaded."""

    def predict_proba(self, X):
        X = np.asarray(X, dtype=np.float32)
        Xt = torch.as_tensor(X).unsqueeze(1)
        self.model_.to(TORCH_DEVICE).eval()
        outputs = []
        with torch.inference_mode():
            for start in range(0, len(Xt), 2048):
                batch = Xt[start:start + 2048].to(TORCH_DEVICE)
                outputs.append(torch.softmax(self.model_(batch), dim=1).cpu())
        self.model_.cpu()
        return torch.cat(outputs).numpy()

    def predict(self, X):
        return self.classes_[self.predict_proba(X).argmax(axis=1)]


# The joblib was saved from __main__: if this file is imported as a module, register the
# classes in __main__ so joblib can rebuild them.
if __name__ != "__main__":
    _main = sys.modules.get("__main__")
    if _main is not None:
        for _n in ("_SegmentedCNN", "CNN1DClassifier", "RANDOM_STATE", "TORCH_DEVICE"):
            setattr(_main, _n, globals()[_n])


# ════════════════════════════════════════════════════════════════════════════
# 2. Configuration
# ════════════════════════════════════════════════════════════════════════════
DEFAULT_CFG = {
    "WL_MIN": 400.0, "WL_MAX": 1650.0, "SENSOR_GAP_NM": 50.0,
    "SEG_MISSING_MAX": 0.15, "SEG_GAP_MAX": 10,
    "FLAT_RANGE_MAX": 0.05, "FLAT_PCT": (10, 90), "FLAT_MODE": "all",
    "SG_SMOOTH": (11, 2), "SG_D1": (15, 3),
}
META_COLS = {"Measure Type", "Date", "Time", "Counter", "Label", "Class",
             "Material", "ID", "Object", "Sample", "Timestamp"}

CLASS_COLORS = {
    "cementitious": "#6c757d", "brick": "#f73e0b", "wood": "#7c4220",
    "polymer": "#2980b9", "metal": "#e2c12c", "ceramic": "#319b07",
    "gypsum": "#c6c6ae", "stone": "#7e47b2",
}

# Spectral-quality settings: this diagnostic measures ONLY high-frequency noise.
# HF is the noise sigma (4th-difference MAD) relative to each signature's signal LEVEL,
# so it does not depend on slopes or absorption bands. VIS and NIR use the same
# thresholds and noise-label criterion so neither region is favored.
# NOTE: starting values - recalibrate with calibrate_noise.py.
QUALITY_CFG = {
    # Tuple: (good_noise, bad_noise, local_window_threshold), shared by all sensors.
    "HF_VIS": (0.01, 0.06, 0.06),
    "HF_NIR": (0.01, 0.06, 0.06),
    "HF_OTHER": (0.01, 0.06, 0.06),
    "LEVEL_FLOOR": 0.05,
    "FLOOR": 0.02,
    "NOISE_LABEL_VIS": 0.25,
    "NOISE_LABEL_NIR": 0.25,
    "NOISE_LABEL_OTHER": 0.25,
    "GRADES": [(75, "High"), (55, "Medium"), (35, "Low"), (0, "Poor")],
}
GRADE_COLORS = {"High": "#2e8b57", "Medium": "#d9a400", "Low": "#e07b00", "Poor": "#c0392b",
                "Invalid": "#7f8c8d"}


# ════════════════════════════════════════════════════════════════════════════
# 3. Preprocessing (identical to the training notebook)
# ════════════════════════════════════════════════════════════════════════════
def sensor_segments(wl, gap_nm=50.0):
    breaks = np.where(np.diff(np.asarray(wl, dtype=float)) > gap_nm)[0]
    bounds = [0] + [int(b + 1) for b in breaks] + [len(wl)]
    return [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]


def longest_gap(mask):
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return 0
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    return int((ends - starts).max()) if len(starts) else 0


def quality_filter(X, wl, cfg):
    """Fitness filter for the model (the notebook one). Returns cleaned X, valid mask and reason."""
    X = np.asarray(X, dtype=float)
    X = np.where(X == 0, np.nan, X)
    segments = sensor_segments(wl, cfg["SENSOR_GAP_NM"])
    valid = np.ones(len(X), dtype=bool)
    reasons = np.array([""] * len(X), dtype=object)
    names = ["VIS", "NIR"] + [f"S{i}" for i in range(3, 10)]

    for k, (start, end) in enumerate(segments):
        block = X[:, start:end]
        missing = np.isnan(block).mean(axis=1)
        gaps = np.array([longest_gap(np.isnan(row)) for row in block])
        bad = ~((missing < cfg["SEG_MISSING_MAX"]) & (gaps <= cfg["SEG_GAP_MAX"]))
        for i in np.flatnonzero(bad):
            if not reasons[i]:
                reasons[i] = f"sensor {names[k]} has insufficient signal (missing data)"
        valid &= ~bad

    for i in np.flatnonzero(valid):
        flat_flags = []
        for start, end in segments:
            row = X[i, start:end]
            finite = row[np.isfinite(row)]
            if len(finite) < 2:
                flat_flags.append(True)
                continue
            lo, hi = np.percentile(finite, cfg["FLAT_PCT"])
            flat_flags.append((hi - lo) <= cfg["FLAT_RANGE_MAX"])
        flat = all(flat_flags) if cfg["FLAT_MODE"] == "all" else any(flat_flags)
        if flat:
            valid[i] = False
            reasons[i] = "flat spectrum (no spectral contrast)"

    for i in np.flatnonzero(valid):
        for start, end in segments:
            row = X[i, start:end]
            good = np.isfinite(row)
            if good.sum() >= 2 and not good.all():
                row[~good] = np.interp(np.flatnonzero(~good), np.flatnonzero(good), row[good])
                X[i, start:end] = row
    return X, valid, reasons


def resample_segments(X, wl_src, wl_dst, cfg):
    src_segments = sensor_segments(wl_src, cfg["SENSOR_GAP_NM"])
    dst_segments = sensor_segments(wl_dst, cfg["SENSOR_GAP_NM"])
    out = np.full((len(X), len(wl_dst)), np.nan, dtype=float)
    for (ss, se), (ds, de) in zip(src_segments, dst_segments):
        for i, row in enumerate(X):
            good = np.isfinite(row[ss:se])
            if good.sum() >= 2:
                out[i, ds:de] = np.interp(wl_dst[ds:de], np.asarray(wl_src)[ss:se][good],
                                          row[ss:se][good])
    return out


def sg_segmented(X, wl, order, window, poly):
    out = np.empty_like(X, dtype=float)
    for start, end in sensor_segments(wl):
        n = end - start
        win = min(window, n if n % 2 else n - 1)
        if win <= poly:
            out[:, start:end] = 0.0 if order else X[:, start:end]
        else:
            delta = float(np.median(np.diff(wl[start:end]))) if n > 1 else 1.0
            out[:, start:end] = savgol_filter(X[:, start:end], window_length=win,
                                              polyorder=poly, deriv=order, delta=delta, axis=1)
    return out


def snv_segmented(X, wl):
    out = np.empty_like(X, dtype=float)
    for start, end in sensor_segments(wl):
        block = X[:, start:end]
        mean = np.nanmean(block, axis=1, keepdims=True)
        std = np.nanstd(block, axis=1, keepdims=True)
        std[std == 0] = 1.0
        out[:, start:end] = (block - mean) / std
    return out


def features(X, wl, feat, cfg):
    sw, sp = cfg["SG_SMOOTH"]
    smoothed = sg_segmented(X, wl, 0, int(sw), int(sp))
    if feat == "RAW":
        return smoothed
    if feat == "SNV":
        return snv_segmented(smoothed, wl)
    if feat == "SNV+D1":
        dw, dp = cfg["SG_D1"]
        return sg_segmented(snv_segmented(smoothed, wl), wl, 1, int(dw), int(dp))
    raise ValueError(f"Unknown feature set: {feat!r}")


def wavelength_columns(df):
    found = []
    for col in df.columns:
        if col in META_COLS:
            continue
        match = re.fullmatch(r"\s*(\d+(?:\.\d+)?)\s*", str(col))
        if match and pd.api.types.is_numeric_dtype(df[col]):
            value = float(match.group(1))
            if 400 <= value <= 1650:
                found.append((value, col))
    found.sort(key=lambda x: x[0])
    return found


# ════════════════════════════════════════════════════════════════════════════
# 4. Spectral quality index
# ════════════════════════════════════════════════════════════════════════════
# Measures the quality of the MEASUREMENT (the spectrum), not of the material.
# It is computed independently per sensor on the raw spectrum; the final score
# is the score of the worst sensor because the model needs every sensor.
#
# The quality score and explanation are based ONLY on high-frequency noise and
# its extent; the displayed noise label is unified for each sensor.
# Spike detection, quantization and completeness penalties are not part of this
# diagnostic. It does not affect the material classifier's predictions or preprocessing.


def _quant_step(x):
    """Smallest positive step between unique finite exported values."""
    u = np.unique(np.round(x[np.isfinite(x)], 6))
    d = np.diff(u)
    return float(d.min()) if len(d) else np.nan


def _grade(score):
    for thr, name in QUALITY_CFG["GRADES"]:
        if score >= thr:
            return name
    return "Poor"


def _score(v, good, bad, floor):
    """Map a metric to [floor, 1], with a log-linear decline from good to bad."""
    if not np.isfinite(v):
        return floor
    if v <= good:
        return 1.0
    return float(np.clip(
        1.0 - np.log10(v / good) / np.log10(bad / good), floor, 1.0
    ))


def _mad_sigma(d, c=70.0):
    """Robust sigma of white noise from a 4th difference (var = C(8,4)*sigma^2 = 70*sigma^2).
    The 4th difference cancels the local curvature of slopes and absorption bands."""
    return 1.4826 * np.median(np.abs(d - np.median(d))) / np.sqrt(c)


def segment_quality(x_raw, cfg, sensor_name="OTHER"):
    """Quality of one sensor based ONLY on spectral noise (zeros = missing)."""
    q = QUALITY_CFG
    hf_cfg = q.get(f"HF_{sensor_name}", q["HF_OTHER"])
    hf_good, hf_bad, local_threshold = hf_cfg
    n = len(x_raw)
    x = np.where(x_raw == 0, np.nan, np.asarray(x_raw, dtype=float))
    miss = float(np.isnan(x).mean())
    gap = longest_gap(np.isnan(x))

    out = {
        "missing": miss, "gap": gap, "contrast": np.nan, "qstep": np.nan,
        "range_frac": np.nan, "m_HF": np.nan,
        "m_SPIKE": 0.0, "m_QUANT": np.nan,
        "f_complete": 1.0, "f_HF": 1.0, "f_SPIKE": 1.0, "f_QUANT": 1.0,
        "noise_extent": np.nan, "score": 0.0, "usable": False,
    }

    if (miss >= cfg["SEG_MISSING_MAX"]
            or gap > cfg["SEG_GAP_MAX"]
            or np.isfinite(x).sum() < 40):
        return out

    out["usable"] = True
    good = np.isfinite(x)
    xi = np.interp(np.arange(n), np.flatnonzero(good), x[good])
    # Normalize by the signal LEVEL (not by contrast): contrast includes slopes and
    # absorption bands, which hid the noise in non-flat spectra and exaggerated it in
    # flat ones. Level keeps the metric scale-invariant (e.g. reflectance 2-3 overshoot).
    level = max(float(np.median(np.abs(xi))), QUALITY_CFG["LEVEL_FLOOR"])

    # Robust high-frequency noise estimate from the 4th difference (shape-independent).
    # No spike detector is used; the MAD is robust to a small number of outliers.
    d4 = np.diff(xi, n=4)
    hf = _mad_sigma(d4) / level if len(d4) >= 10 else 0.0

    # Fraction of spectral windows with elevated local noise.
    window, stride = 15, 5
    local_hf = []
    for start in range(0, len(d4) - window + 1, stride):
        d4_window = d4[start:start + window]
        if len(d4_window) < 10:
            continue
        local_hf.append(float(_mad_sigma(d4_window) / level))

    noise_extent = (float(np.mean(np.asarray(local_hf) > local_threshold))
                    if local_hf else 0.0)
    hf_score = _score(hf, hf_good, hf_bad, q["FLOOR"])
    # Noise-only score: global HF noise, penalized when noise is widespread.
    score = hf_score * (1.0 - 0.5 * noise_extent)

    out.update(contrast=level, m_HF=hf, f_HF=hf_score,
               score=float(np.clip(score, 0.0, 1.0)),
               noise_extent=noise_extent)
    return out


def spectrum_quality(row_raw, segments, cfg, valid_for_model, reject_reason=""):
    names = ["VIS", "NIR"] + [f"S{i}" for i in range(3, 10)]
    segs = [
        segment_quality(row_raw[start:end], cfg,
                        sensor_name=names[i] if i < 2 else "OTHER")
        for i, (start, end) in enumerate(segments)
    ]

    # Moderate the harshness of the overall score while retaining a penalty
    # for the poorest sensor.
    if not valid_for_model:
        score = 0.0
    else:
        sensor_scores = [s["score"] for s in segs if s["usable"]]
        worst = min(sensor_scores) if sensor_scores else 0.0
        mean_score = float(np.mean(sensor_scores)) if sensor_scores else 0.0
        score = 100.0 * (0.65 * mean_score + 0.35 * worst)
    if not valid_for_model:
        reason = reject_reason
    else:
        # One unified noise label per sensor: do not distinguish noise that is
        # widespread from high-frequency noise that is more localized. Spikes,
        # quantization and other artifact labels remain disabled.
        noisy_sensors = []
        for i, seg in enumerate(segs):
            if not seg["usable"]:
                continue
            label_threshold = QUALITY_CFG.get(
                f"NOISE_LABEL_{names[i]}", QUALITY_CFG["NOISE_LABEL_OTHER"]
            )
            extent = seg["noise_extent"]
            has_noise = (
                seg["f_HF"] < 0.70
                or (np.isfinite(extent) and extent >= label_threshold * 0.5)
            )
            if has_noise:
                noisy_sensors.append(names[i])

        vis_noisy = "VIS" in noisy_sensors
        nir_noisy = "NIR" in noisy_sensors
        if vis_noisy and nir_noisy:
            reason = "Noise in VIS and NIR"
        elif vis_noisy:
            reason = "Noise in VIS"
        elif nir_noisy:
            reason = "Noise in NIR"
        elif noisy_sensors:
            reason = "Noise in other sensor(s)"
        elif any(seg["usable"] for seg in segs):
            reason = "No significant noise detected"
        else:
            reason = "No usable sensor spectrum for noise assessment"

    return {
        "score": score,
        "grade": _grade(score) if valid_for_model else "Invalid",
        "segments": {names[i]: segs[i] for i in range(len(segs))},
        "reason": reason,
        "usable": bool(valid_for_model),
    }


# ════════════════════════════════════════════════════════════════════════════
# 5. Analysis of a whole CSV
# ════════════════════════════════════════════════════════════════════════════
def load_bundle(path):
    bundle = joblib.load(path)
    required = {"model", "scaler", "le", "wl", "feat"}
    if not isinstance(bundle, dict) or not required.issubset(bundle):
        raise ValueError("The bundle does not contain the expected keys: model, scaler, le, wl, feat.")
    return bundle


class Analysis:
    def __init__(self, df, bundle, min_conf=0.80):
        cfg = dict(DEFAULT_CFG)
        cfg.update(bundle.get("preprocessing_config") or {})
        cfg["SG_SMOOTH"], cfg["SG_D1"] = tuple(cfg["SG_SMOOTH"]), tuple(cfg["SG_D1"])
        self.cfg, self.min_conf = cfg, min_conf
        classes = np.asarray(bundle["le"].classes_)
        wl_grid = np.asarray(bundle["wl"], dtype=float)

        cols = wavelength_columns(df)
        if len(cols) < 100:
            raise ValueError("No wavelength (nm) columns were found in the CSV.")
        wl = np.array([c[0] for c in cols])
        X = df[[c[1] for c in cols]].to_numpy(dtype=float)
        m = (wl >= cfg["WL_MIN"]) & (wl <= cfg["WL_MAX"])
        self.wl, self.X_raw = wl[m], X[:, m]
        self.segments = sensor_segments(self.wl, cfg["SENSOR_GAP_NM"])

        X_clean, valid, reasons = quality_filter(self.X_raw, self.wl, cfg)
        n = len(df)
        pred_i = np.full(n, -1)
        conf = np.full(n, np.nan)
        vidx = np.flatnonzero(valid)
        if len(vidx):
            Xg = resample_segments(X_clean[vidx], self.wl, wl_grid, cfg)
            ok = np.isfinite(Xg).all(axis=1)
            for i in vidx[~ok]:
                valid[i], reasons[i] = False, "could not be resampled onto the model grid"
            vidx, Xg = vidx[ok], Xg[ok]
            if len(vidx):
                F = features(Xg, wl_grid, bundle["feat"], cfg)
                P = bundle["model"].predict_proba(bundle["scaler"].transform(F))
                pred_i[vidx] = P.argmax(axis=1)
                conf[vidx] = P.max(axis=1)

        self.valid, self.reasons = valid, reasons
        self.material = np.array([classes[i] if i >= 0 else "" for i in pred_i], dtype=object)
        self.conf = conf
        self.quality = [spectrum_quality(self.X_raw[i], self.segments, cfg, valid[i], reasons[i])
                        for i in range(n)]
        self.labels = df["Label"].astype(str).to_numpy() if "Label" in df.columns else None

    def verdict(self, i):
        """Text shown as the result for row i."""
        if not self.valid[i]:
            return "NOT EVALUABLE", "#7f8c8d"
        if self.conf[i] < self.min_conf:
            return "UNDETERMINED", "#7f8c8d"
        return self.material[i].upper(), CLASS_COLORS.get(self.material[i], "#555555")

    def summary(self):
        return pd.DataFrame({
            "row": np.arange(1, len(self.valid) + 1),
            "material": [self.verdict(i)[0].lower() for i in range(len(self.valid))],
            "confidence": np.round(self.conf, 3),
            "quality": [round(q["score"], 1) for q in self.quality],
            "quality_grade": [q["grade"] for q in self.quality],
        })


# ════════════════════════════════════════════════════════════════════════════
# 6. Figure (same for the window and for PNG)
# ════════════════════════════════════════════════════════════════════════════
def _luminance(hex_color):
    from matplotlib.colors import to_rgb
    r, g, b = to_rgb(hex_color)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def _darken(hex_color, f=0.7):
    from matplotlib.colors import to_rgb, to_hex
    return to_hex(tuple(c * f for c in to_rgb(hex_color)))


def draw_result(fig, an, i):
    fig.clear()
    fig.patch.set_facecolor("white")
    text, color = an.verdict(i)
    q = an.quality[i]
    fg = "black" if _luminance(color) > 0.6 else "white"

    # ── Header: detected material (left) and quality (right)
    axb = fig.add_axes([0.02, 0.84, 0.60, 0.14]); axb.axis("off")
    axb.add_patch(__import__("matplotlib.patches", fromlist=["x"]).FancyBboxPatch(
        (0, 0), 1, 1, boxstyle="round,pad=0,rounding_size=0.08", fc=color, ec="none",
        transform=axb.transAxes))
    axb.text(0.04, 0.70, "DETECTED MATERIAL", color=fg, alpha=.85, fontsize=9, va="center",
             transform=axb.transAxes)
    axb.text(0.04, 0.32, text, color=fg, fontsize=30, fontweight="bold", va="center",
             transform=axb.transAxes)
    if an.valid[i]:
        axb.text(0.96, 0.32, f"confidence {an.conf[i]:.0%}", color=fg, fontsize=12, ha="right",
                 va="center", transform=axb.transAxes)
    # Invalid-spectrum diagnostics are shown only in SPECTRUM QUALITY, never here.

    qcol = GRADE_COLORS[q["grade"]]
    axq = fig.add_axes([0.64, 0.84, 0.34, 0.14]); axq.axis("off")
    axq.add_patch(__import__("matplotlib.patches", fromlist=["x"]).FancyBboxPatch(
        (0, 0), 1, 1, boxstyle="round,pad=0,rounding_size=0.08", fc="white", ec=qcol, lw=2.5,
        transform=axq.transAxes))
    axq.text(0.05, 0.84, "SPECTRUM QUALITY", fontsize=8.5, color="#555", va="center",
             transform=axq.transAxes)
    axq.text(0.25, 0.58, f"{q['score']:.0f}", fontsize=26, fontweight="bold", color=qcol,
             va="center", ha="right", transform=axq.transAxes)
    axq.text(0.27, 0.58, f"/100  ·  {q['grade']}", fontsize=13, color=qcol, va="center",
             transform=axq.transAxes)
    sc = q["segments"]
    seg_txt = "  ".join(f"{k} {v['score']*100:.0f}" for k, v in sc.items()) if q["usable"] else ""
    axq.text(0.05, 0.27, seg_txt, fontsize=8, color="#333", va="center", transform=axq.transAxes)
    axq.text(0.05, 0.10, q["reason"], fontsize=7.8, color="#555", va="center",
             transform=axq.transAxes)

    # ── Measured spectrum
    ax = fig.add_axes([0.075, 0.10, 0.905, 0.68])
    wl, y = an.wl, an.X_raw[i].astype(float).copy()
    zero = (y == 0)
    y[zero] = np.nan
    line_color = _darken(color) if (an.valid[i] and an.conf[i] >= an.min_conf) else "#34495e"
    if _luminance(line_color) > 0.55:
        line_color = _darken(line_color, 0.6)

    fin = y[np.isfinite(y)]
    lo = min(0.0, float(fin.min())) if len(fin) else 0.0
    hi = max(1.0, float(fin.max())) if len(fin) else 1.0
    pad = 0.06 * (hi - lo)
    ymin, ymax = max(lo - pad, -0.6), min(hi + pad, 2.6)
    ax.set_ylim(ymin, ymax)
    ax.set_xlim(an.cfg["WL_MIN"], an.cfg["WL_MAX"])

    # the gap between sensors stays EMPTY (no line, no interpolation)
    for k, (s, e) in enumerate(an.segments):
        ax.plot(wl[s:e], y[s:e], color=line_color, lw=1.6, solid_capstyle="round")
    names = ["VIS", "NIR"] + [f"S{k}" for k in range(3, 10)]
    for k, (s, e) in enumerate(an.segments):
        ax.text((wl[s] + wl[e - 1]) / 2, 1.0, names[k], transform=ax.get_xaxis_transform(),
                ha="center", va="top", fontsize=12, fontweight="bold", color="#7f8c8d")
    for (s0, e0), (s1, e1) in zip(an.segments[:-1], an.segments[1:]):
        a, b = wl[e0 - 1], wl[s1]
        ax.axvspan(a, b, facecolor="#ecf0f1", edgecolor="#b2bec3", hatch="//", lw=0, zorder=0)
        ax.text((a + b) / 2, 0.5, "GAP", transform=ax.get_xaxis_transform(), ha="center",
                va="center", fontsize=14, fontweight="bold", color="#7f8c8d", rotation=90)
        ax.axvline(a, color="#b2bec3", lw=.8); ax.axvline(b, color="#b2bec3", lw=.8)

    if zero.any():  # missing readings (zeros): red ticks at the bottom
        ax.plot(wl[zero], np.full(zero.sum(), ymin), "|", color="#c0392b", ms=9, mew=1.2,
                clip_on=False, label="missing reading (0)")
        ax.legend(loc="upper right", bbox_to_anchor=(1.0, 0.93), frameon=False, fontsize=8)
    ax.set_xlabel("Wavelength (nm)")
    ax.set_ylabel("Measured reflectance")
    ax.grid(alpha=.25)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    fig.text(0.075, 0.015, f"Spectrum {i + 1} of {len(an.valid)}", fontsize=8, color="#777")
    return fig


# ════════════════════════════════════════════════════════════════════════════
# 7. Desktop window (tkinter + matplotlib)
# ════════════════════════════════════════════════════════════════════════════
def run_gui(model_path, csv_path, min_conf):
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    root = tk.Tk()
    root.title("VIS-NIR material classifier")
    root.geometry("1280x760")
    state = {"an": None, "bundle": None, "i": 0}

    top = ttk.Frame(root); top.pack(fill="x", padx=8, pady=6)
    status = tk.StringVar(value="Open a spectra CSV.")

    # Confidence threshold control (0-100%). Updating it re-evaluates
    # the verdicts for the already-loaded spectra without re-running inference.
    threshold_var = tk.DoubleVar(value=min_conf * 100.0)
    ttk.Label(top, text="Confidence threshold:").pack(side="left", padx=(12, 4))
    threshold_spin = ttk.Spinbox(
        top, from_=0, to=100, increment=1, width=5,
        textvariable=threshold_var
    )
    threshold_spin.pack(side="left")
    ttk.Label(top, text="%").pack(side="left", padx=(2, 8))

    body = ttk.Frame(root); body.pack(fill="both", expand=True, padx=8, pady=(0, 8))

    tree = ttk.Treeview(body, columns=("n", "mat", "q"), show="headings", height=20)
    for c, t, w in (("n", "#", 45), ("mat", "Material", 110), ("q", "Quality", 60)):
        tree.heading(c, text=t); tree.column(c, width=w, anchor="center")
    for g, col in GRADE_COLORS.items():
        tree.tag_configure(g, foreground=col)
    sb = ttk.Scrollbar(body, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=sb.set)
    tree.pack(side="left", fill="y"); sb.pack(side="left", fill="y")

    fig = Figure(figsize=(10, 6.6), dpi=100)
    canvas = FigureCanvasTkAgg(fig, master=body)
    canvas.get_tk_widget().pack(side="left", fill="both", expand=True, padx=(8, 0))

    def refresh_threshold(*_):
        an = state["an"]
        try:
            threshold_pct = float(threshold_var.get())
        except (ValueError, tk.TclError):
            return
        if not 0 <= threshold_pct <= 100:
            return
        if an is not None:
            an.min_conf = threshold_pct / 100.0
            s = an.summary()
            for k, r in s.iterrows():
                iid = str(k)
                if tree.exists(iid):
                    tree.item(iid, values=(r["row"], r["material"], f'{r["quality"]:.0f}'),
                              tags=(r["quality_grade"],))
            show(state["i"])

    def show(i):
        an = state["an"]
        if an is None:
            return
        i = max(0, min(len(an.valid) - 1, i))
        state["i"] = i
        draw_result(fig, an, i)
        canvas.draw_idle()
        iid = str(i)
        tree.selection_set(iid); tree.see(iid)

    def load_csv(path):
        try:
            status.set("Processing…"); root.update_idletasks()
            if state["bundle"] is None:
                state["bundle"] = load_bundle(model_path)
            df = pd.read_csv(path)
            an = Analysis(df, state["bundle"], min_conf)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Error", str(exc)); status.set("Error."); return
        state["an"] = an
        tree.delete(*tree.get_children())
        s = an.summary()
        for k, r in s.iterrows():
            tree.insert("", "end", iid=str(k), values=(r["row"], r["material"], f"{r['quality']:.0f}"),
                        tags=(r["quality_grade"],))
        status.set(f"{Path(path).name} · {len(s)} spectra")
        show(0)

    def open_csv():
        p = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if p:
            load_csv(p)

    def export():
        if state["an"] is None:
            return
        p = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if p:
            state["an"].summary().to_csv(p, index=False, encoding="utf-8-sig")

    def save_png():
        if state["an"] is None:
            return
        p = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG", "*.png")])
        if p:
            fig.savefig(p, dpi=200)

    ttk.Button(top, text="Open CSV…", command=open_csv).pack(side="left")
    ttk.Button(top, text="◀", width=3, command=lambda: show(state["i"] - 1)).pack(side="left", padx=(12, 2))
    ttk.Button(top, text="▶", width=3, command=lambda: show(state["i"] + 1)).pack(side="left")
    ttk.Button(top, text="Save figure…", command=save_png).pack(side="left", padx=(12, 0))
    ttk.Button(top, text="Export results…", command=export).pack(side="left", padx=6)
    ttk.Label(top, textvariable=status).pack(side="left", padx=12)
    threshold_var.trace_add("write", refresh_threshold)

    tree.bind("<<TreeviewSelect>>", lambda e: (tree.selection() and
              int(tree.selection()[0]) != state["i"] and show(int(tree.selection()[0]))))
    root.bind("<Left>", lambda e: show(state["i"] - 1))
    root.bind("<Right>", lambda e: show(state["i"] + 1))

    if csv_path:
        root.after(100, lambda: load_csv(csv_path))
    root.mainloop()


# ════════════════════════════════════════════════════════════════════════════
# 8. Command line
# ════════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="VIS-NIR construction-material classifier")
    ap.add_argument("csv", nargs="?", help="spectra CSV (if omitted, it is chosen in the window)")
    ap.add_argument("--model", default=str(Path(__file__).with_name("classifier.joblib")))
    ap.add_argument("--min-conf", type=float, default=0.80,
                    help="below this softmax output the result is shown as UNDETERMINED")
    ap.add_argument("--row", type=int, default=1, help="row (1-based) for --png")
    ap.add_argument("--png", help="save the --row figure to this PNG and exit (no window)")
    ap.add_argument("--out-csv", help="save the summary table and exit (no window)")
    a = ap.parse_args()

    if a.png or a.out_csv:
        if not a.csv:
            ap.error("--png/--out-csv require a CSV")
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib.figure import Figure
        an = Analysis(pd.read_csv(a.csv), load_bundle(a.model), a.min_conf)
        if a.out_csv:
            an.summary().to_csv(a.out_csv, index=False, encoding="utf-8-sig")
            print(f"Summary saved to {a.out_csv}")
        if a.png:
            fig = Figure(figsize=(11, 6.6), dpi=130)
            draw_result(fig, an, a.row - 1)
            fig.savefig(a.png)
            print(f"Figure saved to {a.png}")
        return
    run_gui(a.model, a.csv, a.min_conf)


if __name__ == "__main__":
    main()
