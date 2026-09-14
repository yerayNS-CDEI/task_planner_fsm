from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import joblib
import numpy as np
import pandas as pd

SENSOR_GAP_NM = 50.0
NEAR_EXACT_NM = 0.20


def _segments(wl: np.ndarray, gap_nm: float = SENSOR_GAP_NM) -> list[tuple[int, int]]:
    breaks = np.where(np.diff(wl) > gap_nm)[0]
    bounds = [0] + [int(i + 1) for i in breaks] + [len(wl)]
    return [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]


def _as_float_name(name: Any) -> float | None:
    try:
        return float(str(name).strip().replace(",", "."))
    except (TypeError, ValueError):
        return None


def _read_input(path: Path) -> tuple[pd.DataFrame, str]:
    """Read a normal CSV or the text-based .xls export produced by the HSI software.

    The provided .xls example is not a binary Excel workbook; it is a tab-separated
    text file with comma decimal separators. We detect that format from the file
    content rather than relying only on the extension.
    """
    first = path.read_bytes()[:8192]
    text_probe = first.decode("utf-8", errors="ignore")
    if "\t" in text_probe and ("Measure Type" in text_probe or path.suffix.lower() == ".xls"):
        return pd.read_csv(path, sep="\t", dtype=str, keep_default_na=False), "spectrometer_tsv_xls"
    return pd.read_csv(path, dtype=str, keep_default_na=False), "csv"


def _extract_spectral_columns(df: pd.DataFrame) -> tuple[list[str], np.ndarray]:
    cols: list[str] = []
    wl: list[float] = []
    for c in df.columns:
        v = _as_float_name(c)
        if v is not None:
            cols.append(c)
            wl.append(v)
    if not cols:
        raise ValueError("No wavelength columns were found in the input file.")
    order = np.argsort(np.asarray(wl, dtype=float))
    return [cols[int(i)] for i in order], np.asarray(wl, dtype=float)[order]


def _numeric_matrix(df: pd.DataFrame, cols: list[str]) -> np.ndarray:
    out = np.empty((len(df), len(cols)), dtype=float)
    for j, c in enumerate(cols):
        s = df[c].astype(str).str.strip().str.replace(",", ".", regex=False)
        out[:, j] = pd.to_numeric(s, errors="coerce").to_numpy(dtype=float)
    return out


def _resample_preserving_missingness(
    X: np.ndarray,
    source_wl: np.ndarray,
    target_wl: np.ndarray,
    near_exact_nm: float = NEAR_EXACT_NM,
) -> np.ndarray:
    """Map spectra to the model wavelength grid without inventing across missing data.

    Interpolation is performed independently inside each sensor segment (VIS/SWIR).
    A target band is interpolated only when both adjacent source bands are finite.
    Therefore missing source values remain visible to Benjamin's own quality filter.
    """
    out = np.full((len(X), len(target_wl)), np.nan, dtype=float)
    source_segments = _segments(source_wl)
    target_segments = _segments(target_wl)
    if len(source_segments) != len(target_segments):
        raise ValueError(
            f"Sensor-segment mismatch: input has {len(source_segments)} segments, "
            f"model has {len(target_segments)}."
        )

    for (ss, se), (ts, te) in zip(source_segments, target_segments):
        sw = source_wl[ss:se]
        block = X[:, ss:se]
        for out_j, t in enumerate(target_wl[ts:te], start=ts):
            nearest = int(np.argmin(np.abs(sw - t)))
            if abs(float(sw[nearest] - t)) <= near_exact_nm:
                out[:, out_j] = block[:, nearest]
                continue

            pos = int(np.searchsorted(sw, t))
            if pos <= 0 or pos >= len(sw):
                # No extrapolation. A tiny endpoint wavelength-calibration mismatch
                # is already handled by NEAR_EXACT_NM above.
                continue

            left, right = pos - 1, pos
            lv = block[:, left]
            rv = block[:, right]
            valid = np.isfinite(lv) & np.isfinite(rv)
            frac = float((t - sw[left]) / (sw[right] - sw[left]))
            out[valid, out_j] = lv[valid] + frac * (rv[valid] - lv[valid])
    return out


def _format_wavelength(w: float) -> str:
    # Stable column names that float() can parse and Benjamin's code can match.
    return f"{float(w):.10g}"


def prepare_input_for_benjamin(
    input_path: str | Path,
    model_path: str | Path,
) -> tuple[pd.DataFrame, dict]:
    """Convert a real sensor export into the exact 450-band grid stored in the model.

    This adapter performs only file-format/wavelength-grid adaptation. It deliberately
    does NOT reproduce Benjamin's spectral cleaning. Zeros, missing values and physical
    quality checks remain the responsibility of the preserved original predict.py.
    """
    input_path = Path(input_path)
    raw, detected_format = _read_input(input_path)
    n_input_rows = int(len(raw))

    excluded_by_measure_type: dict[str, int] = {}
    if "Measure Type" in raw.columns:
        measure = raw["Measure Type"].astype(str).str.strip()
        counts = measure.value_counts(dropna=False).to_dict()
        sample_mask = measure.str.casefold().eq("sample")
        excluded_by_measure_type = {
            str(k): int(v) for k, v in counts.items() if str(k).strip().casefold() != "sample"
        }
        work = raw.loc[sample_mask].copy().reset_index(drop=True)
    else:
        work = raw.copy().reset_index(drop=True)

    source_cols, source_wl = _extract_spectral_columns(work)
    X = _numeric_matrix(work, source_cols)

    bundle = joblib.load(model_path)
    if not isinstance(bundle, dict) or bundle.get("wl") is None:
        raise ValueError("The classifier bundle does not contain the training wavelength array 'wl'.")
    target_wl = np.asarray(bundle["wl"], dtype=float)

    X_target = _resample_preserving_missingness(X, source_wl, target_wl)

    metadata_candidates = ["Measure Type", "Date", "Time", "Counter", "Label", "Class"]
    metadata_cols = [c for c in metadata_candidates if c in work.columns]
    meta = work[metadata_cols].copy()
    spectral = pd.DataFrame(
        X_target,
        columns=[_format_wavelength(w) for w in target_wl],
        index=meta.index,
    )
    prepared = pd.concat([meta, spectral], axis=1)

    src_segments = _segments(source_wl)
    tgt_segments = _segments(target_wl)
    diagnostics = {
        "input_file": input_path.name,
        "detected_format": detected_format,
        "input_rows": n_input_rows,
        "rows_sent_to_classifier": int(len(prepared)),
        "excluded_non_sample_rows": int(n_input_rows - len(prepared)),
        "excluded_by_measure_type": excluded_by_measure_type,
        "source_band_count": int(len(source_wl)),
        "model_band_count": int(len(target_wl)),
        "source_wavelength_range_nm": [float(source_wl.min()), float(source_wl.max())],
        "model_wavelength_range_nm": [float(target_wl.min()), float(target_wl.max())],
        "source_segments_nm": [
            [float(source_wl[s]), float(source_wl[e - 1])] for s, e in src_segments
        ],
        "model_segments_nm": [
            [float(target_wl[s]), float(target_wl[e - 1])] for s, e in tgt_segments
        ],
        "missing_values_after_grid_mapping": int(np.isnan(X_target).sum()),
        "rows_with_any_missing_after_grid_mapping": int(np.isnan(X_target).any(axis=1).sum()),
        "adapter_note": (
            "Only input-format and wavelength-grid adaptation is done here. "
            "Benjamin's original quality filter remains authoritative for zeros, NaNs, "
            "reflectance limits, spikes and noise."
        ),
    }
    return prepared, diagnostics
