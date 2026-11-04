from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


SENSOR_PREFIX = re.compile(r"^(VIS|NIR|SWIR)_", re.IGNORECASE)
REFLECTANCE_CSV = "reflectance.csv"


def _as_float_name(name: Any) -> float | None:
    """Wavelength from a column name: "325,2998" (LENZ .xls) or "VIS_325.3" (robot session)."""
    text = SENSOR_PREFIX.sub("", str(name).strip())
    try:
        return float(text.replace(",", "."))
    except (TypeError, ValueError):
        return None


def resolve_input(path: str | Path) -> tuple[Path, str]:
    """Return the spectra file and the name used for its output folder.

    A robot acquisition session is a folder holding `reflectance.csv`; it can be given
    as the folder or as the file. Its output folder is named after the session, since
    every session's file has the same name.
    """
    path = Path(path).resolve()
    if path.is_dir():
        path = path / REFLECTANCE_CSV
        if not path.exists():
            raise FileNotFoundError(f"No {REFLECTANCE_CSV} in session folder {path.parent}")
    if not path.exists():
        raise FileNotFoundError(path)
    name = path.parent.name if path.name.lower() == REFLECTANCE_CSV else path.stem
    return path, name


def _read_input(path: Path) -> tuple[pd.DataFrame, str]:
    """Read a normal CSV or the text-based .xls export produced by the LENZ software.

    The real .xls export is not a binary Excel workbook; it is a tab-separated
    text file with comma decimal separators. We detect that format from the file
    content rather than relying only on the extension.
    """
    first = path.read_bytes()[:8192]
    text_probe = first.decode("utf-8", errors="ignore")
    if "\t" in text_probe and ("Measure Type" in text_probe or path.suffix.lower() == ".xls"):
        return pd.read_csv(path, sep="\t", dtype=str, keep_default_na=False), "spectrometer_tsv_xls"
    return pd.read_csv(path, dtype=str, keep_default_na=False), "csv"


def prepare_input_for_benjamin(input_path: str | Path) -> tuple[pd.DataFrame, np.ndarray, dict]:
    """Convert a sensor export into the numeric table Benjamin's classifier.py expects.

    Only file-format adaptation is done here:
      - keep rows whose `Measure Type` is `Sample` (reference/dark rows are calibration records);
      - wavelength column names -> plain dotted numbers ("325,2998" / "VIS_325.3" -> "325.2998" / "325.3");
      - comma decimals -> numeric values.

    Wavelength-range cropping, resampling to the model grid, quality filtering and
    preprocessing remain the responsibility of Benjamin's original code (v2).

    Returns the prepared table, the 1-based data-row number of each kept row in the
    original file (for traceability) and a small input summary.
    """
    input_path = Path(input_path)
    raw, detected_format = _read_input(input_path)
    n_input_rows = int(len(raw))
    source_rows = np.arange(1, n_input_rows + 1)

    excluded_by_measure_type: dict[str, int] = {}
    if "Measure Type" in raw.columns:
        measure = raw["Measure Type"].astype(str).str.strip()
        sample_mask = measure.str.casefold().eq("sample").to_numpy()
        excluded_by_measure_type = {
            str(k): int(v) for k, v in measure[~sample_mask].value_counts().items()
        }
        raw = raw.loc[sample_mask]
        source_rows = source_rows[sample_mask]
    raw = raw.reset_index(drop=True)

    spectral: dict[str, pd.Series] = {}
    metadata_cols: list[str] = []
    for c in raw.columns:
        wl = _as_float_name(c)
        if wl is None:
            metadata_cols.append(c)
            continue
        s = raw[c].astype(str).str.strip().str.replace(",", ".", regex=False)
        spectral[f"{wl:.10g}"] = pd.to_numeric(s, errors="coerce")
    if not spectral:
        raise ValueError("No wavelength columns were found in the input file.")

    prepared = pd.concat([raw[metadata_cols], pd.DataFrame(spectral)], axis=1)

    summary = {
        "input_file": input_path.name,
        "detected_format": detected_format,
        "input_rows": n_input_rows,
        "rows_sent_to_classifier": int(len(prepared)),
        "excluded_by_measure_type": excluded_by_measure_type,
        "source_band_count": len(spectral),
    }
    return prepared, source_rows, summary
