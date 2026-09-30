"""Shared helpers for the GPR post-processing pipeline.

Conventions
-----------
* A "source element" is one scanned wall/slab; its B-scan is split into tiles
  named ``<element>_..._native_r{row}_c{col}.png``.
* All coordinates after stitching live in a single per-element pixel frame
  whose origin is the top-left of tile (r0, c0).
"""
from __future__ import annotations

import datetime as _dt
import re
from pathlib import Path
from typing import Any

import pandas as pd
import yaml

_GRID_RE = re.compile(r"_r(\d+)_c(\d+)\b")


def load_config(path: str | Path) -> dict[str, Any]:
    """Load the YAML pipeline configuration."""
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def parse_source_and_grid(image_name: str) -> tuple[str, int, int]:
    """Return ``(source_element, row, col)`` parsed from a tile filename.

    Falls back to ``(stem, 0, 0)`` when no ``_r{row}_c{col}`` pattern is found.
    """
    stem = Path(image_name).stem
    m = _GRID_RE.search(stem)
    if not m:
        return stem, 0, 0
    row, col = int(m.group(1)), int(m.group(2))
    source = stem[: m.start()]
    return source, row, col


def element_label(source: str) -> str:
    """Human-readable element name from a source id.

    Strips the acquisition/processing timestamp suffix so that e.g.
    ``Wall_1001_20260219_..._native`` -> ``Wall_1001`` and
    ``floor001_20260219_..._native`` -> ``floor001``.
    """
    parts = re.split(r"_\d{8}", source, maxsplit=1)
    return parts[0] if parts and parts[0] else source


def load_detections(csv_path: str | Path) -> pd.DataFrame:
    """Load the model detections CSV and attach source/grid columns."""
    df = pd.read_csv(csv_path)
    req = {"image", "confidence", "x1", "y1", "x2", "y2"}
    missing = req - set(df.columns)
    if missing:
        raise ValueError(f"detections CSV missing columns: {sorted(missing)}")
    parsed = df["image"].map(parse_source_and_grid)
    df["source"] = [p[0] for p in parsed]
    df["row"] = [p[1] for p in parsed]
    df["col"] = [p[2] for p in parsed]
    df["element"] = df["source"].map(element_label)
    for c in ("x1", "y1", "x2", "y2", "confidence"):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def make_results_dir(base: str | Path) -> Path:
    """Create and return ``base/<YYYYmmdd_HHMMSS>/``."""
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(base) / stamp
    out.mkdir(parents=True, exist_ok=True)
    return out


def load_manifest(path: str | Path) -> dict:
    """Load the v9 crop manifest -> {tile_file: {src_x0,src_y0,source_w,source_h,vscale}}.

    Enables EXACT stitching (uses the real per-tile source offset, including the
    clamped last tile) instead of assuming a fixed stride.
    """
    df = pd.read_csv(path)
    out = {}
    for _, r in df.iterrows():
        out[str(r["tile_file"])] = {
            "src_x0": float(r["src_x0"]), "src_y0": float(r["src_y0"]),
            "source_w": float(r["source_w"]), "source_h": float(r["source_h"]),
            "vscale": float(r.get("vscale", 1.0)),
            "element": element_label(parse_source_and_grid(str(r["tile_file"]))[0]),
        }
    return out


def element_source_w(manifest: dict) -> dict:
    """element -> source_w (px), from the manifest."""
    m = {}
    for v in manifest.values():
        m[v["element"]] = v["source_w"]
    return m
