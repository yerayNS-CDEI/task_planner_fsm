"""Step 1 - Geometry-aware consolidation.

Removes spurious wide "envelope" boxes, stitches tile-local detections into a
single per-element pixel frame, and de-duplicates overlapping detections with
class-agnostic non-maximum suppression (NMS).

Input : detections CSV (image,confidence,x1,y1,x2,y2,...)
Output: consolidated.csv  (element, gx1, gy1, gx2, gy2, confidence, width_px)

Usage:
    python 01_consolidate.py --config config.yaml --out results/<ts>/consolidated.csv
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from gpr_pp_utils import load_config, load_detections, make_results_dir, load_manifest


def _stride(tile: float, overlap: float) -> float:
    return tile * (1.0 - overlap)


def stitch(df: pd.DataFrame, tile: float, ox: float, oy: float) -> pd.DataFrame:
    """Map tile-local boxes to a global per-element frame using the grid index."""
    sx, sy = _stride(tile, ox), _stride(tile, oy)
    out = df.copy()
    out["gx1"] = df["col"] * sx + df["x1"]
    out["gx2"] = df["col"] * sx + df["x2"]
    out["gy1"] = df["row"] * sy + df["y1"]
    out["gy2"] = df["row"] * sy + df["y2"]
    out["width_px"] = out["gx2"] - out["gx1"]
    return out


def stitch_manifest(df: pd.DataFrame, manifest: dict) -> pd.DataFrame:
    """Exact stitch using the v9 manifest src_x0/src_y0 (handles clamped last tile)."""
    out = df.copy()
    sx0 = df["image"].map(lambda n: manifest.get(n, {}).get("src_x0", 0.0))
    sy0 = df["image"].map(lambda n: manifest.get(n, {}).get("src_y0", 0.0))
    out["gx1"] = sx0 + df["x1"]; out["gx2"] = sx0 + df["x2"]
    out["gy1"] = sy0 + df["y1"]; out["gy2"] = sy0 + df["y2"]
    out["width_px"] = out["gx2"] - out["gx1"]
    return out


def drop_envelopes(df: pd.DataFrame, factor: float) -> pd.DataFrame:
    """Drop boxes wider than ``factor`` x the per-element median width."""
    keep = []
    for _, g in df.groupby("element"):
        med = float(np.median(g["width_px"]))
        keep.append(g[g["width_px"] <= factor * med])
    return pd.concat(keep, ignore_index=True) if keep else df


def _inter(a: np.ndarray, b: np.ndarray) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    return max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    inter = _inter(a, b)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def _iomin(a: np.ndarray, b: np.ndarray) -> float:
    """Intersection over the SMALLER box area.

    Catches a small detection mostly contained inside a larger one - i.e. two
    wrongly-overlapping detections on a SINGLE physical hyperbola whose IoU is low
    only because their areas differ. Plain IoU misses these; IoMin does not.
    """
    inter = _inter(a, b)
    amin = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
    return inter / amin if amin > 0 else 0.0


def drop_by_fill_rate(df: pd.DataFrame, tile_px: float, max_fill: float) -> pd.DataFrame:
    """Drop boxes whose area exceeds ``max_fill`` x one tile area (envelope blobs).

    Mask-area analogue of the MS-code ``small_fill_rate`` gate, adapted to bbox area
    since per-pixel masks are not exported. NO-OP unless ``max_fill`` is set in config.
    """
    if not max_fill:
        return df
    tile_area = float(tile_px) ** 2
    area = (df["gx2"] - df["gx1"]) * (df["gy2"] - df["gy1"])
    return df[area <= max_fill * tile_area].copy()


def nms(df: pd.DataFrame, iou_thr: float, iomin_thr: float | None = None) -> pd.DataFrame:
    """Class-agnostic NMS within each element (merges tile-overlap duplicates).

    Suppresses a lower-confidence box j (relative to a kept box i) when EITHER
    IoU(i, j) > iou_thr OR - if ``iomin_thr`` is set - IoMin(i, j) > iomin_thr
    (the containment test for wrongly-overlapping same-object detections).
    """
    kept_rows = []
    for _, g in df.groupby("element"):
        boxes = g[["gx1", "gy1", "gx2", "gy2"]].to_numpy()
        scores = g["confidence"].to_numpy()
        order = scores.argsort()[::-1]
        suppressed = np.zeros(len(order), dtype=bool)
        keep_idx = []
        for i in range(len(order)):
            if suppressed[i]:
                continue
            keep_idx.append(order[i])
            for j in range(i + 1, len(order)):
                if suppressed[j]:
                    continue
                bi, bj = boxes[order[i]], boxes[order[j]]
                if _iou(bi, bj) > iou_thr or (
                        iomin_thr is not None and _iomin(bi, bj) > iomin_thr):
                    suppressed[j] = True
        kept_rows.append(g.iloc[keep_idx])
    return pd.concat(kept_rows, ignore_index=True)


def load_polygons(masks_path: str | None) -> dict:
    """Build {(image, round(x1,1), round(y1,1)): {poly, area}} from masks.json.

    ``poly`` is the full-mask contour in TILE pixels (same frame as x1/y1); ``area``
    is the mask pixel area. Empty dict if the file or polygons are unavailable.
    """
    if not masks_path:
        return {}
    try:
        data = json.load(open(masks_path))
    except Exception:
        return {}
    out = {}
    for m in data:
        k = (m["image"], round(float(m["x1"]), 1), round(float(m["y1"]), 1))
        out[k] = {"poly": m.get("polygon", []), "area": float(m.get("area_px", 0) or 0)}
    return out


def _poly_iomin(poly_a, area_a, poly_b, area_b) -> float:
    """Intersection-over-min-area of two polygons (rasterised). 0 if not computable.

    Real-mask version of the bbox _iomin: catches curved-hyperbola duplicates whose
    masks overlap even when their boxes do not (and vice-versa). Areas are the stored
    mask pixel areas; the intersection is rasterised only in the boxes' overlap window.
    """
    if len(poly_a) < 3 or len(poly_b) < 3 or area_a <= 0 or area_b <= 0:
        return 0.0
    try:
        import cv2
    except Exception:
        return 0.0
    A = np.asarray(poly_a, dtype=np.int32)
    B = np.asarray(poly_b, dtype=np.int32)
    x0 = max(A[:, 0].min(), B[:, 0].min()); y0 = max(A[:, 1].min(), B[:, 1].min())
    x1 = min(A[:, 0].max(), B[:, 0].max()); y1 = min(A[:, 1].max(), B[:, 1].max())
    w, h = int(x1 - x0 + 1), int(y1 - y0 + 1)
    if w <= 0 or h <= 0 or w * h > 8_000_000:
        return 0.0
    ca = np.zeros((h, w), np.uint8); cb = np.zeros((h, w), np.uint8)
    cv2.fillPoly(ca, [A - [x0, y0]], 1)
    cv2.fillPoly(cb, [B - [x0, y0]], 1)
    inter = int(np.logical_and(ca, cb).sum())
    return inter / min(area_a, area_b)


def suppress_by_mask_overlap(df: pd.DataFrame, polylookup: dict, manifest: dict,
                             iomin_thr: float) -> pd.DataFrame:
    """Per element, drop the lower-confidence of any pair whose masks overlap > thr.

    Polygons are shifted to the global frame by the per-tile src_x0 so detections from
    the same object in overlapping tiles are compared in one frame.
    """
    keep_parts = []
    for _, g in df.groupby("element"):
        g = g.sort_values("confidence", ascending=False)
        rows = list(g.itertuples(index=True))
        polys, areas = [], []
        for r in rows:
            img = getattr(r, "image")
            key = (img, round(float(getattr(r, "x1")), 1), round(float(getattr(r, "y1")), 1))
            info = polylookup.get(key, {})
            sx0 = manifest.get(img, {}).get("src_x0", 0.0) if manifest else 0.0
            polys.append([[p[0] + sx0, p[1]] for p in info.get("poly", [])])
            areas.append(info.get("area", 0.0))
        n = len(rows)
        keep = [True] * n
        for i in range(n):
            if not keep[i]:
                continue
            for j in range(i + 1, n):
                if keep[j] and _poly_iomin(polys[i], areas[i], polys[j], areas[j]) > iomin_thr:
                    keep[j] = False
        kept_idx = [rows[k].Index for k in range(n) if keep[k]]
        keep_parts.append(g.loc[kept_idx])
    return pd.concat(keep_parts) if keep_parts else df


def consolidate(cfg: dict) -> pd.DataFrame:
    cc = cfg["consolidate"]
    df = load_detections(cfg["io"]["detections_csv"])
    df = df[df["confidence"] >= cc["min_confidence"]].copy()
    man_path = cfg["io"].get("crop_manifest")
    manifest = load_manifest(man_path) if man_path else {}
    if manifest:
        df = stitch_manifest(df, manifest)
    else:
        t = cfg["tiling"]
        df = stitch(df, t["tile_size"], t["overlap_x"], t["overlap_y"])
    df = drop_envelopes(df, cc["envelope_width_factor"])
    df = drop_by_fill_rate(df, cfg["tiling"]["tile_size"], cc.get("fill_rate_max"))
    df = nms(df, cc["nms_iou"], cc.get("nms_iomin"))
    # Real mask-overlap suppression (uses full polygons from masks.json). Opt-in:
    # supersedes the bbox-IoMin stopgap when masks are available. NO-OP if unset.
    mo = cc.get("mask_overlap_iomin")
    if mo:
        polylookup = load_polygons(cfg["io"].get("masks_file"))
        if polylookup:
            df = suppress_by_mask_overlap(df, polylookup, manifest, float(mo))
    cols = ["element", "source", "gx1", "gy1", "gx2", "gy2", "confidence", "width_px"]
    return df[cols].sort_values(["element", "gx1"]).reset_index(drop=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    raw = load_detections(cfg["io"]["detections_csv"])
    cons = consolidate(cfg)
    out = Path(args.out) if args.out else make_results_dir(cfg["io"]["results_dir"]) / "consolidated.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    cons.to_csv(out, index=False)
    print(f"[step1] raw detections      : {len(raw)}")
    for el, g in cons.groupby("element"):
        n_raw = int((raw['element'] == el).sum())
        print(f"[step1]   {el:42s} {n_raw:3d} -> {len(g):3d}")
    print(f"[step1] consolidated total  : {len(cons)}")
    print(f"[step1] written -> {out}")


if __name__ == "__main__":
    main()
