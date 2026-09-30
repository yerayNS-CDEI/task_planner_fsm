#!/usr/bin/env python3
"""
gpr_tiler_v9.py
===============

Native-scale GPR B-scan tiler for detection/segmentation training
(YOLO / Mask R-CNN / RT-DETR). v8 supersedes v7.

What changed from v7 (and why)
------------------------------
1. ROBUST VERTICAL BAND DETECTION (the main fix).
   v7 used row-std with a fixed absolute floor (5.0). On images whose deep
   zone carries faint smooth clutter (migration smiles, bottom striping),
   every row clears that floor, so the "largest contiguous block" swallows the
   whole image and you get rows of dead tiles (this was happening on the WIDE
   `floor001`-type images, not just the tall ones).

   v8 uses a TEXTURE-AWARE energy = row_std * mean|vertical-gradient|. Smooth
   clutter has low gradient, so it no longer holds the band open. The keep
   threshold is calibrated against a robust estimate of the image's OWN
   background (median + k*MAD of the deepest 25% of rows, which are reliably
   non-content). This auto-adapts per image:
       floor001 (1200 px): full-height -> top ~36%   (dead bottom removed)
       28tub   (8000 px) : top 19.6%   -> top ~13%   (lands on the hyperbola)
   The proven v7 contiguous-block + gap-bridge logic is kept on top.

   NOTE on MAD: a *whole-band* MAD detector was unreliable in earlier work.
   Here MAD is used ONLY to estimate the background floor from known-deep rows
   (a narrow, safe use), then the contiguous-block logic does the rest.

2. HIGHER DEFAULT X-OVERLAP (0.35).
   Once the band is tight, the grid is tiny, so a hyperbola straddling a column
   boundary could land whole in NO tile. More horizontal overlap guarantees any
   feature appears intact in at least one window, at the cost of a few tiles.

3. OPTIONAL ASPECT CORRECTION (--vscale, OFF by default).
   Some exports over-sample the time/depth axis (e.g. 8000 px deep), which
   smears hyperbolas vertically and makes square tiles cut through them. If you
   confirm from the GP8800 export that px-per-ns (vertical) >> px-per-trace
   (horizontal), set --vscale to the factor that makes pixels physically
   square. This is the ONLY sanctioned resize: it RESTORES true hyperbola
   geometry rather than shrinking it. Default 1.0 = native, no resize.

Saved tiles are real B-scan pixels only: no letterbox, no padding, no aspect
routing. Native scale is preserved (unless you deliberately set --vscale).

Python 3.10+, stdlib + numpy + Pillow.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from PIL import Image

IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
SEED_SALT = "gpr_tiler_v7"  # unchanged so splits stay identical to v7
DEFAULT_SPLIT = (0.70, 0.15, 0.15)

GENERATED_NAME_RE = re.compile(
    r"(_native_r\d+_c\d+$|_t\d{2}_\d{2}$|_tx\d{3}_ty\d{3}$|_lb\d{3}$|"
    r"_lw\d{3}$|_pres\d{3}$|_tile\d{3,}$|_crop_r\d+_c\d+$)", re.IGNORECASE)
GENERATED_DIR_NAMES = {
    "tiles_v5", "tiles_v6", "tiles_v7", "tiles_v8", "tiles_v9", "gpr_native_tiles_v1",
    "rect_crops_final", "images_tiled", "images_tiled_training",
    "images_tiled2", "images_tiled-claudev7", "images_tiled_claudev7",
    "low_detail_review",
}

OUTPUT_SUFFIX_DEFAULT = "_tiles_v9"


@dataclass
class CropRecord:
    source_file: str
    split: str
    tile_file: str
    source_w: int          # ORIGINAL width
    source_h: int          # ORIGINAL height (pre-vscale)
    source_mode: str
    vscale: float          # vertical resample applied before tiling (1.0 = native)
    work_h: int            # height actually tiled (= round(source_h * vscale))
    band_x0: int           # band/src coords are in WORKING (post-vscale) space
    band_y0: int
    band_x1: int
    band_y1: int
    src_x0: int
    src_y0: int
    src_x1: int
    src_y1: int
    row: int
    col: int
    tile_w: int
    tile_h: int
    tile_std: float
    is_near_empty: bool
    is_likely_negative: bool
    status: str


@dataclass
class QCRecord:
    source_file: str
    split: str
    source_w: int
    source_h: int
    source_mode: str
    vscale: float
    work_h: int
    band_w: int
    band_h: int
    band_pct_of_work_h: float
    grid_cols: int
    grid_rows: int
    n_windows: int
    n_kept: int
    n_negative_flagged: int
    status: str


def normalise_gray(img: Image.Image) -> tuple[np.ndarray, str]:
    """Robust uint8 grayscale, incl. 16-bit/float via 1-99 percentile clip."""
    mode = img.mode
    if mode == "L":
        return np.asarray(img, dtype=np.uint8), mode
    if mode in {"RGB", "RGBA", "P", "CMYK", "YCbCr", "HSV", "1"}:
        return np.asarray(img.convert("L"), dtype=np.uint8), mode
    arr = np.asarray(img).astype(np.float32)
    if arr.ndim == 3:
        arr = arr.mean(axis=2)
    finite = np.isfinite(arr)
    if not finite.any():
        return np.zeros(arr.shape, dtype=np.uint8), mode
    valid = arr[finite]
    lo, hi = float(np.percentile(valid, 1)), float(np.percentile(valid, 99))
    if hi <= lo:
        lo, hi = float(valid.min()), float(valid.max())
    if hi <= lo:
        hi = lo + 1.0
    arr = (np.clip(arr, lo, hi) - lo) / (hi - lo) * 255.0
    return np.clip(arr, 0, 255).astype(np.uint8), mode


def _smooth(v: np.ndarray, span: int) -> np.ndarray:
    k = max(1, int(span))
    return np.convolve(v.astype(np.float32), np.ones(k) / k, mode="same")


def resample_vertical(arr: np.ndarray, vscale: float) -> np.ndarray:
    """Resize height by vscale, keep width. Used only for aspect correction.
    Works on 2-D (gray) or 3-D (RGB) uint8 arrays. vscale==1.0 is a no-op."""
    if abs(vscale - 1.0) < 1e-6:
        return arr
    if arr.ndim == 2:
        h, w = arr.shape
        new_h = max(1, int(round(h * vscale)))
        im = Image.fromarray(arr, mode="L").resize((w, new_h), Image.BILINEAR)
        return np.asarray(im, dtype=np.uint8)
    h, w = arr.shape[:2]
    new_h = max(1, int(round(h * vscale)))
    im = Image.fromarray(arr).resize((w, new_h), Image.BILINEAR)
    return np.asarray(im, dtype=np.uint8)


def detect_band_y(gray: np.ndarray, k_peak: float, k_bg: float,
                  floor: float, margin: int, k_exit: float = 0.015,
                  min_band_frac: float = 0.0, crop: int = 640,
                  gap_frac: float = 0.10) -> tuple[int, int]:
    """
    Vertical signal band via TEXTURE-AWARE, background-calibrated energy with a
    HYSTERESIS bottom edge (v9) so hyperbola limbs are never clipped.

    energy(row) = row_std * mean|vertical-gradient|, smoothed.  (v8 metric kept:
    multiplying by gradient suppresses smooth deep clutter while keeping real
    reflectors/hyperbolas, which carry high gradient.)

    Two thresholds:
      thr_hi = max(floor, k_peak*peak, bg_floor)   -> finds the band ONSET (top)
      thr_lo = max(bg_floor, k_exit*peak)          -> bottom walks down while the
        energy stays above this. thr_lo is NEVER below bg_floor, so the descent
        follows the (textured) hyperbola limbs but CANNOT reopen the smooth dead
        bottom that v8 was built to remove.
      bg_floor = median + k_bg*1.4826*MAD over the deepest 25% of rows.

    A minimum-height floor (>= one crop, or min_band_frac*h) guarantees v7-like
    regular tiles and a real bottom for display, never producing thin slivers.
    Native scale preserved -- crop bounds only, no resize.
    """
    h, _ = gray.shape
    g = gray.astype(np.float32)
    rstd = g.std(axis=1)
    rgrad = np.abs(np.diff(g, axis=0)).mean(axis=1)
    rgrad = np.append(rgrad, rgrad[-1] if rgrad.size else 0.0)
    energy = _smooth(rstd * rgrad, max(1, h // 200))

    bg = energy[int(0.75 * h):]
    if bg.size:
        med = float(np.median(bg))
        mad = float(np.median(np.abs(bg - med))) + 1e-6
        bg_floor = med + k_bg * 1.4826 * mad
    else:
        bg_floor = 0.0
    peak = float(energy.max())
    thr_hi = max(floor, k_peak * peak, bg_floor)

    mask = energy > thr_hi
    if not mask.any():
        return 0, h

    # bridge gaps shorter than gap_tol so one block isn't split by minor dips
    gap_tol = max(1, int(round(h * gap_frac)))
    filled = mask.copy()
    run_start: int | None = None
    for i, on in enumerate(mask):
        if not on and run_start is None:
            run_start = i
        elif on and run_start is not None:
            if i - run_start <= gap_tol:
                filled[run_start:i] = True
            run_start = None
    # pick the longest contiguous True run in the gap-bridged mask
    best_len = best_s = best_e = 0
    s: int | None = None
    for i, on in enumerate(filled):
        if on and s is None:
            s = i
        elif not on and s is not None:
            if i - s > best_len:
                best_len, best_s, best_e = i - s, s, i
            s = None
    if s is not None and len(filled) - s > best_len:
        best_len, best_s, best_e = len(filled) - s, s, len(filled)

    # HYSTERESIS bottom: descend while textured limbs persist above thr_lo.
    thr_lo = max(bg_floor, k_exit * peak)
    y_bot = best_e
    while y_bot < h and energy[y_bot] > thr_lo:
        y_bot += 1

    y0 = max(0, best_s - margin)
    y1 = min(h, y_bot + margin)

    # MIN-HEIGHT FLOOR: never a sliver; default floor is one crop.
    min_h = min(h, max(crop, int(min_band_frac * h)))
    if (y1 - y0) < min_h:
        y1 = min(h, y0 + min_h)
    return y0, y1


def trim_constant_columns(gray: np.ndarray, x_floor: float,
                          margin: int) -> tuple[int, int]:
    """Trim only near-CONSTANT edge columns (true gray borders), not quiet signal."""
    _, w = gray.shape
    cs = _smooth(gray.std(axis=0), max(1, w // 200))
    cols = np.where(cs > x_floor)[0]
    if cols.size == 0:
        return 0, w
    x0 = max(0, int(cols.min()) - margin)
    x1 = min(w, int(cols.max()) + 1 + margin)
    return x0, x1


def plan_starts(length: int, crop: int, overlap: float) -> list[int]:
    if length <= crop:
        return [0]
    step = max(1, int(round(crop * (1.0 - overlap))))
    starts = list(range(0, length - crop + 1, step))
    if starts[-1] != length - crop:
        starts.append(length - crop)
    return sorted(set(starts))


def is_generated(path: Path) -> bool:
    if any(p.lower() in GENERATED_DIR_NAMES for p in path.parts):
        return True
    return bool(GENERATED_NAME_RE.search(path.stem))


def collect_images(source: Path, recursive: bool) -> list[Path]:
    if source.is_file():
        return [source.resolve()]
    it = source.rglob("*") if recursive else source.glob("*")
    seen: set[Path] = set()
    out: list[Path] = []
    for p in sorted(it):
        if p.suffix.lower() in IMG_EXTS and not is_generated(p):
            rp = p.resolve()
            if rp not in seen:
                seen.add(rp)
                out.append(rp)
    return out


def immediate_dataset_folders(root: Path, recursive: bool) -> list[Path]:
    folders: list[Path] = []
    for child in sorted(root.iterdir()):
        if not child.is_dir():
            continue
        if is_generated(child):
            continue
        if child.name.lower().endswith(OUTPUT_SUFFIX_DEFAULT.lower()):
            continue
        if collect_images(child, recursive):
            folders.append(child.resolve())
    return folders


def output_for_dataset_folder(dataset_folder: Path, suffix: str) -> Path:
    return dataset_folder.parent / f"{dataset_folder.name}{suffix}"


def write_csvs(out: Path, all_recs: list[CropRecord], qcs: list[QCRecord]) -> None:
    if not all_recs:
        return
    from datetime import datetime
    st = datetime.now().strftime("%Y%m%d_%H%M%S")
    mp = out / f"crop_manifest_{st}.csv"
    with mp.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(asdict(all_recs[0]).keys()))
        wr.writeheader()
        for r in all_recs:
            wr.writerow(asdict(r))
    qp = out / f"crop_qc_{st}.csv"
    with qp.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(asdict(qcs[0]).keys()))
        wr.writeheader()
        for q in qcs:
            wr.writerow(asdict(q))
    print(f"Manifest -> {mp}\nQC       -> {qp}")


def split_for(stem: str, ratios: tuple[float, float, float]) -> str:
    tr, va, te = ratios
    tot = tr + va + te
    tr, va = tr / tot, va / tot
    v = int(hashlib.sha1((SEED_SALT + stem).encode()).hexdigest()[:8], 16) / 0xFFFFFFFF
    return "train" if v < tr else "val" if v < tr + va else "test"


def process(path: Path, out: Path, crop: int, ox: float, oy: float,
            k_peak: float, k_bg: float, floor: float, margin: int,
            trim_x: bool, x_floor: float, vscale: float,
            empty_std: float, neg_std: float, drop_empty: bool,
            split: str, split_dirs: bool, save_rgb: bool, dry: bool,
            k_exit: float = 0.015, min_band_frac: float = 0.0
            ) -> tuple[list[CropRecord], QCRecord]:
    pil = Image.open(path)
    gray, mode = normalise_gray(pil)
    color = np.asarray(pil.convert("RGB")) if save_rgb else gray
    orig_h, orig_w = gray.shape

    # Optional one-time aspect correction (vertical resample). Off by default.
    gray = resample_vertical(gray, vscale)
    if save_rgb:
        color = resample_vertical(color, vscale)
    else:
        color = gray
    h, w = gray.shape

    y0, y1 = detect_band_y(gray, k_peak, k_bg, floor, margin,
                           k_exit=k_exit, min_band_frac=min_band_frac, crop=crop)
    x0, x1 = (0, w)
    if trim_x:
        x0, x1 = trim_constant_columns(gray[y0:y1, :], x_floor, margin)
    band_g = gray[y0:y1, x0:x1]
    band_c = color[y0:y1, x0:x1]
    bh, bw = band_g.shape

    xs, ys = plan_starts(bw, crop, ox), plan_starts(bh, crop, oy)
    dest = (out / split) if split_dirs else out
    if not dry:
        dest.mkdir(parents=True, exist_ok=True)

    recs: list[CropRecord] = []
    kept = neg = 0
    for r, sy in enumerate(ys):
        for c, sx in enumerate(xs):
            ex, ey = min(bw, sx + crop), min(bh, sy + crop)
            cg = band_g[sy:ey, sx:ex]
            std = float(cg.std()) if cg.size else 0.0
            near_empty = std < empty_std
            is_neg = std < neg_std
            keep = not (drop_empty and near_empty)
            name = f"{path.stem}_native_r{r:03d}_c{c:03d}.png"
            status = "kept" if keep else "dropped_empty"
            if keep:
                kept += 1
                neg += int(is_neg)
                if not dry:
                    cc = band_c[sy:ey, sx:ex]
                    Image.fromarray(cc).save(dest / name)
            recs.append(CropRecord(
                source_file=str(path), split=split,
                tile_file=name if keep else "",
                source_w=orig_w, source_h=orig_h, source_mode=mode,
                vscale=round(vscale, 4), work_h=h,
                band_x0=x0, band_y0=y0, band_x1=x1, band_y1=y1,
                src_x0=x0 + sx, src_y0=y0 + sy, src_x1=x0 + ex, src_y1=y0 + ey,
                row=r, col=c, tile_w=ex - sx, tile_h=ey - sy,
                tile_std=round(std, 2),
                is_near_empty=bool(near_empty), is_likely_negative=bool(is_neg),
                status=status))
    qc = QCRecord(
        source_file=str(path), split=split, source_w=orig_w, source_h=orig_h,
        source_mode=mode, vscale=round(vscale, 4), work_h=h,
        band_w=bw, band_h=bh, band_pct_of_work_h=round(100 * bh / max(1, h), 1),
        grid_cols=len(xs), grid_rows=len(ys), n_windows=len(xs) * len(ys),
        n_kept=kept, n_negative_flagged=neg,
        status="OK" if kept else "WARN_no_kept")
    return recs, qc


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", required=True, type=Path,
                    help="Root folder, dataset subfolder, or single image.")
    ap.add_argument("--output", required=False, type=Path, default=None,
                    help="Output folder for single-folder mode. Ignored in --batch-subfolders.")
    ap.add_argument("--batch-subfolders", action="store_true",
                    help="Process every immediate subfolder under --source -> sibling '<folder>_tiles_v8'.")
    ap.add_argument("--output-suffix", default=OUTPUT_SUFFIX_DEFAULT,
                    help=f"Suffix for --batch-subfolders outputs (default: {OUTPUT_SUFFIX_DEFAULT}).")
    ap.add_argument("--recursive", action="store_true")
    ap.add_argument("--crop-size", type=int, default=640)
    ap.add_argument("--overlap", type=float, default=None,
                    help="Sets both axes; overridden by --overlap-x/-y if given.")
    ap.add_argument("--overlap-x", type=float, default=0.35,
                    help="Horizontal overlap (default 0.35: keeps hyperbolas whole in >=1 tile).")
    ap.add_argument("--overlap-y", type=float, default=0.20)
    # --- band detection (v8 texture-aware, background-calibrated) ---
    ap.add_argument("--energy-frac", type=float, default=0.05,
                    help="k_peak: keep rows with energy >= this fraction of peak.")
    ap.add_argument("--bg-k", type=float, default=4.0,
                    help="Background floor = median + bg_k*1.4826*MAD of deepest 25%% of rows.")
    ap.add_argument("--energy-floor", type=float, default=1.0,
                    help="Hard minimum energy floor (safety; adaptive bg floor usually dominates).")
    ap.add_argument("--band-margin", type=int, default=32)
    ap.add_argument("--energy-exit", type=float, default=0.015,
                    help="k_exit: hysteresis bottom descends while energy > "
                         "max(bg_floor, k_exit*peak). Lower keeps more limb.")
    ap.add_argument("--min-band-frac", type=float, default=0.0,
                    help="Min band height as a fraction of source height. "
                         "Floor is always >= one crop, so 0.0 = one full crop tall.")
    ap.add_argument("--trim-x", action="store_true",
                    help="Trim near-constant edge columns (gray borders) only.")
    ap.add_argument("--x-floor", type=float, default=3.0)
    # --- optional aspect correction ---
    ap.add_argument("--vscale", type=float, default=1.0,
                    help="Vertical resample factor applied ONCE before tiling (1.0=native, OFF). "
                         "Set <1.0 to de-stretch an over-sampled depth axis so pixels are "
                         "physically square; this restores hyperbola geometry.")
    # --- tile flagging ---
    ap.add_argument("--empty-std", type=float, default=3.0,
                    help="Tiles below this std are flagged near_empty.")
    ap.add_argument("--negative-std", type=float, default=12.0,
                    help="Tiles below this std are flagged likely_negative (kept).")
    ap.add_argument("--drop-empty", action="store_true",
                    help="Actually drop near_empty tiles (default: keep + flag).")
    ap.add_argument("--split", type=float, nargs=3, default=list(DEFAULT_SPLIT),
                    metavar=("TRAIN", "VAL", "TEST"))
    ap.add_argument("--split-dirs", action="store_true",
                    help="Write into train/ val/ test/ subfolders.")
    ap.add_argument("--rgb", action="store_true", help="Save 3-channel (default: grayscale).")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    if a.overlap is not None:
        a.overlap_x = a.overlap_y = a.overlap
    if a.crop_size < 64:
        ap.error("--crop-size must be >= 64")
    if a.vscale <= 0:
        ap.error("--vscale must be > 0")

    jobs: list[tuple[Path, Path]] = []
    if a.batch_subfolders:
        if not a.source.is_dir():
            ap.error("--batch-subfolders requires --source to be a directory root.")
        folders = immediate_dataset_folders(a.source, a.recursive)
        if not folders:
            print("No dataset subfolders with source images found.", file=sys.stderr)
            return 1
        jobs = [(folder, output_for_dataset_folder(folder, a.output_suffix)) for folder in folders]
        print(f"Batch mode: {len(jobs)} dataset folder(s) found under {a.source}")
        print(f"Output rule: <dataset_folder>{a.output_suffix}\n")
    else:
        if a.output is None:
            ap.error("--output is required unless --batch-subfolders is used.")
        jobs = [(a.source.resolve(), a.output.resolve())]

    total_all_tiles = total_all_windows = failed_jobs = 0

    for job_idx, (src_root, out_root) in enumerate(jobs, start=1):
        GENERATED_DIR_NAMES.add(out_root.name.lower())
        images = collect_images(src_root, a.recursive)
        if not images:
            print(f"[{job_idx}/{len(jobs)}] SKIP {src_root} -> no source images after filtering")
            continue
        if not a.dry_run:
            out_root.mkdir(parents=True, exist_ok=True)

        print("\n" + "=" * 100)
        print(f"[{job_idx}/{len(jobs)}] Source -> {src_root}")
        print(f"[{job_idx}/{len(jobs)}] Output -> {out_root}")
        print(("DRY RUN" if a.dry_run else "RUN") +
              f" | crop={a.crop_size} overlap=({a.overlap_x},{a.overlap_y}) "
              f"vscale={a.vscale} grayscale={not a.rgb} drop_empty={a.drop_empty}")
        print(f"{len(images)} image(s)\n")
        print(f"{'image':<46}{'split':<6}{'src':<13}{'band':<13}{'band%':<7}{'grid':<7}{'kept':<6}{'neg'}")
        print("-" * 100)

        all_recs: list[CropRecord] = []
        qcs: list[QCRecord] = []
        try:
            for img in images:
                sp = split_for(img.stem, tuple(a.split))
                recs, qc = process(img, out_root, a.crop_size, a.overlap_x, a.overlap_y,
                                   a.energy_frac, a.bg_k, a.energy_floor, a.band_margin,
                                   a.trim_x, a.x_floor, a.vscale,
                                   a.empty_std, a.negative_std, a.drop_empty,
                                   sp, a.split_dirs, a.rgb, a.dry_run,
                                   a.energy_exit, a.min_band_frac)
                all_recs += recs
                qcs.append(qc)
                print(f"{img.name[:45]:<46}{sp:<6}{f'{qc.source_w}x{qc.source_h}':<13}"
                      f"{f'{qc.band_w}x{qc.band_h}':<13}{f'{qc.band_pct_of_work_h}%':<7}"
                      f"{f'{qc.grid_cols}x{qc.grid_rows}':<7}{qc.n_kept:<6}{qc.n_negative_flagged}")
        except Exception as e:
            failed_jobs += 1
            print(f"ERROR while processing {src_root}: {e}", file=sys.stderr)
            continue

        print("-" * 100)
        for s in ("train", "val", "test"):
            n = sum(q.n_kept for q in qcs if q.split == s)
            m = sum(1 for q in qcs if q.split == s)
            print(f"  {s:<5}: {m:3d} images -> {n:4d} tiles")
        job_tiles = sum(q.n_kept for q in qcs)
        job_windows = sum(q.n_windows for q in qcs)
        total_all_tiles += job_tiles
        total_all_windows += job_windows
        print(f"TOTAL for folder: {job_tiles} tiles / {job_windows} windows")

        if not a.dry_run and all_recs:
            write_csvs(out_root, all_recs, qcs)

    print("\n" + "=" * 100)
    print(f"ALL DONE: {total_all_tiles} tiles / {total_all_windows} windows")
    if failed_jobs:
        print(f"WARNING: {failed_jobs} folder(s) failed; check messages above.")
    return 0 if failed_jobs == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
