"""Client-facing B-scan figures.

Colours follow Tahzeeb's GPRiQ / MIP-R overlay standard (v2 bscan_figure.py):
green mask = AI hyperbola, yellow = hyperbola fit, red = AI detection not retained.
All drawing is done at the native resolution of the rendered B-scan.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from matplotlib.path import Path as MplPath
from PIL import Image, ImageDraw, ImageFont

DET = (0, 200, 83)        # #00c853
FIT = (255, 212, 0)       # #ffd400
EXCL = (230, 40, 40)      # #e62828
MASK_ALPHA = 0.30
APEX = (255, 255, 255)

_FONT_FILES = {
    False: ["C:/Windows/Fonts/arial.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"],
    True: ["C:/Windows/Fonts/arialbd.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
           "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"],
}


def _font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    for path in _FONT_FILES[bold]:
        try:
            return ImageFont.truetype(path, size=size)
        except Exception:
            pass
    return ImageFont.load_default(size=size)


def load_global_masks(masks_json: Path | None, crop_manifest: Path, detections: list[dict]) -> list[dict[str, Any]]:
    """Mask R-CNN masks of one gain, moved from tile pixels to full B-scan pixels.

    Tile -> full image: x = src_x0 + x_tile, y = (src_y0 + y_tile) / vscale
    (src_x0/src_y0 from the v9 crop manifest, working space = source space when vscale = 1).
    """
    if masks_json is None or not Path(masks_json).exists():
        return []
    records = json.loads(Path(masks_json).read_text(encoding="utf-8"))
    man = pd.read_csv(crop_manifest).set_index("tile_file")
    scores = {(d["image"], float(d["x1"]), float(d["y1"]), float(d["x2"]), float(d["y2"])): float(d["confidence"])
              for d in detections}
    out = []
    for m in records:
        tile = m.get("image", "")
        if tile not in man.index or not m.get("polygon"):
            continue
        t = man.loc[tile]
        sx, sy, vs = float(t["src_x0"]), float(t["src_y0"]), float(t.get("vscale", 1.0) or 1.0)
        poly = np.asarray(m["polygon"], dtype=float)
        poly = np.column_stack([sx + poly[:, 0], (sy + poly[:, 1]) / vs])
        box = (sx + float(m["x1"]), (sy + float(m["y1"])) / vs, sx + float(m["x2"]), (sy + float(m["y2"])) / vs)
        key = (tile, float(m["x1"]), float(m["y1"]), float(m["x2"]), float(m["y2"]))
        out.append({"polygon": poly, "box": box, "score": scores.get(key)})
    return out


def _base(path: Path) -> Image.Image:
    return Image.open(path).convert("L").convert("RGB")


def _blend_masks(img: Image.Image, masks: list[dict], colours: list[tuple[int, int, int]]) -> Image.Image:
    """Fill every mask once (overlaps from neighbouring tiles do not darken) and outline each one."""
    layer = Image.new("RGB", img.size, (0, 0, 0))
    alpha = Image.new("L", img.size, 0)
    dl, da = ImageDraw.Draw(layer), ImageDraw.Draw(alpha)
    for m, c in zip(masks, colours):
        pts = [tuple(p) for p in m["polygon"]]
        if len(pts) >= 3:
            dl.polygon(pts, fill=c)
            da.polygon(pts, fill=int(255 * MASK_ALPHA))
    out = Image.composite(layer, img, alpha)
    d = ImageDraw.Draw(out)
    lw = max(2, img.width // 900)
    for m, c in zip(masks, colours):
        pts = [tuple(p) for p in m["polygon"]]
        if len(pts) >= 3:
            d.line(pts + [pts[0]], fill=c, width=lw, joint="curve")
    return out


def _tag(d: ImageDraw.ImageDraw, xy: tuple[float, float], text: str, fill, font, size: tuple[int, int],
         placed: list[tuple[float, float, float, float]]) -> None:
    """Label box at xy; moved down in steps until it does not cover an already placed label."""
    pad = max(3, font.size // 5)
    l, t, r, b = d.textbbox((0, 0), text, font=font)
    w, h = r - l + 2 * pad, b - t + 2 * pad
    x = min(max(0, xy[0]), size[0] - w)
    y = min(max(0, xy[1]), size[1] - h)
    for _ in range(20):
        if not any(x < px1 and px0 < x + w and y < py1 and py0 < y + h for px0, py0, px1, py1 in placed):
            break
        y = min(y + h + 2, size[1] - h)
    placed.append((x, y, x + w, y + h))
    d.rectangle([x, y, x + w, y + h], fill=fill)
    d.text((x + pad - l, y + pad - t), text, fill=(0, 0, 0), font=font)


def _with_header(img: Image.Image, title: str, subtitle: str, legend: list[tuple[str, tuple, str]]) -> Image.Image:
    """White band above the radargram with the title and legend: no text is written on the data."""
    s = img.width / 1200
    ft, fs, fl = _font(int(26 * s), True), _font(int(17 * s)), _font(int(17 * s))
    row = int(34 * s)
    band = int(18 * s) + row + int(26 * s) + row + int(10 * s)
    out = Image.new("RGB", (img.width, img.height + band), (255, 255, 255))
    out.paste(img, (0, band))
    d = ImageDraw.Draw(out)
    x0, y = int(16 * s), int(12 * s)
    d.text((x0, y), title, fill=(31, 42, 51), font=ft)
    y += row
    d.text((x0, y), subtitle, fill=(90, 103, 115), font=fs)
    y += int(30 * s)
    x = x0
    for label, colour, kind in legend:
        sw = int(28 * s)
        if kind == "patch":
            d.rectangle([x, y + 4 * s, x + sw, y + 20 * s], fill=tuple(int(255 - MASK_ALPHA * (255 - c)) for c in colour),
                        outline=colour, width=max(2, int(2 * s)))
        elif kind == "line":
            d.line([x, y + 12 * s, x + sw, y + 12 * s], fill=colour, width=max(3, int(4 * s)))
        else:
            r = 7 * s
            d.ellipse([x + sw / 2 - r, y + 12 * s - r, x + sw / 2 + r, y + 12 * s + r], fill=colour, outline=(0, 0, 0),
                      width=max(1, int(2 * s)))
        d.text((x + sw + 8 * s, y), label, fill=(31, 42, 51), font=fl)
        x += sw + 8 * s + d.textlength(label, font=fl) + 30 * s
    return out


def draw_segmentation_masks(base_png: Path, masks: list[dict], output_path: Path, title: str) -> None:
    """Processed B-scan with every mask and score returned by the Mask R-CNN (raw model output)."""
    img = _base(base_png)
    img = _blend_masks(img, masks, [DET] * len(masks))
    d = ImageDraw.Draw(img)
    font = _font(max(14, int(16 * img.width / 1200)), True)
    placed: list[tuple[float, float, float, float]] = []
    for m in sorted(masks, key=lambda m: -(m["score"] or 0.0)):
        if m["score"] is not None:
            x1, y1 = m["box"][0], m["box"][1]
            _tag(d, (x1, y1 - font.size * 1.6), f"Hyperbola {m['score']:.2f}", DET, font, img.size, placed)
    _with_header(
        img, title,
        f"Mask R-CNN segmentation (raw model output, all tiles): {len(masks)} masks",
        [("AI hyperbola mask + score", DET, "patch")],
    ).save(output_path)


def _supports(m: dict, ax: float, ay: float, radius: float) -> bool:
    """True when a retained apex lies on the mask (within `radius` px of its outline)."""
    return MplPath(m["polygon"]).contains_point((ax, ay), radius=radius) or \
        MplPath(m["polygon"]).contains_point((ax, ay), radius=-radius)


def draw_final_detections(
    base_png: Path,
    masks: list[dict],
    accepted: pd.DataFrame,
    scan_distance_m: float,
    time_window_ns: float,
    output_path: Path,
    title: str,
) -> None:
    """Processed B-scan with masks, fitted hyperbolas, apexes and one tag per retained detection.

    A mask is green when a retained hyperbola apex lies in its box, red otherwise
    (removed by consolidation, fitting, dedup or consensus).
    """
    img = _base(base_png)
    w, h = img.size
    apexes = []
    for r in accepted.itertuples(index=False):
        ax = float(r.x_m) / scan_distance_m * (w - 1)
        ay = float(r.twt_ns) / time_window_ns * (h - 1)
        apexes.append((r, ax, ay))

    radius = 0.01 * w
    colours = [DET if any(_supports(m, ax, ay, radius) for _, ax, ay in apexes) else EXCL for m in masks]
    img = _blend_masks(img, masks, colours)
    d = ImageDraw.Draw(img)
    s = w / 1200
    font = _font(max(14, int(17 * s)), True)

    for r, ax, ay in apexes:
        x0, t0 = float(r.x_m), float(r.twt_ns)
        v, hw = max(float(r.v_m_ns), 1e-6), max(float(r.arc_hw_m), 0.02)
        xs = np.linspace(max(0.0, x0 - hw), min(scan_distance_m, x0 + hw), 200)
        ts = np.sqrt(max(t0, 1e-6) ** 2 + (2.0 * (xs - x0) / v) ** 2)
        pts = [(xm / scan_distance_m * (w - 1), tn / time_window_ns * (h - 1)) for xm, tn in zip(xs, ts)]
        pts = [p for p in pts if 0 <= p[1] < h]
        if len(pts) >= 2:
            d.line(pts, fill=(0, 0, 0), width=max(5, int(7 * s)), joint="curve")
            d.line(pts, fill=FIT, width=max(3, int(4 * s)), joint="curve")
        rad = max(6, int(8 * s))
        d.ellipse([ax - rad, ay - rad, ax + rad, ay + rad], fill=APEX, outline=(0, 0, 0), width=max(2, int(3 * s)))

    placed: list[tuple[float, float, float, float]] = []
    for r, ax, ay in apexes:
        depth = getattr(r, "depth_cm_assumed", None)
        conf = getattr(r, "confidence_mean", None)
        text = f"{r.detection_id}  x {float(r.x_m):.2f} m"
        if depth is not None and np.isfinite(depth):
            text += f"  ·  {float(depth):.1f} cm"
        if conf is not None and np.isfinite(conf):
            text += f"  ·  {float(conf):.2f}"
        _tag(d, (ax + 14 * s, ay - font.size * 2.2), text, FIT, font, img.size, placed)

    n_excl = colours.count(EXCL)
    legend = [("AI hyperbola (retained)", DET, "patch"), ("hyperbola fit", FIT, "line"), ("apex", APEX, "dot")]
    if n_excl:
        legend.insert(1, ("AI detection not retained", EXCL, "patch"))
    _with_header(
        img, title,
        f"{len(apexes)} hyperbola(s) retained  ·  {len(masks)} AI masks  ·  "
        f"depth assumes epsilon_r (see JSON)  ·  scan {scan_distance_m:.2f} m, {time_window_ns:.2f} ns",
        legend,
    ).save(output_path)
