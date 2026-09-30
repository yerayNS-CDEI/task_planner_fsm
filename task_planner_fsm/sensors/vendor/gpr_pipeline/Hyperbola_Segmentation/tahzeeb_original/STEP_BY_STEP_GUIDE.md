# GPR post-processing — step-by-step guide

Turns raw model detections (`detections.csv`) into consolidated objects, physical
depths, per-element recovery quantities, and the viewer/material-passport exports.
Verified end-to-end on the real 167-detection file (→ 121 consolidated objects, 4 elements).

```
postprocessing/
├── config.yaml                # calibration + thresholds (EDIT THIS)
├── detections.csv             # input (your model output)
├── gpr_pp_utils.py            # shared IO / parsing helpers
├── 01_consolidate.py          # step 1
├── 02_hyperbola_fit.py        # step 2
├── 03_quantify.py             # step 3
├── 04_export.py               # step 4
├── run_postprocessing.py      # runs 1→4 in one call
└── sample_results/            # example outputs from the real detections.csv
```

## 0. Prerequisites

```bash
pip install numpy pandas pyyaml scipy --break-system-packages
# (scipy only needed for per-object velocity fitting from masks)
```

Put your model's `detections.csv` next to the scripts. Required columns:
`image, confidence, x1, y1, x2, y2` (extra columns like `class`, `inference_ms` are ignored).
Tile filenames must carry the grid index `..._r{row}_c{col}.png` so tiles can be stitched.

## 1. Configure (the only file you edit)

Open `config.yaml` and set the **calibration** block from your GP8800 survey — this is
what makes depths and quantities real rather than placeholder:

| key | meaning |
|---|---|
| `px_per_m` | traces per metre (from the survey-wheel trace interval) |
| `time_window_ns` | two-way time represented by the full B-scan height |
| `image_height_px` | full B-scan height in px |
| `epsilon_r` | host permittivity (concrete ≈ 4–8); sets velocity `v = c/√εr` |
| `tiling.tile_size`, `overlap_x` | must match the tiler that produced the detections |

The defaults run, but `time_window_ns` is a **placeholder** — depths scale with it.

## 2. Run everything (recommended)

```bash
python run_postprocessing.py --config config.yaml
```

Creates `results/<timestamp>/` with all five outputs and prints site totals:

```
recoverable steel : 960 kg
indicative value  : EUR 385
CO2 avoided       : 1.54 t
objects (consolidated): 121  from 167 raw
```

## 3. Or run the steps one by one

**Step 1 — Consolidation** (`01_consolidate.py`)
Drops over-wide "envelope" boxes (`> envelope_width_factor × median width`), stitches
tile-local boxes into one per-element pixel frame using the grid index and stride, then
class-agnostic NMS (`nms_iou`) removes tile-overlap duplicates.
```bash
python 01_consolidate.py --config config.yaml --out results/run/consolidated.csv
```
→ `consolidated.csv` (`element, gx1, gy1, gx2, gy2, confidence, width_px`)

**Step 2 — Physical parameters** (`02_hyperbola_fit.py`)
Computes each apex (box top + `apex_frac`), converts to two-way time and cover depth via
the calibration. With masks (`masks_file`) it fits `t(x)=√(t₀²+(2(x−x₀)/v)²)` per object to
recover velocity and εr; without masks it uses `epsilon_r` from the config.
```bash
python 02_hyperbola_fit.py --config config.yaml --in results/run/consolidated.csv --out results/run/objects.csv
```
→ `objects.csv` (`element, x_m, apex_px, twt_ns, v_m_ns, eps_r, depth_cm, confidence`)

**Step 3 — Quantification** (`03_quantify.py`)
Per element: object count, bar pitch (mean ± sd from plausible inter-bar gaps),
reinforcement-layer count (1 or 2 via a depth-gap split — this is what flagged Wall_1001 as
two-layer), cover depth, material interpretation, recoverable steel mass, value and CO₂.
```bash
python 03_quantify.py --config config.yaml --in results/run/objects.csv --out results/run/element_summary.csv
```
→ `element_summary.csv`

**Step 4 — Export** (`04_export.py`)
```bash
python 04_export.py --config config.yaml --objects results/run/objects.csv \
    --summary results/run/element_summary.csv --outdir results/run/
```
→ `gpr_data.json` (drop into the digital-twin viewer / material-map platforms) and
`material_passport.csv` (one audit row per element).

## 4. What you get (from the real detections.csv)

`material_passport.csv`:

| element | type | interpretation | n | layers | pitch cm | steel kg |
|---|---|---|---|---|---|---|
| floor001 | floor_slab | RCC slab — steel mesh | 29 | 1 | 22.3 | 274 |
| Wall_1001 | wall | reinforced concrete — rebar (two layers) | 41 | 2 | 29.6 | 323 |
| Wall_2001 | wall | reinforced concrete — rebar | 6 | 1 | 18.0 | 157 |
| Wall_3001 | wall | reinforced concrete — rebar | 45 | 1 | 23.9 | 207 |

## Notes & honest caveats

- **Consolidation count is configurable.** With `overlap_x=0.35`/`nms_iou=0.45` the real file
  reduces 167→121; the manuscript's 140 used different overlap/IoU. Tune to your tiler and
  report the settings.
- **Depth/value are calibration-dependent.** They are only meaningful once `time_window_ns`,
  `px_per_m` and `epsilon_r` are set from the GP8800.
- **Velocity fitting needs masks.** Box-only input uses the config εr (constant velocity);
  supply instance masks to fit per-object velocity and εr (the `fit_velocity_from_mask` hook).
- **Report recall with precision.** Consolidation removes duplicates, not misses; missed rebar
  is the costly error in recovery planning.
- **Layer split is a heuristic** (largest depth gap ≥ `layer_gap_cm`); validate on site.
- **Element type** is inferred from the scan name; rename scans or extend the mapping to label
  partitions/columns explicitly.
- Code standards: Python 3.10+, type hints, docstrings, seed 42, YAML config, timestamped
  `results/`. Designed for Claude Code execution; this guide is the runbook.
