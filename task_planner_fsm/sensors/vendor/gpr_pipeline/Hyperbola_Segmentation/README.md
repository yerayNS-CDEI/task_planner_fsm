# GPR — Hyperbola segmentation (Tahzeeb)

Detects GPR hyperbolae in a Proceq GP8800 B-scan and returns their position, depth and
fit. It does **not** infer wall material from GPR.

Normally run through `GPR/codes/run_gpr.py`, which runs this branch and the line branch on the
same scan. It can also be run on its own (see *Run*).

## Contents

| Path | What it is |
|---|---|
| `tahzeeb_original/` | Tahzeeb's supplied code, **unchanged**: tiler `gpr_tiler_v9.py`, Mask R-CNN inference `inference_gpr.py`, post-processing `01_consolidate → 02_hyperbola_fit → 02b_dedup → 03_quantify → 04_export`, exact `config.yaml` (2026-09-10). |
| `models/best.pt` | Supplied Mask R-CNN ResNet101 weights (~241 MB). |
| `gpr_integration/` | DISCOVER integration layer (preprocessing, adapters, consensus, figures, outputs). |
| `run_gpr_pipeline.py` | CLI for this branch only. |
| `check_setup.py` | Checks that the code, weights and configs are in place. |
| `SOURCE_MANIFEST.txt` | Origin of every supplied file. |

Shared preprocessing comes from `GPR/codes/GPRTools/` (not duplicated here). Dependencies: `GPR/requirements.txt`.

## Pipeline

```text
GP8800 .sgy + .csv
→ GPRTools: time_zero → background_removal → gain (40 dB)
→ full grayscale B-scan, no axes
→ Tahzeeb gpr_tiler_v9.py: 640×640 tiles, overlap-x 0.35
→ Mask R-CNN ResNet101 (one model load for all tiles)
→ Tahzeeb 01_consolidate → 02_hyperbola_fit → 02b_dedup (per gain)
→ cross-gain consensus (demo: one gain, min_gain_support 1)
→ robot result (gpr_result.json) + Tahzeeb/MIP export (gpr_data.json) + figures
```

- **Time window.** The GP8800 sidecar reports 16 ns; Proceq confirmed −4 ns, so the
  effective window is **12 ns**. GPRTools applies the correction once; the integration
  checks for 12 ns.
- **Depth** uses **εr = 6.0** (concrete), exported as an assumption, not a measurement.
- **Gains.** The code supports several renderings of the same acquisition
  (`preprocessing.gains_db`, consensus tolerance ±6 cm / ±4 cm). The demo uses a
  single 40 dB gain. `gain_support` is a robustness indicator, not statistical replication.
- Tahzeeb's `config.yaml` is kept unchanged; the integration writes a runtime copy that
  only overrides paths and scan calibration (scan length, image size, time window, εr).

## Outputs

With `run_gpr.py` they go to `GPR/test_output/<scan>/hyperbola/`, next to the
scan's `.sgy` + `.csv`:

| File | Content |
|---|---|
| `gpr_bscan_raw.png` | B-scan **as acquired** (no time-zero, background removal or gain). Same GPRTools renderer and settings as the processed image. |
| `gpr_bscan_processed.png` | B-scan **after the GPRTools filters**, nothing drawn on it: exactly the image the model is tiled from. |
| `gpr_segmentation_masks.png` | Processed B-scan with **every Mask R-CNN mask** and its score (raw model output). Neighbouring tiles overlap by 35 %, so one hyperbola can show as 2–3 overlapping masks. |
| `gpr_final_detections.png` | Masks (green = supports a retained hyperbola, red = not retained), hyperbola fits (yellow), apexes (white) and one tag per detection (`id · x · depth · confidence`). |
| `gpr_data.json` | Tahzeeb's JSON (MIP schema), from his `03_quantify` + `04_export` after consensus. |
| `gpr_result.json` | **Internal robot/POKEYE JSON**: the `dict` returned by `run_gpr_pipeline()`. |

- The **global JSON for Tecnalia** (both GPR branches + HSI) is built by `FULL PIPELINE/Global JSON/`.
- Colours follow Tahzeeb's GPRiQ / MIP-R standard (v2 `bscan_figure.py`). That script is
  not used: it needs `hyperbolas.py`, `wall_layers.py` and a newer result structure that
  were not supplied.
- Masks are mapped from tile to B-scan pixels with the v9 crop manifest:
  `x = src_x0 + x_tile`, `y = (src_y0 + y_tile) / vscale`.
- Tiles, masks and fitting tables are written to work folders inside the output folder
  and deleted after a successful run (`outputs.keep_workspace: true` keeps them).
- Each file can be switched off in `gpr_integration/config.yaml` (`outputs.save_*`).

### `gpr_result.json` (robot/POKEYE)

- `hyperbola_detected`, `n_valid_detections`
- `calibration` (scan length, corrected time window, assumed εr), `consensus`
- `detections[]`: `position.x_m / x_cm / x_relative`, `depth.depth_cm` (+ assumed εr),
  `geometry` (apex_px, twt_ns, velocity, arc half-width), `robustness` (confidence, gains)
- `files`, `compatibility`

Horizontal positions are **local to the B-scan** (metres along the scan), not robot/world
coordinates: the software layer must transform them with the robot pose and the GPR
sensor extrinsics.

`gpr_data.json` is the downstream MIP format and includes interpretation assumptions
(element/material/recovery); it is not the robot-facing sensor conclusion.

## Run

```bash
pip install -r ../../requirements.txt     # GPR/requirements.txt
python check_setup.py
python run_gpr_pipeline.py path/to/scan.sgy                     # -> GPR/test_output/<scan>/hyperbola/
python run_gpr_pipeline.py path/to/scan.sgy --output-dir out/   # exact output folder
```

```python
from gpr_integration import run_gpr_pipeline
result = run_gpr_pipeline("scan.sgy", output_dir="path/to/folder")
```

The `scan.csv` sidecar must sit next to the `.sgy`. Optional overrides: `--weights`,
`--tahzeeb-config`, `--config`.

## Notes and known issues

- **CPU inference.** `inference.device` is `cpu`: on the lab PC torch reports CUDA but
  cuDNN fails to load (`Cannot load symbol cudnnCreate`) and the process dies without a
  traceback. Use `null`/`cuda` only where CUDA/cuDNN work. ~40 s per scan on CPU.
- **UTF-8.** Tahzeeb's scripts print `→`. The CLIs switch stdout to UTF-8; when calling
  `run_gpr_pipeline()` from another program on Windows set `PYTHONIOENCODING=utf-8`.
- **Zero scan distance.** Scans whose sidecar has `trace_distance = 0` are rejected with a
  `ValueError` (the B-scan cannot be calibrated).
- **Apex accuracy (open).** Diagnosed on the test scans, not fixed yet:
  1. `pulse_offset_samples = 21` is in pixels, but the rendered image height depends on
     the scan length, so it is 0.06 ns on one scan and 0.16 ns on another (intended ≈ 0.28 ns).
  2. Hyperbolas whose apex lies at time-zero (very shallow) get the offset as their depth (≈ 1 cm).
  3. Crossings of neighbouring hyperbola arms are detected as hyperbolas (Castelldefels).
  4. With fixed εr = 6 the fits are too narrow where the wall is not concrete
     (Castelldefels arms imply ≈ 0.31 m/ns).
  5. `02_hyperbola_fit.load_ridge_lookup` ignores the tile `src_y0` (second tile row).
- `02c_size_estimate.py` is referenced by Tahzeeb's `config.yaml` but was not supplied; it
  is not needed by the 01→02→02b→03→04 chain.
