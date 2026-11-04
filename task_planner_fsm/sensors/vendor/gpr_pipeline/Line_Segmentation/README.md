# GPR — Line segmentation (Berta)

Detects horizontal lines (layers, faces) in a Proceq GP8800 B-scan and returns their
depth below the front face.

Normally run through `GPR/codes/run_gpr.py`, which runs this branch and the hyperbola branch on
the same scan. It can also be run on its own (see *Run*).

## Contents

| Path | What it is |
|---|---|
| `berta_original/line_segmentation.py` | Berta's detector, **unchanged**. A new version from her can replace this file. |
| `line_integration/` | DISCOVER integration layer: preprocessing, time-zero merge, depth conversion, figures. |
| `config.yaml` | Gain, time-zero method, assumed εr, rendering and outputs. |
| `run_line_pipeline.py` | CLI for this branch only. |
| `check_setup.py` | Checks code and dependencies. |

Shared preprocessing comes from `GPR/codes/GPRTools/` (not duplicated here). Dependencies: `GPR/requirements.txt`.

## Pipeline: detect before cropping at time-zero

```text
raw .sgy + .csv
   ├─> gain 40 dB only ──> full B-scan ──> Berta line detector
   └─> time-zero estimated on the raw data (index only, the detector image is not cropped)
                                        └─> merged after detection ──> two-way time, depth (cm)
```

The detector sees the whole gain-only B-scan, so a shallow line cannot be cropped away
before detection. Time-zero is then used only to convert image rows into depth.

```text
v = c / sqrt(εr)      εr = 6.0 (assumed, concrete)      depth = v · two_way_time / 2
```

The 12 ns window (GP8800 16 ns − 4 ns, confirmed by Proceq) is applied by GPRTools.

## Outputs

With `run_gpr.py` they go to `GPR/test_output/<scan>/line/`, next to the scan's
`.sgy` + `.csv`:

| File | Content |
|---|---|
| `line_bscan_raw.png` | B-scan **as acquired** (no gain, no time-zero), nothing drawn on it. |
| `line_bscan_processed.png` | B-scan **after processing** (gain 40 dB only): exactly the image Berta's detector sees, nothing drawn on it. |
| `line_bscan_depth.png` | Final figure in the layout of Tahzeeb's v2 `bscan_figure.py`: B-scan with **depth below the front face [cm]** on the left axis, front face (time-zero, purple) and every line (orange, with its band); beside it the averaged trace with each line's id and depth and two-way time on the right axis; below, the line table and a card (time-zero, assumed speed, gain, record, scan length). No text on the radargram. |
| `line_result.json` | **Internal robot/POKEYE JSON**: the `dict` returned by `run_line_pipeline()`. |

- The **global JSON for Tecnalia** (both GPR branches + HSI) is built by `FULL PIPELINE/Global JSON/`.
- The depth figure shows from just above time-zero to the deepest line plus half its
  depth (at least 2 ns). A line above time-zero is drawn red and dashed.
- Each file can be switched off in `config.yaml` (`outputs.save_*`).
- Intermediate files are deleted after a successful run.

### `line_result.json` (robot/POKEYE)

```python
{
    "detected": True,
    "n_lines": 1,
    "lines": [{
        "id": "L001",
        "position_relative": 10.4,          # 0..100 over the full B-scan
        "depth_cm_approx": 1.4,             # from time-zero; null if clearly above it
        "thickness_cm_approx": 1.8,         # radar band extent, NOT a validated thickness
        "distance_to_next_cm_approx": None,
        "twt_from_time_zero_ns": 0.24,
        "confidence": 0.98,
        "before_time_zero_reference": False,
        ...                                 # Berta's original fields, kept for traceability
    }],
    "source": {...}, "time_zero_reference": {...}, "depth_calibration": {...}, "files": {...}
}
```

A ±1-sample tolerance around time-zero is treated as 0 cm.

## Run

```bash
pip install -r ../../requirements.txt     # GPR/requirements.txt
python check_setup.py
python run_line_pipeline.py path/to/scan.sgy                  # -> GPR/test_output/<scan>/line/
python run_line_pipeline.py path/to/scan.sgy --output out/    # exact output folder
```

```python
from line_integration import run_line_pipeline
result = run_line_pipeline("scan.sgy", output_dir="path/to/folder")
```

The `scan.csv` sidecar must sit next to the `.sgy`. Scans whose sidecar has
`trace_distance = 0` are rejected with a `ValueError`.
