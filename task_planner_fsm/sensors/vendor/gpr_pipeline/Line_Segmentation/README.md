# DISCOVER / OLIWALL - GPR Line Segmentation (Task 1b)

This folder integrates Berta's horizontal-line detector with the shared `../GPRTools` package and converts the detected vertical image coordinates into approximate centimetres using a time-zero reference and an assumed relative permittivity of `epsilon_r = 6`.

## Key rule: detect first, then redefine coordinates from time-zero

The B-scan passed to Berta is generated with **gain only** (default `40 dB`). No time-zero crop and no background removal are applied before line detection.

Time-zero is estimated independently on the raw matrix. Only **after** Berta has detected the lines do we redefine the coordinate system:

```text
robot y = 0  <=>  time-zero
robot y = 100 <=> last raw sample
```

```text
raw SEGY + sidecar CSV
        |
        +--> gain 40 dB only --> full B-scan --> Berta line detector
        |
        +--> time-zero estimation only
                                  |
                                  +--> merge AFTER detection
                                         |
                                         +--> rebase coordinates: T0 = 0
                                         +--> clamp any pre-T0 line to T0
                                         +--> convert to approximate cm
```

### Time-zero priority rule

If Berta detects a line **before** the time-zero sample, that line is moved to time-zero for all robot-facing outputs:

```text
corrected_sample = max(detected_sample, time_zero_sample)
```

Therefore negative depth is never reported. The original Berta coordinate is retained only in diagnostic/audit fields.

With the B-scan convention used by GPRTools (`0` at the top and time increasing downwards), a line at sample 35 with time-zero at sample 40 is snapped to sample 40. A line at sample 40 with time-zero at sample 35 is already after time-zero and remains at sample 40.

## Physical conversion

GPRTools applies the Proceq-confirmed correction from exported `16 ns` to an effective `12 ns` window.

```text
v = c / sqrt(epsilon_r)
epsilon_r = 6.0
depth = v * two_way_time / 2
```

`epsilon_r = 6` is an **assumption representative of concrete**, not a measured material property.

Important interpretation:

- `position_relative`: robot-facing `0..100` coordinate **after time-zero correction**.
- `depth_cm_approx`: approximate line-centre depth after time-zero.
- `thickness_cm_approx`: depth-equivalent extent of the detected dark band; **not validated physical object/material thickness**.
- `distance_to_next_cm_approx`: recomputed between corrected line centres, after any T0 snapping.
- `position_relative_original`: original Berta coordinate on the complete untrimmed B-scan, kept only for audit/debugging.

## Shared folder layout

```text
GPR_DISCOVER_PIPELINE_v4/
├── GPRTools/
├── Hyperbola_Segmentation/
└── Line_Segmentation/
```

Berta's supplied `line_segmentation.py` remains unchanged in `berta_original/`. All DISCOVER/GPR integration logic lives in `line_integration/`.

## Input

A GP8800 non-mesh SEGY file and its sidecar CSV with the same basename:

```text
scan.sgy
scan.csv
```

## Run

```bash
pip install -r requirements.txt
python check_setup.py
python run_line_pipeline.py "path/to/scan.sgy"
```

## Robot / ROS2-facing use

```python
from line_integration import run_line_pipeline
result = run_line_pipeline("path/to/scan.sgy")
```

Relevant structure:

```python
{
    "detected": True,
    "n_lines": 2,
    "lines": [
        {
            "id": "L001",
            "position_relative": 0.0,       # relative to T0, not to original image
            "depth_cm_approx": 0.0,
            "snapped_to_time_zero": True,
            "position_relative_original": 5.7,
            "confidence": 0.91
        }
    ],
    "time_zero_reference": {
        "robot_relative_coordinate": 0.0,
        "has_priority_over_line_detector": True
    }
}
```

## Outputs

```text
outputs/<scan>/
├── 01_gain_only/
├── 02_line_detection/        # original Berta detections
├── 03_time_zero_merge/       # corrected/T0-rebased robot interpretation
├── results_finales.csv
├── resultados_resumen.csv
├── resultados_lineas.csv
├── line_result.json
├── preprocessing_manifest.json
└── run_manifest.json
```

`results_finales.csv` now exposes `posicion_lineas_relativa` in the **time-zero-rebased coordinate system**. Berta's original coordinate is kept separately as `posicion_lineas_relativa_original_berta`.

## v4 recursive batch mode

The CLI now accepts either one SEGY file or a directory:

```bash
python run_line_pipeline.py "path/to/scan.sgy"
python run_line_pipeline.py "path/to/acquisitions" --output "path/to/results"
```

For directory input, all `.sgy` / `.segy` files are found recursively. Each scan needs its
same-basename `.csv` sidecar. The folder hierarchy is preserved under the batch output
root, and one failed/missing scan does not stop the rest.

Batch-level files:

```text
batch_summary.csv
batch_summary.json
```

Python API:

```python
from line_integration import run_line_batch
summary = run_line_batch("path/to/acquisitions", output_root="path/to/results")
```

The original `run_line_pipeline()` single-scan API remains available.
