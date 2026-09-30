# GPR_DISCOVER_PIPELINE_v4

Unified DISCOVER / OLIWALL GPR package for the robot/software team.

```text
GPR_DISCOVER_PIPELINE_v4/
├── GPRTools/                  # shared GP8800 loading and preprocessing
├── Hyperbola_Segmentation/    # Task 1a - Tahzeeb Mask R-CNN pipeline
├── Line_Segmentation/         # Task 1b - Berta line detector + time-zero/depth merge
└── run_gpr_processing.py      # one scan OR recursive folder runner
```

`GPRTools/` exists only once and is shared by both interpretation branches.

## New in v4: recursive folder processing

All runners still accept a single `.sgy`, but they can now also receive a directory.
Directories are scanned recursively for `*.sgy` and `*.segy` files. Each acquisition must
have a same-basename GP8800 sidecar CSV beside it:

```text
acquisitions/
├── room_1/
│   ├── wall_A.sgy
│   ├── wall_A.csv
│   ├── ceiling.sgy
│   └── ceiling.csv
└── room_2/
    └── wall_B/
        ├── scan_003.sgy
        └── scan_003.csv
```

The input directory hierarchy is preserved under the output directory. A missing sidecar
or a processing error for one scan does **not** abort the remaining scans; it is recorded
in `batch_summary.csv` and `batch_summary.json`.

Processing is intentionally sequential. This avoids loading/running several large Mask
R-CNN jobs concurrently on the robot computer.

## Recommended command: run both GPR branches

From this folder:

```bash
python run_gpr_processing.py "path/to/acquisitions"
```

This recursively runs both hyperbola and line processing. Default output:

```text
outputs/<input_folder>/
├── Hyperbola_Segmentation/
│   └── <same relative input folders>/<scan>/...
├── Line_Segmentation/
│   └── <same relative input folders>/<scan>/...
├── batch_summary.csv
└── batch_summary.json
```

Choose a runtime directory explicitly:

```bash
python run_gpr_processing.py "path/to/acquisitions" \
  --output-root "path/to/results"
```

Run only one branch if required:

```bash
python run_gpr_processing.py "path/to/acquisitions" --mode hyperbola
python run_gpr_processing.py "path/to/acquisitions" --mode line
```

The same top-level command also accepts one `.sgy` file.

## Branch-specific commands

Hyperbola only:

```bash
cd Hyperbola_Segmentation
python run_gpr_pipeline.py "path/to/scan.sgy"
python run_gpr_pipeline.py "path/to/acquisition_folder" --output-dir "path/to/results"
```

Line only:

```bash
cd Line_Segmentation
python run_line_pipeline.py "path/to/scan.sgy"
python run_line_pipeline.py "path/to/acquisition_folder" --output "path/to/results"
```

For directory input, `--output-dir` / `--output` is the **batch output root**. The relative
input folder structure is then reproduced underneath it.

## Task 1a - hyperbola segmentation

Current demo configuration uses **gain 40 dB only**. The pipeline is:

`SEGY + CSV -> GPRTools(time-zero + background removal + gain 40) -> tiler 640x640 -> Mask R-CNN ResNet101 -> Tahzeeb consolidate/fitting/dedup -> outputs`.

Primary robot-facing output per scan: `gpr_result.json` / returned Python `dict`.

A scan with zero valid hyperbolas is a normal valid result, not a processing error. In that case the pipeline returns `hyperbola_detected = false` and `n_valid_detections = 0`. The integration layer also handles the case where the model returns no detections, or all candidates are removed by Tahzeeb's consolidation filters, without aborting a batch.
Horizontal GPR coordinates are local to the B-scan/scan direction, not robot/world
coordinates. Software must transform them using the acquisition pose and sensor extrinsics.

Approximate depth currently assumes `epsilon_r = 6.0`.

## Task 1b - line segmentation

The detector sees a **gain-40-only, untrimmed B-scan**. Time-zero is estimated on the raw
data in a separate branch and merged only after line detection. Robot-facing coordinates
are then re-based so `time-zero = 0`. A detected line before time-zero is snapped to
exactly time-zero.

Approximate depth uses the Proceq-corrected 12 ns window and assumed `epsilon_r = 6.0`.

Primary robot-facing output per scan: `line_result.json` / returned Python `dict`.

## Batch summaries

Each branch generates its own summary, and `run_gpr_processing.py` also creates a combined
summary. Important statuses are:

- `OK` - scan processed.
- `SKIPPED_MISSING_SIDECAR` - `.sgy/.segy` found but the same-basename `.csv` is absent.
- `ERROR` - processing failed; the error text is recorded and the batch continues.

The summaries are for orchestration/audit. ROS2 should consume the per-scan compact Python
`dict`/JSON result rather than parse diagnostic CSV/XLSX files during normal operation.

## Dependencies

Each segmentation folder has its own `requirements.txt`. Both share `GPRTools`, whose SEGY
reader requires `obspy`. Hyperbola inference includes the supplied Mask R-CNN ResNet101
weights under `Hyperbola_Segmentation/models/best.pt`.
