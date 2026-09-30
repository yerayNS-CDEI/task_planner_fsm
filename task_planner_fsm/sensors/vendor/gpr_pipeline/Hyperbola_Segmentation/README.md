# DISCOVER / OLIWALL — GPR processing integration (v3)

Robot-facing integration for the Proceq GP8800. The package keeps the two supplied
codebases intact and adds only an orchestration layer:

- `../GPRTools/` — shared CDEI GPRTools folder at the parent project level.
- `tahzeeb_original/` — Tahzeeb's supplied tiler, Mask R-CNN inference, post-processing,
  exact `config.yaml`, and MIP export code.
- `models/best.pt` — supplied Mask R-CNN ResNet101 weights.
- `gpr_integration/` — new DISCOVER integration code.

## Current scope

The robot pipeline reports hyperbola detections and their location/geometric/depth
parameters. It does **not** claim to infer wall material from GPR.

Pipeline:

`GP8800 .sgy + .csv`
→ shared `../GPRTools`: `time_zero → background_removal → gain`
→ several grayscale, axis-free full B-scans
→ Tahzeeb `gpr_tiler_v9.py` (640×640)
→ Mask R-CNN ResNet101
→ per gain: `01_consolidate → 02_hyperbola_fit → 02b_dedup`
→ cross-gain physical-space consensus
→ robot result + Tahzeeb/MIP compatibility export.

### Why Tahzeeb post-processing is run per gain

The different gains are renderings of the **same acquisition**, not independent scans.
Tahzeeb `01_consolidate.py` removes tile-overlap duplicates within one B-scan, so mixing
all gains before that step could incorrectly NMS detections from different renderings.
Each gain therefore gets an isolated 01→02→02b chain. Only the resulting physical
objects are compared across gains.

### One model load, not ten

The v3 adapter tiles every gain first, then runs Tahzeeb's unmodified inference function
**once over all tiles**. It subsequently splits `detections.csv` and `masks.json` back by
gain before post-processing. This avoids repeatedly loading the ~241 MB ResNet101 model
while preserving the required per-gain isolation.

## Confirmed settings from Tahzeeb / Proceq

- Model: **Mask R-CNN ResNet101** (`models/best.pt`).
- Model input: **640×640** tiles.
- Tiler: `gpr_tiler_v9.py`; the integration explicitly uses `crop-size=640`,
  `overlap-x=0.35`, `overlap-y=0.0`, matching the supplied Tahzeeb config.
- The tiler detects the signal/reflection band and removes the uninformative lower part
  of the large B-scan before producing tiles.
- GP8800 sidecar `Time Window = 16 ns` is erroneous for these exports. Proceq confirmed
  **−4 ns**, therefore the effective raw window is **12 ns**. GPRTools already applies
  the subtraction; the integration checks for 12 ns and never subtracts twice.
- Depth fallback uses **εr = 6.0**, a representative concrete relative permittivity.
  It is explicitly exported as an assumption, not as a sensor-measured material value.

Tahzeeb's supplied `config.yaml` is preserved unchanged under `tahzeeb_original/`.
Runtime copies override only file paths and scan-specific calibration such as scan length,
actual rendered image dimensions, the corrected time window and εr=6.

## Gain configuration

The current demo configuration uses **one gain only: 40 dB**. Consequently
`min_gain_support: 1`, so the retained cross-gain layer behaves as a passthrough after
Tahzeeb's per-B-scan post-processing. The multi-gain architecture remains in the code so
it can be re-enabled later without redesigning the pipeline.

If multiple gains are re-enabled, the existing association tolerances are ±6 cm
horizontally and ±4 cm in depth. These are configurable integration parameters, not
validated physical constants.

## Primary software interface

```python
from gpr_integration import run_gpr_pipeline

result = run_gpr_pipeline("scan.sgy")
```

A matching `scan.csv` sidecar must be next to the `.sgy`. The returned `dict` is
JSON-serializable and is the intended hand-off to the software/ROS2 layer.

Main fields:

- `hyperbola_detected`
- `n_valid_detections`
- calibration/provenance
- consensus settings
- `detections[]`
  - `position.x_m`, `x_cm`, `x_relative`
  - `depth.depth_cm`, assumed εr and provenance
  - apex, TWT, velocity, fitted arc half-width
  - model confidence
  - supporting gains and cross-gain dispersion

ROS2 transport/message definitions remain outside this repository.

## Output folder

Top-level outputs for one scan include:

- `gpr_result.json` — **primary robot/software output**.
- `gpr_detections.csv` — final accepted cross-gain detections.
- `gpr_results.xlsx` — human-readable audit workbook.
- `gpr_consensus_audit.csv` — all consensus candidates, including rejected ones.
- `gpr_gain_summary.csv` — counts per gain.
- `gpr_final_detections.png` — final multi-gain overlay.
- `gpr_consensus_objects.csv` — final detections converted to Tahzeeb's object schema.
- `gpr_data.json` — **Tahzeeb schema v2 / MIP compatibility JSON**.
- `element_summary.csv` and `material_passport.csv` — Tahzeeb 03/04 downstream outputs.
- `tahzeeb_viz/` — optional Tahzeeb step-5 visualization.

Intermediate folders preserve tiles, raw model outputs, masks, consolidated detections,
object fitting and objects removed by dedup for every gain.

### Important distinction: `gpr_result.json` vs `gpr_data.json`

`gpr_result.json` is the sensor result recommended for robot integration.

`gpr_data.json` is generated by executing Tahzeeb's supplied `03_quantify.py` and
`04_export.py` **after the cross-gain consensus**, so it can be imported by the MIP/
digital-twin workflow. `03_quantify.py` also contains element-type/material/recovery
interpretations (steel, value, CO2, etc.). Those fields are downstream assumptions and
must not be interpreted as GPR material-classification capability.

If an element type is genuinely known from another source, it can be set in
`tahzeeb_final_export.element_type_override`. Otherwise Tahzeeb's original name-based
fallback is left untouched.

## Installation / setup

```bash
pip install -r requirements.txt
python check_setup.py
```

GPRTools' SEGY reader requires `obspy`. The model is large; GPU execution can be selected
through `inference.device` if appropriate in the deployment environment.

CLI:

```bash
python run_gpr_pipeline.py path/to/scan.sgy
```

Optional overrides:

```bash
python run_gpr_pipeline.py path/to/scan.sgy \
  --weights path/to/best.pt \
  --tahzeeb-config path/to/config.yaml \
  --output-dir path/to/output
```

No source-code path is hard-coded to a developer machine. Tahzeeb's original config keeps
his Windows paths exactly as supplied for traceability, but the integration creates a
runtime config with the active scan/output paths.

## One optional Tahzeeb file not supplied

The supplied `config.yaml` now contains a `size:` block referring to
`02c_size_estimate.py`, but that script is not present in the supplied Tahzeeb archive.
It is **not required** by his current `run_postprocessing.py` or by the MIP JSON chain
(01→02→02b→03→04), so it does not block the predemo pipeline. If object-diameter/size
estimation is wanted, request that exact script rather than recreating it.


## Parent-folder layout

In this combined DISCOVER package, GPRTools is not duplicated inside this folder. The hyperbola integration imports it from `../GPRTools`.

The demo configuration is currently set to a single `40 dB` gain (`min_gain_support: 1`).

## v4 recursive batch mode

The CLI now accepts either one SEGY file or a directory:

```bash
python run_gpr_pipeline.py "path/to/scan.sgy"
python run_gpr_pipeline.py "path/to/acquisitions" --output-dir "path/to/results"
```

For a directory, all `.sgy` / `.segy` files are found recursively. Each one must have its
same-basename `.csv` sidecar. The relative input directory structure is preserved beneath
the batch output root. Processing continues after a missing sidecar or per-scan failure.

Batch-level files:

```text
batch_summary.csv
batch_summary.json
```

Python API:

```python
from gpr_integration import run_gpr_batch
summary = run_gpr_batch("path/to/acquisitions", output_root="path/to/results")
```

The original `run_gpr_pipeline()` single-scan API is unchanged.

**Current v4 demo setting:** one gain only, `40 dB`, with `min_gain_support: 1`. Older
multi-gain text above describes the retained scalable architecture, but no cross-gain vote
is currently required in the demo configuration.
