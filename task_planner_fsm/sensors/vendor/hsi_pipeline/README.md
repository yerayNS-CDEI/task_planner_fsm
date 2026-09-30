# DISCOVER — Hyperspectral material-classification pipeline (v2)

This folder is the integration package for task **(2)** of the OLIWALL sensor pipeline.
It preserves Benjamin's classifier and `predict.py` unchanged and adds only a thin adapter/output layer for the robot software.

## Scope

Input acquisition -> format/wavelength adaptation -> Benjamin quality filter + SNV+D1 + scaler + XGBoost -> material/confidence outputs.

This package does **not** implement ROS2 transport. The Python function returns a plain `dict` that the software team can serialize into its own ROS2 message/service/action interface.

## Accepted input

### 1. Real spectrometer export (`.xls`)

The supplied example `Data_18_06_2026.xls` is **not a binary Excel workbook**. It is a tab-separated text export with comma decimal separators. The pipeline reads that format directly; no manual conversion to Excel/CSV is required.

It may contain `Sample`, `Reference`, `Dark Reference` and `Dark Sample` rows. Only rows whose `Measure Type` is `Sample` are sent to the material classifier. Reference/dark rows are acquisition/calibration records and are counted in `input_diagnostics.json`, not classified as materials.

### 2. CSV

A CSV with one spectrum per row is also accepted.

## Wavelength-grid adaptation

The real sensor export contains 512 wavelength columns. Benjamin's classifier bundle stores the exact 450 wavelengths used during training (400.43–1648 nm, VIS + SWIR).

The integration layer maps the input spectrum to those exact 450 model wavelengths **inside each sensor segment separately**. No interpolation is performed across the VIS/SWIR gap. Missing source values are not silently bridged: a target value is interpolated only when both neighbouring source bands exist.

This is an input-format/grid adapter only. It intentionally does **not** replace Benjamin's cleaning logic. His original `predict.py` remains authoritative for:

- zero -> missing value handling;
- incomplete-segment rejection;
- residual missing-value interpolation;
- reflectance clipping/physical limits;
- spike and excessive-noise rejection;
- SNV + first derivative;
- StandardScaler;
- XGBoost classification.

## Run

From the package root:

```bash
pip install -r requirements.txt
python check_setup.py
python run_hsi_pipeline.py "path/to/Data_18_06_2026.xls"
```

Optional output directory:

```bash
python run_hsi_pipeline.py "path/to/input.xls" --output-dir "outputs/my_scan"
```

## Outputs

For each run:

```text
outputs/<input_stem>/
├── classification_results.csv   # detailed result for every Sample row
├── hsi_result.json              # dict/JSON intended for software / ROS2 mapping
├── summary.json                 # compact counts by status/material
├── input_diagnostics.json       # format, rows, bands and adapter diagnostics
└── run_manifest.json            # input/model/config/output traceability
```

`classification_results.csv` contains Benjamin's `clase_predicha`, `confianza`, quality-filter result and `prediccion_final`, plus available input metadata.

## ROS2-facing Python API

```python
from hsi_integration import run_hyperspectral_pipeline

result = run_hyperspectral_pipeline("scan.xls")
```

For each spectrum the relevant operational fields are:

```python
{
    "detected": True,
    "material": "gypsum",
    "confidence": 0.994,
    "status": "detected"
}
```

If quality is insufficient, `detected=False`, `material=None`, and `status` is `quality_rejected`. If the spectrum is valid but the maximum class probability is below the configured threshold, `status` is `low_confidence`.

The default confidence threshold is **0.80** and is configurable in `hsi_integration/config.json`.

## Validation with the supplied real file

The package was executed end-to-end on `Data_18_06_2026.xls`:

- 6,530 total acquisition rows;
- 6,475 `Sample` rows sent to the classifier;
- 55 reference/dark rows excluded from material classification;
- 5,353 samples passed Benjamin's quality filter;
- 4,680 samples were accepted above the 0.80 confidence threshold.

As a sanity check only, the example file contains descriptive `Label` values. For labels that map unambiguously to one of the model classes (`gypsum`, `brick`, `ceramic`, `cementitious`, `polymer`), **98.4% of the accepted predictions matched the material encoded in the label**. This is a validation of this example run, not a claim of general model accuracy.
