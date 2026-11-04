# Hyperspectral — material classification (Benjamin v2)

Classifies the material of each VIS-NIR spectrum measured by the robot and returns
material, confidence and spectral quality.

```text
Hyperspectral/
├── README.md, requirements.txt
├── codes/
│   ├── run_hsi_pipeline.py      # CLI
│   ├── check_setup.py
│   ├── benjamin_original/       # Benjamin v2, unchanged: classifier.py + classifier.joblib (+ SOURCE_MANIFEST.txt)
│   └── hsi_integration/         # DISCOVER layer: input adapter, robot JSON, figures, config.json
├── test_data/                   # one real robot session
└── test_output/                 # outputs of the test session
```

## Pipeline

```text
acquisition (robot session reflectance.csv | LENZ .xls | CSV)
→ format adapter (decimal commas, VIS_/NIR_ prefixes, keep Sample rows)
→ Benjamin v2: quality filter → resampling to the model grid (per sensor, no interpolation across the gap)
  → SG smoothing + SNV + 1st derivative → StandardScaler → segmented 1D-CNN (PyTorch)
→ material / confidence / 0-100 spectral quality
```

Classes: brick, cementitious, ceramic, gypsum, metal, polymer, wood. Confidence
threshold **0.80** (`codes/hsi_integration/config.json`, keep it in sync with POKEYE).

## Accepted input

1. **Robot acquisition session** — `session_<date>_<time>/` with `reflectance.csv` (pass the
   folder or the file). One row per trigger: `Seq`, `Timestamp`, wall/line/segment/trigger,
   `Travel_m`, pose (`Map_X/Y/Z`, `Frame`), acquisition `Status`/`Reason`, spectra
   `VIS_<nm>` / `NIR_<nm>`. Reflectance is computed by the acquisition software as
   `(sample − dark) / (white − dark)` per channel with the session's `calibration.json`;
   sessions without `reflectance.csv` (raw counts only) are not supported.
2. **LENZ export (`.xls`)** — tab-separated text with comma decimals; only `Measure Type = Sample`
   rows are classified (reference/dark rows are only counted).
3. **CSV** with one spectrum per row and numeric wavelength (nm) column names.

## Outputs

```text
test_output/<session>/
├── reflectance.csv                  # acquisition CSV with the raw data, unmodified copy of the input
├── hsi_classification_summary.csv   # inference results, one row per spectrum
├── spectra/spectrum_00001.png …     # one image per spectrum (Benjamin's figure)
├── hsi_material_overview.png        # summary: one panel per result, its spectra overlaid
└── hsi_result.json                  # internal robot/POKEYE JSON
```

- `hsi_classification_summary.csv`: `row` (1-based), `source_row` (row in the input file),
  `material`, `confidence`, `quality`, `quality_grade`.
- `hsi_material_overview.png` groups spectra by Benjamin's verdict: each material,
  `UNDETERMINED` (below the threshold) and `NOT EVALUABLE` (quality filter).
- `spectra/` takes ~0.2 s per spectrum; switch it off with `outputs.save_spectrum_images: false`.
- The **global JSON for Tecnalia** (`hyperspectral_camera` block) is built by `FULL PIPELINE/Global JSON/`.

### `hsi_result.json` (robot/POKEYE)

```python
{
    "sensor": "hyperspectral", "input_file": "reflectance.csv",
    "classifier": {"author": "Benjamin", "version": "v2", ...},
    "confidence_threshold": 0.8, "input_summary": {...},
    "n_samples": 16, "n_detected": 14,
    "samples": [{
        "sample_index": 0, "source_row": 1,
        "detected": True, "material": "metal", "confidence": 0.9999,
        "status": "detected",              # detected | low_confidence | quality_rejected
        "reason": None,
        "spectrum_quality": 100.0, "spectrum_quality_grade": "High",
        "metadata": {"Seq": "1", "Timestamp": "...", "Travel_m": "0.0", "Frame": "map",
                     "Map_X": "-9.0631", "Map_Y": "-7.78062", "Map_Z": "1.50015", "Status": "accepted"}
    }]
}
```

| Benjamin | `status` |
|---|---|
| material shown | `detected` |
| `UNDETERMINED` | `low_confidence` |
| `NOT EVALUABLE` | `quality_rejected` (`reason` = Benjamin's reason) |

`metadata.Status` is the **acquisition** status of the robot software, not the
classification `status`. With exactly one sample, `detected / material / confidence /
status / reason` are also copied to the top level (single-measurement payload for POKEYE).

## Run

```bash
pip install -r requirements.txt
cd codes
python check_setup.py
python run_hsi_pipeline.py ../test_data/session_20261002_131059  # -> Hyperspectral/test_output/<session>/
python run_hsi_pipeline.py path/to/measurement.xls --output-dir out/
```

```python
from hsi_integration import run_hyperspectral_pipeline
result = run_hyperspectral_pipeline("path/to/session", output_dir="path/to/folder")
```

- The model bundle was saved with **scikit-learn 1.9.0**.
- Inference runs on CPU (`torch_device` in `config.json`; on the lab PC CUDA is detected
  but cuDNN fails to load).

## Test

`test_data/session_20261002_131059` (robot session, 2026-10-02, 16 spectra, no
ground-truth label): 14 detected (12 polymer, 2 metal), 1 low_confidence (0.78),
1 quality_rejected (flat spectrum; the acquisition software had already marked it
`rejected_stability`, VIS underexposed). POKEYE is requested for 2 spectra.

The LENZ `.xls` path was checked with 120 spectra of `dataset_LENZ.csv` rewritten as a
LENZ-style export: results identical to Benjamin's `classifier.py` run standalone.

Benjamin v1 (XGBoost, `predict.py`) is kept only in
`CODES - Benjamin (Hyperspectral, material classification)/v1`.
