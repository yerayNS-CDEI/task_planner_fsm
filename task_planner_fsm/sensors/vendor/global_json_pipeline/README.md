# Global JSON — GPR + hyperspectral part (for Tecnalia)

The global JSON sent to Tecnalia combines every sensor of the robot plus data that only
the robot knows (poses, world coordinates, element naming). **This module fills the GPR and
hyperspectral part.** Every robot-dependent field is written as `null` so the robot
software team can fill it.

```text
Global JSON/
├── README.md, requirements.txt
├── codes/
│   ├── run_global_json.py        # CLI: runs GPR (both branches) + HSI, then builds the JSON
│   └── global_json/              # builder.py + config.json
├── test_data/elements.json       # which acquisitions belong to each element (uses ../GPR and ../Hyperspectral test data)
└── test_output/                  # gpr_hyperspectral.json + the sensor outputs it points to
```

## Run

```bash
cd codes
python run_global_json.py                      # test_data/elements.json -> test_output/
python run_global_json.py --reuse              # only rebuild the JSON from results already in test_output/
python run_global_json.py path/to/elements.json --output-root path/to/outputs
```

`elements.json` (paths relative to the file):

```json
{"elements": {"column1": {"gpr_scans": ["scan_a.sgy", "scan_b.sgy"],
                          "hyperspectral_session": "session_20261002_131059"}}}
```

Each `.sgy` needs its Screening Eagle `.csv` next to it. The element key (`column1`) is the
top-level key of the JSON; in the robot it should be the element name the software team uses.

## Output

```text
test_output/
├── gpr_hyperspectral.json
└── <element>/
    ├── gpr/<scan>/                  # same as GPR/test_output/<scan>/ (sgy + csv, hyperbola/, line/)
    └── hyperspectral/<session>/     # same as Hyperspectral/test_output/<session>/
```

`data_pointer` / `data_pointer_csv` are relative to the JSON file (`"path_mode": "absolute"`
in `config.json` writes absolute paths instead).

## Format

Changes with respect to the draft example (`gpr_hyperspectral_comentado.json`), structure kept:

| Field | Draft | Now |
|---|---|---|
| `wall_material` | `"concrete"` | `""` (reserved for the future) |
| `data_pointer` | `""` | path of the GPR `.sgy` (its `.csv` sidecar is next to it) |
| `further_inspection` | object with `required` + list of ids | removed |
| `depth_profile` | segments with `from`/`to` coordinates + one depth | `wall_layers`: list of layers, shallow to deep, each with `layer`, `depth` (top), `thickness`, `material` |
| unknown values | `"Unknown"` strings | `null` |
| `{value, error, unit}` | key order varied | always `value, error, unit` |
| `"object type"` | key with a space | `object_type` |
| trailing comma at the end | invalid JSON | removed |
| layout | one key per line | every `{value, error, unit}` and x/y/z point on one line (`dumps_compact`) |

Fields **added** (small, needed by the robot team or by the reader):

- `continuous_gpr[].bscan_length` (sidecar scan distance).
- `discrete_gpr[].position_along_bscan`: distance of the hyperbola apex from the B-scan start.
  With the robot pose this gives `point_coordinates`.
- `discrete_gpr[].confidence` (Mask R-CNN score).
- `hyperspectral_camera.measurements[].status`
  (`detected | low_confidence | quality_rejected`; `material` is `null` unless `detected`).

### Who fills what

| Field | Filled by |
|---|---|
| `continuous_gpr[].bscan_coordinates` (start/end, world) | robot software team (`null`) |
| `discrete_gpr[].point_coordinates` (world) | robot software team (`null`), from `position_along_bscan` + pose |
| `discrete_gpr[].material`, `size` | `null`: the GPR does not classify material or measure diameter |
| `hyperspectral_camera.measurements[].timestamp`, `point_coordinates` | robot software team (`null`) |
| `hyperspectral_camera_2` (XIMEA) | not processed here: empty |
| everything else | this module |

### How the values are obtained

- **`discrete_gpr`**: one entry per object of Tahzeeb's `hyperbola/gpr_data.json`, i.e. only
  the hyperbolas that survive his whole chain (01 consolidation/NMS → 02 fit → 02b dedup →
  04 export), not the raw Mask R-CNN detections (Castelldefels test: 29 raw masks → 14).
  `depth` = apex depth with εr = 6 (assumed). `depth.error` = depth × σv/v, with σv/v
  from the same file (`velSigmaMns`, floor 0.005 m/ns). This does **not** include the εr
  assumption itself nor the apex-location error (not solved yet, see
  `GPR/codes/Hyperbola_Segmentation/README.md`). `position_along_bscan.error` is `null`.
- **`wall_layers`**: built from the horizontal lines of `line/line_result.json` (Berta).
  Each line below the surface is an interface; the surface (time zero) is depth 0, and a
  line at or before time zero is that surface reflection, not an interface. Layer *k* goes
  from interface *k−1* (the surface for *k* = 1) to interface *k*: `depth` = its top,
  `thickness` = bottom − top. Below the deepest interface the back of the wall is not seen,
  so no layer is written there. `material` = `"unknown"` for now (`layer_material`).
  Interface error = depth × σv/v combined with half the line-band thickness; thickness
  error combines the errors of its top and bottom interfaces.
- **`hyperspectral_camera`**: one measurement per spectrum of `hsi_result.json`; `row` is
  the row in `reflectance.csv`; `data_pointer_csv` points to that CSV.

## Test

`test_data/elements.json` defines two elements with the module test data:

| Element | GPR | HSI |
|---|---|---|
| `hercal_demo_20261002` | `scan_wall line 1 seg 1_20261002_141034.826` | `session_20261002_131059` (same day) |
| `castelldefels_20261008` | `scan_wall line 1 seg 1_20261008_110058.012` | none (empty block) |
