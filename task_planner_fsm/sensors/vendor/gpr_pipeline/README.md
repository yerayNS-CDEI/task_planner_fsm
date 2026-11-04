# GPR (Proceq GP8800) — hyperbola + line segmentation

```text
GPR/
├── README.md, requirements.txt    # dependencies of both branches
├── codes/
│   ├── run_gpr.py                 # runs both branches on one or more scans
│   ├── GPRTools/                  # shared CDEI loading/preprocessing library (used by both branches)
│   ├── Hyperbola_Segmentation/    # Tahzeeb: Mask R-CNN hyperbola detection + fit
│   └── Line_Segmentation/         # Berta: horizontal-line detection + depth
├── test_data/                     # 2 scans (.sgy + Screening Eagle .csv) for testing
└── test_output/                   # outputs of the test scans
```

## Run

```bash
pip install -r requirements.txt
cd codes
python run_gpr.py                         # every scan in GPR/test_data/ -> GPR/test_output/
python run_gpr.py path/to/scan.sgy        # one scan; its .csv must sit next to it
python run_gpr.py path/to/folder --output-root path/to/outputs
```

A branch that fails on a scan is reported and does not stop the other branch.

## Outputs per scan

```text
test_output/<scan>/
├── <scan>.sgy                    # acquisition, unmodified
├── <scan>.csv                    # Screening Eagle sidecar, unmodified
├── hyperbola/
│   ├── gpr_bscan_raw.png         # B-scan before processing
│   ├── gpr_bscan_processed.png   # after GPRTools filters (time-zero, background removal, gain)
│   ├── gpr_segmentation_masks.png  # Mask R-CNN masks + scores
│   ├── gpr_final_detections.png  # masks + hyperbola fits + apexes
│   ├── gpr_data.json             # Tahzeeb's JSON (MIP schema)
│   └── gpr_result.json           # internal robot/POKEYE JSON
└── line/
    ├── line_bscan_raw.png        # B-scan before processing
    ├── line_bscan_processed.png  # gain-only image seen by Berta's detector
    ├── line_bscan_depth.png      # final depth figure (Tahzeeb v2 layout)
    └── line_result.json          # internal robot/POKEYE JSON
```

**Global JSON for Tecnalia:** built from these outputs by `../Global JSON/` (one
`continuous_gpr` entry per scan, one `discrete_gpr` entry per hyperbola, lines as
`wall_layers`).

## Test data

`test_data/`:

| Scan | Origin | Length | Result |
|---|---|---|---|
| `scan_wall line 1 seg 1_20261002_141034.826` | Hercal demo | 1.32 m | 4 hyperbolas, 1 line |
| `scan_wall line 1 seg 1_20261008_110058.012` | Castelldefels | 4.54 m | 14 hyperbolas, 1 line |

The other 55 scans (Hercal demo and Castelldefels) were moved out of FULL PIPELINE to
`OLIWALL CODES + API/GPR_DATA_ARCHIVE/` (checksum-verified copy).

## Common to both branches

- **GPRTools** exists once; both branches import it. Its algorithms are unchanged; only
  optional DZT imports were made lazy (`codes/GPRTools/CHANGES_DISCOVER.md`).
- **Time window:** the GP8800 sidecar reports 16 ns; Proceq confirmed −4 ns → 12 ns
  (applied once by GPRTools).
- **Depth** uses an assumed εr = 6.0 (concrete), not a measured value.
- **Coordinates:** horizontal positions are along the B-scan, not robot/world
  coordinates. The software layer must transform them with the robot pose and the GPR
  extrinsics.
- **Invalid scans:** a sidecar with `trace_distance = 0` makes both branches fail with a
  `ValueError` (7 of the 57 archived scans: Hercal 20261001_170917, 20261002_105342,
  _105557, _134055, _141808; Castelldefels 20261008_110532, _132709).
- **Windows:** inference runs on CPU (cuDNN fails on the lab PC) and the CLIs switch stdout
  to UTF-8 for Tahzeeb's prints. See `codes/Hyperbola_Segmentation/README.md`.
