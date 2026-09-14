# DISCOVER / OLIWALL - GPR processing package

Single parent package for the GPR work delivered to the robot/software team.

```text
GPR_DISCOVER_robot_pipeline/
├── GPRTools/                  # shared source of truth for GP8800 loading/preprocessing
├── Hyperbola_Segmentation/    # Task 1a - Tahzeeb Mask R-CNN + post-processing
└── Line_Segmentation/         # Task 1b - Berta horizontal-line detector + physical calibration
```

## Why GPRTools is shared

`GPRTools/` exists only once. Both interpretation branches import this sibling package instead of keeping their own copies. Future changes to GPRTools therefore do not need to be manually duplicated into the hyperbola and line projects.

## Task 1a - Hyperbola segmentation

This is the latest supplied Tahzeeb pipeline, reorganized so it imports `../GPRTools` rather than containing a private GPRTools copy. The current demo configuration is set to a single `40 dB` gain, with `min_gain_support: 1` and visualization at `40 dB`.

Use its own README for execution details.

## Task 1b - Line segmentation

This branch starts directly from GP8800 `.sgy + .csv`.

For line detection, the image is generated using **gain only (40 dB)**. Time-zero is computed independently on the raw data and merged only after Berta has detected the lines. Robot-facing coordinates are then re-based so **time-zero = 0**. Any line detected before time-zero is snapped to time-zero, because the time-zero estimate has priority over the line detector.

Approximate physical coordinates use the corrected GP8800 `12 ns` window and assumed `epsilon_r = 6.0`:

```text
v = c / sqrt(epsilon_r)
depth = v * TWT / 2
```

See `Line_Segmentation/README.md` for details and caveats.

## Dependencies

Each segmentation folder contains its own `requirements.txt`. Both rely on the shared GPRTools SEGY reader, which requires `obspy`.

## GPRTools import note

The GPR signal-processing algorithms are unchanged. A few package/dispatcher import files were made lazy/defensive so the SEGY-only path does not force unrelated optional DZT dependencies. See `GPRTools/CHANGES_DISCOVER.md`.
