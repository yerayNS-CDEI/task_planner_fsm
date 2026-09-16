# Sensor data (not in git)

Runtime input/output of the sensor processing. Everything except this file and
the `.gitkeep` markers is ignored by git. Override the root with the
`sensor_data_dir` ROS parameter.

```
raw/
  hyperspectral/session_<stamp>/   raw_samples.jsonl, calibration.json, metrics.json
                                   (written by ScanWall's HyperspectralSampler)
  hyperspectral/raw_data/          raw_intensity_<date>.csv (GDS/GRF/GSM counts)
  hyperspectral/reflectance/       reflectance_<date>.csv
  hyperspectral/inspections/       Hyperspectral_data_<date>.csv, registre_inspeccions.json
  hyperspectral/mti_log.json, Dataset_Entrenament_Nou.csv
                                   (the four above: arm_control's inspection_manager;
                                   override with --output-dir or $HYPERSPECTRAL_DATA_DIR)
  gpr/incoming/                    GP8800 scans: <name>.sgy + <name>.csv (+ .json).
                                   ScanWall unpacks each line's export here as
                                   <key>_<name>.*; hand-copied exports work too
  gpr/session_<stamp>/gpr_lines.jsonl
                                   one row per GPR line scanned (segment geometry,
                                   timestamps, export outcome) written by ScanWall
  gpr/session_<stamp>/exports/     the raw export zips, one per line (<key>_<stamp>.zip)
processed/session_<stamp>/
  hsi/       input.csv, hsi_result.json, classification_results.csv, samples.csv
  gpr/       gpr_summary.json, processed_files.json (+ NO_DRILL constraints per scan)
  gpr/<key>/ gpr_result.json, line_result.json + the pipelines' audit files
  pokeye/    decisions.json, targets.json (accepted + blocked, with the reason), pokeye_request.json
```

The `<stamp>` is shared by `raw/` and `processed/` for one mission.
