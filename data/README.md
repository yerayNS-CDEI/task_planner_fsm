# Sensor data (not in git)

Runtime input/output of the sensor processing. Everything except this file and
the `.gitkeep` markers is ignored by git. Override the root with the
`sensor_data_dir` ROS parameter.

```
raw/
  hyperspectral/session_<stamp>/   raw_samples.jsonl, calibration.json, metrics.json
                                   (written by ScanWall's HyperspectralSampler)
  gpr/incoming/                    drop the GP8800 exports here: <name>.sgy + <name>.csv
  gpr/session_<stamp>/gpr_lines.jsonl
                                   one row per GPR line scanned (segment geometry,
                                   timestamps) written by ScanWall
processed/session_<stamp>/
  hsi/       input.csv, hsi_result.json, classification_results.csv, samples.csv
  gpr/<key>/ gpr_result.json, line_result.json + the pipelines' audit files
  pokeye/    decisions.json, targets.json, pokeye_request.json
```

The `<stamp>` is shared by `raw/` and `processed/` for one mission.
