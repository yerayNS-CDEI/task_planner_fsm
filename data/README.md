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
  gpr/incoming/                    shared inbox for HAND-COPIED GP8800 scans
                                   (<name>.sgy + <name>.csv); read by every session,
                                   after the session's own folder
  gpr/session_<stamp>/gpr_lines.jsonl
                                   one row per GPR line scanned (segment geometry,
                                   timestamps, line status, export outcome) written
                                   by ScanWall. With no hyperspectral session on
                                   disk, the offline run picks the GPR session with
                                   the newest manifest (-p sensor_session_id:=<stamp>
                                   names one; the folder may be renamed by hand)
  gpr/session_<stamp>/exports/     the raw export zips, one per line (<key>_<stamp>.zip)
  gpr/session_<stamp>/incoming/    the zips unpacked: <key>_<name>.sgy/.csv/.json --
                                   what SensorDataProcessing runs on for this mission
processed/session_<stamp>/
  hsi/       reflectance.csv (copy), input.csv, hsi_result.json, hsi_classification_summary.csv, samples.csv,
             hsi_material_overview.png, spectra/ (one PNG per spectrum)
  gpr/       gpr_summary.json, processed_files.json (+ NO_DRILL constraints per scan)
  gpr/<key>/            <name>.sgy + <name>.csv (copy of the export)
  gpr/<key>/hyperbola/  gpr_result.json, gpr_data.json (MIP) + B-scan/mask/detection PNGs
  gpr/<key>/line/       line_result.json + raw/processed/depth B-scan PNGs
  pokeye/    decisions.json, targets.json (accepted + blocked, with the reason),
             pokeye_config.json (the package config as run), pokeye_request.json
  gpr_hyperspectral.json   global JSON for Tecnalia (GPR + HSI), one element per wall
```

The `<stamp>` is shared by `raw/` and `processed/` for one mission.
