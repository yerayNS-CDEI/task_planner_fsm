# Vendored sensor pipelines

Code delivered by the DISCOVER/OLIWALL sensor team, copied here **unmodified**
(the only additions are empty `__init__.py` files so `find_packages()` ships the
tree on a non-symlink install). The FSM never imports these by their dotted
path: `task_planner_fsm.sensors` puts each project root on `sys.path` and uses
the public entry points exactly as the delivered READMEs describe.

| Folder here | Delivered as | Entry point |
|---|---|---|
| `gpr_pipeline/` | `FULL PIPELINE/GPR/codes` (2026-11-03) | `gpr_integration.run_gpr_pipeline`, `line_integration.run_line_pipeline` |
| `hsi_pipeline/` | `FULL PIPELINE/Hyperspectral/codes` (2026-11-03, Benjamin v2) | `hsi_integration.run_hyperspectral_pipeline` |
| `pokeye_decision_pipeline/` | `FULL PIPELINE/Pokeye Decision/codes` (2026-11-03, policy v2) | `pokeye_decision.decide_pokeye` |
| `global_json_pipeline/` | `FULL PIPELINE/Global JSON/codes` (2026-11-03) | `global_json.build_element`, `global_json.dumps_compact` |

Version suffixes are dropped from the folder names so a newer delivery can be
dropped in without touching any import path; record the new version in the
table above when you do.

## GPR v4 and HSI v1 -> FULL PIPELINE (2026-11-03)

The sensor team now ships every module in one `FULL PIPELINE/` folder. Each
module's `codes/` is what is vendored here; its `README.md` and
`requirements.txt` sit one level up in the delivery and are copied to the
folder root, where the earlier deliveries had them. `POKEYE Decision` and the
new `Global JSON` module were taken in a second pass on 2026-11-04; see the
next section.

**HSI: new classifier.** Benjamin v2 replaces the XGBoost model (`predict.py`)
with a segmented 1D-CNN in PyTorch (`classifier.py`, 117 kB bundle) that also
scores spectral quality 0-100. The integration reads the robot session's
`reflectance.csv` directly now, and each sample gains `source_row`,
`spectrum_quality` and `spectrum_quality_grade`; the keys `hsi.py` reads
(`sample_index`, `status`, `detected`, `material`, `confidence`, `reason`,
`metadata.Counter`) are unchanged. The device moved from the pickled XGBoost
models (the `joblib.load` wrapper `hsi.models_on` is gone) to the config's
`torch_device`, which `write_runtime_config` sets from `hsi_device`. The bundle
was pickled under numpy 2 and scikit-learn 1.9.0; with the existing
`numpy._core` alias it loads on the Jetson's numpy 1.24 / scikit-learn 1.7.2
and reproduces the delivered test output on all 16 spectra. Outputs per run:
`hsi_result.json`, `hsi_classification_summary.csv`,
`hsi_material_overview.png`, `spectra/*.png`; the v1 `classification_results.csv`,
`summary.json`, `input_diagnostics.json` and `run_manifest.json` are gone (and
deleted from the output folder if present).

**GPR hyperbolae.** Result keys are unchanged. Behaviour changes:

- `run_gpr_pipeline(output_dir=...)` is now the exact output folder; v4
  appended `<scan>/` to it. Results land in `gpr/<key>/hyperbolae/` directly.
- After a successful run the intermediate folders (`01_preprocessed` ...
  `05_consensus`) and the CSV/Excel side files are deleted; what stays is
  `gpr_result.json`, `gpr_data.json` and four PNGs (raw, processed, masks,
  final detections). `openpyxl` is no longer needed.
- The empty-detection fix moved again: instead of v4's `_consolidate_safely`
  (and its `audit.empty_detection_reason`), the adapter wraps Tahzeeb's
  `nms()` for empty input and skips consolidation when inference found
  nothing. Tahzeeb's files are still untouched.
- `inference.device` is delivered as `cpu` (see Local modifications).

**GPR lines.** The time-zero policy changed: a line the detector places
before time-zero is no longer snapped onto it. It keeps its signed position
(`position_from_time_zero_cm_signed`, `before_time_zero_reference`) and its
`depth_cm_approx` is `None`; `snapped_to_time_zero` is gone (`gpr.py` now
reports `before_time_zero` instead) and `TIMEZERO_POLICY.md` was dropped with
it. `distance_to_next_cm_approx` is Berta's own spacing again rather than
recomputed after the snap. A zero or invalid `trace_distance` in the sidecar
is now a clear `ValueError` before rendering. The CSV outputs are gone;
`line_result.json` plus three PNGs remain.

**Removed:** the v4 batch runners (`run_gpr_batch`/`run_gpr_path`,
`run_line_batch`/`run_line_path`, `run_gpr_processing.py`); the FSM never used
them. `run_gpr.py` is the delivery's new CLI over the test data.

## POKEYE v2 -> FULL PIPELINE, and the new Global JSON (2026-11-04)

**POKEYE: the policy changed, not just the code.** `decide_pokeye` now takes a
whole `hsi_result.json` (`samples` with `metadata.Wall_Index/Line_Idx/Seg_Idx/
Travel_m`) and the GPR hyperbola results, and decides:

| HSI | Before (v2 as vendored) | Now |
|---|---|---|
| `detected` | no action | no action |
| `quality_rejected` | **POKEYE** | no action (a sensor problem, not a material doubt) |
| `low_confidence` | POKEYE | POKEYE at the point, or **one** request for the wall when it is >= 50 % low-confidence (and >= 2 points) |
| on a hyperbola (<= 0.10 m along its line) | (FSM map screen only) | `BLOCKED_BY_GPR` |
| line without an analysed GPR result | (FSM coverage rule only) | no action, `GPR_CLEARANCE_UNKNOWN` |
| malformed | `HOLD` | no action, `INVALID_HSI_RESULT` |

How the FSM calls it (`sensors/pokeye.py`):

- **Once per wall.** The package matches a sample to its GPR line by
  `Line_Idx/Seg_Idx` only, so a call spanning walls would pair a sample with
  another wall's line of the same index.
- **GPR from the cumulative `gpr_summary.json`**, the compact hyperbolae turned
  back into the list form the package documents. A line swept per
  polarization pools both scans' hyperbolae and counts as analysed only if
  both were.
- **`Travel_m` vs `x_m`.** Both are plate travel from the segment start
  (`hyperspectral_sampler` / `gpr_sweep` reset at segment begin); the sensors'
  offsets on the plate are not applied, as in `gpr.scan_to_map`.
- **`require_gpr_clearance`** follows `pokeye_require_gpr_coverage` through a
  runtime copy of the config (`pokeye/pokeye_config.json`); every other value
  is the delivered one.
- **The FSM layers stay on top**, and they only ever remove targets: requests
  are clustered into targets (a wall request is kept whatever
  `pokeye_cluster_min_samples`), then screened in the map frame against the
  NO_DRILL zones (`pokeye_no_drill_tolerance_m`, 0.15 m) and the scanned-line
  coverage rule. A cluster centroid is not any of the points the package
  checked, and POKEYE's own RANDOM drilling needs the zones anyway.

`build_gpr_drilling_constraints` is gone from the delivery. `no_drill.py` now
builds the NO_DRILL positions from the hyperbola detections itself, field for
field as before, and a hyperbola result whose `status` is not `ok` is an
invalid constraint (not an empty one).

**Global JSON** (`sensors/global_json.py`). The builder is used as delivered
-- `build_element` per wall -- and writes `processed/session_<stamp>/
gpr_hyperspectral.json` in its compact layout. What the delivery leaves `null`
for the robot software is filled in: B-scan start/end and hyperbola apex in
the map frame (by `gpr.map_transform`, the same transform as the NO_DRILL
zones), each spectrum's timestamp and map pose, and `row` /
`data_pointer_csv` pointing at a copy of `reflectance.csv` (the acquisition
with the poses) instead of the adapter's `input.csv`. Elements are
`wall_NN`, plus `unassociated` for scans that matched no line. Coordinate
errors stay `null`. For the pointers to hold, each GPR scan folder now uses
the delivery's own layout -- `.sgy` + `.csv` next to `hyperbola/` and `line/`
(previously `hyperbolae/` and `lines/`, with no acquisition copy).

## GPR v2.1 -> v4

Additive for the FSM: the single-scan entry points the adapters call keep their
signature and their result keys, so `sensors/gpr.py` is unchanged.

The delivery gains recursive folder processing -- `gpr_integration.run_gpr_batch`
/ `run_gpr_path`, `line_integration.run_line_batch` / `run_line_path`, and a
top-level `run_gpr_processing.py` that runs both branches over a directory tree.
The FSM does not use them: it walks the incoming folder itself (`find_scan_files`
+ the per-scan registry), associates every scan with the line it was acquired on,
and processes only what is new, which the vendor's batch output layout has no
notion of. They are vendored because `line_integration/__init__.py` imports
`batch` at module import time, and because keeping the tree byte-identical to the
delivery is what makes the next diff cheap.

The other v4 change is upstream's own fix for the empty-detection crash that was
patched locally in v2.1 (see below).

## POKEYE v1 -> v2 (superseded by the section above)

Purely additive on the vendor side; the HSI trigger policy is unchanged.
`decide_pokeye` gained a keyword-only `gpr_result=` and a new public
`build_gpr_drilling_constraints(gpr_result)` that turns hyperbola detections
into `NO_DRILL` positions. GPR still does **not** trigger POKEYE: a confident
HSI classification returns `pokeye_required=false` while still carrying the
constraints, because POKEYE may drill later for another reason (including
externally requested RANDOM drilling).

Everything the delivered README leaves to software lives in
`sensors/no_drill.py`: the local B-scan `x_m` -> map-frame transform, the
project exclusion tolerance the sensor package refuses to invent, and the
rejection of candidate drill targets that fall inside a zone.

One project rule there goes beyond the delivery: a drill target must also lie
on a GPR line that was scanned *and* analysed. The sensor package has nothing
to say about unscanned wall — it only reports what it found where it looked —
so treating "no B-scan" as "not drillable" is the FSM's decision, not the
vendor's, and it is what makes the coverage half of `no_drill.py` ours to
maintain across future deliveries.

## Not copied

- `Hyperbola_Segmentation/models/best.pt` (241 MB) -> `task_planner_fsm/models/gpr/best.pt`
  (identical in the FULL PIPELINE delivery)
- `benjamin_original/classifier.joblib` (117 kB) -> `task_planner_fsm/models/hsi/classifier.joblib`
- each module's `test_data/` and `test_output/` (demo inputs and outputs), except
  `Pokeye Decision/test_data/` (32 kB of example messages), which
  `test/test_sensor_pokeye.py` checks the policy against
- the delivery `.zip` archives and the hyperspectral requirements PDF
- `POKEYE_DECISION_pipeline_v2/README_v1_original.md`: the previous version's
  README, shipped alongside the current one. Git already holds every earlier
  version of this folder, so a superseded copy on disk is one more thing that
  can be read by mistake. Same for any future delivery: vendor what the package
  runs on today, and let `git log` answer what it used to be.

The delivered `config.yaml` / `config.json` files still point at the original
model locations. The adapters in `task_planner_fsm/sensors/` override those
paths explicitly (`weights_path=` for the GPR, a runtime copy of the HSI config
with an absolute `model_path`), so the vendor defaults are never used.

## Local modifications

Kept to the minimum and marked with a `DISCOVER FSM integration` comment so a
newer delivery can be diffed against them:

- `gpr_pipeline/Hyperbola_Segmentation/gpr_integration/config.yaml`:
  `inference.device` is `null` (CUDA when torch sees it) instead of the
  delivered `cpu`. The team pinned the CPU because cuDNN fails to load on
  their lab PC; the Jetson's CUDA works and runs the delivered test scan in
  ~20 s instead of ~80 s, with the same 14 hyperbolae.

The HSI `relative_to` patch carried since v2 is gone: the v2 integration no
longer writes `run_manifest.json`, which was the only place it was needed.

## Fixed upstream in v4

- `gpr_pipeline/Hyperbola_Segmentation/tahzeeb_original/01_consolidate.py`:
  `nms()` ended in `pd.concat(kept_rows)`, which raises
  `ValueError: No objects to concatenate` when the scan has no detections
  (header-only `detections.csv`), so a clean B-scan failed the hyperbola
  pipeline -- and, through the FSM's "scanned *and* analysed" rule, the
  cleanest lines on the wall became the ones nothing could be drilled on.
  Seen on 3 of 10 lines of the 2026-09-17 wall and reported to Tahzeeb; v2.1
  carried a local patch here. v4 handles it in the integration layer instead
  (`gpr_integration/postprocess_adapter.py`, `_consolidate_safely`: it catches
  exactly that `ValueError`, returns an empty consolidated frame and records
  `audit.empty_detection_reason`), leaving Tahzeeb's source untouched, so the
  local patch is dropped and this file is back to the delivered version.

## Internal layout is load-bearing

`Hyperbola_Segmentation` and `Line_Segmentation` import `../GPRTools`;
`hsi_integration` loads `../benjamin_original/classifier.py` by path; `gpr_integration/config.yaml`
references `../tahzeeb_original`. Keep the three project trees exactly as they
are inside their folder.
