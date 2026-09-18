# Vendored sensor pipelines

Code delivered by the DISCOVER/OLIWALL sensor team, copied here **unmodified**
(the only additions are empty `__init__.py` files so `find_packages()` ships the
tree on a non-symlink install). The FSM never imports these by their dotted
path: `task_planner_fsm.sensors` puts each project root on `sys.path` and uses
the public entry points exactly as the delivered READMEs describe.

| Folder here | Delivered as | Entry point |
|---|---|---|
| `gpr_pipeline/` | `GPR_DISCOVER_PIPELINE_v4` | `gpr_integration.run_gpr_pipeline`, `line_integration.run_line_pipeline` |
| `hsi_pipeline/` | `HYPERSPECTRAL_DISCOVER_pipeline_v2` | `hsi_integration.run_hyperspectral_pipeline` |
| `pokeye_decision_pipeline/` | `POKEYE_DECISION_pipeline_v2` | `pokeye_decision.decide_pokeye`, `pokeye_decision.build_gpr_drilling_constraints` |

Version suffixes are dropped from the folder names so a newer delivery can be
dropped in without touching any import path; record the new version in the
table above when you do.

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

## POKEYE v1 -> v2

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
- `benjamin_original/classifier.joblib` (27 MB) -> `task_planner_fsm/models/hsi/classifier.joblib`
- `benjamin_original/dataset_LENZ.csv`, `Recap.pdf` (training material, unused at runtime)
- `HYPERSPECTRAL_DISCOVER_pipeline_v2/example_output/` (3 MB of demo output)
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

- `hsi_pipeline/hsi_integration/pipeline.py`: `run_manifest["model"]` used
  `model_path.relative_to(PROJECT_ROOT)`, which raises when the classifier is
  outside the project (it lives in `task_planner_fsm/models/hsi/`). It now
  records the absolute path in that case.

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
`hsi_integration` imports `../benjamin_original`; `gpr_integration/config.yaml`
references `../tahzeeb_original`. Keep the three project trees exactly as they
are inside their folder.
