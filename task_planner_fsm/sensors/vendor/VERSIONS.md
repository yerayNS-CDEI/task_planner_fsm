# Vendored sensor pipelines

Code delivered by the DISCOVER/OLIWALL sensor team, copied here **unmodified**
(the only additions are empty `__init__.py` files so `find_packages()` ships the
tree on a non-symlink install). The FSM never imports these by their dotted
path: `task_planner_fsm.sensors` puts each project root on `sys.path` and uses
the public entry points exactly as the delivered READMEs describe.

| Folder here | Delivered as | Entry point |
|---|---|---|
| `gpr_pipeline/` | `GPR_DISCOVER_robot_pipeline_v2_1` | `gpr_integration.run_gpr_pipeline`, `line_integration.run_line_pipeline` |
| `hsi_pipeline/` | `HYPERSPECTRAL_DISCOVER_pipeline_v2` | `hsi_integration.run_hyperspectral_pipeline` |
| `pokeye_decision_pipeline/` | `POKEYE_DECISION_pipeline_v1` | `pokeye_decision.decide_pokeye` |

Version suffixes are dropped from the folder names so a newer delivery can be
dropped in without touching any import path; record the new version in the
table above when you do.

## Not copied

- `Hyperbola_Segmentation/models/best.pt` (241 MB) -> `task_planner_fsm/models/gpr/best.pt`
- `benjamin_original/classifier.joblib` (27 MB) -> `task_planner_fsm/models/hsi/classifier.joblib`
- `benjamin_original/dataset_LENZ.csv`, `Recap.pdf` (training material, unused at runtime)
- `HYPERSPECTRAL_DISCOVER_pipeline_v2/example_output/` (3 MB of demo output)
- the delivery `.zip` archives and the hyperspectral requirements PDF

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

## Internal layout is load-bearing

`Hyperbola_Segmentation` and `Line_Segmentation` import `../GPRTools`;
`hsi_integration` imports `../benjamin_original`; `gpr_integration/config.yaml`
references `../tahzeeb_original`. Keep the three project trees exactly as they
are inside their folder.
