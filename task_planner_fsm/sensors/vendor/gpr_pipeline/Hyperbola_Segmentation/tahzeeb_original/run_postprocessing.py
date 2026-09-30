"""Run the complete GPR post-processing pipeline (steps 1-4) in one call.

    python run_postprocessing.py --config config.yaml

Creates results/<timestamp>/ containing:
    consolidated.csv  objects.csv  element_summary.csv
    gpr_data.json     material_passport.csv

All random behaviour is seeded for reproducibility.
"""
from __future__ import annotations

import argparse
import random

import numpy as np

import importlib

from gpr_pp_utils import load_config, load_detections, make_results_dir

step1 = importlib.import_module("01_consolidate")
step2 = importlib.import_module("02_hyperbola_fit")
step3 = importlib.import_module("03_quantify")
step2b = importlib.import_module("02b_dedup")
step4 = importlib.import_module("04_export")
step5 = importlib.import_module("05_visualize")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)

    seed = int(cfg["io"].get("seed", 42))
    random.seed(seed)
    np.random.seed(seed)

    out = make_results_dir(cfg["io"]["results_dir"])
    print(f"=== GPR post-processing  (results -> {out}) ===")

    raw = load_detections(cfg["io"]["detections_csv"])
    print(f"[run] input detections: {len(raw)}")

    # Step 1
    cons = step1.consolidate(cfg)
    cons_path = out / "consolidated.csv"
    cons.to_csv(cons_path, index=False)
    print(f"[run] step1 consolidate -> {len(cons)} ({cons_path.name})")

    # Step 2
    obj = step2.compute(cfg, cons)
    obj_path = out / "objects.csv"
    obj.to_csv(obj_path, index=False)
    print(f"[run] step2 physical    -> {len(obj)} ({obj_path.name})")

    # Step 2b - duplicate-hyperbola removal
    obj, removed = step2b.run(cfg, obj)
    obj.to_csv(out / "objects_dedup.csv", index=False)
    removed.to_csv(out / "objects_dedup_removed.csv", index=False)
    print(f"[run] step2b dedup      -> {len(obj)} kept ({len(removed)} removed)")

    # Step 3
    summ = step3.quantify(cfg, obj)
    summ_path = out / "element_summary.csv"
    summ.to_csv(summ_path, index=False)
    print(f"[run] step3 quantify    -> {len(summ)} elements ({summ_path.name})")

    # Step 4
    import json
    viewer = step4.build_viewer_json(obj, summ, cfg)
    (out / "gpr_data.json").write_text(json.dumps(viewer, indent=2), encoding="utf-8")
    summ[["element", "type", "interpretation", "n_objects", "n_layers",
          "bar_pitch_cm", "cover_min_cm", "area_m2",
          "recoverable_steel_kg", "value_eur", "co2_avoided_kg"]].to_csv(
        out / "material_passport.csv", index=False)
    print(f"[run] step4 export      -> gpr_data.json, material_passport.csv")

    # Step 5 - visualisation overlays
    try:
        import importlib as _il; _il.reload(step5)
        step5_objs = out / "objects_dedup.csv"
        import subprocess  # run as standalone to reuse its file IO
        viz = out / "viz"
        step5.main.__wrapped__ if False else None
        # call directly
        import pandas as _pd
        _obj = _pd.read_csv(step5_objs)
        # emulate CLI args
        import sys as _sys
        _argv = _sys.argv
        _sys.argv = ["05_visualize.py","--config",args.config,"--objects",str(step5_objs),"--outdir",str(viz)]
        step5.main(); _sys.argv=_argv
        print(f"[run] step5 visualise   -> {viz}")
    except Exception as e:
        print(f"[run] step5 visualise skipped: {e}")

    print("\n=== SITE TOTALS ===")
    print(f"  recoverable steel : {summ['recoverable_steel_kg'].sum():.0f} kg")
    print(f"  indicative value  : EUR {summ['value_eur'].sum():.0f}")
    print(f"  CO2 avoided       : {summ['co2_avoided_kg'].sum()/1000:.2f} t")
    print(f"  objects (consolidated): {len(obj)}  from {len(raw)} raw")
    print(f"\nAll outputs in: {out}")


if __name__ == "__main__":
    main()
