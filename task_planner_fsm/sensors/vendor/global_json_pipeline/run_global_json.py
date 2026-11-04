#!/usr/bin/env python3
"""Run GPR + hyperspectral and write the GPR/HSI part of the global OLIWALL JSON.

    python run_global_json.py                         # test_data/elements.json -> test_output/
    python run_global_json.py path/to/elements.json --output-root path/to/outputs
    python run_global_json.py --reuse                 # rebuild the JSON from existing results

elements.json says which acquisitions belong to each inspected element (paths relative
to the manifest):

    {"elements": {"column1": {"gpr_scans": ["a.sgy", "b.sgy"],
                              "hyperspectral_session": "session_20261002_131059"}}}

Output:
    <output-root>/
    ├── gpr_hyperspectral.json          global JSON (robot fields left null)
    └── <element>/
        ├── gpr/<scan>/                 run_gpr.py outputs (sgy + csv, hyperbola/, line/)
        └── hyperspectral/<session>/    run_hyperspectral_pipeline outputs
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

CODES = Path(__file__).resolve().parent
MODULE_ROOT = CODES.parent
PIPELINE_ROOT = MODULE_ROOT.parent
for p in (CODES, PIPELINE_ROOT / "GPR" / "codes", PIPELINE_ROOT / "GPR" / "codes" / "Hyperbola_Segmentation",
          PIPELINE_ROOT / "Hyperspectral" / "codes"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from global_json import build_element, dumps_compact, load_config  # noqa: E402
from gpr_integration.utils import portable_scan_stem  # noqa: E402


def _resolve(base: Path, p: str) -> Path:
    q = Path(p)
    return (q if q.is_absolute() else base / q).resolve()


def _session_name(path: Path) -> str:
    return path.parent.name if path.is_file() else path.name


def main() -> int:
    p = argparse.ArgumentParser(description="DISCOVER / OLIWALL: global JSON (GPR + hyperspectral part)")
    p.add_argument("manifest", nargs="?", type=Path, default=MODULE_ROOT / "test_data" / "elements.json")
    p.add_argument("--output-root", type=Path, default=MODULE_ROOT / "test_output")
    p.add_argument("--config", type=Path, default=None, help="global_json config (default: global_json/config.json)")
    p.add_argument("--reuse", action="store_true",
                   help="do not rerun the sensor pipelines; build the JSON from results already in --output-root")
    args = p.parse_args()
    for stream in (sys.stdout, sys.stderr):
        stream.reconfigure(encoding="utf-8", errors="replace")

    cfg = load_config(args.config)
    manifest_path = args.manifest.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    out_root = args.output_root.resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    if not args.reuse:
        from hsi_integration import run_hyperspectral_pipeline
        from run_gpr import run_scan

    document = {}
    for element_id, spec in manifest["elements"].items():
        el_dir = out_root / element_id
        scan_dirs = []
        for sgy in spec.get("gpr_scans", []):
            sgy = _resolve(manifest_path.parent, sgy)
            if not args.reuse:
                run_scan(sgy, el_dir / "gpr")
            scan_dirs.append(el_dir / "gpr" / portable_scan_stem(sgy))

        hsi_dir = None
        if spec.get("hyperspectral_session"):
            session = _resolve(manifest_path.parent, spec["hyperspectral_session"])
            hsi_dir = el_dir / "hyperspectral" / _session_name(session)
            if not args.reuse:
                run_hyperspectral_pipeline(session, output_dir=hsi_dir)

        document[element_id] = build_element(scan_dirs, hsi_dir, out_root, cfg)
        block = document[element_id][0]
        print(json.dumps({
            "element": element_id,
            "continuous_gpr": len(block["continuous_gpr"]),
            "discrete_gpr": len(block["discrete_gpr"]),
            "hyperspectral_measurements": len(block["hyperspectral_camera"]["measurements"]),
        }))

    out_json = out_root / cfg.get("output_filename", "gpr_hyperspectral.json")
    out_json.write_text(dumps_compact(document) + "\n", encoding="utf-8")
    print(f"written: {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
