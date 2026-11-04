#!/usr/bin/env python3
"""Run both GPR branches (hyperbola + line segmentation) on GP8800 scans.

    python run_gpr.py                      # every scan in GPR/test_data/ -> GPR/test_output/
    python run_gpr.py path/to/scan.sgy     # one scan (its .csv sidecar must be next to it)
    python run_gpr.py folder/ --output-root path/to/outputs

Per scan:
    <output-root>/<scan>/
    ├── <scan>.sgy + <scan>.csv   unmodified acquisition (GP8800 / Screening Eagle)
    ├── hyperbola/                Hyperbola_Segmentation deliverables
    └── line/                     Line_Segmentation deliverables

The global JSON for Tecnalia is built from these folders by
FULL PIPELINE/Global JSON/codes/run_global_json.py.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import traceback
from pathlib import Path
from typing import Any

CODES = Path(__file__).resolve().parent
GPR_ROOT = CODES.parent
for sub in ("Hyperbola_Segmentation", "Line_Segmentation"):
    if str(CODES / sub) not in sys.path:
        sys.path.insert(0, str(CODES / sub))

from gpr_integration import run_gpr_pipeline  # noqa: E402
from gpr_integration.utils import portable_scan_stem  # noqa: E402
from line_integration import run_line_pipeline  # noqa: E402


def _scans(inputs: list[Path]) -> list[Path]:
    out: list[Path] = []
    for p in inputs:
        out += sorted(p.glob("*.sgy")) if p.is_dir() else [p]
    return out


def run_scan(sgy: Path, output_root: Path) -> dict[str, Any]:
    """Process one scan with both branches. A failing branch does not stop the other one."""
    sgy = sgy.resolve()
    scan_dir = output_root / portable_scan_stem(sgy)
    scan_dir.mkdir(parents=True, exist_ok=True)
    for src in (sgy, sgy.with_suffix(".csv")):
        if src.exists() and (scan_dir / src.name).resolve() != src:
            shutil.copy2(src, scan_dir / src.name)

    result: dict[str, Any] = {"scan": sgy.name, "folder": str(scan_dir)}
    for branch, run in (("hyperbola", run_gpr_pipeline), ("line", run_line_pipeline)):
        try:
            result[branch] = run(sgy, output_dir=scan_dir / branch)
        except Exception as exc:  # noqa: BLE001 - reported per branch
            result[branch] = {"status": "error", "error": f"{type(exc).__name__}: {exc}"}
            traceback.print_exc()
    return result


def main() -> int:
    p = argparse.ArgumentParser(description="DISCOVER / OLIWALL GPR: hyperbola + line segmentation")
    p.add_argument("inputs", nargs="*", type=Path, default=[GPR_ROOT / "test_data"],
                   help=".sgy files or folders of .sgy files (default: GPR/test_data/)")
    p.add_argument("--output-root", type=Path, default=GPR_ROOT / "test_output")
    args = p.parse_args()
    # Tahzeeb's scripts print non-ASCII characters; avoid UnicodeEncodeError on Windows consoles.
    for stream in (sys.stdout, sys.stderr):
        stream.reconfigure(encoding="utf-8", errors="replace")

    failed = 0
    for sgy in _scans(args.inputs):
        r = run_scan(sgy, args.output_root.resolve())
        hyp, line = r["hyperbola"], r["line"]
        failed += sum(1 for b in (hyp, line) if b.get("status") == "error")
        print(json.dumps({
            "scan": r["scan"],
            "folder": r["folder"],
            "hyperbola": hyp.get("error") or f"{hyp.get('n_valid_detections')} hyperbola(s)",
            "line": line.get("error") or f"{line.get('n_lines')} line(s)",
        }, ensure_ascii=False))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
