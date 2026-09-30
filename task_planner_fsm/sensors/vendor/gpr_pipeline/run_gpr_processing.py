#!/usr/bin/env python3
"""Top-level DISCOVER GPR runner.

Processes either one GP8800 SEGY or an entire directory recursively. By default
it runs both interpretation branches: hyperbola segmentation and line segmentation.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent
HYP_DIR = ROOT / "Hyperbola_Segmentation"
LINE_DIR = ROOT / "Line_Segmentation"
for folder in (HYP_DIR, LINE_DIR):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from gpr_integration import run_gpr_batch, run_gpr_pipeline  # noqa: E402
from line_integration import run_line_batch, run_line_pipeline  # noqa: E402


def _default_output(input_path: Path) -> Path:
    label = input_path.name if input_path.is_dir() else input_path.stem
    return ROOT / "outputs" / label


def _rows_by_key(summary: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if not summary:
        return {}
    return {str(row.get("relative_sgy", "")): row for row in summary.get("scans", [])}


def _write_combined_summary(
    output_root: Path,
    input_root: Path,
    hyper: dict[str, Any] | None,
    line: dict[str, Any] | None,
    mode: str,
) -> dict[str, Any]:
    h = _rows_by_key(hyper)
    l = _rows_by_key(line)
    keys = sorted(set(h) | set(l))
    rows: list[dict[str, Any]] = []
    for key in keys:
        hr = h.get(key, {})
        lr = l.get(key, {})
        rows.append({
            "relative_sgy": key,
            "sidecar_csv": hr.get("sidecar_csv") or lr.get("sidecar_csv") or "",
            "hyperbola_status": hr.get("status", "NOT_RUN"),
            "n_hyperbolas": hr.get("n_valid_detections", ""),
            "hyperbola_output_dir": hr.get("output_dir", ""),
            "hyperbola_error": hr.get("error", ""),
            "line_status": lr.get("status", "NOT_RUN"),
            "n_lines": lr.get("n_lines", ""),
            "line_output_dir": lr.get("output_dir", ""),
            "line_error": lr.get("error", ""),
        })

    fields = [
        "relative_sgy", "sidecar_csv",
        "hyperbola_status", "n_hyperbolas", "hyperbola_output_dir", "hyperbola_error",
        "line_status", "n_lines", "line_output_dir", "line_error",
    ]
    with (output_root / "batch_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "pipeline": "GPR_DISCOVER_PIPELINE_v4",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": mode,
        "input_root": str(input_root),
        "output_root": str(output_root),
        "hyperbola_summary": None if hyper is None else {
            k: hyper.get(k) for k in ("total_sgy_found", "ok", "skipped_missing_sidecar", "errors")
        },
        "line_summary": None if line is None else {
            k: line.get(k) for k in ("total_sgy_found", "ok", "skipped_missing_sidecar", "errors")
        },
        "scans": rows,
    }
    with (output_root / "batch_summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    return summary


def run_directory(input_path: Path, output_root: Path, mode: str) -> dict[str, Any]:
    output_root.mkdir(parents=True, exist_ok=True)
    hyper = None
    line = None
    if mode in {"both", "hyperbola"}:
        hyper = run_gpr_batch(
            input_path,
            output_root=output_root / "Hyperbola_Segmentation",
        )
    if mode in {"both", "line"}:
        line = run_line_batch(
            input_path,
            output_root=output_root / "Line_Segmentation",
        )
    return _write_combined_summary(output_root, input_path, hyper, line, mode)


def run_single(input_path: Path, output_root: Path, mode: str) -> dict[str, Any]:
    output_root.mkdir(parents=True, exist_ok=True)
    result: dict[str, Any] = {
        "pipeline": "GPR_DISCOVER_PIPELINE_v4",
        "mode": mode,
        "input": str(input_path),
        "output_root": str(output_root),
    }
    if mode in {"both", "hyperbola"}:
        try:
            hyp = run_gpr_pipeline(
                input_path,
                output_dir=output_root / "Hyperbola_Segmentation",
            )
            result["hyperbola"] = {
                "status": "OK",
                "hyperbola_detected": hyp.get("hyperbola_detected"),
                "n_valid_detections": hyp.get("n_valid_detections"),
            }
        except Exception as exc:
            result["hyperbola"] = {"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}
    if mode in {"both", "line"}:
        try:
            line_out = output_root / "Line_Segmentation" / input_path.stem
            lin = run_line_pipeline(input_path, output_dir=line_out)
            result["line"] = {
                "status": "OK",
                "detected": lin.get("detected"),
                "n_lines": lin.get("n_lines"),
            }
        except Exception as exc:
            result["line"] = {"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}
    with (output_root / "run_summary.json").open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    return result


def main() -> int:
    p = argparse.ArgumentParser(
        description="DISCOVER GPR v4 - process one scan or recursively process an acquisition folder"
    )
    p.add_argument("input_path", type=Path, help=".sgy/.segy file OR directory")
    p.add_argument("--output-root", type=Path, default=None, help="Runtime output root")
    p.add_argument(
        "--mode", choices=["both", "hyperbola", "line"], default="both",
        help="Which GPR interpretation branch to run (default: both)",
    )
    a = p.parse_args()
    inp = a.input_path.expanduser().resolve()
    if not inp.exists():
        raise FileNotFoundError(inp)
    out = a.output_root.expanduser().resolve() if a.output_root else _default_output(inp).resolve()
    if inp.is_dir():
        summary = run_directory(inp, out, a.mode)
    else:
        summary = run_single(inp, out, a.mode)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
