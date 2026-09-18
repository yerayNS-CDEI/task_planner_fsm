from __future__ import annotations

import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SUPPORTED_EXTENSIONS = {".sgy", ".segy"}


def discover_scans(input_path: str | Path) -> tuple[Path, list[Path]]:
    path = Path(input_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)

    if path.is_file():
        if path.suffix.lower() not in SUPPORTED_EXTENSIONS:
            raise ValueError(f"Expected .sgy/.segy or a directory, got: {path}")
        return path.parent, [path]

    scans = sorted(
        p for p in path.rglob("*")
        if p.is_file() and p.suffix.lower() in SUPPORTED_EXTENSIONS
    )
    return path, scans


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "relative_sgy", "sidecar_csv", "status", "lines_detected",
        "n_lines", "output_dir", "error",
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})


def run_line_batch(
    input_path: str | Path,
    output_root: str | Path | None = None,
    config_path: str | Path | None = None,
) -> dict[str, Any]:
    """Recursively process every GP8800 SEGY under a directory."""
    input_root, scans = discover_scans(input_path)
    if Path(input_path).expanduser().resolve().is_file():
        raise ValueError("run_line_batch expects a directory. Use run_line_pipeline for one file.")

    project_root = Path(__file__).resolve().parents[1]
    batch_root = (
        Path(output_root).expanduser().resolve()
        if output_root is not None
        else (project_root / "outputs" / input_root.name).resolve()
    )
    batch_root.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    for idx, scan in enumerate(scans, start=1):
        rel = scan.relative_to(input_root)
        sidecar = scan.with_suffix(".csv")
        scan_output_dir = batch_root / rel.parent / scan.stem

        print(f"[line {idx}/{len(scans)}] {rel.as_posix()}")

        if not sidecar.exists():
            rows.append({
                "relative_sgy": rel.as_posix(),
                "sidecar_csv": "",
                "status": "SKIPPED_MISSING_SIDECAR",
                "lines_detected": "",
                "n_lines": "",
                "output_dir": str(scan_output_dir),
                "error": f"Missing sidecar: {sidecar.name}",
            })
            print(f"  -> skipped: missing {sidecar.name}")
            continue

        try:
            from .pipeline import run_line_pipeline
            result = run_line_pipeline(
                input_sgy=scan,
                output_dir=scan_output_dir,
                config_path=config_path,
            )
            rows.append({
                "relative_sgy": rel.as_posix(),
                "sidecar_csv": sidecar.name,
                "status": "OK",
                "lines_detected": bool(result.get("detected", False)),
                "n_lines": int(result.get("n_lines", 0)),
                "output_dir": str(scan_output_dir),
                "error": "",
            })
            print(f"  -> OK: {result.get('n_lines', 0)} lines")
        except Exception as exc:
            rows.append({
                "relative_sgy": rel.as_posix(),
                "sidecar_csv": sidecar.name,
                "status": "ERROR",
                "lines_detected": "",
                "n_lines": "",
                "output_dir": str(scan_output_dir),
                "error": f"{type(exc).__name__}: {exc}",
            })
            print(f"  -> ERROR: {type(exc).__name__}: {exc}")

    counts = {
        "total_sgy_found": len(scans),
        "ok": sum(r["status"] == "OK" for r in rows),
        "skipped_missing_sidecar": sum(r["status"] == "SKIPPED_MISSING_SIDECAR" for r in rows),
        "errors": sum(r["status"] == "ERROR" for r in rows),
    }
    summary = {
        "pipeline": "DISCOVER_GPR_Line_Batch_v4",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "input_root": str(input_root),
        "output_root": str(batch_root),
        **counts,
        "scans": rows,
    }
    _write_csv(batch_root / "batch_summary.csv", rows)
    with (batch_root / "batch_summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    return summary


def run_line_path(
    input_path: str | Path,
    output_dir: str | Path | None = None,
    config_path: str | Path | None = None,
) -> dict[str, Any]:
    path = Path(input_path).expanduser().resolve()
    if path.is_dir():
        return run_line_batch(path, output_root=output_dir, config_path=config_path)
    from .pipeline import run_line_pipeline
    return run_line_pipeline(path, output_dir=output_dir, config_path=config_path)
