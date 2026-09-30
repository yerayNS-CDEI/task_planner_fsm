from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

import pandas as pd


def tile_one_gain(
    image_path: Path,
    output_dir: Path,
    tahzeeb_dir: Path,
    cfg: dict[str, Any],
) -> dict[str, Any]:
    """Run Tahzeeb's unmodified gpr_tiler_v9.py on one full B-scan."""
    output_dir.mkdir(parents=True, exist_ok=True)
    script = tahzeeb_dir / "gpr_tiler_v9.py"
    if not script.exists():
        raise FileNotFoundError(script)

    extra_args = [str(v) for v in cfg.get("tiling", {}).get("extra_args", [])]
    cmd = [sys.executable, str(script), "--source", str(image_path), "--output", str(output_dir)] + extra_args
    proc = subprocess.run(cmd, capture_output=True, text=True)
    (output_dir / "tiler_stdout.txt").write_text(proc.stdout or "", encoding="utf-8")
    (output_dir / "tiler_stderr.txt").write_text(proc.stderr or "", encoding="utf-8")
    if proc.returncode != 0:
        raise RuntimeError(f"gpr_tiler_v9.py failed for {image_path.name}:\n{proc.stderr}\n{proc.stdout}")

    manifests = sorted(output_dir.glob("crop_manifest_*.csv"))
    if len(manifests) != 1:
        raise RuntimeError(f"Expected exactly one crop_manifest_*.csv in {output_dir}, found {len(manifests)}")
    manifest_path = manifests[0]
    mdf = pd.read_csv(manifest_path)
    if mdf.empty:
        raise RuntimeError(f"Tiler produced no kept tiles for {image_path.name}")

    qcs = sorted(output_dir.glob("crop_qc_*.csv"))
    return {
        "tiles_dir": output_dir,
        "crop_manifest": manifest_path,
        "crop_qc": qcs[0] if qcs else None,
        "n_tiles": int(len(mdf)),
        "source_w": int(mdf.iloc[0]["source_w"]),
        "source_h": int(mdf.iloc[0]["source_h"]),
        "work_h": int(mdf.iloc[0]["work_h"]),
        "vscale": float(mdf.iloc[0].get("vscale", 1.0)),
        "tile_size": int(mdf.iloc[0]["tile_w"]),
    }
