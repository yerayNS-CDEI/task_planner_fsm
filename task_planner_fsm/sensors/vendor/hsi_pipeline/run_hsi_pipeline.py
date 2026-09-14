#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json

from hsi_integration.pipeline import run_hyperspectral_pipeline


def main() -> int:
    p = argparse.ArgumentParser(description="DISCOVER hyperspectral material-classification pipeline")
    p.add_argument("input", help="Sensor export (.xls text export) or CSV containing spectra")
    p.add_argument("--output-dir", default=None, help="Optional output directory")
    p.add_argument("--config", default=None, help="Optional integration config JSON")
    args = p.parse_args()

    result = run_hyperspectral_pipeline(args.input, args.output_dir, args.config)
    compact = {
        "sensor": result["sensor"],
        "input_file": result["input_file"],
        "n_samples": result["n_samples"],
        "n_detected": result["n_detected"],
    }
    if result["n_samples"] == 1:
        compact.update({k: result.get(k) for k in ["detected", "material", "confidence", "status"]})
    print(json.dumps(compact, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
