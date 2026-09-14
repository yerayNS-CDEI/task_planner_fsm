#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from gpr_integration import run_gpr_pipeline


def main() -> int:
    p = argparse.ArgumentParser(description="DISCOVER / OLIWALL GPR processing pipeline")
    p.add_argument("input_sgy", type=Path, help="GP8800 .sgy file (matching .csv sidecar required)")
    p.add_argument("--config", type=Path, default=None, help="Integration config YAML")
    p.add_argument("--output-dir", type=Path, default=None, help="Override output root")
    p.add_argument("--weights", type=Path, default=None, help="Tahzeeb Mask R-CNN ResNet101 weights")
    p.add_argument("--tahzeeb-config", type=Path, default=None, help="Tahzeeb exact post-processing config.yaml")
    args = p.parse_args()
    result = run_gpr_pipeline(
        input_sgy=args.input_sgy,
        config_path=args.config,
        output_dir=args.output_dir,
        weights_path=args.weights,
        tahzeeb_config_path=args.tahzeeb_config,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
