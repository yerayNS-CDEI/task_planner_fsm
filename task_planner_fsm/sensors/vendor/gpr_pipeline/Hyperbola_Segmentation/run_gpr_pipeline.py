#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from gpr_integration.batch import run_gpr_path


def main() -> int:
    p = argparse.ArgumentParser(
        description="DISCOVER / OLIWALL GPR hyperbola pipeline - one SEGY or recursive folder"
    )
    p.add_argument(
        "input_path", type=Path,
        help="GP8800 .sgy/.segy file OR a directory to scan recursively"
    )
    p.add_argument("--config", type=Path, default=None, help="Integration config YAML")
    p.add_argument(
        "--output-dir", type=Path, default=None,
        help=(
            "Single file: output root used by the scan pipeline. "
            "Directory: batch output root; input subfolder structure is preserved."
        ),
    )
    p.add_argument("--weights", type=Path, default=None, help="Tahzeeb Mask R-CNN ResNet101 weights")
    p.add_argument("--tahzeeb-config", type=Path, default=None, help="Tahzeeb exact post-processing config.yaml")
    args = p.parse_args()

    result = run_gpr_path(
        input_path=args.input_path,
        config_path=args.config,
        output_dir=args.output_dir,
        weights_path=args.weights,
        tahzeeb_config_path=args.tahzeeb_config,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
