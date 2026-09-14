from __future__ import annotations

import argparse
import json
from pathlib import Path

from line_integration import run_line_pipeline


def main() -> int:
    p = argparse.ArgumentParser(description="DISCOVER Task 1b - GPR horizontal line segmentation")
    p.add_argument("sgy", type=Path, help="Input GP8800 .sgy/.segy (sidecar .csv must be alongside it)")
    p.add_argument("--output", type=Path, default=None, help="Optional output directory")
    p.add_argument("--config", type=Path, default=None, help="Optional integration config.yaml")
    a = p.parse_args()
    result = run_line_pipeline(a.sgy, output_dir=a.output, config_path=a.config)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
