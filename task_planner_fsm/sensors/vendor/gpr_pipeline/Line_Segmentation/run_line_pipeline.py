from __future__ import annotations

import argparse
import json
from pathlib import Path

from line_integration.batch import run_line_path


def main() -> int:
    p = argparse.ArgumentParser(
        description="DISCOVER Task 1b - one GP8800 SEGY or recursive folder"
    )
    p.add_argument(
        "input_path", type=Path,
        help="Input GP8800 .sgy/.segy OR directory to scan recursively; each scan needs a same-basename .csv"
    )
    p.add_argument(
        "--output", type=Path, default=None,
        help=(
            "Single file: exact output directory. "
            "Directory: batch output root; input subfolder structure is preserved."
        ),
    )
    p.add_argument("--config", type=Path, default=None, help="Optional integration config.yaml")
    a = p.parse_args()
    result = run_line_path(a.input_path, output_dir=a.output, config_path=a.config)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
