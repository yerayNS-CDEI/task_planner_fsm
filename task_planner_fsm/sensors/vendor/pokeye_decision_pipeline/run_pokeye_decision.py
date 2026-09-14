from __future__ import annotations

import argparse
import json
from pathlib import Path

from pokeye_decision import decide_from_json_file


def main() -> int:
    ap = argparse.ArgumentParser(
        description="DISCOVER: decide whether POKEYE is required from an HSI result JSON."
    )
    ap.add_argument("input_json", help="Single HSI message JSON or hsi_result.json")
    ap.add_argument("--output", default=None, help="Output decision JSON path")
    ap.add_argument("--config", default=None, help="Optional decision config JSON")
    args = ap.parse_args()

    input_path = Path(args.input_json)
    output = Path(args.output) if args.output else Path("outputs") / input_path.stem / "pokeye_decision.json"
    result = decide_from_json_file(input_path, output_json=output, config_path=args.config)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"\nWritten -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
