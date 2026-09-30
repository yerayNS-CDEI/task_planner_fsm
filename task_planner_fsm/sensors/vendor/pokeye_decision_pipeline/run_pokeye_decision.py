from __future__ import annotations

import argparse
import json
from pathlib import Path

from pokeye_decision import decide_from_json_file


def main() -> int:
    ap = argparse.ArgumentParser(
        description="DISCOVER: HSI-based POKEYE decision with optional GPR no-drill constraints."
    )
    ap.add_argument("input_json", help="Single HSI message JSON or hsi_result.json")
    ap.add_argument(
        "--gpr-json",
        default=None,
        help="Optional gpr_result.json. Hyperbolae become NO_DRILL constraints; they do not trigger POKEYE.",
    )
    ap.add_argument("--output", default=None, help="Output decision JSON path")
    ap.add_argument("--config", default=None, help="Optional decision config JSON")
    args = ap.parse_args()

    input_path = Path(args.input_json)
    output = Path(args.output) if args.output else Path("outputs") / input_path.stem / "pokeye_decision.json"
    result = decide_from_json_file(
        input_path,
        gpr_json=args.gpr_json,
        output_json=output,
        config_path=args.config,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"\nWritten -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
