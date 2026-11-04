from pathlib import Path
import json

root = Path(__file__).resolve().parent
config = root / "pokeye_decision" / "config.json"
module = root / "pokeye_decision" / "decision.py"
print("POKEYE decision setup")
print(f"  decision.py : {'OK' if module.exists() else 'MISSING'}")
print(f"  config.json : {'OK' if config.exists() else 'MISSING'}")
if config.exists():
    data = json.loads(config.read_text(encoding='utf-8'))
    print(f"  HSI threshold: {data.get('default_confidence_threshold')}")
    print(f"  wall rule    : >= {data.get('wall_low_confidence_ratio')} low_confidence "
          f"and >= {data.get('wall_min_low_confidence')} points -> 1 POKEYE request per wall")
    print(f"  GPR exclusion: {data.get('gpr_exclusion_radius_m')} m around each hyperbola "
          f"(require GPR clearance: {data.get('require_gpr_clearance')})")
