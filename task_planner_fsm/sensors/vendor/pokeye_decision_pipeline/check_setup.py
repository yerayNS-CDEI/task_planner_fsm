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
