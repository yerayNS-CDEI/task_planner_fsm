from pathlib import Path
import sys

root = Path(__file__).resolve().parent
checks = {
    "Benjamin predict.py": root / "benjamin_original" / "predict.py",
    "classifier.joblib": root / "benjamin_original" / "classifier.joblib",
    "integration config": root / "hsi_integration" / "config.json",
}
failed = False
for name, path in checks.items():
    ok = path.exists()
    print(f"{'OK' if ok else 'MISSING':7s} {name}: {path.relative_to(root)}")
    failed |= not ok

for module in ["pandas", "numpy", "scipy", "sklearn", "xgboost", "joblib"]:
    try:
        __import__(module)
        print(f"OK      python module: {module}")
    except Exception as exc:
        print(f"MISSING python module: {module} ({exc})")
        failed = True

sys.exit(1 if failed else 0)
