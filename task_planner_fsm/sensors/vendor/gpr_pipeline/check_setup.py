from __future__ import annotations

import importlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main() -> int:
    required_paths = [
        ROOT / "GPRTools",
        ROOT / "Hyperbola_Segmentation" / "models" / "best.pt",
        ROOT / "Hyperbola_Segmentation" / "tahzeeb_original" / "config.yaml",
        ROOT / "Line_Segmentation" / "berta_original" / "line_segmentation.py",
    ]
    ok = True
    print("GPR_DISCOVER_PIPELINE_v4 setup check")
    for path in required_paths:
        exists = path.exists()
        print(f"[{'OK' if exists else 'MISSING'}] {path.relative_to(ROOT)}")
        ok = ok and exists

    modules = ["numpy", "pandas", "yaml", "scipy", "PIL", "matplotlib", "cv2", "openpyxl", "obspy", "torch", "torchvision"]
    for name in modules:
        try:
            importlib.import_module(name)
            print(f"[OK] python module: {name}")
        except Exception as exc:
            print(f"[MISSING] python module: {name} ({exc})")
            ok = False

    print("\nReady." if ok else "\nSetup incomplete. Run: pip install -r requirements.txt")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
