from __future__ import annotations

from pathlib import Path
import sys

root = Path(__file__).resolve().parent
project_root = root.parent
checks = {
    "Shared GPRTools": project_root / "GPRTools",
    "Berta original code": root / "berta_original" / "line_segmentation.py",
    "Integration wrapper": root / "line_integration" / "pipeline.py",
    "Configuration": root / "config.yaml",
}
ok = True
for name, path in checks.items():
    exists = path.exists()
    print(f"{'OK' if exists else 'MISSING':7s} {name}: {path}")
    ok = ok and exists

try:
    import cv2  # noqa
    import numpy  # noqa
    import yaml  # noqa
    import obspy  # noqa
    print("OK      Python dependencies: cv2, numpy, yaml, obspy")
except Exception as e:
    print(f"MISSING Python dependency: {e}")
    ok = False

raise SystemExit(0 if ok else 2)
