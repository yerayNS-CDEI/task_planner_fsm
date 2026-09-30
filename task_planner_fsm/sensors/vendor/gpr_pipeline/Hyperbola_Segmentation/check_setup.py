#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

from gpr_integration.utils import load_yaml, resolve_from


def main() -> int:
    cfg_path = Path(__file__).resolve().parent / "gpr_integration" / "config.yaml"
    cfg = load_yaml(cfg_path)
    project_root = Path(__file__).resolve().parent.parent
    checks = {
        "Shared GPRTools folder": project_root / "GPRTools",
        "Tahzeeb code folder": resolve_from(cfg_path, cfg["paths"]["tahzeeb_dir"]),
        "Mask R-CNN ResNet101 weights": resolve_from(cfg_path, cfg["paths"]["weights"]),
        "Tahzeeb post-processing config": resolve_from(cfg_path, cfg["paths"]["tahzeeb_config"]),
    }
    ok = True
    for name, path in checks.items():
        exists = path.exists()
        print(f"[{'OK' if exists else 'MISSING'}] {name}: {path}")
        ok = ok and exists

    plot = cfg.get("preprocessing", {}).get("plot", {})
    print(f"[OK] GPRTools render: plot_scale={plot.get('plot_scale')} render_upsample={plot.get('render_upsample')}")
    print(f"[OK] Tiler args: {cfg.get('tiling', {}).get('extra_args', [])}")
    print(f"[OK] Model: {cfg.get('inference', {}).get('model_type')} / {cfg.get('inference', {}).get('backbone')}")
    print(f"[OK] Effective GP8800 time window expected: {cfg.get('metadata', {}).get('expected_corrected_time_window_ns')} ns")

    tahzeeb_dir = checks["Tahzeeb code folder"]
    optional_size = tahzeeb_dir / "02c_size_estimate.py"
    if not optional_size.exists():
        print("[OPTIONAL MISSING] 02c_size_estimate.py: referenced by the supplied config 'size' block, "
              "but not required by the current 01->02->02b->03->04 pipeline")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
