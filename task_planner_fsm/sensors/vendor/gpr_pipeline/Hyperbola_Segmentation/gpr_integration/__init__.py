"""DISCOVER / OLIWALL GPR robot-integration layer."""
from __future__ import annotations

from pathlib import Path
from typing import Any


def run_gpr_pipeline(
    input_sgy: str | Path,
    config_path: str | Path | None = None,
    output_dir: str | Path | None = None,
    weights_path: str | Path | None = None,
    tahzeeb_config_path: str | Path | None = None,
) -> dict[str, Any]:
    from .pipeline import run_gpr_pipeline as _run
    return _run(input_sgy, config_path, output_dir, weights_path, tahzeeb_config_path)


__all__ = ["run_gpr_pipeline"]
