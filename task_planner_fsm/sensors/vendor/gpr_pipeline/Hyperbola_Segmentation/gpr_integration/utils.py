from __future__ import annotations

import importlib.util
import json
import re
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import yaml

_DATE_RE = re.compile(r"_20\d{6}")


def load_yaml(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def resolve_from(base_file: str | Path, value: str | Path) -> Path:
    p = Path(value)
    if p.is_absolute():
        return p
    return (Path(base_file).resolve().parent / p).resolve()


def portable_scan_stem(path: str | Path) -> str:
    """Filesystem-safe stem while preserving acquisition dates used by Tahzeeb parsers."""
    stem = Path(path).stem
    stem = re.sub(r"[^A-Za-z0-9_-]+", "_", stem).strip("_")
    if not stem:
        stem = "gpr_scan"
    # Tahzeeb's ridge lookup splits filenames at '_202'. If a scan has no date,
    # inject a compatibility date token so his unmodified code still maps masks
    # and consolidated detections to the same element label.
    if not _DATE_RE.search(stem):
        stem += "_20260101"
    return stem


def scan_element_id(model_stem: str) -> str:
    m = _DATE_RE.search(model_stem)
    return model_stem[:m.start()] if m else model_stem


def gain_token(gain_db: float) -> str:
    sign = "p" if gain_db >= 0 else "m"
    txt = f"{abs(float(gain_db)):g}".replace(".", "p")
    return f"{sign}{txt}dB"


def json_dump(data: Any, path: str | Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def load_module_from_path(
    name: str,
    path: str | Path,
    support_dir: str | Path | None = None,
) -> ModuleType:
    if support_dir is not None:
        support = str(Path(support_dir).resolve())
        if support not in sys.path:
            sys.path.insert(0, support)
    spec = importlib.util.spec_from_file_location(name, str(Path(path).resolve()))
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot import module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module
