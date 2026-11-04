"""Build the GPR + hyperspectral part of the global OLIWALL JSON (partner: Tecnalia).

The global JSON has one top-level key per inspected element (e.g. "column1") whose value
is a one-item list with the sensor blocks:

    continuous_gpr          one entry per GPR B-scan        (Line_Segmentation -> wall_layers)
    discrete_gpr            one entry per detected hyperbola (Hyperbola_Segmentation)
    hyperspectral_camera    VIS-NIR spectrometer session     (Benjamin v2 classifier)
    hyperspectral_camera_2  XIMEA camera                     (not processed here: left empty)

Everything that depends on the robot (B-scan start/end in world coordinates, the world
coordinates of each GPR detection, element naming) is left as null for the robot software
team. Unknown numbers are null, never strings such as "Unknown".
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

CONFIG_PATH = Path(__file__).resolve().parent / "config.json"
UNIT = "m"


def load_config(config_path: str | Path | None = None) -> dict[str, Any]:
    return json.loads(Path(config_path or CONFIG_PATH).read_text(encoding="utf-8"))


# ── small helpers ────────────────────────────────────────────────────────────────

def quantity(value: float | None, error: float | None = None, unit: str = UNIT) -> dict[str, Any]:
    """{value, error, unit} with the same key order everywhere in the JSON."""
    r = lambda v: None if v is None else round(float(v), 4)  # noqa: E731
    return {"value": r(value), "error": r(error), "unit": unit}


def empty_point() -> dict[str, Any]:
    """x/y/z in the robot/world frame, to be filled by the robot software team."""
    return {axis: quantity(None) for axis in ("x", "y", "z")}


def _float(v: Any) -> float | None:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _pointer(path: Path, json_dir: Path, cfg: dict[str, Any]) -> str:
    if cfg.get("path_mode", "relative") == "absolute":
        return path.resolve().as_posix()
    return Path(os.path.relpath(path.resolve(), json_dir.resolve())).as_posix()


def _read_json(path: Path) -> dict[str, Any] | None:
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


# ── GPR ──────────────────────────────────────────────────────────────────────────

def _tahzeeb_element(scan_dir: Path) -> dict[str, Any]:
    """Element of Tahzeeb's gpr_data.json (04_export): the final hyperbolas of his chain
    (01 consolidate -> 02 fit -> 02b dedup -> cross-gain consensus -> 03/04 export)."""
    mip = _read_json(scan_dir / "hyperbola" / "gpr_data.json") or {}
    return next(iter(mip.get("elements", [])), {})


def _relative_velocity_sigma(element: dict[str, Any], cfg: dict[str, Any]) -> float:
    """sigma_v / v of the element (Tahzeeb: MAD-based, floored); floor if absent."""
    floor = float(cfg.get("velocity_sigma_floor_m_ns", 0.005))
    v, s = _float(element.get("velMns")), _float(element.get("velSigmaMns"))
    if v and s is not None:
        return max(s, floor) / v
    return floor / 0.1224  # epsilon_r = 6 velocity used by both branches


def gpr_scan_blocks(
    scan_dir: Path,
    json_dir: Path,
    cfg: dict[str, Any],
    first_discrete_index: int = 1,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """One continuous_gpr entry + its discrete_gpr entries for a run_gpr.py scan folder."""
    hyp = _read_json(scan_dir / "hyperbola" / "gpr_result.json") or {}
    line = _read_json(scan_dir / "line" / "line_result.json") or {}
    element = _tahzeeb_element(scan_dir)
    scan_id = hyp.get("scan_id") or scan_dir.name
    sgy = next(iter(sorted(scan_dir.glob("*.sgy"))), None)
    rel_sigma_v = _relative_velocity_sigma(element, cfg)

    length = (hyp.get("calibration") or {}).get("scan_distance_m") or (line.get("source") or {}).get("scan_distance_m")

    discrete = []
    for k, obj in enumerate(element.get("objects", []), start=first_discrete_index):
        depth_m = _float(obj.get("d"))
        depth_m = None if depth_m is None else depth_m / 100.0
        discrete.append({
            "discrete_gpr_id": f"dgpr-{k:03d}",
            "continuous_gpr_id": scan_id,
            "position_along_bscan": quantity(_float(obj.get("xM"))),
            "point_coordinates": empty_point(),
            "material": None,
            "depth": quantity(depth_m, None if depth_m is None else depth_m * rel_sigma_v),
            "size": quantity(None),
            "object_type": cfg.get("discrete_object_type", "hyperbola"),
            "confidence": _float(obj.get("c")),
        })

    # Interfaces inside the wall (Berta's horizontal lines), as (depth_m, error_m).
    interfaces = []
    for ln in line.get("lines", []):
        if ln.get("near_time_zero_reference") or ln.get("before_time_zero_reference"):
            continue  # direct wave / air-wall surface reflection: the top of layer 1, depth 0
        d_cm = _float(ln.get("depth_cm_approx"))
        if d_cm is None or d_cm <= 0:
            continue
        half_band_cm = (_float(ln.get("thickness_cm_approx")) or 0.0) / 2.0
        interfaces.append((d_cm / 100.0, math.hypot(d_cm * rel_sigma_v, half_band_cm) / 100.0))
    interfaces.sort()

    # Layer k lies between interface k-1 (the surface for k = 1) and interface k, shallow
    # to deep. Below the deepest interface the back of the wall is not seen: no layer.
    layers, top, top_err = [], 0.0, 0.0
    for k, (bottom, bottom_err) in enumerate(interfaces, start=1):
        layers.append({
            "layer": k,
            "depth": quantity(top, top_err),
            "thickness": quantity(bottom - top, math.hypot(top_err, bottom_err)),
            "material": cfg.get("layer_material", "unknown"),
        })
        top, top_err = bottom, bottom_err

    continuous = {
        "continuous_gpr_id": scan_id,
        "bscan_coordinates": {"beginning": empty_point(), "end": empty_point()},
        "bscan_length": quantity(length),
        "wall_material": cfg.get("wall_material", ""),
        "data_pointer": _pointer(sgy, json_dir, cfg) if sgy else "",
        "wall_layers": layers,
    }
    errors = {b: r["error"] for b, r in (("hyperbola", hyp), ("line", line)) if r.get("status") == "error"}
    if errors:
        continuous["processing_errors"] = errors
    return continuous, discrete


# ── hyperspectral ────────────────────────────────────────────────────────────────

def hyperspectral_block(session_dir: Path | None, json_dir: Path, cfg: dict[str, Any], prefix: str = "hsc") -> dict[str, Any]:
    """hyperspectral_camera block from a run_hyperspectral_pipeline output folder."""
    hsi = _read_json(session_dir / "hsi_result.json") if session_dir else None
    if not hsi:
        return {"data_pointer_csv": "", "measurements": []}

    measurements = []
    for s in hsi.get("samples", []):
        measurements.append({
            "id": f"{prefix}-{s['sample_index'] + 1:03d}",
            "timestamp": None,  # robot data
            "point_coordinates": empty_point(),  # robot data
            "row": s.get("source_row"),
            "material": s.get("material"),
            "confidence": None if s.get("confidence") is None else round(float(s["confidence"]), 3),
            "status": s.get("status"),
        })
    return {
        "data_pointer_csv": _pointer(session_dir / hsi["input_file"], json_dir, cfg),
        "measurements": measurements,
    }


# ── element / document ───────────────────────────────────────────────────────────

def dumps_compact(obj: Any, width: int = 180, indent: int = 2, _level: int = 0) -> str:
    """JSON text that keeps any object/list on one line when it fits in `width` chars,
    so every {value, error, unit} and every x/y/z point takes a single line."""
    flat = json.dumps(obj, ensure_ascii=False)
    pad = " " * (indent * _level)
    if not isinstance(obj, (dict, list)) or not obj or (_level > 0 and len(pad) + len(flat) <= width):
        return flat
    inner = " " * (indent * (_level + 1))
    if isinstance(obj, dict):
        items = [f"{inner}{json.dumps(k, ensure_ascii=False)}: {dumps_compact(v, width, indent, _level + 1)}"
                 for k, v in obj.items()]
        return "{\n" + ",\n".join(items) + "\n" + pad + "}"
    items = [inner + dumps_compact(v, width, indent, _level + 1) for v in obj]
    return "[\n" + ",\n".join(items) + "\n" + pad + "]"


def build_element(
    gpr_scan_dirs: list[Path],
    hsi_session_dir: Path | None,
    json_dir: Path,
    cfg: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    """Value of one element key ("column1": [ ... ]) of the global JSON."""
    cfg = cfg or load_config()
    continuous, discrete = [], []
    for scan_dir in gpr_scan_dirs:
        c, d = gpr_scan_blocks(scan_dir, json_dir, cfg, first_discrete_index=len(discrete) + 1)
        continuous.append(c)
        discrete += d
    return [{
        "continuous_gpr": continuous,
        "discrete_gpr": discrete,
        "hyperspectral_camera": hyperspectral_block(hsi_session_dir, json_dir, cfg),
        "hyperspectral_camera_2": {"data_pointer_csv": "", "measurements": []},
    }]
