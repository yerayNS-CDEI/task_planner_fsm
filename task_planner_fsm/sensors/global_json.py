"""The GPR + hyperspectral part of the global JSON for Tecnalia.

The sensor team's ``global_json`` builder (FULL PIPELINE) turns one element's
GPR scan folders and HSI results into the agreed blocks -- ``continuous_gpr``
(one per B-scan, with its wall layers), ``discrete_gpr`` (one per hyperbola)
and ``hyperspectral_camera`` (one measurement per spectrum) -- and writes every
value only the robot knows as ``null``. This module is the robot side:

* **Elements.** One per wall, keyed ``wall_00``, ``wall_01``... A scan that
  matched no scanned line goes under ``unassociated``.
* **GPR coordinates.** The B-scan start/end and each hyperbola apex, in the
  map frame on the wall surface, by the same ``x_m`` -> wall transform the
  NO_DRILL zones use (``gpr.map_transform``: from the plate's seated pose
  along the sweep direction, GPR antenna offset not applied). A scan with no
  line geometry keeps ``null``; one whose record lacks the seated pose falls
  back to the planned scan line ~0.6 m off the wall (and logs a warning).
  The agreed schema has no field to say which, so none is added here.
* **HSI fields.** Each measurement's timestamp and map pose, from the sweep
  record, and ``row`` / ``data_pointer_csv`` re-pointed at the session's
  ``reflectance.csv`` -- the acquisition with the poses -- instead of the
  adapter's intermediate ``input.csv``.

Coordinate errors stay ``null``: the robot has no pose covariance to give.
Rebuilt from scratch on every pass, so it always covers the whole mission so
far. Pure file work over results already on disk; no ROS, no models.
"""

import os
from pathlib import Path

from . import import_vendor
from .gpr import map_transform

FILENAME = "gpr_hyperspectral.json"
UNASSOCIATED = "unassociated"


def element_key(wall_index):
    return UNASSOCIATED if wall_index is None else f"wall_{int(wall_index):02d}"


def _pointer(path, json_dir, cfg):
    """A data pointer as the builder writes its own (``path_mode`` in its config)."""
    if cfg.get("path_mode", "relative") == "absolute":
        return Path(path).resolve().as_posix()
    return Path(os.path.relpath(Path(path).resolve(), Path(json_dir).resolve())).as_posix()


def _point(builder, xyz):
    """``{x, y, z}`` of ``{value, error, unit}``; all null when ``xyz`` is None."""
    if xyz is None:
        return builder.empty_point()
    return {axis: builder.quantity(v) for axis, v in zip(("x", "y", "z"), xyz)}


def _scans_by_wall(gpr_results):
    """``{wall_index: [entry, ...]}``, the latest entry per scan key.

    The cumulative summary appends a pass's entries, so a key re-processed
    later appears twice; its folder holds the later results.
    """
    latest = {}
    for entry in (gpr_results or {}).get("entries", []) or []:
        if entry.get("out_dir"):
            latest[entry.get("key")] = entry
    walls = {}
    for entry in latest.values():
        walls.setdefault((entry.get("line") or {}).get("wall_index"), []).append(entry)
    return walls


def _fill_gpr(builder, block, entries):
    """Map-frame B-scan ends and hyperbola positions, in place."""
    to_map_by_scan = {}
    for continuous, entry in zip(block["continuous_gpr"], entries):
        to_map = map_transform(entry.get("line"))
        to_map_by_scan[continuous["continuous_gpr_id"]] = to_map
        if to_map is None:
            continue
        length = continuous["bscan_length"]["value"]
        continuous["bscan_coordinates"] = {
            "beginning": _point(builder, to_map(0.0)),
            "end": _point(builder, to_map(length) if length is not None else None),
        }
    for discrete in block["discrete_gpr"]:
        to_map = to_map_by_scan.get(discrete["continuous_gpr_id"])
        x_m = discrete["position_along_bscan"]["value"]
        if to_map is not None and x_m is not None:
            discrete["point_coordinates"] = _point(builder, to_map(x_m))


def _fill_hsi(builder, block, samples, wall_index, reflectance_csv, json_dir, cfg):
    """Keep this wall's measurements and give them the robot's fields.

    The builder numbers measurements in ``hsi_result.json`` order, which is
    the order of ``samples`` (the adapter joins them by row). A length
    mismatch means the two disagree, and then nothing is attached rather than
    attaching a pose to the wrong spectrum.
    """
    hsi_block = block["hyperspectral_camera"]
    measurements = hsi_block["measurements"]
    if len(measurements) != len(samples):
        hsi_block["measurements"] = []
        return
    kept = []
    for measurement, sample in zip(measurements, samples):
        if sample.get("wall_index") != wall_index:
            continue
        measurement["timestamp"] = sample.get("timestamp")
        measurement["point_coordinates"] = _point(builder, sample.get("pose_map"))
        if reflectance_csv:
            measurement["row"] = sample.get("reflectance_row")
        kept.append(measurement)
    hsi_block["measurements"] = kept
    if not kept:
        hsi_block["data_pointer_csv"] = ""
    elif reflectance_csv:
        hsi_block["data_pointer_csv"] = _pointer(reflectance_csv, json_dir, cfg)


def build_document(out_path, gpr_results=None, hsi=None):
    """Write the global JSON (GPR + HSI part) to ``out_path``. Returns stats.

    ``gpr_results`` is the session's cumulative ``gpr_summary.json``; ``hsi``
    is what :func:`sensors.hsi.classify_session` returned. Either may be None.
    """
    global_json = import_vendor("global_json")
    builder = global_json.builder
    cfg = global_json.load_config()
    out_path = Path(out_path)
    json_dir = out_path.parent

    scans = _scans_by_wall(gpr_results)
    samples = (hsi or {}).get("samples") or []
    hsi_json = (hsi or {}).get("hsi_result_json")
    hsi_dir = Path(hsi_json).parent if hsi_json and samples else None
    walls = set(scans) | {s.get("wall_index") for s in samples}

    document, stats = {}, {"elements": {}}
    for wall in sorted(walls, key=lambda w: (w is None, w if w is not None else 0)):
        entries = sorted(scans.get(wall, []), key=lambda e: str(e.get("key")))
        block = global_json.build_element(
            [Path(e["out_dir"]) for e in entries], hsi_dir, json_dir, cfg)[0]
        _fill_gpr(builder, block, entries)
        if hsi_dir is not None:
            _fill_hsi(builder, block, samples, wall, (hsi or {}).get("reflectance_csv"),
                      json_dir, cfg)
        key = element_key(wall)
        document[key] = [block]
        stats["elements"][key] = {
            "continuous_gpr": len(block["continuous_gpr"]),
            "discrete_gpr": len(block["discrete_gpr"]),
            "hyperspectral_measurements": len(block["hyperspectral_camera"]["measurements"]),
        }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(global_json.dumps_compact(document) + "\n", encoding="utf-8")
    stats["path"] = str(out_path)
    return stats


def describe(stats):
    """``wall_00: 3 B-scans, 7 hyperbolae, 120 spectra; wall_01: ...``"""
    return "; ".join(
        f"{key}: {e['continuous_gpr']} B-scan(s), {e['discrete_gpr']} hyperbolae, "
        f"{e['hyperspectral_measurements']} spectra"
        for key, e in stats.get("elements", {}).items()
    ) or "no data"
