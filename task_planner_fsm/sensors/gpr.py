"""GPR post-processing: exported GP8800 scans -> hyperbolae and lines on the wall.

The probe keeps its traces. ScanWall starts and stops each line over HTTP and,
after the line stop, pulls ``POST /measurement/export/raw`` -- a zip holding
``<name>_<stamp>/<name>.sgy`` plus the ``.csv`` sidecar (and a ``.json``) --
and unpacks it flat into the session's own ``incoming/`` with the line key as
a prefix (``unpack_export``). The shared ``data/raw/gpr/incoming/`` is only
for files dropped there by hand, and is read after the session's folder. This
module keeps the fewest assumptions it can:

* any ``.sgy`` with a sidecar in an incoming folder is a scan;
* a scan is tied to a line from ScanWall's manifest by name if the file name
  contains the line key or the measurement name, else by time (the most recent
  line started before the file was written);
* a scan that matches nothing is processed anyway and reported as unassociated
  -- the data is still worth having, it just cannot be placed on the wall.

Per scan both delivered pipelines run: hyperbola segmentation (Mask R-CNN) and
horizontal-line segmentation. Their ``x_m`` is local to the scan; with the
matched line's segment endpoints it becomes a map-frame point. Depth stays as
the pipelines report it (approximate, epsilon_r = 6 assumed).

Each hyperbola result is also handed to the POKEYE package, which turns it into
NO_DRILL constraints (policy v2). They are attached to the entry as
``no_drill``, already carried onto the wall by the same transform, because this
is the only point in the mission where the raw ``gpr_result`` and the scanned
line's endpoints are both in hand. GPR still does not send POKEYE anywhere --
see ``no_drill.py`` for what the constraint then does.

Files already processed in this session are remembered in
``processed_files.json`` so the state, which runs after every wall, does not
redo earlier walls. Blocking functions for a BackgroundJob; no ROS.
"""

import json
import os
import zipfile
from pathlib import Path

import numpy as np

from . import VendorUnavailable, import_vendor, require
from . import no_drill
from .manifest import read_gpr_lines

REGISTRY_FILENAME = "processed_files.json"
SUMMARY_FILENAME = "gpr_summary.json"


# ----------------------------------------------------------------------
# Unpacking the app's export into the incoming folder
# ----------------------------------------------------------------------
EXPORT_MEMBER_SUFFIXES = (".sgy", ".segy", ".csv", ".json")


def unpack_export(zip_path, incoming_dir, key):
    """Flatten the ``.sgy``/``.csv``/``.json`` members of an export zip into
    ``incoming_dir`` as ``<key>_<original name>``. Returns the paths written.

    The pipeline finds a scan by its ``.sgy`` and needs the same-stem ``.csv``
    beside it, so the sidecars are written first and the ``.sgy`` last: a
    directory listing taken mid-way sees no half-delivered scan. Member paths
    are reduced to their basename (no directory traversal from the archive).
    """
    incoming_dir = Path(os.path.expanduser(str(incoming_dir)))
    incoming_dir.mkdir(parents=True, exist_ok=True)
    written = []
    with zipfile.ZipFile(zip_path) as zf:
        members = [
            m for m in zf.infolist()
            if not m.is_dir() and m.filename.lower().endswith(EXPORT_MEMBER_SUFFIXES)
        ]
        # sidecars first, traces last
        members.sort(key=lambda m: m.filename.lower().endswith((".sgy", ".segy")))
        for m in members:
            name = os.path.basename(m.filename)
            target = incoming_dir / f"{key}_{name}"
            partial = target.with_name(target.name + ".part")
            with zf.open(m) as src, open(partial, "wb") as dst:
                dst.write(src.read())
            os.replace(partial, target)
            written.append(str(target))
    return written


# ----------------------------------------------------------------------
# Finding and matching scans
# ----------------------------------------------------------------------
def _as_dirs(incoming):
    """One folder or a list of them, expanded; a missing folder is skipped."""
    items = incoming if isinstance(incoming, (list, tuple)) else [incoming]
    dirs = []
    for item in items:
        d = Path(os.path.expanduser(str(item)))
        if d.is_dir() and d not in dirs:
            dirs.append(d)
    return dirs


def find_scan_files(incoming):
    """``[{sgy, csv, stem, mtime}]`` for every SEGY with a sidecar, oldest
    first. ``incoming`` is one folder or several (the session's own, then the
    shared inbox)."""
    found = []
    for incoming_dir in _as_dirs(incoming):
        for sgy in list(incoming_dir.glob("*.sgy")) + list(incoming_dir.glob("*.segy")):
            csv = sgy.with_suffix(".csv")
            if not csv.is_file():
                continue            # the pipelines need the sidecar; wait for it
            found.append({
                "sgy": str(sgy),
                "csv": str(csv),
                "stem": sgy.stem,
                "mtime": os.path.getmtime(sgy),
            })
    return sorted(found, key=lambda f: f["mtime"])


def _line_t_start(line):
    try:
        return float(line.get("t_start_epoch"))
    except (TypeError, ValueError):
        return None


def match_files_to_lines(files, lines):
    """Pair each file with at most one manifest line. Returns ``[(file, line|None)]``.

    Name match first (the key ``w02_l01_s00`` or the measurement name inside
    the file stem), then the time fallback. A line is used at most once.
    """
    taken = set()
    pairs = []
    pending = []
    for f in files:
        stem = f["stem"].lower()
        hit = None
        for i, line in enumerate(lines):
            if i in taken:
                continue
            key = str(line.get("key", "")).lower()
            name = str(line.get("measurement_name", "")).lower().replace(" ", "_")
            if (key and key in stem) or (name and name in stem.replace(" ", "_")):
                hit = i
                break
        if hit is None:
            pending.append(f)
        else:
            taken.add(hit)
            pairs.append((f, lines[hit]))

    for f in pending:
        best, best_t = None, None
        for i, line in enumerate(lines):
            if i in taken:
                continue
            t0 = _line_t_start(line)
            if t0 is None or t0 > f["mtime"]:
                continue
            if best_t is None or t0 > best_t:
                best, best_t = i, t0
        if best is None:
            pairs.append((f, None))
        else:
            taken.add(best)
            pairs.append((f, lines[best]))
    return pairs


# ----------------------------------------------------------------------
# Geo-referencing
# ----------------------------------------------------------------------
def scan_to_map(x_m, seg_start, seg_end):
    """Point ``x_m`` metres along the scan, from ``seg_start`` towards ``seg_end``.

    Straight-line sweep along the wall; the plate's z is the segment's. The
    GPR antenna offset from the plate TCP is not applied here (unknown yet).
    """
    start = np.asarray(seg_start, dtype=float)
    end = np.asarray(seg_end, dtype=float)
    axis = end - start
    norm = float(np.linalg.norm(axis[:2]))
    if norm < 1e-6:
        return [round(float(v), 4) for v in start]
    unit = np.array([axis[0] / norm, axis[1] / norm, 0.0])
    return [round(float(v), 4) for v in start + unit * float(x_m)]


def map_transform(line):
    """``x_m -> [x, y, z]`` for a scanned line, or None when it has no geometry.

    One place decides whether a scan can be placed on the wall at all, so the
    compact detections and the NO_DRILL constraints can never disagree about it.
    """
    if not line or not line.get("seg_start") or not line.get("seg_end"):
        return None
    return lambda x_m: scan_to_map(x_m, line["seg_start"], line["seg_end"])


def _compact_hyperbolae(result, line):
    to_map = map_transform(line)
    dets = []
    for d in result.get("detections", []):
        pos = d.get("position", {})
        depth = d.get("depth", {})
        rob = d.get("robustness", {})
        entry = {
            "id": d.get("id"),
            "x_m": pos.get("x_m"),
            "x_relative": pos.get("x_relative"),
            "depth_cm": depth.get("depth_cm"),
            "confidence": rob.get("confidence_max", rob.get("confidence_mean")),
            "gain_support": rob.get("gain_support"),
        }
        if to_map is not None and pos.get("x_m") is not None:
            entry["position_map"] = to_map(pos["x_m"])
        dets.append(entry)
    return {
        "detected": bool(result.get("hyperbola_detected")),
        "n": int(result.get("n_valid_detections", len(dets))),
        "detections": dets,
        "scan_distance_m": result.get("calibration", {}).get("scan_distance_m"),
    }


def _compact_lines(result):
    return {
        "detected": bool(result.get("detected")),
        "n": int(result.get("n_lines", 0)),
        "lines": [{
            "id": ln.get("id"),
            "depth_cm": ln.get("depth_cm_approx"),
            "thickness_cm": ln.get("thickness_cm_approx"),
            "distance_to_next_cm": ln.get("distance_to_next_cm_approx"),
            "confidence": ln.get("confidence"),
            "snapped_to_time_zero": ln.get("snapped_to_time_zero"),
        } for ln in result.get("lines", [])],
    }


# ----------------------------------------------------------------------
# Running the pipelines
# ----------------------------------------------------------------------
def process_scan(scan, line, out_dir, weights_path, logger=None,
                 run_hyperbolae=True, run_lines=True):
    """Both pipelines over one scan. Never raises for a pipeline failure: the
    error is recorded in the entry so one bad file does not sink the others."""
    key = (line or {}).get("key") or scan["stem"]
    scan_dir = Path(out_dir) / key
    scan_dir.mkdir(parents=True, exist_ok=True)
    entry = {
        "key": key,
        "sgy": scan["sgy"],
        "line": line,
        "associated": line is not None,
        "out_dir": str(scan_dir),
        "hyperbolae": None,
        "no_drill": None,
        "lines": None,
        "errors": {},
    }

    if run_hyperbolae:
        try:
            gpr_integration = import_vendor("gpr_integration")
            result = gpr_integration.run_gpr_pipeline(
                scan["sgy"], output_dir=str(scan_dir / "hyperbolae"),
                weights_path=str(weights_path))
            entry["hyperbolae"] = _compact_hyperbolae(result, line)
            # The POKEYE package owns the NO_DRILL policy; the raw result goes
            # to it unchanged, and only the frame is ours to supply.
            entry["no_drill"] = no_drill.constraints_from_result(
                result, to_map=map_transform(line), source_key=key)
        except BaseException as exc:            # noqa: BLE001 -- recorded per scan
            entry["errors"]["hyperbolae"] = f"{type(exc).__name__}: {exc}"
            if logger is not None:
                logger.error(f"GPR hyperbola pipeline failed on {scan['sgy']}: {exc}")

    if run_lines:
        try:
            line_integration = import_vendor("line_integration")
            result = line_integration.run_line_pipeline(
                scan["sgy"], output_dir=str(scan_dir / "lines"))
            entry["lines"] = _compact_lines(result)
        except BaseException as exc:            # noqa: BLE001
            entry["errors"]["lines"] = f"{type(exc).__name__}: {exc}"
            if logger is not None:
                logger.error(f"GPR line pipeline failed on {scan['sgy']}: {exc}")
    return entry


def _load_registry(path):
    if not Path(path).is_file():
        return {}
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def _registry_key(scan):
    return f"{scan['sgy']}@{scan['mtime']:.0f}"


def load_summary(out_dir):
    """The cumulative ``gpr_summary.json`` for this session, or None.

    Every pass appends to it, so this is the whole mission's GPR record --
    which is what the NO_DRILL constraints have to be read from. The wall just
    scanned is usually in the pass that just ran, but a scan that arrived late,
    or a pass with no new exports at all, must not lose the wall its hyperbolae
    belong to.
    """
    summary = _load_registry(Path(out_dir) / SUMMARY_FILENAME)
    return summary if isinstance(summary, dict) and summary.get("entries") else None


def pending_files(incoming, out_dir):
    """Scans in the incoming folder(s) not yet processed into ``out_dir``.
    Cheap: directory listings and a small JSON, safe to call every FSM tick."""
    registry = _load_registry(Path(out_dir) / REGISTRY_FILENAME)
    return [f for f in find_scan_files(incoming) if _registry_key(f) not in registry]


def process_incoming(incoming, manifest_path, out_dir, weights_path,
                     logger=None, run_hyperbolae=True, run_lines=True):
    """Process every new scan in the incoming folder(s). Blocking.

    Returns::

        {"n_files": int, "n_new": int, "n_associated": int,
         "n_hyperbolae": int, "n_lines": int, "entries": [...],
         "summary_json": str}
    """
    # Without obspy no SEGY can be read at all; without torch only the
    # hyperbola half is lost, so that one degrades instead of failing.
    require("obspy", "yaml", "cv2", "scipy", "PIL")
    hyperbolae_skipped = None
    if run_hyperbolae:
        try:
            require("torch", "torchvision", "pandas", "openpyxl")
        except VendorUnavailable as exc:
            hyperbolae_skipped = str(exc)
            run_hyperbolae = False
            if logger is not None:
                logger.warn(f"GPR: hyperbola segmentation skipped ({exc}); running the line pipeline only")

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    registry_path = out_dir / REGISTRY_FILENAME
    registry = _load_registry(registry_path)
    # Cumulative summary: this pass appends to whatever earlier walls produced.
    summary_path = out_dir / SUMMARY_FILENAME
    previous = _load_registry(summary_path)
    all_entries = previous.get("entries", []) if isinstance(previous, dict) else []

    files = find_scan_files(incoming)
    new_files = [f for f in files if _registry_key(f) not in registry]
    lines = read_gpr_lines(manifest_path)
    # Lines already tied to a processed file are not offered again.
    used_keys = {v.get("key") for v in registry.values() if v.get("associated")}
    free_lines = [ln for ln in lines if ln.get("key") not in used_keys]

    entries = []
    for scan, line in match_files_to_lines(new_files, free_lines):
        if logger is not None:
            logger.info(f"GPR: processing {Path(scan['sgy']).name}"
                        + (f" as line {line.get('key')}" if line else " (unassociated)"))
        entry = process_scan(scan, line, out_dir, weights_path, logger,
                             run_hyperbolae=run_hyperbolae, run_lines=run_lines)
        entries.append(entry)
        registry[_registry_key(scan)] = {
            "key": entry["key"], "associated": entry["associated"],
            "out_dir": entry["out_dir"], "errors": entry["errors"],
        }
        # Registry and summary are written together, per scan: a run that is
        # killed (or a state left) between two scans must not leave a scan
        # marked processed, and so never offered again, yet missing from the
        # summary the NO_DRILL constraints are read from.
        all_entries.append(entry)
        with open(summary_path, "w") as handle:
            json.dump({"entries": all_entries}, handle, indent=2)
        with open(registry_path, "w") as handle:
            json.dump(registry, handle, indent=2)

    result = {
        "n_files": len(files),
        "n_new": len(new_files),
        "n_associated": sum(1 for e in entries if e["associated"]),
        "n_hyperbolae": sum((e["hyperbolae"] or {}).get("n", 0) for e in entries),
        "n_no_drill": sum((e["no_drill"] or {}).get("n_no_drill_positions", 0) for e in entries),
        "n_no_drill_unlocated": sum(
            (e["no_drill"] or {}).get("n_no_drill_positions", 0)
            - (e["no_drill"] or {}).get("n_located", 0) for e in entries),
        "n_lines": sum((e["lines"] or {}).get("n", 0) for e in entries),
        "n_failed": sum(1 for e in entries if e["errors"]),
        "hyperbolae_skipped": hyperbolae_skipped,
        "entries": entries,
        "summary_json": str(summary_path),
    }
    if not entries and not summary_path.is_file():
        with open(summary_path, "w") as handle:
            json.dump({"entries": all_entries}, handle, indent=2)
    return result


def describe_entry(entry):
    hyp = entry.get("hyperbolae") or {}
    lines = entry.get("lines") or {}
    parts = [f"{entry['key']}:"]
    if entry["errors"]:
        parts.append("errors " + ", ".join(entry["errors"]))
    if hyp:
        parts.append(f"{hyp.get('n', 0)} hyperbolae")
    nd = entry.get("no_drill") or {}
    if nd.get("n_no_drill_positions"):
        parts.append(f"{nd['n_no_drill_positions']} NO_DRILL "
                     f"({nd.get('n_located', 0)} placed on the wall)")
    if lines:
        parts.append(f"{lines.get('n', 0)} lines")
    if not entry["associated"]:
        parts.append("(not tied to a scanned line; no map position)")
    return " ".join(parts)
