"""Hyperspectral material classification over a recorded sweep.

The sweep's ``reflectance.csv`` (written by ``SessionProcessor``) already *is*
the classifier's input: Benjamin's v2 model (a 1D-CNN) reads the robot
session format directly, ``VIS_``/``NIR_`` headers included, and resamples
onto its own wavelength grid. The adapter still writes its own copy, to drop
the rows that never got a reflectance and to carry back the metadata we need.

So the adapter is thin:

    reflectance.csv  ->  input.csv (numeric headers)  ->  run_hyperspectral_pipeline
                     ->  join results back onto wall/line/seg/pose by row order

The join is by row order rather than by column because the vendor's input
adapter only forwards its own metadata names (``Counter``, ``Label``...).
``Counter`` is filled with our ``Seq`` as a cross-check that the join is right.

Everything here is a blocking function meant to run inside a
:class:`~task_planner_fsm.sensors.background_job.BackgroundJob`; no ROS.
"""

import csv
import json
import os
import shutil
from pathlib import Path

from ..utils import hyperspectral_processing as hp
from . import import_vendor, paths, require

INPUT_FILENAME = "input.csv"
CONFIG_FILENAME = "config.json"
SAMPLES_FILENAME = "samples.csv"
SUMMARY_FILENAME = "wall_summary.json"

# Vendor default; kept in sync with the POKEYE decision package (the HSI
# message carries the threshold, and the decision honours it).
DEFAULT_CONFIDENCE_THRESHOLD = 0.8

# Columns of reflectance.csv that describe the sample rather than its spectrum.
_META_COLUMNS = (
    "Seq", "Timestamp", "Wall_Index", "Line_Idx", "Seg_Idx", "Trigger_Idx",
    "Travel_m", "Frame", "X", "Y", "Z", "Map_X", "Map_Y", "Map_Z",
    "Status", "Reason",
)

_SPECTRAL_PREFIXES = ("VIS_", "NIR_")


def _int_or_none(value):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _float_or_none(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if v == v else None       # NaN -> None


def _xyz(row, keys):
    vals = [_float_or_none(row.get(k)) for k in keys]
    return None if any(v is None for v in vals) else vals


# ----------------------------------------------------------------------
# Step 1: the pipeline's input file
# ----------------------------------------------------------------------
def build_input_csv(reflectance_csv, input_csv):
    """Rewrite ``reflectance.csv`` with numeric wavelength headers.

    Rows that never produced a valid reflectance (``rejected_calibration``) are
    left out: they carry no physical spectrum for the quality filter to judge.
    Everything else goes through -- including rows our own stability check
    rejected, because the vendor's filter is the authoritative one now and we
    keep our verdict alongside as metadata.

    Returns the per-row metadata, in the order the rows were written, which is
    the order the pipeline reports ``sample_index`` in.
    """
    reflectance_csv = Path(reflectance_csv)
    input_csv = Path(input_csv)
    input_csv.parent.mkdir(parents=True, exist_ok=True)

    metadata = []
    with open(reflectance_csv, newline="") as src, open(input_csv, "w", newline="") as dst:
        reader = csv.reader(src)
        header = next(reader, None)
        if header is None:
            return metadata
        col = {name: i for i, name in enumerate(header)}
        spectral = [(i, name) for i, name in enumerate(header)
                    if name.startswith(_SPECTRAL_PREFIXES)]
        if not spectral:
            raise ValueError(f"{reflectance_csv} has no VIS_/NIR_ spectral columns")

        writer = csv.writer(dst)
        # ``Counter`` is one of the metadata names the vendor adapter forwards,
        # so it comes back inside each result sample and lets us assert the join.
        writer.writerow(["Counter"] + [name.split("_", 1)[1] for _, name in spectral])

        data_row = 0
        for row in reader:
            if not row:
                continue
            data_row += 1
            status = row[col["Status"]] if "Status" in col else ""
            if status == hp.REJECTED_CALIBRATION:
                continue
            seq = row[col["Seq"]] if "Seq" in col else str(len(metadata) + 1)
            writer.writerow([seq] + [row[i] for i, _ in spectral])
            get = lambda name: row[col[name]] if name in col and col[name] < len(row) else None  # noqa: E731
            metadata.append({
                "seq": _int_or_none(seq),
                # 1-based data row in reflectance.csv: what the global JSON
                # points at, since input.csv skips rows.
                "reflectance_row": data_row,
                "timestamp": get("Timestamp") or None,
                "wall_index": _int_or_none(get("Wall_Index")),
                "line_idx": _int_or_none(get("Line_Idx")),
                "seg_idx": _int_or_none(get("Seg_Idx")),
                "trigger_idx": _int_or_none(get("Trigger_Idx")),
                # Plate travel from the segment start, as a GPR hyperbola's
                # x_m on that line is (the sensors' offsets on the plate are
                # not applied, as in gpr.scan_to_map): the POKEYE package
                # compares the two.
                "travel_m": _float_or_none(get("Travel_m")),
                "frame": get("Frame") or None,
                "pose": _xyz({k: get(k) for k in ("X", "Y", "Z")}, ("X", "Y", "Z")),
                "pose_map": _xyz({k: get(k) for k in ("Map_X", "Map_Y", "Map_Z")},
                                 ("Map_X", "Map_Y", "Map_Z")),
                "fsm_status": status,
                "fsm_reason": get("Reason") or "",
            })
    return metadata


# ----------------------------------------------------------------------
# Step 2: runtime config
# ----------------------------------------------------------------------
def write_runtime_config(config_path, model_path, confidence_threshold, device=""):
    """The vendor config with our model location, threshold and device.

    Their ``config.json`` points at ``benjamin_original/classifier.joblib``,
    which is not shipped (see ``models/README.md``). ``model_path`` may be
    absolute: the pipeline joins it onto its project root with ``pathlib``,
    which yields the absolute path unchanged. An empty ``device`` becomes the
    vendor's ``"auto"`` (CUDA when torch sees it).
    """
    vendor_cfg = paths.HSI_PROJECT / "hsi_integration" / "config.json"
    cfg = {}
    if vendor_cfg.is_file():
        with open(vendor_cfg) as handle:
            cfg = json.load(handle)
    cfg["model_path"] = str(Path(model_path).resolve())
    cfg["confidence_threshold"] = float(confidence_threshold)
    cfg["keep_columns"] = ["Counter"]
    cfg["torch_device"] = device or "auto"
    config_path = Path(config_path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    with open(config_path, "w") as handle:
        json.dump(cfg, handle, indent=2)
    return config_path


# ----------------------------------------------------------------------
# Step 3: join results back onto the sweep record
# ----------------------------------------------------------------------
def join_results(metadata, hsi_result):
    """One flat dict per sample: sweep metadata + the classifier's verdict."""
    samples = []
    by_index = {int(s.get("sample_index", i)): s
                for i, s in enumerate(hsi_result.get("samples", []))}
    for i, meta in enumerate(metadata):
        verdict = by_index.get(i)
        entry = dict(meta)
        if verdict is None:
            entry.update({"status": "missing", "detected": False, "material": None,
                          "confidence": None, "reason": "no result for this row"})
        else:
            counter = _int_or_none((verdict.get("metadata") or {}).get("Counter"))
            if counter is not None and meta["seq"] is not None and counter != meta["seq"]:
                raise ValueError(
                    f"HSI result row {i} carries Counter={counter} but the input "
                    f"row was Seq={meta['seq']}: the join by row order is broken")
            entry.update({
                "status": verdict.get("status"),
                "detected": bool(verdict.get("detected")),
                "material": verdict.get("material"),
                "confidence": _float_or_none(verdict.get("confidence")),
                "reason": verdict.get("reason"),
            })
        samples.append(entry)
    return samples


def summarize_by_wall(samples):
    """``{wall_index: {n, status: {...}, material: {...}}}`` for the log."""
    out = {}
    for s in samples:
        wall = s.get("wall_index")
        bucket = out.setdefault(wall, {"n": 0, "status": {}, "material": {}})
        bucket["n"] += 1
        bucket["status"][s["status"]] = bucket["status"].get(s["status"], 0) + 1
        if s.get("detected") and s.get("material"):
            bucket["material"][s["material"]] = bucket["material"].get(s["material"], 0) + 1
    return out


def write_samples_csv(samples, path):
    fields = ["seq", "timestamp", "wall_index", "line_idx", "seg_idx", "trigger_idx",
              "travel_m", "frame", "x", "y", "z", "map_x", "map_y", "map_z",
              "fsm_status", "status", "detected", "material", "confidence", "reason"]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for s in samples:
            pose = s.get("pose") or [None, None, None]
            pose_map = s.get("pose_map") or [None, None, None]
            writer.writerow([
                s.get("seq"), s.get("timestamp"), s.get("wall_index"), s.get("line_idx"),
                s.get("seg_idx"), s.get("trigger_idx"), s.get("travel_m"), s.get("frame"),
                *pose, *pose_map,
                s.get("fsm_status"), s.get("status"), s.get("detected"),
                s.get("material"), s.get("confidence"), s.get("reason"),
            ])
    return str(path)


# ----------------------------------------------------------------------
# The whole thing
# ----------------------------------------------------------------------
# Where the 1D-CNN runs: the vendor config's ``torch_device``, which Benjamin's
# classifier would otherwise set to CUDA whenever torch sees it. The model is
# ~100 kB and a session is a few dozen spectra, so the GPU buys nothing worth
# taking it from the GPR's Mask R-CNN; the delivered config pins the CPU too.
DEFAULT_DEVICE = "cpu"


def classify_session(session_dir, out_dir, model_path,
                     confidence_threshold=DEFAULT_CONFIDENCE_THRESHOLD, logger=None,
                     device=DEFAULT_DEVICE):
    """Classify every spectrum of a processed session. Blocking.

    ``device`` is the torch device the classifier runs on (``hsi_device`` in
    ctx; see DEFAULT_DEVICE for why it is the CPU, empty for CUDA-if-present).

    Returns::

        {
          "n_input": int, "n_classified": int,
          "samples": [...],              # see join_results
          "by_wall": {...},              # see summarize_by_wall
          "input_csv": str, "hsi_result_json": str, "samples_csv": str,
          "confidence_threshold": float,
        }

    Raises :class:`~task_planner_fsm.sensors.VendorUnavailable` when the
    classifier stack is not importable, FileNotFoundError when the session has
    no ``reflectance.csv`` or the model is missing.
    """
    session_dir = Path(os.path.expanduser(str(session_dir)))
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    reflectance_csv = session_dir / hp.REFLECTANCE_FILENAME
    if not reflectance_csv.is_file():
        raise FileNotFoundError(
            f"no {hp.REFLECTANCE_FILENAME} in {session_dir}: run the reflectance "
            f"pass first (SensorDataProcessing phase 'hyperspectral')")
    model_path = Path(model_path)
    if not model_path.is_file():
        raise FileNotFoundError(
            f"HSI classifier not found at {model_path} (see models/README.md)")

    # The classifier bundle unpickles into a torch model plus sklearn scaler
    # and label encoder; fail here, in one line, rather than inside joblib.
    require("joblib", "torch", "sklearn", "scipy", "pandas", "matplotlib")

    input_csv = out_dir / INPUT_FILENAME
    metadata = build_input_csv(reflectance_csv, input_csv)
    result = {
        "n_input": len(metadata),
        "n_classified": 0,
        "samples": [],
        "by_wall": {},
        "input_csv": str(input_csv),
        "reflectance_csv": None,
        "hsi_result_json": None,
        "samples_csv": None,
        "confidence_threshold": float(confidence_threshold),
    }
    if not metadata:
        if logger is not None:
            logger.warn("HSI: the session has no spectra to classify")
        return result

    config_path = write_runtime_config(out_dir / CONFIG_FILENAME, model_path,
                                       confidence_threshold, device)

    # Imported here, not at module load: this is where torch/joblib are needed.
    hsi_integration = import_vendor("hsi_integration")
    hsi_result = hsi_integration.run_hyperspectral_pipeline(
        str(input_csv), output_dir=str(out_dir), config_path=str(config_path))

    samples = join_results(metadata, hsi_result)
    result["samples"] = samples
    result["n_classified"] = len(samples)
    result["by_wall"] = summarize_by_wall(samples)
    result["hsi_result_json"] = str(out_dir / "hsi_result.json")
    result["samples_csv"] = write_samples_csv(samples, out_dir / SAMPLES_FILENAME)
    # The acquisition the global JSON's ``row`` numbers refer to, next to the
    # results so the processed session carries it when sent on.
    reflectance_copy = out_dir / hp.REFLECTANCE_FILENAME
    if reflectance_copy.resolve() != reflectance_csv.resolve():
        shutil.copy2(reflectance_csv, reflectance_copy)
    result["reflectance_csv"] = str(reflectance_copy)
    with open(out_dir / SUMMARY_FILENAME, "w") as handle:
        json.dump({str(k): v for k, v in result["by_wall"].items()}, handle, indent=2)
    return result


def describe_wall(wall_index, bucket):
    """One log line per wall.

    ``wall 2: 143 samples, 120 detected (gypsum x118, brick x2), 15 low_confidence, 8 quality_rejected``
    """
    status = bucket.get("status", {})
    materials = ", ".join(f"{m} x{n}" for m, n in
                          sorted(bucket.get("material", {}).items(), key=lambda kv: -kv[1]))
    parts = [f"{status.get('detected', 0)} detected" + (f" ({materials})" if materials else "")]
    for key in ("low_confidence", "quality_rejected", "missing"):
        if status.get(key):
            parts.append(f"{status[key]} {key}")
    return f"wall {wall_index}: {bucket.get('n', 0)} samples, " + ", ".join(parts)
