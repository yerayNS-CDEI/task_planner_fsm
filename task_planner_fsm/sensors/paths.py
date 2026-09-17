"""Where the sensor processing reads and writes.

One module answers every "where is it?" question so the raw recorders (sweep
time), the processing state (post-sweep) and the offline CLI cannot drift onto
different directories. Everything hangs off the package root, next to
``launch/`` and ``scripts/``::

    task_planner_fsm/
    ├── data/       raw/  processed/      <- sensor_data_dir
    ├── models/     gpr/  hsi/            <- sensor_models_dir
    └── task_planner_fsm/sensors/vendor/  <- the delivered pipelines

The package is built with ``--symlink-install``, so ``__file__`` resolves into
the source tree and these folders are used in place -- no copy of a 241 MB
weight file on every build. On a machine where the layout differs (the Jetson),
``sensor_data_dir`` / ``sensor_models_dir`` move the roots and
``gpr_weights_path`` / ``hsi_model_path`` pin single files.

No ROS in here: ``ctx`` is only ever read with ``.get``, so the same functions
serve the FSM node and the command-line tools.
"""

import os
from datetime import datetime
from pathlib import Path

# .../task_planner_fsm/task_planner_fsm/sensors/paths.py -> .../task_planner_fsm
PACKAGE_ROOT = Path(__file__).resolve().parents[2]
VENDOR_DIR = Path(__file__).resolve().parent / "vendor"

GPR_MANIFEST_FILENAME = "gpr_lines.jsonl"


def _get(ctx, key, default=None):
    if ctx is None:
        return default
    value = ctx.get(key)
    return default if value in (None, "") else value


def _expand(value):
    return Path(os.path.expanduser(str(value)))


# ----------------------------------------------------------------------
# Roots
# ----------------------------------------------------------------------
def data_dir(ctx=None) -> Path:
    """Root of the runtime data tree (``sensor_data_dir`` overrides)."""
    return _expand(_get(ctx, "sensor_data_dir", PACKAGE_ROOT / "data"))


def models_dir(ctx=None) -> Path:
    """Root of the model weights (``sensor_models_dir`` overrides)."""
    return _expand(_get(ctx, "sensor_models_dir", PACKAGE_ROOT / "models"))


# ----------------------------------------------------------------------
# Session
# ----------------------------------------------------------------------
def new_session_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def session_id(ctx=None) -> str:
    """The mission's stamp, shared by ``raw/`` and ``processed/``.

    Resolved once and cached in ``ctx["sensor_session_id"]``. If the
    hyperspectral sampler already opened its session (it names the directory
    ``session_<stamp>``), that stamp is adopted so both halves of the record sit
    side by side; otherwise a fresh one is minted.
    """
    if ctx is None:
        return new_session_stamp()
    existing = ctx.get("sensor_session_id")
    if existing:
        return str(existing)
    hs_dir = ctx.get("hyperspectral_session_dir")
    if hs_dir:
        name = Path(str(hs_dir)).name
        stamp = name[len("session_"):] if name.startswith("session_") else name
    else:
        stamp = new_session_stamp()
    ctx["sensor_session_id"] = stamp
    return stamp


# ----------------------------------------------------------------------
# Raw (sweep-time) locations
# ----------------------------------------------------------------------
def raw_hyperspectral_root(ctx=None) -> Path:
    """Directory holding the ``session_<stamp>`` hyperspectral records."""
    return data_dir(ctx) / "raw" / "hyperspectral"


def raw_session_dirs(ctx=None):
    """Every ``session_<stamp>`` under the raw hyperspectral root, oldest first.

    Stamps are ``YYYYMMDD_HHMMSS`` so lexical order is chronological; the
    directories the sampler creates always carry one, and anything else in
    that folder (the camera node's own CSVs, notes) is not a session.
    """
    root = raw_hyperspectral_root(ctx)
    if not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if p.is_dir() and p.name.startswith("session_"))


def latest_raw_session_dir(ctx=None):
    """The most recent recorded session, or None when there is none yet."""
    sessions = raw_session_dirs(ctx)
    return sessions[-1] if sessions else None


def gpr_incoming_dir(ctx=None) -> Path:
    """The shared inbox for hand-copied GP8800 exports (``.sgy`` + ``.csv``)
    (``gpr_incoming_dir``). Anything here is offered to EVERY session's
    processing; the exports ScanWall pulls itself go to the session's own
    folder instead (``gpr_session_incoming_dir``)."""
    return _expand(_get(ctx, "gpr_incoming_dir", data_dir(ctx) / "raw" / "gpr" / "incoming"))


def gpr_session_dir(ctx=None) -> Path:
    """This mission's GPR folder: manifest, export zips, unpacked scans."""
    return data_dir(ctx) / "raw" / "gpr" / f"session_{session_id(ctx)}"


def gpr_session_incoming_dir(ctx=None) -> Path:
    """Where ScanWall unpacks this mission's own exports. Per session, so a
    ``w00_l00_s00`` from yesterday can never be mistaken for today's."""
    return gpr_session_dir(ctx) / "incoming"


def gpr_incoming_dirs(ctx=None):
    """The folders SensorDataProcessing reads scans from, own session first."""
    return [gpr_session_incoming_dir(ctx), gpr_incoming_dir(ctx)]


def gpr_manifest_path(ctx=None) -> Path:
    """The per-mission record of GPR lines written by ScanWall."""
    return gpr_session_dir(ctx) / GPR_MANIFEST_FILENAME


# ----------------------------------------------------------------------
# Processed (post-sweep) locations
# ----------------------------------------------------------------------
def processed_dir(ctx=None) -> Path:
    """``processed/session_<stamp>`` for this mission (``sensor_results_dir`` overrides)."""
    override = _get(ctx, "sensor_results_dir")
    if override:
        return _expand(override)
    return data_dir(ctx) / "processed" / f"session_{session_id(ctx)}"


def hsi_results_dir(ctx=None) -> Path:
    return processed_dir(ctx) / "hsi"


def gpr_results_dir(ctx=None) -> Path:
    return processed_dir(ctx) / "gpr"


def pokeye_results_dir(ctx=None) -> Path:
    return processed_dir(ctx) / "pokeye"


# ----------------------------------------------------------------------
# Models
# ----------------------------------------------------------------------
def gpr_weights_path(ctx=None) -> Path:
    """Mask R-CNN weights (``gpr_weights_path`` overrides the file)."""
    return _expand(_get(ctx, "gpr_weights_path", models_dir(ctx) / "gpr" / "best.pt"))


def hsi_model_path(ctx=None) -> Path:
    """Benjamin's classifier bundle (``hsi_model_path`` overrides the file)."""
    return _expand(_get(ctx, "hsi_model_path", models_dir(ctx) / "hsi" / "classifier.joblib"))


# ----------------------------------------------------------------------
# Vendor projects
# ----------------------------------------------------------------------
GPR_PROJECT = VENDOR_DIR / "gpr_pipeline"
HYPERBOLA_PROJECT = GPR_PROJECT / "Hyperbola_Segmentation"
LINE_PROJECT = GPR_PROJECT / "Line_Segmentation"
HSI_PROJECT = VENDOR_DIR / "hsi_pipeline"
POKEYE_PROJECT = VENDOR_DIR / "pokeye_decision_pipeline"
