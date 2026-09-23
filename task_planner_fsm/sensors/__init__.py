"""FSM-side adapters around the DISCOVER sensor pipelines.

The pipelines under ``vendor/`` are plain-Python, file-in / dict-out libraries
delivered by the sensor team (see ``vendor/VERSIONS.md``). They know nothing
about ROS, robot poses or the FSM; the modules here supply that:

    hsi.py      the recorded sweep reflectance -> material per sample
    gpr.py      exported GP8800 scans -> hyperbolae/lines, mapped onto the wall
    pokeye.py   per-sample decisions -> clustered drilling targets
    no_drill.py GPR hyperbolae -> map-frame zones no drill target may enter
    paths.py    the one place that knows where data and models live
    manifest.py the GPR line record ScanWall writes while sweeping
    background_job.py  run a pipeline off the FSM tick

Importing this package is cheap on purpose. The heavy dependencies (torch,
obspy, xgboost) are only pulled in by :func:`import_vendor`, from inside the
processing state, so an FSM without them installed still boots and merely
reports the sensor phases as unavailable.
"""

import importlib
import sys

from . import paths

# Each delivered project imports its siblings by bare top-level name
# (``from gpr_integration import ...``, ``from GPRTools.edits ...``) after
# inserting its own root into sys.path. Doing that insertion here, once, means
# the vendor code runs exactly as delivered and nothing else has to know about
# the layout.
_VENDOR_ROOTS = (
    paths.GPR_PROJECT,          # GPRTools
    paths.HYPERBOLA_PROJECT,    # gpr_integration
    paths.LINE_PROJECT,         # line_integration
    paths.HSI_PROJECT,          # hsi_integration
    paths.POKEYE_PROJECT,       # pokeye_decision
)


def _numpy2_pickle_compat():
    """Let numpy-1.x load pickles written by numpy 2.

    The delivered ``classifier.joblib`` was saved with numpy >= 2, whose
    internals live under ``numpy._core``; ROS Humble ships numpy 1.24, where
    the same modules are ``numpy.core``. Unpickling only needs the names to
    resolve, so aliasing the old modules under the new names is enough. No-op
    on numpy 2.
    """
    import numpy
    if hasattr(numpy, "_core") or "numpy._core" in sys.modules:
        return
    import numpy.core as core
    sys.modules["numpy._core"] = core
    for sub in ("multiarray", "numeric", "umath", "_multiarray_umath"):
        try:
            sys.modules[f"numpy._core.{sub}"] = importlib.import_module(f"numpy.core.{sub}")
        except ImportError:
            pass


def bootstrap_vendor_path():
    """Make the vendored projects importable by their delivered names. Idempotent."""
    _numpy2_pickle_compat()
    for root in _VENDOR_ROOTS:
        entry = str(root)
        if entry not in sys.path:
            sys.path.append(entry)


class VendorUnavailable(RuntimeError):
    """A delivered pipeline cannot be imported (missing dependency or file).

    Raised instead of the bare ImportError so callers can tell "this machine
    lacks torch" from a bug, and degrade to skipping the phase.
    """


def import_vendor(module_name):
    """Import a vendored module (``"gpr_integration"``, ``"hsi_integration"``...).

    Wraps ImportError/OSError in :class:`VendorUnavailable` with the dependency
    named, since a Mask R-CNN import failing on a missing ``torch`` should read
    as "install requirements-sensors.txt", not as a traceback.
    """
    bootstrap_vendor_path()
    try:
        return importlib.import_module(module_name)
    except ImportError as exc:
        raise VendorUnavailable(
            f"cannot import {module_name!r}: {exc}. Install the sensor "
            f"dependencies with: pip install -r requirements-sensors.txt"
        ) from exc
    except OSError as exc:              # a shared library that fails to load
        raise VendorUnavailable(f"cannot load {module_name!r}: {exc}") from exc


def require(*module_names):
    """Fail early with :class:`VendorUnavailable` if a dependency is missing.

    The vendored packages import their heavy dependencies lazily (torch when
    the model is loaded, xgboost when joblib unpickles the classifier), so a
    successful ``import_vendor`` proves nothing. The adapters call this before
    starting work so a missing library is reported as one clear line, not as
    a traceback from deep inside a pipeline.
    """
    missing = []
    for name in module_names:
        try:
            importlib.import_module(name)
        except ImportError:
            missing.append(name)
    if missing:
        raise VendorUnavailable(
            f"missing Python package(s): {', '.join(missing)}. Install them with: "
            f"pip install -r requirements-sensors.txt"
        )
