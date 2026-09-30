"""Report whether this machine can run the sensor processing.

Checks the three things SensorDataProcessing needs and the FSM cannot verify
at boot without paying the import cost: the Python dependencies of the
vendored pipelines, the model weights, and the data folders. Exit status 0 when
everything is in place, 1 when the GPR stack is missing but the HSI/POKEYE
chain works (the mission still runs, without GPR), 2 when the HSI chain itself
is broken.

    ros2 run task_planner_fsm check_sensor_setup
"""

import argparse
import importlib
import sys

from .sensors import VendorUnavailable, import_vendor, paths

_MODULES = {
    "shared": ["numpy", "pandas", "scipy", "yaml", "PIL", "matplotlib", "cv2"],
    "hsi": ["sklearn", "xgboost", "joblib"],
    "gpr": ["obspy", "torch", "torchvision", "openpyxl"],
}


def _check_module(name):
    try:
        mod = importlib.import_module(name)
    except Exception as exc:                    # noqa: BLE001
        return False, f"{type(exc).__name__}: {exc}"
    return True, getattr(mod, "__version__", "")


# Entry points the adapters call by name. An older delivery dropped in over
# the vendor folder imports fine and then fails at the first wall, so the
# version is checked by the function that has to be there, not by a number.
_VENDOR_ENTRY_POINTS = {
    "pokeye_decision": ("decide_pokeye", "build_gpr_drilling_constraints"),
    "hsi_integration": ("run_hyperspectral_pipeline",),
    "line_integration": ("run_line_pipeline",),
    "gpr_integration": ("run_gpr_pipeline",),
}


def _check_vendor(name):
    try:
        module = import_vendor(name)
    except VendorUnavailable as exc:
        return False, str(exc)
    except Exception as exc:                    # noqa: BLE001
        return False, f"{type(exc).__name__}: {exc}"
    absent = [fn for fn in _VENDOR_ENTRY_POINTS.get(name, ()) if not hasattr(module, fn)]
    if absent:
        return False, (f"imported, but {', '.join(absent)} is missing -- this looks "
                       f"like an older delivery (see sensors/vendor/VERSIONS.md)")
    return True, ""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--data-dir", default=None, help="override sensor_data_dir")
    parser.add_argument("--models-dir", default=None, help="override sensor_models_dir")
    args = parser.parse_args(argv)
    ctx = {k: v for k, v in (("sensor_data_dir", args.data_dir),
                             ("sensor_models_dir", args.models_dir)) if v}

    def line(ok, label, detail=""):
        tag = "OK     " if ok else "MISSING"
        print(f"[{tag}] {label}" + (f": {detail}" if detail else ""))

    print("== Python dependencies ==")
    missing = {group: [] for group in _MODULES}
    for group, names in _MODULES.items():
        for name in names:
            ok, detail = _check_module(name)
            line(ok, f"{name} ({group})", detail if (detail and not ok) else detail)
            if not ok:
                missing[group].append(name)

    print("\n== Vendored pipelines ==")
    vendor_ok = {}
    for name in ("pokeye_decision", "hsi_integration", "line_integration", "gpr_integration"):
        ok, detail = _check_vendor(name)
        vendor_ok[name] = ok
        line(ok, name, detail)

    print("\n== Models ==")
    hsi_model = paths.hsi_model_path(ctx)
    gpr_weights = paths.gpr_weights_path(ctx)
    line(hsi_model.is_file(), f"HSI classifier {hsi_model}")
    line(gpr_weights.is_file(), f"GPR weights {gpr_weights}")

    print("\n== Data folders ==")
    for label, path in (("data dir", paths.data_dir(ctx)),
                        ("raw hyperspectral", paths.raw_hyperspectral_root(ctx)),
                        ("GPR shared inbox", paths.gpr_incoming_dir(ctx)),
                        ("processed", paths.data_dir(ctx) / "processed")):
        line(path.is_dir(), f"{label} {path}")

    try:
        import torch  # noqa: F401
        print("\n== Compute ==")
        print(f"[INFO   ] torch {torch.__version__}, CUDA available: {torch.cuda.is_available()}"
              + (f" ({torch.cuda.get_device_name(0)})" if torch.cuda.is_available() else ""))
    except Exception:                           # noqa: BLE001
        pass

    hsi_ok = (not missing["shared"] and not missing["hsi"] and vendor_ok["hsi_integration"]
              and vendor_ok["pokeye_decision"] and hsi_model.is_file())
    gpr_ok = (not missing["gpr"] and vendor_ok["gpr_integration"]
              and vendor_ok["line_integration"] and gpr_weights.is_file())
    print()
    if hsi_ok and gpr_ok:
        print("All sensor processing available.")
        return 0
    if hsi_ok:
        print("HSI classification + POKEYE decision available; GPR processing NOT available.")
        print("Install: pip install -r requirements-sensors.txt  and copy models/gpr/best.pt")
        return 1
    print("HSI classification NOT available; SensorDataProcessing will skip it.")
    print("Install: pip install -r requirements-sensors.txt  and copy models/hsi/classifier.joblib")
    return 2


if __name__ == "__main__":
    sys.exit(main())
