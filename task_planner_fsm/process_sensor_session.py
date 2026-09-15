"""Run the SensorDataProcessing chain over a recorded session, offline.

The same code the FSM state runs, minus ROS, in the foreground: reflectance
pass, DISCOVER material classification, GPR pipelines over the incoming
exports, POKEYE decisions and target clustering. For re-processing a mission
after the fact (a retrained model, a tuned clustering radius), for testing the
sensor stack on a machine without the robot, and for the Jetson timing work.

    ros2 run task_planner_fsm process_sensor_session 20260914_153000
    ros2 run task_planner_fsm process_sensor_session data/raw/hyperspectral/session_20260914_153000 --wall 2
    ros2 run task_planner_fsm process_sensor_session <session> --skip-gpr --radius 0.2

Results land in ``data/processed/session_<stamp>/`` exactly as they would
during a mission (``--out`` overrides).
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path

from .sensors import VendorUnavailable, gpr, hsi, paths, pokeye
from .utils import hyperspectral_processing as hp


class _Logger:
    """The four-method logger the sensor modules expect, over ``logging``."""

    def __init__(self):
        logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
        self._log = logging.getLogger("process_sensor_session")

    def info(self, msg, **kwargs):
        self._log.info(msg)

    def warn(self, msg, **kwargs):
        self._log.warning(msg)

    def error(self, msg, **kwargs):
        self._log.error(msg)

    def debug(self, msg, **kwargs):
        self._log.debug(msg)


def _resolve_session(arg, ctx):
    """Accept a stamp, ``session_<stamp>`` or a path to the raw session dir."""
    p = Path(os.path.expanduser(arg))
    if p.is_dir():
        return p
    name = arg if arg.startswith("session_") else f"session_{arg}"
    candidate = paths.raw_hyperspectral_root(ctx) / name
    if candidate.is_dir():
        return candidate
    raise SystemExit(f"session not found: {arg} (looked in {paths.raw_hyperspectral_root(ctx)})")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("session", help="session stamp, session_<stamp>, or raw session directory")
    parser.add_argument("--wall", type=int, default=None,
                        help="cluster POKEYE targets for this wall only (default: all walls)")
    parser.add_argument("--out", default=None, help="results directory (default: data/processed/session_<stamp>)")
    parser.add_argument("--data-dir", default=None, help="override sensor_data_dir")
    parser.add_argument("--models-dir", default=None, help="override sensor_models_dir")
    parser.add_argument("--threshold", type=float, default=hsi.DEFAULT_CONFIDENCE_THRESHOLD,
                        help="HSI confidence threshold (default %(default)s)")
    parser.add_argument("--hsi-device", default=hsi.DEFAULT_DEVICE,
                        help="where XGBoost predicts, cpu or cuda (default %(default)s)")
    parser.add_argument("--skip-reflectance", action="store_true",
                        help="reuse the existing reflectance.csv")
    parser.add_argument("--skip-gpr", action="store_true", help="do not run the GPR pipelines")
    parser.add_argument("--gpr-incoming", default=None, help="folder with the .sgy/.csv exports")
    parser.add_argument("--radius", type=float, default=None, help="pokeye_cluster_radius_m")
    parser.add_argument("--min-samples", type=int, default=None, help="pokeye_cluster_min_samples")
    parser.add_argument("--spacing", type=float, default=None, help="pokeye_min_target_spacing_m")
    parser.add_argument("--max-targets", type=int, default=None, help="pokeye_max_targets_per_wall")
    args = parser.parse_args(argv)

    log = _Logger()
    ctx = {}
    for key, value in (("sensor_data_dir", args.data_dir), ("sensor_models_dir", args.models_dir),
                       ("sensor_results_dir", args.out), ("gpr_incoming_dir", args.gpr_incoming),
                       ("pokeye_cluster_radius_m", args.radius),
                       ("pokeye_cluster_min_samples", args.min_samples),
                       ("pokeye_min_target_spacing_m", args.spacing),
                       ("pokeye_max_targets_per_wall", args.max_targets)):
        if value is not None:
            ctx[key] = value

    session_dir = _resolve_session(args.session, ctx)
    ctx["hyperspectral_session_dir"] = str(session_dir)
    name = session_dir.name
    ctx["sensor_session_id"] = name[len("session_"):] if name.startswith("session_") else name
    log.info(f"session {session_dir} -> {paths.processed_dir(ctx)}")

    # 1) reflectance
    if args.skip_reflectance and (session_dir / hp.REFLECTANCE_FILENAME).is_file():
        log.info("reflectance: reusing existing reflectance.csv")
    else:
        t0 = time.monotonic()
        result = hp.process_session(str(session_dir), predict_fn=None, logger=log)
        log.info(f"reflectance: {result['processed_samples']} samples in {time.monotonic() - t0:.1f} s")

    # 2) classification
    samples = []
    threshold = args.threshold
    try:
        t0 = time.monotonic()
        result = hsi.classify_session(session_dir, paths.hsi_results_dir(ctx),
                                      paths.hsi_model_path(ctx), threshold, logger=log,
                                      device=args.hsi_device)
        samples = result["samples"]
        log.info(f"hsi: {result['n_classified']} spectra classified in {time.monotonic() - t0:.1f} s")
        for wall, bucket in sorted(result["by_wall"].items(), key=lambda kv: (kv[0] is None, kv[0])):
            log.info("  " + hsi.describe_wall(wall, bucket))
    except (VendorUnavailable, FileNotFoundError) as exc:
        log.warn(f"hsi: skipped ({exc})")

    # 3) GPR
    if not args.skip_gpr:
        try:
            t0 = time.monotonic()
            result = gpr.process_incoming(
                paths.gpr_incoming_dir(ctx), paths.gpr_manifest_path(ctx),
                paths.gpr_results_dir(ctx), paths.gpr_weights_path(ctx), logger=log,
                run_hyperbolae=paths.gpr_weights_path(ctx).is_file())
            log.info(f"gpr: {result['n_new']} new scan(s) in {time.monotonic() - t0:.0f} s, "
                     f"{result['n_hyperbolae']} hyperbolae, {result['n_lines']} lines, "
                     f"{result['n_failed']} failed")
            for entry in result["entries"]:
                log.info("  " + gpr.describe_entry(entry))
        except VendorUnavailable as exc:
            log.warn(f"gpr: skipped ({exc})")

    # 4) decision + targets
    if samples:
        try:
            decisions = pokeye.decide_samples(samples, threshold)
            params = pokeye.ClusterParams.from_ctx(ctx)
            targets, stats = pokeye.cluster_targets(decisions, params, wall_index=args.wall)
            d_path, t_path = pokeye.write_outputs(paths.pokeye_results_dir(ctx), decisions, targets, stats)
            ds = pokeye.decision_stats(decisions)
            log.info(f"pokeye: {ds['n_pokeye_required']}/{ds['n']} samples require POKEYE, "
                     f"{ds['n_hold']} hold -> {stats['n_targets']} target(s) "
                     f"(radius {params.radius_m} m, min {params.min_samples} samples, "
                     f"spacing {params.min_spacing_m} m, cap {params.max_targets_per_wall})")
            log.info(f"  {pokeye.describe_targets(targets)}")
            log.info(f"  {d_path}\n  {t_path}")
        except VendorUnavailable as exc:
            log.warn(f"pokeye: skipped ({exc})")
    else:
        log.info("pokeye: no classified samples, nothing to decide")
    return 0


if __name__ == "__main__":
    sys.exit(main())
