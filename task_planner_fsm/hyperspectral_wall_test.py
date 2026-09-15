"""Sweep the wall in front of a hand-parked robot with the hyperspectral camera
on, GPR off, and record the raw spectra to disk. Nothing else.

WHAT IT RUNS
------------
The real ScanWall, not a mock of it -- the same unfold, column, plate approach,
lead-in, press and arm sweep the mission runs -- but entered sideways:

    ArmUnfolding  ->  ScanWall  ->  Finished
    (unfolded_front_fsm)   (one partition, hyperspectral sampling)

* The base is taken to be parked at its scan pose already, facing the wall.
  A one-partition wall is synthesised from where it stands (``fsm_node
  --wall-source in-front``) and ``scan_wall_assume_parked`` keeps the base
  there: no map-derived walls, no NavigateToTarget, no partition transit.
* ``hyperspectral_enabled`` is on and ``gpr_enabled`` off. The GPR distance
  triggers stay on -- they are a topic, and the hyperspectral sampler shares
  their sweep frame and axis.
* Only RAW spectra are written (``data/raw/hyperspectral/session_<stamp>/``:
  ``raw_samples.jsonl``, ``calibration.json``, ``metrics.json``). Reflectance
  and material labels are SensorDataProcessing's job, which this run skips by
  default; ``--stop-after SensorDataProcessing`` runs it in place, or process
  the session later with ``ros2 run task_planner_fsm process_sensor_session``.

BEFORE YOU START
----------------
* ``hyperspectral_node`` must be up and calibrated (interactive GDS/GRF). The
  preflight here refuses to move the arm otherwise, because the sweep would
  silently skip every capture ("no calibration cached").
* Park the robot square to the wall at the scan standoff
  (``partition_base_standoff_m``, 1.15 m from the wall face). The arm measures
  the real distance with the plate sensors; the heading and the lateral
  placement of the sweep are trusted from the base pose.
* No map, no localisation: the wall and the sweep are expressed in ``odom``
  (``--world-frame``), which the robot stack publishes from lidar/wheel odometry
  whether or not rtabmap localisation is up, and a parked base does not move
  relative to it. Readiness is gated on ``/tf`` and ``/joint_states`` only.
* The robot stack (``move_robot.launch.py``) is launched by the FSM as usual.
  If it is already running in another terminal -- or you brought up a reduced
  stack without rtabmap -- pass ``--no-launch-stack``.

    ros2 run task_planner_fsm hyperspectral_wall_test --line-z 1.0
    ros2 run task_planner_fsm hyperspectral_wall_test --line-z 0.9 1.4 --no-launch-stack
    ros2 run task_planner_fsm hyperspectral_wall_test --sim true --skip-preflight

Anything after ``--ros-args`` is handed to the FSM node unchanged, so every
ScanWall/sampler knob is reachable:

    ... --ros-args -p hyperspectral_sample_spacing_m:=0.08 -p sweep_speed_mps:=0.04
"""

import argparse
import sys

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from arm_control.srv import HyperspectralCommand

from task_planner_fsm import fsm_node

STOP_AFTER_CHOICES = ("ScanWall", "SensorDataProcessing", "ArmFolding")
DEFAULT_STOP_AFTER = "ScanWall"
DEFAULT_LINE_Z = 1.0
DEFAULT_SERVICE = "hyperspectral/measurement"
DEFAULT_WORLD_FRAME = "odom"


def build_fsm_argv(parsed, extra):
    """Compose fsm_node's argv for the bench run.

    Pure, so it can be pinned by a test: the whole point of the tool is that
    these flags are the right ones, and getting one of them wrong (the pose,
    the parked knob, the stop state) moves the robot in a way the operator did
    not ask for.

    ``extra`` is whatever followed the tool's own options; a leading
    ``--ros-args`` is absorbed so the caller's parameters join ours in one group.
    """
    params = [
        ("hyperspectral_enabled", "true"),
        ("gpr_enabled", "false"),
        # The arm sweeps and the base parks square to the wall; that is what
        # makes the front-unfolded plate face the wall (utils.wall_approach).
        ("sweep_use_arm", "true"),
        ("nav_face_wall", "true"),
        ("scan_wall_unfolded_pose", "unfolded_front_fsm"),
        ("scan_wall_assume_parked", "true"),
        # Fixed frame for the wall, the sweep goal and the recorded poses.
        # odom needs no localisation; the base is parked, so it is as good as
        # map for the duration of a sweep.
        ("scan_world_frame", parsed.world_frame),
        # Gate readiness on what the sweep needs, not on rtabmap's odometry
        # topic, so a stack without rtabmap passes.
        ("stack_ready_topics", "[/tf,/joint_states]"),
        ("bootstrap_wall_length_m", f"{float(parsed.length):.3f}"),
        ("bootstrap_wall_lines_z",
         "[" + ",".join(f"{float(z):.3f}" for z in parsed.line_z) + "]"),
    ]
    if parsed.spacing is not None:
        params.append(("hyperspectral_sample_spacing_m", f"{float(parsed.spacing):.4f}"))
    if parsed.min_period is not None:
        params.append(("hyperspectral_min_sample_period_s", f"{float(parsed.min_period):.3f}"))
    if parsed.speed is not None:
        params.append(("sweep_speed_mps", f"{float(parsed.speed):.4f}"))
    if parsed.service != DEFAULT_SERVICE:
        params.append(("hyperspectral_service", parsed.service))

    argv = [
        "--sim", "true" if parsed.sim else "false",
        "--initial-state", "ArmUnfolding",
        "--scan-phase", "1",
        "--wall-source", "in-front",
        "--stop-after", parsed.stop_after,
    ]
    if parsed.no_launch_stack:
        argv.append("--no-launch-stack")

    argv.append("--ros-args")
    for name, value in params:
        argv += ["-p", f"{name}:={value}"]
    extra = list(extra)
    if extra and extra[0] == "--ros-args":
        extra = extra[1:]
    return argv + extra


class _Preflight(Node):
    """One-shot camera check on its own rclpy context.

    Its own context so it can be shut down cleanly before the FSM node
    initialises the default one -- re-initialising a context that was shut
    down is not something rclpy promises to support.
    """

    def __init__(self, service_name, context):
        super().__init__("hyperspectral_wall_test_preflight", context=context)
        self.client = self.create_client(HyperspectralCommand, service_name)
        self._spinner = SingleThreadedExecutor(context=context)
        self._spinner.add_node(self)

    def request(self, command, timeout_s):
        request = HyperspectralCommand.Request()
        request.command = command
        future = self.client.call_async(request)
        self._spinner.spin_until_future_complete(future, timeout_sec=timeout_s)
        if not future.done():
            self.client.remove_pending_request(future)
            return None
        return future.result()


def preflight(service_name, wait_s=10.0):
    """True when the camera service answers and holds a GDS + GRF calibration.

    Same two meta commands the sweep sampler fetches on the pre-approach
    ticks; asking here means an uncalibrated camera is found before the
    dashboard play and the unfold, not after.
    """
    context = Context()
    rclpy.init(args=[], context=context)
    node = None
    try:
        node = _Preflight(service_name, context)
        log = node.get_logger()
        log.info(f"Preflight: waiting up to {wait_s:.0f}s for {service_name} ...")
        if not node.client.wait_for_service(timeout_sec=wait_s):
            log.error(
                f"{service_name} never appeared. Start hyperspectral_node and run "
                f"its interactive GDS/GRF calibration first."
            )
            return False
        ok = True
        for command in ("GET_GDS", "GET_GRF"):
            result = node.request(command, timeout_s=10.0)
            if result is None or not (result.vis_ok and result.nir_ok):
                message = getattr(result, "message", "no response")
                log.error(
                    f"{command} unavailable ({message}). Run the interactive "
                    f"GDS/GRF calibration on hyperspectral_node before sweeping."
                )
                ok = False
            else:
                log.info(f"Preflight: {command} present.")
        return ok
    finally:
        if node is not None:
            node._spinner.remove_node(node)
            node.destroy_node()
        rclpy.shutdown(context=context)


def main(args=None):
    parser = argparse.ArgumentParser(
        prog="hyperspectral_wall_test",
        description="Unfold to unfolded_front_fsm and sweep one partition of the "
                    "wall in front of the (already parked) robot with the "
                    "hyperspectral camera sampling. GPR off. Raw spectra only.",
    )
    parser.add_argument("--sim", type=str, default="false", choices=["true", "false"],
                        help="simulation mode, as for fsm_node (default: false)")
    parser.add_argument("--line-z", type=float, nargs="+", default=[DEFAULT_LINE_Z],
                        metavar="Z",
                        help="map-frame height(s) of the scan line(s); the column "
                             "brings the plate there. Several values sweep every "
                             f"height at this one base stop (default: {DEFAULT_LINE_Z})")
    parser.add_argument("--length", type=float, default=0.8,
                        help="scan-line length in metres, centred on the robot. "
                             "Keep it within one partition "
                             "(partition_max_length_m, 0.8): the base never moves "
                             "(default: 0.8)")
    parser.add_argument("--stop-after", type=str, default=DEFAULT_STOP_AFTER,
                        choices=STOP_AFTER_CHOICES,
                        help="last state to run before Finished. ScanWall leaves "
                             "only raw spectra on disk; SensorDataProcessing also "
                             "computes reflectance and labels; ArmFolding folds "
                             f"the arm afterwards (default: {DEFAULT_STOP_AFTER})")
    parser.add_argument("--no-launch-stack", action="store_true",
                        help="the robot stack (move_robot.launch.py) is already "
                             "running; attach instead of launching it")
    parser.add_argument("--skip-preflight", action="store_true",
                        help="do not check the camera service and its GDS/GRF "
                             "calibration before starting")
    parser.add_argument("--spacing", type=float, default=None,
                        help="hyperspectral_sample_spacing_m override (metres of "
                             "plate travel between captures; default 0.10)")
    parser.add_argument("--min-period", type=float, default=None,
                        help="hyperspectral_min_sample_period_s override "
                             "(seconds between captures; default 1.5)")
    parser.add_argument("--speed", type=float, default=None,
                        help="sweep_speed_mps override (default 0.05)")
    parser.add_argument("--service", type=str, default=DEFAULT_SERVICE,
                        help="hyperspectral measurement service name")
    parser.add_argument("--world-frame", type=str, default=DEFAULT_WORLD_FRAME,
                        help="fixed frame the wall and sweep are expressed in. "
                             "odom needs no map or localisation "
                             f"(default: {DEFAULT_WORLD_FRAME}; use map to "
                             "record poses a mission could reuse)")

    argv = args if args is not None else sys.argv[1:]
    parsed, extra = parser.parse_known_args(argv)
    parsed.sim = parsed.sim.lower() == "true"

    if not parsed.skip_preflight and not preflight(parsed.service):
        return 1

    fsm_argv = build_fsm_argv(parsed, extra)
    print("[hyperspectral_wall_test] fsm_node " + " ".join(fsm_argv), flush=True)
    fsm_node.main(fsm_argv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
