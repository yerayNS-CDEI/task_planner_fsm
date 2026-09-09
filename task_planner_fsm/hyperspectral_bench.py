"""Measure the hyperspectral camera's GSM round-trip, and size the sweep
sampler's parameters from it.

WHY THIS EXISTS
---------------
``hyperspectral_sample_spacing_m`` and ``hyperspectral_min_sample_period_s`` are
the one input the sweep design cannot derive from the code. They depend on how
long one GSM actually takes -- integration time, TCP framing, the node's
internal retries -- on the machine that will be running it, under the load it
will be running under. This tool measures that and prints the parameters.

HOW TO RUN IT
-------------
Twice, on the Jetson, and compare:

1. Idle, with only ``hyperspectral_node`` running. This is what the camera can
   do with the whole machine to itself.
2. **With Nav2 and the FSM stack up.** This is the number the parameters must be
   sized from, and it is the one that has never been measured.

The gap between them answers the open question about the protocol's timing:
every margin in the TCP layer is a wall-clock ``time.sleep()`` tuned on a
desktop, and ``flush_buffer`` uses a 10 ms socket timeout to mean "the queue is
empty". On a loaded Jetson, 10 ms of scheduling delay is routine. If the loaded
success rate is materially worse than the idle one, that is why.

Captures are fired ONE AT A TIME, never overlapped: the node holds the sensors
in a mutually-exclusive callback group, so concurrent requests would measure
queueing, not the camera.

    ros2 run task_planner_fsm hyperspectral_bench --count 100 --speed 0.05
"""

import argparse
import csv
import os
import statistics
import sys
import time

import rclpy
from rclpy.node import Node

from arm_control.srv import HyperspectralCommand

from task_planner_fsm.utils import hyperspectral_processing as hp


class HyperspectralBench(Node):
    def __init__(self, service_name):
        super().__init__("hyperspectral_bench")
        self.client = self.create_client(HyperspectralCommand, service_name)
        self.service_name = service_name

    def wait_for_service(self, timeout_s):
        return self.client.wait_for_service(timeout_sec=timeout_s)

    def request(self, command, timeout_s):
        """Send one command and time the round trip.

        Returns ``(elapsed_s, result)``; ``result`` is None on timeout, which
        classify_capture reports as FAILED_NO_RESPONSE -- the same outcome the
        sweep sampler would record.
        """
        request = HyperspectralCommand.Request()
        request.command = command
        started = time.monotonic()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        elapsed = time.monotonic() - started
        if not future.done():
            # Leave nothing in flight to land during the next measurement.
            self.client.remove_pending_request(future)
            return elapsed, None
        return elapsed, future.result()

    def check_calibration(self):
        """Confirm GDS and GRF are in the node's memory.

        Timing an uncalibrated camera measures the right thing for the wrong
        session: the sweep will refuse to sample without these, so a bench run
        that ignores them can bless parameters for a mission that cannot start.
        """
        ok = True
        for command in ("GET_GDS", "GET_GRF"):
            _, result = self.request(command, timeout_s=10.0)
            if result is None or not (result.vis_ok and result.nir_ok):
                message = getattr(result, "message", "no response")
                self.get_logger().error(
                    f"{command} unavailable ({message}). Run the interactive "
                    f"GDS/GRF calibration on hyperspectral_node first."
                )
                ok = False
            else:
                self.get_logger().info(f"{command} present.")
        return ok


def _percentile(values, fraction):
    """Nearest-rank percentile. No numpy interpolation games on ~100 samples."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round(fraction * len(ordered))) - 1))
    return ordered[index]


def _report(node, samples, speed_mps, spacing_floor):
    """Print the timing summary and the parameters it implies."""
    latencies = [s["elapsed"] for s in samples]
    good = [s["elapsed"] for s in samples if s["outcome"] == hp.OK]
    counts = {}
    for sample in samples:
        counts[sample["outcome"]] = counts.get(sample["outcome"], 0) + 1

    total = len(samples)
    ok_count = counts.get(hp.OK, 0)
    success_rate = (ok_count / total) if total else 0.0

    log = node.get_logger()
    log.info("=" * 68)
    log.info(f"GSM round trip over {total} captures")
    log.info("=" * 68)
    log.info(f"  success rate : {ok_count}/{total} ({success_rate * 100.0:.1f}%)")
    for outcome, count in sorted(counts.items(), key=lambda kv: -kv[1]):
        if outcome != hp.OK:
            log.info(f"  {outcome:<20}: {count}")

    if not latencies:
        log.error("No captures completed; nothing to size the parameters from.")
        return 1

    # Latency of SUCCESSFUL captures drives the parameters -- that is the work
    # the sweep actually waits on. All-sample latency is reported next to it
    # because a high failure rate with fast failures would otherwise look fast.
    basis = good or latencies
    p50 = _percentile(basis, 0.50)
    p95 = _percentile(basis, 0.95)
    log.info(f"  min / mean   : {min(basis):.3f} s / {statistics.fmean(basis):.3f} s")
    log.info(f"  p50 / p95    : {p50:.3f} s / {p95:.3f} s")
    log.info(f"  max          : {max(basis):.3f} s")
    if good and len(good) != total:
        log.info(f"  (all captures incl. failures: mean "
                 f"{statistics.fmean(latencies):.3f} s, max {max(latencies):.3f} s)")

    # --- the parameters ---
    # The period floor is p95, not the mean: the sampler holds one capture in
    # flight and skips any trigger that arrives while it is busy, so sizing on
    # the average would turn the slow tail into skipped points rather than slow
    # ones. Rounded up to a tenth so the recommendation is a number worth typing.
    period = max(0.1, round(p95 + 0.05, 1))
    # And the spacing has to be at least as far as the plate travels in that
    # time, or the sweep asks for samples faster than the camera can answer.
    spacing = max(spacing_floor, speed_mps * period)
    spacing = round(spacing + 0.004, 2)

    log.info("-" * 68)
    log.info(f"Recommended for a sweep at {speed_mps:.3f} m/s:")
    log.info(f"  -p hyperspectral_min_sample_period_s:={period}")
    log.info(f"  -p hyperspectral_sample_spacing_m:={spacing}")
    log.info(f"  ({spacing / speed_mps:.1f} s of plate travel between samples, "
             f"p95 round trip {p95:.2f} s)")
    if success_rate < 0.9:
        log.warn(
            f"Success rate is {success_rate * 100.0:.0f}%. The sweep degrades "
            f"rather than aborting, so this shows up as gaps in coverage, not a "
            f"failed mission -- but check the failure breakdown above before "
            f"trusting a wall scanned at this rate."
        )
    log.info("=" * 68)
    return 0


def _write_csv(path, samples):
    path = os.path.expanduser(path)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "elapsed_s", "outcome", "detail"])
        for sample in samples:
            writer.writerow([sample["index"], f"{sample['elapsed']:.4f}",
                             sample["outcome"], sample["detail"]])
    return path


def main(args=None):
    parser = argparse.ArgumentParser(
        prog="hyperspectral_bench",
        description="Time the hyperspectral GSM round trip and size the sweep "
                    "sampler's parameters from it.",
    )
    parser.add_argument("--count", type=int, default=50,
                        help="number of GSM captures to time (default: 50)")
    parser.add_argument("--speed", type=float, default=0.05,
                        help="sweep speed in m/s the spacing is computed for "
                             "(default: 0.05)")
    parser.add_argument("--interval", type=float, default=0.0,
                        help="idle seconds between captures (default: 0, "
                             "back to back)")
    parser.add_argument("--timeout", type=float, default=90.0,
                        help="per-capture timeout in seconds. The node retries a "
                             "GSM three times internally, so a real reply can "
                             "legitimately take tens of seconds (default: 90)")
    parser.add_argument("--min-spacing", type=float, default=0.05,
                        help="floor for the recommended spacing in metres "
                             "(default: 0.05)")
    parser.add_argument("--service", type=str, default="hyperspectral/measurement",
                        help="measurement service name")
    parser.add_argument("--output", type=str, default=None,
                        help="write per-capture timings to this CSV")
    parser.add_argument("--skip-calibration-check", action="store_true",
                        help="do not verify GDS/GRF are loaded before timing")

    argv = args if args is not None else sys.argv[1:]
    parsed, remaining = parser.parse_known_args(argv)

    rclpy.init(args=remaining)
    node = HyperspectralBench(parsed.service)
    status = 0
    try:
        node.get_logger().info(f"Waiting for {parsed.service} ...")
        if not node.wait_for_service(10.0):
            node.get_logger().error(
                f"{parsed.service} never appeared. Is hyperspectral_node running?")
            return 1
        if not parsed.skip_calibration_check and not node.check_calibration():
            return 1

        node.get_logger().info(
            f"Timing {parsed.count} GSM captures, one at a time"
            + (f", {parsed.interval:.2f} s apart" if parsed.interval else
               ", back to back") + " ..."
        )
        samples = []
        for index in range(1, parsed.count + 1):
            elapsed, result = node.request("GSM", parsed.timeout)
            outcome, detail, _, _ = hp.classify_capture(result)
            samples.append({"index": index, "elapsed": elapsed,
                            "outcome": outcome, "detail": detail})
            marker = "ok " if outcome == hp.OK else "FAIL"
            node.get_logger().info(
                f"  [{index:>4}/{parsed.count}] {marker} {elapsed:6.3f} s"
                + (f"  {outcome}: {detail}" if outcome != hp.OK else "")
            )
            if parsed.interval > 0.0 and index < parsed.count:
                time.sleep(parsed.interval)

        if parsed.output:
            node.get_logger().info(f"Per-capture timings: "
                                   f"{_write_csv(parsed.output, samples)}")
        status = _report(node, samples, parsed.speed, parsed.min_spacing)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
