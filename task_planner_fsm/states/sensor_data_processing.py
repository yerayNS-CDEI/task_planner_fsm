"""Turn what the scan states recorded into results.

Runs as a small phase machine rather than one blocking pass, because it has
three tenants with very different shapes and only the first exists today:

    hyperspectral -> gpr -> external -> done

``hyperspectral`` is local file work, but a mission-sized record is thousands of
samples and labelling each one is a service call, so it is pumped a batch per
tick instead of run to completion in one. ``gpr`` is a placeholder: that data
lives on the Proceq/iPad and never enters ROS, so retrieving it will be a
network fetch with its own waiting and retries -- the phase is here so it drops
in beside the others instead of forcing a restructure. ``external`` is the
existing /sensor_data_processing service call, unchanged.

Keeping every phase inside run() is what makes each tick bounded: the FSM keeps
ticking, the RViz panel gets a live progress count, and a wedged service cannot
freeze the state machine in a callback nothing can interrupt.
"""

import os

import rclpy
from example_interfaces.srv import SetBool

from arm_control.srv import PredictMaterial

from ..state import State
from ..utils import hyperspectral_processing as hp


class SensorDataProcessing(State):
    # Samples processed per tick. Small enough that a tick stays well inside the
    # FSM period even when every sample costs an ML round trip, large enough
    # that a few thousand samples do not take minutes of ticking.
    HYPERSPECTRAL_BATCH = 25

    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None
        self.ml_client = None
        self._phase = "hyperspectral"
        self._processor = None

    def on_enter(self, ctx):
        node = ctx["node"]
        ctx["data_processed"] = False
        ctx["drilling_required"] = False
        ctx["error_triggered"] = False
        self.future = None
        self._processor = None
        self._phase = "hyperspectral"
        node.get_logger().info(f"[{self.name}] Processing recorded sensor data.")

    def run(self, ctx):
        if self._phase == "hyperspectral":
            self._run_hyperspectral(ctx)
            return
        if self._phase == "gpr":
            self._run_gpr(ctx)
            return
        if self._phase == "external":
            self._run_external(ctx)
            return

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        if not ctx.get("data_processed"):
            return None
        if ctx.get("drilling_required"):
            return "SendDataToPokeye"
        return "ArmFolding"

    def on_exit(self, ctx):
        # A pass abandoned mid-way (an error transition, say) must not leave its
        # half-written CSV behind: close() drops the temporary file and keeps
        # whatever report was already on disk.
        if self._processor is not None:
            self._processor.close()
            self._processor = None

    # ------------------------------------------------------------------
    # Phase 1: hyperspectral
    # ------------------------------------------------------------------
    def _run_hyperspectral(self, ctx):
        """Turn the sweep's raw spectra into reflectance, labels and metrics.

        The second half of the collect/process split: ScanWall recorded raw GSM
        counts and the session's GDS/GRF while the robot moved, and everything
        produced here is a pure function of that record. Nothing touches the
        camera, so a camera since switched off -- or a mission re-processed days
        later with a retrained model -- both work.

        Deliberately non-fatal. The GPR is the primary sensor and the drilling
        decision does not depend on material labels, so a failure here is logged
        and the state moves on. The raw record is already on disk and can be
        re-processed by hand at any time: losing the report is recoverable,
        losing the wall is not.
        """
        node = ctx["node"]
        session_dir = ctx.get("hyperspectral_session_dir")
        if not session_dir:
            self._phase = "gpr"          # sampling was disabled, or no sweep ran
            return
        session_dir = os.path.expanduser(str(session_dir))

        if self._processor is None:
            try:
                self._processor = hp.SessionProcessor(
                    session_dir,
                    predict_fn=self._make_predict_fn(ctx),
                    logger=node.get_logger(),
                )
            except Exception as exc:            # noqa: BLE001
                node.get_logger().error(
                    f"[{self.name}] hyperspectral processing could not start: "
                    f"{exc}. The raw record in {session_dir} is intact and can "
                    f"be re-processed."
                )
                ctx["hyperspectral_processed"] = False
                self._phase = "gpr"
                return
            node.get_logger().info(
                f"[{self.name}] processing {self._processor.total} hyperspectral "
                f"samples from {session_dir}"
            )

        self.set_activity(
            ctx,
            f"Processing hyperspectral samples "
            f"({self._processor.processed}/{self._processor.total})",
            progress_current=self._processor.processed,
            progress_total=max(1, self._processor.total),
        )

        batch = int(ctx.get("hyperspectral_batch_size", self.HYPERSPECTRAL_BATCH))
        try:
            self._processor.step(budget=max(1, batch))
        except Exception as exc:                # noqa: BLE001
            node.get_logger().error(
                f"[{self.name}] hyperspectral processing failed: {exc}. The raw "
                f"record in {session_dir} is intact and can be re-processed."
            )
            self._processor.close()
            self._processor = None
            ctx["hyperspectral_processed"] = False
            self._phase = "gpr"
            return

        if not self._processor.done:
            return                              # more batches on later ticks

        try:
            result = self._processor.finish()
        except Exception as exc:                # noqa: BLE001
            node.get_logger().error(
                f"[{self.name}] could not write the hyperspectral report: {exc}")
            self._processor.close()
            self._processor = None
            ctx["hyperspectral_processed"] = False
            self._phase = "gpr"
            return
        self._processor = None
        self._publish_hyperspectral(ctx, result)
        self._phase = "gpr"

    def _publish_hyperspectral(self, ctx, result):
        """Record the outcome in ctx and log the per-wall coverage summary."""
        node = ctx["node"]
        metrics = result["metrics"]
        ctx["hyperspectral_processed"] = True
        ctx["hyperspectral_reflectance_csv"] = result["reflectance_csv"]
        ctx["hyperspectral_metrics_json"] = result["metrics_json"]
        ctx["hyperspectral_totals"] = metrics.totals()

        node.get_logger().info(f"[{self.name}] {metrics.summary_line('mission')}")
        for wall, totals in sorted(metrics.walls().items()):
            node.get_logger().info(
                f"[{self.name}]   wall {wall}: "
                f"{totals['triggered']} points triggered, "
                f"{totals['captured']} captured, "
                f"{totals['capture_failed']} capture failures, "
                f"{totals['skipped']} skipped, "
                f"{totals[hp.ACCEPTED]} accepted, "
                f"{totals['rejected']} rejected "
                f"(yield {totals['yield'] * 100.0:.0f}%)"
            )
        reasons = metrics.reject_reasons()
        if reasons:
            node.get_logger().info(
                f"[{self.name}]   rejections: "
                + ", ".join(f"{k} x{v}" for k, v in reasons.items())
            )
        if result.get("labelling_aborted"):
            node.get_logger().warn(
                f"[{self.name}]   material labels are incomplete: the ML service "
                f"stopped answering part-way. Reflectance and coverage metrics "
                f"are unaffected."
            )
        node.get_logger().info(
            f"[{self.name}] hyperspectral report: {result['reflectance_csv']}")

    def _make_predict_fn(self, ctx):
        """Return a ``predict_fn(vis, nir)`` for the ML service, or None.

        Blocking is acceptable HERE and nowhere else in this integration: the
        robot is stationary, no sweep is in flight, nothing is being timed, and
        the batching in _run_hyperspectral bounds how many of these land in one
        tick. That is exactly what deferring the prediction buys.

        Returns None when the service is absent, so a mission without the ML
        node still produces reflectance and the full coverage metrics; the
        labels can be added later from the same raw file.
        """
        node = ctx["node"]
        if not bool(ctx.get("hyperspectral_predict_material", True)):
            return None
        if self.ml_client is None:
            self.ml_client = node.create_client(
                PredictMaterial,
                str(ctx.get("hyperspectral_ml_service",
                            "hyperspectral/predict_material")),
            )
        timeout = float(ctx.get("hyperspectral_ml_timeout_s", 5.0))
        if not self.ml_client.wait_for_service(timeout_sec=timeout):
            node.get_logger().warn(
                f"[{self.name}] ML service unavailable; writing reflectance and "
                f"metrics without material labels."
            )
            return None

        def predict(vis, nir):
            request = PredictMaterial.Request()
            # float32[] on the wire; the model casts to float64 internally.
            request.vis_spectrum = [float(v) for v in vis]
            request.nir_spectrum = [float(v) for v in nir]
            future = self.ml_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                self.ml_client.remove_pending_request(future)
                # Not a verdict about the sample -- the service did not answer.
                # Raised so the processor can trip its breaker and stop paying
                # this timeout on every remaining sample of the mission.
                raise hp.PredictionUnavailable(f"no reply within {timeout:.1f} s")
            response = future.result()
            if response is None:
                raise hp.PredictionUnavailable("service returned no result")
            return response.material, float(response.confidence)

        return predict

    # ------------------------------------------------------------------
    # Phase 2: GPR
    # ------------------------------------------------------------------
    def _run_gpr(self, ctx):
        """Placeholder for the GPR post-processing.

        Nothing to do yet: the traces never enter ROS -- ScanWall only starts and
        stops the line, and the data stays on the Proceq/iPad. Retrieving and
        processing it will mean a network fetch, so it belongs here, as a phase
        that can wait across ticks, rather than inline in another state.

        Structured as its own phase now so adding it is additive.
        """
        self._phase = "external"

    # ------------------------------------------------------------------
    # Phase 3: the external processing service
    # ------------------------------------------------------------------
    def _run_external(self, ctx):
        """Call /sensor_data_processing and wait for its verdict."""
        node = ctx["node"]
        self.set_activity(ctx, "Processing the scan sensor data")

        if self.future is None:
            if self.client is None:
                self.client = node.create_client(
                    SetBool, "/sensor_data_processing")
            if not self.client.wait_for_service(timeout_sec=2.0):
                node.get_logger().error(
                    f"[{self.name}] Service /sensor_data_processing not available.")
                ctx["error_triggered"] = True
                return
            node.get_logger().info(
                f"[{self.name}] Calling the service /sensor_data_processing")
            request = SetBool.Request()
            request.data = True
            self.future = self.client.call_async(request)
            return

        if not self.future.done():
            return

        result = self.future.result()
        self.future = None
        if result and result.success:
            node.get_logger().info(f"[{self.name}] Sensor data processed correctly.")
            ctx["data_processed"] = True
            ctx["drilling_required"] = True
        else:
            node.get_logger().error(
                f"[{self.name}] Error while processing sensor data.")
            ctx["error_triggered"] = True
        self._phase = "done"
