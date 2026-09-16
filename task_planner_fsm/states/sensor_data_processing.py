"""Turn what the scan states recorded into results, and decide about POKEYE.

Runs as a small phase machine rather than one blocking pass, because the
tenants have very different shapes and costs:

    hyperspectral -> hsi_classify -> gpr -> decision -> [external] -> done

``hyperspectral`` is the reflectance pass over the sweep's raw record, pumped a
batch per tick (thousands of samples, cheap each); it is skipped unless the
camera was enabled for the sweep (``hyperspectral_enabled``), so a mission run
without it never re-publishes an older session's spectra. ``hsi_classify`` runs the
DISCOVER material classifier over that reflectance -- one XGBoost call over
the whole session, seconds -- in a background thread. ``gpr`` runs the
delivered hyperbola and line segmentation over whatever GP8800 exports have
reached ``data/raw/gpr/incoming`` (Mask R-CNN: minutes on a CPU), also in a
background thread. ``decision`` applies the sensor team's POKEYE policy to
every classified sample and clusters the flagged ones into a handful of drill
targets for the wall just scanned. ``external`` is the old
/sensor_data_processing mock, kept only for simulation runs with no sensor data
so the FSM still exercises SendDataToPokeye there.

Keeping every phase inside run() is what makes each tick bounded: the FSM
keeps ticking, the RViz panel gets live progress, and neither a wedged service
nor a five-minute model can freeze the state machine.

Every sensor phase is non-fatal. The raw records are on disk and can be
re-processed offline (``ros2 run task_planner_fsm process_sensor_session``);
losing a report is recoverable, losing the wall is not. Only the mock service
path still raises Error, as it always did.
"""

import os
import time

from example_interfaces.srv import SetBool

from ..sensors import VendorUnavailable, gpr, hsi, paths, pokeye
from ..sensors.background_job import BackgroundJob
from ..state import State
from ..utils import hyperspectral_processing as hp


class SensorDataProcessing(State):
    # Samples processed per tick in the reflectance pass. Small enough that a
    # tick stays well inside the FSM period, large enough that a few thousand
    # samples do not take minutes of ticking.
    HYPERSPECTRAL_BATCH = 25

    PHASES = ("hyperspectral", "hsi_classify", "gpr", "decision", "external", "done")

    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None
        self._phase = "hyperspectral"
        self._processor = None
        self._job = None
        self._gpr_wait_started = None
        self._hsi = None            # result of sensors.hsi.classify_session
        self._gpr = None            # result of sensors.gpr.process_incoming

    # ------------------------------------------------------------------
    # State protocol
    # ------------------------------------------------------------------
    def on_enter(self, ctx):
        node = ctx["node"]
        ctx["data_processed"] = False
        ctx["drilling_required"] = False
        ctx["error_triggered"] = False
        ctx["pokeye_targets"] = []
        self.future = None
        self._processor = None
        self._job = None
        self._gpr_wait_started = None
        self._hsi = None
        self._gpr = None
        self._phase = "hyperspectral"
        ctx["sensor_results_dir"] = str(paths.processed_dir(ctx))
        node.get_logger().info(
            f"[{self.name}] Processing recorded sensor data "
            f"(results in {ctx['sensor_results_dir']})."
        )

    def run(self, ctx):
        handler = getattr(self, f"_run_{self._phase}", None)
        if handler is not None:
            handler(ctx)

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
        if self._job is not None and not self._job.done:
            # A thread cannot be killed; it is a daemon and finishes on its own
            # with its files intact. Just stop listening to it.
            ctx["node"].get_logger().warn(
                f"[{self.name}] leaving with the {self._job.name} job still "
                f"running; its output will land on disk but not in ctx."
            )
        self._job = None

    def _advance(self, ctx, next_phase):
        self._phase = next_phase

    # ------------------------------------------------------------------
    # Phase 1: reflectance
    # ------------------------------------------------------------------
    def _hyperspectral_processing_enabled(self, ctx):
        """Whether the hyperspectral record is processed at all.

        The counterpart of ``gpr_processing_enabled``, with one deliberate
        difference: it defaults to the capture flag (``hyperspectral_enabled``,
        off) instead of to True. The GPR phase looks for exports that have not
        been processed yet and simply finds none when the probe was off, but
        this phase processes whatever session directory ctx points at. With the
        camera off that can only be an older session -- a bench run from a
        previous day -- whose reflectance and material labels would then be
        published as if this mission had just measured them.

        An explicit ``hyperspectral_processing_enabled`` wins, which is how a
        deliberate re-processing run (``--initial-state SensorDataProcessing``,
        see fsm_node._bootstrap_sensor_processing) processes a recorded session
        with the camera long gone.
        """
        explicit = ctx.get("hyperspectral_processing_enabled")
        if explicit is not None:
            return bool(explicit)
        return bool(ctx.get("hyperspectral_enabled", False))

    def _run_hyperspectral(self, ctx):
        """Turn the sweep's raw spectra into reflectance and coverage metrics.

        The second half of the collect/process split: ScanWall recorded raw GSM
        counts and the session's GDS/GRF while the robot moved, and everything
        produced here is a pure function of that record. Nothing touches the
        camera, so a camera since switched off -- or a mission re-processed
        days later -- both work.

        Material labels are NOT assigned here any more. The per-sample service
        round trip to arm_control's ml_inference_node is gone; the DISCOVER
        classifier runs over the finished reflectance.csv in the next phase.
        """
        node = ctx["node"]
        if not self._hyperspectral_processing_enabled(ctx):
            node.get_logger().info(
                f"[{self.name}] hyperspectral disabled; skipping reflectance and "
                f"material classification (no camera on this run, so any record on "
                f"disk is from an earlier one)."
            )
            self._advance(ctx, "gpr")
            return
        session_dir = ctx.get("hyperspectral_session_dir")
        if not session_dir:
            self._advance(ctx, "gpr")          # sampling was disabled, or no sweep ran
            return
        session_dir = os.path.expanduser(str(session_dir))

        if self._processor is None:
            try:
                self._processor = hp.SessionProcessor(
                    session_dir, predict_fn=None, logger=node.get_logger())
            except Exception as exc:            # noqa: BLE001
                node.get_logger().error(
                    f"[{self.name}] hyperspectral processing could not start: "
                    f"{exc}. The raw record in {session_dir} is intact and can "
                    f"be re-processed."
                )
                ctx["hyperspectral_processed"] = False
                self._advance(ctx, "gpr")
                return
            node.get_logger().info(
                f"[{self.name}] processing {self._processor.total} hyperspectral "
                f"samples from {session_dir}"
            )

        self.set_activity(
            ctx,
            f"Computing reflectance "
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
            self._advance(ctx, "gpr")
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
            self._advance(ctx, "gpr")
            return
        self._processor = None
        self._publish_hyperspectral(ctx, result)
        self._advance(ctx, "hsi_classify")

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
        node.get_logger().info(
            f"[{self.name}] hyperspectral report: {result['reflectance_csv']}")

    # ------------------------------------------------------------------
    # Phase 2: material classification (DISCOVER HSI pipeline)
    # ------------------------------------------------------------------
    def _run_hsi_classify(self, ctx):
        """Classify every spectrum of the session with Benjamin's classifier.

        One call over the whole reflectance.csv, in a background thread: the
        model is loaded once and XGBoost is vectorised, so the whole session
        costs seconds -- but seconds we still do not spend inside a tick.

        The whole session, not just this wall, because the record spans the
        mission and the classifier is cheap; the decision phase then narrows to
        the wall just scanned.
        """
        node = ctx["node"]
        if self._job is None:
            session_dir = ctx.get("hyperspectral_session_dir")
            if (not self._hyperspectral_processing_enabled(ctx)
                    or not session_dir or not ctx.get("hyperspectral_processed")):
                self._advance(ctx, "gpr")
                return
            model = paths.hsi_model_path(ctx)
            if not model.is_file():
                node.get_logger().error(
                    f"[{self.name}] HSI classifier not found at {model}; skipping "
                    f"material classification (see models/README.md)."
                )
                self._advance(ctx, "gpr")
                return
            node.get_logger().info(f"[{self.name}] classifying materials with {model.name}")
            self._job = BackgroundJob(
                hsi.classify_session,
                os.path.expanduser(str(session_dir)),
                paths.hsi_results_dir(ctx),
                model,
                confidence_threshold=float(
                    ctx.get("hsi_confidence_threshold", hsi.DEFAULT_CONFIDENCE_THRESHOLD)),
                logger=node.get_logger(),
                device=str(ctx.get("hsi_device", hsi.DEFAULT_DEVICE)),
                name="hsi-classify",
            )

        self.set_activity(
            ctx, f"Classifying materials ({self._job.elapsed_s:.0f} s)")
        if not self._job.done:
            return

        job, self._job = self._job, None
        if job.failed:
            level = "warn" if isinstance(job.error, VendorUnavailable) else "error"
            getattr(node.get_logger(), level)(
                f"[{self.name}] material classification skipped: {job.error}")
            if level == "error":
                node.get_logger().debug(job.traceback)
            self._advance(ctx, "gpr")
            return

        self._hsi = job.result
        ctx["hsi_result_json"] = self._hsi["hsi_result_json"]
        ctx["hsi_samples_csv"] = self._hsi["samples_csv"]
        ctx["hsi_samples"] = self._hsi["samples"]
        node.get_logger().info(
            f"[{self.name}] classified {self._hsi['n_classified']} spectra in "
            f"{job.elapsed_s:.1f} s (threshold {self._hsi['confidence_threshold']:.2f})"
        )
        for wall, bucket in sorted(self._hsi["by_wall"].items(),
                                   key=lambda kv: (kv[0] is None, kv[0])):
            node.get_logger().info(f"[{self.name}]   {hsi.describe_wall(wall, bucket)}")
        self._advance(ctx, "gpr")

    # ------------------------------------------------------------------
    # Phase 3: GPR
    # ------------------------------------------------------------------
    def _run_gpr(self, ctx):
        """Run the hyperbola and line pipelines over new GP8800 exports.

        The traces never enter ROS: ScanWall starts and stops the line, the GPR
        API is meant to drop the export into ``data/raw/gpr/incoming``. That
        hand-off is untested, so this phase waits at most ``gpr_wait_timeout_s``
        (default 0: process what is already there) and never blocks the
        mission on a file that may not come.

        Per v1 policy GPR results are stored and logged; they do not vote on
        POKEYE.
        """
        node = ctx["node"]
        if not bool(ctx.get("gpr_processing_enabled", True)):
            self._advance(ctx, "decision")
            return

        if self._job is None:
            incoming = paths.gpr_incoming_dir(ctx)
            out_dir = paths.gpr_results_dir(ctx)
            pending = gpr.pending_files(incoming, out_dir)
            if not pending:
                timeout = float(ctx.get("gpr_wait_timeout_s", 0.0))
                if self._gpr_wait_started is None:
                    self._gpr_wait_started = time.monotonic()
                waited = time.monotonic() - self._gpr_wait_started
                if waited < timeout:
                    self.set_activity(
                        ctx, f"Waiting for GPR exports in {incoming} ({waited:.0f}/{timeout:.0f} s)")
                    return
                node.get_logger().info(
                    f"[{self.name}] no new GPR exports in {incoming}; skipping GPR processing.")
                self._advance(ctx, "decision")
                return

            weights = paths.gpr_weights_path(ctx)
            run_hyp = bool(ctx.get("gpr_run_hyperbolae", True))
            if run_hyp and not weights.is_file():
                node.get_logger().warn(
                    f"[{self.name}] GPR weights not found at {weights}; running the "
                    f"line pipeline only (see models/README.md).")
                run_hyp = False
            node.get_logger().info(
                f"[{self.name}] processing {len(pending)} GPR export(s) from {incoming}")
            self._job = BackgroundJob(
                gpr.process_incoming,
                incoming,
                paths.gpr_manifest_path(ctx),
                out_dir,
                weights,
                logger=node.get_logger(),
                run_hyperbolae=run_hyp,
                run_lines=bool(ctx.get("gpr_run_lines", True)),
                name="gpr-process",
            )

        self.set_activity(ctx, f"Processing GPR scans ({self._job.elapsed_s:.0f} s)")
        if not self._job.done:
            return

        job, self._job = self._job, None
        if job.failed:
            level = "warn" if isinstance(job.error, VendorUnavailable) else "error"
            getattr(node.get_logger(), level)(
                f"[{self.name}] GPR processing skipped: {job.error}")
            if level == "error":
                node.get_logger().debug(job.traceback)
            self._advance(ctx, "decision")
            return

        self._gpr = job.result
        ctx["gpr_results"] = self._gpr
        ctx["gpr_summary_json"] = self._gpr["summary_json"]
        node.get_logger().info(
            f"[{self.name}] GPR: {self._gpr['n_new']} scan(s) processed in "
            f"{job.elapsed_s:.0f} s, {self._gpr['n_associated']} tied to a scanned "
            f"line, {self._gpr['n_hyperbolae']} hyperbolae, {self._gpr['n_lines']} "
            f"lines, {self._gpr['n_failed']} failed"
        )
        for entry in self._gpr["entries"]:
            node.get_logger().info(f"[{self.name}]   {gpr.describe_entry(entry)}")
        self._advance(ctx, "decision")

    # ------------------------------------------------------------------
    # Phase 4: the POKEYE decision
    # ------------------------------------------------------------------
    def _use_mock(self, ctx):
        """Whether the legacy /sensor_data_processing service decides instead.

        Explicit ``sensor_processing_mock`` wins. Otherwise only a simulation run
        with nothing classified falls back to it, so Gazebo keeps walking the
        full SendDataToPokeye cycle exactly as before.
        """
        explicit = ctx.get("sensor_processing_mock")
        if explicit is not None:
            return bool(explicit)
        return bool(ctx.get("sim", False)) and not (self._hsi and self._hsi["samples"])

    def _run_decision(self, ctx):
        """Per-sample POKEYE decisions, clustered into targets for this wall."""
        node = ctx["node"]
        if self._use_mock(ctx):
            self._advance(ctx, "external")
            return

        samples = (self._hsi or {}).get("samples") or []
        wall_index = ctx.get("current_wall_index")
        if not samples:
            node.get_logger().info(
                f"[{self.name}] no classified spectra; nothing to decide, POKEYE not required.")
            self._finish(ctx, targets=[], decisions=[])
            return

        self.set_activity(ctx, f"Deciding POKEYE targets for wall {wall_index}")
        try:
            threshold = float((self._hsi or {}).get(
                "confidence_threshold", hsi.DEFAULT_CONFIDENCE_THRESHOLD))
            decisions = pokeye.decide_samples(samples, threshold)
            params = pokeye.ClusterParams.from_ctx(ctx)
            targets, stats = pokeye.cluster_targets(decisions, params, wall_index=wall_index)
            decisions_json, targets_json = pokeye.write_outputs(
                paths.pokeye_results_dir(ctx), decisions, targets, stats)
        except VendorUnavailable as exc:
            node.get_logger().warn(
                f"[{self.name}] POKEYE decision package unavailable ({exc}); "
                f"POKEYE not required by default.")
            self._finish(ctx, targets=[], decisions=[])
            return
        except Exception as exc:                # noqa: BLE001
            node.get_logger().error(
                f"[{self.name}] POKEYE decision failed: {exc}; POKEYE not required by default.")
            self._finish(ctx, targets=[], decisions=[])
            return

        dstats = pokeye.decision_stats(decisions)
        node.get_logger().info(
            f"[{self.name}] POKEYE decisions over {dstats['n']} samples: "
            f"{dstats['n_pokeye_required']} require POKEYE, {dstats['n_no_action']} "
            f"no action, {dstats['n_hold']} hold"
            + (f" ({', '.join(f'{k} x{v}' for k, v in dstats['reasons'].items())})"
               if dstats["reasons"] else "")
        )
        node.get_logger().info(
            f"[{self.name}] wall {wall_index}: {stats['n_flagged']} flagged samples "
            f"-> {stats['n_clusters']} clusters ({stats['n_clusters_too_small']} too "
            f"small, {stats['n_unlocated']} without map pose) -> {stats['n_targets']} "
            f"target(s): {pokeye.describe_targets(targets)}"
        )
        ctx["pokeye_decisions_json"] = decisions_json
        ctx["pokeye_targets_json"] = targets_json
        self._finish(ctx, targets=targets, decisions=decisions)

    def _finish(self, ctx, targets, decisions):
        ctx["pokeye_targets"] = targets
        ctx["pokeye_n_decisions"] = len(decisions)
        ctx["drilling_required"] = bool(targets)
        ctx["data_processed"] = True
        self._advance(ctx, "done")

    # ------------------------------------------------------------------
    # Phase 5 (fallback): the external mock service
    # ------------------------------------------------------------------
    def _run_external(self, ctx):
        """Call /sensor_data_processing and take its verdict (simulation only)."""
        node = ctx["node"]
        self.set_activity(ctx, "Processing the scan sensor data (mock service)")

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
        self._advance(ctx, "done")

    def _run_done(self, ctx):
        pass
