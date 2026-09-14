"""Sweep-time half of the hyperspectral integration: fire one capture every N
centimetres of sensor-plate travel and record the raw result.

Shaped like ColumnController -- a plain object owned by a state, driven through
``configure`` / ``start_line`` / ``stop_line`` / ``abort`` -- so ScanWall gains
a handful of one-line calls instead of another two hundred lines.

WHY THIS IS NOT THE GPR PATTERN
-------------------------------
The GPR is continuous and self-triggered: the FSM starts a line, the probe's own
encoder clocks the traces, the FSM stops the line. The hyperspectral camera is
discrete and point-wise -- one GSM is one spot, 140-210 ms of integration plus
protocol overhead -- and nothing triggers it. The FSM has to decide when.

It samples on distance, not time, and the distance is the SENSOR PLATE's, not
the base's. That matters because an arm-only sweep parks the base entirely, so
anything measured against the chassis would never advance; and because a
distance rule carries over unchanged to ScanFloor/ScanCeiling.

WHAT IT DELIBERATELY DOES NOT DO
--------------------------------
* **No blocking.** Every camera call is ``call_async`` with a done-callback. The
  FSM node spins single-threaded, and the node behind this service sleeps for
  the integration time and retries up to three times -- worst case about a
  minute. A blocking wait anywhere here would stall the sweep, the /cmd_vel
  crawl and the FSM tick together.
* **No processing.** Reflectance, quality gating and material prediction all
  happen later, from the recorded file
  (:mod:`task_planner_fsm.utils.hyperspectral_processing`). Nothing in the FSM
  reacts to a material label mid-sweep, so per-sample ML would buy nothing and
  cost a second service round-trip per point.
* **No aborting the scan.** Unlike the GPR -- the primary sensor, whose failure
  calls ``self.fail()`` -- a hyperspectral hiccup is recorded and skipped. It
  must never kill a wall line.
* **No median of N captures.** The 5-capture median in arm_control's CLI is a
  stationary-sample concept: at 0.05 m/s five captures 0.7 s apart span about
  25 cm, so the median would mix five different spots. One capture per point.
"""

import math
import os
import time

from arm_control.srv import HyperspectralCommand

from . import hyperspectral_processing as hp


class HyperspectralSampler:
    """Distance-triggered raw-spectrum recorder for a wall sweep."""

    # --- defaults, all overridable from ctx / ROS params ---

    # Plate travel between two samples. 10 cm is the starting point from the
    # design doc; it is pending confirmation against a Jetson bench measurement
    # of the real GSM round-trip time, which is the one number the design cannot
    # derive from the code.
    SAMPLE_SPACING_M = 0.10
    # Floor on the interval between two captures, whatever the distance says.
    # At 0.05 m/s a 10 cm spacing is 2 s apart anyway, so this never binds on a
    # base sweep -- it is there for the arm sweep, where the plate can cross
    # 10 cm far faster than the camera can answer.
    MIN_SAMPLE_PERIOD_S = 1.5
    # Plate-pose sampling rate. Only needs to be well above
    # sweep_speed / spacing (0.05 / 0.10 = 0.5 Hz); 10 Hz quantises the trigger
    # position to ~5 mm, far below the spot size, at negligible cost.
    SAMPLE_RATE_HZ = 10.0
    # Same deadband and sanity cap the GPR trigger sampler uses: below the first
    # is TF jitter rather than motion, above the second is a localisation jump
    # rather than plate travel.
    MIN_STEP_M = 0.0005
    MAX_JUMP_M = 0.05
    # Give up on an in-flight capture after this long and free the slot. The
    # camera node retries a GSM three times internally, so a genuine reply can
    # legitimately take tens of seconds; this only catches a call that will
    # never come back at all.
    CAPTURE_TIMEOUT_S = 45.0
    SERVICE_NAME = "hyperspectral/measurement"

    def __init__(self, owner_name="HyperspectralSampler"):
        self.owner_name = owner_name
        self._client = None
        self._session_dir = None
        self._recorder = None
        self.metrics = hp.SweepMetrics()

        # Calibration, fetched once per mission and cached for the whole session.
        self._calibration_ready = False
        self._calibration_future = None
        self._calibration_stage = None      # "GET_GDS" -> "GET_GRF" -> "GET_MTI"
        self._calibration = {}

        # Per-segment sampling state.
        self._timer = None
        self._axis = None                   # unit sweep direction in _ref
        self._ref = "map"
        self._pose_fn = None
        self._last_xyz = None
        self._residual = 0.0
        self._travel = 0.0
        self._trigger_count = 0
        self._tf_warned = False

        # In-flight capture (at most one).
        self._pending = None
        self._pending_started = 0.0
        self._pending_meta = None
        self._last_capture_t = 0.0

        # Identity of the segment being swept, stamped onto every raw sample.
        self._wall_index = None
        self._line_idx = None
        self._seg_idx = None

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------
    @staticmethod
    def enabled(ctx):
        """Whether to sample at all. Off by default, mirroring ``gpr_enabled``."""
        return bool(ctx.get("hyperspectral_enabled", False))

    def spacing(self, ctx):
        spacing = float(ctx.get("hyperspectral_sample_spacing_m", self.SAMPLE_SPACING_M))
        return spacing if spacing > 0.0 else self.SAMPLE_SPACING_M

    def configure(self, node, ctx):
        """Create the service client and open the session directory.

        Idempotent: ScanWall calls it on every entry, and a mission scans many
        walls, but the session -- and therefore the calibration and the raw file
        -- must span all of them.
        """
        if not self.enabled(ctx):
            return False
        if self._client is None:
            self._client = node.create_client(
                HyperspectralCommand,
                str(ctx.get("hyperspectral_service", self.SERVICE_NAME)),
            )
        if self._session_dir is None:
            # ctx carries the session across states so the post-sweep processing
            # state finds the same directory without guessing.
            existing = ctx.get("hyperspectral_session_dir")
            self._session_dir = (
                os.path.expanduser(str(existing)) if existing
                else hp.new_session_dir(ctx)
            )
            ctx["hyperspectral_session_dir"] = self._session_dir
            # Reattach rather than restart: if the session already has metrics
            # (the FSM was restarted mid-mission, or an earlier scan state used
            # the same session), keep those segments. Saving is a whole-file
            # rewrite, so starting empty here would silently drop every wall
            # already swept.
            self.metrics = hp.SweepMetrics.load(self._session_dir)
            node.get_logger().info(
                f"[{self.owner_name}] hyperspectral session: {self._session_dir}"
            )
        if self._recorder is None:
            # Re-opened rather than created: ScanWall exits and re-enters once
            # per scan line, and abort() closes the file each time. The session
            # -- and the raw record -- spans every line of every wall.
            self._recorder = hp.RawRecorder(self._session_dir).open()
        return True

    # ------------------------------------------------------------------
    # Calibration (once per mission)
    # ------------------------------------------------------------------
    def calibration_ready(self, ctx):
        """True once GDS/GRF are cached on disk for this session.

        Non-blocking and re-entrant: call it every tick until it returns True.
        It walks GET_GDS -> GET_GRF -> GET_MTI one async call at a time.

        These are meta commands: the node answers them from memory without
        touching the hardware, because the operator captured the dark current
        and the white reference during the interactive calibration before the
        mission started. So this is cheap, and it is also the moment an
        uncalibrated or unreachable camera is discovered -- before a sweep is
        recorded against a calibration that does not exist.
        """
        if self._calibration_ready:
            return True
        if self._client is None:
            return False
        node = ctx["node"]

        # Loop rather than return after each response: a completed stage
        # dispatches the next one in the same call, so the whole walk costs one
        # FSM tick plus the service round-trips instead of six ticks.
        while True:
            if self._calibration_future is None:
                stage = {
                    None: "GET_GDS",
                    "GET_GDS": "GET_GRF",
                    "GET_GRF": "GET_MTI",
                }.get(self._calibration_stage)
                if stage is None:
                    return self._finish_calibration(ctx)
                if not self._client.service_is_ready():
                    node.get_logger().warn(
                        f"[{self.owner_name}] {self.SERVICE_NAME} not available "
                        f"yet; is hyperspectral_node running and calibrated?",
                        throttle_duration_sec=5.0,
                    )
                    return False
                request = HyperspectralCommand.Request()
                request.command = stage
                self._calibration_stage = stage
                self._calibration_future = self._client.call_async(request)
                return False

            if not self._calibration_future.done():
                return False

            result = self._calibration_future.result()
            self._calibration_future = None
            stage = self._calibration_stage
            if result is None:
                node.get_logger().error(
                    f"[{self.owner_name}] {stage} returned no response; "
                    f"hyperspectral sampling stays off for this sweep."
                )
                # Rewind to the start: a half-fetched calibration must never be
                # saved, and the next tick retries the whole walk.
                self._calibration_stage = None
                return False

            if stage == "GET_MTI":
                # message is "<vis_mti>|<nir_mti>" -- what was COMMANDED, not a
                # hardware read-back. There is no read-back anywhere in the
                # stack, so this is provenance, not evidence.
                try:
                    vis_mti, nir_mti = str(result.message).split("|")
                    self._calibration["mti_vis"] = int(vis_mti)
                    self._calibration["mti_nir"] = int(nir_mti)
                except (ValueError, AttributeError):
                    self._calibration["mti_vis"] = None
                    self._calibration["mti_nir"] = None
                return self._finish_calibration(ctx)

            if not (result.vis_ok and result.nir_ok):
                node.get_logger().error(
                    f"[{self.owner_name}] {stage} failed: {result.message}. The "
                    f"operator must run the interactive calibration on "
                    f"hyperspectral_node before the sweep starts."
                )
                self._calibration_stage = None
                return False
            key = "gds" if stage == "GET_GDS" else "grf"
            self._calibration[f"{key}_vis"] = list(result.vis_spectrum)
            self._calibration[f"{key}_nir"] = list(result.nir_spectrum)
            # Fall through to dispatch the next stage without waiting a tick.

    def _finish_calibration(self, ctx):
        """Persist the fetched calibration and unlock sampling."""
        node = ctx["node"]
        try:
            hp.save_calibration(
                self._session_dir,
                self._calibration["gds_vis"], self._calibration["gds_nir"],
                self._calibration["grf_vis"], self._calibration["grf_nir"],
                mti_vis=self._calibration.get("mti_vis"),
                mti_nir=self._calibration.get("mti_nir"),
            )
        except (KeyError, OSError) as exc:
            node.get_logger().error(
                f"[{self.owner_name}] could not save the calibration: {exc}")
            self._calibration_stage = None
            return False
        self._calibration_ready = True
        node.get_logger().info(
            f"[{self.owner_name}] calibration cached for the session "
            f"(GDS + GRF, MTI vis={self._calibration.get('mti_vis')} "
            f"nir={self._calibration.get('mti_nir')})."
        )
        return True

    # ------------------------------------------------------------------
    # Segment lifecycle
    # ------------------------------------------------------------------
    def start_line(self, ctx, seg_start, seg_end, pose_fn, ref="map", axis=None,
                   wall_index=None, line_idx=None, seg_idx=None):
        """Arm the sampler for the segment sweep that is about to start.

        ``pose_fn(ref_frame, timeout_s)`` returns the plate position in
        ``ref_frame`` as ``(x, y, z)`` or None -- ScanWall passes its own
        ``_lookup_plate_xyz``, so the two sensors track the exact same point on
        the robot and there is one TF fallback chain, not two.

        ``ref``/``axis`` come from the caller for the same reason: it has
        already resolved which frame this sweep route should be measured in (an
        arm sweep parks the base, so ``arm_base`` keeps localisation drift out
        of the spacing; a base sweep only moves relative to the world) and
        expressed the sweep direction in it.

        Counters restart per segment. Unlike the GPR there is no d = 0 sample:
        the plate has just settled against the wall and the first capture would
        overlap the press, so the first sample lands one spacing in.
        """
        self.stop_timer(ctx)                       # never two timers
        if not self.enabled(ctx) or self._client is None:
            return False
        node = ctx["node"]
        if not self._calibration_ready:
            # Sampling without GDS/GRF would record spectra that can never be
            # turned into reflectance. Skip the segment loudly instead: the GPR
            # is the primary sensor and its line must still run.
            node.get_logger().warn(
                f"[{self.owner_name}] no calibration cached; this segment will "
                f"not be sampled. Run the interactive GDS/GRF calibration on "
                f"hyperspectral_node before the mission."
            )
            return False

        self._ref = ref
        self._axis = axis
        self._pose_fn = pose_fn
        self._wall_index = wall_index
        self._line_idx = line_idx
        self._seg_idx = seg_idx
        self._residual = 0.0
        self._travel = 0.0
        self._trigger_count = 0
        self._tf_warned = False
        self._last_capture_t = 0.0
        self._last_xyz = pose_fn(ref, 1.0)

        spacing = self.spacing(ctx)
        self.metrics.begin_segment(wall_index, line_idx, seg_idx, spacing)

        rate = max(1.0, float(
            ctx.get("hyperspectral_sample_rate_hz", self.SAMPLE_RATE_HZ)))
        self._timer = node.create_timer(1.0 / rate, lambda: self._tick(ctx))
        node.get_logger().info(
            f"[{self.owner_name}] hyperspectral sampling armed: one capture "
            f"every {spacing * 100.0:.1f} cm of plate travel in '{ref}', "
            f"polled at {rate:.0f} Hz."
        )
        return True

    def stop_line(self, ctx, aborted=False):
        """Disarm at the end of a segment and close its metrics record."""
        if self._timer is None and self.metrics.current is None:
            return
        node = ctx["node"]
        self.stop_timer(ctx)
        # A capture dispatched just before the sweep ended is still valid data --
        # the plate was on the wall when it fired -- so let it land rather than
        # cancelling it. It is attributed to the segment that is closing here,
        # which is where it was actually taken.
        self.metrics.end_segment(travel_m=self._travel, aborted=aborted)
        self.save_metrics(ctx)
        node.get_logger().info(
            f"[{self.owner_name}] {self.metrics.summary_line('segment')} "
            f"over {self._travel:.3f} m of plate travel."
        )

    def abort(self, ctx):
        """Leave the state: disarm, close any open segment, flush the record."""
        self.stop_line(ctx, aborted=True)
        # Save unconditionally, even when stop_line had nothing open to close. A
        # capture dispatched at the end of the last segment can land after that
        # segment was already saved, and this is the only point left that would
        # persist it -- otherwise the raw file holds a spectrum the metrics never
        # counted, and the processing pass reports more samples than were taken.
        self.save_metrics(ctx)
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None

    def save_metrics(self, ctx):
        """Persist the metrics file, if a session is open."""
        if not self._session_dir:
            return None
        try:
            return self.metrics.save(self._session_dir)
        except OSError as exc:
            ctx["node"].get_logger().warn(
                f"[{self.owner_name}] could not save metrics: {exc}")
            return None

    def stop_timer(self, ctx):
        """Cancel the sampling timer without touching the metrics record."""
        if self._timer is not None:
            self._timer.cancel()
            try:
                ctx["node"].destroy_timer(self._timer)
            except Exception:                       # noqa: BLE001
                pass
            self._timer = None
        self._last_xyz = None
        self._pose_fn = None

    def reset(self):
        """Forget per-segment state (mission-level session/calibration stay)."""
        self._timer = None
        self._axis = None
        self._last_xyz = None
        self._residual = 0.0
        self._travel = 0.0
        self._trigger_count = 0
        self._pending = None
        self._pending_meta = None

    # ------------------------------------------------------------------
    # Distance trigger
    # ------------------------------------------------------------------
    def _tick(self, ctx):
        """Timer callback: accumulate plate travel and fire on every spacing.

        The distance bookkeeping mirrors ScanWall's GPR trigger sampler, and for
        the same reasons:

        * Advance is the SIGNED PROJECTION of the displacement onto the sweep
          axis, not the raw 3-D path length. Path length would let TF jitter
          accumulate as phantom travel -- a random walk never cancels once you
          take its magnitude -- and fire captures with the robot standing still.
          Projected, perpendicular noise drops out and along-axis noise averages
          to zero.
        * The residual CARRIES ACROSS ticks and is subtracted, not zeroed, so
          samples stay on a fixed distance grid however irregular the polling.
        * Backwards motion holds instead of firing, clamped to one spacing, so a
          Nav2 recovery does not leave a dead zone the width of the retreat.

        The one difference: the GPR fires a trigger per whole spacing even if
        several fall in one tick, because a pulse is free. A capture is not, so
        at most one is dispatched per tick and the residual keeps the rest.
        """
        pose_fn = self._pose_fn
        if pose_fn is None:
            return
        node = ctx["node"]
        # timeout 0.0: a momentary TF gap must never block the executor.
        xyz = pose_fn(self._ref, 0.0)
        if xyz is None:
            if not self._tf_warned:
                node.get_logger().warn(
                    f"[{self.owner_name}] {self._ref}->plate transform "
                    f"unavailable; no captures until it returns."
                )
                self._tf_warned = True
            return
        self._tf_warned = False

        last = self._last_xyz
        if last is None:
            self._last_xyz = xyz            # late anchor; next tick measures
            return

        axis = self._axis
        if axis is None:
            step = math.dist(xyz, last)
        else:
            step = ((xyz[0] - last[0]) * axis[0]
                    + (xyz[1] - last[1]) * axis[1]
                    + (xyz[2] - last[2]) * axis[2])

        min_step = float(ctx.get("hyperspectral_min_step_m", self.MIN_STEP_M))
        if abs(step) < min_step:
            # Below the noise floor: keep the anchor so genuinely slow motion
            # still accumulates instead of being discarded tick by tick.
            return
        max_jump = float(ctx.get("hyperspectral_max_jump_m", self.MAX_JUMP_M))
        if abs(step) > max_jump:
            node.get_logger().warn(
                f"[{self.owner_name}] plate pose jumped {step:.3f} m in one "
                f"sample (> {max_jump:.3f} m); re-anchoring without capturing."
            )
            self._last_xyz = xyz
            return

        self._last_xyz = xyz
        self._travel += step
        self._residual += step
        spacing = self.spacing(ctx)
        if self._residual < -spacing:
            self._residual = -spacing
        if self._residual < spacing:
            return

        self._residual -= spacing
        self._trigger_count += 1
        self.metrics.record_trigger()
        self._dispatch_capture(ctx, xyz)

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------
    def _dispatch_capture(self, ctx, xyz):
        """Send one GSM, or record why we did not.

        A point we could not sample is recorded as deliberately as one we could:
        a sweep with a gap because the camera was busy and a sweep with a gap
        because nobody asked look identical in the spectra alone.
        """
        node = ctx["node"]
        now = time.monotonic()

        if self._pending is not None:
            if now - self._pending_started > float(
                    ctx.get("hyperspectral_capture_timeout_s", self.CAPTURE_TIMEOUT_S)):
                node.get_logger().warn(
                    f"[{self.owner_name}] capture #{self._pending_meta} timed "
                    f"out after {now - self._pending_started:.0f} s; freeing the slot."
                )
                self._record(ctx, hp.FAILED_NO_RESPONSE, xyz,
                             detail="capture timed out")
                self._pending = None
                self._pending_meta = None
            else:
                self._record(ctx, hp.SKIPPED_BUSY, xyz,
                             detail="previous capture still in flight")
                return

        min_period = float(ctx.get(
            "hyperspectral_min_sample_period_s", self.MIN_SAMPLE_PERIOD_S))
        if self._last_capture_t and (now - self._last_capture_t) < min_period:
            self._record(ctx, hp.SKIPPED_RATE, xyz,
                         detail=f"{now - self._last_capture_t:.2f} s < "
                                f"{min_period:.2f} s minimum period")
            return

        if not self._client.service_is_ready():
            self._record(ctx, hp.FAILED_NO_RESPONSE, xyz,
                         detail="service not ready")
            return

        request = HyperspectralCommand.Request()
        request.command = "GSM"
        # The pose is stamped in the frame the sweep is measured in, which is
        # the one that actually locates the sample on the wall. The node only
        # echoes these into its own CSV; nothing downstream of it interprets them.
        request.x_coord = float(xyz[0])
        request.y_coord = float(xyz[1])
        request.z_coord = float(xyz[2])

        try:
            future = self._client.call_async(request)
        except Exception as exc:                    # noqa: BLE001
            self._record(ctx, hp.FAILED_EXCEPTION, xyz, detail=str(exc))
            return

        self._pending = future
        self._pending_started = now
        self._pending_meta = self._trigger_count
        self._last_capture_t = now
        trigger_idx = self._trigger_count
        travel = self._travel
        sample_xyz = tuple(xyz)
        # Looked up NOW, not in the done-callback: the plate keeps moving while
        # the camera integrates, and the map pose must be where the sample was
        # taken, not where the reply arrived.
        sample_map = self._map_pose(xyz)
        future.add_done_callback(
            lambda fut: self._on_capture(ctx, fut, trigger_idx, travel, sample_xyz, sample_map)
        )

    def _map_pose(self, xyz):
        """The plate in ``map`` at this instant, or None.

        The sweep frame (``self._ref``) is what the spacing is measured in; for
        an arm sweep that is ``arm_base``, which the base carries away between
        partitions of the same wall. Anything that later has to put samples
        from different partitions on one wall -- the POKEYE target clustering
        -- needs a world position captured at the same instant, so it is
        recorded alongside. Zero timeout: a miss costs the map pose of one
        sample, never a stall of the sampling tick.
        """
        if self._ref == "map":
            return tuple(xyz)
        if self._pose_fn is None:
            return None
        try:
            pose = self._pose_fn("map", 0.0)
        except Exception:                           # noqa: BLE001
            return None
        return None if pose is None else tuple(pose)

    def _on_capture(self, ctx, future, trigger_idx, travel, xyz, xyz_map=None):
        """Done-callback: classify the response and append it to the record."""
        if future is self._pending:
            self._pending = None
            self._pending_meta = None
        try:
            result = future.result()
        except Exception as exc:                    # noqa: BLE001
            self._record(ctx, hp.FAILED_EXCEPTION, xyz, detail=str(exc),
                         trigger_idx=trigger_idx, travel_m=travel, xyz_map=xyz_map)
            return

        outcome, detail, vis, nir = hp.classify_capture(result)
        self._record(ctx, outcome, xyz, detail=detail, vis=vis, nir=nir,
                     trigger_idx=trigger_idx, travel_m=travel, xyz_map=xyz_map)

    def _record(self, ctx, outcome, xyz, detail="", vis=None, nir=None,
                trigger_idx=None, travel_m=None, xyz_map=None):
        """Count the outcome and append it to the raw file."""
        self.metrics.record_collection(
            outcome, self._wall_index, self._line_idx, self._seg_idx)
        if self._recorder is None:
            return
        if xyz_map is None and xyz is not None:
            xyz_map = self._map_pose(xyz)
        self._recorder.write(
            outcome,
            wall_index=self._wall_index,
            line_idx=self._line_idx,
            seg_idx=self._seg_idx,
            trigger_idx=self._trigger_count if trigger_idx is None else trigger_idx,
            travel_m=self._travel if travel_m is None else travel_m,
            pose=xyz,
            frame=self._ref,
            detail=detail,
            vis=vis,
            nir=nir,
            pose_map=xyz_map,
        )
        if outcome != hp.OK:
            ctx["node"].get_logger().warn(
                f"[{self.owner_name}] sample #{trigger_idx or self._trigger_count} "
                f"{outcome}: {detail}"
            )
