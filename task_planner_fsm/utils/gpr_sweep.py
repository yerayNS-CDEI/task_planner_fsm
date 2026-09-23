"""Sweep-time half of the GPR integration: drive the GP8800 over its HTTP API,
clock it from the sensor plate's travel, and record where each line lies.

Shaped like HyperspectralSampler -- a plain object owned by ScanWall, driven
through ``begin_segment`` / ``arm_triggers`` / ``note_contact`` /
``end_segment`` / ``abort`` -- so the scan strategy only decides WHEN each of
those happens. The API flow, the trigger sampler and the line manifest are the
ones sensor_implementation proved on the base-placement sweep; what differs per
strategy is only the moment the plate is on the wall and about to travel.

THE THREE PIECES
----------------
* **Probe (HTTP).** ``/probe/connect`` with the serial and the probe's static
  IP (so the connection completes without anyone accepting it on the GP App),
  ``/measurement/start`` (LINE_SCAN), ``/measurement/line/start``; at the end
  ``line/stop``, wait for ``GET /measurement/line`` to report it finished,
  ``/measurement/export/raw`` (zip unpacked into the session's ``incoming/``),
  ``/measurement/stop``. A failure to START aborts the scan (a sweep the GPR
  did not record is a wasted wall); a failure to stop or export is logged and
  recorded -- the traces stay on the app.

* **Triggers.** The probe clocks a trace per encoder step, and the encoder is
  an ESP32 fake wheel (gpr_trigger_bridge). One ``/gpr/trigger`` message per
  ``gpr_trigger_distance_m`` of SENSOR-PLATE travel along the segment, sampled
  from TF on a timer of its own (the FSM ticks at 1 Hz). A line opened on the
  probe with no triggers yet records nothing, which is why the line can be
  started before the plate is even near the wall.

* **Line manifest.** One JSON row per line (``sensors/manifest.py``): segment
  endpoints in the world frame, the measurement name, times, trigger count and
  travel, and the probe's own report. The export carries no robot pose, so this
  is the only thing that puts a hyperbola found "0.8 m along the scan" back on
  the wall.

UNDER THE WHOLE-BODY SWEEP
--------------------------
The press is inside wbc_sweep_controller, so ScanWall starts the line before it
launches the node and arms the triggers on the node's first ``running: seated``:
the plate's pose at that moment is d = 0. Travel before it -- the base pre-roll
with the plate 20 cm off the wall -- is never counted.

When the contact comes unseated mid-sweep the triggers keep firing on travel,
so trace ``i`` stays at ``i * spacing`` along the line; the stretch is recorded
instead (``unseated``: plate travel at the unseat and at the re-seat) for the
processing to distrust. Pausing them would shift every later trace.
"""

import math
import time
import zipfile

import rclpy.time
import requests
from rclpy.duration import Duration
from std_msgs.msg import UInt32

from ..sensors import gpr as gpr_export
from ..sensors import manifest as gpr_manifest
from ..sensors import paths as sensor_paths


class GprSweep:
    """GP8800 line scans clocked by sensor-plate travel, for one ScanWall."""

    # --- probe (GP API), all overridable from ctx / ROS params ---
    BASE_URL = "http://192.168.1.239:9000"
    SERIAL = "GP88-007-0081"
    # Static IP of the probe. Sent with /probe/connect so the connection
    # completes without the operator accepting it on the GP App (iPad).
    PROBE_IP = "192.168.1.99"
    TIMEOUT_S = 30.0
    EXPORT_PATH = "/measurement/export/raw"
    # After line/stop: how long to wait for GET /measurement/line to report the
    # line finished, the pause before the export, and before its one retry.
    LINE_FINISH_TIMEOUT_S = 5.0
    EXPORT_DELAY_S = 1.0
    EXPORT_RETRY_DELAY_S = 2.0

    # --- triggers ---
    # Plate travel between two triggers; must equal the bridge's
    # trigger_distance_m (launch/task_planner.launch.py).
    TRIGGER_DISTANCE_M = 0.005
    # Plate-pose sampling rate. Well above sweep_speed / spacing
    # (0.045 / 0.005 = 9 Hz), so the trigger position quantises to ~1 mm.
    TRIGGER_RATE_HZ = 50.0
    # Per-sample deadband: below this is TF jitter, not motion, and stays on
    # the anchor (a 1 mm/s drift would otherwise fire a trigger every 5 s).
    TRIGGER_MIN_STEP_M = 0.0005
    # Per-sample sanity cap: above this is a localisation jump (rtabmap
    # re-publishing map->odom), not plate travel. Re-anchor, do not burst.
    TRIGGER_MAX_JUMP_M = 0.05
    # Log every Nth trigger at info (~200 per metre); the rest go to debug.
    TRIGGER_LOG_EVERY = 20
    # The bridge publishes once a second; older than this, it is gone.
    BRIDGE_MAX_AGE_S = 3.0

    EE_FRAMES = (
        "arm_tool0", "arm_wrist_3_link", "arm_ee_link", "arm_flange",
        "tool0", "wrist_3_link", "ee_link", "flange",
    )

    def __init__(self, name, owner):
        self.name = name
        # The owning State, for set_activity (the UI's "what is it doing") only.
        self._owner = owner
        self.measurement_active = False
        self.line_active = False
        self._measurement_name = None
        self._segment = None          # (wall_index, line_idx, seg_idx)
        self._record = None           # open manifest row
        self._pub = None
        self._timer = None
        self._ref = "map"
        self._axis = None
        self._last_xyz = None
        self._residual = 0.0
        self._travel = 0.0
        self._count = 0
        self._tf_warned = False
        self._ee_frame = None
        self._seated = None           # last contact state note_contact was given

    # ------------------------------------------------------------------
    # Switches
    # ------------------------------------------------------------------
    def enabled(self, ctx):
        """Drive the probe? On for the real robot, off in sim; ``gpr_enabled``
        overrides either way (bench testing, or scanning without the probe)."""
        return bool(ctx.get("gpr_enabled", not bool(ctx.get("sim", False))))

    def triggers_enabled(self, ctx):
        """Emit distance triggers? Deliberately independent of ``enabled``: the
        triggers are worth watching on the topic while validating the spacing
        even with the probe off."""
        return bool(ctx.get("gpr_trigger_enabled", True))

    def bridge_ready(self, ctx):
        """(ok, reason) from the last gpr_trigger_bridge status. Always ok unless
        ``gpr_trigger_bridge_required`` is set (real robot with the ESP32): sim
        and bench runs keep working with nothing plugged in."""
        if not (self.triggers_enabled(ctx) and bool(ctx.get("gpr_trigger_bridge_required", False))):
            return True, ""
        status = ctx.get("gpr_trigger_bridge_status")
        stamp = ctx.get("gpr_trigger_bridge_status_stamp")
        topic = ctx.get("gpr_trigger_bridge_status_topic", "/gpr_trigger_bridge/status")
        if not isinstance(status, dict) or stamp is None:
            return False, f"no GPR trigger bridge status on '{topic}' (is gpr_trigger_bridge running?)"
        age = time.time() - float(stamp)
        if age > self.BRIDGE_MAX_AGE_S:
            return False, f"GPR trigger bridge status is {age:.1f}s old (node stopped?)"
        if not status.get("alive"):
            return False, (
                f"GPR trigger receiver {status.get('receiver', '?')} is not answering "
                f"(last reply {status.get('last_rx_age_s', '?')}s ago)"
            )
        return True, ""

    @property
    def armed(self):
        return self._timer is not None

    # ------------------------------------------------------------------
    # Segment lifecycle
    # ------------------------------------------------------------------
    def begin_segment(self, ctx, wall_index, line_idx, seg_idx, seg_start, seg_end, frame,
                      sweep="wbc"):
        """Open the line record, connect, open the measurement and start the
        line -- all before the plate travels. Returns None, or the reason the
        scan must abort (the caller fails the state; ``abort`` cleans up)."""
        self._segment = (wall_index, int(line_idx), int(seg_idx))
        self._seated = None
        ok, reason = self.bridge_ready(ctx)
        if not ok:
            return reason
        self._open_record(ctx, seg_start, seg_end, frame, sweep)
        if not self.enabled(ctx):
            return None
        if self.measurement_active:
            self._log(ctx).warn(
                f"[{self.name}] GPR: measurement '{self._measurement_name}' is still "
                f"open; finishing it before starting a new one.")
            self._finish_line(ctx)
        err = self._start_measurement(ctx)
        if err:
            return err
        err = self._start_line(ctx)
        if err:
            return err
        self._record["probe_active"] = True
        return None

    def arm_triggers(self, ctx, ref_frame, seg_start, seg_end, speed_mps):
        """Start clocking the probe: the plate's pose NOW is d = 0 and fires the
        first trigger; every later one is a whole spacing further along the
        segment. ``ref_frame`` is what the plate travel is measured in -- the
        world frame for a sweep that moves the base (``gpr_trigger_reference_frame``
        overrides)."""
        if self._record is not None:
            self._record["t_armed_epoch"] = round(time.time(), 3)
        if not self.triggers_enabled(ctx):
            return
        self._stop_timer(ctx, log_summary=False)
        node = ctx["node"]
        ref = str(ctx.get("gpr_trigger_reference_frame") or ref_frame)
        axis = self._axis_in_frame(ctx, ref, ref_frame, self._sweep_axis(seg_start, seg_end))
        if axis is None:
            self._log(ctx).warn(
                f"[{self.name}] GPR triggers: no sweep direction in '{ref}'; "
                f"measuring path length instead of travel along the segment.")
        self._ref, self._axis = ref, axis
        if self._pub is None:
            self._pub = node.create_publisher(
                UInt32, str(ctx.get("gpr_trigger_topic", "/gpr/trigger")), 50)
        self._residual = self._travel = 0.0
        self._count = 0
        self._tf_warned = False
        self._last_xyz = self._lookup_plate_xyz(ctx, ref)
        rate = max(1.0, float(ctx.get("gpr_trigger_rate_hz", self.TRIGGER_RATE_HZ)))
        spacing = self._spacing(ctx)
        if speed_mps > 0.0 and rate < 2.0 * speed_mps / spacing:
            self._log(ctx).warn(
                f"[{self.name}] GPR trigger sampling at {rate:.0f} Hz is coarse for "
                f"{spacing * 100.0:.2f} cm spacing at {speed_mps:.3f} m/s; triggers "
                f"will come in bursts. Raise gpr_trigger_rate_hz.")
        self._timer = node.create_timer(1.0 / rate, lambda: self._tick(ctx))
        self._log(ctx).info(
            f"[{self.name}] GPR triggers armed: one every {spacing * 100.0:.2f} cm of "
            f"plate travel in '{ref}', sampled at {rate:.0f} Hz.")
        if self._last_xyz is not None:
            self._emit(ctx)
        else:
            self._log(ctx).warn(
                f"[{self.name}] GPR triggers: no {ref}->plate transform yet; the d=0 "
                f"trigger fires on the first pose the sampler gets.")

    def note_contact(self, ctx, seated):
        """Record where along the line the contact came unseated and re-seated.
        Only meaningful once armed; before that the plate has not started."""
        if self._record is None or not self.armed or seated == self._seated:
            return
        self._seated = seated
        stretches = self._record.setdefault("unseated", [])
        if not seated:
            stretches.append({"from_m": round(self._travel, 4),
                              "t_from_epoch": round(time.time(), 3)})
            self._log(ctx).warn(
                f"[{self.name}] GPR: contact unseated at {self._travel:.3f} m along "
                f"the line; traces from here are marked doubtful.")
        elif stretches and "to_m" not in stretches[-1]:
            stretches[-1].update({"to_m": round(self._travel, 4),
                                  "t_to_epoch": round(time.time(), 3)})
            self._log(ctx).info(
                f"[{self.name}] GPR: contact re-seated at {self._travel:.3f} m.")

    def end_segment(self, ctx):
        """Plate stopped: disarm, seal the record, stop/export/close the line on
        the probe, write the record. Best-effort, never aborts: the sweep is
        done. Safe to call when nothing was started."""
        self._stop_timer(ctx)
        self._seal_record()
        self._finish_line(ctx)
        self._close_record(ctx)

    def abort(self, ctx):
        """State left mid-segment. Same as ``end_segment``; the name says why."""
        self.end_segment(ctx)

    # ------------------------------------------------------------------
    # Probe: GP API over HTTP (blocking -- called from the FSM tick)
    # ------------------------------------------------------------------
    def _request(self, ctx, method, path, json_body=None):
        """One request; the response, or None on a transport error. Any 2xx is
        success (starts return 200, stops 204); errors carry
        ``{"error":{"code","message"}}``."""
        log = self._log(ctx)
        base_url = str(ctx.get("gpr_base_url", self.BASE_URL)).rstrip("/")
        try:
            resp = requests.request(
                method, f"{base_url}{path}", json=json_body,
                timeout=float(ctx.get("gpr_timeout", self.TIMEOUT_S)))
        except requests.exceptions.RequestException as e:
            log.error(f"[{self.name}] GPR {method} {path} failed: {e}")
            return None
        ctype = resp.headers.get("Content-Type", "")
        if "json" in ctype or ctype.startswith("text/") or not resp.content:
            body = resp.text.strip()[:200]
        else:
            body = f"{len(resp.content)} bytes of {ctype.split(';')[0]}"
        if resp.status_code >= 400:
            log.error(f"[{self.name}] GPR {method} {path} -> HTTP {resp.status_code}: {body}")
        else:
            log.info(f"[{self.name}] GPR {method} {path} -> HTTP {resp.status_code}"
                     + (f": {body}" if body else ""))
        return resp

    @staticmethod
    def _ok(resp, *also_ok):
        return resp is not None and (resp.status_code < 400 or resp.status_code in also_ok)

    @staticmethod
    def _data(resp):
        """The ``data`` object of a GP API response (or the bare object, as the
        app answers for some endpoints), or None."""
        if resp is None or not resp.content:
            return None
        try:
            payload = resp.json()
        except ValueError:
            return None
        if not isinstance(payload, dict):
            return None
        data = payload.get("data")
        return data if isinstance(data, dict) else payload

    def _connect(self, ctx):
        """200 = connected, 406 = already connected; both fine."""
        serial = ctx.get("gpr_serial", self.SERIAL)
        probe_ip = ctx.get("gpr_ip", self.PROBE_IP)
        self._log(ctx).info(f"[{self.name}] GPR: connecting to probe {serial} at {probe_ip}.")
        self._activity(ctx, "Connecting to the GPR probe")
        resp = self._request(ctx, "POST", "/probe/connect",
                             {"serialNumber": serial, "ip": probe_ip})
        return self._ok(resp, 406)

    def _measurement_name_for(self):
        _, line_idx, seg_idx = self._segment
        return f"scan_wall line {line_idx + 1} seg {seg_idx + 1}"

    def _start_measurement(self, ctx):
        if not self._connect(ctx):
            return "GPR probe connection failed"
        name = self._measurement_name_for()
        self._activity(ctx, "Starting the GPR line-scan measurement")
        resp = self._request(ctx, "POST", "/measurement/start",
                             {"type": "LINE_SCAN", "name": name})
        if not self._ok(resp):
            return "GPR failed to start the measurement"
        # Keep the name WE asked for: on 2026-09-17 the app echoed the previous
        # measurement's name while the export it produced carried the right one.
        echoed = (self._data(resp) or {}).get("name")
        if echoed and echoed != name:
            self._log(ctx).warn(
                f"[{self.name}] GPR app named the measurement '{echoed}' "
                f"(asked for '{name}'); recording the requested name.")
        self._measurement_name = name
        self.measurement_active = True
        if self._record is not None:
            self._record["measurement_name"] = name
        return None

    def _start_line(self, ctx):
        self._activity(ctx, "Starting the GPR scan line")
        resp = self._request(ctx, "POST", "/measurement/line/start")
        if not self._ok(resp):
            return "GPR failed to start the scan line"
        self.line_active = True
        self._log(ctx).info(
            f"[{self.name}] GPR line started; traces come with the triggers, once "
            f"the plate is seated on the wall.")
        return None

    def _wait_line_finished(self, ctx):
        """Poll GET /measurement/line until the app reports the line finished.
        What it said (started/finished/scans/length) goes in the manifest: it is
        what tells a short line from an export that raced the stop."""
        timeout = float(ctx.get("gpr_line_finish_timeout_s", self.LINE_FINISH_TIMEOUT_S))
        deadline = time.time() + timeout
        info = None
        while True:
            data = self._data(self._request(ctx, "GET", "/measurement/line"))
            if data is not None:
                info = {k: data.get(k) for k in ("started", "finished", "scans", "length", "index")}
                if info["finished"] or not info["started"]:
                    self._log(ctx).info(
                        f"[{self.name}] GPR line finished: {info['scans']} scans over "
                        f"{info['length']} (app units).")
                    return info
            if time.time() >= deadline:
                self._log(ctx).warn(
                    f"[{self.name}] GPR line not reported finished within {timeout:.0f} s "
                    f"(last: {info}); exporting anyway.")
                return info
            time.sleep(0.5)

    def _export(self, ctx):
        """Pull the line's zip, keep it under the session's ``exports/`` and
        unpack it, key-prefixed, into the session's ``incoming/``. Returns the
        outcome for the manifest."""
        path = str(ctx.get("gpr_export_path", self.EXPORT_PATH))
        self._activity(ctx, "Exporting the GPR line")
        resp = self._request(ctx, "POST", path)
        result = {"ok": False,
                  "status": None if resp is None else int(resp.status_code),
                  "measurement_name": self._measurement_name}
        if not self._ok(resp):
            self._log(ctx).warn(
                f"[{self.name}] GPR export failed (HTTP {result['status']}); the "
                f"traces stay on the app under '{self._measurement_name}'.")
            return result
        key = gpr_manifest.line_key(*self._segment)
        try:
            exports_dir = sensor_paths.gpr_session_dir(ctx) / "exports"
            exports_dir.mkdir(parents=True, exist_ok=True)
            zip_path = exports_dir / f"{key}_{time.strftime('%Y%m%d_%H%M%S')}.zip"
            zip_path.write_bytes(resp.content)
            result["zip"] = str(zip_path)
            files = gpr_export.unpack_export(
                zip_path, sensor_paths.gpr_session_incoming_dir(ctx), key)
        except (OSError, zipfile.BadZipFile) as e:
            result["error"] = str(e)
            self._log(ctx).error(
                f"[{self.name}] GPR export downloaded but could not be stored/unpacked: {e}")
            return result
        result["ok"] = True
        result["files"] = files
        if not any(f.lower().endswith((".sgy", ".segy")) for f in files):
            self._log(ctx).warn(
                f"[{self.name}] GPR export {zip_path.name} holds no .sgy; nothing "
                f"for the processing to pick up.")
        else:
            self._log(ctx).info(
                f"[{self.name}] GPR line exported: {len(resp.content)} bytes -> "
                f"{zip_path.name}, {len(files)} file(s) unpacked into "
                f"{sensor_paths.gpr_session_incoming_dir(ctx)}.")
        return result

    def _finish_line(self, ctx):
        """line/stop -> wait finished -> export (one retry) -> measurement/stop,
        the order the GP API flow chart prescribes. A transport failure on the
        stop means the app is unreachable: the rest is skipped rather than each
        call waiting out its timeout. Flags are cleared regardless."""
        if not (self.line_active or self.measurement_active):
            return
        log = self._log(ctx)
        reachable = True
        if self.line_active:
            self._activity(ctx, "Stopping the GPR scan line")
            resp = self._request(ctx, "POST", "/measurement/line/stop")
            self.line_active = False
            reachable = resp is not None
            if not self._ok(resp):
                log.warn(f"[{self.name}] GPR line stop failed.")
            if reachable:
                line_info = self._wait_line_finished(ctx)
                if self._record is not None:
                    self._record["probe_line"] = line_info
            if reachable and bool(ctx.get("gpr_export_enabled", True)):
                time.sleep(float(ctx.get("gpr_export_delay_s", self.EXPORT_DELAY_S)))
                export = self._export(ctx)
                if not export["ok"]:
                    retry_delay = float(ctx.get("gpr_export_retry_delay_s", self.EXPORT_RETRY_DELAY_S))
                    log.warn(f"[{self.name}] GPR export: retrying once in {retry_delay:.0f} s.")
                    time.sleep(retry_delay)
                    if export["status"] == 403:     # the app's "not connected"
                        self._connect(ctx)
                    first = export
                    export = self._export(ctx)
                    export["first_attempt"] = {
                        k: first.get(k) for k in ("status", "error") if k in first}
                if self._record is not None:
                    self._record["export"] = export
        if self.measurement_active:
            self.measurement_active = False
            if not reachable:
                log.error(
                    f"[{self.name}] GPR app unreachable; leaving measurement "
                    f"'{self._measurement_name}' open on the app.")
            else:
                self._activity(ctx, "Stopping the GPR measurement")
                if self._ok(self._request(ctx, "POST", "/measurement/stop")):
                    log.info(f"[{self.name}] GPR measurement '{self._measurement_name}' stopped.")
                else:
                    log.warn(f"[{self.name}] GPR measurement stop failed.")
        self._measurement_name = None

    # ------------------------------------------------------------------
    # Triggers: one per ``gpr_trigger_distance_m`` of plate travel
    # ------------------------------------------------------------------
    def _spacing(self, ctx):
        spacing = float(ctx.get("gpr_trigger_distance_m", self.TRIGGER_DISTANCE_M))
        return spacing if spacing > 0.0 else self.TRIGGER_DISTANCE_M

    def _stop_timer(self, ctx, log_summary=True):
        if self._timer is not None:
            self._timer.cancel()
            ctx["node"].destroy_timer(self._timer)
            self._timer = None
            if log_summary:
                self._log(ctx).info(
                    f"[{self.name}] GPR triggers: {self._count} fired over "
                    f"{self._travel:.3f} m of plate travel.")
        self._axis = None
        self._last_xyz = None

    def _tick(self, ctx):
        """Accumulate the plate's SIGNED advance along the segment and fire a
        trigger per whole spacing. Projected, sideways TF noise drops out and
        along-axis noise averages to zero (a path length would random-walk into
        phantom travel with the robot still). The residual carries across
        samples so triggers stay on a fixed grid; backing up holds them, with
        at most one spacing of hysteresis."""
        xyz = self._lookup_plate_xyz(ctx, self._ref, timeout_s=0.0)
        if xyz is None:
            if not self._tf_warned:
                self._log(ctx).warn(
                    f"[{self.name}] GPR triggers: {self._ref}->plate transform "
                    f"unavailable; no triggers until it returns.")
                self._tf_warned = True
            return
        self._tf_warned = False
        last = self._last_xyz
        if last is None:
            # Late anchor: TF was not up at arming, so this pose is d = 0.
            self._last_xyz = xyz
            if self._count == 0:
                self._emit(ctx)
            return
        axis = self._axis
        if axis is None:
            step = math.dist(xyz, last)
        else:
            step = sum((xyz[i] - last[i]) * axis[i] for i in range(3))
        if abs(step) < float(ctx.get("gpr_trigger_min_step_m", self.TRIGGER_MIN_STEP_M)):
            return
        max_jump = float(ctx.get("gpr_trigger_max_jump_m", self.TRIGGER_MAX_JUMP_M))
        if abs(step) > max_jump:
            self._log(ctx).warn(
                f"[{self.name}] GPR triggers: plate pose jumped {step:.3f} m in one "
                f"sample (> {max_jump:.3f} m); re-anchoring without firing.")
            self._last_xyz = xyz
            return
        self._last_xyz = xyz
        self._travel += step
        self._residual += step
        spacing = self._spacing(ctx)
        if self._residual < -spacing:
            self._residual = -spacing
        # Integer grid, not subtract-while: at an exact multiple repeated
        # subtraction leaves 4.9999e-3 and drops the last trigger.
        pending = int(math.floor(self._residual / spacing + 1e-9))
        if pending <= 0:
            return
        self._residual -= pending * spacing
        for _ in range(pending):
            self._emit(ctx)

    def _emit(self, ctx):
        """One trigger on the topic. The UDP hop to the ESP32 is the bridge's
        job, so this 50 Hz callback never waits on the Wi-Fi."""
        self._count += 1
        if self._pub is not None:
            self._pub.publish(UInt32(data=self._count))
        _, line_idx, seg_idx = self._segment or (None, 0, 0)
        text = (f"[{self.name}] GPR trigger #{self._count} (plate travel "
                f"{self._travel:.3f} m, line {line_idx + 1}, segment {seg_idx + 1})")
        every = int(ctx.get("gpr_trigger_log_every", self.TRIGGER_LOG_EVERY))
        if every > 0 and self._count % every == 0:
            self._log(ctx).info(text)
        else:
            self._log(ctx).debug(text)

    @staticmethod
    def _sweep_axis(seg_start, seg_end):
        """Unit XY vector from ``seg_start`` to ``seg_end``, or None."""
        if seg_start is None or seg_end is None:
            return None
        dx, dy = seg_end[0] - seg_start[0], seg_end[1] - seg_start[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        return (dx / norm, dy / norm)

    @staticmethod
    def _rotate_vec(q, v):
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        tx = 2.0 * (y * v[2] - z * v[1])
        ty = 2.0 * (z * v[0] - x * v[2])
        tz = 2.0 * (x * v[1] - y * v[0])
        return (v[0] + w * tx + (y * tz - z * ty),
                v[1] + w * ty + (z * tx - x * tz),
                v[2] + w * tz + (x * ty - y * tx))

    def _axis_in_frame(self, ctx, ref, world, axis_xy):
        """The world-frame sweep direction as a 3-D unit vector in ``ref``."""
        if axis_xy is None:
            return None
        if ref == world:
            return (axis_xy[0], axis_xy[1], 0.0)
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        try:
            tf = tf_buffer.lookup_transform(ref, world, rclpy.time.Time(), Duration(seconds=1.0))
        except Exception:
            return None
        v = self._rotate_vec(tf.transform.rotation, (axis_xy[0], axis_xy[1], 0.0))
        norm = math.sqrt(sum(c * c for c in v))
        return None if norm < 1e-6 else tuple(c / norm for c in v)

    def _lookup_plate_xyz(self, ctx, ref_frame, timeout_s=1.0):
        """The plate TCP in ``ref_frame``, or None. The first EE frame that
        resolves is cached so the 50 Hz sampler does not re-probe eight frames
        a tick; it passes ``timeout_s=0`` so a TF gap cannot block the executor."""
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        tool = str(ctx.get("arm_tool_frame", "arm_tool0"))
        chain = [tool] + [f for f in self.EE_FRAMES if f != tool]
        if self._ee_frame is not None:
            chain = [self._ee_frame] + [f for f in chain if f != self._ee_frame]
        timeout = Duration(seconds=float(timeout_s))
        for frame in chain:
            try:
                if tf_buffer.can_transform(ref_frame, frame, rclpy.time.Time(), timeout):
                    t = tf_buffer.lookup_transform(
                        ref_frame, frame, rclpy.time.Time(), timeout).transform.translation
                    self._ee_frame = frame
                    return (float(t.x), float(t.y), float(t.z))
            except Exception:
                continue
        return None

    # ------------------------------------------------------------------
    # Line manifest
    # ------------------------------------------------------------------
    def _open_record(self, ctx, seg_start, seg_end, frame, sweep):
        """Written whether or not the probe is enabled: a file copied over by
        hand still deserves a position on the wall."""
        wall_index, line_idx, seg_idx = self._segment

        def _pt(p):
            return None if p is None else [round(float(v), 4) for v in p]

        self._record = {
            "wall_index": wall_index,
            "line_idx": line_idx,
            "seg_idx": seg_idx,
            "seg_start": _pt(seg_start),
            "seg_end": _pt(seg_end),
            "frame": frame,
            "measurement_name": self._measurement_name_for(),
            "sweep": sweep,
            "arm_sweep": False,
            "probe_active": False,
            "trigger_distance_m": self._spacing(ctx),
            "t_start": gpr_manifest.utc_now(),
            "t_start_epoch": round(time.time(), 3),
        }

    def _seal_record(self):
        """Stamp the end of the line; the first call wins, so it is when the
        plate stopped, not when the export finished."""
        record = self._record
        if record is None or "t_stop" in record:
            return
        stretches = record.get("unseated") or []
        if stretches and "to_m" not in stretches[-1]:
            stretches[-1]["to_m"] = round(self._travel, 4)   # never re-seated
        record.update({
            "t_stop": gpr_manifest.utc_now(),
            "t_stop_epoch": round(time.time(), 3),
            "trigger_count": int(self._count),
            "travel_m": round(float(self._travel), 4),
        })

    def _close_record(self, ctx):
        record, self._record = self._record, None
        if record is None:
            return
        if gpr_manifest.append_gpr_line(ctx, record) is None:
            self._log(ctx).warn(
                f"[{self.name}] could not write the GPR line manifest; this line's "
                f"scan will not be placed on the wall.")

    # ------------------------------------------------------------------
    def _log(self, ctx):
        return ctx["node"].get_logger()

    def _activity(self, ctx, text):
        if self._owner is not None:
            self._owner.set_activity(ctx, text, publish=True)
