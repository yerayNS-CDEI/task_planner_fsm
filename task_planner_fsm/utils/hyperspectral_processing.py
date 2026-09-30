"""Offline half of the hyperspectral sweep: turn recorded raw spectra into
reflectance, material labels and coverage metrics.

Nothing in here talks to the camera, and nothing in here is a ROS node. The
sweep records raw GSM counts plus the session's GDS/GRF calibration; every
number downstream of that is a pure function of those three, so it can be
recomputed at any time -- after the mission, on a different machine, or again
next year with a retrained model. That is the whole reason collection and
processing are split (see HYPERSPECTRAL_FSM_INTEGRATION.md 5.3): storing only
reflectance would be lossy, and running the ML per sample during the sweep buys
nothing because no FSM decision depends on the label.

The one impure hook is ``predict_fn`` in :func:`process_session` -- a callable
labelling one sample -- so this module never imports rclpy. The FSM no longer
uses it: material classification is a separate pass over the ``reflectance.csv``
written here, run by :mod:`task_planner_fsm.sensors.hsi` with the DISCOVER
classifier, so the ``Material``/``Confidence`` columns stay empty in a mission
record and the verdicts live in ``processed/<session>/hsi/samples.csv``.

Reflectance is the formula inherited from R&D and must not be "improved":

    R = (GSM - GDS) / (GRF - GDS)

Two rules that look like bugs and are not:

* **Negatives are never clipped.** Thermal-noise negatives have to survive into
  the ML pipeline or the ``StandardScaler`` inside it misbehaves.
* **R > 1.0 is expected and correct** for dry materials -- the white reference
  absorbs IR. The model is trained on values above one.
"""

import csv
import json
import os
from datetime import datetime, timezone

import numpy as np

from ..sensors import paths as sensor_paths

# Every spectrum from either sensor is 256 points. A response of any other
# length means a truncated TCP frame, not a short spectrum.
SPECTRUM_LENGTH = 256

# Wavelength axes, mirroring arm_control's hyperspectral_node/inspection_manager
# so the CSV headers written here line up with the ones written there.
#
# UNRESOLVED (open question 1 in HYPERSPECTRAL_FSM_INTEGRATION.md 7): the vendor
# example sensors_lib/vis_nir_example.py puts VIS at 400-1000 nm instead. The
# model is unaffected either way because its features are positional, but every
# CSV column header depends on which is right. Check the calibration sheet, and
# override with ``hyperspectral_vis_wavelengths`` if it turns out to be the
# vendor range.
VIS_WL_MIN, VIS_WL_MAX = 325.3, 792.6
NIR_WL_MIN, NIR_WL_MAX = 991.0, 1707.0


def vis_wavelengths():
    """VIS wavelength axis, 256 points."""
    return np.linspace(VIS_WL_MIN, VIS_WL_MAX, SPECTRUM_LENGTH)


def nir_wavelengths():
    """NIR wavelength axis, 256 points."""
    return np.linspace(NIR_WL_MIN, NIR_WL_MAX, SPECTRUM_LENGTH)


# ----------------------------------------------------------------------
# Sample outcomes
# ----------------------------------------------------------------------
# One vocabulary shared by the sweep-time recorder and the processing pass, so
# the metrics JSON has a single set of keys end to end. Collection outcomes are
# decided while the robot moves and can never be revisited; processing outcomes
# are recomputed from the raw record every time this module runs.

# --- collection (written during the sweep) ---
OK = "ok"                                   # valid 256-point VIS+NIR pair stored
SKIPPED_BUSY = "skipped_busy"               # previous capture still in flight
SKIPPED_RATE = "skipped_rate_limit"         # min sample period not yet elapsed
FAILED_NO_RESPONSE = "failed_no_response"   # no reply from the service / node down
FAILED_SENSOR = "failed_sensor"             # vis_ok/nir_ok false, or bad status
FAILED_LENGTH = "failed_length"             # spectrum not SPECTRUM_LENGTH long
FAILED_EXCEPTION = "failed_exception"       # anything raised in the call path

COLLECTION_OUTCOMES = (
    OK, SKIPPED_BUSY, SKIPPED_RATE,
    FAILED_NO_RESPONSE, FAILED_SENSOR, FAILED_LENGTH, FAILED_EXCEPTION,
)
# The subset that means "the camera was asked and did not deliver". Kept apart
# from the SKIPPED_* pair, which are the sampler declining to ask at all and are
# therefore a tuning signal (spacing too tight for the round-trip), not a fault.
COLLECTION_FAILURES = (
    FAILED_NO_RESPONSE, FAILED_SENSOR, FAILED_LENGTH, FAILED_EXCEPTION,
)

# --- processing (recomputed from the raw record) ---
ACCEPTED = "accepted"
REJECTED_CALIBRATION = "rejected_calibration"   # GRF-GDS <= 0 over too much of the band
REJECTED_STABILITY = "rejected_stability"       # failed check_stability
REJECTED_ML = "rejected_ml"                     # model errored or rejected the sample

PROCESSING_OUTCOMES = (
    ACCEPTED, REJECTED_CALIBRATION, REJECTED_STABILITY, REJECTED_ML,
)


class PredictionUnavailable(Exception):
    """The ML service itself is not answering.

    Distinct from the model rejecting a sample: a rejection is a verdict about
    the spectrum and belongs in the record, while this says nothing was asked
    successfully at all. The processor counts these separately and stops calling
    after a few in a row, rather than spending the per-call timeout on every
    remaining sample of the mission.
    """


def classify_capture(result, spectrum_length=SPECTRUM_LENGTH):
    """Classify a HyperspectralCommand response into a collection outcome.

    Returns ``(outcome, detail, vis, nir)``; the spectra are None unless the
    outcome is OK. Duck-typed on the response object, so this module still
    imports no ROS.

    Shared by the sweep sampler and the bench-timing tool on purpose: the
    latter exists to size the sampler's parameters, and it can only do that if
    both agree on what counts as a successful capture.
    """
    if result is None:
        return FAILED_NO_RESPONSE, "service returned no result", None, None
    if not (getattr(result, "vis_ok", False) and getattr(result, "nir_ok", False)):
        detail = (
            f"vis_ok={getattr(result, 'vis_ok', None)} "
            f"status={getattr(result, 'vis_status', None)}, "
            f"nir_ok={getattr(result, 'nir_ok', None)} "
            f"status={getattr(result, 'nir_status', None)}: "
            f"{getattr(result, 'message', '')}"
        )
        return FAILED_SENSOR, detail, None, None
    vis = list(result.vis_spectrum)
    nir = list(result.nir_spectrum)
    if len(vis) != spectrum_length or len(nir) != spectrum_length:
        # A short spectrum is a truncated TCP frame, not a short reading.
        return FAILED_LENGTH, f"vis={len(vis)}, nir={len(nir)}", None, None
    return OK, "", vis, nir


# ----------------------------------------------------------------------
# Reflectance
# ----------------------------------------------------------------------
def reflectance(gsm, gds, grf, min_valid_fraction=0.5):
    """Normalise one raw spectrum to reflectance.

    Returns ``(values, ok, reason)``. ``values`` is always a 256-point float64
    array so callers can log it either way; ``ok`` is False when the calibration
    cannot support a normalisation at all.

    ``where=grf_net > 0`` leaves the bins whose white reference is at or below
    the dark current at exactly zero rather than dividing by ~0 and producing
    huge values. A handful of such bins is normal at the UV end -- that is why
    :func:`check_stability` trims the edges. But if most of the band is dead the
    calibration itself is wrong (the classic cause is the operator following the
    English requirements doc, which says to put the LID ON for the white
    reference; it must be OFF, white paper at Z = 30 mm, or GRF collapses onto
    GDS and every reflectance comes out zero). Catch that here, where the reason
    can be reported, instead of letting it surface as "underexposed VIS" later.
    """
    gsm = np.asarray(gsm, dtype=np.float64)
    gds = np.asarray(gds, dtype=np.float64)
    grf = np.asarray(grf, dtype=np.float64)

    if gsm.size != SPECTRUM_LENGTH or gds.size != SPECTRUM_LENGTH or grf.size != SPECTRUM_LENGTH:
        return np.zeros(SPECTRUM_LENGTH), False, (
            f"length mismatch (gsm={gsm.size}, gds={gds.size}, grf={grf.size})"
        )

    # The "tare": subtract the dark current from both the sample and the white
    # reference before dividing.
    gsm_net = gsm - gds
    grf_net = grf - gds

    valid = grf_net > 0
    valid_fraction = float(np.count_nonzero(valid)) / SPECTRUM_LENGTH
    values = np.divide(
        gsm_net, grf_net, out=np.zeros(SPECTRUM_LENGTH), where=valid
    )
    # No clipping. Negatives and values above 1.0 are both meaningful.
    if valid_fraction < min_valid_fraction:
        return values, False, (
            f"white reference above dark current in only "
            f"{valid_fraction * 100.0:.0f}% of bins "
            f"(need {min_valid_fraction * 100.0:.0f}%) -- suspect the GRF capture"
        )
    return values, True, ""


# ----------------------------------------------------------------------
# Stability / quality gate
# ----------------------------------------------------------------------
# Thresholds lifted verbatim from arm_control's inspection_manager._check_stability
# so a sample accepted by the CLI is accepted here too.
STABILITY_LIMITS = {
    "vis_noise": 0.2,
    "nir_noise": 0.15,
    "vis_max_jump": 0.6,
    "nir_max_jump": 0.5,
    "vis_mean_min": 0.01,
    "vis_max": 2.0,
    "nir_max": 2.5,
    "vis_min": -0.2,
    "nir_zero_ratio": 0.1,
}


def check_stability(vis, nir, limits=None):
    """Quality gate for one normalised sample.

    Returns ``(ok, reason)``; ``reason`` is "" when the sample passes and a
    short human-readable string naming the failed criterion otherwise, which is
    what the metrics histogram groups on.

    The trims (``VIS[40:-20]``, ``NIR[20:-20]``) are deliberate: they drop the
    UV end, where the denominator of the reflectance is near zero and the ratio
    is meaningless. Do not widen them to "use all the data".

    Unlike the CLI this never drops the sample -- the caller records the reason
    and keeps the raw spectrum, which is the point of collecting rejections as a
    metric rather than discarding them.
    """
    lim = dict(STABILITY_LIMITS)
    if limits:
        lim.update(limits)

    vis = np.asarray(vis, dtype=np.float64)
    nir = np.asarray(nir, dtype=np.float64)
    if vis.size != SPECTRUM_LENGTH or nir.size != SPECTRUM_LENGTH:
        return False, f"length mismatch (vis={vis.size}, nir={nir.size})"

    vis_check = vis[40:-20]
    nir_check = nir[20:-20]

    vis_noise = float(np.std(np.diff(vis_check)))
    nir_noise = float(np.std(np.diff(nir_check)))
    vis_max_jump = float(np.max(np.abs(np.diff(vis_check))))
    nir_max_jump = float(np.max(np.abs(np.diff(nir_check))))
    vis_mean = float(np.mean(vis_check))
    vis_max = float(np.max(vis_check))
    nir_max = float(np.max(nir_check))
    vis_min = float(np.min(vis_check))
    # Zeros are counted over the FULL NIR trace, not the trim: a run of exact
    # zeros is a truncated TCP frame, and it usually lands in the edges.
    nir_zero_ratio = float(np.count_nonzero(nir == 0)) / float(nir.size)

    if vis_noise > lim["vis_noise"]:
        return False, f"VIS noise {vis_noise:.3f} > {lim['vis_noise']:.3f}"
    if nir_noise > lim["nir_noise"]:
        return False, f"NIR noise {nir_noise:.3f} > {lim['nir_noise']:.3f}"
    if vis_max_jump > lim["vis_max_jump"]:
        return False, f"VIS spike {vis_max_jump:.3f} > {lim['vis_max_jump']:.3f}"
    if nir_max_jump > lim["nir_max_jump"]:
        return False, f"NIR spike {nir_max_jump:.3f} > {lim['nir_max_jump']:.3f}"
    if vis_mean < lim["vis_mean_min"]:
        return False, f"VIS underexposed (mean {vis_mean:.3f} < {lim['vis_mean_min']:.3f})"
    if vis_max > lim["vis_max"] or nir_max > lim["nir_max"]:
        return False, (
            f"saturation/specular reflection (VIS {vis_max:.2f}, NIR {nir_max:.2f})"
        )
    if vis_min < lim["vis_min"]:
        return False, f"severe negatives (VIS min {vis_min:.2f} < {lim['vis_min']:.2f})"
    if nir_zero_ratio > lim["nir_zero_ratio"]:
        return False, f"NIR frame {nir_zero_ratio * 100.0:.1f}% zeros"
    return True, ""


# ----------------------------------------------------------------------
# Session layout on disk
# ----------------------------------------------------------------------
# <root>/session_<stamp>/
#     calibration.json    GDS + GRF + MTI, fetched once at the start of the mission
#     raw_samples.jsonl   one line per trigger, successes AND failures
#     reflectance.csv     written by process_session()
#     metrics.json        written by process_session()
#
# raw_samples is JSON Lines, not CSV, for two reasons: it is append-only so a
# crash mid-sweep still leaves every completed line readable, and a failed
# sample carries no spectrum, which a fixed 519-column CSV row cannot express
# without inventing filler that later reads as data.

RAW_FILENAME = "raw_samples.jsonl"
CALIBRATION_FILENAME = "calibration.json"
REFLECTANCE_FILENAME = "reflectance.csv"
METRICS_FILENAME = "metrics.json"

def session_root(ctx=None):
    """Directory holding all sessions: ``data/raw/hyperspectral`` in the package
    (see :mod:`task_planner_fsm.sensors.paths`). ``hyperspectral_output_dir``
    overrides."""
    override = ctx.get("hyperspectral_output_dir") if ctx is not None else None
    if override:
        return os.path.expanduser(str(override))
    return str(sensor_paths.raw_hyperspectral_root(ctx))


def new_session_dir(ctx=None, stamp=None):
    """Create and return a fresh session directory.

    The stamp is the mission's ``sensor_session_id`` so the processed results
    (``data/processed/session_<stamp>``) sit next to this raw record.
    """
    stamp = stamp or sensor_paths.session_id(ctx)
    path = os.path.join(session_root(ctx), f"session_{stamp}")
    os.makedirs(path, exist_ok=True)
    return path


def _utc_now():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


# ----------------------------------------------------------------------
# Metrics
# ----------------------------------------------------------------------
class SweepMetrics:
    """Per-segment counters for one mission, rolled up per wall and overall.

    A "segment" here is one sweep of one partition of one scan line -- exactly
    the unit ScanWall arms and disarms the sampler around, and the same unit the
    GPR line scan uses, so the two sensors' coverage numbers line up.

    Counters are kept per segment rather than only in aggregate because that is
    what makes a bad number actionable: "38 stability rejections" is noise,
    "every sample on wall 2 line 1 segment 3 was rejected for saturation" says
    the plate was catching a specular reflection on that stretch of wall.

    The collection counters are filled in during the sweep and are final once
    the segment ends. The processing counters are cleared and refilled every
    time :func:`process_session` runs, so re-processing with a new model or new
    thresholds never double-counts.
    """

    def __init__(self):
        self.segments = []
        self._current = None

    # -- collection side -------------------------------------------------
    def begin_segment(self, wall_index, line_idx, seg_idx, spacing_m):
        """Open a segment record. Closes any record left open by an abort."""
        if self._current is not None:
            self.end_segment(travel_m=None, aborted=True)
        self._current = {
            "wall_index": wall_index,
            "line_idx": line_idx,
            "seg_idx": seg_idx,
            "spacing_m": spacing_m,
            "started_at": _utc_now(),
            "finished_at": None,
            "aborted": False,
            "travel_m": None,
            "triggered": 0,
            "collection": {k: 0 for k in COLLECTION_OUTCOMES},
            "processing": {k: 0 for k in PROCESSING_OUTCOMES},
            "reject_reasons": {},
        }
        self.segments.append(self._current)
        return self._current

    def record_trigger(self):
        """A distance trigger fired -- a point we intended to sample."""
        if self._current is not None:
            self._current["triggered"] += 1

    def record_collection(self, outcome, wall_index=None, line_idx=None,
                          seg_idx=None):
        """Outcome of one attempted (or deliberately skipped) capture.

        The segment identity is passed explicitly because a capture dispatched
        near the end of a sweep legitimately lands AFTER its segment has been
        closed -- the sampler lets it, since the plate was on the wall when it
        fired. Counting it against ``_current`` would either drop it (no open
        segment) or, worse, credit it to the next segment. Look it up by
        identity instead, and fall back to the open segment only when the caller
        does not know where it belongs.
        """
        seg = None
        if wall_index is not None or line_idx is not None or seg_idx is not None:
            seg = self._find(wall_index, line_idx, seg_idx)
        if seg is None:
            seg = self._current
        if seg is None:
            return
        bucket = seg["collection"]
        bucket[outcome] = bucket.get(outcome, 0) + 1

    def end_segment(self, travel_m=None, aborted=False):
        """Close the open segment record."""
        if self._current is None:
            return
        self._current["finished_at"] = _utc_now()
        self._current["travel_m"] = travel_m
        self._current["aborted"] = bool(aborted)
        self._current = None

    @property
    def current(self):
        return self._current

    # -- processing side -------------------------------------------------
    def clear_processing(self):
        """Drop every processing counter, keeping the collection ones."""
        for seg in self.segments:
            seg["processing"] = {k: 0 for k in PROCESSING_OUTCOMES}
            seg["reject_reasons"] = {}

    def record_processing(self, wall_index, line_idx, seg_idx, outcome, reason=""):
        """Attribute a processing outcome to the segment the sample came from.

        Samples whose segment is not in the record (a raw file processed without
        its metrics, say) land in a synthetic "orphan" segment rather than being
        dropped, so the totals still add up to the number of raw lines.
        """
        seg = self._find(wall_index, line_idx, seg_idx)
        if seg is None:
            seg = self.begin_segment(wall_index, line_idx, seg_idx, spacing_m=None)
            seg["orphan"] = True
            self.end_segment()
        bucket = seg["processing"]
        bucket[outcome] = bucket.get(outcome, 0) + 1
        if reason:
            # Group by the criterion, not the exact numbers in the message, or
            # every rejection becomes its own histogram bucket.
            key = reason.split("(")[0].strip() or reason
            seg["reject_reasons"][key] = seg["reject_reasons"].get(key, 0) + 1

    def _find(self, wall_index, line_idx, seg_idx):
        for seg in self.segments:
            if (seg["wall_index"] == wall_index
                    and seg["line_idx"] == line_idx
                    and seg["seg_idx"] == seg_idx):
                return seg
        return None

    # -- roll-up ---------------------------------------------------------
    @staticmethod
    def _blank_totals():
        totals = {"triggered": 0, "segments": 0}
        totals.update({k: 0 for k in COLLECTION_OUTCOMES})
        totals.update({k: 0 for k in PROCESSING_OUTCOMES})
        return totals

    @classmethod
    def _accumulate(cls, totals, seg):
        totals["segments"] += 1
        totals["triggered"] += seg["triggered"]
        for key, value in seg["collection"].items():
            totals[key] = totals.get(key, 0) + value
        for key, value in seg["processing"].items():
            totals[key] = totals.get(key, 0) + value

    @staticmethod
    def _derive(totals):
        """Add the ratios a human actually reads off the summary."""
        triggered = totals["triggered"] or 0
        captured = totals.get(OK, 0)
        failed = sum(totals.get(k, 0) for k in COLLECTION_FAILURES)
        skipped = totals.get(SKIPPED_BUSY, 0) + totals.get(SKIPPED_RATE, 0)
        accepted = totals.get(ACCEPTED, 0)
        rejected = sum(
            totals.get(k, 0) for k in PROCESSING_OUTCOMES if k != ACCEPTED
        )
        totals["captured"] = captured
        totals["capture_failed"] = failed
        totals["skipped"] = skipped
        totals["rejected"] = rejected
        # Of the points we meant to sample, how many produced a spectrum...
        totals["capture_rate"] = (captured / triggered) if triggered else 0.0
        # ...and of those spectra, how many survived processing. Kept separate:
        # a low capture rate is a hardware/timing problem, a low acceptance rate
        # is an optics/calibration problem, and averaging them hides both.
        totals["acceptance_rate"] = (accepted / captured) if captured else 0.0
        # End to end: usable material readings per intended sample point.
        totals["yield"] = (accepted / triggered) if triggered else 0.0
        return totals

    def walls(self):
        """Per-wall roll-up, keyed by wall index as a string (JSON-friendly)."""
        by_wall = {}
        for seg in self.segments:
            key = str(seg["wall_index"])
            totals = by_wall.setdefault(key, self._blank_totals())
            self._accumulate(totals, seg)
        return {k: self._derive(v) for k, v in by_wall.items()}

    def totals(self):
        """Mission-wide roll-up."""
        totals = self._blank_totals()
        for seg in self.segments:
            self._accumulate(totals, seg)
        return self._derive(totals)

    def reject_reasons(self):
        """Mission-wide histogram of rejection criteria."""
        merged = {}
        for seg in self.segments:
            for key, value in seg["reject_reasons"].items():
                merged[key] = merged.get(key, 0) + value
        return dict(sorted(merged.items(), key=lambda kv: -kv[1]))

    def to_dict(self, extra=None):
        orphans = sum(1 for seg in self.segments if seg.get("orphan"))
        doc = {
            "generated_at": _utc_now(),
            # Non-zero means raw samples were processed whose segment was not in
            # the sweep's own record -- the two halves disagree about what was
            # collected, and the ratios below are not trustworthy until that is
            # explained (usually a metrics.json from a different run).
            "orphan_segments": orphans,
            "totals": self.totals(),
            "walls": self.walls(),
            "reject_reasons": self.reject_reasons(),
            "segments": self.segments,
        }
        if extra:
            doc.update(extra)
        return doc

    def save(self, session_dir, extra=None):
        path = os.path.join(session_dir, METRICS_FILENAME)
        tmp = path + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(self.to_dict(extra), handle, indent=2)
        os.replace(tmp, path)   # atomic: never leave a half-written metrics file
        return path

    @classmethod
    def load(cls, session_dir):
        """Reload a metrics file so processing can add to the sweep's counters."""
        path = os.path.join(session_dir, METRICS_FILENAME)
        metrics = cls()
        if not os.path.isfile(path):
            return metrics
        try:
            with open(path) as handle:
                doc = json.load(handle)
        except (OSError, ValueError):
            return metrics
        metrics.segments = doc.get("segments", []) or []
        return metrics

    def summary_line(self, scope="sweep"):
        """One-line human summary, for the ROS log."""
        t = self.totals()
        return (
            f"hyperspectral {scope}: {t['triggered']} points triggered, "
            f"{t['captured']} captured ({t['capture_rate'] * 100.0:.0f}%), "
            f"{t['capture_failed']} capture failures, {t['skipped']} skipped, "
            f"{t[ACCEPTED]} accepted / {t['rejected']} rejected "
            f"({t['acceptance_rate'] * 100.0:.0f}% of captures), "
            f"yield {t['yield'] * 100.0:.0f}%"
        )


# ----------------------------------------------------------------------
# Raw record
# ----------------------------------------------------------------------
class RawRecorder:
    """Append-only writer for the sweep's raw samples.

    Opened once per session and flushed after every line: the sweep is the one
    part of this that cannot be repeated, so a line that reached the OS is worth
    more than a fast write. One line is a few KB and samples arrive at most
    every couple of seconds, so the cost is irrelevant next to the sweep itself.

    Failures are written too, with no spectrum. A sweep that recorded 40 points
    and a sweep that recorded 40 points after 12 timeouts are very different
    walls, and only the record can tell them apart afterwards.
    """

    def __init__(self, session_dir):
        self.path = os.path.join(session_dir, RAW_FILENAME)
        self._handle = None
        self._seq = 0

    def open(self):
        if self._handle is None:
            self._handle = open(self.path, "a", buffering=1)
        return self

    def close(self):
        if self._handle is not None:
            try:
                self._handle.flush()
                os.fsync(self._handle.fileno())
            except OSError:
                pass
            self._handle.close()
            self._handle = None

    def write(self, outcome, wall_index, line_idx, seg_idx, trigger_idx,
              travel_m=None, pose=None, frame=None, detail="",
              vis=None, nir=None, pose_map=None):
        """Record one sample. Returns the sequence number, or None if unwritable.

        ``pose`` is in the sweep's own frame (``arm_base`` for an arm sweep,
        which the base leaves between partitions of the same wall), so it
        cannot be turned into a world position after the fact. ``pose_map`` is
        the same point looked up in ``map`` at capture time, for anything that
        has to place samples from different partitions together -- the POKEYE
        target clustering, for one.
        """
        if self._handle is None:
            self.open()
        self._seq += 1
        row = {
            "seq": self._seq,
            "t": _utc_now(),
            "outcome": outcome,
            "detail": detail,
            "wall_index": wall_index,
            "line_idx": line_idx,
            "seg_idx": seg_idx,
            "trigger_idx": trigger_idx,
            "travel_m": None if travel_m is None else round(float(travel_m), 5),
            "frame": frame,
            "pose": None if pose is None else [round(float(v), 5) for v in pose],
            "pose_map": None if pose_map is None else [round(float(v), 5) for v in pose_map],
        }
        if vis is not None and nir is not None:
            # ints: these are raw uint16 ADC counts, and writing them as floats
            # would triple the file for no added information.
            row["vis"] = [int(v) for v in vis]
            row["nir"] = [int(v) for v in nir]
        try:
            self._handle.write(json.dumps(row) + "\n")
        except (OSError, TypeError, ValueError):
            return None
        return self._seq


def read_raw_samples(session_dir):
    """Yield the recorded samples in order, skipping unparseable lines.

    A truncated final line is normal if the process died mid-sweep; it is worth
    losing that one sample rather than the whole file.
    """
    path = os.path.join(session_dir, RAW_FILENAME)
    if not os.path.isfile(path):
        return
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except ValueError:
                continue


def save_calibration(session_dir, gds_vis, gds_nir, grf_vis, grf_nir,
                     mti_vis=None, mti_nir=None, extra=None):
    """Persist the session's dark-current and white-reference spectra.

    Written once, when the sweep starts, because these are constant for the
    session and every reflectance in it is computed against them. Storing them
    beside the raw counts is what makes the record self-contained: the session
    directory alone is enough to recompute everything, with no access to the
    camera or to the node's memory.

    ``mti_*`` is what was COMMANDED, not a hardware read-back -- there is no
    read-back anywhere in the stack, so do not treat it as evidence of the
    integration time the sensor actually used.
    """
    doc = {
        "saved_at": _utc_now(),
        "gds_vis": [int(v) for v in gds_vis],
        "gds_nir": [int(v) for v in gds_nir],
        "grf_vis": [int(v) for v in grf_vis],
        "grf_nir": [int(v) for v in grf_nir],
        "mti_vis_commanded": mti_vis,
        "mti_nir_commanded": mti_nir,
    }
    if extra:
        doc.update(extra)
    path = os.path.join(session_dir, CALIBRATION_FILENAME)
    tmp = path + ".tmp"
    with open(tmp, "w") as handle:
        json.dump(doc, handle)
    os.replace(tmp, path)
    return path


def load_calibration(session_dir):
    """Return the saved calibration dict, or None."""
    path = os.path.join(session_dir, CALIBRATION_FILENAME)
    if not os.path.isfile(path):
        return None
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


# ----------------------------------------------------------------------
# Processing pass
# ----------------------------------------------------------------------
# Strings the ML node returns that are NOT material names. inspection_manager
# only filters on the "ERROR" prefix, so a low-confidence rejection is written
# into its CSV as if it were a material (finding R2 in the handoff doc); both
# are treated as rejections here.
ML_REJECT_PREFIXES = ("ERROR", "REBUTJAT")


def is_ml_rejection(material):
    """True when the ML response is a status string rather than a material."""
    if not material:
        return True
    return str(material).upper().startswith(ML_REJECT_PREFIXES)


class SessionProcessor:
    """Resumable processing pass over one recorded session.

    Exists in this shape because the FSM state that drives it must stay
    responsive: it is called from ``run()`` a batch at a time, so every tick
    stays bounded, the RViz panel gets a real progress count, and a mission with
    thousands of samples cannot freeze the state machine. :func:`process_session`
    wraps it for offline use, where running to completion in one call is fine.

    Safe to run repeatedly on the same session: the processing counters are
    cleared up front and the CSV is written to a temporary file that only
    replaces the real one on :meth:`finish`, so an interrupted pass leaves the
    previous report intact rather than a half-written one.
    """

    # Consecutive PredictionUnavailable errors after which labelling is dropped
    # for the rest of the pass. Without this, an ML node that is up but wedged
    # costs the per-call timeout on every remaining sample -- minutes to hours
    # on a real mission -- to produce nothing but rejections.
    ML_FAILURE_LIMIT = 5

    def __init__(self, session_dir, predict_fn=None, limits=None,
                 min_valid_fraction=0.5, logger=None,
                 ml_failure_limit=ML_FAILURE_LIMIT):
        self.session_dir = session_dir
        self.predict_fn = predict_fn
        self.limits = limits
        self.min_valid_fraction = min_valid_fraction
        self.logger = logger
        self.ml_failure_limit = ml_failure_limit

        calibration = load_calibration(session_dir)
        if calibration is None:
            raise FileNotFoundError(
                f"no {CALIBRATION_FILENAME} in {session_dir}: the sweep never "
                f"recorded its GDS/GRF, so reflectance cannot be computed"
            )
        self._gds_vis = np.asarray(calibration["gds_vis"], dtype=np.float64)
        self._gds_nir = np.asarray(calibration["gds_nir"], dtype=np.float64)
        self._grf_vis = np.asarray(calibration["grf_vis"], dtype=np.float64)
        self._grf_nir = np.asarray(calibration["grf_nir"], dtype=np.float64)

        self.metrics = SweepMetrics.load(session_dir)
        self.metrics.clear_processing()

        self.processed = 0
        self.done = False
        self._ml_consecutive_failures = 0
        self._ml_disabled = False

        # Counted up front so the caller can show progress. One cheap pass over
        # the file; the spectra are not parsed.
        self.total = self._count_capturable(session_dir)

        self._csv_path = os.path.join(session_dir, REFLECTANCE_FILENAME)
        self._tmp_path = self._csv_path + ".tmp"
        self._handle = open(self._tmp_path, "w", newline="")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(self._header())
        self._samples = read_raw_samples(session_dir)

    @staticmethod
    def _count_capturable(session_dir):
        """Number of raw lines that carry a spectrum to process."""
        path = os.path.join(session_dir, RAW_FILENAME)
        if not os.path.isfile(path):
            return 0
        count = 0
        with open(path) as handle:
            for line in handle:
                # Substring test rather than a full JSON parse: this runs over
                # every line of a mission-long file just to size a progress bar.
                if '"outcome": "ok"' in line:
                    count += 1
        return count

    @staticmethod
    def _header():
        header = [
            "Seq", "Timestamp", "Wall_Index", "Line_Idx", "Seg_Idx",
            "Trigger_Idx", "Travel_m", "Frame", "X", "Y", "Z",
            "Map_X", "Map_Y", "Map_Z",
            "Status", "Reason", "Material", "Confidence",
        ]
        header += [f"VIS_{wl:.1f}" for wl in vis_wavelengths()]
        header += [f"NIR_{wl:.1f}" for wl in nir_wavelengths()]
        return header

    def step(self, budget=25):
        """Process up to ``budget`` samples. Returns how many were handled.

        Sets ``self.done`` when the record is exhausted; the caller then calls
        :meth:`finish`.
        """
        if self.done:
            return 0
        handled = 0
        while handled < budget:
            row = next(self._samples, None)
            if row is None:
                self.done = True
                break
            if row.get("outcome") != OK:
                # Never produced a spectrum. Already counted on the collection
                # side during the sweep; nothing to process here.
                continue
            if row.get("vis") is None or row.get("nir") is None:
                continue
            self._process_one(row)
            handled += 1
            self.processed += 1
        return handled

    def _process_one(self, row):
        wall_index = row.get("wall_index")
        line_idx = row.get("line_idx")
        seg_idx = row.get("seg_idx")

        vis_norm, vis_ok, vis_reason = reflectance(
            row["vis"], self._gds_vis, self._grf_vis, self.min_valid_fraction)
        nir_norm, nir_ok, nir_reason = reflectance(
            row["nir"], self._gds_nir, self._grf_nir, self.min_valid_fraction)

        material, confidence = "", ""
        if not (vis_ok and nir_ok):
            status = REJECTED_CALIBRATION
            reason = vis_reason or nir_reason
        else:
            stable, reason = check_stability(vis_norm, nir_norm, self.limits)
            if not stable:
                status = REJECTED_STABILITY
            elif self.predict_fn is None or self._ml_disabled:
                status, reason = ACCEPTED, ""
            else:
                status, reason, material, confidence = self._predict(
                    vis_norm, nir_norm)

        self.metrics.record_processing(
            wall_index, line_idx, seg_idx, status, reason)

        pose = row.get("pose") or [None, None, None]
        pose_map = row.get("pose_map") or [None, None, None]
        data_row = [
            row.get("seq"), row.get("t"), wall_index, line_idx, seg_idx,
            row.get("trigger_idx"), row.get("travel_m"), row.get("frame"),
            pose[0], pose[1], pose[2],
            pose_map[0], pose_map[1], pose_map[2],
            status, reason, material, confidence,
        ]
        data_row += [f"{v:.6f}" for v in vis_norm]
        data_row += [f"{v:.6f}" for v in nir_norm]
        self._writer.writerow(data_row)

    def _predict(self, vis_norm, nir_norm):
        """Label one sample, tripping the breaker if the service stops answering."""
        try:
            material, confidence = self.predict_fn(vis_norm, nir_norm)
        except PredictionUnavailable as exc:
            self._ml_consecutive_failures += 1
            if self._ml_consecutive_failures >= self.ml_failure_limit:
                self._ml_disabled = True
                if self.logger is not None:
                    self.logger.warn(
                        f"ML service failed {self._ml_consecutive_failures} times "
                        f"in a row; labelling is off for the rest of this pass. "
                        f"Reflectance and coverage metrics are unaffected, and "
                        f"labels can be added later from the same raw file."
                    )
            return REJECTED_ML, f"ML service unavailable: {exc}", "", ""
        except Exception as exc:            # noqa: BLE001
            # A bug in the model or the wrapper: real, but not a reason to stop.
            return REJECTED_ML, f"predict failed: {exc}", "", ""

        self._ml_consecutive_failures = 0
        if is_ml_rejection(material):
            return REJECTED_ML, f"model returned {material!r}", "", ""
        return ACCEPTED, "", material, confidence

    def finish(self):
        """Publish the report: swap in the CSV and save the metrics."""
        self._handle.close()
        os.replace(self._tmp_path, self._csv_path)
        metrics_path = self.metrics.save(
            self.session_dir,
            extra={"processed_at": _utc_now(),
                   "processed_samples": self.processed,
                   "labelling_aborted": self._ml_disabled},
        )
        if self.logger is not None:
            self.logger.info(self.metrics.summary_line("mission"))
            reasons = self.metrics.reject_reasons()
            if reasons:
                top = ", ".join(f"{k} x{v}" for k, v in list(reasons.items())[:5])
                self.logger.info(f"hyperspectral rejection reasons: {top}")
        return {
            "session_dir": self.session_dir,
            "reflectance_csv": self._csv_path,
            "metrics_json": metrics_path,
            "processed_samples": self.processed,
            "metrics": self.metrics,
            "labelling_aborted": self._ml_disabled,
        }

    def close(self):
        """Abandon the pass without touching the existing report."""
        if self._handle is not None and not self._handle.closed:
            self._handle.close()
        if os.path.isfile(self._tmp_path):
            try:
                os.remove(self._tmp_path)
            except OSError:
                pass


def process_session(session_dir, predict_fn=None, limits=None,
                    min_valid_fraction=0.5, logger=None):
    """Process a whole session in one call.

    The offline entry point, and the one the tests use. The FSM drives
    :class:`SessionProcessor` directly instead, so it can process in batches
    without blocking the state machine.

    Pure with respect to the camera: it reads only what the sweep wrote, so a
    mission can be re-processed days later, on another machine, with a retrained
    model. Rejected samples are written to the CSV like any other, with their
    status and reason -- they are evidence of where the sweep struggled, and
    dropping them is what makes a bad wall indistinguishable from an unscanned one.
    """
    processor = SessionProcessor(
        session_dir, predict_fn=predict_fn, limits=limits,
        min_valid_fraction=min_valid_fraction, logger=logger)
    try:
        while not processor.done:
            processor.step(budget=256)
    except BaseException:
        processor.close()
        raise
    return processor.finish()
