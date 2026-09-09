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

The one impure hook is ``predict_fn`` in :func:`process_session` -- the caller
passes a callable wrapping the ``PredictMaterial`` service, so this module never
imports rclpy.

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

DEFAULT_ROOT = "~/hyperspectral_sweeps"


def session_root(ctx=None):
    """Directory holding all sessions. ``hyperspectral_output_dir`` overrides."""
    root = DEFAULT_ROOT
    if ctx is not None:
        root = ctx.get("hyperspectral_output_dir") or DEFAULT_ROOT
    return os.path.expanduser(str(root))


def new_session_dir(ctx=None, stamp=None):
    """Create and return a fresh session directory."""
    stamp = stamp or datetime.now().strftime("%Y%m%d_%H%M%S")
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
              vis=None, nir=None):
        """Record one sample. Returns the sequence number, or None if unwritable."""
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


def process_session(session_dir, predict_fn=None, limits=None,
                    min_valid_fraction=0.5, logger=None):
    """Turn a recorded session into reflectance rows and coverage metrics.

    Pure with respect to the camera: it reads only what the sweep wrote. Safe to
    run repeatedly -- the processing counters are cleared first and the CSV is
    rewritten from scratch, so re-running after a model change gives a clean
    result rather than an appended second copy.

    ``predict_fn(vis, nir) -> (material, confidence)`` wraps the PredictMaterial
    service; pass None to compute reflectance and quality without labelling
    (which is also the offline path, where no ROS graph is running).

    Rejected samples are written to the CSV like any other, with their status
    and reason in dedicated columns. They are evidence of where the sweep
    struggled, and dropping them -- which is what the CLI does -- is what makes
    a bad wall indistinguishable from a wall nobody scanned.
    """
    calibration = load_calibration(session_dir)
    if calibration is None:
        raise FileNotFoundError(
            f"no {CALIBRATION_FILENAME} in {session_dir}: the sweep never "
            f"recorded its GDS/GRF, so reflectance cannot be computed"
        )

    gds_vis = np.asarray(calibration["gds_vis"], dtype=np.float64)
    gds_nir = np.asarray(calibration["gds_nir"], dtype=np.float64)
    grf_vis = np.asarray(calibration["grf_vis"], dtype=np.float64)
    grf_nir = np.asarray(calibration["grf_nir"], dtype=np.float64)

    metrics = SweepMetrics.load(session_dir)
    metrics.clear_processing()

    vis_wls = vis_wavelengths()
    nir_wls = nir_wavelengths()
    csv_path = os.path.join(session_dir, REFLECTANCE_FILENAME)
    tmp_csv = csv_path + ".tmp"

    processed = 0
    with open(tmp_csv, "w", newline="") as handle:
        writer = csv.writer(handle)
        header = [
            "Seq", "Timestamp", "Wall_Index", "Line_Idx", "Seg_Idx",
            "Trigger_Idx", "Travel_m", "Frame", "X", "Y", "Z",
            "Status", "Reason", "Material", "Confidence",
        ]
        header += [f"VIS_{wl:.1f}" for wl in vis_wls]
        header += [f"NIR_{wl:.1f}" for wl in nir_wls]
        writer.writerow(header)

        for row in read_raw_samples(session_dir):
            if row.get("outcome") != OK:
                # A capture that never produced a spectrum. Already counted on
                # the collection side during the sweep; nothing to process.
                continue
            vis_raw = row.get("vis")
            nir_raw = row.get("nir")
            if vis_raw is None or nir_raw is None:
                continue

            wall_index = row.get("wall_index")
            line_idx = row.get("line_idx")
            seg_idx = row.get("seg_idx")
            processed += 1

            vis_norm, vis_ok, vis_reason = reflectance(
                vis_raw, gds_vis, grf_vis, min_valid_fraction)
            nir_norm, nir_ok, nir_reason = reflectance(
                nir_raw, gds_nir, grf_nir, min_valid_fraction)

            material, confidence = "", ""
            if not (vis_ok and nir_ok):
                status = REJECTED_CALIBRATION
                reason = vis_reason or nir_reason
            else:
                stable, reason = check_stability(vis_norm, nir_norm, limits)
                if not stable:
                    status = REJECTED_STABILITY
                elif predict_fn is None:
                    status = ACCEPTED
                    reason = ""
                else:
                    try:
                        material, confidence = predict_fn(vis_norm, nir_norm)
                    except Exception as exc:            # noqa: BLE001
                        material, confidence = "", ""
                        status, reason = REJECTED_ML, f"predict failed: {exc}"
                    else:
                        if is_ml_rejection(material):
                            status = REJECTED_ML
                            reason = f"model returned {material!r}"
                            material = ""
                        else:
                            status, reason = ACCEPTED, ""

            metrics.record_processing(
                wall_index, line_idx, seg_idx, status, reason)

            pose = row.get("pose") or [None, None, None]
            data_row = [
                row.get("seq"), row.get("t"), wall_index, line_idx, seg_idx,
                row.get("trigger_idx"), row.get("travel_m"), row.get("frame"),
                pose[0], pose[1], pose[2],
                status, reason, material, confidence,
            ]
            data_row += [f"{v:.6f}" for v in vis_norm]
            data_row += [f"{v:.6f}" for v in nir_norm]
            writer.writerow(data_row)

    os.replace(tmp_csv, csv_path)
    metrics_path = metrics.save(
        session_dir,
        extra={"processed_at": _utc_now(), "processed_samples": processed},
    )
    if logger is not None:
        logger.info(metrics.summary_line("mission"))
        reasons = metrics.reject_reasons()
        if reasons:
            top = ", ".join(f"{k} x{v}" for k, v in list(reasons.items())[:5])
            logger.info(f"hyperspectral rejection reasons: {top}")

    return {
        "session_dir": session_dir,
        "reflectance_csv": csv_path,
        "metrics_json": metrics_path,
        "processed_samples": processed,
        "metrics": metrics,
    }
