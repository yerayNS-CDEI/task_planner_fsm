"""GPR hyperbolae as places POKEYE must not drill.

The sensor team's POKEYE package (FULL PIPELINE, policy v2) decides from the
HSI result whether POKEYE is needed and already refuses a point that sits on a
hyperbola of its own line (see :mod:`.pokeye`). That check is one-dimensional
and only covers the HSI requests. A hyperbola in the B-scan is a reflector --
rebar, a pipe, a cable -- so wherever it sits, nobody drills, whatever sent
POKEYE there, including a RANDOM target the FSM never chose. This module turns
every hyperbola into a map-frame NO_DRILL zone for exactly that.

The project rule is two-sided, and the second side is the stronger one:

* a **detected hyperbola** forbids drilling at its position; and
* **anywhere GPR did not scan is not drillable at all.** A clean B-scan is
  evidence a place is safe; no B-scan is not evidence of anything. So a target
  only survives if a scanned and analysed GPR line covers it, and the same
  applies to a RANDOM location POKEYE picks for itself -- which is why the
  scanned segments travel to POKEYE alongside the forbidden points.

Note the asymmetry: the first rule needs a hyperbola to fire, the second fires
on the absence of data. With no GPR at all, nothing is drillable.

The earlier delivery built the NO_DRILL positions itself
(``build_gpr_drilling_constraints``); the FULL PIPELINE one dropped it, so
:func:`constraints_from_result` now reads them straight off the hyperbola
detections, in the same shape. Three things the sensor package leaves to the
software are ours:

1. **The frame.** ``x_m`` is metres along the B-scan, not a map coordinate. It
   only becomes a place once the scanned line's endpoints are applied -- which
   ScanWall recorded in the GPR manifest and only the FSM knows.
2. **The tolerance.** A hyperbola is a point; a drill bit and a segmentation
   apex both have width. The sensor package states outright that
   ``arc_half_width_m`` is diagnostic geometry and *not* a safety radius. Its
   own decision uses 0.10 m along the line; :data:`DEFAULT_TOLERANCE_M` is the
   FSM's map-frame radius, wider on purpose -- see the note there.
3. **The rejection.** Screening a candidate drill position against the zones
   before it is executed.

A hyperbola from a scan that could not be tied to a scanned line has no map
position and therefore cannot block anything. Those are counted and reported
rather than quietly dropped; ``block_on_unlocated`` turns them into a veto on
the whole wall for a mission that would rather stop than guess.

Pure geometry and policy: no ROS, no import of :mod:`.gpr` (which calls in
here), and the map transform arrives as a callable so this module never has to
know how a scan sits on a wall.
"""

from collections import Counter

import numpy as np

# Provisional. The sensor package's own decision blocks a point within 0.10 m
# of a hyperbola along its line (``gpr_exclusion_radius_m``), but the project
# has not agreed a map-frame radius, so the FSM applies its own until it does:
# generous enough to cover the apex-localisation error of the segmentation plus
# a drill bit, small enough not to veto a wall. It is a placeholder for a
# number that has to come from the safety discussion, NOT a validated figure --
# override it with ``pokeye_no_drill_tolerance_m`` and record the agreed value
# in the README when there is one.
DEFAULT_TOLERANCE_M = 0.15

# How far off a scanned line a target may sit and still count as covered by it.
# Also provisional, and for a concrete reason: the GPR antenna's offset from the
# plate TCP is not known yet and so is not applied when a scan is placed on the
# wall (see ``gpr.scan_to_map``), and a cluster centroid drifts from the samples
# it was built from. 0.25 m absorbs both and reaches exactly halfway to the next
# line of a 0.5 m sweep, so consecutive bands meet without overlapping. Tighten
# it to leave an honest unscanned gap between lines -- refusing more is the safe
# direction for a rule whose whole point is not to claim knowledge we lack.
DEFAULT_LINE_TOLERANCE_M = 0.25

MAP_FRAME = "map"
LOCAL_FRAME = "gpr_scan_local"

# Why a target was refused. Only the first is about a hyperbola; the other two
# are about not knowing.
NO_DRILL_HYPERBOLA = "GPR_HYPERBOLA_DETECTED"
NO_DRILL_NOT_SCANNED = "NOT_ON_A_SCANNED_GPR_LINE"
NO_DRILL_NOT_ANALYSED = "GPR_SCAN_NOT_ANALYSED"
NO_DRILL_UNLOCATED = "GPR_NO_DRILL_POSITION_NOT_LOCATED"


# ----------------------------------------------------------------------
# The vendor constraint, placed on the wall
# ----------------------------------------------------------------------
def _as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _positions_from_detections(gpr_result):
    """One NO_DRILL position per hyperbola, in scan-local metres.

    Field for field what ``build_gpr_drilling_constraints`` returned before
    the FULL PIPELINE delivery dropped it. A detection with no horizontal
    location cannot become a constraint and is skipped, not guessed.
    """
    positions = []
    for i, det in enumerate(gpr_result.get("detections") or []):
        if not isinstance(det, dict):
            continue
        if str(det.get("type", "hyperbola")).strip().lower() not in ("hyperbola", ""):
            continue
        pos = det.get("position") if isinstance(det.get("position"), dict) else {}
        depth = det.get("depth") if isinstance(det.get("depth"), dict) else {}
        robustness = det.get("robustness") if isinstance(det.get("robustness"), dict) else {}
        geometry = det.get("geometry") if isinstance(det.get("geometry"), dict) else {}
        x_m = _as_float(pos.get("x_m"))
        x_cm = _as_float(pos.get("x_cm"))
        if x_m is None and x_cm is not None:
            x_m = x_cm / 100.0
        x_relative = _as_float(pos.get("x_relative"))
        if x_m is None and x_relative is None:
            continue
        positions.append({
            "source_detection_id": det.get("id", f"H{i + 1:03d}"),
            "reason": NO_DRILL_HYPERBOLA,
            "instruction": "NO_DRILL",
            "x_m": x_m,
            "x_cm": None if x_m is None else x_m * 100.0,
            "x_relative": x_relative,
            "depth_cm_approx": _as_float(depth.get("depth_cm")),
            "confidence": _as_float(robustness.get("confidence_mean", det.get("confidence"))),
            "diagnostic_arc_half_width_m": _as_float(geometry.get("arc_half_width_m")),
        })
    return positions


def constraints_from_result(gpr_result, to_map=None, source_key=None):
    """NO_DRILL positions for one scan, each carried into the map frame.

    ``gpr_result`` is the raw dict from ``gpr_integration.run_gpr_pipeline``
    (the same content as its ``gpr_result.json``). ``to_map`` maps a
    scan-local ``x_m`` onto the wall and returns ``[x, y, z]``; without it --
    an export that matched no scanned line -- the positions stay local and are
    marked ``located: False``.

    A result that is not a readable hyperbola result is ``constraint_valid:
    False``, never an empty constraint: the scan was not analysed, and that
    must not read as "analysed and clean".
    """
    if not isinstance(gpr_result, dict):
        message = "No GPR hyperbola result." if gpr_result is None else "GPR result is not an object."
        return {"source_key": source_key, "constraint_valid": False, "hyperbola_detected": False,
                "n_no_drill_positions": 0, "n_located": 0, "no_drill_positions": [],
                "message": message}
    if not isinstance(gpr_result.get("detections", []), list) or (
            gpr_result.get("status") not in (None, "ok")):
        return {"source_key": source_key, "constraint_valid": False, "hyperbola_detected": False,
                "n_no_drill_positions": 0, "n_located": 0, "no_drill_positions": [],
                "message": (f"GPR hyperbola result is not usable (status "
                            f"{gpr_result.get('status')!r}).")}

    positions = []
    for entry in _positions_from_detections(gpr_result):
        placed = dict(entry)
        position_map = None
        x_m = entry.get("x_m")
        if to_map is not None and x_m is not None:
            try:
                position_map = [round(float(v), 4) for v in to_map(float(x_m))]
            except (TypeError, ValueError):
                position_map = None
        placed["position_map"] = position_map
        placed["frame_id"] = MAP_FRAME if position_map else LOCAL_FRAME
        placed["located"] = position_map is not None
        positions.append(placed)

    return {
        "source_key": source_key,
        "constraint_valid": True,
        "hyperbola_detected": bool(gpr_result.get("hyperbola_detected", bool(positions))),
        "n_no_drill_positions": len(positions),
        "n_located": sum(1 for p in positions if p["located"]),
        "no_drill_positions": positions,
        "message": None,
    }


# ----------------------------------------------------------------------
# Zones
# ----------------------------------------------------------------------
def tolerance_from_ctx(ctx):
    """``pokeye_no_drill_tolerance_m``, or the provisional default."""
    if ctx is None:
        return DEFAULT_TOLERANCE_M
    value = ctx.get("pokeye_no_drill_tolerance_m")
    if value in (None, ""):
        return DEFAULT_TOLERANCE_M
    return float(value)


def zones_from_gpr(gpr_results, tolerance_m=DEFAULT_TOLERANCE_M, wall_index=None):
    """Every placeable NO_DRILL position as a map-frame sphere. ``(zones, stats)``.

    ``gpr_results`` is what :func:`sensors.gpr.process_incoming` returned (or
    the accumulated ``gpr_summary.json``); each entry carries the constraints
    built for its scan. ``wall_index`` keeps only the scans of one wall, matching
    how the POKEYE targets are clustered.

    The zone is a sphere, not a vertical band: a hyperbola found along a line
    swept at one height says an object is at that point, and a target a metre
    higher on the same wall is a different place. Depth is behind the wall,
    along the drilling axis, so it never enters the distance -- any depth at a
    blocked ``(x, y, z)`` is reason enough not to drill there.
    """
    zones = []
    stats = {
        "wall_index": wall_index,
        "tolerance_m": float(tolerance_m),
        "n_scans_with_constraints": 0,
        "n_no_drill_positions": 0,
        "n_zones": 0,
        "n_unlocated": 0,
        "n_invalid": 0,
    }
    for entry in (gpr_results or {}).get("entries", []) or []:
        scan_wall = (entry.get("line") or {}).get("wall_index")
        # A scan tied to a different wall is not this wall's business. One tied
        # to no line at all is kept whatever the filter: it cannot produce a
        # zone, but its unplaceable positions must still be counted here rather
        # than vanish between the wall filter and the drill.
        if wall_index is not None and scan_wall is not None and scan_wall != wall_index:
            continue
        constraints = entry.get("no_drill")
        if not constraints:
            continue
        if not constraints.get("constraint_valid"):
            stats["n_invalid"] += 1
            continue
        found = constraints.get("no_drill_positions") or []
        if found:
            stats["n_scans_with_constraints"] += 1
        stats["n_no_drill_positions"] += len(found)
        for position in found:
            if not position.get("located"):
                stats["n_unlocated"] += 1
                continue
            zones.append({
                "zone_id": f"{entry.get('key', 'scan')}_{position.get('source_detection_id', '?')}",
                "source_key": entry.get("key"),
                "source_detection_id": position.get("source_detection_id"),
                "reason": position.get("reason", "GPR_HYPERBOLA_DETECTED"),
                "instruction": "NO_DRILL",
                "frame_id": MAP_FRAME,
                "position": position["position_map"],
                "radius_m": float(tolerance_m),
                "wall_index": scan_wall,
                "x_m_scan_local": position.get("x_m"),
                "depth_cm_approx": position.get("depth_cm_approx"),
                "confidence": position.get("confidence"),
            })
    stats["n_zones"] = len(zones)
    return zones, stats


def line_tolerance_from_ctx(ctx):
    """``pokeye_scanned_line_tolerance_m``, or the provisional default."""
    if ctx is None:
        return DEFAULT_LINE_TOLERANCE_M
    value = ctx.get("pokeye_scanned_line_tolerance_m")
    if value in (None, ""):
        return DEFAULT_LINE_TOLERANCE_M
    return float(value)


def scanned_lines_from_gpr(gpr_results, wall_index=None):
    """The wall's GPR lines that were scanned AND analysed. ``(lines, stats)``.

    These are the only places a drill may go. A line qualifies when an export
    was tied to it and the hyperbola pipeline produced a result the POKEYE
    package could read -- a scan whose analysis failed leaves the wall exactly
    as unknown as one that never happened, so it does not qualify and is
    counted separately.

    Returned as map-frame segments, because that is what both the screening
    here and POKEYE's own RANDOM target picking need.
    """
    lines = []
    stats = {"n_scans": 0, "n_lines": 0, "n_not_analysed": 0, "n_unplaced": 0}
    for entry in (gpr_results or {}).get("entries", []) or []:
        line = entry.get("line") or {}
        scan_wall = line.get("wall_index")
        if wall_index is not None and scan_wall is not None and scan_wall != wall_index:
            continue
        stats["n_scans"] += 1
        if not line.get("seg_start") or not line.get("seg_end"):
            # An export tied to no line, or a line ScanWall could not place.
            stats["n_unplaced"] += 1
            continue
        constraints = entry.get("no_drill")
        if not constraints or not constraints.get("constraint_valid"):
            stats["n_not_analysed"] += 1
            continue
        lines.append({
            "line_id": entry.get("key") or line.get("key"),
            "wall_index": scan_wall,
            "frame_id": MAP_FRAME,
            "seg_start": [float(v) for v in line["seg_start"]],
            "seg_end": [float(v) for v in line["seg_end"]],
            "n_hyperbolae": constraints.get("n_no_drill_positions", 0),
        })
    stats["n_lines"] = len(lines)
    return lines, stats


def distance_to_segment(position, seg_start, seg_end):
    """Shortest distance from a point to a segment, ends included."""
    p = np.asarray(position, dtype=float)
    a = np.asarray(seg_start, dtype=float)
    b = np.asarray(seg_end, dtype=float)
    axis = b - a
    length_sq = float(axis @ axis)
    if length_sq < 1e-12:                       # a degenerate line is a point
        return float(np.linalg.norm(p - a))
    # Clamped projection: a target past either end of the sweep was not scanned,
    # however close it is to the line the segment lies on.
    t = min(1.0, max(0.0, float((p - a) @ axis) / length_sq))
    return float(np.linalg.norm(p - (a + t * axis)))


def covering_line(position, lines, tolerance_m=DEFAULT_LINE_TOLERANCE_M):
    """The nearest scanned line that covers ``position``, or None."""
    if position is None or not lines:
        return None
    nearest, nearest_d = None, None
    for line in lines:
        distance = distance_to_segment(position, line["seg_start"], line["seg_end"])
        if distance <= tolerance_m and (nearest_d is None or distance < nearest_d):
            nearest, nearest_d = line, distance
    if nearest is None:
        return None
    covered = dict(nearest)
    covered["distance_m"] = round(nearest_d, 4)
    return covered


# ----------------------------------------------------------------------
# Screening a candidate drill position
# ----------------------------------------------------------------------
def violated_zone(position, zones):
    """The nearest zone ``position`` falls inside, or None.

    The one function any drill target has to pass, whether it came from the HSI
    decision or from an external RANDOM request.
    """
    if position is None or not zones:
        return None
    point = np.asarray(position, dtype=float)
    nearest, nearest_d = None, None
    for zone in zones:
        distance = float(np.linalg.norm(np.asarray(zone["position"], dtype=float) - point))
        if distance <= zone["radius_m"] and (nearest_d is None or distance < nearest_d):
            nearest, nearest_d = zone, distance
    if nearest is None:
        return None
    blocked = dict(nearest)
    blocked["distance_m"] = round(nearest_d, 4)
    return blocked


def refusal(position, zones, lines=None, line_tolerance_m=DEFAULT_LINE_TOLERANCE_M,
            require_coverage=True):
    """Why ``position`` may not be drilled, or None when it may.

    The single gate for any candidate drill point in the map frame, wherever it
    came from -- an HSI cluster, an operator, or a RANDOM pick. Coverage is
    checked before the hyperbolae: "we never looked here" is a different and
    worse answer than "we looked and found a pipe".
    """
    if require_coverage:
        covered = covering_line(position, lines or [], line_tolerance_m)
        if covered is None:
            return {
                "reason": NO_DRILL_NOT_SCANNED,
                "instruction": "NO_DRILL",
                "line_tolerance_m": float(line_tolerance_m),
                "n_scanned_lines": len(lines or []),
                "message": ("No GPR line scanned and analysed this place, so nothing "
                            "is known about what is behind it."),
            }
    zone = violated_zone(position, zones)
    if zone is None:
        return None
    return {
        "reason": zone["reason"],
        "instruction": "NO_DRILL",
        "zone_id": zone["zone_id"],
        "source_key": zone["source_key"],
        "source_detection_id": zone["source_detection_id"],
        "distance_m": zone["distance_m"],
        "radius_m": zone["radius_m"],
        "depth_cm_approx": zone.get("depth_cm_approx"),
    }


def is_drillable(position, zones, lines=None,
                 line_tolerance_m=DEFAULT_LINE_TOLERANCE_M, require_coverage=True):
    """True when ``position`` (map frame) may be drilled.

    Called with ``require_coverage=False`` and no lines this is the hyperbola
    check alone, which is what a caller that has already established coverage
    some other way wants.
    """
    return refusal(position, zones, lines, line_tolerance_m, require_coverage) is None


def screen_targets(targets, zones, lines=None, line_tolerance_m=DEFAULT_LINE_TOLERANCE_M,
                   require_coverage=True, unlocated=0, block_on_unlocated=False):
    """Split clustered targets into the drillable ones and the blocked ones.

    Returns ``(allowed, blocked, stats)``. A blocked target keeps its shape and
    gains ``blocked_by`` so the rejection is auditable: the operator can see
    which hyperbola -- or which absence of a scan -- vetoed which target, rather
    than a target that silently went missing between the decision and the
    hand-off.

    ``require_coverage`` is the project rule that a drill only goes where GPR
    has been. Turning it off leaves the hyperbola veto alone and is for bring-up
    and simulation, where there are no exports to have coverage from.

    ``block_on_unlocated`` is the cautious mission: NO_DRILL positions that
    could not be placed on the wall (a GPR export that matched no scanned line)
    veto every target instead of only being reported. It is not the same
    concern as coverage -- those hyperbolae may well belong to a line that *was*
    scanned, and we simply cannot tell which.
    """
    stats = {
        "n_targets_before": len(targets),
        "n_blocked": 0,
        "n_allowed": 0,
        "n_zones": len(zones),
        "n_scanned_lines": len(lines or []),
        "require_coverage": bool(require_coverage),
        "line_tolerance_m": float(line_tolerance_m),
        "n_unlocated": int(unlocated),
        "blocked_on_unlocated": False,
        "reasons": {},
    }

    if block_on_unlocated and unlocated:
        blocked = []
        for target in targets:
            vetoed = dict(target)
            vetoed["blocked_by"] = {
                "reason": NO_DRILL_UNLOCATED,
                "instruction": "NO_DRILL",
                "n_unlocated": int(unlocated),
                "message": (
                    f"{unlocated} GPR hyperbola position(s) could not be placed on "
                    f"the wall; with pokeye_no_drill_block_on_unlocated set, no "
                    f"target on this wall may be drilled."
                ),
            }
            blocked.append(vetoed)
        stats.update(n_blocked=len(blocked), n_allowed=0, blocked_on_unlocated=True,
                     reasons={NO_DRILL_UNLOCATED: len(blocked)})
        return [], blocked, stats

    allowed, blocked = [], []
    for target in targets:
        why = refusal(target.get("position"), zones, lines,
                      line_tolerance_m, require_coverage)
        if why is None:
            allowed.append(target)
            continue
        vetoed = dict(target)
        vetoed["blocked_by"] = why
        blocked.append(vetoed)

    stats["n_allowed"] = len(allowed)
    stats["n_blocked"] = len(blocked)
    stats["reasons"] = dict(Counter(t["blocked_by"]["reason"] for t in blocked))
    return allowed, blocked, stats


def describe_zones(zones):
    return "; ".join(
        f"{z['zone_id']} at ({z['position'][0]:.2f}, {z['position'][1]:.2f}, "
        f"{z['position'][2]:.2f}) r={z['radius_m']:.2f} m"
        + (f", depth ~{z['depth_cm_approx']:.1f} cm" if z.get("depth_cm_approx") is not None else "")
        for z in zones
    ) or "none"


def describe_lines(lines):
    return "; ".join(
        f"{ln['line_id']} ({ln['seg_start'][0]:.2f}, {ln['seg_start'][1]:.2f}, "
        f"{ln['seg_start'][2]:.2f}) -> ({ln['seg_end'][0]:.2f}, {ln['seg_end'][1]:.2f}, "
        f"{ln['seg_end'][2]:.2f})"
        for ln in lines
    ) or "none"


def describe_blocked(blocked):
    return "; ".join(
        f"{t.get('target_id', '?')} blocked by {t['blocked_by'].get('zone_id', t['blocked_by']['reason'])}"
        + (f" ({t['blocked_by']['distance_m']:.2f} m < {t['blocked_by']['radius_m']:.2f} m)"
           if "radius_m" in t["blocked_by"] else "")
        for t in blocked
    ) or "none"
