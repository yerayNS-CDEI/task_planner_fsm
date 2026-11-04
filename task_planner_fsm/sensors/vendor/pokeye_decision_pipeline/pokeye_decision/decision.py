from __future__ import annotations

import json
from pathlib import Path
from typing import Any


VALID_HSI_STATUSES = {"detected", "low_confidence", "quality_rejected"}


def load_config(config_path: str | Path | None = None) -> dict:
    path = Path(config_path) if config_path else Path(__file__).with_name("config.json")
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _as_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _decision(
    decision: str,
    reason: str,
    requested_action: str,
    message: str,
    evidence: dict | None,
    *,
    valid: bool = True,
) -> dict:
    return {
        "decision_valid": valid,
        "pokeye_required": decision == "SEND_TO_POKEYE",
        "decision": decision,
        "reason": reason,
        "requested_action": requested_action,
        "message": message,
        "hsi_evidence": evidence,
    }


# ---------------------------------------------------------------------------
# HSI classification of one measurement
# ---------------------------------------------------------------------------

def _classify_hsi(hsi: Any, confidence_threshold: float) -> tuple[str, dict | None, str]:
    """Return (hsi_class, evidence, message) for one HSI measurement.

    hsi_class is one of: confident, low_confidence, quality_rejected, invalid.
    """
    if not isinstance(hsi, dict):
        return "invalid", None, "HSI result is not a dictionary/object."

    status_raw = hsi.get("status")
    status = str(status_raw).strip().lower() if status_raw is not None else None
    detected = hsi.get("detected")
    material = hsi.get("material")
    confidence = _as_float(hsi.get("confidence"))

    evidence = {
        "status": status,
        "detected": detected if isinstance(detected, bool) else None,
        "material": material,
        "confidence": confidence,
        "confidence_threshold": float(confidence_threshold),
        "sensor_reason": hsi.get("reason"),
    }

    # Explicit sensor outcomes from the HSI pipeline are authoritative.
    if status == "quality_rejected":
        return "quality_rejected", evidence, ""
    if status == "low_confidence":
        return "low_confidence", evidence, ""

    if status == "detected":
        # A 'detected' status must still be internally coherent.
        if detected is not True:
            return "invalid", evidence, "HSI status is 'detected' but detected is not true."
        if material is None or str(material).strip() == "":
            return "invalid", evidence, "HSI status is 'detected' but no material was provided."
        if confidence is None:
            return "invalid", evidence, "HSI status is 'detected' but confidence is missing/non-numeric."
        if confidence < confidence_threshold:
            # Defensive: ROS2 message and HSI threshold became inconsistent.
            return "low_confidence", evidence, ""
        return "confident", evidence, ""

    # Compatibility fallback for a minimal ROS2 payload that omits `status`.
    if status is None and isinstance(detected, bool) and confidence is not None:
        if detected is True and material not in (None, "") and confidence >= confidence_threshold:
            return "confident", evidence, ""
        if confidence < confidence_threshold:
            return "low_confidence", evidence, ""

    return "invalid", evidence, f"Unrecognized or incomplete HSI status: {status_raw!r}."


# ---------------------------------------------------------------------------
# GPR clearance: POKEYE must never act where a hyperbola was detected
# ---------------------------------------------------------------------------

def _sample_position_m(sample: dict) -> float | None:
    """Position of the measurement along the scanned line (m), same origin as GPR x_m."""
    pos = _as_float(sample.get("position_along_scan_m"))
    if pos is None and isinstance(sample.get("metadata"), dict):
        pos = _as_float(sample["metadata"].get("Travel_m"))
    return pos


def _line_key(sample: dict) -> tuple[str, str] | None:
    meta = sample.get("metadata") if isinstance(sample.get("metadata"), dict) else {}
    line, seg = meta.get("Line_Idx"), meta.get("Seg_Idx")
    if line is None or seg is None:
        return None
    return str(line).strip(), str(seg).strip()


def _gpr_for_sample(sample: dict, gpr: dict | list | None) -> dict | None:
    """Pick the GPR hyperbola result that covers this HSI sample's line/segment.

    `gpr` is either one gpr_result dict (applies to every sample) or a list of
    {"line_idx", "seg_idx", "gpr_result"} entries matched against the sample metadata.
    """
    if gpr is None:
        return None
    if isinstance(gpr, dict):
        return gpr
    key = _line_key(sample)
    if key is None:
        return None
    for entry in gpr:
        if (str(entry.get("line_idx")).strip(), str(entry.get("seg_idx")).strip()) == key:
            return entry.get("gpr_result")
    return None


def _gpr_check(sample: dict, gpr: dict | list | None, radius_m: float) -> dict:
    """Return {"status": clear|blocked|unknown, ...} for the sample position."""
    check: dict = {"status": "unknown", "exclusion_radius_m": radius_m}
    gpr_result = _gpr_for_sample(sample, gpr)
    if gpr_result is None:
        check["detail"] = "No GPR hyperbola result available for this line/segment."
        return check
    check["scan_id"] = gpr_result.get("scan_id")
    if gpr_result.get("status") != "ok":
        check["detail"] = f"GPR hyperbola result status is {gpr_result.get('status')!r}."
        return check

    pos = _sample_position_m(sample)
    check["position_along_scan_m"] = pos
    if pos is None:
        check["detail"] = "HSI sample has no position along the scan (Travel_m)."
        return check
    scan_len = _as_float((gpr_result.get("calibration") or {}).get("scan_distance_m"))
    if scan_len is not None and not (-radius_m <= pos <= scan_len + radius_m):
        check["detail"] = f"Position {pos:.3f} m is outside the GPR scan (0-{scan_len:.3f} m)."
        return check

    nearest = None
    for det in gpr_result.get("detections") or []:
        x = _as_float((det.get("position") or {}).get("x_m"))
        if x is None:
            continue
        dist = abs(x - pos)
        if nearest is None or dist < nearest["distance_m"]:
            nearest = {"id": det.get("id"), "x_m": x, "distance_m": round(dist, 4)}
    check["nearest_hyperbola"] = nearest
    if nearest is not None and nearest["distance_m"] <= radius_m:
        check["status"] = "blocked"
        check["detail"] = f"GPR hyperbola {nearest['id']} at {nearest['x_m']:.3f} m is within {radius_m} m."
    else:
        check["status"] = "clear"
    return check


def _gate_pokeye(base: dict, check: dict, *, require_gpr: bool, pokeye_action: str) -> dict:
    """Turn a POKEYE candidate into SEND / BLOCKED / NO_ACTION according to the GPR check."""
    evidence = base["hsi_evidence"]
    if check["status"] == "blocked":
        d = _decision(
            "BLOCKED_BY_GPR", "GPR_HYPERBOLA_AT_TARGET", "NONE",
            "HSI is not confident, but GPR detected a hyperbola here; POKEYE must not act at this point.",
            evidence,
        )
    elif check["status"] == "unknown" and require_gpr:
        d = _decision(
            "NO_ACTION", "GPR_CLEARANCE_UNKNOWN", "NONE",
            "HSI is not confident, but the absence of GPR hyperbolas at this point cannot be verified; POKEYE is not sent.",
            evidence,
        )
    else:
        d = _decision(
            "SEND_TO_POKEYE", base["reason"], pokeye_action, base["message"], evidence,
        )
    d["gpr_check"] = check
    return d


def _base_decision(hsi_class: str, evidence: dict | None, invalid_msg: str) -> dict:
    if hsi_class == "invalid":
        return _decision("NO_ACTION", "INVALID_HSI_RESULT", "NONE", invalid_msg, evidence, valid=False)
    if hsi_class == "confident":
        return _decision(
            "NO_ACTION", "HSI_CONFIDENT_CLASSIFICATION", "NONE",
            "HSI material classification is sufficiently confident; POKEYE is not required.",
            evidence,
        )
    if hsi_class == "quality_rejected":
        return _decision(
            "NO_ACTION", "HSI_QUALITY_REJECTED", "NONE",
            "HSI acquisition did not pass the quality filter; this is a sensor problem, not a material doubt, so POKEYE is not requested.",
            evidence,
        )
    # low_confidence: candidate, still subject to the GPR gate and wall aggregation.
    return _decision(
        "SEND_TO_POKEYE", "HSI_LOW_CONFIDENCE", "MATERIAL_IDENTIFICATION",
        "HSI classification confidence is insufficient; request destructive material identification.",
        evidence,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def _wall_key(sample: Any) -> str:
    if isinstance(sample, dict) and isinstance(sample.get("metadata"), dict):
        w = sample["metadata"].get("Wall_Index")
        if w is not None:
            return str(w).strip()
    return "unknown"


def decide_pokeye(
    hsi_result: dict,
    *,
    gpr_result: dict | list | None = None,
    config_path: str | Path | None = None,
    target_context: dict | None = None,
) -> dict:
    """Return a POKEYE decision from the HSI result, gated by the GPR hyperbola result.

    Policy v2:
      - HSI confident -> NO_ACTION
      - HSI quality_rejected -> NO_ACTION (sensor problem, not a material doubt)
      - HSI low_confidence -> POKEYE, but only where GPR shows no hyperbola
      - a wall that is mostly low_confidence -> ONE POKEYE request for the whole wall
        (needs the whole wall: `hsi_result.json` with `samples`)
      - malformed HSI, or GPR clearance not verifiable -> NO_ACTION (reason says why)

    `gpr_result` is the `gpr_result.json` of the hyperbola branch, or a list of
    {"line_idx", "seg_idx", "gpr_result"} when the wall has several GPR lines.
    """
    cfg = load_config(config_path)
    default_threshold = float(cfg.get("default_confidence_threshold", 0.8))
    pokeye_action = str(cfg.get("pokeye_action", "MATERIAL_IDENTIFICATION"))
    wall_ratio = float(cfg.get("wall_low_confidence_ratio", 0.5))
    wall_min = int(cfg.get("wall_min_low_confidence", 2))
    radius_m = float(cfg.get("gpr_exclusion_radius_m", 0.1))
    require_gpr = bool(cfg.get("require_gpr_clearance", True))

    def point_decision(sample: Any, threshold: float) -> dict:
        hsi_class, evidence, msg = _classify_hsi(sample, threshold)
        base = _base_decision(hsi_class, evidence, msg)
        if hsi_class != "low_confidence":
            return base
        check = _gpr_check(sample, gpr_result, radius_m)
        return _gate_pokeye(base, check, require_gpr=require_gpr, pokeye_action=pokeye_action)

    threshold = default_threshold
    if isinstance(hsi_result, dict):
        t = _as_float(hsi_result.get("confidence_threshold"))
        if t is not None:
            threshold = t

    samples = hsi_result.get("samples") if isinstance(hsi_result, dict) else None
    if not (isinstance(samples, list) and not all(k in hsi_result for k in ("status", "detected"))):
        # Single operational measurement: no wall context, point-level rule only.
        decision = point_decision(hsi_result, threshold)
        decision["mode"] = "single"
        decision["confidence_threshold"] = threshold
        if isinstance(hsi_result, dict):
            if "sample_index" in hsi_result:
                decision["sample_index"] = hsi_result.get("sample_index")
            if hsi_result.get("metadata"):
                decision["metadata"] = hsi_result.get("metadata")
        if target_context is not None:
            # Opaque pass-through: this package never interprets poses/frames.
            decision["target_context"] = target_context
        return decision

    # Batch: whole acquisition, grouped by wall.
    decisions: list[dict] = []
    classes: list[str] = []
    for i, sample in enumerate(samples):
        hsi_class, evidence, msg = _classify_hsi(sample, threshold)
        d = _base_decision(hsi_class, evidence, msg)
        d["sample_index"] = sample.get("sample_index", i) if isinstance(sample, dict) else i
        if isinstance(sample, dict) and sample.get("metadata"):
            d["metadata"] = sample.get("metadata")
        decisions.append(d)
        classes.append(hsi_class)

    walls: dict[str, list[int]] = {}
    for i, sample in enumerate(samples):
        walls.setdefault(_wall_key(sample), []).append(i)

    wall_summaries = []
    for wall, idx in walls.items():
        n_classified = sum(classes[i] in ("confident", "low_confidence") for i in idx)
        low = [i for i in idx if classes[i] == "low_confidence"]
        ratio = len(low) / n_classified if n_classified else 0.0
        wall_level = len(low) >= wall_min and ratio >= wall_ratio
        requested = None

        if wall_level:
            # One request for the whole wall: the least confident point that GPR allows.
            order = sorted(low, key=lambda i: (decisions[i]["hsi_evidence"]["confidence"] is None,
                                               decisions[i]["hsi_evidence"]["confidence"] or 0.0, i))
            checks = {i: _gpr_check(samples[i], gpr_result, radius_m) for i in low}
            for i in order:
                if checks[i]["status"] == "clear" or (checks[i]["status"] == "unknown" and not require_gpr):
                    requested = i
                    break
            for i in low:
                if i == requested:
                    decisions[i] = {**decisions[i], **_gate_pokeye(decisions[i], checks[i],
                                    require_gpr=require_gpr, pokeye_action=pokeye_action)}
                    decisions[i]["reason"] = "HSI_LOW_CONFIDENCE_WALL"
                    decisions[i]["message"] = ("Most of this wall is HSI low_confidence; a single POKEYE "
                                               "material identification is requested for the whole wall.")
                elif requested is not None:
                    decisions[i].update(_decision(
                        "NO_ACTION", "COVERED_BY_WALL_REQUEST", "NONE",
                        "Low confidence, but already covered by the single POKEYE request for this wall.",
                        decisions[i]["hsi_evidence"],
                    ))
                    decisions[i]["covered_by_sample_index"] = decisions[requested]["sample_index"]
                    decisions[i]["gpr_check"] = checks[i]
                else:
                    # No point of the wall can be used: report why for each one.
                    decisions[i] = {**decisions[i], **_gate_pokeye(decisions[i], checks[i],
                                    require_gpr=require_gpr, pokeye_action=pokeye_action)}
        else:
            for i in low:
                check = _gpr_check(samples[i], gpr_result, radius_m)
                decisions[i] = {**decisions[i], **_gate_pokeye(decisions[i], check,
                                require_gpr=require_gpr, pokeye_action=pokeye_action)}

        for i in idx:
            decisions[i]["wall_index"] = wall
        wall_summaries.append({
            "wall_index": wall,
            "n_samples": len(idx),
            "n_classified": n_classified,
            "n_low_confidence": len(low),
            "low_confidence_ratio": round(ratio, 4),
            "aggregation": "wall" if wall_level else "point",
            "n_pokeye_requests": sum(int(decisions[i]["pokeye_required"]) for i in idx),
            "requested_sample_indices": [decisions[i]["sample_index"] for i in idx
                                         if decisions[i]["pokeye_required"]],
        })

    result = {
        "mode": "batch",
        "decision_scope": "per_sample",
        "confidence_threshold": threshold,
        "policy": {
            "wall_low_confidence_ratio": wall_ratio,
            "wall_min_low_confidence": wall_min,
            "gpr_exclusion_radius_m": radius_m,
            "require_gpr_clearance": require_gpr,
            "gpr_result_provided": gpr_result is not None,
        },
        "n_samples": len(decisions),
        "n_pokeye_required": sum(int(d["pokeye_required"]) for d in decisions),
        "n_blocked_by_gpr": sum(int(d["decision"] == "BLOCKED_BY_GPR") for d in decisions),
        "n_not_verifiable": sum(int(d["reason"] in ("GPR_CLEARANCE_UNKNOWN", "INVALID_HSI_RESULT"))
                                for d in decisions),
        "any_pokeye_required": any(d["pokeye_required"] for d in decisions),
        "walls": wall_summaries,
        "decisions": decisions,
        "note": "Navigation/action should use the per-sample decisions with pokeye_required=true.",
    }
    if target_context is not None:
        result["target_context"] = target_context
    return result


def decide_from_json_file(
    input_json: str | Path,
    *,
    gpr_json: str | Path | None = None,
    output_json: str | Path | None = None,
    config_path: str | Path | None = None,
    target_context: dict | None = None,
) -> dict:
    with Path(input_json).open("r", encoding="utf-8") as f:
        payload = json.load(f)
    gpr_result = None
    if gpr_json is not None:
        with Path(gpr_json).open("r", encoding="utf-8") as f:
            gpr_result = json.load(f)
    result = decide_pokeye(
        payload,
        gpr_result=gpr_result,
        config_path=config_path,
        target_context=target_context,
    )
    if output_json is not None:
        output_path = Path(output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result
