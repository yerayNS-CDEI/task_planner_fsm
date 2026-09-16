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


def build_gpr_drilling_constraints(gpr_result: dict | None) -> dict:
    """Build NO-DRILL constraints from the GPR hyperbola result.

    Important scope distinction:
    - GPR does NOT decide whether POKEYE is requested.
    - GPR only constrains where destructive drilling may be performed.

    Coordinates are intentionally kept in the local GPR/B-scan frame. The software
    integration layer must transform them to the robot/world frame before applying
    them to a drill target.

    No exclusion radius is invented here. A validated safety/tolerance margin must
    be defined by the project integration/safety logic.
    """
    if gpr_result is None:
        return {
            "gpr_result_provided": False,
            "constraint_valid": False,
            "coordinate_frame": "gpr_scan_local",
            "n_no_drill_positions": 0,
            "no_drill_positions": [],
            "requires_coordinate_transform": True,
            "requires_exclusion_tolerance_definition": True,
            "message": "No GPR hyperbola result was provided; no GPR-based drilling constraints are available.",
        }

    if not isinstance(gpr_result, dict):
        return {
            "gpr_result_provided": True,
            "constraint_valid": False,
            "coordinate_frame": "gpr_scan_local",
            "n_no_drill_positions": 0,
            "no_drill_positions": [],
            "requires_coordinate_transform": True,
            "requires_exclusion_tolerance_definition": True,
            "message": "GPR result is not a dictionary/object.",
        }

    detections = gpr_result.get("detections")
    if detections is None:
        detections = []
    if not isinstance(detections, list):
        return {
            "gpr_result_provided": True,
            "constraint_valid": False,
            "coordinate_frame": "gpr_scan_local",
            "n_no_drill_positions": 0,
            "no_drill_positions": [],
            "requires_coordinate_transform": True,
            "requires_exclusion_tolerance_definition": True,
            "message": "GPR result does not contain a valid detections list.",
        }

    no_drill = []
    for i, det in enumerate(detections):
        if not isinstance(det, dict):
            continue
        det_type = str(det.get("type", "hyperbola")).strip().lower()
        if det_type not in {"hyperbola", ""}:
            continue

        pos = det.get("position") if isinstance(det.get("position"), dict) else {}
        depth = det.get("depth") if isinstance(det.get("depth"), dict) else {}
        geometry = det.get("geometry") if isinstance(det.get("geometry"), dict) else {}

        x_m = _as_float(pos.get("x_m"))
        x_cm = _as_float(pos.get("x_cm"))
        x_relative = _as_float(pos.get("x_relative"))
        if x_m is None and x_cm is not None:
            x_m = x_cm / 100.0
        if x_cm is None and x_m is not None:
            x_cm = x_m * 100.0

        # A hyperbola without a usable horizontal location cannot become an
        # actionable no-drill constraint, so it is skipped rather than guessed.
        if x_m is None and x_relative is None:
            continue

        no_drill.append({
            "source_detection_id": det.get("id", f"H{i + 1:03d}"),
            "reason": "GPR_HYPERBOLA_DETECTED",
            "instruction": "NO_DRILL",
            "x_m": x_m,
            "x_cm": x_cm,
            "x_relative": x_relative,
            "depth_cm_approx": _as_float(depth.get("depth_cm")),
            "confidence": _as_float(
                (det.get("robustness") or {}).get("confidence_mean")
                if isinstance(det.get("robustness"), dict)
                else det.get("confidence")
            ),
            "diagnostic_arc_half_width_m": _as_float(geometry.get("arc_half_width_m")),
        })

    hyperbola_flag = bool(gpr_result.get("hyperbola_detected", len(no_drill) > 0))
    return {
        "gpr_result_provided": True,
        "constraint_valid": True,
        "hyperbola_detected": hyperbola_flag,
        "coordinate_frame": "gpr_scan_local",
        "policy": "A detected GPR hyperbola creates a NO_DRILL location. GPR does not itself trigger POKEYE.",
        "n_no_drill_positions": len(no_drill),
        "no_drill_positions": no_drill,
        "requires_coordinate_transform": True,
        "requires_exclusion_tolerance_definition": True,
        "exclusion_tolerance_m": None,
        "warning": (
            "The GPR x coordinate is local to the B-scan. Software must transform it "
            "to the robot/world frame and apply a project-approved spatial tolerance. "
            "diagnostic_arc_half_width_m is not a validated safety radius."
        ),
    }


def _single_measurement_decision(
    hsi: dict,
    *,
    confidence_threshold: float,
    pokeye_action: str,
) -> dict:
    """Decide whether POKEYE is required for one HSI classification result.

    Policy v2 keeps the v1 trigger logic unchanged:
      - valid HSI detection at/above threshold -> no POKEYE request
      - HSI low confidence -> POKEYE material identification
      - HSI quality rejection -> POKEYE material identification
      - malformed/inconsistent HSI message -> HOLD/ERROR

    GPR is intentionally not used here as a trigger. GPR drilling constraints are
    attached separately by ``decide_pokeye``.
    """
    if not isinstance(hsi, dict):
        return {
            "decision_valid": False,
            "pokeye_required": False,
            "decision": "HOLD",
            "reason": "INVALID_HSI_RESULT",
            "requested_action": "HOLD",
            "message": "HSI result is not a dictionary/object.",
            "hsi_evidence": None,
        }

    status_raw = hsi.get("status")
    status = str(status_raw).strip().lower() if status_raw is not None else None
    detected = hsi.get("detected")
    material = hsi.get("material")
    confidence = _as_float(hsi.get("confidence"))
    sensor_reason = hsi.get("reason")

    evidence = {
        "status": status,
        "detected": detected if isinstance(detected, bool) else None,
        "material": material,
        "confidence": confidence,
        "confidence_threshold": float(confidence_threshold),
        "sensor_reason": sensor_reason,
    }

    if status == "quality_rejected":
        return {
            "decision_valid": True,
            "pokeye_required": True,
            "decision": "SEND_TO_POKEYE",
            "reason": "HSI_QUALITY_REJECTED",
            "requested_action": pokeye_action,
            "message": "HSI acquisition did not pass the quality filter; request destructive material identification.",
            "hsi_evidence": evidence,
        }

    if status == "low_confidence":
        return {
            "decision_valid": True,
            "pokeye_required": True,
            "decision": "SEND_TO_POKEYE",
            "reason": "HSI_LOW_CONFIDENCE",
            "requested_action": pokeye_action,
            "message": "HSI classification confidence is insufficient; request destructive material identification.",
            "hsi_evidence": evidence,
        }

    if status == "detected":
        if detected is not True:
            return {
                "decision_valid": False,
                "pokeye_required": False,
                "decision": "HOLD",
                "reason": "INCONSISTENT_HSI_RESULT",
                "requested_action": "HOLD",
                "message": "HSI status is 'detected' but detected is not true.",
                "hsi_evidence": evidence,
            }
        if material is None or str(material).strip() == "":
            return {
                "decision_valid": False,
                "pokeye_required": False,
                "decision": "HOLD",
                "reason": "INCONSISTENT_HSI_RESULT",
                "requested_action": "HOLD",
                "message": "HSI status is 'detected' but no material was provided.",
                "hsi_evidence": evidence,
            }
        if confidence is None:
            return {
                "decision_valid": False,
                "pokeye_required": False,
                "decision": "HOLD",
                "reason": "INCONSISTENT_HSI_RESULT",
                "requested_action": "HOLD",
                "message": "HSI status is 'detected' but confidence is missing/non-numeric.",
                "hsi_evidence": evidence,
            }
        if confidence < confidence_threshold:
            return {
                "decision_valid": True,
                "pokeye_required": True,
                "decision": "SEND_TO_POKEYE",
                "reason": "HSI_LOW_CONFIDENCE",
                "requested_action": pokeye_action,
                "message": "HSI reports a material, but confidence is below the decision threshold.",
                "hsi_evidence": evidence,
            }
        return {
            "decision_valid": True,
            "pokeye_required": False,
            "decision": "NO_ACTION",
            "reason": "HSI_CONFIDENT_CLASSIFICATION",
            "requested_action": "NONE",
            "message": "HSI material classification is sufficiently confident; POKEYE is not required by HSI.",
            "hsi_evidence": evidence,
        }

    if status is None and isinstance(detected, bool):
        if detected is True and material not in (None, "") and confidence is not None:
            if confidence >= confidence_threshold:
                return {
                    "decision_valid": True,
                    "pokeye_required": False,
                    "decision": "NO_ACTION",
                    "reason": "HSI_CONFIDENT_CLASSIFICATION",
                    "requested_action": "NONE",
                    "message": "HSI material classification is sufficiently confident; POKEYE is not required by HSI.",
                    "hsi_evidence": evidence,
                }
            return {
                "decision_valid": True,
                "pokeye_required": True,
                "decision": "SEND_TO_POKEYE",
                "reason": "HSI_LOW_CONFIDENCE",
                "requested_action": pokeye_action,
                "message": "HSI classification confidence is insufficient; request destructive material identification.",
                "hsi_evidence": evidence,
            }

        if detected is False:
            return {
                "decision_valid": True,
                "pokeye_required": True,
                "decision": "SEND_TO_POKEYE",
                "reason": "HSI_NOT_CONFIDENTLY_IDENTIFIED",
                "requested_action": pokeye_action,
                "message": "HSI did not return a confident material identification; request destructive material identification.",
                "hsi_evidence": evidence,
            }

    return {
        "decision_valid": False,
        "pokeye_required": False,
        "decision": "HOLD",
        "reason": "INVALID_HSI_RESULT",
        "requested_action": "HOLD",
        "message": f"Unrecognized or incomplete HSI status: {status_raw!r}.",
        "hsi_evidence": evidence,
    }


def decide_pokeye(
    hsi_result: dict,
    *,
    gpr_result: dict | None = None,
    config_path: str | Path | None = None,
    target_context: dict | None = None,
) -> dict:
    """Return the HSI-based POKEYE decision plus optional GPR no-drill constraints.

    ``pokeye_required`` is still decided exclusively from HSI. If a GPR hyperbola
    result is provided, it is converted into drilling constraints that must also be
    respected when POKEYE drills for another reason, including RANDOM drilling.
    """
    cfg = load_config(config_path)
    default_threshold = float(cfg.get("default_confidence_threshold", 0.8))
    pokeye_action = str(cfg.get("pokeye_action", "MATERIAL_IDENTIFICATION"))
    drilling_constraints = build_gpr_drilling_constraints(gpr_result)

    if not isinstance(hsi_result, dict):
        decision = _single_measurement_decision(
            hsi_result,
            confidence_threshold=default_threshold,
            pokeye_action=pokeye_action,
        )
        decision["drilling_constraints"] = drilling_constraints
        if target_context is not None:
            decision["target_context"] = target_context
        return decision

    threshold = _as_float(hsi_result.get("confidence_threshold"))
    if threshold is None:
        threshold = default_threshold

    samples = hsi_result.get("samples")
    if isinstance(samples, list) and not all(k in hsi_result for k in ("status", "detected")):
        decisions = []
        for i, sample in enumerate(samples):
            d = _single_measurement_decision(
                sample,
                confidence_threshold=threshold,
                pokeye_action=pokeye_action,
            )
            d["sample_index"] = sample.get("sample_index", i) if isinstance(sample, dict) else i
            if isinstance(sample, dict) and sample.get("metadata"):
                d["metadata"] = sample.get("metadata")
            decisions.append(d)

        result = {
            "mode": "batch",
            "decision_scope": "per_sample",
            "confidence_threshold": threshold,
            "n_samples": len(decisions),
            "n_pokeye_required": sum(int(d["pokeye_required"]) for d in decisions),
            "n_hold": sum(int(d["decision"] == "HOLD") for d in decisions),
            "any_pokeye_required": any(d["pokeye_required"] for d in decisions),
            "decisions": decisions,
            "drilling_constraints": drilling_constraints,
            "note": "Batch summary only. Navigation/action should use the per-sample decision associated with a known robot target.",
        }
        if target_context is not None:
            result["target_context"] = target_context
        return result

    decision = _single_measurement_decision(
        hsi_result,
        confidence_threshold=threshold,
        pokeye_action=pokeye_action,
    )
    decision["mode"] = "single"
    decision["confidence_threshold"] = threshold
    decision["drilling_constraints"] = drilling_constraints
    if "sample_index" in hsi_result:
        decision["sample_index"] = hsi_result.get("sample_index")
    if hsi_result.get("metadata"):
        decision["metadata"] = hsi_result.get("metadata")
    if target_context is not None:
        decision["target_context"] = target_context
    return decision


def decide_from_json_file(
    input_json: str | Path,
    *,
    gpr_json: str | Path | None = None,
    output_json: str | Path | None = None,
    config_path: str | Path | None = None,
    target_context: dict | None = None,
) -> dict:
    input_path = Path(input_json)
    with input_path.open("r", encoding="utf-8") as f:
        payload = json.load(f)

    gpr_payload = None
    if gpr_json is not None:
        with Path(gpr_json).open("r", encoding="utf-8") as f:
            gpr_payload = json.load(f)

    result = decide_pokeye(
        payload,
        gpr_result=gpr_payload,
        config_path=config_path,
        target_context=target_context,
    )
    if output_json is not None:
        output_path = Path(output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result
