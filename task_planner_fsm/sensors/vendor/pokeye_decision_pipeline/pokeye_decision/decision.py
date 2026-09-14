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


def _single_measurement_decision(
    hsi: dict,
    *,
    confidence_threshold: float,
    pokeye_action: str,
) -> dict:
    """Decide whether POKEYE is required for one HSI classification result.

    This function is deliberately transport-agnostic. `hsi` can be the dict returned
    by the hyperspectral pipeline directly or a dict reconstructed from a ROS2 message.

    Policy v1:
      - valid HSI detection at/above threshold -> no POKEYE
      - HSI low confidence -> POKEYE material identification
      - HSI quality rejection -> POKEYE material identification
      - malformed/inconsistent message -> HOLD/ERROR, never trigger POKEYE automatically
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

    # Explicit sensor outcomes from the HSI pipeline are authoritative.
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
        # A 'detected' status must still be internally coherent. Invalid messages do
        # not cause an automatic destructive action; they are returned as HOLD.
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
            # Defensive check in case the ROS2 message and HSI threshold become inconsistent.
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
            "message": "HSI material classification is sufficiently confident; POKEYE is not required.",
            "hsi_evidence": evidence,
        }

    # Compatibility fallback for a minimal ROS2 payload that might omit `status`.
    if status is None and isinstance(detected, bool):
        if detected is True and material not in (None, "") and confidence is not None:
            if confidence >= confidence_threshold:
                return {
                    "decision_valid": True,
                    "pokeye_required": False,
                    "decision": "NO_ACTION",
                    "reason": "HSI_CONFIDENT_CLASSIFICATION",
                    "requested_action": "NONE",
                    "message": "HSI material classification is sufficiently confident; POKEYE is not required.",
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
            # Without an explicit status we cannot distinguish quality rejection from
            # low confidence. Both are valid reasons for the same v1 POKEYE action.
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
    config_path: str | Path | None = None,
    target_context: dict | None = None,
) -> dict:
    """Return a POKEYE decision from the HSI result.

    Preferred ROS2 usage is one HSI measurement at a time. For convenience, the
    function also accepts a full `hsi_result.json` containing `samples`; in that case
    it returns one decision per sample and an `any_pokeye_required` summary. It does
    not invent a navigation target or aggregate multiple spectra into one material.
    """
    cfg = load_config(config_path)
    default_threshold = float(cfg.get("default_confidence_threshold", 0.8))
    pokeye_action = str(cfg.get("pokeye_action", "MATERIAL_IDENTIFICATION"))

    if not isinstance(hsi_result, dict):
        decision = _single_measurement_decision(
            hsi_result,
            confidence_threshold=default_threshold,
            pokeye_action=pokeye_action,
        )
        if target_context is not None:
            decision["target_context"] = target_context
        return decision

    threshold = _as_float(hsi_result.get("confidence_threshold"))
    if threshold is None:
        threshold = default_threshold

    samples = hsi_result.get("samples")
    # Full HSI output / batch mode.
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
            "note": "Batch summary only. Navigation/action should use the per-sample decision associated with a known robot target.",
        }
        if target_context is not None:
            result["target_context"] = target_context
        return result

    # Single operational measurement.
    decision = _single_measurement_decision(
        hsi_result,
        confidence_threshold=threshold,
        pokeye_action=pokeye_action,
    )
    decision["mode"] = "single"
    decision["confidence_threshold"] = threshold
    if "sample_index" in hsi_result:
        decision["sample_index"] = hsi_result.get("sample_index")
    if hsi_result.get("metadata"):
        decision["metadata"] = hsi_result.get("metadata")
    if target_context is not None:
        # Opaque pass-through: this package never interprets poses/frames.
        decision["target_context"] = target_context
    return decision


def decide_from_json_file(
    input_json: str | Path,
    *,
    output_json: str | Path | None = None,
    config_path: str | Path | None = None,
    target_context: dict | None = None,
) -> dict:
    input_path = Path(input_json)
    with input_path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    result = decide_pokeye(
        payload,
        config_path=config_path,
        target_context=target_context,
    )
    if output_json is not None:
        output_path = Path(output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result
