# DISCOVER — OLIWALL -> POKEYE decision layer (v1)

Task **(3)**. This package decides whether POKEYE should be requested based **only on the hyperspectral (HSI) result**.

## Scope v1

The current policy is intentionally small and conservative:

- HSI `detected` with confidence >= threshold -> **POKEYE not required**.
- HSI `low_confidence` -> **send to POKEYE** for `MATERIAL_IDENTIFICATION`.
- HSI `quality_rejected` -> **send to POKEYE** for `MATERIAL_IDENTIFICATION`.
- malformed/inconsistent HSI message -> **HOLD / ERROR**. It does **not** automatically trigger a destructive action.

GPR hyperbola/line results are deliberately not used in v1.

## ROS2 boundary

This package does **not** implement ROS2 topics/messages. The software team can deserialize its HSI ROS2 message into a Python dict, call the function below, then serialize the returned dict into its preferred ROS2 interface.

Preferred operational input (one measurement at a time):

```python
{
    "detected": True,
    "material": "gypsum",
    "confidence": 0.994,
    "status": "detected",
    "reason": None,
    "confidence_threshold": 0.8
}
```

The function also accepts the `hsi_result.json` produced by `HYPERSPECTRAL_DISCOVER_pipeline_v2`. If that file contains multiple `samples`, a decision is returned for each sample. That batch result is for inspection/integration; it is **not** an instruction to navigate to an unspecified position.

## Python API

```python
from pokeye_decision import decide_pokeye

hsi_msg = {
    "detected": False,
    "material": None,
    "confidence": 0.63,
    "status": "low_confidence",
    "reason": "confidence below threshold 0.800",
}

decision = decide_pokeye(hsi_msg)
```

Result:

```python
{
    "decision_valid": True,
    "pokeye_required": True,
    "decision": "SEND_TO_POKEYE",
    "reason": "HSI_LOW_CONFIDENCE",
    "requested_action": "MATERIAL_IDENTIFICATION",
    "hsi_evidence": {...}
}
```

If software already knows the robot target/pose, it can be passed through without this package interpreting it:

```python
decision = decide_pokeye(
    hsi_msg,
    target_context={"frame_id": "map", "target_id": "surface_004"}
)
```

`target_context` is opaque metadata: this package does not invent coordinates or navigation commands.

## CLI / JSON test

```bash
python check_setup.py
python test_decision.py
python run_pokeye_decision.py examples/hsi_detected.json
python run_pokeye_decision.py examples/hsi_low_confidence.json
python run_pokeye_decision.py examples/hsi_quality_rejected.json
```

Default CLI output:

```text
outputs/<input_name>/pokeye_decision.json
```

## Output meanings

- `pokeye_required`: boolean operational decision when `decision_valid=true`.
- `decision`: `NO_ACTION`, `SEND_TO_POKEYE`, or `HOLD`.
- `reason`: machine-readable reason.
- `requested_action`: currently only `MATERIAL_IDENTIFICATION`, `NONE`, or `HOLD`.
- `hsi_evidence`: HSI material/confidence/status copied into the decision for traceability.

## Confidence threshold

By default the decision threshold is `0.80`, matching the current HSI integration. If the HSI message includes `confidence_threshold`, that value takes precedence so the two components remain synchronized.
