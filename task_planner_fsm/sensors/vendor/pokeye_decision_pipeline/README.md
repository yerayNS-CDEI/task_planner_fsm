# DISCOVER — OLIWALL -> POKEYE decision layer (v2)

Task **(3)**. This package has two deliberately separate responsibilities:

1. **Decide whether HSI requires POKEYE material identification.**
2. **Convert detected GPR hyperbolae into NO-DRILL constraints** that POKEYE must respect whenever it drills, including externally requested **RANDOM** drilling.

## Important scope rule

**GPR does not trigger POKEYE.** The decision `pokeye_required` is still based only on the HSI result.

A GPR hyperbola has a different role: if a hyperbola is detected at a horizontal GPR position, that position is returned as **NO_DRILL**, because the reflector may correspond to metal/rebar/pipe/cable or another subsurface object that should not be drilled blindly.

## HSI trigger policy

```text
HSI detected with confidence >= threshold -> NO_ACTION
HSI low_confidence                     -> SEND_TO_POKEYE
HSI quality_rejected                   -> SEND_TO_POKEYE
invalid/inconsistent HSI message       -> HOLD
```

## GPR drilling policy

```text
GPR hyperbola detected at x -> NO_DRILL at that GPR-local position
No hyperbola                -> no GPR-derived forbidden position
```

This constraint applies independently of why POKEYE is drilling. In particular, **RANDOM drilling must also avoid all GPR hyperbola locations**.

### Coordinate warning

The GPR horizontal coordinate is **local to the B-scan / scan path**. It is not a robot/world coordinate. The software team must transform the local GPR position to the robot/global frame using the scan pose, GPR extrinsics and scan direction.

The sensor package intentionally does **not** invent a safety radius around a hyperbola. The project must agree a spatial tolerance/exclusion radius and software must apply it after coordinate transformation. `arc_half_width_m` is included only as diagnostic geometry and is **not** a validated drilling safety radius.

## Python API

```python
from pokeye_decision import decide_pokeye

decision = decide_pokeye(
    hsi_result=hsi_msg,
    gpr_result=gpr_result,       # optional gpr_result.json content
    target_context=target_context,
)
```

A typical output is:

```python
{
    "decision_valid": True,
    "pokeye_required": True,
    "decision": "SEND_TO_POKEYE",
    "reason": "HSI_LOW_CONFIDENCE",
    "requested_action": "MATERIAL_IDENTIFICATION",
    "hsi_evidence": {...},
    "drilling_constraints": {
        "coordinate_frame": "gpr_scan_local",
        "n_no_drill_positions": 2,
        "no_drill_positions": [
            {
                "source_detection_id": "H001",
                "instruction": "NO_DRILL",
                "reason": "GPR_HYPERBOLA_DETECTED",
                "x_m": 0.55,
                "x_cm": 55.0,
                "x_relative": 0.31,
                "depth_cm_approx": 4.2
            }
        ],
        "requires_coordinate_transform": True,
        "requires_exclusion_tolerance_definition": True,
        "exclusion_tolerance_m": None
    }
}
```

A confident HSI classification can still return `pokeye_required=false` while carrying GPR drilling constraints. This is intentional: POKEYE may later be requested for another reason, including RANDOM drilling.

## ROS2 boundary

ROS2 is not implemented here. Recommended integration is:

1. deserialize the HSI result into a Python `dict`;
2. optionally provide the latest associated `gpr_result.json` / GPR dict;
3. call `decide_pokeye(...)`;
4. publish the HSI decision and the transformed NO-DRILL constraints using project-defined ROS2 interfaces.

The software layer owns measurement/pose association and coordinate transforms.

## CLI

```bash
python check_setup.py
python test_decision.py
python run_pokeye_decision.py examples/hsi_low_confidence.json \
    --gpr-json examples/gpr_hyperbolas.json
```

Default output:

```text
outputs/<input_name>/pokeye_decision.json
```

## Main fields

- `pokeye_required`: whether HSI requests POKEYE.
- `decision`: `NO_ACTION`, `SEND_TO_POKEYE`, or `HOLD`.
- `requested_action`: currently `MATERIAL_IDENTIFICATION`, `NONE`, or `HOLD`.
- `drilling_constraints.no_drill_positions[]`: GPR-derived local positions where drilling is forbidden.
- `drilling_constraints.constraint_valid`: whether the GPR constraint payload could be interpreted.
- `target_context`: optional opaque metadata passed through from software.

## Confidence threshold

Default HSI threshold: `0.80`. If the HSI message contains `confidence_threshold`, that value takes precedence so the decision layer remains synchronized with HSI.
