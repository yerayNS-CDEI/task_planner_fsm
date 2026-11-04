# DISCOVER — OLIWALL -> POKEYE decision layer (v2)

Task **(3)**. This package decides whether POKEYE should be requested from the **hyperspectral (HSI) result**, and makes sure POKEYE never acts **where the GPR detected a hyperbola**.

```text
Pokeye Decision/
├── README.md, requirements.txt
├── codes/
│   ├── run_pokeye_decision.py   # CLI
│   ├── test_decision.py         # unit tests (no files needed)
│   ├── check_setup.py
│   └── pokeye_decision/         # decision.py + config.json
├── test_data/                   # HSI examples (single points and whole walls) + a synthetic GPR result
└── test_output/                 # their decisions + the decision on the real HSI test session
```

Python standard library only (`requirements.txt` is empty on purpose).

## Policy v2

| HSI result | GPR at that point | Decision | `requested_action` |
|---|---|---|---|
| `detected`, confidence ≥ 0.80 | (not checked) | `NO_ACTION` | `NONE` |
| `quality_rejected` | (not checked) | `NO_ACTION` (`HSI_QUALITY_REJECTED`) | `NONE` |
| `low_confidence`, isolated in its wall | no hyperbola | `SEND_TO_POKEYE` (`HSI_LOW_CONFIDENCE`) | `MATERIAL_IDENTIFICATION` |
| `low_confidence`, most of the wall | no hyperbola | **one** `SEND_TO_POKEYE` for the wall (`HSI_LOW_CONFIDENCE_WALL`); the other points `NO_ACTION` (`COVERED_BY_WALL_REQUEST`) | `MATERIAL_IDENTIFICATION` once |
| `low_confidence` | hyperbola within 0.10 m | `BLOCKED_BY_GPR` (`GPR_HYPERBOLA_AT_TARGET`) | `NONE` |
| `low_confidence` | cannot be verified (no GPR result, no position, outside the scan) | `NO_ACTION` (`GPR_CLEARANCE_UNKNOWN`) | `NONE` |
| malformed / inconsistent | — | `NO_ACTION` (`INVALID_HSI_RESULT`, `decision_valid=false`) | `NONE` |

- **Quality rejected** is a sensor problem (bad spectrum), not a doubt about the material, so it does not request POKEYE.
- **Wall rule.** A wall is one acquisition (`metadata.Wall_Index`). Among its classified points (`detected` + `low_confidence`; `quality_rejected` excluded), if the share of `low_confidence` is ≥ `wall_low_confidence_ratio` (0.5) and there are at least `wall_min_low_confidence` (2) of them, only **one** POKEYE request is sent for the wall. It goes to the least confident point that GPR allows; if that point is on a hyperbola, the next one is used. If every low-confidence point is on a hyperbola, the wall gets no request. Otherwise (isolated low confidence, e.g. 1/10) POKEYE is requested at each of those points.
- **GPR rule.** The HSI point position along the line (`metadata.Travel_m`, or `position_along_scan_m`) is compared with each hyperbola `position.x_m` of the GPR `gpr_result.json` for the same line. Within `gpr_exclusion_radius_m` (0.10 m) the point is blocked. Both positions must share the same origin and direction along the line; that pairing is the software team's responsibility.
- **Safety default.** `require_gpr_clearance: true`: without GPR evidence that the point is free of hyperbolas, POKEYE is not sent (`NO_ACTION`, `GPR_CLEARANCE_UNKNOWN`). Set it to `false` only if POKEYE may act without GPR.
- The wall rule needs the whole wall at once (`hsi_result.json` with `samples`). A single message only gets the point rule.

All values are in `codes/pokeye_decision/config.json`.

## ROS2 boundary

This package does **not** implement ROS2 topics/messages. The software team can deserialize its HSI and GPR messages into Python dicts, call the function below, then serialize the returned dict into its preferred ROS2 interface.

## Python API

```python
from pokeye_decision import decide_pokeye

decision = decide_pokeye(hsi_result, gpr_result=gpr_result)
```

- `hsi_result`: one HSI message, or the full `hsi_result.json` from `../Hyperspectral` (batch: one decision per sample, grouped by wall).
- `gpr_result`: the `gpr_result.json` of the hyperbola branch for that line, or, for a wall with several GPR lines, a list `[{"line_idx": "0", "seg_idx": "0", "gpr_result": {...}}, ...]` matched against the HSI `metadata.Line_Idx/Seg_Idx`.
- `target_context` (optional): opaque metadata copied to the output; this package does not invent coordinates or navigation commands.

Single result:

```python
{
    "decision_valid": True,
    "pokeye_required": True,
    "decision": "SEND_TO_POKEYE",
    "reason": "HSI_LOW_CONFIDENCE",
    "requested_action": "MATERIAL_IDENTIFICATION",
    "hsi_evidence": {...},
    "gpr_check": {"status": "clear", "position_along_scan_m": 3.0,
                  "nearest_hyperbola": {"id": "H002", "x_m": 2.5, "distance_m": 0.5}, ...}
}
```

Batch result: `decisions[]` (one per sample, with `wall_index`), `walls[]` (`aggregation` = `wall` / `point`, `requested_sample_indices`), and counters `n_pokeye_required`, `n_blocked_by_gpr`, `n_not_verifiable` (GPR clearance unknown or invalid HSI).

## CLI / JSON test

```bash
cd codes
python check_setup.py
python test_decision.py
python run_pokeye_decision.py ../test_data/hsi_low_confidence.json --gpr ../test_data/gpr_hyperbola_example.json
python run_pokeye_decision.py ../test_data/hsi_wall_mostly_low_confidence.json --gpr ../test_data/gpr_hyperbola_example.json
python run_pokeye_decision.py ../../Hyperspectral/test_output/session_20261002_131059/hsi_result.json \
       --gpr ../../GPR/test_output/scan_wall_line_1_seg_1_20261008_110058_012/hyperbola/gpr_result.json \
       --output ../test_output/session_20261002_131059/pokeye_decision.json
```

Default CLI output (when `--output` is omitted): `Pokeye Decision/test_output/<input_name>/pokeye_decision.json`.

| Test input (`--gpr gpr_hyperbola_example.json`, hyperbolas at 1.0 and 2.5 m) | Result |
|---|---|
| `hsi_detected` | `NO_ACTION` |
| `hsi_quality_rejected` | `NO_ACTION` |
| `hsi_low_confidence` (3.0 m) | `SEND_TO_POKEYE` |
| `hsi_low_confidence_on_hyperbola` (2.52 m) | `BLOCKED_BY_GPR` |
| `hsi_wall_mostly_low_confidence` (8/10 low) | 1 request (sample 1; the least confident, sample 5, is on a hyperbola) |
| `hsi_wall_one_low_confidence` (1/9 low + 1 quality rejected) | 1 request at sample 6 |

Real HSI session (16 spectra): 1 `low_confidence` (sample 5, 2.51 m) and 1 `quality_rejected` (sample 15) -> no POKEYE request. The `low_confidence` point is `BLOCKED_BY_GPR` by hyperbola H008 (2.50 m) of the 2026-10-08 GPR scan. **That HSI/GPR pairing is only illustrative**: the two acquisitions are not from the same line.

## Output meanings

- `pokeye_required`: `true` only for `SEND_TO_POKEYE`.
- `decision`: `NO_ACTION`, `SEND_TO_POKEYE`, or `BLOCKED_BY_GPR`.
- `reason`: machine-readable reason (see the table above).
- `requested_action`: `MATERIAL_IDENTIFICATION` or `NONE`.
- `hsi_evidence`: HSI material/confidence/status copied into the decision for traceability.
- `gpr_check`: `status` (`clear` / `blocked` / `unknown`), point position, nearest hyperbola and distance.

## Confidence threshold

By default the decision threshold is `0.80`, matching the current HSI integration. If the HSI message includes `confidence_threshold`, that value takes precedence so the two components remain synchronized.
