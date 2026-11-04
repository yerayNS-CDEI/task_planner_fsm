from pokeye_decision import decide_pokeye


GPR = {
    "status": "ok", "scan_id": "test",
    "calibration": {"scan_distance_m": 5.0},
    "detections": [{"id": "H001", "position": {"x_m": 1.0}}, {"id": "H002", "position": {"x_m": 2.5}}],
}


def _meta(travel):
    return {"Wall_Index": "0", "Line_Idx": "0", "Seg_Idx": "0", "Travel_m": str(travel)}


def _sample(i, status, conf, travel=None):
    return {
        "sample_index": i, "detected": status == "detected",
        "material": "gypsum" if status == "detected" else None,
        "confidence": conf, "status": status,
        "metadata": _meta(0.5 * i if travel is None else travel),
    }


def test_detected_no_pokeye():
    r = decide_pokeye({
        "detected": True, "material": "gypsum", "confidence": 0.994,
        "status": "detected", "confidence_threshold": 0.8,
    })
    assert r["decision_valid"] is True
    assert r["pokeye_required"] is False
    assert r["decision"] == "NO_ACTION"


def test_low_confidence_pokeye_where_gpr_is_clear():
    r = decide_pokeye(_sample(0, "low_confidence", 0.7848, travel=3.0), gpr_result=GPR)
    assert r["pokeye_required"] is True
    assert r["decision"] == "SEND_TO_POKEYE"
    assert r["reason"] == "HSI_LOW_CONFIDENCE"
    assert r["gpr_check"]["status"] == "clear"


def test_low_confidence_on_hyperbola_is_blocked():
    r = decide_pokeye(_sample(0, "low_confidence", 0.7848, travel=2.52), gpr_result=GPR)
    assert r["pokeye_required"] is False
    assert r["decision"] == "BLOCKED_BY_GPR"
    assert r["gpr_check"]["nearest_hyperbola"]["id"] == "H002"


def test_low_confidence_without_gpr_no_action():
    r = decide_pokeye(_sample(0, "low_confidence", 0.7848, travel=3.0))
    assert r["pokeye_required"] is False
    assert r["decision"] == "NO_ACTION"
    assert r["reason"] == "GPR_CLEARANCE_UNKNOWN"


def test_low_confidence_outside_gpr_scan_no_action():
    r = decide_pokeye(_sample(0, "low_confidence", 0.7848, travel=7.0), gpr_result=GPR)
    assert r["decision"] == "NO_ACTION"
    assert r["reason"] == "GPR_CLEARANCE_UNKNOWN"


def test_quality_rejected_no_pokeye():
    r = decide_pokeye({
        "detected": False, "material": None, "confidence": None,
        "status": "quality_rejected", "reason": "bad spectrum",
    }, gpr_result=GPR)
    assert r["pokeye_required"] is False
    assert r["decision"] == "NO_ACTION"
    assert r["reason"] == "HSI_QUALITY_REJECTED"


def test_wall_mostly_low_confidence_single_request():
    confs = [0.95, 0.61, 0.72, 0.70, 0.66, 0.55, 0.74, 0.69, 0.71, 0.97]
    samples = [_sample(i, "detected" if c >= 0.8 else "low_confidence", c) for i, c in enumerate(confs)]
    r = decide_pokeye({"confidence_threshold": 0.8, "samples": samples}, gpr_result=GPR)
    assert r["n_pokeye_required"] == 1
    assert r["walls"][0]["aggregation"] == "wall"
    # Least confident point (0.55 at 2.5 m) is on a hyperbola -> next least confident (0.61).
    assert r["walls"][0]["requested_sample_indices"] == [1]
    assert r["decisions"][1]["reason"] == "HSI_LOW_CONFIDENCE_WALL"
    assert r["decisions"][2]["reason"] == "COVERED_BY_WALL_REQUEST"


def test_wall_all_low_confidence_on_hyperbolas_blocked():
    samples = [_sample(0, "low_confidence", 0.6, travel=1.0), _sample(1, "low_confidence", 0.7, travel=2.5)]
    r = decide_pokeye({"samples": samples}, gpr_result=GPR)
    assert r["n_pokeye_required"] == 0
    assert r["n_blocked_by_gpr"] == 2


def test_wall_isolated_low_confidence_point_request():
    samples = [_sample(i, "detected", 0.97) for i in range(10)]
    samples[6] = _sample(6, "low_confidence", 0.74)
    samples[8] = _sample(8, "quality_rejected", None)
    r = decide_pokeye({"samples": samples}, gpr_result=GPR)
    assert r["walls"][0]["aggregation"] == "point"
    assert r["walls"][0]["requested_sample_indices"] == [6]
    assert r["decisions"][8]["decision"] == "NO_ACTION"


def test_malformed_no_action_instead_of_triggering_destructive_action():
    r = decide_pokeye({"status": "detected", "detected": True, "material": None, "confidence": 0.99})
    assert r["decision_valid"] is False
    assert r["pokeye_required"] is False
    assert r["decision"] == "NO_ACTION"
    assert r["reason"] == "INVALID_HSI_RESULT"


if __name__ == "__main__":
    test_detected_no_pokeye()
    test_low_confidence_pokeye_where_gpr_is_clear()
    test_low_confidence_on_hyperbola_is_blocked()
    test_low_confidence_without_gpr_no_action()
    test_low_confidence_outside_gpr_scan_no_action()
    test_quality_rejected_no_pokeye()
    test_wall_mostly_low_confidence_single_request()
    test_wall_all_low_confidence_on_hyperbolas_blocked()
    test_wall_isolated_low_confidence_point_request()
    test_malformed_no_action_instead_of_triggering_destructive_action()
    print("All tests passed.")
