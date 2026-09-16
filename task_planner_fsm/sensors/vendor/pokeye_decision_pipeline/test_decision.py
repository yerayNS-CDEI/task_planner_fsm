from pokeye_decision import build_gpr_drilling_constraints, decide_pokeye


def sample_gpr():
    return {
        "hyperbola_detected": True,
        "n_valid_detections": 2,
        "detections": [
            {
                "id": "H001",
                "type": "hyperbola",
                "position": {"x_m": 0.55, "x_cm": 55.0, "x_relative": 0.31},
                "depth": {"depth_cm": 4.2},
                "geometry": {"arc_half_width_m": 0.12},
                "robustness": {"confidence_mean": 0.93},
            },
            {
                "id": "H002",
                "type": "hyperbola",
                "position": {"x_m": 1.12, "x_cm": 112.0, "x_relative": 0.63},
                "depth": {"depth_cm": 7.8},
                "geometry": {"arc_half_width_m": 0.10},
                "robustness": {"confidence_mean": 0.88},
            },
        ],
    }


def test_detected_no_pokeye():
    r = decide_pokeye({
        "detected": True, "material": "gypsum", "confidence": 0.994,
        "status": "detected", "confidence_threshold": 0.8,
    })
    assert r["decision_valid"] is True
    assert r["pokeye_required"] is False
    assert r["decision"] == "NO_ACTION"


def test_low_confidence_pokeye():
    r = decide_pokeye({
        "detected": False, "material": None, "confidence": 0.7848,
        "status": "low_confidence", "confidence_threshold": 0.8,
    })
    assert r["pokeye_required"] is True
    assert r["reason"] == "HSI_LOW_CONFIDENCE"


def test_quality_rejected_pokeye():
    r = decide_pokeye({
        "detected": False, "material": None, "confidence": None,
        "status": "quality_rejected", "reason": "bad spectrum",
    })
    assert r["pokeye_required"] is True
    assert r["reason"] == "HSI_QUALITY_REJECTED"


def test_malformed_holds_instead_of_triggering_destructive_action():
    r = decide_pokeye({"status": "detected", "detected": True, "material": None, "confidence": 0.99})
    assert r["decision_valid"] is False
    assert r["pokeye_required"] is False
    assert r["decision"] == "HOLD"


def test_gpr_hyperbolas_become_no_drill_constraints_but_do_not_trigger_pokeye():
    r = decide_pokeye({
        "detected": True, "material": "gypsum", "confidence": 0.99,
        "status": "detected", "confidence_threshold": 0.8,
    }, gpr_result=sample_gpr())
    assert r["pokeye_required"] is False
    c = r["drilling_constraints"]
    assert c["constraint_valid"] is True
    assert c["n_no_drill_positions"] == 2
    assert c["no_drill_positions"][0]["instruction"] == "NO_DRILL"
    assert c["no_drill_positions"][0]["x_m"] == 0.55


def test_low_confidence_and_gpr_constraints_coexist():
    r = decide_pokeye({
        "detected": False, "material": None, "confidence": 0.40,
        "status": "low_confidence", "confidence_threshold": 0.8,
    }, gpr_result=sample_gpr())
    assert r["pokeye_required"] is True
    assert r["decision"] == "SEND_TO_POKEYE"
    assert r["drilling_constraints"]["n_no_drill_positions"] == 2


def test_empty_gpr_is_valid_and_has_no_forbidden_positions():
    c = build_gpr_drilling_constraints({
        "hyperbola_detected": False,
        "n_valid_detections": 0,
        "detections": [],
    })
    assert c["constraint_valid"] is True
    assert c["n_no_drill_positions"] == 0


if __name__ == "__main__":
    test_detected_no_pokeye()
    test_low_confidence_pokeye()
    test_quality_rejected_pokeye()
    test_malformed_holds_instead_of_triggering_destructive_action()
    test_gpr_hyperbolas_become_no_drill_constraints_but_do_not_trigger_pokeye()
    test_low_confidence_and_gpr_constraints_coexist()
    test_empty_gpr_is_valid_and_has_no_forbidden_positions()
    print("All tests passed.")
