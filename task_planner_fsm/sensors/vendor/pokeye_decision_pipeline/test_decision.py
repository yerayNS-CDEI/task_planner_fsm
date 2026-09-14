from pokeye_decision import decide_pokeye


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


if __name__ == "__main__":
    test_detected_no_pokeye()
    test_low_confidence_pokeye()
    test_quality_rejected_pokeye()
    test_malformed_holds_instead_of_triggering_destructive_action()
    print("All tests passed.")
