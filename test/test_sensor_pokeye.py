"""POKEYE decision layer: the vendor policy wall by wall with its GPR lines, and
our clustering of the requests into a short list of drill targets."""

import json

import numpy as np
import pytest

from task_planner_fsm.sensors import gpr, paths, pokeye

EXAMPLES = paths.POKEYE_PROJECT / "test_data"


def _sample(seq, status, x, y=0.0, z=1.0, wall=2, line=0, material=None,
            confidence=None, pose_map=True):
    detected = status == "detected"
    return {
        "seq": seq, "wall_index": wall, "line_idx": line, "seg_idx": 0,
        "travel_m": x,
        "frame": "arm_base", "pose": [x, y, z],
        "pose_map": [x, y, z] if pose_map else None,
        "status": status, "detected": detected,
        "material": (material or "gypsum") if detected else None,
        "confidence": confidence if confidence is not None else (0.95 if detected else 0.5),
        "reason": None if detected else f"reason for {status}",
    }


def _gpr_entry(wall=2, line=0, hyperbolae_x=(), length=10.0, status="ok", key=None):
    """One ``gpr_summary.json`` entry as ``gpr.process_scan`` writes it."""
    result = {"status": status, "calibration": {"scan_distance_m": length},
              "detections": [{"id": f"H{i + 1:03d}", "position": {"x_m": x}}
                             for i, x in enumerate(hyperbolae_x)]}
    hyp = gpr._compact_hyperbolae(result, None)
    hyp["status"] = status
    return {"key": key or f"w{wall:02d}_l{line:02d}_s00",
            "line": {"wall_index": wall, "line_idx": line, "seg_idx": 0},
            "hyperbolae": hyp}


def _clean_gpr(*walls):
    return {"entries": [_gpr_entry(wall=w) for w in walls or (2,)]}


def _good_wall(n, wall=2, seq0=1000):
    """Confident detections far from any patch, so a wall's low-confidence
    share stays under the package's 50 % wall rule."""
    return [_sample(seq0 + i, "detected", 9.0 + 0.01 * i, y=5.0, wall=wall) for i in range(n)]


def _decide(samples, gpr_results=None, config=None):
    return pokeye.decide_samples(samples, 0.8, gpr_results=gpr_results, config_path=config)


def _no_clearance_config(tmp_path):
    return pokeye.runtime_config(tmp_path, require_gpr_clearance=False)


# ----------------------------------------------------------------------
# Layer 1: the vendor decision
# ----------------------------------------------------------------------
def _from_message(msg):
    """A delivered single-measurement message in the adapter's sample shape."""
    meta = msg["metadata"]
    return {"seq": int(meta["Seq"]), "wall_index": int(meta["Wall_Index"]),
            "line_idx": int(meta["Line_Idx"]), "seg_idx": int(meta["Seg_Idx"]),
            "travel_m": float(meta["Travel_m"]), "pose_map": [0.0, 0.0, 0.0],
            **{k: msg[k] for k in ("status", "detected", "material", "confidence", "reason")}}


def test_vendor_examples_decide_as_their_readme_says():
    """The delivered examples with their GPR result (hyperbolae at 1.0 and
    2.5 m) are the contract; the GPR goes in through the summary entry."""
    with open(EXAMPLES / "gpr_hyperbola_example.json") as handle:
        result = json.load(handle)
    entry = {"key": "w00_l00_s00", "line": {"wall_index": 0, "line_idx": 0, "seg_idx": 0},
             "hyperbolae": dict(gpr._compact_hyperbolae(result, None), status="ok")}
    expected = {
        "hsi_detected.json": ("NO_ACTION", "HSI_CONFIDENT_CLASSIFICATION"),
        "hsi_quality_rejected.json": ("NO_ACTION", "HSI_QUALITY_REJECTED"),
        "hsi_low_confidence.json": ("SEND_TO_POKEYE", "HSI_LOW_CONFIDENCE"),
        "hsi_low_confidence_on_hyperbola.json": ("BLOCKED_BY_GPR", "GPR_HYPERBOLA_AT_TARGET"),
    }
    for name, (decision, reason) in expected.items():
        with open(EXAMPLES / name) as handle:
            sample = _from_message(json.load(handle))
        # A confident neighbour keeps the lone low-confidence point under the wall rule.
        [d, _] = _decide([sample, _from_message(json.load(open(EXAMPLES / "hsi_detected.json")))],
                         gpr_results={"entries": [entry]})
        assert (d["decision"], d["reason"]) == (decision, reason), name
        assert d["target_context"]["seq"] == sample["seq"]


def test_a_mostly_low_confidence_wall_gets_one_request():
    samples = [_sample(i, "low_confidence", 0.1 * i, confidence=0.5 + 0.01 * i)
               for i in range(1, 9)] + _good_wall(2)
    decisions = _decide(samples, _clean_gpr())
    sent = [d for d in decisions if d["pokeye_required"]]
    assert len(sent) == 1
    assert sent[0]["reason"] == pokeye.WALL_REQUEST_REASON
    assert sent[0]["target_context"]["seq"] == 1          # the least confident
    assert sum(d["reason"] == "COVERED_BY_WALL_REQUEST" for d in decisions) == 7
    assert pokeye.decision_stats(decisions)["n_wall_requests"] == 1


def test_quality_rejected_and_malformed_verdicts_never_ask_for_pokeye():
    samples = [_sample(1, "quality_rejected", 0.1), _sample(2, "missing", 0.2)] + _good_wall(5)
    decisions = _decide(samples, _clean_gpr())
    assert [d["reason"] for d in decisions[:2]] == ["HSI_QUALITY_REJECTED", "INVALID_HSI_RESULT"]
    assert not any(d["pokeye_required"] for d in decisions)


def test_without_gpr_on_the_line_a_request_is_not_sent():
    """The package's safety default: no evidence the point is clear, no POKEYE."""
    samples = [_sample(1, "low_confidence", 0.5)] + _good_wall(5)
    [d] = _decide(samples, gpr_results=None)[:1]
    assert (d["decision"], d["reason"]) == ("NO_ACTION", "GPR_CLEARANCE_UNKNOWN")
    [d] = _decide(samples, gpr_results={"entries": [_gpr_entry(line=1)]})[:1]
    assert d["reason"] == "GPR_CLEARANCE_UNKNOWN"         # another line's scan clears nothing


def test_a_line_whose_hyperbola_analysis_failed_clears_nothing():
    samples = [_sample(1, "low_confidence", 0.5)] + _good_wall(5)
    [d] = _decide(samples, {"entries": [_gpr_entry(status="error")]})[:1]
    assert d["reason"] == "GPR_CLEARANCE_UNKNOWN"


def test_a_hyperbola_in_either_polarization_blocks_the_point():
    samples = [_sample(1, "low_confidence", 2.5)] + _good_wall(5)
    entries = [_gpr_entry(key="w02_l00_s00_copol"),
               _gpr_entry(key="w02_l00_s00_xpol", hyperbolae_x=(2.52,))]
    [d] = _decide(samples, {"entries": entries})[:1]
    assert d["decision"] == "BLOCKED_BY_GPR"
    assert d["gpr_check"]["nearest_hyperbola"]["id"] == "w02_l00_s00_xpol:H001"


def test_another_walls_line_with_the_same_index_is_never_used():
    """The package keys GPR lines by line/seg only; walls are decided apart."""
    samples = [_sample(1, "low_confidence", 2.5, wall=2)] + _good_wall(5, wall=2)
    gpr_results = {"entries": [_gpr_entry(wall=2),
                               _gpr_entry(wall=3, hyperbolae_x=(2.5,))]}
    [d] = _decide(samples, gpr_results)[:1]
    assert d["decision"] == "SEND_TO_POKEYE"


def test_the_runtime_config_follows_the_coverage_switch(tmp_path):
    path = pokeye.runtime_config(tmp_path, require_gpr_clearance=False)
    cfg = json.loads(path.read_text())
    assert cfg["require_gpr_clearance"] is False
    assert cfg["gpr_exclusion_radius_m"] == 0.1           # the rest stays the sensor team's
    samples = [_sample(1, "low_confidence", 0.5)] + _good_wall(5)
    [d] = _decide(samples, None, config=path)[:1]
    assert d["decision"] == "SEND_TO_POKEYE"


def test_decisions_carry_the_sample_context():
    samples = [_sample(1, "low_confidence", 0.1)] + _good_wall(3)
    [d] = _decide(samples, _clean_gpr())[:1]
    assert d["target_context"]["position"] == [0.1, 0.0, 1.0]
    assert d["target_context"]["frame_id"] == "map"
    assert d["target_context"]["located"] is True
    assert d["target_context"]["travel_m"] == 0.1


# ----------------------------------------------------------------------
# Layer 2: clustering
# ----------------------------------------------------------------------
def _flagged_run(seqs, x0, n, step=0.02, wall=2):
    """``n`` consecutive low-confidence samples starting at x0, ``step`` apart."""
    return [_sample(seqs + i, "low_confidence", x0 + i * step, wall=wall) for i in range(n)]


def _cluster(samples, params=None, wall_index=None, walls=(2,)):
    decisions = _decide(samples, _clean_gpr(*walls))
    return pokeye.cluster_targets(decisions, params or pokeye.ClusterParams(),
                                  wall_index=wall_index)


def test_a_run_of_flagged_samples_becomes_one_target_at_its_centroid():
    samples = _flagged_run(1, 0.50, 6) + _good_wall(20)          # 0.50 .. 0.60
    targets, stats = _cluster(samples, wall_index=2)
    assert len(targets) == 1
    t = targets[0]
    assert t["target_id"] == "w02_c01"
    assert t["n_samples"] == 6
    assert t["position"] == pytest.approx([0.55, 0.0, 1.0], abs=1e-6)
    assert t["reason"] == "HSI_LOW_CONFIDENCE"
    assert t["requested_action"] == "MATERIAL_IDENTIFICATION"
    assert t["sample_seqs"] == [1, 2, 3, 4, 5, 6]
    assert stats["n_clusters"] == 1 and stats["n_targets"] == 1


def test_stray_flagged_samples_are_not_targets():
    """A single low-confidence spectrum in a wall of good ones is noise."""
    samples = [_sample(2, "low_confidence", 0.5),
               _sample(4, "low_confidence", 1.5),
               _sample(5, "low_confidence", 1.52)] + _good_wall(10)   # two, below min 3
    targets, stats = _cluster(samples, pokeye.ClusterParams(min_samples=3))
    assert targets == []
    assert stats["n_flagged"] == 3
    assert stats["n_clusters"] == 2
    assert stats["n_clusters_too_small"] == 2


def test_the_wall_request_is_a_target_whatever_the_cluster_floor():
    """The package already reduced a mostly low-confidence wall to one point."""
    samples = _flagged_run(1, 0.0, 8) + _good_wall(2)
    targets, stats = _cluster(samples, pokeye.ClusterParams(min_samples=3), wall_index=2)
    assert len(targets) == 1
    assert targets[0]["reason"] == pokeye.WALL_REQUEST_REASON
    assert targets[0]["n_samples"] == 1
    assert stats["n_clusters_too_small"] == 0


def test_two_separate_patches_give_two_targets():
    samples = _flagged_run(1, 0.0, 5) + _flagged_run(10, 2.0, 5) + _good_wall(20)
    targets, _ = _cluster(samples)
    assert [t["target_id"] for t in targets] == ["w02_c01", "w02_c02"]
    assert sorted(t["position"][0] for t in targets) == pytest.approx([0.04, 2.04], abs=1e-6)


def test_targets_closer_than_the_spacing_are_merged():
    """Two clusters 20 cm apart (further than the 15 cm radius, closer than the
    30 cm spacing) fold into one weighted target."""
    samples = _flagged_run(1, 0.0, 6) + _flagged_run(10, 0.30, 3) + _good_wall(20)
    params = pokeye.ClusterParams(radius_m=0.15, min_samples=3, min_spacing_m=0.30)
    targets, stats = _cluster(samples, params)
    assert stats["n_targets_before_spacing"] == 2
    assert len(targets) == 1
    t = targets[0]
    assert t["n_samples"] == 9
    assert t["merged_from"] == 2
    # Weighted towards the bigger cluster (centroids 0.05 and 0.32).
    assert 0.05 < t["position"][0] < 0.32
    assert t["sample_seqs"] == list(range(1, 7)) + [10, 11, 12]


def test_the_per_wall_cap_keeps_the_biggest_clusters():
    samples = []
    for k, n in enumerate((3, 8, 5, 4, 6, 7)):
        samples += _flagged_run(100 * k + 1, 1.2 * k, n)
    samples += _good_wall(60)
    targets, stats = _cluster(samples, pokeye.ClusterParams(max_targets_per_wall=3))
    assert stats["n_targets_before_cap"] == 6
    assert [t["n_samples"] for t in targets] == [8, 7, 6]
    assert [t["target_id"] for t in targets] == ["w02_c01", "w02_c02", "w02_c03"]


def test_clustering_is_restricted_to_the_wall_just_scanned():
    """The hyperspectral session spans the mission; only this wall is sent."""
    samples = (_flagged_run(1, 0.0, 5, wall=1) + _good_wall(20, wall=1)
               + _flagged_run(10, 5.0, 5, wall=2) + _good_wall(20, wall=2, seq0=2000))
    decisions = _decide(samples, _clean_gpr(1, 2))
    targets, stats = pokeye.cluster_targets(decisions, wall_index=2)
    assert len(targets) == 1
    assert targets[0]["wall_index"] == 2
    assert stats["n_flagged"] == 5
    everything, _ = pokeye.cluster_targets(decisions, wall_index=None)
    assert len(everything) == 2


def test_flagged_samples_without_a_map_pose_are_counted_not_drilled():
    samples = _flagged_run(1, 0.0, 5) + _good_wall(20)
    for s in samples[:5]:
        s["pose_map"] = None            # arm_base only, base since moved
    targets, stats = _cluster(samples)
    assert targets == []
    assert stats["n_flagged"] == 5 and stats["n_unlocated"] == 5


def test_single_linkage_chains_neighbours():
    """A chain of points each within the radius of the next is one cluster even
    though its ends are far apart; a gap breaks it."""
    pts = np.array([[0.0, 0], [0.1, 0], [0.2, 0], [0.3, 0], [1.0, 0], [1.1, 0]])
    labels = pokeye._single_linkage(pts, 0.15)
    assert len(set(labels[:4])) == 1
    assert len(set(labels[4:])) == 1
    assert labels[0] != labels[4]


def test_outputs_are_written_and_params_come_from_ctx(tmp_path):
    ctx = {"pokeye_cluster_radius_m": 0.2, "pokeye_cluster_min_samples": 2,
           "pokeye_min_target_spacing_m": 0.5, "pokeye_max_targets_per_wall": 1}
    params = pokeye.ClusterParams.from_ctx(ctx)
    assert (params.radius_m, params.min_samples, params.min_spacing_m,
            params.max_targets_per_wall) == (0.2, 2, 0.5, 1)
    decisions = _decide(_flagged_run(1, 0.0, 4) + _good_wall(20), _clean_gpr())
    targets, stats = pokeye.cluster_targets(decisions, params)
    d_path, t_path = pokeye.write_outputs(tmp_path, decisions, targets, stats)
    with open(d_path) as handle:
        assert json.load(handle)["stats"]["n_pokeye_required"] == 4
    with open(t_path) as handle:
        saved = json.load(handle)
    assert saved["stats"]["params"]["radius_m"] == 0.2
    assert saved["targets"][0]["target_id"] == "w02_c01"
    assert "w02_c01" in pokeye.describe_targets(targets)
