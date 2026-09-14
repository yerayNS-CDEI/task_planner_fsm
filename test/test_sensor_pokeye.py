"""POKEYE decision layer: the vendor policy per sample, and our clustering of
the flagged samples into a short list of drill targets."""

import json

import numpy as np
import pytest

from task_planner_fsm.sensors import paths, pokeye


def _sample(seq, status, x, y=0.0, z=1.0, wall=2, material=None, confidence=None,
            pose_map=True):
    detected = status == "detected"
    return {
        "seq": seq, "wall_index": wall, "line_idx": 0, "seg_idx": 0,
        "frame": "arm_base", "pose": [x, y, z],
        "pose_map": [x, y, z] if pose_map else None,
        "status": status, "detected": detected,
        "material": material if detected else None,
        "confidence": confidence if confidence is not None else (0.95 if detected else 0.5),
        "reason": None if detected else f"reason for {status}",
    }


# ----------------------------------------------------------------------
# Layer 1: the vendor decision
# ----------------------------------------------------------------------
def test_vendor_examples_decide_as_their_readme_says():
    """The delivered example messages are the contract; check them verbatim."""
    examples = paths.POKEYE_PROJECT / "examples"
    expected = {
        "hsi_detected.json": ("NO_ACTION", False),
        "hsi_low_confidence.json": ("SEND_TO_POKEYE", True),
        "hsi_quality_rejected.json": ("SEND_TO_POKEYE", True),
    }
    for name, (decision, required) in expected.items():
        with open(examples / name) as handle:
            msg = json.load(handle)
        sample = {"seq": 1, "wall_index": 0, "pose": [0, 0, 0], "pose_map": [0, 0, 0],
                  **msg}
        [d] = pokeye.decide_samples([sample], 0.8)
        assert d["decision"] == decision, name
        assert d["pokeye_required"] is required, name
        assert d["target_context"]["seq"] == 1


def test_decisions_carry_the_sample_context_through_untouched():
    samples = [
        _sample(1, "detected", 0.0, material="gypsum"),
        _sample(2, "low_confidence", 0.1),
        _sample(3, "quality_rejected", 0.2),
        _sample(4, "missing", 0.3),         # our marker for "no verdict": must HOLD
    ]
    decisions = pokeye.decide_samples(samples, 0.8)
    assert [d["decision"] for d in decisions] == [
        "NO_ACTION", "SEND_TO_POKEYE", "SEND_TO_POKEYE", "HOLD"]
    assert decisions[1]["target_context"]["position"] == [0.1, 0.0, 1.0]
    assert decisions[1]["target_context"]["frame_id"] == "map"
    assert decisions[1]["target_context"]["located"] is True
    stats = pokeye.decision_stats(decisions)
    assert stats == {
        "n": 4, "n_pokeye_required": 2, "n_no_action": 1, "n_hold": 1,
        "reasons": {"HSI_LOW_CONFIDENCE": 1, "HSI_QUALITY_REJECTED": 1},
    }


def test_a_confident_detection_below_threshold_still_requires_pokeye():
    """The decision honours the threshold in the message, so the two packages
    cannot drift apart silently."""
    [d] = pokeye.decide_samples(
        [_sample(1, "detected", 0.0, material="brick", confidence=0.7)], 0.8)
    assert d["decision"] == "SEND_TO_POKEYE"
    assert d["reason"] == "HSI_LOW_CONFIDENCE"


# ----------------------------------------------------------------------
# Layer 2: clustering
# ----------------------------------------------------------------------
def _flagged_run(seqs, x0, n, step=0.02, status="low_confidence", wall=2):
    """``n`` consecutive flagged samples starting at x0, ``step`` apart."""
    return [_sample(seqs + i, status, x0 + i * step, wall=wall) for i in range(n)]


def test_a_run_of_flagged_samples_becomes_one_target_at_its_centroid():
    samples = _flagged_run(1, 0.50, 6)                      # 0.50 .. 0.60
    decisions = pokeye.decide_samples(samples, 0.8)
    targets, stats = pokeye.cluster_targets(decisions, pokeye.ClusterParams(), wall_index=2)
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
    samples = [_sample(1, "detected", 0.0, material="gypsum"),
               _sample(2, "low_confidence", 0.5),
               _sample(3, "detected", 1.0, material="gypsum"),
               _sample(4, "low_confidence", 1.5),
               _sample(5, "low_confidence", 1.52)]        # two, still below min 3
    decisions = pokeye.decide_samples(samples, 0.8)
    targets, stats = pokeye.cluster_targets(decisions, pokeye.ClusterParams(min_samples=3))
    assert targets == []
    assert stats["n_flagged"] == 3
    assert stats["n_clusters"] == 2
    assert stats["n_clusters_too_small"] == 2


def test_two_separate_patches_give_two_targets():
    samples = _flagged_run(1, 0.0, 5) + _flagged_run(10, 2.0, 5)
    decisions = pokeye.decide_samples(samples, 0.8)
    targets, _ = pokeye.cluster_targets(decisions, pokeye.ClusterParams())
    assert [t["target_id"] for t in targets] == ["w02_c01", "w02_c02"]
    assert sorted(t["position"][0] for t in targets) == pytest.approx([0.04, 2.04], abs=1e-6)


def test_targets_closer_than_the_spacing_are_merged():
    """Two clusters 20 cm apart (further than the 15 cm radius, closer than the
    30 cm spacing) fold into one weighted target."""
    samples = _flagged_run(1, 0.0, 6) + _flagged_run(10, 0.30, 3)   # gap 0.20 > radius
    decisions = pokeye.decide_samples(samples, 0.8)
    params = pokeye.ClusterParams(radius_m=0.15, min_samples=3, min_spacing_m=0.30)
    targets, stats = pokeye.cluster_targets(decisions, params)
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
        samples += _flagged_run(100 * k + 1, 2.0 * k, n)
    decisions = pokeye.decide_samples(samples, 0.8)
    targets, stats = pokeye.cluster_targets(
        decisions, pokeye.ClusterParams(max_targets_per_wall=3))
    assert stats["n_targets_before_cap"] == 6
    assert [t["n_samples"] for t in targets] == [8, 7, 6]
    assert [t["target_id"] for t in targets] == ["w02_c01", "w02_c02", "w02_c03"]


def test_clustering_is_restricted_to_the_wall_just_scanned():
    """The hyperspectral session spans the mission; only this wall is sent."""
    samples = _flagged_run(1, 0.0, 5, wall=1) + _flagged_run(10, 5.0, 5, wall=2)
    decisions = pokeye.decide_samples(samples, 0.8)
    targets, stats = pokeye.cluster_targets(decisions, wall_index=2)
    assert len(targets) == 1
    assert targets[0]["wall_index"] == 2
    assert stats["n_flagged"] == 5
    everything, _ = pokeye.cluster_targets(decisions, wall_index=None)
    assert len(everything) == 2


def test_flagged_samples_without_a_map_pose_are_counted_not_drilled():
    samples = _flagged_run(1, 0.0, 5)
    for s in samples:
        s["pose_map"] = None            # arm_base only, base since moved
    decisions = pokeye.decide_samples(samples, 0.8)
    targets, stats = pokeye.cluster_targets(decisions)
    assert targets == []
    assert stats["n_flagged"] == 5 and stats["n_unlocated"] == 5


def test_hold_decisions_never_become_targets():
    samples = [_sample(i, "missing", 0.01 * i) for i in range(1, 10)]
    decisions = pokeye.decide_samples(samples, 0.8)
    assert all(d["decision"] == "HOLD" for d in decisions)
    targets, stats = pokeye.cluster_targets(decisions)
    assert targets == [] and stats["n_flagged"] == 0


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
    decisions = pokeye.decide_samples(_flagged_run(1, 0.0, 4), 0.8)
    targets, stats = pokeye.cluster_targets(decisions, params)
    d_path, t_path = pokeye.write_outputs(tmp_path, decisions, targets, stats)
    with open(d_path) as handle:
        assert json.load(handle)["stats"]["n_pokeye_required"] == 4
    with open(t_path) as handle:
        saved = json.load(handle)
    assert saved["stats"]["params"]["radius_m"] == 0.2
    assert saved["targets"][0]["target_id"] == "w02_c01"
    assert "w02_c01" in pokeye.describe_targets(targets)
