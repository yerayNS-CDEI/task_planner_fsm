"""From per-sample HSI verdicts to a short list of places POKEYE should drill.

Three layers, deliberately separate:

1. **Decision** -- the sensor team's ``decide_pokeye`` (FULL PIPELINE, policy
   v2), called once per wall with that wall's GPR hyperbola results, unchanged.
   Only ``low_confidence`` asks for POKEYE; ``detected`` and
   ``quality_rejected`` (a sensor problem, not a material doubt) do not, and a
   malformed verdict is ``NO_ACTION``. A wall that is mostly low-confidence
   gets **one** request for the whole wall. A requested point within 0.10 m of
   a hyperbola on its own line is ``BLOCKED_BY_GPR``, and one whose line has no
   analysed GPR result is ``NO_ACTION`` (``GPR_CLEARANCE_UNKNOWN``).

2. **Aggregation** -- ours. A wall sweep yields hundreds of samples a few
   centimetres apart, so "one drill per flagged sample" is not a target list,
   it is a perforated wall. Flagged samples are clustered by proximity in the
   map frame, small clusters are dropped as noise, each surviving cluster
   becomes one target at its centroid, targets closer than a minimum spacing
   are merged, and the count per wall is capped. Every knob is a ctx param.
   The package's wall request is already one point by design, so it is kept
   whatever the cluster-size floor.

3. **Screening** -- in :mod:`.no_drill`, in the map frame, on the finished
   targets: a cluster centroid is not any one of the points the package
   checked, so the hyperbola veto and the scanned-line rule are applied again
   where the drill would actually go. A blocked target is kept with the
   hyperbola that vetoed it rather than quietly disappearing.

No ROS. Pure numpy for the clustering so it runs on the Jetson without sklearn.
"""

import json
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from . import import_vendor

REQUESTED_ACTION = "MATERIAL_IDENTIFICATION"
DECISIONS_FILENAME = "decisions.json"
TARGETS_FILENAME = "targets.json"
CONFIG_FILENAME = "pokeye_config.json"

# The package's reason for its single request on a mostly low-confidence wall.
WALL_REQUEST_REASON = "HSI_LOW_CONFIDENCE_WALL"


@dataclass
class ClusterParams:
    """Aggregation knobs, with the defaults agreed for the first field runs."""
    # Two flagged samples belong to the same cluster if within this distance.
    radius_m: float = 0.15
    # A cluster smaller than this is a stray spectrum, not a drill site.
    min_samples: int = 3
    # Targets closer than this are merged (a drill needs elbow room).
    min_spacing_m: float = 0.30
    # Largest clusters win when a wall has more.
    max_targets_per_wall: int = 5

    @classmethod
    def from_ctx(cls, ctx):
        p = cls()
        return cls(
            radius_m=float(ctx.get("pokeye_cluster_radius_m", p.radius_m)),
            min_samples=int(ctx.get("pokeye_cluster_min_samples", p.min_samples)),
            min_spacing_m=float(ctx.get("pokeye_min_target_spacing_m", p.min_spacing_m)),
            max_targets_per_wall=int(ctx.get("pokeye_max_targets_per_wall", p.max_targets_per_wall)),
        )


# ----------------------------------------------------------------------
# Layer 1: the vendor decision, one call per wall
# ----------------------------------------------------------------------
def _hsi_sample(sample, index):
    """One entry of the ``hsi_result.json`` ``samples`` list the package reads.

    ``Wall_Index`` groups the wall rule; ``Line_Idx`` / ``Seg_Idx`` pick the GPR
    result of the sample's line; ``Travel_m`` is compared with its hyperbolae.
    """
    return {
        "sample_index": index,
        "detected": bool(sample.get("detected")),
        "material": sample.get("material"),
        "confidence": sample.get("confidence"),
        "status": sample.get("status"),
        "reason": sample.get("reason"),
        "metadata": {
            "Seq": sample.get("seq"),
            "Wall_Index": sample.get("wall_index"),
            "Line_Idx": sample.get("line_idx"),
            "Seg_Idx": sample.get("seg_idx"),
            "Travel_m": sample.get("travel_m"),
        },
    }


def _target_context(sample):
    """Software-owned metadata, attached to each decision for the clustering."""
    pose_map = sample.get("pose_map")
    if pose_map is None and sample.get("frame") == "map":
        pose_map = sample.get("pose")
    return {
        "seq": sample.get("seq"),
        "wall_index": sample.get("wall_index"),
        "line_idx": sample.get("line_idx"),
        "seg_idx": sample.get("seg_idx"),
        "travel_m": sample.get("travel_m"),
        "frame_id": "map" if pose_map is not None else sample.get("frame"),
        "position": pose_map if pose_map is not None else sample.get("pose"),
        "located": pose_map is not None,
    }


def gpr_lines_for_wall(gpr_results, wall_index):
    """The wall's GPR hyperbola results in the list form ``decide_pokeye`` takes.

    ``[{"line_idx", "seg_idx", "gpr_result"}]``, one per scanned line, built
    from the compact entries of ``gpr_summary.json``. A line swept once per
    polarization has two scans: their hyperbolae are pooled (a reflector in
    either one blocks), and the line counts as analysed only if both were --
    the package treats anything but ``status: ok`` as clearance unknown.
    """
    by_line = {}
    for entry in (gpr_results or {}).get("entries", []) or []:
        line = entry.get("line") or {}
        if line.get("wall_index") != wall_index or line.get("line_idx") is None:
            continue
        key = (line.get("line_idx"), line.get("seg_idx"))
        merged = by_line.setdefault(key, {
            "scan_id": [], "status": "ok", "detections": [], "scan_distance_m": None})
        merged["scan_id"].append(entry.get("key"))
        hyp = entry.get("hyperbolae")
        if not hyp or hyp.get("status", "ok") != "ok":
            merged["status"] = "not_analysed"
            continue
        length = hyp.get("scan_distance_m")
        if length is not None:
            merged["scan_distance_m"] = max(float(length), merged["scan_distance_m"] or 0.0)
        for det in hyp.get("detections", []):
            merged["detections"].append({
                "id": f"{entry.get('key')}:{det.get('id')}",
                "position": {"x_m": det.get("x_m")},
            })
    return [{
        "line_idx": line_idx,
        "seg_idx": seg_idx,
        "gpr_result": {
            "scan_id": "+".join(str(k) for k in m["scan_id"]),
            "status": m["status"],
            "calibration": {"scan_distance_m": m["scan_distance_m"]},
            "detections": m["detections"],
        },
    } for (line_idx, seg_idx), m in sorted(by_line.items(), key=lambda kv: str(kv[0]))]


def runtime_config(out_dir, require_gpr_clearance=True):
    """The package's ``config.json`` with the FSM's GPR-clearance switch.

    ``require_gpr_clearance`` follows ``pokeye_require_gpr_coverage``: off only
    for bring-up and simulation, where there are no exports to clear anything.
    Every other value stays the sensor team's.
    """
    pokeye_decision = import_vendor("pokeye_decision")
    cfg = pokeye_decision.decision.load_config(None)
    cfg["require_gpr_clearance"] = bool(require_gpr_clearance)
    path = Path(out_dir) / CONFIG_FILENAME
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as handle:
        json.dump(cfg, handle, indent=2)
    return path


def decide_samples(samples, confidence_threshold, gpr_results=None, config_path=None):
    """Run ``decide_pokeye`` over the samples, one batch per wall. Returns the
    per-sample decisions in input order, each with its ``target_context``.

    Per wall because the package matches GPR lines by ``Line_Idx`` /
    ``Seg_Idx`` only: two walls in one call would pair a sample with another
    wall's line of the same index. ``gpr_results`` is the cumulative
    ``gpr_summary.json`` (or what this pass produced); None means no GPR.
    """
    pokeye_decision = import_vendor("pokeye_decision")
    by_wall = {}
    for i, sample in enumerate(samples):
        by_wall.setdefault(sample.get("wall_index"), []).append(i)

    decisions = [None] * len(samples)
    for wall, indices in by_wall.items():
        hsi_batch = {
            "confidence_threshold": float(confidence_threshold),
            "samples": [_hsi_sample(samples[i], i) for i in indices],
        }
        gpr_lines = gpr_lines_for_wall(gpr_results, wall) if gpr_results else None
        result = pokeye_decision.decide_pokeye(
            hsi_batch, gpr_result=gpr_lines or None,
            config_path=str(config_path) if config_path else None)
        for i, decision in zip(indices, result["decisions"]):
            decision["target_context"] = _target_context(samples[i])
            decisions[i] = decision
    return decisions


def decision_stats(decisions):
    counts = Counter(d.get("decision") for d in decisions)
    reasons = Counter(d.get("reason") for d in decisions
                      if d.get("reason") != "HSI_CONFIDENT_CLASSIFICATION")
    return {
        "n": len(decisions),
        "n_pokeye_required": sum(1 for d in decisions if d.get("pokeye_required")),
        "n_no_action": counts.get("NO_ACTION", 0),
        "n_blocked_by_gpr": counts.get("BLOCKED_BY_GPR", 0),
        "n_wall_requests": sum(1 for d in decisions if d.get("pokeye_required")
                               and d.get("reason") == WALL_REQUEST_REASON),
        "reasons": dict(reasons),
    }


# ----------------------------------------------------------------------
# Layer 2: clustering
# ----------------------------------------------------------------------
def _single_linkage(points, radius):
    """Cluster labels by single linkage: O(n^2) time, O(n) memory.

    One distance vector per point rather than a full matrix, so a mission of a
    few thousand samples stays in a few MB. Union-find with path halving.
    """
    n = len(points)
    parent = np.arange(n)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        d = np.linalg.norm(points[i + 1:] - points[i], axis=1)
        for j in np.nonzero(d <= radius)[0]:
            ri, rj = find(i), find(int(j) + i + 1)
            if ri != rj:
                parent[rj] = ri
    return np.array([find(i) for i in range(n)])


def _majority(values):
    values = [v for v in values if v is not None]
    return Counter(values).most_common(1)[0][0] if values else None


def _make_target(wall_index, members, points):
    """One target from a cluster of decisions and their map positions."""
    centroid = points.mean(axis=0)
    reasons = Counter(m.get("reason") for m in members)
    confidences = [m["hsi_evidence"].get("confidence") for m in members
                   if isinstance(m.get("hsi_evidence"), dict)
                   and m["hsi_evidence"].get("confidence") is not None]
    ctxs = [m.get("target_context", {}) for m in members]
    return {
        "wall_index": wall_index,
        "frame_id": "map",
        "position": [round(float(v), 4) for v in centroid],
        "n_samples": len(members),
        "reason": reasons.most_common(1)[0][0],
        "reasons": dict(reasons),
        "requested_action": _majority(m.get("requested_action") for m in members) or REQUESTED_ACTION,
        "line_idx": _majority(c.get("line_idx") for c in ctxs),
        "seg_idx": _majority(c.get("seg_idx") for c in ctxs),
        "sample_seqs": sorted(c.get("seq") for c in ctxs if c.get("seq") is not None),
        "hsi_evidence": {
            "mean_confidence": round(float(np.mean(confidences)), 4) if confidences else None,
            "statuses": dict(Counter(
                m["hsi_evidence"].get("status") for m in members
                if isinstance(m.get("hsi_evidence"), dict))),
        },
    }


def _merge_by_spacing(targets, min_spacing):
    """Greedy: biggest first; a target too close to an accepted one folds into it."""
    accepted = []
    for t in sorted(targets, key=lambda t: -t["n_samples"]):
        pos = np.asarray(t["position"])
        nearest = None
        for a in accepted:
            if np.linalg.norm(np.asarray(a["position"]) - pos) < min_spacing:
                nearest = a
                break
        if nearest is None:
            accepted.append(t)
            continue
        # Weighted centroid so the merged target sits where most samples are.
        na, nt = nearest["n_samples"], t["n_samples"]
        merged = (np.asarray(nearest["position"]) * na + pos * nt) / (na + nt)
        nearest["position"] = [round(float(v), 4) for v in merged]
        nearest["n_samples"] = na + nt
        for reason, count in t["reasons"].items():
            nearest["reasons"][reason] = nearest["reasons"].get(reason, 0) + count
        nearest["reason"] = Counter(nearest["reasons"]).most_common(1)[0][0]
        nearest["sample_seqs"] = sorted(nearest["sample_seqs"] + t["sample_seqs"])
        nearest.setdefault("merged_from", 1)
        nearest["merged_from"] += 1
    return accepted


def cluster_targets(decisions, params=None, wall_index=None):
    """Turn the flagged decisions into drilling targets.

    ``wall_index`` restricts the output to one wall (the one just scanned);
    None clusters every wall present. Returns ``(targets, stats)``.
    """
    params = params or ClusterParams()
    flagged = [d for d in decisions if d.get("pokeye_required")]
    if wall_index is not None:
        flagged = [d for d in flagged
                   if d.get("target_context", {}).get("wall_index") == wall_index]
    located = [d for d in flagged if d.get("target_context", {}).get("located")]
    stats = {
        "wall_index": wall_index,
        "n_flagged": len(flagged),
        "n_unlocated": len(flagged) - len(located),
        "n_clusters": 0,
        "n_clusters_too_small": 0,
        "n_targets_before_spacing": 0,
        "n_targets_before_cap": 0,
        "n_targets": 0,
        "params": asdict(params),
    }
    if not located:
        return [], stats

    targets = []
    by_wall = {}
    for d in located:
        by_wall.setdefault(d["target_context"].get("wall_index"), []).append(d)

    for wall, members in sorted(by_wall.items(), key=lambda kv: (kv[0] is None, kv[0])):
        points = np.asarray([m["target_context"]["position"] for m in members], dtype=float)
        labels = _single_linkage(points, params.radius_m)
        wall_targets = []
        for label in np.unique(labels):
            idx = np.nonzero(labels == label)[0]
            stats["n_clusters"] += 1
            wall_request = any(members[i].get("reason") == WALL_REQUEST_REASON for i in idx)
            if len(idx) < params.min_samples and not wall_request:
                stats["n_clusters_too_small"] += 1
                continue
            wall_targets.append(_make_target(wall, [members[i] for i in idx], points[idx]))
        stats["n_targets_before_spacing"] += len(wall_targets)
        wall_targets = _merge_by_spacing(wall_targets, params.min_spacing_m)
        stats["n_targets_before_cap"] += len(wall_targets)
        wall_targets = sorted(wall_targets, key=lambda t: -t["n_samples"])[:params.max_targets_per_wall]
        for i, t in enumerate(wall_targets):
            wtxt = f"w{int(wall):02d}" if wall is not None else "wxx"
            t["target_id"] = f"{wtxt}_c{i + 1:02d}"
        targets.extend(wall_targets)

    stats["n_targets"] = len(targets)
    return targets, stats


# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------
def write_outputs(out_dir, decisions, targets, stats, zones=None, blocked=None):
    """Write ``decisions.json`` and ``targets.json``. Returns both paths.

    ``targets.json`` carries the NO_DRILL zones and the targets they vetoed
    alongside the accepted ones, so the file answers "why is there no target
    here?" without cross-reading the GPR summary.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    decisions_path = out_dir / DECISIONS_FILENAME
    targets_path = out_dir / TARGETS_FILENAME
    with open(decisions_path, "w") as handle:
        json.dump({"stats": decision_stats(decisions), "decisions": decisions},
                  handle, indent=2)
    with open(targets_path, "w") as handle:
        json.dump({
            "stats": stats,
            "targets": targets,
            "blocked_targets": blocked or [],
            "drilling_constraints": {
                "coordinate_frame": "map",
                "policy": "A detected GPR hyperbola forbids drilling at its "
                          "position, whatever sent POKEYE there (RANDOM included).",
                "n_no_drill_zones": len(zones or []),
                "no_drill_zones": zones or [],
            },
        }, handle, indent=2)
    return str(decisions_path), str(targets_path)


def describe_targets(targets):
    return "; ".join(
        f"{t['target_id']} at ({t['position'][0]:.2f}, {t['position'][1]:.2f}, "
        f"{t['position'][2]:.2f}) from {t['n_samples']} samples [{t['reason']}]"
        for t in targets
    ) or "none"
