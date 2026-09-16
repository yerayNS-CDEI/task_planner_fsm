"""GPR hyperbolae as NO_DRILL constraints (POKEYE decision policy v2).

Two things are being checked, and they are not the same thing: that the vendor
policy is applied as delivered (a hyperbola forbids drilling, and never asks
for it), and that the three pieces the sensor package refuses to supply -- the
map frame, the exclusion radius, the rejection -- do what the FSM claims.
"""

import json

import pytest

from task_planner_fsm.sensors import gpr, no_drill, paths, pokeye


def _vendor_gpr_result():
    """The delivered example, which is the contract for the field names."""
    with open(paths.POKEYE_PROJECT / "examples" / "gpr_hyperbolas.json") as handle:
        return json.load(handle)


def _line(wall=2, start=(0.0, 0.0, 1.0), end=(2.0, 0.0, 1.0)):
    return {"key": f"w{wall:02d}_l00_s00", "wall_index": wall,
            "seg_start": list(start), "seg_end": list(end)}


def _entry(key="w02_l00_s00", line=None, constraints=None):
    return {"key": key, "line": line, "associated": line is not None,
            "no_drill": constraints}


def _constraints(result=None, line=None, key="w02_l00_s00"):
    result = _vendor_gpr_result() if result is None else result
    return no_drill.constraints_from_result(
        result, to_map=gpr.map_transform(line), source_key=key)


# ----------------------------------------------------------------------
# The vendor constraint, placed on the wall
# ----------------------------------------------------------------------
def test_the_delivered_example_becomes_two_placed_no_drill_positions():
    out = _constraints(line=_line(start=(1.0, 2.0, 1.0), end=(3.0, 2.0, 1.0)))
    assert out["constraint_valid"] is True
    assert out["hyperbola_detected"] is True
    assert out["n_no_drill_positions"] == 2
    assert out["n_located"] == 2
    first = out["no_drill_positions"][0]
    assert first["instruction"] == "NO_DRILL"
    assert first["reason"] == "GPR_HYPERBOLA_DETECTED"
    assert first["source_detection_id"] == "H001"
    # x_m = 0.55 along a segment starting at (1, 2), so 0.55 m further in x.
    assert first["position_map"] == [1.55, 2.0, 1.0]
    assert first["frame_id"] == "map"
    assert first["depth_cm_approx"] == 4.2
    assert out["no_drill_positions"][1]["position_map"] == [2.12, 2.0, 1.0]


def test_a_scan_tied_to_no_line_keeps_its_positions_but_cannot_place_them():
    """An export that matched nothing is still a hyperbola; it is just nowhere."""
    out = _constraints(line=None)
    assert out["n_no_drill_positions"] == 2
    assert out["n_located"] == 0
    assert all(p["located"] is False for p in out["no_drill_positions"])
    assert all(p["frame_id"] == "gpr_scan_local" for p in out["no_drill_positions"])
    assert all(p["position_map"] is None for p in out["no_drill_positions"])


def test_a_scan_with_no_hyperbola_is_a_valid_constraint_with_nothing_in_it():
    out = _constraints({"hyperbola_detected": False, "n_valid_detections": 0,
                        "detections": []}, line=_line())
    assert out["constraint_valid"] is True
    assert out["n_no_drill_positions"] == 0


def test_a_result_the_vendor_cannot_read_is_marked_invalid_not_empty():
    """The difference matters: "no hyperbolae" is a clean wall, "unreadable" is
    a constraint we do not have."""
    out = _constraints({"detections": "not a list"}, line=_line())
    assert out["constraint_valid"] is False
    assert out["n_no_drill_positions"] == 0
    assert out["message"]


# ----------------------------------------------------------------------
# Zones
# ----------------------------------------------------------------------
def test_zones_carry_the_tolerance_the_project_chose():
    results = {"entries": [_entry(line=_line(), constraints=_constraints(line=_line()))]}
    zones, stats = no_drill.zones_from_gpr(results, tolerance_m=0.25)
    assert [z["radius_m"] for z in zones] == [0.25, 0.25]
    assert [z["zone_id"] for z in zones] == ["w02_l00_s00_H001", "w02_l00_s00_H002"]
    assert [z["position"] for z in zones] == [[0.55, 0.0, 1.0], [1.12, 0.0, 1.0]]
    assert zones[0]["wall_index"] == 2
    assert zones[0]["frame_id"] == "map"
    assert stats["n_zones"] == 2 and stats["n_unlocated"] == 0
    assert stats["tolerance_m"] == 0.25


def test_the_tolerance_is_the_placeholder_until_the_project_sets_one():
    assert no_drill.tolerance_from_ctx(None) == no_drill.DEFAULT_TOLERANCE_M
    assert no_drill.tolerance_from_ctx({}) == no_drill.DEFAULT_TOLERANCE_M
    assert no_drill.tolerance_from_ctx({"pokeye_no_drill_tolerance_m": ""}) == \
        no_drill.DEFAULT_TOLERANCE_M
    assert no_drill.tolerance_from_ctx({"pokeye_no_drill_tolerance_m": 0.4}) == 0.4


def test_only_this_walls_scans_constrain_this_wall():
    results = {"entries": [
        _entry("w01_l00_s00", _line(wall=1), _constraints(line=_line(wall=1))),
        _entry("w02_l00_s00", _line(wall=2), _constraints(line=_line(wall=2))),
    ]}
    zones, stats = no_drill.zones_from_gpr(results, wall_index=2)
    assert {z["source_key"] for z in zones} == {"w02_l00_s00"}
    assert stats["n_zones"] == 2
    everything, _ = no_drill.zones_from_gpr(results)
    assert len(everything) == 4


def test_an_unplaceable_hyperbola_survives_the_wall_filter_as_a_count():
    """It belongs to no wall, so filtering by wall must not make it disappear:
    it is the one case where the FSM knows something is there and not where."""
    results = {"entries": [
        _entry("w02_l00_s00", _line(wall=2), _constraints(line=_line(wall=2))),
        _entry("orphan", None, _constraints(line=None)),
    ]}
    zones, stats = no_drill.zones_from_gpr(results, wall_index=2)
    assert stats["n_zones"] == 2
    assert stats["n_unlocated"] == 2
    assert stats["n_no_drill_positions"] == 4


def test_an_invalid_constraint_is_counted_separately():
    results = {"entries": [_entry(line=_line(), constraints=_constraints(
        {"detections": "not a list"}, line=_line()))]}
    zones, stats = no_drill.zones_from_gpr(results)
    assert zones == [] and stats["n_invalid"] == 1


def test_a_gpr_pass_with_no_constraints_yields_no_zones():
    """GPR disabled, skipped, or run without the hyperbola weights."""
    for results in (None, {}, {"entries": []}, {"entries": [_entry(line=_line())]}):
        zones, stats = no_drill.zones_from_gpr(results)
        assert zones == []
        assert stats["n_no_drill_positions"] == 0 and stats["n_invalid"] == 0


# ----------------------------------------------------------------------
# Screening a candidate position
# ----------------------------------------------------------------------
def _scanned(seg_start=(-1.0, 0.0, 1.0), seg_end=(3.0, 0.0, 1.0), line_id="w02_l00_s00"):
    """A line long enough to cover the positions used below."""
    return [{"line_id": line_id, "wall_index": 2, "frame_id": "map",
             "seg_start": list(seg_start), "seg_end": list(seg_end), "n_hyperbolae": 1}]


def _zone(position, radius=0.15, zone_id="z1"):
    return {"zone_id": zone_id, "source_key": "w02_l00_s00", "source_detection_id": "H001",
            "reason": "GPR_HYPERBOLA_DETECTED", "instruction": "NO_DRILL",
            "frame_id": "map", "position": list(position), "radius_m": radius,
            "wall_index": 2, "depth_cm_approx": 4.2}


def test_a_position_inside_the_radius_is_refused_and_says_by_what():
    zones = [_zone([1.0, 0.0, 1.0])]
    lines = _scanned()
    assert no_drill.is_drillable([1.5, 0.0, 1.0], zones, lines) is True
    assert no_drill.is_drillable([1.1, 0.0, 1.0], zones, lines) is False
    blocked = no_drill.violated_zone([1.1, 0.0, 1.0], zones)
    assert blocked["zone_id"] == "z1"
    assert blocked["distance_m"] == pytest.approx(0.1, abs=1e-6)


def test_the_radius_boundary_is_inclusive_and_distance_is_three_dimensional():
    zones = [_zone([0.0, 0.0, 1.0], radius=0.2)]
    # Coverage is not what is under test here, so only the hyperbola rule runs.
    def drillable(p):
        return no_drill.is_drillable(p, zones, require_coverage=False)
    assert drillable([0.2, 0.0, 1.0]) is False                         # exactly on it
    assert drillable([0.21, 0.0, 1.0]) is True
    # A target a metre higher on the same wall is a different place.
    assert drillable([0.0, 0.0, 2.0]) is True


def test_the_nearest_violated_zone_is_the_one_reported():
    zones = [_zone([0.0, 0.0, 0.0], radius=1.0, zone_id="far"),
             _zone([0.5, 0.0, 0.0], radius=1.0, zone_id="near")]
    assert no_drill.violated_zone([0.4, 0.0, 0.0], zones)["zone_id"] == "near"


def test_nothing_is_refused_when_there_are_no_zones():
    assert no_drill.is_drillable([0.0, 0.0, 1.0], [], _scanned()) is True
    assert no_drill.violated_zone(None, [_zone([0, 0, 0])]) is None


# ----------------------------------------------------------------------
# Coverage: only a scanned and analysed line may be drilled
# ----------------------------------------------------------------------
def test_only_lines_that_were_scanned_and_analysed_are_drillable():
    """A scan whose hyperbola analysis failed leaves the wall as unknown as one
    that never happened, so it is not a place a drill may go."""
    results = {"entries": [
        _entry("w02_l00_s00", _line(), _constraints(line=_line())),
        # Analysed, but the pipeline result was unreadable.
        _entry("w02_l01_s00", _line(), _constraints({"detections": "junk"}, line=_line())),
        # The hyperbola pipeline never ran (no weights, or it crashed).
        _entry("w02_l02_s00", _line(), None),
        # An export that matched no line: nothing to be on.
        _entry("orphan", None, _constraints(line=None)),
    ]}
    lines, stats = no_drill.scanned_lines_from_gpr(results, wall_index=2)
    assert [ln["line_id"] for ln in lines] == ["w02_l00_s00"]
    assert stats["n_lines"] == 1
    assert stats["n_not_analysed"] == 2
    assert stats["n_unplaced"] == 1
    assert lines[0]["seg_start"] == [0.0, 0.0, 1.0] and lines[0]["frame_id"] == "map"


def test_no_gpr_at_all_means_nothing_is_drillable():
    for results in (None, {}, {"entries": []}):
        lines, stats = no_drill.scanned_lines_from_gpr(results)
        assert lines == [] and stats["n_lines"] == 0
    assert no_drill.is_drillable([0.0, 0.0, 1.0], [], []) is False
    why = no_drill.refusal([0.0, 0.0, 1.0], [], [])
    assert why["reason"] == "NOT_ON_A_SCANNED_GPR_LINE"
    assert why["n_scanned_lines"] == 0


def test_distance_is_measured_to_the_segment_not_its_infinite_line():
    """Past the end of the sweep is unscanned however close the line is."""
    a, b = [0.0, 0.0, 1.0], [2.0, 0.0, 1.0]
    assert no_drill.distance_to_segment([1.0, 0.0, 1.0], a, b) == pytest.approx(0.0)
    assert no_drill.distance_to_segment([1.0, 0.3, 1.0], a, b) == pytest.approx(0.3)
    assert no_drill.distance_to_segment([3.0, 0.0, 1.0], a, b) == pytest.approx(1.0)
    assert no_drill.distance_to_segment([0.0, 0.0, 1.5], a, b) == pytest.approx(0.5)
    # A degenerate line is a point, not a division by zero.
    assert no_drill.distance_to_segment([1.0, 0.0, 1.0], a, a) == pytest.approx(1.0)


def test_a_target_beyond_the_swept_stretch_is_not_covered():
    lines = [{"line_id": "w02_l00_s00", "wall_index": 2, "frame_id": "map",
              "seg_start": [0.0, 0.0, 1.0], "seg_end": [2.0, 0.0, 1.0], "n_hyperbolae": 0}]
    assert no_drill.covering_line([1.0, 0.0, 1.0], lines, 0.25)["line_id"] == "w02_l00_s00"
    assert no_drill.covering_line([1.0, 0.2, 1.0], lines, 0.25) is not None
    assert no_drill.covering_line([1.0, 0.3, 1.0], lines, 0.25) is None   # too far off
    assert no_drill.covering_line([2.4, 0.0, 1.0], lines, 0.25) is None   # past the end
    # The default reaches exactly halfway to the next line of a 0.5 m sweep, so
    # consecutive bands meet without overlapping. A tighter value leaves an
    # honest gap between lines; that is the project's call.
    half = no_drill.DEFAULT_LINE_TOLERANCE_M
    assert no_drill.covering_line([1.0, 0.0, 1.0 + half], lines, half) is not None
    assert no_drill.covering_line([1.0, 0.0, 1.0 + half + 0.01], lines, half) is None


def test_a_target_off_every_scanned_line_is_refused_before_the_hyperbolae():
    """The order matters for the audit: "never looked here" is a different and
    worse answer than "looked, and found a pipe"."""
    targets = [_target("w02_c01", [0.0, 0.0, 1.0]),      # on the line
               _target("w02_c02", [0.0, 0.0, 3.0])]      # nowhere near it
    zones = [_zone([0.0, 0.0, 3.0])]                     # and on a hyperbola, too
    allowed, blocked, stats = no_drill.screen_targets(targets, zones, _scanned())
    assert [t["target_id"] for t in allowed] == ["w02_c01"]
    assert blocked[0]["blocked_by"]["reason"] == "NOT_ON_A_SCANNED_GPR_LINE"
    assert stats["require_coverage"] is True
    assert stats["n_scanned_lines"] == 1


def test_the_coverage_rule_can_be_turned_off_for_bring_up():
    targets = [_target("w02_c01", [0.0, 0.0, 9.0])]
    allowed, blocked, stats = no_drill.screen_targets(targets, [], [], require_coverage=False)
    assert allowed == targets and blocked == []
    assert stats["require_coverage"] is False


def test_the_line_tolerance_is_the_projects_to_set():
    assert no_drill.line_tolerance_from_ctx(None) == no_drill.DEFAULT_LINE_TOLERANCE_M
    assert no_drill.line_tolerance_from_ctx({}) == no_drill.DEFAULT_LINE_TOLERANCE_M
    assert no_drill.line_tolerance_from_ctx(
        {"pokeye_scanned_line_tolerance_m": 0.1}) == 0.1


# ----------------------------------------------------------------------
# Screening the wall's targets
# ----------------------------------------------------------------------
def _target(target_id, position, n_samples=5):
    return {"target_id": target_id, "wall_index": 2, "frame_id": "map",
            "position": list(position), "n_samples": n_samples,
            "reason": "HSI_LOW_CONFIDENCE", "requested_action": "MATERIAL_IDENTIFICATION"}


def test_a_target_on_a_hyperbola_is_dropped_and_kept_with_its_reason():
    targets = [_target("w02_c01", [0.55, 0.0, 1.0]), _target("w02_c02", [1.90, 0.0, 1.0])]
    zones = [_zone([0.55, 0.0, 1.0], zone_id="w02_l00_s00_H001")]
    allowed, blocked, stats = no_drill.screen_targets(targets, zones, _scanned())
    assert [t["target_id"] for t in allowed] == ["w02_c02"]
    assert [t["target_id"] for t in blocked] == ["w02_c01"]
    assert blocked[0]["blocked_by"]["zone_id"] == "w02_l00_s00_H001"
    assert blocked[0]["blocked_by"]["reason"] == "GPR_HYPERBOLA_DETECTED"
    assert blocked[0]["blocked_by"]["distance_m"] == 0.0
    # The target itself is untouched, so the audit still shows what was dropped.
    assert blocked[0]["n_samples"] == 5
    assert stats["n_targets_before"] == 2 and stats["n_blocked"] == 1
    assert stats["n_allowed"] == 1 and stats["n_zones"] == 1
    assert stats["reasons"] == {"GPR_HYPERBOLA_DETECTED": 1}
    assert stats["blocked_on_unlocated"] is False
    assert "w02_c01" in no_drill.describe_blocked(blocked)


def test_without_zones_every_target_passes():
    targets = [_target("w02_c01", [0.0, 0.0, 1.0])]
    allowed, blocked, stats = no_drill.screen_targets(targets, [], _scanned())
    assert allowed == targets and blocked == []
    assert stats["n_allowed"] == 1


def test_unplaceable_hyperbolae_only_veto_when_the_mission_asks_for_it():
    """A wall with a properly scanned line AND an orphan export: the target is
    covered, but there are hyperbolae somewhere on the wall we cannot place."""
    targets = [_target("w02_c01", [0.0, 0.0, 1.0])]
    lines = _scanned()
    allowed, blocked, stats = no_drill.screen_targets(targets, [], lines, unlocated=2)
    assert allowed == targets and blocked == []
    assert stats["n_unlocated"] == 2 and stats["blocked_on_unlocated"] is False

    allowed, blocked, stats = no_drill.screen_targets(
        targets, [], lines, unlocated=2, block_on_unlocated=True)
    assert allowed == []
    assert blocked[0]["blocked_by"]["reason"] == "GPR_NO_DRILL_POSITION_NOT_LOCATED"
    assert stats["blocked_on_unlocated"] is True

    # No unplaceable positions: the cautious setting changes nothing.
    allowed, _, _ = no_drill.screen_targets(targets, [], lines, unlocated=0,
                                            block_on_unlocated=True)
    assert allowed == targets


# ----------------------------------------------------------------------
# End to end, against the real vendor policy
# ----------------------------------------------------------------------
def _flagged(seq, x, wall=2):
    return {"seq": seq, "wall_index": wall, "line_idx": 0, "seg_idx": 0,
            "frame": "map", "pose": [x, 0.0, 1.0], "pose_map": [x, 0.0, 1.0],
            "status": "low_confidence", "detected": False, "material": None,
            "confidence": 0.4, "reason": "below threshold"}


def test_hsi_asks_for_a_drill_where_gpr_saw_a_pipe_and_the_drill_does_not_happen():
    """The whole point of policy v2 in one run: the two sensors disagree about
    the same place, and the constraint wins without changing the decision."""
    samples = [_flagged(i, 0.50 + 0.02 * i) for i in range(5)]        # around x = 0.54
    decisions = pokeye.decide_samples(samples, 0.8)
    assert all(d["pokeye_required"] for d in decisions)

    targets, _ = pokeye.cluster_targets(decisions, wall_index=2)
    assert len(targets) == 1

    # The delivered example puts H001 at x = 0.55 m along a scan that starts at
    # the origin -- right on top of the target HSI just asked for.
    results = {"entries": [_entry(line=_line(), constraints=_constraints(line=_line()))]}
    zones, zstats = no_drill.zones_from_gpr(results, tolerance_m=0.15, wall_index=2)
    lines, _ = no_drill.scanned_lines_from_gpr(results, wall_index=2)
    allowed, blocked, _ = no_drill.screen_targets(targets, zones, lines,
                                                  unlocated=zstats["n_unlocated"])
    assert allowed == []
    assert len(blocked) == 1
    assert blocked[0]["blocked_by"]["source_detection_id"] == "H001"
    # The HSI decision is untouched: GPR vetoed the hole, it did not un-flag the wall.
    assert all(d["pokeye_required"] for d in decisions)


def test_the_written_targets_file_explains_the_missing_target(tmp_path):
    samples = [_flagged(i, 0.50 + 0.02 * i) for i in range(5)]
    decisions = pokeye.decide_samples(samples, 0.8)
    targets, stats = pokeye.cluster_targets(decisions, wall_index=2)
    results = {"entries": [_entry(line=_line(), constraints=_constraints(line=_line()))]}
    zones, _ = no_drill.zones_from_gpr(results, wall_index=2)
    lines, _ = no_drill.scanned_lines_from_gpr(results, wall_index=2)
    allowed, blocked, screen_stats = no_drill.screen_targets(targets, zones, lines)
    stats["no_drill"] = screen_stats
    _, targets_path = pokeye.write_outputs(
        tmp_path, decisions, allowed, stats, zones=zones, blocked=blocked)

    with open(targets_path) as handle:
        saved = json.load(handle)
    assert saved["targets"] == []
    assert saved["blocked_targets"][0]["blocked_by"]["reason"] == "GPR_HYPERBOLA_DETECTED"
    assert saved["drilling_constraints"]["n_no_drill_zones"] == 2
    assert saved["drilling_constraints"]["coordinate_frame"] == "map"
    assert saved["stats"]["no_drill"]["n_blocked"] == 1
