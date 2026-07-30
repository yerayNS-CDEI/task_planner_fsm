from __future__ import annotations

from typing import Any, Dict, Iterable, List


# Horseshoe layout turned on its side so the graph is TALL rather than WIDE and
# fits a docked RViz side panel. Nodes are 220x92 in the panel; the columns are
# spaced 400px (180px gutter for the return edges) and rows 150px (58px of
# vertical breathing room -- the edge router treats nodes as obstacles inflated
# by 8px per side, so rows must stay at least ~110px apart).
#
# The mapping/perception/navigation chain runs TOP->BOTTOM down the middle
# column and fans out into the three scan states side by side at the bottom;
# they converge back into SensorDataProcessing and the return chain runs
# BOTTOM->TOP up the right column. ArmFolding sits level with
# WallTargetSelection so the per-target loop-back edge stays short.
#
# Error is an isolated sink (no edges are routed to it) that lights up when
# active; it floats at the top right, clear of the columns. Coordinates are the
# top-left corner of each node.
_NODE_POSITIONS = {
    # Middle column (top -> bottom): mapping, perception & navigation
    "Initialization": (480, 40),
    "CreateMap": (480, 190),
    "ObjectID": (480, 340),
    "WallLinesComputation": (480, 490),
    "GeometryReconstruction": (480, 640),
    "ComputeWallPoints": (480, 790),
    "WallTargetSelection": (480, 940),
    "NavigateToTarget": (480, 1090),
    "ArmUnfolding": (480, 1240),
    # Bottom of the horseshoe: the three scan branches, side by side
    "ScanFloor": (40, 1390),
    "ScanWall": (480, 1390),
    "ScanCeiling": (880, 1390),
    # Right column (bottom -> top): processing, upload & return
    "SensorDataProcessing": (880, 1240),
    "SendDataToPokeye": (880, 1090),
    "ArmFolding": (880, 940),
    "HomePosition": (880, 790),
    "Finished": (880, 640),
    # Isolated failure sink
    "Error": (880, 40),
    # Disabled legacy states; the coordinates below are from the old wide
    # layout and need redoing if these are ever brought back.
    # "AreasOfInterest": (1320, 440),
    # "WallDiscretization": (1640, 440),
    # "BasePlacement": (1960, 440),
    # "ExhaustiveScan": (1320, 640),
}

_DEFAULT_EDGES = [
    ("Initialization", "CreateMap", "forward"),
    ("CreateMap", "ObjectID", "forward"),
    ("ObjectID", "WallLinesComputation", "forward"),
    ("WallLinesComputation", "GeometryReconstruction", "forward"),
    ("GeometryReconstruction", "ComputeWallPoints", "forward"),
    ("ComputeWallPoints", "WallTargetSelection", "forward"),
    ("WallTargetSelection", "NavigateToTarget", "forward"),
    ("NavigateToTarget", "ArmUnfolding", "forward"),
    ("ArmUnfolding", "ScanWall", "phase1"),
    ("ScanWall", "SensorDataProcessing", "forward"),
    ("ScanWall", "ScanWall", "loop"),
    ("SensorDataProcessing", "ArmFolding", "forward"),
    ("SensorDataProcessing", "SendDataToPokeye", "forward"),
    ("SendDataToPokeye", "ArmFolding", "forward"),
    ("ArmFolding", "WallTargetSelection", "loop"),
    # ("ArmUnfolding", "ExhaustiveScan", "phase2"),
    # ("ArmFolding", "AreasOfInterest", "phase1_complete"),
    # ("AreasOfInterest", "WallDiscretization", "forward"),
    # ("WallDiscretization", "BasePlacement", "forward"),
    # ("BasePlacement", "WallTargetSelection", "loop"),
    # ("ArmFolding", "BasePlacement", "recovery"),
    # ("ExhaustiveScan", "ArmFolding", "forward"),
    ("ArmUnfolding", "ScanFloor", "phase2"),
    ("ScanFloor", "SensorDataProcessing", "phase2"),
    ("ArmUnfolding", "ScanCeiling", "phase3"),
    ("ScanCeiling", "SensorDataProcessing", "phase3"),
    ("ArmFolding", "HomePosition", "scan_complete"),
    ("HomePosition", "Finished", "forward"),
]

_GROUPS = {
    "Initialization": "phase_1",
    "CreateMap": "phase_1",
    "ObjectID": "phase_1",
    "WallLinesComputation": "phase_1",
    "GeometryReconstruction": "phase_1",
    "ComputeWallPoints": "phase_1",
    "WallTargetSelection": "shared",
    "NavigateToTarget": "shared",
    "ArmUnfolding": "shared",
    "ScanWall": "phase_1",
    "ScanFloor": "phase_2",
    "ScanCeiling": "phase_3",
    "SensorDataProcessing": "shared",
    "SendDataToPokeye": "shared",
    "ArmFolding": "shared",
    # "AreasOfInterest": "phase_2",
    # "WallDiscretization": "phase_2",
    # "BasePlacement": "phase_2",
    # "ExhaustiveScan": "phase_2",
    "HomePosition": "phase_3",
    "Finished": "terminal",
    "Error": "terminal",
}


def make_json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(key): make_json_safe(val) for key, val in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [make_json_safe(item) for item in value]

    for attrs in (("x", "y", "z"), ("x", "y"), ("sec", "nanosec")):
        if all(hasattr(value, attr) for attr in attrs):
            return {attr: make_json_safe(getattr(value, attr)) for attr in attrs}

    return str(value)


def build_fsm_graph_payload(states: Iterable[str]) -> Dict[str, List[Dict[str, Any]]]:
    nodes = []
    seen = set()
    for index, state in enumerate(states):
        # Unknown states stack in their own column left of the horseshoe.
        x, y = _NODE_POSITIONS.get(state, (-360, index * 150))
        nodes.append(
            {
                "id": state,
                "label": state,
                "x": x,
                "y": y,
                "group": _GROUPS.get(state, "shared"),
            }
        )
        seen.add(state)

    edges = []
    for from_state, to_state, kind in _DEFAULT_EDGES:
        if from_state in seen and to_state in seen:
            edges.append({"from": from_state, "to": to_state, "kind": kind})

    return {"states": nodes, "edges": edges}
