from __future__ import annotations

from typing import Any, Dict, Iterable, List


_NODE_POSITIONS = {
    "Initialization": (40, 40),
    "CreateMap": (360, 40),
    "ObjectID": (680, 40),
    "WallLinesComputation": (1000, 40),
    "GeometryReconstruction": (1320, 40),
    "ComputeWallPoints": (1640, 40),
    "WallTargetSelection": (360, 240),
    "NavigateToTarget": (680, 240),
    "ArmUnfolding": (1000, 240),
    "ScanWall": (1320, 240),
    "ScanFloor": (1320, 440),
    "ScanCeiling": (1320, 640),
    "SensorDataProcessing": (1640, 240),
    "SendDataToPokeye": (1960, 440),
    "ArmFolding": (2280, 240),
    # "AreasOfInterest": (1320, 440),
    # "WallDiscretization": (1640, 440),
    # "BasePlacement": (1960, 440),
    # "ExhaustiveScan": (1320, 640),
    "HomePosition": (2280, 440),
    "Finished": (2280, 640),
    "Error": (2280, 40),
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
        x, y = _NODE_POSITIONS.get(state, (index * 180, 0))
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
