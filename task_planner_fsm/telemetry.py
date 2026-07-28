from __future__ import annotations

from typing import Any, Dict, Iterable, List


# Horseshoe layout for the linear pokeye drill workflow (mirrors the reference
# state diagram). Nodes are 220x92 in the RViz panel; columns are spaced 320px.
# The perception/navigation chain runs LEFT->RIGHT across the top row, turns the
# corner down the right-hand column, and the drill sequence runs RIGHT->LEFT back
# along the bottom row, so states that follow each other are physically adjacent
# and the forward chain never crosses itself.
#
# Note: the reference diagram draws "Arm folding" (ManipulatorFolding) twice --
# once in the top row (TargetSelection -> ManipulatorFolding -> BasePlacement)
# and once bottom-left (StoringToDatabase -> ManipulatorFolding -> HomePosition).
# The panel renders one node per state, so it lives in the top row here; the
# once-per-run "fold and go home" edges are therefore the only longer ones.
#
# Error is an isolated sink (no edges are routed to it) that lights up when
# active; it floats on the left, clear of both rows. Coordinates are the
# top-left corner of each node.
_NODE_POSITIONS = {
    # Top row (L->R): perception, navigation & base placement
    "Initialization": (40, 200),
    "ReceiveNav2Map": (360, 200),
    "GetSemanticMap": (680, 200),
    "WaitForData": (1000, 200),
    "TargetSelection": (1320, 200),
    "ManipulatorFolding": (1640, 200),
    "BasePlacementComputation": (1960, 200),
    "NavigateToTarget": (2280, 200),
    "ManipulatorReachability": (2600, 200),
    "NearbyPointSelection": (2920, 40),  # raised above the row
    "ManipulatorUnfolding": (3240, 200),
    # Right column (top -> bottom): approach & start drilling
    "DrillApproach": (3240, 480),
    "SuctionDrillStart": (3240, 760),
    # Bottom row (R->L): drilling operation & return
    "Drilling": (2920, 760),
    "TakeOutDrill": (2600, 760),
    "SuctionDrillStop": (2280, 760),
    "DrillRetract": (1960, 760),
    "SampleScanning": (1640, 760),
    "StoringToDatabase": (1320, 760),
    "HomePosition": (1000, 760),
    "Finished": (680, 760),
    # Isolated failure sink
    "Error": (40, 480),
}

# Transitions taken from each state's NEXT_STATE_OPTIONS / check_transition.
# ``kind`` is informational only (the panel does not style edges by kind).
# ``-> Error`` edges are intentionally omitted to keep the graph readable; every
# non-terminal state can fail to Error, which is shown as a standalone node.
_DEFAULT_EDGES = [
    ("Initialization", "ReceiveNav2Map", "forward"),
    ("ReceiveNav2Map", "GetSemanticMap", "forward"),
    ("GetSemanticMap", "WaitForData", "forward"),
    ("WaitForData", "TargetSelection", "forward"),
    # oliwall finished without asking for more holes: fold and go home.
    ("WaitForData", "ManipulatorFolding", "branch"),
    ("TargetSelection", "DrillApproach", "branch"),
    ("TargetSelection", "ManipulatorFolding", "branch"),
    ("ManipulatorFolding", "BasePlacementComputation", "forward"),
    ("ManipulatorFolding", "HomePosition", "branch"),
    ("BasePlacementComputation", "NavigateToTarget", "forward"),
    ("NavigateToTarget", "ManipulatorReachability", "forward"),
    ("ManipulatorReachability", "ManipulatorUnfolding", "branch"),
    ("ManipulatorReachability", "NearbyPointSelection", "branch"),
    ("ManipulatorReachability", "BasePlacementComputation", "loop"),
    ("NearbyPointSelection", "ManipulatorUnfolding", "branch"),
    ("NearbyPointSelection", "TargetSelection", "loop"),
    ("ManipulatorUnfolding", "DrillApproach", "forward"),
    ("DrillApproach", "SuctionDrillStart", "forward"),
    ("SuctionDrillStart", "Drilling", "forward"),
    ("Drilling", "TakeOutDrill", "forward"),
    ("Drilling", "TargetSelection", "loop"),
    ("TakeOutDrill", "SuctionDrillStop", "forward"),
    ("SuctionDrillStop", "DrillRetract", "forward"),
    ("DrillRetract", "SampleScanning", "forward"),
    ("SampleScanning", "StoringToDatabase", "forward"),
    ("SampleScanning", "DrillApproach", "loop"),
    ("StoringToDatabase", "TargetSelection", "loop"),
    ("StoringToDatabase", "WaitForData", "loop"),
    ("StoringToDatabase", "ManipulatorFolding", "branch"),
    ("HomePosition", "Finished", "forward"),
]

# Only phase_1 (blue), phase_2 (green) and terminal (red) get distinct colors in
# the panel; anything else ("shared") renders as the default beige.
_GROUPS = {
    # Perception / setup
    "Initialization": "phase_1",
    "ReceiveNav2Map": "phase_1",
    "GetSemanticMap": "phase_1",
    "WaitForData": "phase_1",
    # Navigation & base placement
    "TargetSelection": "shared",
    "ManipulatorFolding": "shared",
    "BasePlacementComputation": "shared",
    "NavigateToTarget": "shared",
    "ManipulatorReachability": "shared",
    "NearbyPointSelection": "shared",
    "HomePosition": "shared",
    # Drilling operation
    "ManipulatorUnfolding": "phase_2",
    "DrillApproach": "phase_2",
    "SuctionDrillStart": "phase_2",
    "Drilling": "phase_2",
    "TakeOutDrill": "phase_2",
    "SuctionDrillStop": "phase_2",
    "DrillRetract": "phase_2",
    "SampleScanning": "phase_2",
    "StoringToDatabase": "phase_2",
    # Terminal
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
