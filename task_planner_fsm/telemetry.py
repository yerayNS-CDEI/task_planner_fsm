from __future__ import annotations

from typing import Any, Dict, Iterable, List


# Horseshoe layout for the linear pokeye drill workflow (mirrors the reference
# state diagram), turned on its side so the graph is TALL rather than WIDE and
# fits a docked RViz side panel. Nodes are 220x92 in the panel; the two columns
# are spaced 400px (180px gutter for the return edges) and rows 150px (58px of
# vertical breathing room -- the edge router treats nodes as obstacles inflated
# by 8px per side, so rows must stay at least ~110px apart).
#
# The perception/navigation chain runs TOP->BOTTOM down the left column, turns
# the corner along the bottom, and the drill sequence runs BOTTOM->TOP back up
# the right column, so states that follow each other are physically adjacent and
# the forward chain never crosses itself.
#
# Note: the reference diagram draws "Arm folding" (ManipulatorFolding) twice --
# once in the forward chain (TargetSelection -> ManipulatorFolding -> BasePlacement)
# and once on the return path (StoringToDatabase -> ManipulatorFolding -> HomePosition).
# The panel renders one node per state, so it lives in the left column here; the
# once-per-run "fold and go home" edges are therefore the only longer ones.
#
# Error is an isolated sink (no edges are routed to it) that lights up when
# active; it floats at the top right, clear of both columns. Coordinates are the
# top-left corner of each node.
_NODE_POSITIONS = {
    # Left column (top -> bottom): perception, navigation & base placement
    "Initialization": (480, 40),
    "ReceiveNav2Map": (480, 190),
    "GetSemanticMap": (480, 340),
    "WaitForData": (480, 490),
    "TargetSelection": (480, 640),
    "ManipulatorFolding": (480, 790),
    "BasePlacementComputation": (480, 940),
    "NavigateToTarget": (480, 1090),
    "ManipulatorReachability": (480, 1240),
    "NearbyPointSelection": (40, 1390),  # pushed left of the column
    "ManipulatorUnfolding": (480, 1390),
    # Bottom of the horseshoe (L->R): approach & start drilling
    "DrillApproach": (480, 1540),
    "SuctionDrillStart": (880, 1540),
    # Right column (bottom -> top): drilling operation & return
    "Drilling": (880, 1390),
    "TakeOutDrill": (880, 1240),
    "SuctionDrillStop": (880, 1090),
    "DrillRetract": (880, 940),
    "SampleScanning": (880, 790),
    "StoringToDatabase": (880, 640),
    "HomePosition": (880, 490),
    "Finished": (880, 340),
    # Isolated failure sink
    "Error": (880, 40),
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
