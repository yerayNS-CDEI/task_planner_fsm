from dataclasses import dataclass
from typing import Any, Dict, Optional


class BTBlackboardAdapter:
    """Adapter over FSM context to support BT-style key access incrementally."""

    def __init__(self, ctx: Dict[str, Any]):
        self._ctx = ctx

    def get(self, key: str, default: Any = None) -> Any:
        return self._ctx.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._ctx[key] = value

    def has(self, key: str) -> bool:
        return key in self._ctx

    def remove(self, key: str) -> None:
        self._ctx.pop(key, None)

    def get_bool(self, key: str, default: bool = False) -> bool:
        value = self._ctx.get(key, default)
        return bool(value)

    def get_int(self, key: str, default: int = 0) -> int:
        value = self._ctx.get(key, default)
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def get_float(self, key: str, default: float = 0.0) -> float:
        value = self._ctx.get(key, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def get_optional(self, key: str) -> Optional[Any]:
        return self._ctx.get(key)

    def raw(self) -> Dict[str, Any]:
        return self._ctx


@dataclass(frozen=True)
class BlackboardKeyContract:
    key: str
    owner: str
    consumers: str
    lifecycle: str


BLACKBOARD_KEY_CONTRACTS = [
    BlackboardKeyContract(
        key="runtime.start_requested",
        owner="runtime gateway (/start_flag)",
        consumers="runtime orchestrator",
        lifecycle="Updated by start topic; persistent for full run.",
    ),
    BlackboardKeyContract(
        key="runtime.error_triggered",
        owner="leaf actions / runtime orchestrator",
        consumers="recovery subtree, terminal selector",
        lifecycle="Set on failures, reset at controlled action boundaries.",
    ),
    BlackboardKeyContract(
        key="runtime.scan_phase",
        owner="bootstrap/runtime init",
        consumers="phase guards and loop selectors",
        lifecycle="Set once at startup for BT v1 and treated as immutable.",
    ),
    BlackboardKeyContract(
        key="runtime.finished",
        owner="terminal success leaf",
        consumers="telemetry and shutdown handler",
        lifecycle="Set true once on success and never cleared in-run.",
    ),
    BlackboardKeyContract(
        key="robot.base.position",
        owner="/rtabmap/odom subscriber",
        consumers="navigation and target selection leaves",
        lifecycle="Continuously refreshed from odometry.",
    ),
    BlackboardKeyContract(
        key="robot.home.position",
        owner="Initialization leaf",
        consumers="HomePosition leaf",
        lifecycle="Set during initialization and reused until shutdown.",
    ),
    BlackboardKeyContract(
        key="robot.home.orientation",
        owner="Initialization leaf",
        consumers="HomePosition leaf",
        lifecycle="Set during initialization and reused until shutdown.",
    ),
    BlackboardKeyContract(
        key="robot.column_current_height",
        owner="/joint_states subscriber",
        consumers="ExhaustiveScan leaf",
        lifecycle="Continuously refreshed while joint data is available.",
    ),
    BlackboardKeyContract(
        key="world.walls.data",
        owner="ComputeWallPoints leaf",
        consumers="selection, discretization and bootstrap helpers",
        lifecycle="Set after map/perception stage; replaced only on recomputation.",
    ),
    BlackboardKeyContract(
        key="world.aoi.data",
        owner="AreasOfInterest leaf",
        consumers="WallDiscretization leaf",
        lifecycle="Set after AOI computation and retained through phase 2.",
    ),
    BlackboardKeyContract(
        key="world.wall_discretization.results",
        owner="WallDiscretization leaf",
        consumers="BasePlacement and ExhaustiveScan leaves",
        lifecycle="Set in phase transition; replaced when recomputation is requested.",
    ),
    BlackboardKeyContract(
        key="scan.target.wall",
        owner="WallTargetSelection leaf",
        consumers="NavigateToTarget and scan leaves",
        lifecycle="Updated each loop iteration before navigation.",
    ),
    BlackboardKeyContract(
        key="scan.target.point",
        owner="WallTargetSelection leaf",
        consumers="NavigateToTarget and scan leaves",
        lifecycle="Updated each loop iteration before navigation.",
    ),
    BlackboardKeyContract(
        key="scan.base.selected",
        owner="BasePlacement/WallTargetSelection leaf",
        consumers="NavigateToTarget and ExhaustiveScan leaves",
        lifecycle="Updated when selecting next base; cleared only when phase is complete.",
    ),
    BlackboardKeyContract(
        key="scan.base.selected_index",
        owner="BasePlacement/WallTargetSelection leaf",
        consumers="ExhaustiveScan and progress tracking",
        lifecycle="Updated with selected base and retained for completion accounting.",
    ),
    BlackboardKeyContract(
        key="scan.base.optimal_results",
        owner="BasePlacement leaf",
        consumers="WallTargetSelection and ExhaustiveScan leaves",
        lifecycle="Set once per discretization run; replaced on recompute.",
    ),
    BlackboardKeyContract(
        key="scan.base.panel_index_map",
        owner="BasePlacement leaf",
        consumers="ExhaustiveScan leaf",
        lifecycle="Set alongside optimal base results; replaced on recompute.",
    ),
    BlackboardKeyContract(
        key="world.walls.remaining",
        owner="ScanWall/WallTargetSelection leaves",
        consumers="phase-1 loop guard",
        lifecycle="Decremented per wall scan until zero.",
    ),
    BlackboardKeyContract(
        key="scan.panels.remaining",
        owner="ExhaustiveScan leaf",
        consumers="phase-2 loop guard",
        lifecycle="Decremented as base indices complete until zero.",
    ),
    BlackboardKeyContract(
        key="scan.base.completed_indices",
        owner="ExhaustiveScan leaf",
        consumers="WallTargetSelection and loop guards",
        lifecycle="Appended when a base completes; retained for whole run.",
    ),
    BlackboardKeyContract(
        key="scan.phase1.done",
        owner="ScanWall leaf",
        consumers="phase switch guard",
        lifecycle="Set true once phase-1 walls are consumed.",
    ),
    BlackboardKeyContract(
        key="scan.phase2.done",
        owner="ExhaustiveScan leaf",
        consumers="terminal sequence to Home/Finished",
        lifecycle="Set true when exhaustive scan loop completes.",
    ),
]
