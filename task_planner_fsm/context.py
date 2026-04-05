from typing import Any, Callable, Dict, List, Optional, Tuple, TypedDict


class FsmContext(TypedDict, total=False):
    node: Any
    start: bool
    map_ready: bool
    error_triggered: bool
    last_state: Optional[str]
    scan_phase: int
    execution_status: bool
    planner_goal_failed: bool
    sim: bool
    publish_fsm_current: Callable[[str], None]
    publish_fsm_transition: Callable[[str, str, str], None]
    tf_buffer: Any
    mapping_cmd: List[str]
    _procs: Dict[str, Any]
    fsm_initial_state: str
    is_initial_entry: bool
    home_position: Any
    home_orientation: Any
    base_position: Any
    base_orientation: Any
    odom_received: bool
    walls_data: List[Dict[str, Tuple]]
    target_scan_wall: Tuple
    target_scan_point: Tuple
    selected_base: Tuple[float, float]
    selected_base_idx: int
    wall_discretization_results: Dict[str, List]
    optimal_base_results: Dict[int, Tuple[float, float]]
    completed_base_indices: List[int]
    recompute_base_placement: bool
    base_recompute_retry_counts: Dict[int, int]
    scan_done: bool
    exhaustive_scan_done: bool
    panels_left: int
    column_current_height: float
