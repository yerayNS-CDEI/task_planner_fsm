# BT Blackboard Key Mapping (Initial)

This document maps current FSM `ctx` keys to BT blackboard-style keys for incremental migration.

## Runtime Control
- `start` -> `runtime.start_requested`
- `error_triggered` -> `runtime.error_triggered`
- `fsm_initial_state` -> `runtime.initial_state`
- `last_state` -> `runtime.last_state`
- `scan_phase` -> `runtime.scan_phase`

## Navigation + Execution
- `execution_status` -> `nav.execution_status`
- `planner_goal_failed` -> `nav.goal_failed`
- `nav_client` -> `nav.client`

## Mapping + World State
- `map_ready` -> `map.ready`
- `walls_data` -> `world.walls.data`
- `walls_left` -> `world.walls.remaining`
- `target_scan_wall` -> `world.scan.target_wall`
- `target_scan_point` -> `world.scan.target_point`

## Base Placement + Scan Progress
- `optimal_base_results` -> `scan.base.optimal_results`
- `base_to_panel_indices` -> `scan.base.panel_index_map`
- `selected_base_idx` -> `scan.base.selected_index`
- `selected_base` -> `scan.base.selected_position`
- `completed_base_indices` -> `scan.base.completed_indices`
- `panels_left` -> `scan.panels.remaining`
- `scan_done` -> `scan.phase1.done`
- `exhaustive_scan_done` -> `scan.phase2.done`

## Robot Pose + TF
- `base_position` -> `robot.base.position`
- `base_orientation` -> `robot.base.orientation`
- `home_position` -> `robot.home.position`
- `home_orientation` -> `robot.home.orientation`
- `odom_received` -> `robot.odom.received`
- `tf_buffer` -> `robot.tf.buffer`

## Process Management
- `_procs` -> `runtime.processes.managed`
- `mapping_cmd` -> `runtime.processes.mapping_command`

## Notes
- Keep using current keys in FSM states until each state is migrated.
- New BT nodes should prefer blackboard naming, while adapters keep compatibility.

## Ownership + Lifecycle Contract (BT v1)
This contract is intentionally strict for migration parity and allows later refactors after BT stabilization.

### Execution control
- `runtime.start_requested`
  - Owner: runtime gateway (`/start_flag` subscription in runtime node)
  - Consumers: BT root guard/runtime orchestrator
  - Lifecycle: persistent for full run; toggled only by explicit start signal handling.
- `runtime.error_triggered`
  - Owner: BT action leaves and runtime orchestrator
  - Consumers: recovery subtree and terminal failure branch
  - Lifecycle: set on operation failures; reset only at controlled action-entry boundaries.
- `runtime.scan_phase`
  - Owner: runtime bootstrap
  - Consumers: phase guard conditions
  - Lifecycle: set once at startup; immutable in BT v1.
- `runtime.finished`
  - Owner: terminal success leaf
  - Consumers: shutdown/telemetry
  - Lifecycle: write-once true when mission completes.

### Localization + robot state
- `robot.base.position`, `robot.base.orientation`
  - Owner: odometry subscriber
  - Consumers: navigation, target selection, homing
  - Lifecycle: continuously refreshed.
- `robot.home.position`, `robot.home.orientation`
  - Owner: initialization leaf
  - Consumers: homing leaf
  - Lifecycle: set once after initialization; reused through runtime.
- `robot.column_current_height`
  - Owner: joint-state subscriber
  - Consumers: exhaustive scan leaf
  - Lifecycle: continuously refreshed while joint data is available.

### Mapping + perception artifacts
- `world.walls.data`
  - Owner: map/perception preparation leaves
  - Consumers: target selection and phase transition leaves
  - Lifecycle: set after preprocessing and replaced only on explicit recomputation.
- `world.aoi.data`
  - Owner: areas-of-interest leaf
  - Consumers: wall discretization
  - Lifecycle: set at phase change; retained for phase 2.
- `world.wall_discretization.results`
  - Owner: wall discretization leaf
  - Consumers: base placement + exhaustive scan
  - Lifecycle: set once per discretization run; replace on recompute request.

### Planning outputs
- `scan.target.wall`, `scan.target.point`
  - Owner: wall-target selection leaf
  - Consumers: navigation + scan leaves
  - Lifecycle: overwritten each loop iteration.
- `scan.base.selected`, `scan.base.selected_index`
  - Owner: base selection logic (phase-dependent)
  - Consumers: navigation + exhaustive scan
  - Lifecycle: updated per iteration; cleared only when phase completes.
- `scan.base.optimal_results`, `scan.base.panel_index_map`
  - Owner: base placement leaf
  - Consumers: target selection + exhaustive scan
  - Lifecycle: replace as a pair whenever base placement recomputes.

### Loop progress
- `world.walls.remaining`
  - Owner: scan/selection leaves
  - Consumers: phase-1 loop guard
  - Lifecycle: decrement to zero during phase 1.
- `scan.panels.remaining`
  - Owner: exhaustive scan leaf
  - Consumers: phase-2 loop guard
  - Lifecycle: decrement to zero during phase 2.
- `scan.base.completed_indices`
  - Owner: exhaustive scan leaf
  - Consumers: selection + progress guards
  - Lifecycle: append-only for one mission run.
- `scan.phase1.done`, `scan.phase2.done`
  - Owner: scan leaves
  - Consumers: phase transition/termination guards
  - Lifecycle: set true once per phase completion.
