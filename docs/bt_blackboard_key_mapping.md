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
