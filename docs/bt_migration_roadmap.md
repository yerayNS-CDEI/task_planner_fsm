# Behavior Tree Migration Roadmap (Incremental)

## Goal
Migrate the current ROS2 FSM implementation to an incremental Behavior Tree (BT) architecture with minimal behavior regression risk.

## Current Constraints
- The FSM has a large bootstrap path in `task_planner_fsm/fsm_node.py`.
- Runtime state is heavily shared through `ctx`.
- Some states manage external processes and recovery behavior.
- Interactive bootstrap prompts exist for partial-state runs.

## Milestone Plan

### Milestone 1 — Planning + Code Cleanliness Baseline
Scope:
- Define the migration sequence and safety checks.
- Identify FSM responsibilities to preserve during migration.
- Prepare branch/PR workflow for incremental delivery.

Deliverables:
- This roadmap document.
- Validation checklist for reviewers.

Acceptance checks:
- Migration phases are explicit and ordered.
- Dependencies and local setup are documented.
- Reviewers can execute verification steps.

---

### Milestone 2 — FSM Clean Refactor (No Behavior Change)
Scope:
- Extract static FSM/bootstrap configuration from `fsm_node.py` into dedicated modules.
- Reduce high-density logic sections into smaller private methods.
- Keep runtime behavior identical.

Deliverables:
- Refactored organization for constants/bootstrap definitions.
- Smaller, easier-to-test method boundaries.

Acceptance checks:
- No transition-order or state-name changes.
- Existing launch/run behavior remains unchanged.
- Refactor is traceable and low-risk.

---

### Milestone 3 — BT-Compatible Abstractions
Scope:
- Introduce BT-ready data abstractions (blackboard-like access layer).
- Add interfaces/adapters that decouple state logic from raw `ctx`.
- Keep FSM operational while enabling hybrid migration.

Deliverables:
- New BT support module(s) used as opt-in scaffolding.
- Mapping notes from FSM context keys to BT blackboard keys.

Acceptance checks:
- FSM still runs unchanged.
- Adapter APIs are stable and documented.
- New abstractions are ready for next BT node migration steps.

---

### Milestone 4 — Phase-1 BT Subtree (Pilot)
Scope:
- Migrate a low-risk path (initialization + map/object preprocessing) to BT nodes.
- Keep remaining flow in FSM.

Deliverables:
- First BT subtree integrated into runtime.
- Deterministic handoff between BT and remaining FSM segments.

Acceptance checks:
- Pilot subtree can be executed end-to-end.
- Fallback/error handling remains equivalent.

---

### Milestone 5 — Navigation/Manipulation BT Migration
Scope:
- Migrate scan-target selection and navigation/manipulation segments.
- Standardize retries/timeouts/recovery as BT decorators/subtrees.

Deliverables:
- BT implementations for operational states currently in FSM.
- Recovery policies represented explicitly in tree structure.

Acceptance checks:
- Retry/recovery behavior matches or improves current FSM behavior.
- Logs/telemetry remain observable.

---

### Milestone 6 — Full BT Runtime + FSM Decommission
Scope:
- Complete migration of remaining states (`ExhaustiveScan`, cleanup, finish/error flows).
- Remove obsolete FSM-only orchestration paths.

Deliverables:
- BT as primary runtime orchestrator.
- FSM retained only if needed for compatibility layer (or removed).

Acceptance checks:
- End-to-end runs complete with BT runtime only.
- Obsolete code paths removed or clearly deprecated.

## Dependencies and Tooling
- ROS2 environment (same distro as current project target).
- Existing Python package dependencies from this repository.
- For BT runtime (planned): `BehaviorTree.CPP`-based stack and visualization tools.

## Installation Notes (Current Repo + Migration Work)
1. Source your ROS2 environment.
2. Build package(s) in workspace:
   - `colcon build --packages-select task_planner_fsm`
3. Source workspace install:
   - `source install/setup.bash`
4. Run FSM baseline:
   - `ros2 run task_planner_fsm fsm_node --sim true`

## Reviewer Checklist Per Milestone PR
- Confirm scope is limited to milestone target.
- Verify no unrelated behavior changes.
- Validate run instructions still work.
- Check that migration notes remain consistent with implemented code.

## Visualization Note
When BT runtime is introduced, plan live graphical monitoring using Groot/Groot2-compatible telemetry.
