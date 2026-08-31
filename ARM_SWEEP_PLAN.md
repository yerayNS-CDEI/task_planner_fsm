# Arm-Driven Wall Sweep — Design & Implementation Plan

> **Status:** implemented. Written as a plan; kept as the reference for the
> reasoning behind the code, which cites its section numbers throughout.
> **Branch:** `base_placement_arm_scan`, created from `main` (at `b7bb9f4`).
> **Written:** 2026-07-31.
> **Updated:** 2026-08-24 — revised arm-sweep control, validation, watchdog, diagnostics, sensor-based orientation correction, and partition-centred base placement.
>
> This document is a self-contained handoff. It assumes no prior conversation
> context: everything needed to pick the work up in a fresh session is here,
> including the verified findings behind each decision and the false starts that
> were ruled out.

---

## 1. Context

**Robot (oliwall):** diff-drive mobile base + rotating turret + vertical lifting
column + **UR10e** (1.3 m reach) carrying a sensor plate with a GPR wheel
(GP Proceq8800) and six distance sensors (3 ultrasonic + 3 ToF).

**Software:** ROS 2 Humble. UR driver installed from apt:

| package | version |
|---|---|
| `ros-humble-ur-robot-driver` | 2.13.2 |
| `ros-humble-ur-controllers` | 2.13.2 |
| `ros-humble-ur-client-library` | 2.13.0 |

**Relevant packages in `~/ros2_ws/src`:**

- `task_planner_fsm` — the FSM. **The oliwall wall-scan FSM lives on the `main`
  branch.** (The `pokeye` branch is a different robot; its `states/legacy/`
  directory holds older copies of these same files. Do not confuse them.)
- `arm_control` — arm planning/IK/control, sensor nodes, controller configs.
- `navi-wall` — nav2 stack, costmaps, wall detection.

---

## 2. The problem

The current wall scan works like this: the arm is pressed against the wall with
the UR `force_mode_controller`, and **the base then drives along the wall with
nav2** to produce the lateral scan motion.

This fails on the real robot:

1. **The base outruns the arm.** Force mode's compliant-axis response cannot
   keep the plate on the wall at even the slowest nav2 velocity, so the sensors
   separate from the surface mid-sweep.
2. **Nav2 routes away from the wall.** The global planner keeps the whole path
   in free space, so it arcs away from walls and small obstacles, and the arm
   over-stretches trying to stay in contact.

### The new approach

Move the base to a fixed scan pose and **perform the lateral sweep with the arm
alone.** Each scan segment is split into partitions of at most a parametrised
length; for each partition the base moves to a pose centred on that partition,
at a configurable wall standoff, with `turret_link +X` pointing into the wall.
The base then remains stationary while the arm approaches the wall, Force Mode
engages, the arm sweeps laterally to the far end, then retracts.

**Note on the costmap issue:** goal *placement* was never the problem — a base
goal can be placed inside the inflation layer. The problem was path *routing*
during navigation. Under the new design that is largely moot: nothing is being
scanned while the base moves, so the path shape between partitions does not
matter. Only the goal checker's tolerances need to accept the final fixed scan
pose. In arm-sweep mode there is no separate chassis-parking correction after
Nav2 reaches that pose.

---

## 3. Verified findings

Everything in this section was checked against the installed packages or the
official documentation. Do not re-derive it from memory — it contains one
result that contradicts the obvious ros2_control-level reasoning.

### 3.1 Controller compatibility (IMPORTANT — non-obvious)

For the **installed Humble driver 2.13.2**, the supported combination for motion
under Force Mode is:

**`force_mode_controller` + `passthrough_trajectory_controller`.**

Not `scaled_joint_trajectory_controller`, not `forward_velocity_controller`,
and not `forward_position_controller` on this deployed version.

At the interface-claim level the combination can look legal because
`ForceModeController` claims only the `force_mode/*` command interfaces
(`task_frame_*`, `selection_vector_*`, `wrench_*`, `type`, `limits_*`,
`damping`, `gain_scaling`, `disable_cmd`, `async_success`) and no joint command
interface. However, the UR hardware interface performs an additional
UR-specific compatibility check during controller switching. In 2.13.2 its
`mode_compatibility_` table explicitly rejects position/velocity + Force Mode
and accepts passthrough + Force Mode.

So the effective chain is:

```
ros2_control resource claiming:
    no direct joint/force interface conflict
            |
            v
URPositionHardwareInterface::prepare_command_mode_switch():
    applies UR-specific mode_compatibility_
            |
            +-- velocity + force mode   -> reject
            +-- passthrough + force mode -> accept
```

The table contents were read from the 2.13.2 source and are **verified**. Entries
set `true`:

```text
FORCE_MODE_GPIO   + PASSTHROUGH_GPIO  = true
PASSTHROUGH_GPIO  + FORCE_MODE_GPIO   = true
PASSTHROUGH_GPIO  + TOOL_CONTACT_GPIO = true
TOOL_CONTACT_GPIO + {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT, PASSTHROUGH_GPIO} = true
{HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT} + TOOL_CONTACT_GPIO = true
```

**Every other combination is `false`**, and `prepare_command_mode_switch` returns
failure on a `false` entry. So position + Force Mode and velocity + Force Mode are
rejected at controller-switch time. This is an *enforced runtime check*, not only
a documentation statement. The member itself is present in the installed headers
at `/opt/ros/humble/include/ur_robot_driver/hardware_interface.hpp:334`.

`passthrough_trajectory_controller` hands the whole waypoint list to the
robot-side trajectory executor, which is the supported way to compose nominal
motion with the robot's native `force_mode()` on this version.

**Version note:** newer/rolling UR documentation currently contains conflicting
statements about Force Mode compatibility with streaming position/velocity
controllers. Some usage pages say those combinations are supported, while the
current driver source still contains compatibility entries rejecting
`HW_IF_VELOCITY + FORCE_MODE`. Do not infer support from the ROS 2 distribution
name alone. For this project, the installed 2.13.2 source is authoritative and
**PTC + Force Mode remains the required architecture**. Re-evaluate this only
when the deployed UR driver is actually upgraded and tested.

Sources for the deployed behavior:
- <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.13.2/ur_robot_driver/src/hardware_interface.cpp>
- <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.13.2/ur_controllers/doc/index.rst>

**Consequence:** streaming velocity control for the lateral sweep is ruled out
with the current installation. Under contact, use **PTC + Force Mode**.

### 3.2 The trajectory pipeline already routes through PTC

Verified end to end:

```
planner_node.py:265        publishes JointTrajectory -> 'planned_trajectory'
                                    |
publisher_joint_trajectory_planned.py:60   subscribes 'planned_trajectory'
                                    |
                           FollowJointTrajectory action
                                    |
                    passthrough_trajectory_controller  (real robot)
```

`arm_control/control/publisher_joint_trajectory_planned.py` takes the target
controller as a `controller_name` parameter. On the real robot it is set to
`passthrough_trajectory_controller`.

The arm sweep needs **no new controllers** — PTC is already the real-robot
execution path.

> ### ⚠ But the sweep must NOT go through `planned_trajectory`
>
> `publisher_joint_trajectory_planned.py` is not a transparent bridge. It
> **discards the velocities and re-times the entire trajectory**:
>
> ```python
> max_joint_speed  = 0.1   # rad/s — intentionally slow for smooth, safe motion
> min_segment_time = 0.9   # seconds — floor for very small moves
> ```
>
> It zeroes velocity at every waypoint and sizes each segment for a
> zero-velocity cubic. For a 0.8 m sweep at 3 cm spacing (~27 waypoints) that is
> **≥24 s with the TCP decelerating to a full stop 27 times** — the exact
> opposite of the constant-speed sweep §9.4 sets out to validate, and it would
> silently defeat the velocity/timing generation in §7.B.
>
> Its own comment explains why: *"the only reliable way to avoid
> PATH_TOLERANCE_VIOLATED in Gazebo"*. That is a JTC concern — PTC has no path
> tolerance (§3.4) — but it matters for the simulation path (§11).
>
> **`wall_sweep_executor` must own its own `FollowJointTrajectory` action
> client**, targeting the controller directly. That is also what makes the
> preempt/replan cycle in §4.2 possible at all (it needs the goal handle), and
> it removes the two-publisher hazard of §3.3 by construction.

### 3.3 `planned_trajectory` is a single-publisher topic in practice

`arm_control/sensors/wall_parallel_controller.py` publishes to
`planned_trajectory` when `output_mode: 'passthrough'` (the real-robot mode —
see its docstring at lines 47-58). Today it stays running *through* the sweep
and is only stopped at `scan_wall.py:1538`.

**Under the new design its command output must stop BEFORE the sweep starts.**
Two publishers driving one PTC is not survivable. The wall-alignment estimation
logic may still be reused in read-only form during the sweep, but it must not
publish `planned_trajectory`; see §4.2 and §7.B.

### 3.4 `passthrough_trajectory_controller` has no path tolerance

`path_tolerance_` is commented out at
`/opt/ros/humble/include/ur_controllers/passthrough_trajectory_controller.hpp:164`;
only goal tolerance and goal-time tolerance exist.

This is **good** — force-mode deviation during a sweep cannot trigger a
mid-trajectory path-tolerance abort. For the arm sweep, deliberately leave the
PTC `goal_tolerance` empty: the final joint configuration can legitimately
differ from the nominal endpoint along the compliant wall-normal axis. Define
scan success at the application layer from PTC completion + valid contact + no
watchdog abort + successful GPR acquisition, rather than exact final joint
error.

### 3.5 Analytic IK already exists

`arm_control/planner/planner_lib/closed_form_algorithm.py`:

```python
closed_form_algorithm(goal_matrix, q_current=None, type=0, return_all_solutions=False)
```

- `type=0` returns the single best solution as a `(6,)` vector.
- `_select_best_solution` (lines 268-294) picks the branch nearest `q_ref` and
  unwraps each joint to the closest equivalent angle, then clamps to ±2π.

**That is exactly the branch-continuity guarantee a Cartesian sweep needs.**
Chain `q_prev` waypoint-to-waypoint and the solution will not flip mid-sweep.

DH parameters are UR10e-specific (`_ur10e_dh_params()`, line 4).

### 3.6 `reachable_wall_segments` is height-independent

`task_planner_fsm/utils/costmap_utils.py:367` splits a scan line into the
sub-segments where a base cell exists within arm reach. It uses **only x and y**
— `z` is read from `p1` and carried through untouched (line 391).

So segmentation is already identical for every line height on a wall.
Computing it once and reusing it is not merely an optimization: it *guarantees*
identical partition boundaries at every height, which matters for stitching.

---

## 4. Design decisions

### 4.1 The sweep is not planned, it is computed

Do **not** use MoveIt or any search-based planner for the sweep. Two reasons:

1. The sensor plate is in contact with the wall, so the collision checker will
   report a collision and refuse to plan.
2. A search-based planner discretizes the *search space*. The sweep path is a
   straight line with a known nominal plate orientation; if the distance
   sensors later detect angular drift, only the **remaining** trajectory is
   recomputed with a corrected orientation (§4.2). There is still no search
   problem.

Instead: sample the line analytically, run analytic IK per sample, and emit a
`JointTrajectory` with positions, velocities, and timing derived from the
requested Cartesian sweep speed. Every Cartesian waypoint lies exactly on the
requested line, but the robot interpolates between **joint-space** waypoints, so
the TCP path between waypoints must be verified rather than assumed to be
sub-millimetre.

**Safety comes from validation, not from a collision checker.** Before sending a
trajectory, validate: non-NaN IK, joint limits, branch continuity
(`max|Δq|`), Jacobian conditioning / minimum singular value, required joint
velocities at the requested sweep speed, and FK-sampled TCP interpolation error
between waypoints. Force Mode's deviation limits remain an additional robot-side
containment mechanism.

### 4.2 Orientation during the sweep: fixed command + sensor-based correction

The passive RX/RY gimbal was tested and did not behave reliably, so it is **not
the initial approach**. Keep RX and RY non-compliant and let the PTC trajectory
command plate orientation. `scan_wall_gimbal_press` remains available as an
experimental switch but defaults to `false`.

During `press_prepare`, `wall_parallel_controller` actively aligns the plate as
it does today. Before the sweep, its **trajectory-command output** is stopped so
it cannot compete with `wall_sweep_executor` on `planned_trajectory`. Capture
the achieved aligned orientation and use it as the nominal orientation for the
initial sweep trajectory.

The six distance sensors should continue to provide wall-orientation feedback
during the sweep. The correction calculation must use the **same geometry and
math as `wall_parallel_controller`**, but without sending motion commands. The
preferred implementation is to extract that calculation into a reusable
read-only estimator (for example
`arm_control/sensors/wall_alignment_estimator.py`) and have both
`wall_parallel_controller` and `wall_sweep_executor` call the same code. If a
refactor is too invasive, an `estimate_only` / monitor mode in
`wall_parallel_controller` is acceptable, provided it cannot publish
`planned_trajectory` in that mode.

The estimator provides at least:

- current plate-to-wall angular error (roll/pitch or equivalent wall-normal
  error),
- corrected wall-parallel target orientation,
- current plate-to-wall distance / sensor consistency information.

While the PTC sweep is running, compare the estimated angular error against a
configurable threshold with hysteresis. If the error remains inside the margin,
do nothing. If it exceeds the threshold:

1. preempt/cancel the currently executing PTC goal (do not try to mutate an
   already transferred trajectory in place),
2. read the current joint state and `arm_base -> arm_tool0` TCP pose,
3. project the current TCP position onto the commanded partition direction to
   recover sweep progress,
4. compute the corrected wall-parallel orientation from the distance sensors,
5. generate and validate a new trajectory from the **current state to the
   original partition endpoint**, preserving the requested lateral speed,
6. blend from the current measured orientation to the corrected orientation
   over a short configurable lateral distance so the correction is not an
   instantaneous angular step,
7. send the replacement PTC goal without intentionally disabling Force Mode on Z.

> **Framing correction.** Force Mode is started by a service call to a
> *different controller* claiming *different interfaces*, and persists until
> `stop_force_mode`. Preempting a PTC goal therefore **cannot** stop it — that
> is not the risk. The open question is how the robot behaves when a trajectory
> is aborted mid-execution *while Force Mode is pressing*: the arm decelerates
> out of the trajectory while Force Mode keeps driving the compliant Z axis.
> Bench-verify that specific transition (deceleration profile, whether the press
> overshoots during the gap, and how cleanly the replacement trajectory picks
> up), not merely "is Force Mode still active".

PTC supports starting/preempting trajectories, but not in-place editing of the
trajectory already forwarded to the robot. Therefore orientation correction is
implemented as bounded replanning of the remaining sweep, not as a second
streaming controller.

Use a minimum replan interval / cooldown and hysteresis to avoid oscillating
between repeated PTC preemptions when the sensor error is near the threshold.
Every replan event must be recorded in the sweep diagnostics (§7.B).

The authoritative **lateral sweep direction** comes from the partition geometry
`normalize(seg_end - seg_start)`. The plate-derived direction
`normalize(cross(z_plate, up_in_arm_base))` is retained as a sanity check only;
it must agree with the partition tangent up to sign and a configured angular
tolerance.

### 4.3 Two steps, not one

Step 1 (partitioning + arm sweep) is the actual fix for the contact problem.
Step 2 (nesting line heights inside partitions) is pure cycle-time efficiency.

**Do them separately.** Step 1 is gated behind a flag and leaves the control
flow intact; step 2 restructures the state's control flow. Doing both at once
means debugging a new contact strategy and a restructured state machine in the
same run on real hardware.

Accept that step 1 alone makes cycle time *worse* than today (see §6 table).
You are testing whether contact holds, not how fast it runs.

---

## 5. Current implementation on `main`

`task_planner_fsm/states/scan_wall.py` (1961 lines) already implements a
per-segment phase machine. Most of what the new design needs is there.

### 5.1 Control flow today

```
run()  [line 1151]
  ├── Phase A: _run_pre_approach   [1056]  arm -> unfolded_fsm pose, column -> line height
  ├── Phase B: _run_scan           [1180]  the segment machine
  └── Phase C: _run_post_scan      [1869]  arm retract; column retract on last line
```

`_run_scan` per-segment phases:

```
transit_clear → transit_clear_wait → transit → transit_wait → park
  → sweep_setup → arm_approach → arm_approach_wait
  → press_prepare → press_settle → sweep_wait
```

**Line heights are the OUTER loop.** `check_transition` (line 1954) returns
`"ScanWall"` while `more_lines`, and `on_enter` resets `_seg_idx = 0` and
`_seg_phase = "transit_clear"`. Every line height re-runs the whole segment
machine, transits and parks included.

> The comment at line 1956 (`# same wall, next height (base stays put)`) is
> **misleading** — the base does not stay put. Serpentine means line *n+1*
> starts where line *n* ended, so the first transit gets skipped by
> `segment_transit_skip_tol`, but on a multi-segment wall the base still
> traverses every segment again for every height.

### 5.2 Existing machinery worth reusing

| what | where |
|---|---|
| reachability-based line splitting | `utils/costmap_utils.py:367` `reachable_wall_segments` |
| base goals at standoff / wall-parallel transits | `costmap_utils.py` `base_standoff_goal`, `wall_parallel_goal` |
| **arm owns the wall distance** (measured, not assumed) | `scan_wall.py:419-526` `_send_arm_z_goal`, `_send_arm_to_clearance` |
| plate-to-wall distance from 6 sensors | `scan_wall.py:359` `_plate_wall_distance` |
| FT-based contact gate | `scan_wall.py:641` `_wall_contact_ready` |
| chassis parking (square to wall) | `scan_wall.py:903` `_run_parking` |
| GPR LINE_SCAN start/stop | `scan_wall.py:755`, `:783` |
| column height for a line z | `scan_wall.py:821` `_column_target_for_line` |
| analytic UR10e IK | `arm_control/planner/planner_lib/closed_form_algorithm.py` |
| JointTrajectory → PTC bridge | `arm_control/control/publisher_joint_trajectory_planned.py` |

---

## 6. Why nesting matters (motivates step 2)

Partitioning multiplies the number of base stops. Worked example — 6 m wall,
3 reachable segments, 4 line heights, 0.8 m partitions (≈8 partitions):

| | base transits to final scan poses | column moves |
|---|---|---|
| today | 4 × 3 = 12 | 4 |
| step 1 only (lines outer) | 4 × 8 = **32** | 4 |
| step 1 + step 2 (lines nested) | **8** | 32 |

Trading repeated Nav2 transits to partition scan poses for column moves is a
straight win: the column is faster and more repeatable than repositioning the
base for every line height.

**Without step 2, partitioning makes cycle time noticeably worse than today.**

---

## 7. STEP 1 — Partitioning + arm sweep

### 7.A New util: `task_planner_fsm/utils/wall_partitioning.py`

```python
partition_segments(segments, max_len, overlap) -> [((sx,sy,z),(ex,ey,z)), ...]
```

Two-stage split:

1. `reachable_wall_segments` first (existing, obstacle-driven).
2. Then a pure geometric split of each surviving segment into **equal-length**
   partitions of ≤ `partition_max_length_m`, with `partition_overlap_m` between
   consecutive partitions.

Distribute evenly rather than leaving a short stub partition at the end.

Applied immediately after `reachable_wall_segments` at `scan_wall.py:1261`.

> **Why this integration point is cheap:** `self._segments` becomes the
> partition list, and the entire existing phase machine keeps working unchanged
> — each partition is just "a segment" to it.

#### Base placement and orientation for each partition

For the arm-driven sweep, the Nav2 goal is the **final scan pose**. There is no
subsequent chassis `park` correction. The base is placed from the partition
geometry as follows.

For partition endpoints `p_start` and `p_end` in the horizontal wall plane,
compute the partition centre and tangent:

```text
p_center = 0.5 * (p_start + p_end)
t_hat    = normalize(p_end - p_start)
```

From `t_hat`, obtain the two possible horizontal wall normals and select the one
that points from the robot/free-space side **into the wall**. Call it
`n_wall`. Reuse the existing wall/costmap side information to resolve the sign;
do not infer it from the arbitrary ordering of the segment endpoints.

The commanded scan-pose position is then:

```text
p_nav_base = p_center - partition_base_standoff_m * n_wall
```

where `p_nav_base` is the position of the base reference frame used by the Nav2
goal (normally the frame already used by `base_standoff_goal`; compensate with
TF if that helper targets a different reference point).
`partition_base_standoff_m` is a dedicated ctx parameter. It must be easy
to change because the best standoff depends on UR10e reach, the turret/column
geometry, plate clearance and IK quality, and will be tuned on the real robot.
Its initial value should be the currently tested standoff used by the existing
scan setup; do not silently couple it to `base_goal_max_offset` or to the plate
retraction parameters.

The orientation requirement is defined on **`turret_link`**, not assumed from a
generic mobile-base axis:

```text
unit_x(turret_link)  ||  n_wall
```

i.e. `turret_link +X` must point directly into the wall at the final scan pose.
The base/turret goal generation must account for the actual fixed/rotating TF
between the mobile-base frame and `turret_link`; do not assume their +X axes are
identical unless TF confirms it. The existing `base_standoff_goal(...)` may be
extended/wrapped to accept the partition centre, explicit standoff and this
orientation requirement.

This gives three separate geometric roles:

```text
p_center  -> where along the wall the robot is centred
n_wall    -> standoff direction and final turret-facing orientation
t_hat     -> authoritative arm sweep direction
```

Because the base remains stationary for the complete arm sweep, `_run_parking`
is not required in `sweep_use_arm=true` mode. Keep the existing parking code
only for the legacy/base-driven A/B path.

### 7.B New node: `arm_control/control/wall_sweep_executor.py`

The FSM must not do IK. This node exposes an action (`SweepLine`: start point,
end point, speed) and owns generation, validation, execution monitoring, and
bounded replanning of each arm-driven sweep.

#### Initial trajectory generation

1. Transform `seg_start` and `seg_end` into `arm_base`. Define the authoritative
   lateral tangent as `normalize(seg_end - seg_start)`.
2. Read `arm_base -> arm_tool0` from TF and compute the plate-derived lateral
   direction `normalize(cross(z_plate, up_in_arm_base))` only as a sanity check.
   Reject if it disagrees with the partition tangent beyond the configured
   tolerance (up to sign).
3. Capture the aligned plate orientation after `press_prepare`. This is the
   nominal orientation for the first trajectory; RX/RY are not force-compliant
   initially (§4.2).
4. Sample the Cartesian line at `waypoint_spacing_m` (3 cm -> ~27 waypoints for
   0.8 m), using the requested action `speed` / `sweep_speed_mps` for timing.
5. Run `closed_form_algorithm(T_i, q_prev, type=0)` per waypoint, chaining seeds
   for branch continuity (§3.5).
6. Compute a nominal joint velocity for every waypoint so the PTC receives
   `positions + velocities + time_from_start`, not a dense position-only path.
   Central finite differences are acceptable initially; a Jacobian-based
   `qdot = J^+ V` implementation is preferable when available. Interior
   waypoints should carry through-velocities; use zero velocity only where a
   deliberate sweep start/stop is required.
7. **Validate the complete trajectory before publishing:** no NaN, joint limits,
   `max|Δq| < max_joint_step_rad`, Jacobian conditioning / minimum singular
   value, required joint velocity limits, and FK-sampled Cartesian interpolation
   error between waypoints. Reject without publishing a partial trajectory.
8. Send the trajectory through the executor's **own `FollowJointTrajectory`
   action client** (see the warning in §3.2) — never via `planned_trajectory`,
   which would strip the velocities and re-time the sweep. The target controller
   is a node parameter so the same executor drives
   `passthrough_trajectory_controller` on the robot and
   `joint_trajectory_controller` in Gazebo (§11). Leave the PTC goal tolerances
   empty on the real robot (§3.4); in Gazebo, set per-joint
   `path_tolerance[i].position = -1` to erase JTC's path constraint (§11).

Because waypoint times are derived from Cartesian distance
`t_i = s_i / sweep_speed_mps`, changing the action `speed` (or the ctx knob used
to populate it) is the single control for commanded sweep speed. The same
trajectory generation path can be reused later if a live speed change is needed
mid-sweep: preempt and regenerate the remaining trajectory with the new speed.

#### Read-only wall alignment and orientation correction

Reuse the wall-orientation calculation from `wall_parallel_controller` without
allowing it to publish arm commands (§4.2). Prefer extracting the sensor geometry
and target-orientation calculation into a shared `wall_alignment_estimator`
module consumed by both nodes.

During execution, if the distance-sensor-derived angular error exceeds
`sweep_orientation_replan_threshold_deg` (with hysteresis and a minimum replan
interval), preempt the current PTC goal, capture the current TCP/joints, compute
sweep progress, and regenerate only the remaining path using the corrected
orientation. Blend the orientation correction over
`sweep_orientation_correction_blend_m`; keep the Cartesian lateral timing based
on the same requested `sweep_speed_mps`.

A replacement trajectory must pass the same IK, Jacobian, joint-velocity and
FK-interpolation validation before it is sent. Force Mode remains active on Z
while the PTC goal is replaced.

#### Continuous contact / safety watchdog

The executor must continuously monitor both independent contact signals during
the whole sweep:

- UR FT data / wall-normal force,
- the six distance sensors and derived plate-to-wall distance / consistency.

Reuse existing tested contact thresholds where possible. A transient shorter
than `sweep_contact_loss_timeout_s` may be ignored; sustained contact loss,
excessive force, invalid/inconsistent distance geometry, PTC failure, or a UR
safety/controller fault aborts the sweep. The executor cancels/preempts the PTC
goal and returns a specific failure reason; `scan_wall.py` then performs the
existing GPR stop -> Force Mode stop -> retract cleanup sequence. Keep those
state-machine responsibilities out of the executor.

#### Diagnostics for bench validation

Record enough data to evaluate whether the new controller actually tracks the
requested sweep and wall orientation. For each sweep, log at least:

- timestamp and partition / line identifiers,
- desired and measured TCP pose,
- desired and measured lateral progress and TCP lateral speed,
- lateral and wall-normal TCP deviation,
- desired vs. measured plate orientation / angular wall-parallel error,
- all six distance readings and their estimated wall plane,
- measured FT wrench,
- PTC goal state and every orientation-replan event.

Provide this as ROS topics suitable for `ros2 bag` and, if convenient, a compact
per-sweep CSV. The recorded data should make it straightforward to plot TCP path
in the wall tangent/normal frame, speed vs. time, force vs. time, and orientation
error vs. time after each bench run.

#### Partition-length feasibility backoff

If validation shows that a nominal partition length is not feasible, do **not**
split that partition in half. Reduce the allowed maximum length by
`partition_length_backoff_m` (initially 0.20 m) and re-run the equal-length
partitioning for the affected reachable segment so full scan coverage and the
configured overlap are preserved:

```
0.80 m max fails -> retry affected reachable segment at 0.60 m max
0.60 m max fails -> retry at 0.40 m max
...
```

Stop when a feasible partitioning is found or `partition_min_length_m` is
reached; below that, report the region unreachable rather than looping.

**Reuse the tool0<->DH-frame calibration trick** that `wall_parallel_controller`
already does at startup (FK of current joints vs. live tool0 TF) rather than
re-deriving flange/tool0 conventions. See its docstring around lines 60-68.

**`arm_base` is static during the sweep** (base at the final scan pose, turret
fixed), so solving in that frame is safe.

### 7.C Changes to `scan_wall.py`

**`transit` phase** — the Nav2 target becomes the **final fixed scan pose**
for that partition, not the segment start. Compute it from the partition centre
and wall normal as specified in §7.A, using the tunable
`partition_base_standoff_m`, and require `turret_link +X` to point into the
wall. Extend/wrap `base_standoff_goal(...)` as needed rather than hard-coding
this geometry inside the FSM.

In `sweep_use_arm=true` mode, reaching this Nav2 goal replaces the old separate
`park` correction: skip `_run_parking` and proceed directly from
`transit_wait` to `sweep_setup`. Keep the `park` phase unchanged only for the
legacy/base-driven A/B path.

**`press_settle` phase** — after `_wall_contact_ready` and
`_gpr_start_measurement_and_line`, instead of `_send_base_goal` /
`_start_sweep_crawl`:
- stop `wall_parallel_controller` from publishing arm trajectories (keep the
  distance sensors running; the shared/read-only alignment estimator remains
  available), then
- send the sweep goal to the executor for `seg_start -> seg_end` using the
  configured `sweep_speed_mps`.

Do not otherwise restructure the existing handoff sequence in Step 1.

**`sweep_wait` phase** — wait on the executor's action result instead of
`self._nav_status`. The executor owns contact watchdogs and any sensor-triggered
PTC preempt/replan cycles. Everything after a successful result (GPR stop,
force-mode stop, arm process teardown, `_segments_ok += 1`,
`_last_swept_point`) stays unchanged. Watchdog or validation failures flow into
the existing retract/failure handling.

**Gate off, do not delete** the base-crawl machinery — it exists solely to make
the *base* crawl slowly, and you want it for A/B testing on real hardware:
`_apply_sweep_speed`, `_restore_sweep_speed`, `_start_sweep_crawl`,
`_sweep_crawl_tick`, `_publish_speed_limit`, `_set_progress_checker`.
Put them behind `ctx["sweep_use_arm"]` (default `true`).

**`_build_force_mode_request`** (`scan_wall.py:561`) — keep the currently tested
Force Mode values for the first arm-sweep tests. Change only the gimbal default:

| field | current / initial Step-1 value | change |
|---|---|---|
| `wrench.force.z` | `5.0` | keep; expose as `scan_wall_press_force_n`, default `5.0` |
| `selection_vector_z` | `True` | keep |
| `selection_vector_rx/ry` | absent | keep non-compliant initially; optional gimbal remains behind `scan_wall_gimbal_press=false` |
| `gain_scaling` | `0.5` | keep |
| `speed_limits.linear.z` | `0.1` | keep; already tested safe with the 5 N force limit |
| `damping_factor` | `0.025` | keep |
| `task_frame` | `arm_tool0`, `x=-0.08` (GPR offset), `type=2` | keep |

> The controller resolves `task_frame` through TF **once**, at service-call
> time, and it stays fixed in the base frame afterwards. That is correct for a
> straight sweep — but it means the frame must be right *before* engaging.

**X/Y deviation limits:** keep the current `0.1` m values initially. Do not raise
them automatically on an abort. Log commanded vs. measured TCP deviation first;
a 100 mm lateral tracking error during a 50 mm/s sweep indicates a control or
trajectory problem that should be diagnosed before widening the containment
limit.

**Contact threshold:** keep the existing tested `_wall_contact_ready` force
threshold paired with the 5 N press-force setting. If the press force is tuned
later, update the contact threshold deliberately in the same test campaign.

### 7.D Parameters (all ctx knobs)

| knob | default | notes |
|---|---|---|
| `sweep_use_arm` | `true` | master A/B switch vs. the existing base sweep |
| `partition_max_length_m` | `0.8` | initial requested reach; **start here, not 2.0** — see §9 |
| `partition_overlap_m` | `0.05` | seam continuity for the GPR line |
| `partition_base_standoff_m` | current tested scan standoff | wall-normal distance from the partition centre to the final base scan pose; tune on hardware |
| `partition_length_backoff_m` | `0.20` | reduce max partition length by 20 cm after a feasibility failure, then repartition the affected reachable segment |
| `partition_min_length_m` | `0.20` | terminate the backoff loop below this length and report unreachable |
| `sweep_speed_mps` | `0.05` | single commanded Cartesian lateral speed used to time the PTC trajectory; easy to change per sweep |
| `waypoint_spacing_m` | `0.03` | initial IK sampling; FK interpolation validation may force local subdivision |
| `max_joint_step_rad` | `0.35` | branch-flip / reconfiguration reject |
| `sweep_orientation_replan_threshold_deg` | `2.0` | initial unvalidated angular drift threshold; bench-tune from logged data |
| `sweep_orientation_replan_hysteresis_deg` | `1.0` | re-arm threshold to avoid repeated replans near the boundary |
| `sweep_orientation_correction_blend_m` | `0.10` | lateral distance over which a corrected orientation is blended in |
| `sweep_orientation_replan_min_interval_s` | `0.5` | cooldown between PTC preempt/replan events |
| `sweep_contact_loss_timeout_s` | `0.20` | sustained force/distance contact-loss dwell before abort; bench-tune |
| `sweep_log_diagnostics` | `true` | record desired/measured TCP, speed, FT, distances, orientation error, replans |
| `scan_wall_press_force_n` | `5.0` | keep currently tested safe Force Mode value initially |
| `scan_wall_gimbal_press` | `false` | RX/RY compliance disabled initially; previous gimbal test was unreliable |
| `sweep_controller_name` | sim: `joint_trajectory_controller`, robot: `passthrough_trajectory_controller` | controller the executor drives; the §11.2 mitigation (run the real path with no press) is a change to this plus `scan_wall_press_force_n` |
| `sweep_goal_tolerance_rad` | sim `0.05`, robot `0.0` | 0 leaves PTC's goal tolerance empty (§3.4); meaningful in Gazebo, where nothing excuses missing the endpoint |
| `min_singular_value` | `0.02` | executor: reject a sweep that passes near a singularity, where the joint speeds needed to hold the commanded Cartesian rate blow up |
| `max_interpolation_error_m` | `0.005` | executor: worst TCP bow off the commanded line BETWEEN waypoints. Waypoints are on the line by construction, so this is the only check that can catch coarse spacing |
| `joint_velocity_margin` | `0.8` | fraction of rated joint speed the sweep may ask for; headroom stops the robot's own speed scaling from stretching the trajectory |
| `sweep_scan_standoff_m` | `0.30` | how far off the wall the plate sweeps when nothing is pressing it. Its own knob, not `scan_wall_plate_offset`: that one sizes the gap force mode presses through, this one has to survive the whole sweep untouched. Ignored under Force Mode |
| `scan_wall_plate_offset` | `0.20` | gap left for force mode to press through; in arm-sweep mode the sweep uses `sweep_scan_standoff_m` instead. Sized against the six range sensors, not the detected geometry. Sweeping on the surface means contact forces in Gazebo and a scrubbing GPR wheel on the robot |
| `sweep_approach_retract_m` | `0.20` | extra clearance the plate approaches and retracts at, so a long lateral move never drags it along the wall face |
| `partition_transit_use_crawl` | `true` | strafe between partition scan poses over /cmd_vel instead of Nav2, which cannot plan between two poses inside the inflation band |
| `partition_transit_speed` | `0.15` | m/s for that strafe |
| `nest_lines_in_partition` | `false` | sweep every height at one base stop (§8). Off until Step 1 is proven |
| `sweep_serpentine_heights` | `false` | alternate the sweep direction per height within a partition (§8.2). Off: a constant direction keeps every GPR line acquired the same way |
| `scan_wall_line_change_plate_offset` | `0.40` | plate standoff while the column moves between heights at one partition; try `0.20` to make the retract and re-approach no-ops |
| `scan_wall_press_force_n` | `5.0` | force-mode press; the contact gate defaults to `-this`, so the pair cannot drift apart |
| `scan_wall_gimbal_press` | `false` | RX/RY compliance; warns loudly when on, since the commanded plate orientation stops being enforced |
| `contact_min_force_n` | `-2.0` | press relaxed past this = contact lost (dwell applies) |
| `contact_max_force_n` | `-25.0` | past this = abort immediately, no dwell |
| `orientation_replan_enabled` | `false` | S6 mid-sweep preempt + regenerate. Off until validated |
| `orientation_replan_threshold_deg` | `4.0` | **not the plan's 2.0** — that is the measured noise median |
| `orientation_replan_dwell_s` | `1.0` | how long the error must persist. Not in the plan; it is what makes the trigger usable |
| `orientation_blend_m` | `0.10` | lateral distance the correction is slerped in over |
| `orientation_max_replans` | `3` | hard cap per partition |
| `sweep_diagnostics_csv_dir` | `""` | directory for one CSV per sweep; empty disables it (the topic is always on) |
| `sweep_diagnostics_rate_hz` | `20.0` | diagnostics sample rate |
| `sweep_timeout_factor` | `4.0` | trajectory watchdog: multiple of the planned duration before a leg is called hung |
| `sweep_timeout_pad_s` | `30.0` | flat grace on top of that |
| `partition_transit_arrive_tol` | `0.15` | arrival tolerance; deliberately looser than the sweep's, and still tighter than `nav_pos_tolerance` |
| `partition_transit_timeout_pad_s` | `60.0` | grace beyond nominal, because the base tracks /cmd_vel ~7x slower than commanded |
| `sweep_max_traverse_m` | `1.10` | longest traverse from where the arm's normal-only approach left the plate to the partition start; carries the whole lateral move |
| `sweep_max_plunge_m` | `0.50` | longest first leg the executor plans as a straight Cartesian line; beyond it the FSM's approach did not land, and the executor says so instead of flipping the wrist branch |
| `lead_in_speed_mps` | `0.08` | the arm's travel from the partition centre (where the base parks) to the partition start crosses open air, not wall |
| `nav_cancel_on_arrival` | `true` | cancel the Nav2 goal once the base reaches the goal POSITION, instead of letting it burn six recovery cycles on a rotation §9 proves it cannot plan |
| `nav_approach_partition` | follows `sweep_use_arm` + `should_face_wall` | NavigateToTarget drives straight to partition 1's scan pose instead of the wall's reachable start, so ScanWall's first transit is a no-op |
| `partition_scan_yaw_tol` | `nav_yaw_tolerance` (`0.25`) | heading error ScanWall accepts when skipping a partition transit. **Must not be tighter than what NavigateToTarget converges to**, or partition 1 re-transits every time |

Existing `scan_wall_plate_offset` (0.20 m) is already the 20 cm retract
distance — no new parameter needed.

## 8. STEP 2 — Nest line heights inside partitions

**Do not start this until step 1 is proven on the real robot.**

Goal: at one base stop, scan **all** line heights before moving the base.
Payoff quantified in §6.

### 8.1 Target control flow

```
compute segments once (height-independent, §3.6)
partition them once
for partition in partitions:            # base moves ONCE per partition
    transit_clear → transit             # Nav2 goal is already the final scan pose; no separate park in arm-sweep mode
    for line_z in scan_lines:           # column moves; base does NOT
        column → line_z
        arm_approach                    # re-measures standoff every time
        press_prepare → press_settle
        sweep (serpentine: alternate direction per height)
        retract
    (next partition)
column retract once, at the very end
```

### 8.2 What has to change

**Segment/partition computation moves out of the per-line path.** Today
`reachable_wall_segments` runs inside `_run_scan` on every re-entry
(`scan_wall.py:1236`). Compute once, cache in ctx, reuse for all heights.
Because segmentation is height-independent (§3.6) this is also *more correct*:
it guarantees identical partition boundaries at every height.

**The line loop moves inside `_run_scan`.** Today line advance happens by state
exit + re-entry (`check_transition` → `"ScanWall"`). That has to become an inner
loop, which means `on_enter`, `_finalize_line`, `_run_post_scan` and
`check_transition` no longer bracket each line. **This is the invasive part** —
a control-flow change to the state, not a phase-body swap.

**`_run_pre_approach` splits.** Today (`scan_wall.py:1056`) it does two things
per line: send the arm to `unfolded_fsm`, then move the column to the line
height.
- The `unfolded_fsm` pose move stays **per wall** (or per partition).
- The column move becomes **per line, inside the partition loop**.
- `_start_distance_sensors` (line 1143) stays at the end of the outer
  pre-approach — it should remain up for the whole wall.

**`_run_post_scan` splits.** Today (`scan_wall.py:1869`) it does:
1. arm retract via re-sending the `unfolded_fsm` pose (lines 1874-1918)
2. column retract, **only when `not self.more_lines`** (lines 1920-1933)
3. `walls_left -= 1` and `scan_done`, also gated on `not self.more_lines`

New split:
- (1) **drops entirely between heights** — confirmed on hardware: a 40 cm plate
  standoff is enough clearance for the column to move, so the column can travel
  from the retracted-but-extended pose and the per-line `unfolded_fsm` re-pose
  is unnecessary. The full re-pose stays only at wall entry/exit.

  Between heights at the same partition the base is *not* moving, so 40 cm is
  more retraction than the situation needs — it exists to stop the plate being
  dragged sideways across the wall during a base transit. Use a separate knob
  (`scan_wall_line_change_plate_offset`, default `0.40`) so it can be tried at
  `0.20` without touching the transit case; at 0.20 the retract and the
  subsequent `arm_approach` become no-ops, saving two arm moves per height.
- (2) and (3) move to the very end of the wall, after the last partition.
  `more_lines` stops being the gate — a new "last partition and last line"
  condition replaces it.

**Serpentine changes level.** Today it alternates per line across the whole
wall. New: partitions advance monotonically along the wall, and the **arm**
sweep direction alternates per height *within* a partition (L→R at z₀, R→L at
z₁, …). This is free and strictly better — the arm ends each sweep where the
next one starts, so there is no return travel.

`_last_swept_point` / `ctx["target_scan_point"]` bookkeeping
(`scan_wall.py:1566-1569`) needs rethinking under this scheme.

**Failure isolation.** A height failing at one partition must skip to the next
height, not abandon the partition. `_segments_ok` becomes two-dimensional
(per partition × per height) or is replaced by a per-partition tally.

**GPR grouping.** At one base stop you now run N_lines LINE_SCANs back to back.
Consider whether one measurement can hold multiple lines rather than
start/stopping the measurement per sweep.

### 8.3 New parameters

| knob | default | notes |
|---|---|---|
| `nest_lines_in_partition` | `false` | enable only after Step 1 is proven; switching it on activates the Step-2 restructuring |
| `scan_wall_line_change_plate_offset` | `0.40` | plate standoff while the column moves between heights at one partition. 0.40 is confirmed sufficient; try `0.20` to save two arm moves per height |

---

## 9. Open questions / bench-test checklist

1. **Base scan-pose standoff and orientation.** Tune
   `partition_base_standoff_m` on hardware and verify from TF/logged poses that
   each Nav2 goal is centred on the partition and finishes with `turret_link +X`
   pointing into the wall. Confirm that skipping `_run_parking` leaves enough
   repeatability for arm approach and that Nav2 goal tolerances do not accept an
   orientation error large enough to hurt reachability.

   > ### MEASURED CONSTRAINT: Nav2 cannot manoeuvre at the scan standoff
   >
   > Not an open question any more — measured in simulation, 2026-08-25.
   >
   > | | value | source |
   > |---|---|---|
   > | published footprint radius | 0.7 m | `dynamic_footprint_publisher` `base_radius` |
   > | arm expansion, on top | up to 0.30 m | `arm_tip_radius` |
   > | costmap inscribed radius | 0.6 m | `nav2_params_omni.yaml` `robot_radius` |
   > | inflation radius | 0.6 m | `nav2_params_omni.yaml` `inflation_radius` |
   >
   > DWB needs roughly **footprint + inscribed = 1.3 m** of centre-to-wall
   > clearance before any pose is collision-free. The scan standoff is
   > necessarily *inside* that band — the arm has to reach the wall, and UR10e
   > reach is 1.3 m — so **every** scan pose sits where DWB refuses to plan.
   > Observed at 1.21 m standoff as:
   >
   > ```
   > No valid trajectories out of 6656!
   > 0.00: BaseObstacle/Trajectory Hits Obstacle
   > 1.00: RotateToGoal/Nonrotation command near goal
   > ```
   >
   > `BaseObstacle` rejects the pure rotations, `RotateToGoal` rejects anything
   > with translation, so the sample set empties. **This is independent of
   > heading** — it reproduces with the legacy along-wall goal too, so it is not
   > caused by the wall-facing change.
   >
   > `base_radius`, `robot_radius` and `inflation_radius` are all fixed by the
   > platform and cannot be reduced, and raising the standoff to 1.3 m would put
   > the arm at full extension — the very thing this redesign exists to avoid.
   > So the final heading correction must **bypass DWB**: `NavigateToTarget`
   > rotates in place over `/cmd_vel` when the position is already within
   > tolerance (`fine_correction_rotate_only`, default on).
   >
   > That is safe where DWB is merely conservative: the base-only footprint is a
   > **circle**, so rotating about its centre sweeps no new area, and DWB's check
   > is a static footprint-vs-inscribed-cells test rather than a swept-volume
   > one. The argument does **not** extend to translation, which is why the full
   > x/y servo stays behind `fine_correction_enabled` (it was seen oscillating).
   >
   > Consequence for §7.A: `partition_base_standoff_m` cannot be chosen to satisfy
   > Nav2. Assume every partition transit ends with a DWB-refused rotation and
   > relies on the in-place servo.

2. **Partition length and feasibility backoff.** `base_goal_max_offset` is
   documented as arm reach ≈1.3 m for the UR10e, but that is the
   *base-to-target* radius, **not** usable lateral sweep at fixed orientation.
   Start at `partition_max_length_m=0.8`. If the validator rejects a reachable
   segment, reduce the maximum by 0.20 m and repartition that affected reachable
   segment (0.8 -> 0.6 -> 0.4 ...), preserving overlap and full coverage. **2.0
   m is not realistic.**

3. **Sensor-based orientation correction (§4.2).** The passive RX/RY gimbal has
   already been tried and was unreliable, so it starts disabled. Bench-test the
   read-only wall-alignment estimator, determine a useful angular deadband from
   recorded sensor/TCP data, and verify that PTC preempt + remaining-trajectory
   regeneration corrects orientation without visible jerk or scan gaps.

4. **Constant-speed sweep.** Verify from logged TCP data that
   `sweep_speed_mps` produces approximately constant wall-tangent TCP speed.
   Check the effect of waypoint spacing, joint velocity assignment, orientation
   correction, robot speed scaling, and PTC preemption on speed ripple.

5. **TCP / force / sensor diagnostics.** Save complete sweep data and generate
   plots after bench tests: tangent-vs-normal TCP path, desired/actual lateral
   speed, wall-normal force, six distance channels, and angular wall-parallel
   error. Use these plots to choose the final orientation threshold, watchdog
   dwell, and interpolation tolerances rather than guessing them.

6. **Force-mode X/Y deviation limits (§7.C).** Keep the existing 0.1 m values
   initially. If a Force Mode abort occurs, inspect the logged commanded vs.
   measured deviations first; do not automatically raise the limits.

7. **Press force / Force Mode gains.** Keep the currently tested safe values
   (5 N, gain scaling 0.5, damping 0.025, Z speed limit 0.1 m/s) for the first
   arm-sweep tests. Tune them only after the new motion architecture is proven,
   one parameter family at a time.

8. **Contact watchdog.** Verify independent detection of contact loss with both
   FT and distance sensors, plus excessive-force and invalid-distance cases.
   Test that the abort path cancels PTC, stops GPR/Force Mode and retracts
   cleanly.

> ### The trajectory BRIDGE also has to stand down, not just the controller
>
> Measured 2026-08-26. §3.3 said "two publishers driving one PTC is not
> survivable" and the fix was taken to be "stop `wall_parallel_controller` before
> the sweep". That is necessary and **not sufficient**:
>
> ```
> 413943  FSM sends the sweep goal (wall_parallel_controller already killed)
> 413954  JTC goal  <- executor's preempt-hold
> 415039  JTC goal  <- executor's TRAVERSE
> 415062  publisher_joint_trajectory_planned: "Executing trajectory failed."
> 415081  JTC goal  <- the BRIDGE, re-dispatching a stale trajectory
> 417452  sweep failed [controller_failed]: traverse finished with status 5 (CANCELED)
> ```
>
> `publisher_joint_trajectory_planned` keeps the last `planned_trajectory` it
> received and re-dispatches it whenever a slot opens (`timer_callback`:
> `trajectory_received and execution_complete`). The executor's own preempt frees
> exactly that slot, so the bridge fires `wall_parallel_controller`'s last target
> and preempts the traverse.
>
> The bridge now takes a latched `trajectory_bridge/hold` (`std_msgs/Bool`). While
> held it cancels its active goal, **drops** the pending trajectory (releasing must
> not fire a target that is stale by a whole partition), and refuses to dispatch.
> The FSM asserts it in `press_settle`, clears it in `sweep_wait`, and clears it
> again in `on_exit` so the arm stack is never left muted. Regression-tested in
> `arm_control/test/test_trajectory_bridge_hold.py`.

> ### Step 2 landed without the invasive restructuring §8.2 expected
>
> §8.2 called the line loop "the invasive part" — moving it inside `_run_scan`
> means `on_enter`, `_finalize_line`, `_run_post_scan` and `check_transition` stop
> bracketing each line. That turned out to be avoidable: the height loop fits
> **inside the existing `_seg_phase` machine** as three new phases, so `run()` and
> `check_transition` are untouched.
>
> ```
> transit_clear -> transit_recenter -> transit -> transit_wait
>      -> line_column        <- column to this partition's FIRST height
>      -> sweep_setup -> arm_approach -> press_prepare -> press_settle -> sweep_wait
>           -> line_change -> line_change_wait -> line_column   (more heights here)
>           -> transit_clear                                    (heights done, next partition)
> ```
>
> `_finalize_line` becomes wall-level when nesting: `more_lines` stays False, so
> the state exits once and `_run_post_scan`'s column retract and `walls_left`
> naturally run at the end of the WALL rather than the end of a line — exactly
> §8.2's requirement, with no change to their gating.
>
> ### The column must be down before the base moves
>
> Nesting exposed a hazard that was always latent: the base transited to the next
> partition with the column still at the **last height of the previous one**. On a
> tall wall that is the full extension, with the arm on top — a tall, top-heavy
> load to drive. (It predates nesting: without it the base already transited
> between partitions at the current line height. Nesting just made it obvious,
> because the column ends each partition at the TOP.)
>
> The transit now defers: `transit` picks the goal and parks it, `transit_column`
> lowers the column to `column_min_height_m`, `transit_send` sends it. Ordering
> matters and is tested: the arm folds first (`transit_recenter`), *then* the
> column drops — retracting with the arm still extended sideways swings it through
> a much wider arc.
>
> Three properties worth keeping:
>
> - **A skipped transit costs no column travel.** The retract sits after the skip
>   test, not before it, so a base already at its scan pose never moves the column.
> - **A column that will not retract fails the wall.** It does not transit anyway —
>   moving the base with the column up is the thing the phase exists to prevent.
> - **Not gated on nesting.** The base moves between partitions in both modes and
>   the hazard is identical.
>
> The column is raised again from `line_column` after the base arrives, so the
> sweep never runs at the retracted height. `test_column_safety.py` pins the
> structure — that transit motion happens in exactly one phase, that only the
> retract reaches it, and that the timeout branch fails rather than falling
> through.
>
> The legacy base-driven sweep (`sweep_use_arm=false`) still moves the base with
> the column up **during the sweep itself** — there the base IS the scan motion, so
> that is inherent to the mode rather than an oversight.
>
> Two details that only appear once the base stops moving between heights:
>
> - **The column must be driven back to the first height after every transit.** It
>   is wherever the previous partition's last height left it, so `transit_wait`
>   routes through `line_column` rather than straight to `sweep_setup`.
> - **The between-heights retract is not the transit retract.** The base is not
>   moving, so the plate only has to clear the wall enough for the column to
>   travel. `scan_wall_line_change_plate_offset` (0.40) is its own knob, and it is
>   `retract_only` — at the scan standoff it collapses to a no-op, which is §8.2's
>   stated goal of saving two arm moves per height.
>
> Failure isolation per §8.2: a height that fails moves to the **next height**, not
> away from the partition. A column that will not reach its height is treated the
> same way rather than stranding the wall.
>
> **Serpentine between heights is OFF.** §8.2 proposes alternating the arm's
> direction per height within a partition — "free and strictly better", since the
> arm would finish each height where the next begins and save a return traverse.
> It is implemented behind `sweep_serpentine_heights` but defaults to false:
> every height sweeps the same way, so consecutive GPR lines are acquired
> identically and nothing downstream has to know which direction a line was
> recorded in. A reversed line is the kind of error that only surfaces much later,
> in stitching.
>
> The cost is a return traverse of one partition length (~0.8 m) between heights.
> That is the executor's traverse leg and it fits inside `sweep_max_traverse_m`
> (1.10 m) — worth re-checking if partitions are ever lengthened.
>
> §6's payoff, recomputed against what the code actually does (the column
> controller skips a command when already at the target, so same-height commands
> cost nothing): 8 partitions x 4 heights is **32 base stops -> 8**, at the cost of
> 3 -> 31 column travels. A single-height wall is a **no-op**, which is where most
> of the regression risk would otherwise sit.

> ### Skipping a rejected partition was silently losing wall
>
> `backoff_lengths()` was written and unit-tested in the first pass and then never
> called, so a partition the executor refused was simply **skipped** — that stretch
> of wall dropped out of the scan with only a warning.
>
> Now a rejection whose reason means "too long for the arm"
> (`ik_unreachable`, `near_singular`, `joint_velocity`, `branch_discontinuity`)
> re-cuts the whole **reachable segment** it came from at
> `max_len - partition_length_backoff_m`, splices the replacements in where the old
> partitions were, rewinds to the first of them and sweeps again. Down to
> `partition_min_length_m`, then the region is reported unreachable rather than
> shortened forever.
>
> Deliberately the segment and not the partition, per §7.B: re-cutting the segment
> as a whole keeps the partitions equal-length and the overlap intact, which
> halving one piece would not. `plan_wall_partitions` now records which reachable
> segment each partition came from (`current_wall_partition_sources`) so a failure
> can be traced back to the segment that has to be re-cut.
>
> The splice is the fiddly part — replacements must land exactly where the old
> partitions were so the list stays in sweep order and neighbouring segments are
> untouched — and it has its own test file.
>
> ### The contact watchdog now has both signals
>
> §7.B asks for two *independent* contact signals. Only the distance half existed.
> Added: contact lost when the press force relaxes past `contact_min_force_n`
> (-2.0 N), and **excessive force past `contact_max_force_n` (-25.0 N), which
> aborts immediately** rather than waiting out the dwell — it is the one failure
> where waiting is itself the damage.
>
> The two signals are kept independent on purpose: a plate resting on a ledge reads
> close but unloaded, and one on a dark surface reads unloaded but is in contact.
> Force is only consulted where a wrench exists, so Gazebo is unaffected.

> ### S6: the plan's replan threshold is the noise floor
>
> First recorded Gazebo run (`rosbag2_2026_08_28-10_37_58`, 3708 samples, 2 full
> partitions + 1 partial). Tracking is excellent — max lateral error **0.1 mm**,
> max normal error **4.7 mm**, orientation error **0.000 deg** — so the trajectory
> side of §7.B is doing its job. Two findings that change the design:
>
> **1. The sweep runs at 29% of the commanded speed.** Median measured TCP rate
> 0.0147 m/s against 0.050 commanded, and the arm is momentarily *stalled* on 6% of
> samples. This is the same 1.5-3.5x slowdown seen in the leg durations, now
> measured on the TCP itself. §9.4 wants a constant wall-tangent speed; it is
> constant, but at a third of what was asked for. Still unexplained.
>
> **2. `wall_tilt_deg` is essentially all noise at the sweep standoff.** Jitter std
> **1.015 deg** about a 1 s-smoothed curve, sample-to-sample median change 0.217
> deg, no trend across the partition. The cause is structural: at the 0.30 m sweep
> standoff the ToFs are past their 0.258 m ceiling, so **`n_sensors_used == 3` for
> the entire run** — three ultrasonics, an exact fit, zero redundancy, and every
> millimetre of range noise maps straight into the fitted normal. (10 mm sigma over
> the 0.342 m baseline predicts ~2.4 deg raw, ~1.0 deg after the alpha=0.3 EMA.
> Observed: 1.015 deg.)
>
> So the §7.D default `sweep_orientation_replan_threshold_deg = 2.0` is **the
> median of the noise**. Measured over the flat wall, where every trigger is by
> definition false:
>
> | threshold | dwell | false replans | raw exceedances |
> |---|---|---|---|
> | **2.0 deg (the plan)** | none | **11** | 32 |
> | 3.0 deg | none | 10 | 22 |
> | 4.0 deg | none | 6 | 10 |
> | 3.0 deg | 1.0 s | **0** | — |
> | **4.0 deg** | **1.0 s** | **0** | — |
>
> **A dwell is required, and the plan does not have one.** §4.2 specifies threshold
> + hysteresis + cooldown only. Hysteresis cannot suppress a noise floor: widening
> it changes nothing until it is wide enough to stop re-arming, at which point the
> trigger has been disabled rather than fixed (measured: at threshold 2.0, a 2.0 deg
> band puts the re-arm point at 0.0 deg, so it fires once and never again — and
> would silently stop correcting real drift too).
>
> Shipped: threshold **4.0 deg**, dwell **1.0 s**, hysteresis 1.0 deg, cooldown
> 5.0 s, blend 0.10 m, at most 3 replans per partition. The recorded tilt trace is
> checked in as `arm_control/test/data/recorded_sweep_tilt.json` and the trigger is
> regression-tested against it, including a guard that the node's declared defaults
> still match the analysis.
>
> **`orientation_replan_enabled` defaults FALSE.** A mid-sweep trajectory preempt
> is not something to switch on silently.
>
> **Caveat worth carrying into S7:** with only three ultrasonics in range this
> channel may not be good enough to correct from at all. If the ToFs come into
> range during the pressed sweep on hardware, the fit gains redundancy and the
> noise floor should drop; if they read below their 0.011 m floor in contact
> instead, it will not, and S6 should stay off.

> ### An empty `-p name:=` kills the node before it can advertise
>
> Adding the CSV knob shipped a spawn command containing
> `-p diagnostics_csv_dir:=` whenever the directory was unset — which is the
> default. rcl cannot parse an empty parameter value and rejects the **whole**
> argument list:
>
> ```
> Failed to parse global arguments
> RCLError: Couldn't parse parameter override rule: '-p diagnostics_csv_dir:='
> ```
>
> The executor died at startup, and the only symptom upstream was
> `'sweep_line' action never came up` — a dead node and a slow one are
> indistinguishable from an action client. Every partition then failed with the
> arm never moving.
>
> Three fixes, because the silence was worse than the bug:
>
> - Optional string parameters are appended only when non-empty.
> - That timeout now says **why**: never started / exited with code N / running but
>   not advertising. The FSM holds the `subprocess.Popen`, so `poll()` is right
>   there — it just was not being read.
> - A dead executor is restarted before the next sweep. It is started once per line
>   in the pre-approach, so without that a single crash silently cost the whole
>   wall.

> ### S5 diagnostics: one message, every channel §7.B asks for
>
> `arm_control/msg/SweepDiagnostics` on `~/diagnostics` (20 Hz, ctx-tunable) plus
> an optional per-sweep CSV (`sweep_diagnostics_csv_dir`, off by default). One
> `ros2 bag` of a run is enough to plot TCP path in the tangent/normal frame,
> speed vs time, force vs time and orientation error vs time — which is what §9
> says the S6 thresholds must be tuned from, rather than the 2.0 deg the plan
> guesses.
>
> This is also where S3 pays off: the executor now runs a `WallAlignmentEstimator`
> **read-only** and records `wall_tilt_deg`, the channel S6 will trigger on, while
> owning the trajectory controller and never publishing an arm command from it.
> It also subscribes the FT wrench, so the force channel is populated wherever a
> broadcaster exists.
>
> Validated end to end against a fake arm that actually executes the trajectories
> it is given (a static joint state hides half the channels — progress and speed
> both read zero, which is how the double-consumer bug below was found):
>
> | leg | progress | plate-to-wall | normal error |
> |---|---|---|---|
> | traverse | 0.48 -> 0.01 | 0.256 -> 0.396 m | +0.056 -> +0.196 |
> | plunge | 0.00 | 0.380 -> 0.213 m | +0.180 -> +0.013 |
> | **sweep** | **0.01 -> 0.99** | **held at 0.200 m** | **max 0.2 mm** |
> | retract | 1.00 | 0.216 -> 0.396 m | +0.016 -> +0.196 |
>
> Measured sweep speed 0.203 m/s against a 0.05 m/s command executed at 4x in the
> harness. `n_sensors_used` reads 6 during the sweep and 3 during the retracted
> legs, which is the ToF ceiling (0.258 m) doing exactly what §S3 predicted.
>
> One bug worth remembering: `_publish_feedback` and `_publish_diagnostics` both
> consumed `_last_sample` to difference the TCP, so whichever ran first ate the
> delta and the other always reported **zero speed**. There is now a single
> producer and the feedback reads it back.

> ### S3: the plane fit had no outlier protection at all
>
> Extracting the estimator (2026-08-27) was meant to be a pure move -- and it was:
> the new module reproduced `wall_parallel_controller`'s fit **bit-identically**
> (0.00e+00 discrepancy) across flat, tilted, noisy, dropped-sensor and outlier
> frames. Writing the tests then found something the extraction was not looking
> for.
>
> **The IRLS/Huber fit gives zero robustness against a sensor that has missed the
> wall.** The validity windows are the *sensors'* ranges, not plausible *wall*
> distances -- an ultrasonic is valid out to 3.90 m -- so one seeing past a wall
> edge passes the window and drags the fit:
>
> | bad reading | fitted normal deflected by |
> |---|---|
> | 0.40 m | 23 deg |
> | 0.90 m | 56 deg |
> | 1.20 m | 65 deg |
> | 2.00 m | 77 deg |
>
> More IRLS iterations change nothing (with six points the MAD is itself
> contaminated, so every weight stays near 1). **This is almost certainly the
> mechanism behind the wall-end reorientation** that made the plate travel for 67 s
> when the approach was aimed at a partition START.
>
> Fixed with a plane-consistency trim ahead of the fit. Two approaches were tried:
>
> - *Gate on spread from the median raw range.* Wrong: a steeply tilted plate has a
>   large spread but small residuals, so it ate legitimate readings exactly when
>   the plate was worst aligned.
> - *Iteratively drop the worst residual.* Also wrong: least squares has a 0%
>   breakdown point, so with **two** bad readings the first fit is dragged far
>   enough that the largest residual belongs to a good sensor — measured, it
>   discarded the two correct ones and left a 75 deg error.
> - **Exhaustive RANSAC consensus** (shipped): fit through every triple of valid
>   sensors, keep the largest set within `max_residual` of that plane. 20 exact 3x3
>   solves at most — nothing at 2 Hz — and no breakdown mode, because a wrong model
>   simply explains fewer points.
>
> Measured after the fix: **0.00 deg** deflection for one *or* two edge outliers at
> every magnitude tested; nothing trimmed beyond the validity windows at any tilt
> from -30 to +30 deg; 1.9% of clean noisy frames drop one of six sensors.
> `DEFAULT_MAX_PLANE_RESIDUAL_M = 0.03` (3x the 0.010 m sensor sigma); set 0 to
> restore the original unguarded behaviour for a hardware A/B.
>
> **What to expect on hardware.** Measured over 1500 frames per row, error against
> the *known* true tilt (not agreement between the two fits — at 10 mm sensor noise
> on a 0.344 m baseline the fit is noise-dominated, so any subset change moves it
> several degrees regardless):
>
> | frames | guarded | unguarded |
> |---|---|---|
> | clean, sigma 10 mm | **1.14 deg** mean error | 1.13 deg |
> | clean, sigma 3 mm | **0.33 deg** | 0.33 deg |
> | one ultrasonic past the wall edge | **4.24 deg** | **39.24 deg** |
>
> So: no measurable cost when every sensor sees the wall, and the wall-edge failure
> mode essentially removed. On ~0.6% of outlier frames the fit is refused outright
> — the controller logs and holds orientation, which is the safe outcome.
>
> Two sizing lessons, both measured rather than reasoned:
>
> - **Never trim below four sensors.** Three fit a plane exactly, so a 3-sensor
>   consensus has zero residual by construction and always wins with nothing left
>   to check it. With the floor at 3, a clean 4-sensor frame (both top ToFs past
>   their 0.258 m ceiling) lost a good sensor and the recovered tilt fell from 10.2
>   to 5.2 deg — the guard was worse than no guard. Floor is now 4, so a 4-sensor
>   frame is not trimmed at all.
> - **The ToF ceiling limits where this helps.** Past ~0.26 m only the three
>   ultrasonics are in range, so the fit runs on exactly three points and the trim
>   cannot engage. The guard therefore protects the close-in press (real robot),
>   **not** the 0.30 m contact-free sweep — where a single bad ultrasonic is still
>   undetectable.
>
> The bench diagnostic table now distinguishes `(x)` out of the sensor's range from
> `(p)` in range but off the common plane, because they mean different things when
> you tilt the plate by hand to check the layout.

> ### The standoff knobs drift apart — keep them derived, not independent
>
> Raising `sweep_scan_standoff_m` from 0.20 to 0.30 silently broke the post-sweep
> sequence, because `scan_wall_transit_plate_offset` stayed at 0.40:
>
> ```
> executor 'retract' leg  -> plate at 0.50 m off the wall   (0.30 sweep + 0.20 retract)
> transit_clear target    ->          0.40 m
> advance = d - target    ->         +0.10 m, and +advance is ALONG plate +Z, INTO the wall
> ```
>
> So every partition ended: retract to 0.50 -> push 10 cm back toward the wall ->
> fold to `unfolded_front_fsm` anyway. Two fixes, one specific and one general:
>
> - In arm-sweep mode `transit_clear` now targets `_approach_standoff()` — the same
>   number the executor's retract leg used — so the two cannot disagree.
> - `_send_arm_to_clearance` takes `retract_only`. A move whose whole purpose is to
>   clear the wall must never close on it, whatever the arithmetic says.
>
> **Plate-to-wall standoffs, in one place** (arm-sweep mode; each is a ctx knob):
>
> | knob | m | what sits there |
> |---|---|---|
> | `sweep_scan_standoff_m` | 0.30 | the sweep plane — where the plate crosses the wall |
> | + `sweep_approach_retract_m` | 0.20 | ⇒ **0.50** = approach / traverse / post-sweep retract plane |
> | `scan_wall_plate_offset` | 0.20 | legacy press gap; arm-sweep mode no longer uses it |
> | `scan_wall_transit_plate_offset` | 0.40 | legacy transit clearance; arm-sweep mode now derives this |
> | `scan_wall_line_change_plate_offset` | 0.40 | Step 2 only, not implemented |
>
> **Base-to-wall standoffs:**
>
> | knob | m | what sits there |
> |---|---|---|
> | `partition_base_standoff_m` | falls back to `min_base_standoff` | the partition scan pose |
> | `min_base_standoff` | 0.75 | hard floor on any nearest-free base goal |
> | `base_wall_standoff` | 1.6 | fixed fallback when the costmap search fails |
> | `base_goal_max_offset` | 1.3 | arm-reach cap on the goal search, not a standoff |
>
> That is five plate knobs and four base knobs for what is really two independent
> quantities plus offsets from them. Audited 2026-08-27 — only two were genuinely
> dead, and both were leftovers from deleted code rather than legacy knobs:
>
> - `max_plunge_m` / `sweep_max_plunge_m` — declared and passed, never read after
>   the traverse check replaced the plunge check. **Removed.**
> - `costmap_utils.wall_outward_normal` — added for the Cartesian partition
>   approach, which was itself deleted. **Removed.**
>
> Everything else is live: `scan_wall_plate_offset` and
> `scan_wall_transit_plate_offset` still drive the legacy base-sweep path (§7.C
> keeps it for A/B testing) *and* `scan_wall_plate_offset` sizes the force-mode Z
> deviation limit on the real robot; `base_wall_standoff` / `min_base_standoff` /
> `base_goal_max_offset` all run inside `base_standoff_goal`, which
> NavigateToTarget still calls. `scan_wall_line_change_plate_offset` has no code
> references because Step 2 is unimplemented — it is planned work, not dead code.
>
> **The same drift reaches the press.** `_build_force_mode_request` sizes the Z
> deviation limit from `scan_wall_plate_offset` (0.20 + 0.05 -> 0.25 m), but in
> arm-sweep mode the arm approaches at `_approach_standoff()` = **0.50 m**. Force
> Mode would run out of compliant travel before touching the wall, and
> `_wall_contact_ready` would burn its 120 s timeout and sweep without contact —
> exactly the failure the knob's own comment predicts. §7.C says not to raise
> deviation limits automatically, so this now logs a warning naming both numbers;
> the standoff and the press force want retuning together in one campaign. **Add
> it to the S7 list.**

> ### The arm re-centres before every base transit
>
> `transit_clear` retracts the plate along the plate normal only, which leaves the
> arm wherever the sweep ended -- up to half a partition off to one side. Strafing
> the base with the arm extended sideways swings the plate through a much larger
> arc than the base itself covers. A `transit_recenter` phase now re-sends the
> named unfolded pose before the transit, so the lateral move is the base's own
> footprint and nothing more, and every partition starts from the same arm
> configuration instead of from wherever the last sweep finished.
>
> **The re-centre must be on every path out of `transit_clear`, including "skip".**
> First cut routed the skip case straight to `transit`. That was invisible until
> the `retract_only` fix made the Z retract a no-op on every partition after the
> first -- at which point the arm stopped folding back at all, and the base began
> strafing with the arm still extended sideways. All three entries into `transit`
> now sit inside `transit_recenter` / `transit_recenter_wait`, so in arm-sweep mode
> the fold-back cannot be skipped.

> ### Force Mode and the executor own the wall-normal axis in turn
>
> The FT contact gate is already implemented and wired, in this repo:
> `fsm_node.py` subscribes `/force_torque_sensor_broadcaster/wrench` into
> `ctx["ft_wrench"]`, `_measured_press_force` reads its Z, and
> `_wall_contact_ready` blocks `press_settle` until `Fz <= scan_wall_touch_force_n`
> (default **-5.0 N**, 120 s timeout, then it proceeds anyway so a mis-reading
> sensor cannot deadlock a run). Nothing about that lives only on the robot.
>
> But the executor as first written would have **broken** it. Sizing the sweep
> standoff from the range sensors is right with no press and wrong with one:
>
> | | no press (Gazebo, §11.2 first hardware run) | Force Mode pressing |
> |---|---|---|
> | who holds the plate off the wall | the executor | Force Mode |
> | range sensors in that state | valid | **below their valid floor -> `plate_wall_distance()` returns None** |
> | `plunge = measured - scan_standoff` | correct | raises, or lifts the plate off the wall it just pressed |
> | post-sweep retract leg | correct | fights the still-active press |
>
> So the executor now branches on the goal's `press` flag: with a press it sweeps
> in the plane the press established (`plunge = 0`), requires no distance reading,
> and skips the retract leg — `transit_clear` pulls the plate back from a measured
> distance once the FSM has released force mode.
>
> **Press ordering, corrected 2026-08-28.** The intended sequence is:
>
> 1. engage Force Mode,
> 2. the arm moves to the first sweep point **while touching the wall**,
> 3. once contact is detected there, the sweep starts.
>
> Step 2 was the part I had backwards. An earlier note here warned that the
> traverse "drags a pressed GPR wheel across the wall" — but the GPR is a *wheel*;
> rolling along the wall is what it is for. The real defect was the opposite:
> under press the executor still pulled the plate `approach_retract_m` (0.20 m)
> OFF the wall to traverse and plunged back at the start. That is a 0.20 m
> commanded retreat while Force Mode presses — it fights the press and trips its
> deviation limits. Fixed: with `press` set the traverse happens in the contact
> plane, with no retreat and no plunge leg.
>
> **Still open (step 3), for hardware:** contact is confirmed at the partition
> CENTRE, before the traverse, and not re-confirmed at the sweep start. On a flat
> wall the press holds through the roll, so this is a robustness gap rather than a
> blocker — and where the contact threshold sits relative to the rolling friction
> is exactly the kind of thing that wants a real robot to tune.

> ### The arm runs 1.5-3.5x slower than its trajectory asks for
>
> Measured in Gazebo, 2026-08-26, from the JTC's own log. Every leg **completes** —
> it just takes far longer than its `time_from_start` specifies:
>
> | leg | planned | actual | ratio |
> |---|---|---|---|
> | plunge | 2.5 s | 8.8 s | 3.5x |
> | sweep | 15.9 s | >33.8 s | >2.1x |
> | arm Z move, -0.031 m | ~0.4-0.9 s | 1.4 s | ~2-3x |
>
> Not a fixed settling overhead (that would give the sweep ~22 s, not >34 s) and
> not obviously speed scaling. **Unexplained** — candidates are gz_ros2_control PID
> tracking plus JTC waiting on `goal_tolerance`, or a sim real-time factor below 1.
>
> The consequence for this work is that the executor's trajectory watchdog exists
> to catch a **hung** controller, not to enforce performance. At `1.5x + 10 s` it
> was cancelling motion that was executing perfectly well — every sweep failed
> `controller_timeout` mid-traverse. Now `trajectory_timeout_factor` (4.0) and
> `trajectory_timeout_pad_s` (30 s), and each completed leg logs its actual /
> planned ratio so the gap stays measurable:
>
> ```
> sweep leg done in 33.1s (planned 15.9s, 2.1x).
> ```
>
> If that ratio ever approaches 4x, the watchdog is about to start firing on
> healthy motion again — chase the slowdown then rather than raising the factor.
>
> The executor is also launched with `use_sim_time` in simulation. Its watchdog
> compares against a trajectory's planned duration, so it has to share the clock
> the controller advances trajectories on.

> ### Partition transits cannot use Nav2 either
>
> Measured 2026-08-26, and it is the same wall as §9's rotation finding. A transit
> between two adjacent partition scan poses is a 0.74 m strafe with both endpoints
> inside the costmap inflation band, so the **global** planner cannot even reach
> the goal cell:
>
> ```
> GridBased: failed to create plan with tolerance 0.50
> Planning algorithm GridBased failed to generate a valid path to (-0.12, 0.67)
> No valid trajectories out of 6656! BaseObstacle/Trajectory Hits Obstacle
> ```
>
> and the recovery behaviours then drive the base backwards, away from the wall,
> undoing the scan pose it had already reached.
>
> Between partitions the move is a pure strafe **along** the wall at constant
> standoff and constant heading, which the omnidirectional base does directly. So
> `partition_transit_use_crawl` (default `true`) routes it through the existing
> `_start_sweep_crawl` /cmd_vel servo, now taking an explicit target yaw, instead
> of Nav2. Same argument §9 made for the in-place rotation: DWB is being
> conservative rather than right.
>
> **The base tracks /cmd_vel far more slowly than commanded.** First measured
> transit covered 0.548 m in 24.4 s against 0.15 m/s commanded — about a seventh
> of the requested speed. Unexplained; likely twist_mux arbitration or a base
> controller limit, and worth chasing separately. Until then the transit uses a
> generous `partition_transit_timeout_pad_s` (60 s) and reports achieved vs
> commanded speed on timeout so the gap stays visible.
>
> `partition_transit_arrive_tol` is 0.15 m, not the sweep's 0.05 m: the scan pose
> itself is only ever reached to `nav_pos_tolerance` (0.30 m), so a tighter transit
> tolerance is stricter than the pose the FSM already accepted, and the arm
> re-measures its own standoff afterwards regardless. The first run failed at
> 0.102 m from the target — inside what Nav2 would have called arrived.
>
> **The cost is real:** the strafe is not obstacle-checked. It is acceptable only
> because `reachable_wall_segments` has already established that a base cell
> exists within arm reach along this stretch of wall. Revisit if partitions ever
> span a region the reachability check did not vet.

9. **GPR cadence.** Partitioning multiplies LINE_SCANs by ~3–4x, each with a
   press/release cycle and an HTTP round trip to the Proceq. Unknown whether the
   instrument tolerates that cadence and whether short overlapping lines stitch
   cleanly downstream. **This is the one place the new approach is genuinely
   worse than the old one.**

10. ~~**Column-move clearance between heights.**~~ **Resolved:** 40 cm of plate
   standoff is enough for the column to move — no `unfolded_fsm` re-pose needed
   between heights. Remaining sub-question: whether 20 cm also suffices, which
   would make the retract/re-approach pair a no-op. See §8.2 and
   `scan_wall_line_change_plate_offset`.

11. **Wall-surface obstacles.** Pre-existing TODO at `scan_wall.py:1226` — the
    sweep assumes a flat wall and the 2D costmap never sees plate-height
    protrusions. The new distance/force watchdog improves reaction to sudden
    wall geometry changes but does not solve obstacle avoidance.

## 10. Implementation order

> **Superseded for phasing by §11.** The items below remain the correct list of
> work; §11 re-orders them by what can actually be validated where, and is the
> order being followed.


1. Add/verify partition scan-pose generation: partition centre + tunable
   `partition_base_standoff_m`, with `turret_link +X` pointing into the wall;
   bypass `_run_parking` in arm-sweep mode and validate the final pose from TF.
   **Done**, plus: the partition plan (reachable split -> partitions -> scan
   poses) is built once by `costmap_utils.plan_wall_partitions` and memoised in
   ctx, so NavigateToTarget approaches partition 1's scan pose directly and
   ScanWall skips that transit. Both order the line through
   `wall_partitioning.sweep_line_order` so they cannot disagree on which
   partition is first.
2. `utils/wall_partitioning.py` + unit-testable in isolation, including the
   0.20 m max-length backoff / repartition behavior.
3. Extract or expose the distance-sensor wall-alignment calculation as a
   read-only reusable estimator; verify its orientation output offline against
   recorded sensor data.
4. `arm_control/control/wall_sweep_executor.py` — generate positions +
   velocities + timing, add IK/Jacobian/FK interpolation validation, and verify
   constant-speed trajectories offline against recorded poses.
5. Add executor diagnostics/logging and the FT + distance-sensor contact
   watchdog. Validate abort handling without GPR first.
6. Wire into `scan_wall.py` behind `sweep_use_arm`, keeping the existing tested
   5 N Force Mode settings and gimbal disabled. Bench-test fixed-orientation arm
   sweeps first.
7. Enable sensor-triggered orientation correction: threshold -> PTC preempt ->
   regenerate remaining trajectory -> resume. Tune threshold/hysteresis/blend
   from the logged TCP/sensor plots.
8. Real-robot A/B against the base sweep and validate GPR stitching/cadence.
9. Only then: Step 2 (nesting), initially with
   `nest_lines_in_partition=false` until explicitly enabled.

---

## 11. Simulation strategy and phased order

**Decision: validate the workflow in Gazebo/RViz, then go straight to the real
robot.** Hybrid/URSim is deliberately skipped.

### 11.1 What the two simulators actually provide

There are two distinct simulation modes in this workspace and they cover
**disjoint halves** of this plan:

| | Full Gazebo (`navi_wall/sim.launch.py`) | Hybrid (`navi_wall/hybrid_simulation.launch.py`) |
|---|---|---|
| arm backend | `gz_ros2_control::GazeboSimROS2ControlPlugin` (`mobile_manipulator.urdf.xacro:192`) | **URSim** — real `ur_robot_driver` (`sim:'false'`, `use_fake_hardware:'false'`, `robot_ip 192.168.56.101`) |
| walls / Nav2 / costmap | yes | base side only |
| 6 plate distance sensors | **yes — real Gazebo raycasts against world walls**, bridged by `arduino_sensors_sim` | no (arm is not in Gazebo) |
| `force_mode_controller` | **no** — `force_mode/*` interfaces do not exist | yes |
| `passthrough_trajectory_controller` | **no** — `trajectory_passthrough` interfaces do not exist | yes |
| `mode_compatibility_` check (§3.1) | no (not UR's hardware interface) | yes |
| wall contact / FT force | no | **no — URSim has no wall** |

Both controller YAMLs (`navi-wall/config/mobile_manipulator_controllers.yaml`,
`arm_only_controllers.yaml`) *list* `force_mode_controller` and
`passthrough_trajectory_controller`, but under `gz_ros2_control` they cannot
activate: the interfaces they claim are not exported. Do not read their presence
in the YAML as evidence they work in Gazebo.

**Neither simulator can validate contact.** Skipping hybrid therefore costs less
than it appears — hybrid would have added only controller-switch and PTC
mechanics, still with no wall.

### 11.2 Consequences of skipping hybrid

The binary `ctx["sim"]` flag is **sufficient**; no `arm_backend` tri-state is
needed. The existing gating already does the right thing:

- `_start_force_mode` no-ops on `ctx["sim"]` (`scan_wall.py:595`)
- `_wall_contact_ready` returns immediately in sim (no FT sensor to wait for)

So the Gazebo sweep runs **contact-free at the 20 cm `scan_wall_plate_offset`**,
exactly as today's sim sweep does. Geometry and sequencing are verified; contact
is not.

**Accepted risk:** the real execution path — PTC + Force Mode — is exercised for
the first time on the robot. Four things differ from what Gazebo proves:

1. PTC takes the supplied velocities and timing as given; Gazebo's JTC re-splines them.
2. Force Mode is active *during* PTC execution.
3. The `mode_compatibility_` switch must accept force_mode + PTC together (§3.1).
4. Preempt/replan behaviour while Force Mode is pressing (§4.2).

**Mitigation, first hour on hardware:** the executor's target controller is a
parameter, so run the real path once with the arm at standoff and **no press**
before enabling Force Mode. That separates "does PTC execute my trajectory
correctly" from "does contact hold".

### 11.3 Gazebo execution path

Same trajectory generation, different action server:

| | Gazebo | real robot |
|---|---|---|
| controller | `joint_trajectory_controller` | `passthrough_trajectory_controller` |
| path tolerance | JTC enforces it | none (§3.4) |
| goal tolerance | keep meaningful — without Force Mode the endpoint should be hit exactly | leave empty (§3.4) |
| Force Mode | skipped | active |

**JTC path tolerance is the concrete Gazebo trap.** The bridge zeroes velocities
precisely because *"finite-difference velocities at intermediate points cause the
cubic spline to overshoot and push the actual joint position outside the
tolerance window"* — and the executor deliberately sends velocities. Fix it in
the goal, not by crippling the trajectory: set
`path_tolerance[i].position = -1` per joint. Per `control_msgs/JointTolerance`,
`-1` erases the tolerance and lets the joint move without restriction (`0` means
"keep the default").

### 11.4 RViz visualisation (new work, not in §7)

Verifying the workflow visually is most of the value of the Gazebo phase:

- **Partitions distinct from reachable segments** — extend
  `publish_wall_segment_markers` (`costmap_utils.py:422`) with a second
  namespace/colour so the partition splits and their overlap are visible.
- **Base scan pose per partition** — position marker plus an arrow for
  `turret_link +X` pointing into the wall, so §7.A's geometry is checkable
  *before* the base moves.
- **Planned sweep line** as a `Path` in `arm_base`, published *before*
  execution — catches a bad tangent or a rejected partition without waiting for
  motion.
- **Desired vs. measured TCP** during the sweep; doubles as the §7.B diagnostics
  topic.

### 11.5 Phased order

| # | work | validated where |
|---|---|---|
| **S1** | `utils/wall_partitioning.py` + length backoff | **unit tests, no sim** |
| **S2** | partition scan pose (centre + `partition_base_standoff_m` + `turret_link +X`), skip `_run_parking` in arm-sweep mode, markers | Gazebo + RViz |
| **S3** | wall-alignment estimator extracted read-only | Gazebo (only place plate raycasts hit real walls) |
| **S4** | executor generation: IK / Jacobian / FK-interpolation / velocities | **offline**, recorded poses |
| **S5** | executor execution via JTC + RViz viz + diagnostics; wire into `scan_wall.py` behind `sweep_use_arm` | Gazebo |

> **S4/S5 status (2026-08-26).** Written; S4 unit-tested offline, S5 smoke-tested
> live against faked joint_states/TF. **Not yet run in Gazebo against a real
> controller.** New files:
>
> | file | what |
> |---|---|
> | `arm_control/planner/planner_lib/ur10e_kinematics.py` | FK, geometric Jacobian, `tool0`<->DH-end calibration — all in the analytic IK's own DH convention |
> | `arm_control/planner/planner_lib/sweep_trajectory.py` | `plan_sweep()`: line sampling, seeded IK chain, Jacobian velocities, and the six validation gates |
> | `arm_control/action/SweepLine.action` | the executor's action |
> | `arm_control/control/wall_sweep_executor.py` | the node: plan → validate → own `FollowJointTrajectory` client → contact watchdog → feedback/RViz |
>
> Measured offline against the real UR10e DH table, 0.8 m partition at 0.05 m/s:
> 28 waypoints, 16 s, max joint step 0.061 rad, sigma_min 0.198, 5% of rated
> joint speed, TCP bow **0.23 mm** against the 5 mm limit. IK/FK roundtrip is
> exact to 1e-15 and the analytic Jacobian matches finite differences to 4e-7.
>
> **Approach is split between the planner and the executor.** The base parks at
> the partition *centre*, so the plate starts in the middle of the partition, a
> long lateral move away from the sweep start. The first Gazebo run showed why
> that move cannot be a straight Cartesian line: the IK chain crossed the wrist
> singularity and flipped a joint by pi
> (`branch_discontinuity: joint step 3.130 rad ... between waypoints 51 and 52`).
>
> So the work is divided by what each side is good at:
>
> | leg | who | why |
> |---|---|---|
> | approach: plate out to `scan_wall_plate_offset + sweep_approach_retract_m`, **normal only, no lateral move** | the FSM, via `_send_arm_to_clearance` | the move that has always worked |
> | traverse: to the partition start AND onto the sweep plane, one diagonal line | executor | carries the WHOLE lateral move; safe as a diagonal because the approach plane is the further of the two, so wall distance decreases monotonically and never drops below `sweep_scan_standoff_m`. Was two legs (lateral, then a pure-normal `plunge`); merged to drop a stop-settle-accelerate cycle between two collision-free legs |
> | sweep: cross the partition | executor | the scan; must be a straight constant-speed Cartesian line |
> | retract: back off by the same margin | executor | symmetric with the approach; clears the plate before the base moves |
>
> ### The FSM must NOT pre-position the plate laterally
>
> Two attempts, both wrong, both worth recording:
>
> 1. **Cartesian approach to the partition START.** It is a wall end, and there
>    half the sensor plate overhangs into open space: the six ranges stop
>    describing one plane, `wall_parallel_controller` fits garbage and commands a
>    large reorientation.
> 2. **Cartesian approach to the partition CENTRE.** Worse. The partition centre
>    lies on the base's own centreline, and the arm planner carries the robot's
>    column as a **static 0.3 m cylinder at the `arm_base` origin**. The endpoint
>    check refused it outright:
>
>    ```
>    Global request goal wrist_3 position is INSIDE ENVIRONMENT OBSTACLE:
>      Cylinder (center=(0.0, 0.0), radius=0.3m, z=[-1.112, 1.3])
>    Position (wrist_3): [0.068, 0.211, 0.361]     -> 0.222 m radial, inside
>    Position (tool0):   [0.276, -0.005, 0.374]    -> 0.276 m radial, inside
>    ```
>
>    And retracting further from the wall makes it *worse*, not better: at this
>    base standoff, "back" is toward the column.
>
> So the FSM moves the arm along the plate normal only, and the **executor's
> traverse carries the entire lateral move** -- out at the standoff, where the
> plate is extended and well clear of the column. Verified offline against the
> real DH table: traverses of 0.4-1.0 m all plan with max joint step ~0.06 rad,
> sigma_min ~0.27 and 8% of rated joint speed. The 1.5 m failure that started this
> was a reach problem, not a length problem.
>
> `sweep_max_traverse_m` is the seam: if the plate is further than that from the
> partition start, the executor rejects with `approach_too_far` and says the
> approach did not land, rather than attempting the move that fails.
>
> ### Killing a controller does not stop its trajectory
>
> Measured in Gazebo, 2026-08-26. The FSM SIGINTs `wall_parallel_controller`
> before handing the arm to the executor. The process dies; the
> `FollowJointTrajectory` goal it already sent **keeps executing**:
>
> ```
> 211564  JTC "Received new action goal"   <- wall_parallel_controller's last command
> 213768  FSM stops wall_parallel_controller
> 216234  FSM sends the sweep goal
> 216736  executor: "plate is 0.90 m from the partition start"
> 278515  JTC "Goal reached, success!"     <- 67 s after it was issued
> ```
>
> A fixed settle delay is guesswork against that. The executor now **preempts and
> waits**: it sends a one-point goal at the current position (both JTC and PTC
> preempt an in-flight goal when a new one arrives, and "stay here" stops the arm),
> then polls `joint_states` until the positions go quiet for `at_rest_dwell_s`
> before it reads anything. Position differencing, not the velocity field --
> `joint_states` is merged from several publishers here and velocities are not
> always populated.
>
> **The sweep plane is offset from the wall, and sized by sensors.** The partition
> line lies on the *detected* wall surface; sweeping along it drives the plate
> into the wall. The executor projects the partition endpoints into a plane held
> `scan_standoff_m` off the wall, with the offset taken from the **measured**
> plate distance rather than the detected geometry -- the six range sensors know
> where the wall really is. Lateral and vertical placement still come from the
> partition geometry, which is authoritative there.
>
> The partition endpoints also carry the wall's *base* scan-line height. The FSM
> re-stamps them with the height being scanned now, or every line after the first
> sweeps at the wrong elevation.
>
> **Live smoke test (no controller running).** With faked `joint_states` and a
> `arm_base->arm_tool0` TF, the node calibrated, transformed the map-frame goal,
> and planned:
>
> ```
> Sweep planned: lead-in 15 wp / 5.0s, sweep 28 wp / 16.0s over 0.800 m at 0.050 m/s
>   (step 0.061 rad, sigma_min 0.198, 5% of rated speed, TCP bow 0.23 mm).
> ```
>
> — identical to the offline numbers, then failed cleanly at
> `controller_unavailable`. The RViz `~/planned_sweep` Path published 28 poses in
> `arm_base` before any motion, as §11.4 wants. All four rejection paths return
> actionable reasons rather than hanging:
>
> | goal | reason | detail |
> |---|---|---|
> | 3 m partition | `ik_unreachable` | no IK solution 0.621 m into the sweep |
> | 2 m/s | `joint_velocity` | joint 2 needs 188% of rated; lower `sweep_speed_mps` |
> | zero length | `degenerate_sweep` | length 0.000000 m |
> | vertical sweep | `tangent_disagreement` | tangent is 90 deg off the plate's lateral axis |
> | no robot state | `setup_failed` | no joint_states for the six arm joints yet |
>
> Still open for the first Gazebo run: whether `joint_trajectory_controller`
> accepts the velocity-carrying trajectory with `path_tolerance = -1` as §11.3
> predicts, and whether the 10 s alignment dwell in `press_prepare` leaves the
> plate parallel enough that the captured orientation is worth holding for a
> whole 0.8 m sweep.
| **S6** | orientation replan loop (preempt → regenerate remainder → resume) | Gazebo — purely kinematic, no force needed |
| **S7** | PTC + Force Mode, contact, watchdog, GPR, parameter tuning | **real robot only** |

### 11.6 What is actually implemented (2026-08-27)

Audited against the code, not against memory.

| item | § | state |
|---|---|---|
| `wall_partitioning.py` + equal-length overlapping split | 7.A | **done**, unit-tested |
| partition scan pose, `turret_link +X` into the wall, markers | 7.A | **done** |
| shared partition plan, NavigateToTarget approaches partition 1 | 7.A | **done** |
| `ur10e_kinematics` + `sweep_trajectory` (IK chain, Jacobian velocities, 6 validation gates) | 7.B | **done**, 42 offline tests |
| `wall_sweep_executor` + `SweepLine`, 4 legs, wired into `scan_wall.py` | 7.B/7.C | **done**, running in Gazebo |
| trajectory-bridge hold so two publishers cannot fight | 3.3 | **done**, 8 regression tests |
| partition transit over /cmd_vel instead of Nav2 | 9 | **done** |
| FT contact gate (`Fz <= scan_wall_touch_force_n`) | 7.C | **already existed**, verified wired |
| partition length backoff | 7.B | **done** — a rejected partition re-cuts its whole reachable segment shorter and retries; coverage preserved |
| `wall_alignment_estimator` extracted read-only, `wall_parallel_controller` refactored onto it | 4.2, S3 | **done**, 28 offline tests; extraction verified bit-identical before the robustness fix below |
| orientation replan (preempt → regenerate remainder → resume) | 4.2, S6 | **implemented**, thresholds derived from a recorded sweep; **`orientation_replan_enabled` defaults FALSE** — flip it for the S6 validation run |
| FT half of the contact watchdog | 7.B | **done** — force-relaxed and excessive-force paths added alongside the distance one; hardware-only to validate |
| full diagnostics (FT wrench, six distances, orientation error, per-sweep CSV) | 7.B | **done** — `arm_control/msg/SweepDiagnostics` on `~/diagnostics` + optional CSV; every channel validated against a moving fake arm |
| press ordering: engage → roll to the sweep start in contact → sweep | S7 | **mostly wired**; the missing piece is re-confirming contact AT the start point before sweeping — to tune on hardware |
| **press reach vs arm-sweep approach standoff** | 7.C | **not reconciled** — 0.25 m deviation limit against a 0.50 m approach; warns at runtime |
| `scan_wall_press_force_n` / `scan_wall_gimbal_press` as ctx knobs | 7.D | **done** — tested values kept as defaults; the contact threshold now defaults to `-press_force` so the two cannot be retuned apart |
| Step 2 (nest line heights inside partitions) | 8 | **implemented**, `nest_lines_in_partition` defaults FALSE; see the note below for what it does and does not restructure |

Nothing above blocks the Gazebo workflow. The first three "not started" rows plus
the S7 press ordering are what stand between here and a real-robot run.

S6 belongs in simulation: the replan is trajectory-level, so Gazebo can validate
the trigger, the blend and that the sweep resumes without a coverage gap. Only
"does Force Mode survive a preempt" (§4.2) is hardware-only.
