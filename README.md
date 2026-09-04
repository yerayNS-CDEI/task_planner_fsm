# task_planner_fsm: Autonomous Wall Scanning Task Orchestrator

A ROS2 Finite State Machine (FSM) package that orchestrates complete autonomous wall scanning missions by coordinating navigation, manipulation, and sensor systems for mobile robots with articulated arms.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Running the FSM](#running-the-fsm)
  - [Complete Simulation Setup](#complete-simulation-setup)
  - [Monitoring with RViz Panel](#monitoring-with-rviz-panel)
- [FSM States](#fsm-states)
- [Workflow](#workflow)
- [Checkpoint Resume (Debugging)](#checkpoint-resume-debugging)
- [Key Features](#key-features)
- [Important Notes](#important-notes)
- [Troubleshooting](#troubleshooting)

## Overview

The **task_planner_fsm** is a sophisticated finite state machine that orchestrates autonomous wall scanning missions by coordinating three main subsystems:
- **navi_wall**: Mobile base navigation and mapping
- **arm_control**: UR10e manipulator control and planning
- **Sensor systems**: Distance sensors, hyperspectral cameras, alignment systems

### Mission Workflow

The FSM executes a complete autonomous inspection mission through **17 distinct states** organized in two phases:

**Phase 1 - Sequential Wall Scanning:**
1. Create environmental map (exploration)
2. Identify objects in environment
3. Reconstruct 3D geometry
4. Compute wall definitions
5. Scan each wall sequentially (navigate → unfold arm → scan → fold arm)

**Phase 2 - Exhaustive Cell Coverage:**
6. Compute areas of interest
7. Discretize walls into inspection cells
8. Compute optimal base positions for full coverage
9. Perform exhaustive cell-by-cell scanning with column height control
10. Return to home position

### System Capabilities

The FSM can:
- **Autonomously explore** and create 3D maps of environments
- **Navigate** to optimal positions for wall scanning
- **Coordinate** arm and base movements for complete coverage
- **Execute** two-phase scanning: coarse sequential scanning followed by detailed exhaustive coverage
- **Adapt** arm column height to reach cells at different elevations (0-1.1m range)
- **Recover** from errors with automated retry mechanisms
- **Resume** from any state (checkpoint system for debugging)

---

## Prerequisites

### Software Requirements

**ROS2 Packages:**
- ROS2 (Humble or later)
- navi_wall - Navigation and mapping system
- arm_control - Manipulator control and planning
- Nav2 - Navigation stack
- RTABMap - SLAM system

**Python Dependencies:**
- rclpy
- tf2_ros, tf2_geometry_msgs
- std_msgs, geometry_msgs, nav_msgs, sensor_msgs
- nav2_msgs (for navigation actions)

**RViz Panel (Optional but Recommended):**
- pluginlib
- rviz_common
- C++ compiler (for building the panel)

### Hardware/Simulation Requirements

**For Complete Simulation:**
- **URSim** - Universal Robots simulator (Docker-based)
  - Required for ExhaustiveScan state with real UR kinematics
  - Model: UR10e
  - Version: 5.17.3 or compatible
  - Accessible at: 192.168.56.101:29999
- **Gazebo** - For mobile base simulation (launched by navi_wall)

**For Real Robot (Not Yet Tested):**
- UR10e robotic arm
- Mobile differential drive base
- Distance sensors (Arduino-based)
- Network connectivity to robot controllers

### Important: Mock Services

> **⚠️ NOTE:** The package currently uses **mock services** for several components. These are placeholders for development and testing. Real implementations are being developed and will replace mock services in future releases.

**Mock Services (temporary):**
- `/object_id_sim` - Object identification (will use real vision system)
- `/start_geometry_reconstruction` - 3D geometry reconstruction (will use real processing)
- `/compute_areas_of_interest` - Interest region computation (will use real analysis)

**Real Services (already integrated):**
- `/compute_wall_discretization` - arm_control package
- `/compute_optimal_base` - arm_control package
- `/arm/send_position` - arm_control package
- `/arm/script_command` - arm_control package
- `/navigate_to_pose` - Nav2 action server

---

## Setup

### Build the Package

```bash
cd ~/ros2_ws

# Build FSM node
colcon build --packages-select task_planner_fsm

# Build RViz panel
colcon build --base-paths src/task_planner_fsm/task_planner_fsm_rviz_panel

# Source workspace
source install/setup.bash
```

### Wall Configuration (Optional)

The FSM uses **predefined wall definitions** for faster testing. These are placeholders for future automatic geometry extraction.

To modify wall definitions for your specific map:

```bash
# Edit wall definitions in ComputeWallPoints state
nano ~/ros2_ws/src/task_planner_fsm/task_planner_fsm/states/compute_wall_points.py
```

Look for the `predefined_walls` section and adjust coordinates based on your environment:

```python
self.predefined_walls = [
            ((x1, y1, z1), (x2, y2, z2)),
            # Add more walls as needed
        ]
```

**Note:** Future versions will extract wall geometry automatically from the reconstructed 3D model.

---

## Running the FSM

### Complete Simulation Setup

To run the full FSM simulation, you need to launch **7 components in separate terminals**. Each component plays a specific role in the system.

#### Terminal 1: Start URSim

```bash
ros2 run ur_client_library start_ursim.sh -m ur10e -v 5.17.3
```

**Purpose:** Launches Universal Robots simulator for realistic UR10e kinematics  
**Port:** 192.168.56.101:29999  
**Required for:** ExhaustiveScan state (cell-by-cell scanning with column control)  
**Note:** Must be running before FSM reaches ExhaustiveScan state

#### Terminal 2: Start Mock Service Server

```bash
ros2 run task_planner_fsm mock_server
```

**Purpose:** Provides placeholder services for components under development  
**Services provided:**
- `/object_id_sim` - Simulates object identification
- `/start_geometry_reconstruction` - Simulates 3D reconstruction
- `/compute_areas_of_interest` - Simulates interest region computation

**Note:** This will be replaced by real service implementations in the future

#### Terminal 3: Wall Discretization Service

```bash
ros2 run arm_control wall_discretization_node
```

**Purpose:** Discretizes walls into panels and cells for exhaustive scanning  
**Service:** `/compute_wall_discretization`  
**Input:** Wall endpoints, cell size, panel dimensions  
**Output:** Panel centers, vertices, cell positions

#### Terminal 4: Optimal Base Placement Service

```bash
ros2 run arm_control optimal_base_service --ros-args -p map_relpath:=resource/rmap.npy
```

**Purpose:** Computes optimal mobile base positions for reaching all scan cells  
**Service:** `/compute_optimal_base`  
**Requirements:** Reachability map file (`resource/rmap.npy`)  
**Input:** Panel vertices and cell positions  
**Output:** Base position (x, y, θ) for maximum coverage

#### Terminal 5: UR Script Command Service

```bash
ros2 run arm_control script_command_service_node
```

**Purpose:** Executes custom UR Script commands for fine control  
**Service:** `/arm/script_command`  
**Used in:** ExhaustiveScan for direct `movel` commands  
**Connects to:** URSim (192.168.56.101) or real robot

#### Terminal 6: FSM Node

```bash
ros2 run task_planner_fsm fsm_node --sim true
```

**Purpose:** Main state machine orchestrator  
**Frequency:** 1 Hz state updates  
**Parameters:**
- `--sim true` - Enable simulation mode (Gazebo for base)
- `--sim false` - Use real hardware (not yet tested)

**What it does:**
- Manages all FSM state transitions
- Launches navigation and mapping processes
- Coordinates arm and base movements
- Publishes current state and transitions

#### Terminal 7: Start Signal

```bash
ros2 topic pub /start_flag std_msgs/Bool "data: true" --once
```

**Purpose:** Trigger FSM to begin mission  
**Result:** FSM transitions from Initialization → CreateMap  
**Alternative:** Can be sent from RViz panel or custom UI

---

### Monitoring with RViz Panel

The package includes a **custom RViz panel** for real-time FSM monitoring. This is **recommended** for understanding system behavior.

#### Build and Launch RViz Panel

```bash
# Panel is built with the package (see Setup section)

# Launch RViz
rviz2
```

#### Add FSM Panel to RViz

1. In RViz menu: **Panels → Add New Panel**
2. Select: **task_planner_fsm_rviz_panel → FsmPanel**
3. Panel appears showing:
   - **Current State**: Name of active FSM state
   - **Last Transition**: Previous state → Current state with reason
   - **Transition Log**: Scrolling history of all transitions

#### Panel Features

- **Auto-refresh**: Updates at 1 Hz
- **Topic Configuration**: Editable topic names for `/fsm/current_state` and `/fsm/transition`
- **Transition Reasons**: Shows why each transition occurred (e.g., "map_ready", "navigation_done")
- **Late Joining**: Works even if started after FSM

**Alternative Monitoring (Terminal):**

```bash
# Watch current state
ros2 topic echo /fsm/current_state

# Watch transitions (JSON format)
ros2 topic echo /fsm/transition
```

---

## FSM States

The FSM consists of **17 states** that execute sequentially with conditional transitions. Each state implements a lifecycle: `on_enter()` (setup) → `run()` (periodic execution) → `check_transition()` (condition check) → `on_exit()` (cleanup).

### State Flow Diagram

```
┌─────────────────┐
│ Initialization  │ (Wait for /start_flag)
└────────┬────────┘
         ↓
┌─────────────────┐
│   CreateMap     │ (Launch exploration, build 3D map)
└────────┬────────┘
         ↓
┌─────────────────┐
│    ObjectID     │ (Detect objects with vision system)
└────────┬────────┘
         ↓
┌─────────────────┐
│   Geometry      │ (Reconstruct 3D environment model)
│ Reconstruction  │
└────────┬────────┘
         ↓
┌─────────────────┐
│ ComputeWall     │ (Define wall geometries)
│    Points       │
└────────┬────────┘
         ↓
    ╔═══════════════════════════════════╗
    ║      PHASE 1: Sequential Wall     ║
    ║          Scanning Loop            ║
    ╚═══════════════════════════════════╝
         ↓
┌─────────────────┐
│  WallTarget     │ (Select nearest unscanned wall)
│   Selection     │
└────────┬────────┘
         ↓
┌─────────────────┐
│  NavigateTo     │ (Navigate base to wall)
│     Target      │
└────────┬────────┘
         ↓
┌─────────────────┐
│      Arm        │ (Extend arm to scanning position)
│   Unfolding     │
└────────┬────────┘
         ↓
┌─────────────────┐
│    ScanWall     │ (Execute wall scan trajectory)
└────────┬────────┘
         ↓
┌─────────────────┐
│      Arm        │ (Retract arm to storage position)
│    Folding      │
└────────┬────────┘
         ↓
    [If walls_left > 0: loop back to WallTargetSelection]
    [If walls_left = 0: continue to Phase 2]
         ↓
    ╔═══════════════════════════════════╗
    ║   PHASE 2: Exhaustive Coverage    ║
    ╚═══════════════════════════════════╝
         ↓
┌─────────────────┐
│  AreasOf        │ (Compute high-priority regions)
│   Interest      │
└────────┬────────┘
         ↓
┌─────────────────┐
│      Wall       │ (Discretize walls into cells)
│ Discretization  │
└────────┬────────┘
         ↓
┌─────────────────┐
│      Base       │ (Compute optimal base positions)
│   Placement     │
└────────┬────────┘
         ↓
┌─────────────────┐
│   Exhaustive    │ (Cell-by-cell scanning with column control)
│      Scan       │
└────────┬────────┘
         ↓
┌─────────────────┐
│      Arm        │ (Retract arm after exhaustive scan)
│    Folding      │
└────────┬────────┘
         ↓
┌─────────────────┐
│      Home       │ (Return to start position)
│    Position     │
└────────┬────────┘
         ↓
┌─────────────────┐
│    Finished     │ (Mission complete - terminal state)
└─────────────────┘

    [On any exception: → Error state with retry]
```

### State Descriptions

| State | Purpose | Key Actions | Transition Condition |
|-------|---------|-------------|----------------------|
| **Initialization** | System startup | Log home position (map origin), wait for external trigger | `/start_flag` received |
| **CreateMap** | Environment mapping | Launch persistent `mapping_stack.launch.py` + separate `exploration.launch.py`; after exploration finishes, run an optional structured densification coverage sweep (`create_map_densify_enabled`) | Map creation complete |
| **ObjectID** | Object detection | Bring up nav stack + YOLO and drive a structured coverage sweep (`object_id_sweep_enabled`) so the camera sees the whole space | Coverage sweep complete |
| **GeometryReconstruction** | 3D model building | Call `/start_geometry_reconstruction`, launch nav_sim with hybrid mode | Reconstruction complete |
| **ComputeWallPoints** | Wall definition | Define 3 predefined walls with endpoints and scan trajectories | Walls data populated |
| **WallTargetSelection** | Next wall selection | Find closest unscanned wall to current robot position | Wall selected |
| **NavigateToTarget** | Base navigation | Send Nav2 goal `/navigate_to_pose`, await result | Navigation complete |
| **ArmUnfolding** | Arm extension | Send `/arm/send_position` for 'unfolded_fsm' pose | Arm movement done |
| **ScanWall** | Wall scanning | Launch sensors, alignment; execute scan trajectory | Scan trajectory complete |
| **ArmFolding** | Arm retraction | Sequential folding: unfolded → folded via `/arm/send_position` | Arm folded |
| **AreasOfInterest** | Interest analysis | Call `/compute_areas_of_interest` service (mock) | Interest areas computed |
| **WallDiscretization** | Cell grid generation | Call `/compute_wall_discretization` for each wall, generate panels & cells | All walls discretized |
| **BasePlacement** | Optimal positioning | Call `/compute_optimal_base` for each panel, store positions by column | All bases computed |
| **ExhaustiveScan** | Full coverage scan | Navigate to each base, scan all cells with column height control (0.0-0.9m) | All panels complete |
| **HomePosition** | Return home | Navigate to (0, 0, 0) origin via Nav2 | Navigation complete |
| **Finished** | Mission end | Log completion, set finished flag | Terminal state |
| **Error** | Error handling | Log error details, retry failed state once | Terminal state |

### Error Recovery Mechanism

The FSM includes automatic error recovery:

1. **Exception Caught**: Any exception in a state's `run()` method is caught
2. **First Attempt**: FSM retries the same state once
3. **Second Failure**: If retry fails, FSM transitions to **Error** state
4. **Error State**: Terminal - logs error context, requires manual recovery

**Example:**
```
NavigateToTarget → Exception (network timeout)
    ↓
Retry NavigateToTarget → Success
    ↓
Continue to ArmUnfolding
```

---

## Workflow

### Complete Autonomous Mission

This is the **primary use case** - running a full autonomous wall scanning mission from start to finish.

**Prerequisites:**
- All 7 terminals launched (see [Complete Simulation Setup](#complete-simulation-setup))
- URSim running and accessible
- All services ready

**Execution:**

```bash
# Terminal 7: Trigger start
ros2 topic pub /start_flag std_msgs/Bool "data: true" --once
```

**Expected Sequence:**

1. **Initialization** (5 seconds)
   - Logs home position from `/rtabmap/odom`
   - Status: "Waiting for start signal..."

2. **CreateMap** (2-5 minutes)
   - Launches: `ros2 launch navi_wall global_exploration.launch.py`
   - Robot autonomously explores environment
   - Builds 3D map with RTABMap
   - Status: "Exploration in progress..."
   - Transition when: `/map_done` topic receives `true`

3. **ObjectID** (10 seconds)
   - Calls: `/object_id_sim` service
   - Mock response (or real vision processing in future)
   - Status: "Object identification complete"

4. **GeometryReconstruction** (15 seconds)
   - Calls: `/start_geometry_reconstruction`
   - Launches: `move_robot.launch.py` with `hybrid_sim:=true`
   - Status: "Geometry reconstruction started"

5. **ComputeWallPoints** (instant)
   - Defines 3 walls with hardcoded coordinates
   - Status: "Walls computed: Wall 1, Wall 2, Wall 3"

6. **Phase 1 Loop** (3-5 minutes per wall)
   
   For each wall:
   
   **WallTargetSelection**
   - Calculates distances from current position
   - Selects nearest unscanned wall
   - Status: "Selected Wall X at distance Y meters"
   
   **NavigateToTarget**
   - Sends Nav2 goal to wall scan position
   - Status: "Navigating to (x, y, θ)..."
   - Monitors navigation action result
   
   **ArmUnfolding**
   - Sends dashboard "play" command to URSim
   - Calls `/arm/send_position` with 'unfolded_fsm'
   - Status: "Arm unfolding..."
   - Waits for `/execution_status` = true
   
   **ScanWall**
   - Launches: `arduino_sensors_sim`
   - Launches: `align_ee_to_wall`
   - Navigates scan trajectory (predefined path along wall)
   - Status: "Scanning wall..."
   
   **ArmFolding**
   - Sequential folding to storage position
   - Status: "Arm folding..."
   
   **Loop**: Decrements `walls_left`, returns to WallTargetSelection if > 0

7. **AreasOfInterest** (5 seconds)
   - Calls: `/compute_areas_of_interest`
   - Mock analysis (future: real data processing)
   - Status: "Interest areas computed"

8. **WallDiscretization** (10-30 seconds)
   - For each wall:
     - Calls `/compute_wall_discretization`
     - Receives panels, cells, heights
   - Status: "Discretized Wall X: Y panels, Z cells"

9. **BasePlacement** (30-60 seconds)
   - For each panel:
     - Calls `/compute_optimal_base`
     - Receives optimal (x, y, θ) position
     - Stores by column index
   - Status: "Computed base for panel X"

10. **ExhaustiveScan** (10-20 minutes)
   
   For each base position:
   - Navigate to base
   - Set column height (0.0 to 0.9m in 0.1m increments)
   - For each cell at current height:
     - Send UR Script `movel` command via `/arm/script_command`
     - Execute scan (hyperspectral, distance, etc.)
   - Track completion by panel
   - Status: "Scanning panel X/Y, base Z, cell A/B"

11. **ArmFolding** (30 seconds)
   - Retract arm to storage position

12. **HomePosition** (1-2 minutes)
   - Navigate to origin (0, 0, 0)
   - Status: "Returning home..."

13. **Finished** (terminal)
   - Status: "Mission complete!"
   - FSM stops

---

## Checkpoint Resume (Debugging)

For **development and debugging**, the FSM supports resuming from any state. This avoids re-running the entire mission when testing specific states.

### Usage

```bash
ros2 run task_planner_fsm fsm_node --sim true \
  --initial-state <STATE_NAME> \
  --scan-phase <1|2>
```

**Parameters:**
- `--initial-state`: Name of state to start from (e.g., "ExhaustiveScan")
- `--scan-phase`: Force scan phase (1 = sequential walls, 2 = exhaustive coverage)

### Bootstrapping

When starting from a non-initial state, the FSM automatically generates synthetic data for required context:

**Example: Resume from ExhaustiveScan**

```bash
ros2 run task_planner_fsm fsm_node --sim true \
  --initial-state ExhaustiveScan \
  --scan-phase 2
```

**What happens:**
1. FSM loads default wall discretization and base placement data
2. User prompted: "Target wall index"
3. User prompted: "Target endpoint (1=start, 2=end)"
4. FSM verifies `nav_sim` is running (launches if needed)
5. Creates action client for arm control
6. Begins ExhaustiveScan state directly

### Common Checkpoint States

| State | Use Case | Prerequisites |
|-------|----------|---------------|
<!-- | **NavigateToTarget** | Test navigation only | URSim not needed, Nav2 required | -->
<!-- | **ArmUnfolding** | Test arm control | URSim required, Nav2 not needed | -->
| **ScanWall** | Test wall scanning | URSim + sensors required |  
| **AreasOfInterest** | Test exhaustive scanning | All services required |
<!-- | **ExhaustiveScan** | Test exhaustive coverage | All services required | -->
<!-- | **WallDiscretization** | Test discretization | wall_discretization_node required | -->
<!-- | **BasePlacement** | Test base optimization | optimal_base_service required | -->

---

## Key Features

### Two-Phase Scanning Strategy

**Phase 1 - Sequential Wall Scanning:**
- **Purpose**: Quick coarse scan of all walls
- **Approach**: One scan trajectory per wall at predefined scan line
- **Speed**: Fast (minutes per wall)
- **Coverage**: Broad overview of each wall surface
- **Use Case**: Initial inspection, defect detection, area selection

**Phase 2 - Exhaustive Cell Coverage:**
- **Purpose**: Detailed comprehensive scan
- **Approach**: Discretized cells scanned individually with optimal base positioning
- **Speed**: Thorough (10-20 minutes for full environment)
- **Coverage**: Complete with no gaps, overlapping cells for redundancy
- **Use Case**: Detailed material analysis, hyperspectral imaging, GPR scanning

### Column Height Control

During ExhaustiveScan, the FSM controls an articulated column to extend vertical reach:

- **Base arm reach**: Z = 0.0 to 0.2m (limited by arm kinematics)
- **Column extension**: 0.0 to 0.9m in 0.1m increments (10 height levels)
- **Total reach**: Z = 0.0 to 1.1m (base reach + column extension)
- **Control**: Reads `/joint_states` for column joint angle
- **Command**: Sends trajectory to `/column_controller/follow_joint_trajectory`

**Algorithm:**
```
For each base position:
    For height = 0.0 to 0.9m (step 0.1m):
        Set column to height
        For each cell at current height:
            Move end-effector to cell center (via movel)
            Execute scan
            Mark cell complete
```

### Process Lifecycle Management

The FSM manages complex subprocess lifecycles:

**Tracked Processes:**
- Gazebo simulation (`gz sim`)
- Navigation stack (`move_robot.launch.py`)
- Exploration (`global_exploration.launch.py`)
- Sensor drivers (`arduino_sensors_sim`)
- Alignment system (`align_ee_to_wall`)

**Features:**
- Graceful shutdown: SIGINT → SIGTERM → SIGKILL sequence
- Timeout handling: 5-second grace period before force kill
- Global cleanup: Registered with `atexit` and signal handlers
- Gazebo handling: Special force-kill patterns (`pkill -9 -f 'gz sim'`)

**Benefits:**
- No zombie processes after FSM shutdown
- Clean recovery from errors
- Proper ROS node shutdown

### Robust State Transitions

Each state implements a strict lifecycle:

```python
def on_enter(self, ctx):
    """Setup: Initialize resources, log entry"""
    
def run(self, ctx):
    """Periodic execution: Main state logic (1 Hz)"""
    
def check_transition(self, ctx):
    """Condition check: Return next state or None"""
    
def on_exit(self, ctx):
    """Cleanup: Release resources, log exit"""
```

**Transition Logging:**
- JSON format: `{"from": "StateA", "to": "StateB", "reason": "condition_met"}`
- Published to: `/fsm/transition`
- Captured by: RViz panel, terminal logs, bag files

### Context-Based State Sharing

All states share a common context dictionary (`ctx`) with:

| Key | Type | Purpose |
|-----|------|---------|
| `node` | RobotFSMNode | ROS2 node reference |
| `sim` | bool | Simulation mode flag |
| `start` | bool | Start signal received |
| `scan_phase` | int | Phase 1 or 2 |
| `map_ready` | bool | Map creation complete |
| `walls_data` | list[dict] | Wall definitions |
| `walls_left` | int | Unscanned walls counter |
| `base_position`, `base_orientation` | tuple | Current robot pose |
| `target_scan_wall`, `target_scan_point` | int | Selected wall/point |
| `wall_discretization_results` | dict | Panels, cells per wall |
| `optimal_base_results` | dict | Base positions by column |
| `column_current_height` | float | Current column extension |
| `completed_base_indices` | list | Processed base positions |
| `execution_status` | bool | Arm movement done flag |
| `nav_client` | ActionClient | Nav2 action client |
| `procs` | dict | Managed subprocess references |

---

## Important Notes

#### 1. Mock Services (Temporary)

> **⚠️ IMPORTANT:** Several services are currently **mocked** for testing purposes. Real implementations are being developed.

**Mock Services:**
- `/object_id_sim` - Returns immediate success without real vision processing
- `/start_geometry_reconstruction` - Returns success without 3D reconstruction
- `/compute_areas_of_interest` - Returns success without analysis

**Impact:**
- ObjectID state: No real object detection performed
- GeometryReconstruction: No 3D model generated
- AreasOfInterest: No intelligent region selection

**Timeline:** Real implementations expected in future releases

**Workaround for Now:**
- Use mock_server for simulations
- FSM continues through these states automatically
- Focus testing on navigation, arm control, and scanning states

#### 2. Hardcoded Wall Definitions

Wall geometries are **predefined** in `ComputeWallPoints` state for faster testing and simulation.

**Customization:**
- Edit `task_planner_fsm/states/compute_wall_points.py`
- Adjust coordinates to match your environment map
- Rebuild package

**Future:** Will extract wall geometries automatically from GeometryReconstruction output

#### 3. URSim Requirement for ExhaustiveScan

The **ExhaustiveScan** state requires URSim to be running for proper UR10e kinematics and script execution.

**Why:**
- Direct `movel` commands via `/arm/script_command`
- Column height control via `/column_controller/follow_joint_trajectory`
- Joint state monitoring via `/joint_states`

**Setup:**
```bash
# Must be started before FSM reaches ExhaustiveScan state
ros2 run ur_client_library start_ursim.sh -m ur10e -v 5.17.3
```

**If URSim Not Running:**
- Script commands will fail
- FSM will transition to Error state
- ExhaustiveScan cannot complete

**Alternative:** Use checkpoint resume to skip ExhaustiveScan during testing

#### 4. Real Robot Status

> **⚠️ NOTE:** The FSM has **not yet been tested on real hardware**. Testing is pending.

**Current Status:**
- ✅ Fully tested in simulation (Gazebo + URSim)
- ⚠️ Real robot testing pending
- ⚠️ Hardware integration verification needed

**Before Real Robot Use:**
- Verify all safety systems operational
- Test individual states with real hardware first
- Use reduced speeds for initial tests
- Have emergency stop accessible

### Performance Considerations

**FSM Update Rate:**
- State machine runs at **1 Hz** (timer callback)
- Adequate for navigation and manipulation coordination
- State transitions typically take seconds to minutes

**Navigation Performance:**
- Phase 1 navigation: 1-2 minutes per wall
- Phase 2 navigation: 30-60 seconds per base position
- Depends on map complexity and obstacle density

**Exhaustive Scan Duration:**
- 10-20 minutes for full environment
- ~5-10 seconds per cell
- Parallelization not currently implemented

---

## Troubleshooting

### Issue: FSM Stuck in Initialization

**Symptom:** FSM remains in Initialization state, status shows "Waiting for start signal..."

**Cause:** `/start_flag` topic not published or FSM didn't receive it

**Solution:**
```bash
# Check if FSM is subscribing to topic
ros2 topic info /start_flag

# Publish start signal
ros2 topic pub /start_flag std_msgs/Bool "data: true" --once

# Verify FSM transitioned
ros2 topic echo /fsm/current_state
```

### Issue: CreateMap Never Completes

**Symptom:** FSM stuck in CreateMap state for extended time

**Possible Causes:**

1. **Exploration not launched:**
   - Check if `global_exploration.launch.py` process running
   - Look for RViz window with exploration visualization

2. **Map done signal not sent:**
   - Verify exploration node publishes to `/map_done`
   - Manually trigger (for testing): `ros2 topic pub /map_done std_msgs/Bool "data: true" --once`

3. **Exploration failed:**
   - Check Gazebo simulation is running
   - Verify robot has working odometry and sensors
   - Check for navigation errors in terminal output

### Issue: NavigateToTarget Fails

**Symptom:** Navigation action returns failure, FSM retries or goes to Error state

**Solutions:**

1. **Nav2 not running:**
   ```bash
   # Check Nav2 action server
   ros2 action list | grep navigate_to_pose
   
   # If missing, restart NAV_sim or move_robot launch
   ```

2. **Goal unreachable:**
   - Target position may be in obstacle or invalid
   - Check RViz global costmap for blocked paths
   - Modify wall positions in `compute_wall_points.py`

3. **Localization lost:**
   - Robot lost position estimate
   - Check `/rtabmap/odom` topic for valid poses
   - Restart localization

### Issue: URSim Connection Failed

**Symptom:** ExhaustiveScan state shows "Script command failed" or connection errors

**Solutions:**

1. **URSim not running:**
   ```bash
   # Check URSim process
   ps aux | grep ursim
   
   # Start URSim if missing
   ros2 run ur_client_library start_ursim.sh -m ur10e -v 5.17.3
   ```

2. **Wrong IP address:**
   - Verify URSim at 192.168.56.101:29999
   - Check Docker network configuration
   - Test connection: `ping 192.168.56.101`

3. **URSim not ready:**
   - Wait 10-20 seconds after launching for full initialization
   - Check URSim GUI shows "Running" state

### Issue: Services Not Found

**Symptom:** State shows "Service /service_name not available"

**Cause:** Required service node not running

**Solution:**

| Service | Required Node | Terminal Command |
|---------|---------------|------------------|
| `/object_id_sim` | mock_server | `ros2 run task_planner_fsm mock_server` |
| `/start_geometry_reconstruction` | mock_server | (same as above) |
| `/compute_areas_of_interest` | mock_server | (same as above) |
| `/compute_wall_discretization` | wall_discretization_node | `ros2 run arm_control wall_discretization_node` |
| `/compute_optimal_base` | optimal_base_service | `ros2 run arm_control optimal_base_service --ros-args -p map_relpath:=resource/rmap.npy` |
| `/arm/script_command` | script_command_service_node | `ros2 run arm_control script_command_service_node` |

**Verify all services:**
```bash
ros2 service list
```

### Issue: FSM Crashes with Exception

**Symptom:** FSM node dies unexpectedly, Python traceback in terminal

**Debugging:**

1. **Check error message** in terminal output
2. **Identify failed state** from stack trace
3. **Common causes:**
   - TF transform lookup failure (robot pose unknown)
   - Service call timeout (server not responding)
   - Invalid data format (e.g., missing keys in context)

**Recovery:**
- Fix underlying issue (start missing node, fix TF tree, etc.)
- Restart FSM
- Or use checkpoint resume to skip problematic state

### Issue: ExhaustiveScan Never Completes

**Symptom:** FSM stuck in ExhaustiveScan for hours, status shows same cell repeatedly

**Possible Causes:**

1. **Column controller not responding:**
   ```bash
   # Check action server
   ros2 action list | grep column_controller
   
   # Check joint states publishing
   ros2 topic hz /joint_states
   ```

2. **Script commands failing silently:**
   - Check URSim GUI for errors
   - Verify `/arm/script_command` service working:
     ```bash
     ros2 service call /arm/script_command arm_control/srv/ScriptCommand "{script: 'movel([0.1,0.1,0.5,0,3.14,0],a=0.1,v=0.05)'}"
     ```

3. **Cell completion not tracked:**
   - Bug in state logic
   - Check `completed_base_indices` in context
   - Monitor `/fsm/current_state` logs

### Issue: Processes Not Cleaning Up

**Symptom:** After FSM stops, Gazebo or other processes still running

**Solution:**

1. **Manual cleanup:**
   ```bash
   # Kill Gazebo
   pkill -9 -f 'gz sim'
   pkill -9 gzserver
   pkill -9 gzclient
   
   # Kill ROS processes
   killall -9 ros2
   
   # Check for remaining processes
   ps aux | grep ros2
   ```

2. **Verify cleanup handlers:**
   - FSM should call `stop_all(ctx)` on exit
   - Check for Python exceptions during shutdown

3. **Use FSM cleanup directly:**
   ```bash
   # If FSM still running, send Ctrl+C
   # Cleanup handlers should trigger automatically
   ```

---

## Additional Resources

### Directory Structure

```
task_planner_fsm/
├── task_planner_fsm/              # Main Python package
│   ├── fsm_node.py               # ROS2 node entry point
│   ├── machine.py                # FSM engine
│   ├── state.py                  # Base State class
│   ├── states/                   # State implementations
│   │   ├── initialization.py
│   │   ├── create_map.py
│   │   ├── object_id.py
│   │   ├── geometry_reconstruction.py
│   │   ├── compute_wall_points.py
│   │   ├── wall_target_selection.py
│   │   ├── navigate.py
│   │   ├── arm_unfolding.py
│   │   ├── arm_folding.py
│   │   ├── scan_wall.py
│   │   ├── areas_of_interest.py
│   │   ├── wall_discretization.py
│   │   ├── base_placement.py
│   │   ├── exhaustive_scan.py
│   │   ├── home_position.py
│   │   ├── finished.py
│   │   ├── error.py
│   │   └── proc_utils.py         # Process management
│   ├── mock_server.py            # Mock service provider
│   └── goal_status_listener.py    # (unused, for future)
├── scripts/                        # Entry point scripts
│   ├── fsm_node
│   ├── mock_server
│   └── goal_status_listener
├── task_planner_fsm_rviz_panel/   # RViz plugin
│   ├── src/
│   ├── include/
│   ├── CMakeLists.txt
│   └── package.xml
├── package.xml
├── setup.py
└── setup.cfg
```

### Key Topics

**Published by FSM:**
- `/fsm/current_state` - String (current state name, 1 Hz)
- `/fsm/transition` - String (JSON transition log, on transitions)
- `/gpr/trigger` - UInt32 (GPR distance trigger, during a ScanWall sweep — see below)

**Subscribed by FSM:**
- `/start_flag` - Bool (FSM start trigger)
- `/rtabmap/odom` - Odometry (robot pose)
- `/joint_states` - JointState (arm/column positions)
- `/execution_status` - Bool (arm movement done)
- `/arm/execution_status` - Bool (alternative arm status)
- `/planner/goal_failed` - Bool (planner failure signal)
- `/map_done` - Bool (map creation complete)

### GPR Distance Triggers (ScanWall)

The GPR's own encoder wheel has to stay in contact with the wall to clock the
probe, which is hard to guarantee even at the right standoff, so a fake encoder
takes its place and the FSM decides when it pulses. During each segment sweep
`ScanWall` samples the sensor-plate (end-effector) pose in the `map` frame and
fires one trigger per fixed distance of plate travel along the wall. The start
of the sweep is itself a trigger position (d = 0), so a segment of length L
yields `floor(L / spacing) + 1` triggers.

The same sampler serves both sweep routes, with two differences.

**When they start.** In arm-sweep mode (`sweep_use_arm`, the default) they are
armed by `wall_sweep_executor`'s `sweep` feedback — the same signal that opens
the GPR line — so the lead-in traverse to the partition start, which slides the
plate along the wall without scanning it, produces none. On the base-driven path
they start with the base motion.

**What they measure against** (`gpr_trigger_reference_frame`). An arm sweep runs
with the base parked, so the plate is measured against `arm_base`: pure forward
kinematics off the joint states, with no odometry or localisation in the number
at all. A base-driven sweep moves the arm base itself, so only a world frame
sees the travel, and it falls back to `map`. The default follows the route; set
the parameter to override it. It matters — with a 1 m sweep and a deliberately
corrupted `map -> arm_base` estimate:

| localisation error | measured in `arm_base` | measured in `map` |
|---|---|---|
| none | 201 triggers | 201 triggers |
| 2 cm drift along the wall | 201 | 204 |
| 3 cm relocalisation jump | 201 | 207 |
| 8 cm jump (over `gpr_trigger_max_jump_m`) | 201 | 200, plus a re-anchor warning |

Zero-mean jitter cancels in either frame (the projection is signed), so it is
drift and jumps that the frame choice protects against. Setting it to `arm_base`
for a *base-driven* sweep would measure almost no travel at all, since the arm
base moves with the plate.

The trigger is currently a stand-in for the hardware: a `std_msgs/UInt32` on
`/gpr/trigger` carrying the trigger index within the segment, plus a log line
(every 20th at info, the rest at debug). Watch it live with:

```bash
ros2 topic echo /gpr/trigger
ros2 topic hz /gpr/trigger      # ~10 Hz at 0.5 cm spacing and 0.05 m/s
```

Tuning knobs (ROS parameters on `robot_fsm_node`, e.g.
`--ros-args -p gpr_trigger_distance_m:=0.01`):

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `gpr_trigger_distance_m` | `0.005` | Plate travel between triggers (0.5 cm) |
| `gpr_trigger_enabled` | `true` | Emit triggers at all (independent of `gpr_enabled`) |
| `gpr_trigger_topic` | `/gpr/trigger` | Topic the triggers are published on |
| `gpr_trigger_reference_frame` | *(route default)* | Frame the plate travel is measured in: `arm_base` for an arm sweep, `map` for a base-driven one |
| `gpr_trigger_rate_hz` | `50.0` | Plate-pose sampling rate; keep well above `sweep_speed / spacing` |
| `gpr_trigger_min_step_m` | `0.0005` | Per-sample deadband rejecting TF jitter |
| `gpr_trigger_max_jump_m` | `0.05` | Per-sample cap; a bigger jump re-anchors instead of firing a burst |
| `gpr_trigger_log_every` | `20` | Log every Nth trigger at info (`0` = none) |

Counters restart at each sweep, which is also each GPR line scan (with
`nest_lines_in_partition` on, once per height at a partition). Travel is
measured as the signed projection onto the segment direction, so plate motion
perpendicular to the wall (the arm's approach, force-mode press) does not clock
the probe. `gpr_trigger_rate_hz` is checked against whichever speed knob governs
the active route — `sweep_speed_mps` for the arm sweep, `sweep_crawl_speed` or
`sweep_speed_limit` for the base one — and warns when sampling is too coarse for
the spacing.

### Integration Packages

- **navi_wall**: Navigation and mapping
  - Launch files: `global_exploration.launch.py`, `move_robot.launch.py`
  - Topics: `/rtabmap/odom`, `/map`
  - Actions: `/navigate_to_pose`

- **arm_control**: Manipulator control
  - Services: `/compute_wall_discretization`, `/compute_optimal_base`, `/arm/send_position`, `/arm/script_command`
  - Topics: `/joint_states`, `/execution_status`
  - Nodes: `wall_discretization_node`, `optimal_base_service`, `script_command_service_node`

---

**Package Maintainer**: [Your contact information]  
**License**: [License type]  
**ROS2 Version**: Humble or later  
**Last Updated**: March 2026  
**Status**: Under active development - Real robot testing pending
