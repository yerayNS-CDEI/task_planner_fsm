# task_planner_fsm

## Check the PR locally (BT path)

From the repository root:

```bash
cd <path-to-your-clone>/task_planner_fsm
```

Build and source:

```bash
colcon build --packages-select task_planner_fsm
source install/setup.bash
```

Run unit tests:

```bash
python -m pytest -q
```

Run the BT node:

```bash
ros2 run task_planner_fsm bt_node --sim true
```

In another terminal (after `source install/setup.bash`), inspect topics:

```bash
ros2 topic echo /fsm/current_state
ros2 topic echo /fsm/transition
```

Trigger start flag once:

```bash
ros2 topic pub --once /start_flag std_msgs/msg/Bool "{data: true}"
```

Expected checks:
- `bt_node` starts without crashing.
- `/fsm/current_state` publishes BT progress.
- `/fsm/transition` publishes transitions.
