# task_planner_fsm_rviz_panel

RViz custom panel plugin that subscribes to:
- `/fsm/current_state` (`std_msgs/String`)
- `/fsm/transition` (`std_msgs/String`, JSON payload with `from`, `to`, `reason`)

## Build

From workspace root:

```bash
cd /home/$USER/ros2_ws
colcon build --base-paths src/task_planner_fsm/task_planner_fsm_rviz_panel
source install/setup.bash
```

## Use In RViz

1. Run your FSM node:

```bash
ros2 run task_planner_fsm fsm_node --sim true
```

2. Start RViz:

```bash
rviz2
```

3. Add panel:
- `Panels` -> `Add New Panel...`
- Select `task_planner_fsm_rviz_panel/FsmPanel`

The panel shows:
- current state
- last transition
- transition log

You can also change topic names from the panel and click `Apply Topics`.
