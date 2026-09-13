"""Keep the suite's ROS traffic off whatever domain the robot is using.

Several tests build real rclpy nodes, and they reuse the production node names
(``wbc_sweep_controller``, ``scan_wall_wbc_test``). On the robot's domain those
nodes join the live graph: their log lines land in the running stack's
``/rosout`` indistinguishable from the robot's own — a test run during a Gazebo
session produced a burst of plausible "Sweep succeeded" messages that cost real
time to trace back. Worse, a test node publishing a command topic can reach a
real controller.

The override is unconditional. ``setdefault`` would be useless: this workspace
exports ``ROS_DOMAIN_ID=1`` from ``.bashrc``, so the variable is always already
set to exactly the domain we need to escape. 77 is chosen to sit OUTSIDE the
[1, 19] range the navi-wall launch files enforce, so it can never name a domain
the robot is on.

Set here rather than in a fixture so it is in place before any test calls
``rclpy.init()`` — that is when the DDS participant reads it.
"""

import os

os.environ["ROS_DOMAIN_ID"] = os.environ.get("TASK_PLANNER_TEST_DOMAIN_ID", "77")
