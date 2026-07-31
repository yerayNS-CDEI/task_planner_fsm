"""Whole-body control for the wall sweep.

Replaces the decomposed ScanWall sweep (Nav2 position goal + force-mode press +
wall_parallel_controller, coordinated by flags and sleeps) with a single velocity
control law that resolves the sweep across the base (3 planar DOF, presented by
sim_controller at the turret axis) and the UR10e (6 DOF) together. Nine actuated
DOF for a six-dimensional task leaves a three-dimensional null space, used here
for posture. The lift column is NOT in this loop: it is position-controlled,
~7 s bandwidth and non-backdrivable, so it stays an outer-loop, per-row height
selector exactly as today.

Design, math and the redundancy-resolution background live in
``docs/scanwall_control_recommendation.tex``.

Module map:
  ``kinematics``  URDF chain -> FK and the whole-body Jacobian (base | arm).
  ``base_model``  sim_controller's diff-drive + turret actuator limits as QP rows.
  ``surface``     the six plate ranges -> the sensed surface frame.
  ``qp``          the velocity QP that turns a task twist into joint commands.
  ``sweep_node``  the ROS node that closes the loop during one segment sweep.

Everything except ``sweep_node`` is pure numpy/scipy and unit-testable without
ROS (see ``test/test_wbc_*.py``).
"""
