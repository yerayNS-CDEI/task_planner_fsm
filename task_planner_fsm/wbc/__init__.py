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
  ``admittance``  the normal axis during a GPR press: contact state + force loop,
                  standing in for UR force_mode, which cannot run alongside a
                  streaming controller.
  ``avoidance``   local costmap -> distance field -> barrier rows for the QP.
  ``qp``          the velocity QP that turns a task twist into joint commands.
  ``streaming``   how the arm is commanded (servoj setpoints or speedj), and
                  what "stop" means for each.
  ``hardware``    the real UR's speed scaling and safety state; inert in sim.
  ``controller_switch``  claiming the arm's streaming controller and giving it back.
  ``sweep_node``  the ROS node that closes the loop during one segment sweep.

Everything except ``sweep_node``, ``streaming``, ``hardware`` and
``controller_switch`` is pure numpy/scipy and unit-testable without ROS (see
``test/test_wbc_*.py``).
"""
