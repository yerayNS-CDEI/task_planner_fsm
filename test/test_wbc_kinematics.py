"""The whole-body Jacobian must agree with finite differences of the FK.

That is the only check that catches the bookkeeping this code exists to get
right: the arm Jacobian rotated into world axes, and the base block's
``z_hat x d`` coupling (turning the base sweeps the plate through the arm's lever
arm). Everything here is pure numpy — no ROS, no robot.
"""

import numpy as np
import pytest

from task_planner_fsm.wbc.kinematics import (
    soft_deadband,
    shift_jacobian_point,
    SerialChain,
    base_jacobian_block,
    rotation_error,
    rpy_to_matrix,
    whole_body_jacobian,
)

# A three-joint arm with a fixed tool offset: shoulder about Z, elbow about Y,
# wrist about Y again, then a fixed frame with a rotated tool. Enough structure
# that a wrong axis or a dropped fixed transform shows up immediately.
URDF = """<?xml version="1.0"?>
<robot name="test_arm">
  <link name="arm_base_link"/>
  <link name="l1"/><link name="l2"/><link name="l3"/><link name="tool"/>
  <joint name="j1" type="revolute">
    <parent link="arm_base_link"/><child link="l1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" velocity="2.0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0.1 0 0.05" rpy="0 0.3 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" velocity="2.0"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" velocity="2.0"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="l3"/><child link="tool"/>
    <origin xyz="0.3 0 0.05" rpy="0 1.5707963 0"/>
  </joint>
</robot>
"""

Q = np.array([0.4, -0.7, 1.1])


def _chain():
    return SerialChain.from_urdf(URDF, "arm_base_link", "tool")


def _twist_between(T_a, T_b, eps):
    """Finite-difference twist taking pose ``T_a`` to ``T_b`` over ``eps``."""
    linear = (T_b[:3, 3] - T_a[:3, 3]) / eps
    angular = rotation_error(T_a[:3, :3], T_b[:3, :3]) / eps
    return np.concatenate((linear, angular))


def test_chain_finds_actuated_joints_and_skips_fixed_ones():
    chain = _chain()
    assert chain.joint_names == ["j1", "j2", "j3"]
    assert chain.n_joints == 3


def test_fk_matches_a_hand_built_transform_at_zero():
    chain = _chain()
    T = chain.fk(np.zeros(3))
    # Rotations at zero: only the j2 origin's 0.3 rad pitch and the tool's pi/2.
    expected_R = rpy_to_matrix(0, 0.3, 0) @ rpy_to_matrix(0, 1.5707963, 0)
    np.testing.assert_allclose(T[:3, :3], expected_R, atol=1e-9)
    # Position: 0.2 up, then the j2 origin, then the pitched 0.4 and 0.3+0.05 legs.
    p = np.array([0, 0, 0.2]) + np.array([0.1, 0, 0.05])
    R2 = rpy_to_matrix(0, 0.3, 0)
    p = p + R2 @ np.array([0.4, 0, 0]) + R2 @ np.array([0.3, 0, 0.05])
    np.testing.assert_allclose(T[:3, 3], p, atol=1e-9)


def test_arm_jacobian_matches_numerical_differentiation():
    chain = _chain()
    J = chain.jacobian(Q)
    eps = 1e-6
    for i in range(chain.n_joints):
        dq = np.zeros(chain.n_joints)
        dq[i] = eps
        numeric = _twist_between(chain.fk(Q), chain.fk(Q + dq), eps)
        np.testing.assert_allclose(J[:, i], numeric, atol=1e-5)


def test_base_block_turning_sweeps_the_lever_arm():
    d = np.array([0.8, -0.3, 1.2])
    J = base_jacobian_block(yaw_turret=0.0, d=d)
    twist = J @ np.array([0.0, 0.0, 1.0])         # 1 rad/s about the turret axis
    np.testing.assert_allclose(twist[:3], np.cross([0, 0, 1], d), atol=1e-12)
    np.testing.assert_allclose(twist[3:], [0, 0, 1], atol=1e-12)


def test_base_block_translation_is_expressed_in_the_turret_frame():
    yaw = 0.7
    J = base_jacobian_block(yaw_turret=yaw, d=np.array([0.5, 0.2, 0.0]))
    # +vx in the turret frame is +x rotated by yaw in the world frame.
    np.testing.assert_allclose(J[:3, 0], [np.cos(yaw), np.sin(yaw), 0.0], atol=1e-12)
    np.testing.assert_allclose(J[:3, 1], [-np.sin(yaw), np.cos(yaw), 0.0], atol=1e-12)


def _world_tip(chain, base_xy, yaw, q, T_turret_armbase):
    """Tip pose in the world for a base at (xy, yaw) — the FK the QP differentiates."""
    T_world_turret = np.eye(4)
    T_world_turret[:3, :3] = rpy_to_matrix(0.0, 0.0, yaw)
    T_world_turret[:3, 3] = [base_xy[0], base_xy[1], 0.0]
    return T_world_turret @ T_turret_armbase @ chain.fk(q)


def test_whole_body_jacobian_matches_numerical_differentiation():
    chain = _chain()
    # The arm sits on the column, offset from the turret axis and rotated —
    # exactly the mount the real robot has, so the coupling terms are exercised.
    T_turret_armbase = np.eye(4)
    T_turret_armbase[:3, :3] = rpy_to_matrix(0.0, 0.0, -2.3562)
    T_turret_armbase[:3, 3] = [0.05, 0.0, 0.73]

    base_xy, yaw = np.array([1.5, -0.4]), 0.6
    T_world_armbase = np.eye(4)
    T_world_armbase[:3, :3] = rpy_to_matrix(0.0, 0.0, yaw)
    T_world_armbase[:3, 3] = [base_xy[0], base_xy[1], 0.0]
    T_world_armbase = T_world_armbase @ T_turret_armbase

    J, T_tip = whole_body_jacobian(
        chain, Q, T_world_armbase, yaw, [base_xy[0], base_xy[1], 0.0])
    assert J.shape == (6, 6)   # 3 base + 3 arm

    eps = 1e-6
    T0 = _world_tip(chain, base_xy, yaw, Q, T_turret_armbase)
    np.testing.assert_allclose(T_tip, T0, atol=1e-9)

    # Base columns: vx and vy are TURRET-frame, so perturb along the rotated axes.
    for i, body_axis in enumerate(([1.0, 0.0], [0.0, 1.0])):
        world_step = rpy_to_matrix(0, 0, yaw)[:2, :2] @ np.array(body_axis) * eps
        numeric = _twist_between(
            T0, _world_tip(chain, base_xy + world_step, yaw, Q, T_turret_armbase), eps)
        np.testing.assert_allclose(J[:, i], numeric, atol=1e-5)

    numeric = _twist_between(
        T0, _world_tip(chain, base_xy, yaw + eps, Q, T_turret_armbase), eps)
    np.testing.assert_allclose(J[:, 2], numeric, atol=1e-5)

    for i in range(chain.n_joints):
        dq = np.zeros(chain.n_joints)
        dq[i] = eps
        numeric = _twist_between(
            T0, _world_tip(chain, base_xy, yaw, Q + dq, T_turret_armbase), eps)
        np.testing.assert_allclose(J[:, 3 + i], numeric, atol=1e-5)


def test_the_lever_arm_starts_at_the_turret_axis_not_the_arm_mount():
    """The arm rides ~0.29 m forward of the axis the base rotates about.

    ``turret_joint`` puts the turret axis 0.167 m ahead of the wheel axle,
    ``column_joint`` puts the column 0.2905 m further forward on the turret, and
    the arm sits on the column. Measuring the base's lever arm from the arm mount
    instead of the turret axis drops that 0.29 m — an error the size of the
    standoff being controlled — and it only shows up when the base is yawing.
    """
    chain = _chain()
    column_offset = np.array([0.2905, 0.0, 0.0])
    T_turret_armbase = np.eye(4)
    T_turret_armbase[:3, 3] = column_offset + np.array([0.0, 0.0, 0.73])

    yaw = 0.0
    p_turret = np.array([2.0, 1.0, 0.0])
    T_world_armbase = np.eye(4)
    T_world_armbase[:3, 3] = p_turret
    T_world_armbase = T_world_armbase @ T_turret_armbase

    J_correct, T_tip = whole_body_jacobian(chain, Q, T_world_armbase, yaw, p_turret)
    # The same call made with the (wrong) arm-mount reference point.
    J_wrong, _ = whole_body_jacobian(
        chain, Q, T_world_armbase, yaw, T_world_armbase[:3, 3])

    # Finite differences about the TURRET axis are what the robot actually does.
    eps = 1e-6
    R = rpy_to_matrix(0.0, 0.0, eps)
    p_tip_rotated = p_turret + R @ (T_tip[:3, 3] - p_turret)
    numeric_linear = (p_tip_rotated - T_tip[:3, 3]) / eps
    np.testing.assert_allclose(J_correct[:3, 2], numeric_linear, atol=1e-5)

    # ... and the arm-mount version is off by exactly the column's lever arm.
    np.testing.assert_allclose(
        J_correct[:3, 2] - J_wrong[:3, 2],
        np.cross([0.0, 0.0, 1.0], T_world_armbase[:3, 3] - p_turret),
        atol=1e-12)
    assert abs((J_correct[:3, 2] - J_wrong[:3, 2])[1]) == pytest.approx(0.2905, abs=1e-9)


def test_rotation_error_is_the_axis_angle_of_the_correction():
    R_current = rpy_to_matrix(0.1, -0.2, 0.3)
    R_desired = rpy_to_matrix(0.1, -0.2, 0.3) @ rpy_to_matrix(0.0, 0.0, 0.25)
    error = rotation_error(R_current, R_desired)
    assert np.linalg.norm(error) == pytest.approx(0.25, abs=1e-9)
    np.testing.assert_allclose(rotation_error(R_current, R_current), np.zeros(3), atol=1e-12)


def test_unreachable_tip_link_is_reported():
    with pytest.raises(ValueError):
        SerialChain.from_urdf(URDF, "l2", "l1")


def test_soft_deadband_is_zero_inside_and_continuous_at_the_edge():
    band = np.radians(1.0)
    assert np.all(soft_deadband([0.5 * band, 0.0, 0.0], band) == 0.0)
    just_over = soft_deadband([1.01 * band, 0.0, 0.0], band)
    assert 0.0 < just_over[0] < 0.02 * band          # not the full 1.01 deg
    # direction is kept, norm reduced by exactly the band
    e = np.array([0.0, 0.03, 0.04])
    out = soft_deadband(e, band)
    np.testing.assert_allclose(out / np.linalg.norm(out), e / np.linalg.norm(e))
    assert np.linalg.norm(out) == pytest.approx(0.05 - band)
    np.testing.assert_allclose(soft_deadband(e, 0.0), e)


def test_shifting_the_jacobian_point_adds_the_lever_of_the_angular_rows():
    """A yaw about z at a point 8 cm along +x moves that point along +y at
    0.08 * w; the angular rows are untouched."""
    J = np.zeros((6, 2))
    J[5, 0] = 1.0            # column 0: rotate about z
    J[0, 1] = 1.0            # column 1: translate along x
    out = shift_jacobian_point(J, [0.08, 0.0, 0.0])
    np.testing.assert_allclose(out[:3, 0], [0.0, 0.08, 0.0])
    np.testing.assert_allclose(out[:3, 1], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(out[3:], J[3:])
