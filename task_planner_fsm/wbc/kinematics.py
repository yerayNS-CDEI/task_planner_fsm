r"""Whole-body kinematics: URDF serial chain FK/Jacobian + the base block.

The whole-body QP needs the 6x9 Jacobian mapping the stacked velocity

    u = [ v_x, v_y, w_z | qdot_1 ... qdot_6 ]
          \__ base __/    \___ UR10e ____/

to the end-effector (sensor plate) twist in world-aligned axes. This module
builds it from two halves:

  * the ARM half, from the URDF itself: a minimal parse of the ``arm_base ->
    plate`` chain, its forward kinematics, and the geometric Jacobian of that
    chain (``SerialChain``);
  * the BASE half, in closed form: the 3-DOF holonomic planar block
    ``base_jacobian_block``. sim_controller feedback-linearizes the diff-drive
    chassis at the TURRET AXIS and presents a holonomic (vx, vy, wz) Twist
    interface there, so the block is exact — the nonholonomy shows up only as
    actuator limits, which live in ``base_model``.

    Mind which point that is: the turret axis is 0.167 m ahead of the wheel axle
    (``base_link -> turret_link``), and the arm is NOT mounted on it. The column
    hangs off the turret 0.2905 m further forward (``turret_joint ->
    column_joint``), and the arm base sits 0.73 m up the column plus its travel.
    That whole assembly is rigid with the turret while the column is held, so a
    turret yaw sweeps the end effector through the lever arm from the TURRET
    AXIS, not from the arm mount. Nothing here hard-codes those offsets: the
    lever arm is measured every cycle as ``p_ee - p_turret_axis`` from TF, so the
    column's height and forward offset are carried automatically.

The column is deliberately absent: it would add a ``[0,0,1,0,0,0]`` column, but
it is position-controlled at ~7 s bandwidth and non-backdrivable, so it stays an
outer-loop per-row height selector and is held fixed here (its current height is
simply part of the measured ``map -> arm_base`` transform).

Pure numpy: no ROS, no Pinocchio, no KDL. The chain parser understands the
subset of URDF the robot actually uses (fixed / revolute / continuous /
prismatic joints with ``origin`` and ``axis``), which keeps the whole loop
dependency-free and unit-testable against numerical differentiation.
"""

import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """URDF ``rpy`` (fixed-axis X-Y-Z, applied in that order) to a rotation matrix."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def axis_angle_to_matrix(axis, angle):
    """Rodrigues' formula for a rotation of ``angle`` about a unit ``axis``."""
    k = np.asarray(axis, dtype=float)
    n = np.linalg.norm(k)
    if n < 1e-12:
        return np.eye(3)
    k = k / n
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def rotation_error(R_current, R_desired):
    """Angular error vector taking ``R_current`` to ``R_desired`` (world axes).

    The axis-angle log of ``R_desired * R_current^T``: its direction is the
    rotation axis and its norm the angle, so ``k * rotation_error(...)`` is a
    proportional angular-velocity command. Used for the plate's parallelism and
    roll tasks.
    """
    R = np.asarray(R_desired, dtype=float) @ np.asarray(R_current, dtype=float).T
    cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = float(np.arccos(cos_theta))
    if theta < 1e-9:
        return np.zeros(3)
    if theta > np.pi - 1e-6:
        # Near pi the skew part vanishes; recover the axis from R + I instead.
        w, V = np.linalg.eigh(R + np.eye(3))
        axis = V[:, int(np.argmax(w))]
        return theta * axis / np.linalg.norm(axis)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return theta * axis / (2.0 * np.sin(theta))


class Joint:
    """One URDF joint on the chain, in parent-link coordinates."""

    def __init__(self, name, jtype, origin_xyz, origin_rpy, axis, lower, upper, velocity):
        self.name = name
        self.type = jtype
        self.origin_xyz = np.asarray(origin_xyz, dtype=float)
        self.origin_rot = rpy_to_matrix(*origin_rpy)
        self.axis = np.asarray(axis, dtype=float)
        self.lower = lower
        self.upper = upper
        self.velocity = velocity

    @property
    def actuated(self):
        return self.type in ("revolute", "continuous", "prismatic")


class SerialChain:
    """A ``root_link -> tip_link`` chain lifted out of a URDF string.

    ``fk`` and ``jacobian`` take only the ACTUATED joint values, in chain order
    (``joint_names``); fixed joints in between are folded into the transforms.
    """

    def __init__(self, joints, root_link, tip_link):
        self.joints = list(joints)
        self.root_link = root_link
        self.tip_link = tip_link
        self.joint_names = [j.name for j in self.joints if j.actuated]

    @property
    def n_joints(self):
        return len(self.joint_names)

    # ------------------------------------------------------------------
    # Parsing
    # ------------------------------------------------------------------
    @classmethod
    def from_urdf(cls, urdf_xml, root_link, tip_link):
        """Build the chain from ``root_link`` down to ``tip_link``.

        Deliberately a minimal parser (``xml.etree``) rather than
        ``urdf_parser_py``: we need four fields per joint, and the robot's URDF
        carries a lot of Gazebo/sensor material a full parser would have to
        understand. Raises ``ValueError`` when either link is unknown or they
        are not on one ancestor path.
        """
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for joint_el in root.findall("joint"):
            child = joint_el.find("child")
            parent = joint_el.find("parent")
            if child is None or parent is None:
                continue
            origin = joint_el.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = _floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
            axis_el = joint_el.find("axis")
            axis = _floats(axis_el.get("xyz") if axis_el is not None else None, (1.0, 0.0, 0.0))
            limit = joint_el.find("limit")
            lower = float(limit.get("lower", -np.pi)) if limit is not None else -np.pi
            upper = float(limit.get("upper", np.pi)) if limit is not None else np.pi
            vmax = float(limit.get("velocity", 0.0)) if limit is not None else 0.0
            by_child[child.get("link")] = (
                parent.get("link"),
                Joint(joint_el.get("name"), joint_el.get("type", "fixed"),
                      xyz, rpy, axis, lower, upper, vmax),
            )

        # Walk up from the tip until the root is reached, then reverse.
        chain, link = [], tip_link
        while link != root_link:
            entry = by_child.get(link)
            if entry is None:
                raise ValueError(
                    f"link '{link}' has no parent joint in the URDF; "
                    f"'{tip_link}' is not a descendant of '{root_link}'"
                )
            parent_link, joint = entry
            chain.append(joint)
            link = parent_link
        chain.reverse()
        return cls(chain, root_link, tip_link)

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    def _frames(self, q):
        """Per-actuated-joint (axis, origin) in root coordinates, plus the tip pose.

        One forward pass: everything ``fk`` and ``jacobian`` need, so callers
        that want both pay for one traversal.
        """
        R = np.eye(3)
        p = np.zeros(3)
        axes, origins, types = [], [], []
        k = 0
        for joint in self.joints:
            p = p + R @ joint.origin_xyz
            R = R @ joint.origin_rot
            if not joint.actuated:
                continue
            axis_root = R @ joint.axis
            axes.append(axis_root)
            origins.append(p.copy())
            types.append(joint.type)
            if joint.type == "prismatic":
                p = p + axis_root * float(q[k])
            else:
                R = R @ axis_angle_to_matrix(joint.axis, float(q[k]))
            k += 1
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        return T, axes, origins, types

    def fk(self, q):
        """4x4 pose of ``tip_link`` in ``root_link`` for actuated joint values ``q``."""
        q = np.asarray(q, dtype=float)
        if q.shape[0] != self.n_joints:
            raise ValueError(f"expected {self.n_joints} joint values, got {q.shape[0]}")
        T, _, _, _ = self._frames(q)
        return T

    def jacobian(self, q):
        """6xN geometric Jacobian in ROOT-frame axes, reference point = the tip.

        Rows are ``[vx, vy, vz, wx, wy, wz]``: revolute joint *i* contributes
        ``z_i x (p_tip - p_i)`` to the linear part and ``z_i`` to the angular
        part; a prismatic joint contributes ``z_i`` and zero. Same convention as
        Pinocchio's ``LOCAL_WORLD_ALIGNED`` frame Jacobian, so it can be swapped
        for one later without touching the QP.
        """
        q = np.asarray(q, dtype=float)
        if q.shape[0] != self.n_joints:
            raise ValueError(f"expected {self.n_joints} joint values, got {q.shape[0]}")
        T, axes, origins, types = self._frames(q)
        p_tip = T[:3, 3]
        J = np.zeros((6, self.n_joints))
        for i, (axis, origin, jtype) in enumerate(zip(axes, origins, types)):
            if jtype == "prismatic":
                J[:3, i] = axis
            else:
                J[:3, i] = np.cross(axis, p_tip - origin)
                J[3:, i] = axis
        return J

    def position_limits(self):
        """(lower, upper) arrays over the actuated joints, in chain order."""
        lower, upper = [], []
        for joint in self.joints:
            if not joint.actuated:
                continue
            if joint.type == "continuous":
                lower.append(-np.inf)
                upper.append(np.inf)
            else:
                lower.append(joint.lower)
                upper.append(joint.upper)
        return np.array(lower), np.array(upper)


def _floats(text, default):
    if not text:
        return np.asarray(default, dtype=float)
    return np.array([float(v) for v in text.split()], dtype=float)


# ----------------------------------------------------------------------
# The base half of the whole-body Jacobian
# ----------------------------------------------------------------------
def base_jacobian_block(yaw_turret, d):
    """6x3 Jacobian of the base twist, task axes in WORLD, command axes in TURRET.

    ``d = p_ee - p_turret_axis`` is the lever arm from the base's rotation axis
    (the turret axis, which is the point sim_controller feedback-linearizes) to
    the end effector, in world coordinates. ``yaw_turret`` is the turret frame's
    heading in the world frame. Measure ``d`` from the TURRET AXIS, not from the
    arm's mount: the column is offset ~0.29 m forward of that axis and the arm
    sits on top of it, so the two differ by a lever arm the size of the standoff
    being controlled. ``turret_footprint`` is the ground projection of the turret
    axis (its broadcaster tracks ``turret_link``), which makes it the right frame
    to take ``p_turret_axis`` from; its vertical offset is harmless, since
    ``z_hat x d`` discards the vertical component of ``d``.

    The command variables are the Twist the base actually accepts — (vx, vy) in
    the TURRET frame, because sim_controller runs ``cmd_type: relative`` — while
    the task lives in world axes, hence the rotation in the first two columns::

        [ cos y   -sin y   -d_y ]
        [ sin y    cos y    d_x ]
        [   0        0       0  ]
        [   0        0       0  ]
        [   0        0       0  ]
        [   0        0       1  ]

    The ``(-d_y, d_x)`` column is the ``z_hat x d`` term: turning the turret
    sweeps the end effector through that lever arm. Keeping the command variables
    body-fixed means the actuator limits in ``base_model`` are plain rows on the
    same variables, with no frame juggling inside the QP.
    """
    d = np.asarray(d, dtype=float)
    c, s = np.cos(yaw_turret), np.sin(yaw_turret)
    J = np.zeros((6, 3))
    J[0, 0], J[1, 0] = c, s
    J[0, 1], J[1, 1] = -s, c
    J[0, 2], J[1, 2] = -d[1], d[0]
    J[5, 2] = 1.0
    return J


def whole_body_jacobian(chain, q_arm, T_world_armbase, yaw_turret, p_turret_world):
    """Assemble the 6x(3+N) whole-body Jacobian in world-aligned axes.

    ``T_world_armbase`` is the measured pose of the arm's mount (from TF, so it
    already carries the turret angle, the column's forward offset and its live
    height), and ``p_turret_world`` is the TURRET AXIS — the point the base
    rotates about, which the arm is mounted well forward of and above. Taking
    both from TF is what keeps the two apart without hard-coding either offset.
    Returns ``(J, T_world_tip)``.
    """
    J_arm_local = chain.jacobian(q_arm)
    R = np.asarray(T_world_armbase, dtype=float)[:3, :3]
    J_arm = np.vstack((R @ J_arm_local[:3, :], R @ J_arm_local[3:, :]))

    T_world_tip = np.asarray(T_world_armbase, dtype=float) @ chain.fk(q_arm)
    d = T_world_tip[:3, 3] - np.asarray(p_turret_world, dtype=float)
    J_base = base_jacobian_block(yaw_turret, d)
    return np.hstack((J_base, J_arm)), T_world_tip
