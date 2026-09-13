"""End-to-end control-law test with a simulated robot, no Gazebo required.

A tiny kinematic simulator stands in for the plant: the base integrates the
Twist it is given (in the turret frame, as sim_controller would), the arm
integrates its joint velocities, and the six ranges are recomputed from the true
plate pose against a known wall plane. Running the node's own ``_control_step``
against it checks the thing that unit tests cannot — that the loop CLOSES: the
plate converges to the standoff, holds the row height, stays parallel, and the
sweep terminates on arc length.
"""

from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from std_msgs.msg import Float64  # noqa: E402

from task_planner_fsm.wbc.kinematics import SerialChain, rpy_to_matrix  # noqa: E402
from task_planner_fsm.wbc.streaming import POSITION, VELOCITY           # noqa: E402
from task_planner_fsm.wbc.surface import SENSOR_XY                      # noqa: E402
from task_planner_fsm.wbc.sweep_node import WholeBodySweepNode          # noqa: E402

# A 6-DOF arm, deliberately not the UR10e: the control law must not depend on
# the arm's particular geometry, only on its Jacobian.
URDF = """<?xml version="1.0"?>
<robot name="sim_arm">
  <link name="arm_base_link"/>
  <link name="l1"/><link name="l2"/><link name="l3"/>
  <link name="l4"/><link name="l5"/><link name="l6"/>
  <joint name="j1" type="revolute"><parent link="arm_base_link"/><child link="l1"/>
    <origin xyz="0 0 0.18" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.1" upper="3.1" velocity="2"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0" rpy="0 1.5707963 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.1" upper="3.1" velocity="2"/></joint>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/>
    <origin xyz="0.6 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.1" upper="3.1" velocity="2"/></joint>
  <joint name="j4" type="revolute"><parent link="l3"/><child link="l4"/>
    <origin xyz="0.55 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.1" upper="3.1" velocity="2"/></joint>
  <joint name="j5" type="revolute"><parent link="l4"/><child link="l5"/>
    <origin xyz="0.12 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.1" upper="3.1" velocity="2"/></joint>
  <joint name="j6" type="revolute"><parent link="l5"/><child link="l6"/>
    <origin xyz="0.10 0 0" rpy="0 1.5707963 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.1" upper="3.1" velocity="2"/></joint>
</robot>
"""

ARM_JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
WALL_X = 3.0           # the wall plane, x = WALL_X, its normal pointing -x
STANDOFF = 0.20
# Plate distance at which the GPR wheel and the four corner caster bars are all
# riding the wall. The range sensors are on the plate face and everything that
# touches stands off it, so this — not zero — is what "in contact" reads.
PLATE_STANDOFF = 0.13
# Where the FSM's arm_approach leaves the plate before a sweep begins.
APPROACH_GAP = 0.20
MOUNT_HEIGHT = 0.9     # column-set arm mount height above the turret footprint


class KinematicRobot:
    """Base + arm state, integrated from the commands the node publishes."""

    def __init__(self, chain, base_xy, yaw, q_arm, dt):
        self.chain = chain
        self.base_xy = np.array(base_xy, dtype=float)
        self.yaw = float(yaw)
        self.q = np.array(q_arm, dtype=float)
        self.dt = dt

    def mount(self):
        T = np.eye(4)
        T[:3, :3] = rpy_to_matrix(0.0, 0.0, self.yaw)
        T[:3, 3] = [self.base_xy[0], self.base_xy[1], MOUNT_HEIGHT]
        return T

    def tip(self):
        return self.mount() @ self.chain.fk(self.q)

    def step(self, twist, arm, mode=POSITION):
        """Integrate one cycle of whatever the node published.

        ``arm`` is joint velocities in velocity mode and a joint-position
        setpoint in position mode, so the plant has to know which interface it
        is being driven through. Both are modelled as perfect trackers within
        the cycle, which is the same fidelity the velocity model always had.
        """
        # sim_controller's interface: the twist is in the turret frame.
        world = rpy_to_matrix(0.0, 0.0, self.yaw)[:2, :2] @ np.array(twist[:2])
        self.base_xy += world * self.dt
        self.yaw += twist[2] * self.dt
        if arm is None:
            return
        if mode == POSITION:
            self.q = np.asarray(arm, dtype=float).copy()
        else:
            self.q += np.asarray(arm, dtype=float) * self.dt

    def plate_gap(self):
        """Distance from the plate centre to the wall, along the plate's own +Z."""
        T = self.tip()
        ray = T[:3, 2]
        if abs(ray[0]) < 1e-9:
            return float("inf")
        return float((WALL_X - T[0, 3]) / ray[0])

    def press_force(self, standoff=PLATE_STANDOFF, stiffness=2.0e4):
        """Contact force through the plate, N, positive = pressing into the wall.

        Contact does NOT begin at zero plate distance. The GPR body has length
        along the wall normal and four bars with caster wheels stand off the
        plate's corners, so everything is riding the surface while the plate
        itself is still ``standoff`` metres off it — and the plate's own range
        sensors report that number, not zero.

        Past the standoff the plate is loading four rigid bars, so ``stiffness``
        is high: 5 N is a quarter of a millimetre of squeeze, which is why the
        press gain has to be small.
        """
        return max(0.0, stiffness * (standoff - self.plate_gap()))

    def ranges(self):
        """What the six sensors see of the wall plane x = WALL_X."""
        T = self.tip()
        R, p = T[:3, :3], T[:3, 3]
        out = []
        for x, y in SENSOR_XY:
            origin = p + R @ np.array([x, y, 0.0])
            ray = R[:, 2]                       # sensors look along plate +Z
            if abs(ray[0]) < 1e-9:
                out.append(np.nan)
                continue
            out.append((WALL_X - origin[0]) / ray[0])
        return np.array(out)


def _node(seg_start, seg_end, **overrides):
    params = [
        rclpy.parameter.Parameter("seg_start", value=list(seg_start)),
        rclpy.parameter.Parameter("seg_end", value=list(seg_end)),
        rclpy.parameter.Parameter("arm_joints", value=ARM_JOINTS),
        rclpy.parameter.Parameter("standoff", value=STANDOFF),
        rclpy.parameter.Parameter("control_rate", value=50.0),
    ]
    params += [rclpy.parameter.Parameter(k, value=v) for k, v in overrides.items()]
    node = WholeBodySweepNode(parameter_overrides=params)
    node.chain = SerialChain.from_urdf(URDF, "arm_base_link", "l6")
    # The chain is injected rather than parsed from a /robot_description, so do
    # by hand the part of _on_robot_description the stream depends on.
    node.arm_stream.set_position_limits(*node.chain.position_limits())
    return node


def _wire(node, robot):
    """Point the node's state readers at the simulated robot."""
    node.joint_positions = dict(zip(ARM_JOINTS, robot.q))
    node.joint_positions["turret_joint"] = 0.0
    node.joint_stamp = 1e12                    # never stale
    node.distances = robot.ranges()
    node.distance_stamp = 1e12
    # Stands in for /force_torque_sensor_broadcaster/wrench, already sign-flipped
    # the way _on_wrench does it. Inert unless the node has a press configured.
    node.press_force = robot.press_force()
    node.wrench_stamp = 1e12
    node._mount_pose = robot.mount
    node._base_pose = lambda: (robot.yaw, np.array([robot.base_xy[0], robot.base_xy[1], 0.0]))

    def _lookup(target_frame, source_frame=None):
        """Stands in for TF, including the costmap-frame lookup the barrier makes."""
        if target_frame != node.base_frame:
            return None
        half = robot.yaw / 2.0
        return (np.array([0.0, 0.0, np.sin(half), np.cos(half)]),
                np.array([robot.base_xy[0], robot.base_xy[1], 0.0]))

    node._lookup = _lookup


def _capture(node, commands):
    """Intercept both command publishers. Returns nothing; fills ``commands``."""
    node.cmd_vel_pub.publish = lambda msg: commands.update(
        twist=[msg.linear.x, msg.linear.y, msg.angular.z])
    node.arm_stream.publisher.publish = lambda msg: commands.update(arm=list(msg.data))


class _Clock:
    """The node's clock, advanced by hand.

    The loop measures real elapsed time in two places that matter — the
    acceleration bound and the setpoint integration — so a harness that let them
    read the wall clock would be measuring how fast the machine running the test
    happens to be. Driving it by hand makes a cycle exactly one control period
    wide, which is also what ``robot.dt`` assumes.
    """

    def __init__(self, t=1000.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


def _install_clock(node):
    """Fake the node's clock, keeping the one already installed if there is one.

    Several tests call ``_run`` twice on the same node to drive the sweep and
    then the retreat. A second clock would send time backwards between them.
    """
    if isinstance(node._now, _Clock):
        return node._now
    clock = _Clock()
    node._now = clock
    return clock


def _stream(node, clock):
    """The stream ticks belonging to one solve.

    The setpoint is published by its own timer, faster than the QP behind it, so
    a "cycle" is one ``_control_step`` plus ``stream_rate / control_rate`` ticks.
    Together they advance the setpoint by exactly one control period of motion,
    which is what the plant integrates.
    """
    for _ in range(int(round(node.stream_rate / node.control_rate))):
        clock.advance(1.0 / node.stream_rate)
        node._stream_step()


def _cycle(node, robot, commands, clock):
    """One solve, its stream ticks, then the plant."""
    node._control_step()
    _stream(node, clock)
    robot.step(commands.get("twist", [0.0, 0.0, 0.0]),
               commands.get("arm"), node.arm_stream.mode)


def _run(node, robot, cycles):
    """Drive the loop, feeding every command back into the simulator."""
    commands = {}
    _capture(node, commands)
    clock = _install_clock(node)
    node.deadline = 1e12
    # What start() does: seed the integrator at the arm's current pose, so the
    # first setpoint is a no-op rather than a jump. Without it the stream has no
    # history to integrate from and holds instead of moving.
    node.arm_stream.reset(robot.q)

    for _ in range(cycles):
        if node.status != "running":
            break
        _wire(node, robot)
        commands.clear()
        _cycle(node, robot, commands, clock)
    return robot


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _start_state(chain, gap=0.35):
    """An arm reaching at the wall from ``gap`` metres, plate slightly yawed off it.

    j2 pitches the arm out along +x (the wall is the plane x = WALL_X), j6 rolls
    the plate so its +Y is up, and j5 leaves a few degrees of tilt for the
    parallelism task to take out. The base is placed so the tip starts exactly
    ``gap`` from the wall — further than the standoff, so the normal loop has a
    real error to close.
    """
    # Start the arm FOLDED, with ~0.2 m of extension still available. The base
    # is deliberately discouraged from moving along the wall normal — that axis
    # belongs to the arm — so an arm that starts near full stretch (this chain
    # maxes out at 1.37 m) cannot close a standoff error at all. The real FSM
    # sets the standoff with _send_arm_to_clearance before the sweep begins and
    # the normal loop only trims it, so reach in hand is the realistic case.
    q = np.array([0.0, -np.pi / 2.0, 0.9, -0.9, 0.08, np.pi / 2.0])
    reach = float(chain.fk(q)[0, 3])
    return KinematicRobot(chain, base_xy=[WALL_X - reach - gap, 0.0],
                          yaw=0.0, q_arm=q, dt=0.02)


def test_the_plate_converges_to_the_standoff_and_holds_the_row_height():
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    start_gap = WALL_X - robot.tip()[0, 3]

    _run(node, robot, cycles=300)

    gap = WALL_X - robot.tip()[0, 3]
    assert abs(start_gap - STANDOFF) > 0.05            # it really had to close a gap
    assert abs(gap - STANDOFF) < 0.05                  # ... and mostly did
    assert abs(robot.tip()[2, 3] - node.row_z) < 0.02  # row height held
    # Plate +Z ends up facing the wall (+x): parallel to within a couple of degrees.
    assert robot.tip()[:3, 2] @ np.array([1.0, 0.0, 0.0]) > np.cos(np.deg2rad(3.0))


def test_the_arm_owns_the_normal_and_the_base_owns_the_tangent():
    """The task-frame ownership the design calls for, and a regression for a bug
    that broke a live run.

    The damping weights make the base far cheaper than the arm, which is right
    for the along-wall travel and wrong for everything else: applied to the
    normal axis too, the base carried the standoff corrections and crept toward
    the wall for the whole sweep, until it sat inside Nav2's collision radius and
    the next transit could not plan. The base's motion along the normal must stay
    a small fraction of its motion along the tangent.
    """
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
    robot = _start_state(node.chain, gap=0.32)      # a standoff error to close
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    start_base = robot.base_xy.copy()
    _run(node, robot, cycles=900)

    travelled = robot.base_xy - start_base
    along_wall = abs(travelled[1])         # the sweep direction (+y)
    into_wall = abs(travelled[0])          # the wall normal (+x)

    assert along_wall > 0.15, "the base should be carrying the sweep"
    assert into_wall < 0.05, f"the base drifted {into_wall:.3f} m along the normal"
    assert into_wall < 0.2 * along_wall, "normal drift should be a small fraction of travel"
    # ... and the standoff still converged, because the ARM closed it.
    assert abs((WALL_X - robot.tip()[0, 3]) - STANDOFF) < 0.05


def test_the_sweep_travels_the_segment_and_reports_success():
    length = 0.35
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, length, 0.0))
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    start_y = robot.tip()[1, 3]

    _run(node, robot, cycles=3000)

    assert node.status == "succeeded"
    assert robot.tip()[1, 3] - start_y == pytest.approx(length, abs=0.05)


def test_the_plate_is_backed_off_the_wall_before_the_arm_is_handed_back():
    """A sweep ends with the plate centimetres off the wall, and everything
    downstream needs it further out — the FSM's Cartesian retraction has to be
    solved through an octomap containing that wall, and routinely fails. Retreat
    along the sensed normal here instead, and hand the arm over already clear."""
    length = 0.3
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, length, 0.0), retreat_standoff=0.40)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    _run(node, robot, cycles=200)          # settle onto the wall and start sweeping
    base_at_sweep_end = robot.base_xy.copy()
    _run(node, robot, cycles=4000)

    assert node.status == "succeeded"
    # The status was withheld until the plate was clear, not published at the
    # moment the sweep itself ended.
    assert WALL_X - robot.tip()[0, 3] == pytest.approx(0.40, abs=0.05)
    # ... and the retreat was the ARM's job: the base held still for it.
    assert np.linalg.norm(robot.base_xy - base_at_sweep_end) < 0.15


@pytest.mark.xfail(reason="NOT FIXED ON THIS BRANCH. _retreat_step still builds "
                          "its angular target as np.zeros(3) with no weight on "
                          "those rows, so the plate wanders ~10 deg off square "
                          "on the way out (the assertion allows 6). The "
                          "docstring below records the hardware consequence; "
                          "this is a real open defect, not stale test code.",
                   strict=True)
def test_the_retreat_keeps_the_plate_square_on_the_way_out():
    """The retreat once weighted its angular rows ZERO — the docstring claimed
    otherwise, which is how it survived — so the plate was free to rotate as it
    came off. It begins with the GPR wheel still loaded against the wall, so a
    free plate twists a pressed wheel against concrete: the crack heard on the
    first hardware run. Worst tilt roughly halved when the rows came back.
    """
    def worst_tilt(**overrides):
        node = _sweep_along_wall(**overrides)
        robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
        node.q_posture = robot.q.copy()
        node.row_z = float(robot.tip()[2, 3])
        node.deadline = 1e12
        node.phase, node.retreat_deadline = "retreat", 1e12
        commands = {}
        _capture(node, commands)
        clock = _install_clock(node)
        node.arm_stream.reset(robot.q)

        tilts = []
        for _ in range(2000):
            _wire(node, robot)
            commands.clear()
            node.surface.update(robot.ranges())
            node._retreat_step()
            if node.phase != "retreat":
                break
            # Angle between plate +Z and the wall normal; 0 is square to it.
            tilts.append(np.degrees(np.arccos(np.clip(
                robot.tip()[:3, 2] @ np.array([1.0, 0.0, 0.0]), -1.0, 1.0))))
            _stream(node, clock)
            robot.step(commands.get("twist", [0.0, 0.0, 0.0]),
                       commands.get("arm"), node.arm_stream.mode)
        assert node.phase != "retreat", "the retreat must actually finish"
        return max(tilts)

    held = worst_tilt()
    free = worst_tilt(weight_angular=0.0)      # how it shipped
    assert held < 6.0, f"plate wandered {held:.1f} deg off square"
    assert held < 0.7 * free


@pytest.mark.xfail(reason="NOT FIXED ON THIS BRANCH. _retreat_step runs no "
                          "posture task, so weight_posture has no effect during "
                          "the retreat and both arms of the comparison return "
                          "the identical error. Open defect, not stale test.",
                   strict=True)
def test_the_retreat_aims_at_the_configuration_it_has_to_hand_back():
    """A 3-DOF pull-back with the base pinned leaves three DOF of redundancy, and
    damping alone spends them by folding the elbow toward the column. The posture
    nudge is what stops that being the default.

    It is a NUDGE deliberately: measured against this fixture, weights of 0.1 and
    above stop the retreat reaching its standoff at all, because pulling toward a
    configuration near the wall fights pulling away from it.
    """
    def retreat_with(weight):
        node = _sweep_along_wall(weight_posture=weight)
        robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
        node.q_posture = robot.q.copy()
        node.row_z = float(robot.tip()[2, 3])
        node.return_joints = list(
            robot.q + np.array([0.0, 0.15, -0.20, 0.10, 0.0, 0.0]))
        node.deadline = 1e12
        node.phase, node.retreat_deadline = "retreat", 1e12
        commands = {}
        _capture(node, commands)
        clock = _install_clock(node)
        node.arm_stream.reset(robot.q)

        start = robot.q.copy()
        for _ in range(2000):
            _wire(node, robot)
            commands.clear()
            node.surface.update(robot.ranges())
            node._retreat_step()
            if node.phase != "retreat":
                break
            _stream(node, clock)
            robot.step(commands.get("twist", [0.0, 0.0, 0.0]),
                       commands.get("arm"), node.arm_stream.mode)
        assert node.phase != "retreat", "the posture pull must not stall the retreat"
        return (float(np.max(np.abs(np.asarray(node.return_joints) - robot.q))),
                float(np.max(np.abs(robot.q - start))))

    # Getting off the wall necessarily moves AWAY from a scan-pose target, so the
    # absolute error grows either way. The claim is only that it grows less, and
    # that the arm swings less far doing it.
    nudged_err, nudged_swing = retreat_with(0.02)
    drifting_err, drifting_swing = retreat_with(0.0)
    assert nudged_err < drifting_err
    assert nudged_swing < drifting_swing


def test_a_failed_sweep_still_backs_the_plate_off():
    """The retreat matters most when the sweep went wrong — that is exactly when
    the arm would otherwise be left pressed against the wall."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.0, 0.0), retreat_standoff=0.40)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    _run(node, robot, cycles=200)

    node.finish("failed", "a made-up reason")
    assert node.status == "running", "status must wait for the retreat"
    _run(node, robot, cycles=3000)

    assert node.status.startswith("failed")
    assert WALL_X - robot.tip()[0, 3] > 0.30


def test_the_arm_is_returned_to_the_configuration_the_planner_left_it_in():
    """Retreating along the normal cannot leave the arm somewhere the FSM's
    planner will accept: too little and A* dies on the wall's dilation, too far
    and wrist_3 enters the robot's own mast cylinder, and the safe window moves
    with the base's standoff. So the sweep finishes by driving the joints back to
    a configuration the planner itself produced."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 0.3, 0.0), retreat_standoff=0.40)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    target = (robot.q + np.array([0.05, 0.10, -0.15, 0.10, 0.05, -0.05])).tolist()
    node.return_joints = list(target)

    _run(node, robot, cycles=200)
    assert node.status == "running", "status must wait for retreat AND return"
    _run(node, robot, cycles=6000)

    assert node.status == "succeeded"
    worst = float(np.max(np.abs(robot.q - np.asarray(target))))
    assert worst < 0.06, f"arm did not return to the planner's pose ({worst:.3f} rad)"


def test_no_return_target_just_ends_after_the_retreat():
    """return_joints is optional — without it the sweep still hands the arm back
    once the plate is clear, rather than hanging in a phase with no target."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 0.3, 0.0), retreat_standoff=0.40)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    assert node.return_joints == []

    _run(node, robot, cycles=4000)
    assert node.status == "succeeded"
    assert WALL_X - robot.tip()[0, 3] == pytest.approx(0.40, abs=0.05)


def test_a_lost_surface_stops_the_sweep_instead_of_pressing_on():
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.0, 0.0))
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    _run(node, robot, cycles=200)          # settle onto the wall first
    assert node.status == "running"

    # The wall drops away (a doorway, a lost reading): every range jumps.
    node.surface.update(node.distances)
    robot_ranges = robot.ranges
    robot.ranges = lambda: robot_ranges() + 1.0
    _run(node, robot, cycles=50)

    assert node.status.startswith("failed")
    assert "obstacle or lost surface" in node.status


def _costmap(obstacles, resolution=0.05, origin=(0.0, -3.0), shape=(160, 160)):
    """An OccupancyGrid in the map frame with rectangular obstacles (world m)."""
    from nav_msgs.msg import OccupancyGrid

    data = np.zeros(shape, dtype=int)
    for x0, y0, x1, y1 in obstacles:
        c0 = int((x0 - origin[0]) / resolution)
        c1 = int((x1 - origin[0]) / resolution)
        r0 = int((y0 - origin[1]) / resolution)
        r1 = int((y1 - origin[1]) / resolution)
        data[max(0, r0):max(0, r1), max(0, c0):max(0, c1)] = 100
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = resolution
    grid.info.width, grid.info.height = shape[1], shape[0]
    grid.info.origin.position.x, grid.info.origin.position.y = origin
    grid.data = [int(v) for v in data.flatten()]
    return grid


def test_an_obstacle_across_the_sweep_slows_the_base():
    """The barrier reaches the base command through the same QP as everything else."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    _wire(node, robot)

    # Free run first: record the along-wall speed the base settles on.
    _run(node, robot, cycles=60)
    free_speed = None
    commands = {}
    _capture(node, commands)
    _wire(node, robot)
    node._control_step()
    free_speed = abs(commands["twist"][1])
    assert free_speed > 0.005, "the base should be sweeping before we block it"

    # Now put something 0.6 m along the sweep direction (+y) from the base.
    base_y = robot.base_xy[1]
    node._on_costmap(_costmap([(robot.base_xy[0] - 1.0, base_y + 0.6,
                                robot.base_xy[0] + 1.0, base_y + 0.9)]))
    _wire(node, robot)
    commands.clear()
    node._control_step()

    assert abs(commands["twist"][1]) < free_speed
    assert node.closest_obstacle < 1.0


def test_the_scanned_wall_does_not_push_the_base_away():
    """The wall is lethal in the costmap by definition; masking is what keeps the
    barrier from fighting the standoff loop for the whole sweep."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    # The wall itself, as the costmap sees it: a slab at x = WALL_X.
    node._on_costmap(_costmap([(WALL_X, -3.0, WALL_X + 0.3, 5.0)]))
    _wire(node, robot)

    node._control_step()
    A, lower = node._avoidance_rows(node.chain.n_joints)
    assert A.shape[0] == 0, "the swept wall must be masked out of the barrier"


def test_avoidance_degrades_loudly_rather_than_silently():
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    _wire(node, robot)

    warnings = []
    node.get_logger().warn = lambda msg, **kw: warnings.append(msg)
    A, _ = node._avoidance_rows(node.chain.n_joints)      # no costmap has arrived

    assert A.shape[0] == 0
    assert any("NO obstacle avoidance" in w for w in warnings)


def test_stale_sensor_data_halts_the_base():
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.0, 0.0), max_hold_seconds=0.0)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    node.deadline = 1e12

    published = []
    node.cmd_vel_pub.publish = lambda msg: published.append(
        (msg.linear.x, msg.linear.y, msg.angular.z))
    _wire(node, robot)
    node.distance_stamp = 0.0              # ancient frame

    for _ in range(3):
        node._control_step()

    assert published and all(cmd == (0.0, 0.0, 0.0) for cmd in published)
    assert node.status.startswith("failed")
    assert "distance_sensors" in node.status


# ----------------------------------------------------------------------
# The hardware layer between the QP and the actuators
# ----------------------------------------------------------------------
def test_halting_the_arm_commands_a_pose_not_a_zero():
    """At node level, the trap wbc/streaming.py exists to avoid.

    Every stop path here used to publish an array of zeros, which is a correct
    velocity command and a full-speed run to the zero configuration on the
    position interface the node now defaults to.
    """
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.0, 0.0))
    robot = _start_state(node.chain)
    commands = {}
    _capture(node, commands)
    _wire(node, robot)

    node.halt()

    assert commands["arm"] == pytest.approx(list(robot.q))
    assert commands["arm"] != [0.0] * 6


def test_speed_scaling_slows_the_base_and_the_arm_together():
    """The whole point of reading the UR's speed scaling in a whole-body loop.

    The slider scales the ARM and not the base. Applied to only half the robot
    it does not slow the sweep down, it desynchronises it: the base keeps
    sweeping along the wall while the arm falls behind the standoff and
    orientation corrections that were supposed to accompany that travel.
    """
    def one_step(scaling):
        node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
        robot = _start_state(node.chain)
        node.q_posture = robot.q.copy()
        node.row_z = float(robot.tip()[2, 3])
        node.deadline = 1e12
        node.hardware._on_scaling(Float64(data=scaling))
        commands = {}
        _capture(node, commands)
        node.arm_stream.reset(robot.q)
        _wire(node, robot)
        node._control_step()
        # The arm's command leaves on the stream timer, so one tick of it is
        # part of "what this cycle published".
        node._stream_step()
        return (np.array(commands["twist"]),
                np.array(commands["arm"]) - robot.q)

    full_twist, full_arm = one_step(1.0)
    half_twist, half_arm = one_step(0.5)

    assert np.linalg.norm(full_twist) > 1e-4
    assert np.linalg.norm(full_arm) > 1e-6
    assert half_twist == pytest.approx(full_twist * 0.5, rel=1e-6, abs=1e-9)
    assert half_arm == pytest.approx(full_arm * 0.5, rel=1e-6, abs=1e-9)


def test_a_protective_stop_stops_the_base_instead_of_sweeping_past_a_frozen_arm():
    """ros2_control does not notice a protective stop; this loop has to.

    Commands keep publishing and the QP keeps solving through one, but the arm
    does not move. Without the gate the base would carry on along the wall
    dragging an arm that is no longer following it.
    """
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.0, 0.0), max_hold_seconds=0.0)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    node.deadline = 1e12
    commands = {}
    _capture(node, commands)
    _wire(node, robot)
    node.arm_stream.reset(robot.q)

    # SafetyMode.PROTECTIVE_STOP, without depending on the UR driver being
    # installed to run this test.
    node.hardware._on_safety_mode(SimpleNamespace(mode=3))
    node._control_step()

    assert commands["twist"] == [0.0, 0.0, 0.0]
    assert commands["arm"] == pytest.approx(list(robot.q))
    assert node.status.startswith("failed")
    assert "PROTECTIVE_STOP" in node.status


def test_resuming_after_a_hold_ramps_up_instead_of_jumping_to_full_speed():
    """A hold leaves a command history of zero, not no history at all.

    Clearing it would let the cycle that recovers from a stale-input hiccup
    jump straight back to sweep speed — the exact step the acceleration bound
    exists to prevent, at the moment the robot's state is least understood.
    """
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0),
                 base_accel_max=[0.05, 0.05, 0.1], max_hold_seconds=1e9)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    node.deadline = 1e12
    commands = {}
    _capture(node, commands)
    node.arm_stream.reset(robot.q)
    _wire(node, robot)
    node.distance_stamp = 0.0                  # ancient frame -> a hold
    node._control_step()
    assert node.u_prev == pytest.approx(np.zeros(3 + node.chain.n_joints))

    _wire(node, robot)                          # inputs recover
    commands.clear()
    node._control_step()

    assert np.all(np.abs(np.array(commands["twist"])[:2]) <= 0.001 + 1e-9)


def test_a_paused_robot_holds_rather_than_crawling():
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.0, 0.0), max_hold_seconds=0.0)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    node.deadline = 1e12
    commands = {}
    _capture(node, commands)
    _wire(node, robot)
    node.hardware._on_scaling(Float64(data=0.0))

    node._control_step()

    assert commands["twist"] == [0.0, 0.0, 0.0]
    assert "scaled" in node.status


def test_the_two_arm_interfaces_sweep_the_same_way():
    """Position streaming is a change of interface, not of control law.

    The QP, the task twist and the base command are untouched by the choice;
    only the last step before the wire differs. If these two diverged, the
    integration would be doing something the velocity path was not.
    """
    def sweep(mode):
        node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0),
                     arm_stream_interface=mode)
        robot = _start_state(node.chain)
        node.q_posture = robot.q.copy()
        node.row_z = float(robot.tip()[2, 3])
        _run(node, robot, cycles=150)
        return robot.tip()[:3, 3], robot.q.copy()

    tip_position, q_position = sweep(POSITION)
    tip_velocity, q_velocity = sweep(VELOCITY)

    assert tip_position == pytest.approx(tip_velocity, abs=1e-3)
    assert q_position == pytest.approx(q_velocity, abs=1e-3)


def _setpoint_step_over(node, robot, elapsed, rate=0.1):
    """One stream tick, ``elapsed`` seconds after the tick before it.

    Returns how far the arm setpoint advanced. The stored velocity is refreshed
    at the moment the tick lands, so the solve is healthy and only the stream's
    own integration period is under test.
    """
    _wire(node, robot)
    node.arm_stream.reset(robot.q)
    clock = _Clock(100.0)
    node._now = clock

    node.cycle_period = 1.0 / node.control_rate
    node.stream_stamp = clock.t
    clock.advance(elapsed)
    node.arm_qdot = np.full(len(node.arm_joints), rate)
    node.arm_qdot_stamp = clock.t

    before = node.arm_stream.command.copy()
    node._stream_step()
    return float(np.max(node.arm_stream.command - before))


def test_a_late_stream_tick_advances_the_arm_setpoint_in_real_time():
    """The desynchronisation behind the first hardware run's jerk, now one rate
    down.

    Integrating the NOMINAL period regardless is what let the arm fall behind:
    at 20 Hz against a nominal 50 the setpoint advanced 20 ms of motion while
    50 ms of clock passed, so the arm tracked at 40% of commanded while the
    base, commanded as a velocity rather than integrated, tracked at 100%. The
    stream inherits that lesson: what it integrates is what elapsed.
    """
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)

    # A tick displaced by a slow solve: 10 ms rather than the nominal 5.
    # 0.1 rad/s for 0.01 s is 0.001 rad, not the 0.0005 the nominal would give.
    assert _setpoint_step_over(node, robot, elapsed=0.01) == pytest.approx(0.001, abs=1e-9)


def test_a_single_late_stream_tick_cannot_jump_the_setpoint():
    """The other half of the clamp: after a stall, one tick must not advance the
    whole gap it slept through."""
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)

    # A whole second late. The cap is stream_period_max_factor nominal stream
    # periods, and the bound is read from the parameters rather than written in
    # as a constant: both are sized from rates actually measured, so they are
    # expected to move, and a test that pinned the number would fail for the
    # wrong reason every time they did. What must not change is that ONE late
    # tick advances a bounded amount rather than the whole gap.
    factor = float(node.get_parameter("stream_period_max_factor").value)
    nominal = 1.0 / node.stream_rate
    step = _setpoint_step_over(node, robot, elapsed=1.0)
    assert step == pytest.approx(0.1 * factor * nominal, abs=1e-9)
    assert step < 0.1 * 1.0, "a second-late tick must not advance a second of motion"


def test_a_fast_stream_tick_does_not_under_integrate():
    """The lower half of the clamp: a burst of early ticks must not leave the
    arm creeping while the base runs at the commanded speed."""
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)

    assert _setpoint_step_over(node, robot, elapsed=0.001) == pytest.approx(0.0005, abs=1e-9)


def test_a_dead_solve_holds_the_arm_instead_of_streaming_on():
    """The hazard the split introduces, and the reason position mode is safe.

    While the solve and the publish shared a timer, a solve that stopped was a
    setpoint that stopped and an arm that froze. On its own timer the stream
    would go on integrating the last velocity it was given and walk the arm down
    the wall with nothing computing where it ought to be.
    """
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)
    _wire(node, robot)
    commands = {}
    _capture(node, commands)
    clock = _Clock()
    node._now = clock
    node.arm_stream.reset(robot.q)
    node.cycle_period = 1.0 / node.control_rate
    node.arm_qdot = np.full(len(node.arm_joints), 0.4)
    node.arm_qdot_stamp = clock.t
    node.stream_stamp = clock.t

    clock.advance(1.0 / node.stream_rate)
    node._stream_step()
    assert np.max(np.abs(np.array(commands["arm"]) - robot.q)) > 0.0, (
        "a fresh velocity should be streaming")

    # The solve stops: no new velocity, and the clock runs on.
    clock.advance(node.arm_command_max_age_factor / node.control_rate + 0.01)
    node._stream_step()

    assert node.arm_command_stale
    # The MEASURED pose. Not a zero — that is a full-speed run to the zero
    # configuration — and not the setpoint it had reached, which is ahead of the
    # arm by exactly the amount it is no longer entitled to.
    assert commands["arm"] == pytest.approx(list(robot.q))

    # ... and it stays held, rather than putting the measurement's own noise on
    # the wire 200 times a second.
    commands.clear()
    for _ in range(5):
        clock.advance(1.0 / node.stream_rate)
        node._stream_step()
    assert "arm" not in commands


def test_halt_stops_both_timers_before_it_holds_the_arm():
    """A cancelled solve is not a stopped arm any more.

    The stream advances the setpoint on its own timer, so halt() has to cancel
    that one too, and cancel it BEFORE the hold — a tick landing after would
    integrate the arm straight back out of the pose it was stopped in.
    """
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)
    _wire(node, robot)
    commands = {}
    _capture(node, commands)
    node.start()
    control_timer, stream_timer = node.control_timer, node.stream_timer
    assert control_timer is not None and stream_timer is not None

    # A velocity in flight, as if a solve had just stored one.
    node.arm_qdot = np.full(len(node.arm_joints), 0.4)
    node.arm_qdot_stamp = node._now()

    node.halt()

    assert control_timer.is_canceled()
    assert stream_timer.is_canceled()
    assert commands["twist"] == [0.0, 0.0, 0.0]
    assert commands["arm"] == pytest.approx(list(robot.q))

    # A tick that was already in flight when halt() ran must find nothing to do.
    commands.clear()
    node._stream_step()
    assert "arm" not in commands


def test_the_stream_rate_changes_how_finely_the_arm_moves_not_how_far():
    """The whole claim of splitting the two rates.

    Same solve, same control rate, same elapsed time: streaming four times as
    often must put the arm in the same place by smaller steps. If the distance
    moved changed with the stream rate, the setpoint would no longer be tracking
    real time and the arm would drift away from the base.
    """
    def sweep(stream_rate):
        node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0), stream_rate=stream_rate)
        robot = _start_state(node.chain)
        node.q_posture = robot.q.copy()
        node.row_z = float(robot.tip()[2, 3])

        setpoints = []
        real_send = node.arm_stream.send

        def spy(qdot, dt, q_measured=None):
            out = real_send(qdot, dt, q_measured)
            setpoints.append(np.asarray(out, dtype=float).copy())
            return out
        node.arm_stream.send = spy

        _run(node, robot, cycles=150)
        steps = np.abs(np.diff(np.array(setpoints), axis=0))
        return robot.tip()[:3, 3].copy(), len(setpoints), steps.max()

    fine_tip, fine_ticks, fine_step = sweep(200.0)
    coarse_tip, coarse_ticks, coarse_step = sweep(50.0)

    assert fine_ticks == 4 * coarse_ticks
    # The staircase's tread, which is what servoj is chasing.
    assert fine_step == pytest.approx(coarse_step / 4.0, rel=0.05), (
        f"setpoint steps {fine_step:.6f} vs {coarse_step:.6f} rad")
    # ... and the arm covered the same ground doing it.
    assert fine_tip == pytest.approx(coarse_tip, abs=1e-4)


def test_the_acceleration_bound_reaches_the_published_base_command():
    """The QP is memoryless; its solution steps when the active set changes."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0),
                 base_accel_max=[0.05, 0.05, 0.1])
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    node.deadline = 1e12
    commands = {}
    _capture(node, commands)
    node.arm_stream.reset(robot.q)
    _wire(node, robot)
    # Standing still, with a command already established — so this cycle is
    # limited rather than passed through as the first one after a stop.
    node.u_prev = np.zeros(3 + node.chain.n_joints)
    node.command_stamp = node._now()

    node._control_step()

    # The QP wants ~0.03 m/s along the wall; one cycle at 50 Hz with a
    # 0.05 m/s^2 bound may only deliver 0.001 m/s of it.
    assert np.all(np.abs(np.array(commands["twist"])[:2]) <= 0.001 + 1e-9)
    assert np.linalg.norm(node.u_prev) > 0.0            # it is ramping, not stuck


# ----------------------------------------------------------------------------
# Constant base travel
# ----------------------------------------------------------------------------
# The fixtures above park the turret facing the WALL, so the sweep is lateral
# motion. That is not the deployed geometry: during a wall scan the turret faces
# ALONG the wall with the wall on its left, which makes the sweep chassis-forward
# motion — pure rolling, both wheels the same way. The helper below is that
# geometry, and it is the one the travel pin is designed for.


def _start_state_along_wall(chain, gap=0.35, y=1.2):
    """The deployed ScanWall geometry, as a starting state.

    The base is yawed -90 deg, so the turret's +y (its LEFT) points at the wall
    at ``x = WALL_X`` and its +x — forward — runs along the wall in world -y.
    The arm's first joint is turned +90 deg to cancel that, which leaves the tip
    and the plate in exactly the pose ``_start_state`` produces: reaching at the
    wall from ``gap`` metres. Only the body underneath is rotated.
    """
    q = np.array([np.pi / 2.0, -np.pi / 2.0, 0.9, -0.9, 0.08, np.pi / 2.0])
    tip_local = chain.fk(q)[:3, 3]
    reach = float(np.hypot(tip_local[0], tip_local[1]))
    return KinematicRobot(chain, base_xy=[WALL_X - reach - gap, y],
                          yaw=-np.pi / 2.0, q_arm=q, dt=0.02)


def _sweep_along_wall(**overrides):
    """A node sweeping in world -y, which is turret-forward in that geometry."""
    return _node((WALL_X, 1.2, 0.0), (WALL_X, 0.0, 0.0), **overrides)


def _travel_history(node, robot, cycles, skip=15):
    """Run the loop, returning the forward base command published each cycle.

    ``skip`` drops the opening cycles, where the acceleration bound is still
    ramping the base up from a standstill and no command is at its target yet.
    """
    commands, history = {}, []
    _capture(node, commands)
    clock = _install_clock(node)
    node.deadline = 1e12
    node.arm_stream.reset(robot.q)
    for _ in range(cycles):
        if node.status != "running":
            break
        _wire(node, robot)
        commands.clear()
        node._control_step()
        history.append(commands.get("twist", [0.0, 0.0, 0.0])[0])
        _stream(node, clock)
        robot.step(commands.get("twist", [0.0, 0.0, 0.0]),
                   commands.get("arm"), node.arm_stream.mode)
    return np.array(history[skip:])


def test_the_base_holds_one_steady_travel_speed_when_the_turret_faces_along_the_wall():
    """The point of the pin: the GPR sees a constant speed, not the solver's opinion.

    A line scan counts distance off its own wheel, so a base velocity that moves
    whenever the QP's active set changes smears the scan in a way nothing
    downstream can undo. With the travel pinned the base does exactly one thing
    for the whole segment.
    """
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    pinned = _travel_history(node, robot, cycles=200)

    assert node.base_travel_pinned, "the deployed geometry should pin the travel"
    # Turret forward IS the sweep direction here, so the whole reference lands on
    # this one axis at the full sweep speed.
    assert pinned.mean() == pytest.approx(node.sweep_speed, abs=1e-3)
    assert pinned.ptp() < 1e-3, f"travel speed varied by {pinned.ptp():.5f} m/s"


def test_pinning_the_travel_beats_letting_the_solver_apportion_it():
    """Same geometry, same wall, only the pin differs — so the spread is the pin's."""
    steady = _sweep_along_wall()
    steady_robot = _start_state_along_wall(steady.chain)
    steady.q_posture = steady_robot.q.copy()
    steady.row_z = float(steady_robot.tip()[2, 3])

    free = _sweep_along_wall(base_constant_travel=False)
    free_robot = _start_state_along_wall(free.chain)
    free.q_posture = free_robot.q.copy()
    free.row_z = float(free_robot.tip()[2, 3])

    pinned = _travel_history(steady, steady_robot, cycles=200)
    apportioned = _travel_history(free, free_robot, cycles=200)

    assert not free.base_travel_pinned
    assert pinned.ptp() < apportioned.ptp(), (
        f"pinned spread {pinned.ptp():.5f} should beat {apportioned.ptp():.5f}")
    # Both still sweep — this is about steadiness, not about going faster.
    assert apportioned.mean() > 0.5 * steady.sweep_speed


def test_a_turret_across_the_sweep_hands_the_travel_back_to_the_solver():
    """Degrade to the old behaviour rather than pinning the wrong axis.

    ``_start_state`` parks the turret facing the wall, so the sweep is lateral
    and the turret's forward axis is not the travel axis at all. Pinning it would
    hold the base at nearly zero along the sweep and leave the arm to make up an
    ever-growing rest, which is worse than not pinning.
    """
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    start_base = robot.base_xy.copy()
    _run(node, robot, cycles=300)

    assert not node.base_travel_pinned
    assert abs(robot.base_xy[1] - start_base[1]) > 0.05, "it should still sweep"


# ----------------------------------------------------------------------------
# The GPR press
# ----------------------------------------------------------------------------
# Force replaces distance on the wall-normal axis, because UR's force_mode cannot
# run alongside a streaming controller. These drive the press through the REAL
# QP against a spring wall, which is the only place the interaction between the
# force loop and the rest of the sweep shows up. There is no force/torque sensor
# in Gazebo, so this harness is where the press gets tested at all.


def _press_node(**overrides):
    """A sweep node with the press on, and a faster approach FLOOR than the robot.

    ``press_approach_min_speed`` is 0.8 mm/s in the field, and it holds for the
    last 1.26 cm — the approach margin — so a press starting at APPROACH_GAP
    takes about 26 s to touch. That is 1300 cycles of QP solves before any test
    here reaches its subject, and the subject is never the approach: the schedule
    is tested directly, at three loop rates, in test_wbc_admittance.py. Raising
    only the floor leaves the schedule's SHAPE intact and shortens the crawl.
    """
    params = dict(press_enabled=True, press_force=5.0, press_gain=5.0e-5,
                  press_v_max=0.005, press_seek_speed=0.01,
                  press_filter_tau=0.09, press_approach_min_speed=0.005)
    params.update(overrides)
    return _sweep_along_wall(**params)


def _press_run(node, robot, cycles, on_cycle=None):
    """Drive the loop, recording (true contact force, base travel) each cycle.

    ``on_cycle`` is called with the cycle index after the node's inputs have
    been wired and before the solve, for tests that change the world mid-sweep
    (a wall that recedes, a wheel that unloads). Wiring first is what lets it
    override a sensed value for that cycle as well as move the fixture.
    """
    forces, travel = [], []
    commands = {}
    _capture(node, commands)
    clock = _install_clock(node)
    node.deadline = 1e12
    node.arm_stream.reset(robot.q)
    for cycle in range(cycles):
        # pending_status, not status: finish() withholds the terminal status
        # until the plate has retreated, and sampling the retreat would read a
        # force of zero for the sweep it is meant to be measuring.
        if node.status != "running" or node.pending_status is not None:
            break
        _wire(node, robot)
        if on_cycle is not None:
            on_cycle(cycle)
        commands.clear()
        node._control_step()
        forces.append(robot.press_force())
        travel.append(abs(commands.get("twist", [0.0, 0.0, 0.0])[0]))
        _stream(node, clock)
        robot.step(commands.get("twist", [0.0, 0.0, 0.0]),
                   commands.get("arm"), node.arm_stream.mode)
    return np.array(forces), np.array(travel)


def test_the_press_finds_the_wall_and_holds_the_target_force():
    node = _press_node()
    robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    forces, _ = _press_run(node, robot, cycles=1500)

    assert node.status == "running", f"the press ended the sweep: {node.status}"
    assert node.press.in_contact, "it should have reached the wall"
    assert forces[-100:].mean() == pytest.approx(5.0, abs=1.0)
    assert forces.max() < 15.0, "no slamming on the way in"


def test_the_base_holds_still_until_the_wheel_is_on_the_wall():
    """The fault from the first hardware run: the base set off at full sweep
    speed while the plate was still 8 cm from contact, so it was a third of the
    way down the wall before the wheel arrived — scrubbing it sideways rather
    than letting it roll, and the GPR only records while the wheel ROTATES.
    """
    node = _press_node()
    robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    forces, travel = _press_run(node, robot, cycles=1500)

    # Find the cycle contact was made, and check the base was parked until then.
    contact = int(np.argmax(forces > 0.0))
    assert contact > 50, "the fixture should start well clear of the wall"
    assert travel[:contact].max() == pytest.approx(0.0, abs=1e-9), (
        f"the base moved {travel[:contact].max():.4f} m/s before contact")
    assert travel[-50:].mean() > 0.5 * node.sweep_speed, "and sweeps once pressed"


def test_the_press_does_not_stop_the_sweep_travelling():
    """Force on the normal, travel on the tangent — once contact is made the
    axes stay independent."""
    node = _press_node()
    robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    start_y = robot.base_xy[1]
    _press_run(node, robot, cycles=1500)

    assert abs(robot.base_xy[1] - start_y) > 0.10, "the base should still be sweeping"
    assert node.press.in_contact


def test_a_wall_that_recedes_mid_sweep_slows_the_base_instead_of_scanning_air():
    """The failure the LATCHING gate left behind.

    ``touched`` opened the travel once and never closed it again, so a wall that
    fell away after a genuine contact — a bay, a reveal, the avoidance barrier
    walking the base off the plane — left the base running at full sweep speed
    with the GPR recording air. Nothing detected it, because the only thing that
    could have was already latched. The gate is a filtered authority now, so the
    base slows itself without anything having to notice the event at all.
    """
    global WALL_X
    node = _press_node()
    # Start 3 cm off contact: this test is about what happens AFTER the wheel
    # arrives, and _press_node already shortens the crawl in front of it.
    robot = _start_state_along_wall(node.chain, gap=PLATE_STANDOFF + 0.03)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    recede_at, was = 500, WALL_X
    try:
        def on_cycle(cycle):
            global WALL_X
            # 6 cm back: far enough to unload the wheel completely, near enough
            # that the three ToF sensors (valid to 0.258 m) still see the plane.
            # The sweep therefore keeps a surface to measure, which keeps this a
            # test of the gate rather than of a lost plane fit.
            WALL_X = was + 0.06 if cycle >= recede_at else was

        forces, travel = _press_run(node, robot, cycles=recede_at + 200,
                                    on_cycle=on_cycle)
    finally:
        WALL_X = was

    assert len(travel) == recede_at + 200, f"the sweep ended early: {node.pending_status}"
    assert forces[recede_at - 1] > 1.0, "the wheel should be loaded before the wall moves"
    swept = travel[recede_at - 50:recede_at].mean()
    assert swept > 0.5 * node.sweep_speed, "and the base should be sweeping"
    assert forces[-1] == 0.0, "the wheel should be off the wall at the end"
    # Four seconds later, at tau = 1.5 s, the authority is down to single figures.
    assert travel[-1] < 0.25 * swept, (
        f"the base was still travelling at {travel[-1]:.4f} m/s against {swept:.4f} "
        f"with the wheel off the wall (authority {node.travel_authority:.2f})")
    assert node.press.touched, "the latch still latches — it is only the ARMING now"


def test_a_brief_hollow_costs_speed_rather_than_stopping_the_base():
    """The objection the old comment raised against a live gate, priced.

    Gating the base on the contact state directly would stop and restart it
    several times a sweep, and each restart is a step against a loaded wheel,
    which scrubs it sideways instead of rolling it. The filter is the answer:
    a short loss of contact costs a slice of the speed and recovers, and the
    base never stops.
    """
    # The REAL approach floor here, not _press_node's shortened one. During the
    # hollow the press drops to SEEK and closes at that floor, so a fast floor
    # would drive the plate 1.5 mm deeper and meet the returning wall at 30 N —
    # which is a finding about the force limit, not about the travel gate, and
    # belongs in its own test rather than in the middle of this one.
    node = _press_node(press_tare_seconds=0.0, press_approach_min_speed=0.0008)
    # Already touching, so the sweep is in the state this test is about within a
    # few cycles rather than after a 25 s approach. No tare: the wheel is loaded
    # from cycle zero, and taring in contact is a fault by design.
    robot = _start_state_along_wall(node.chain, gap=PLATE_STANDOFF - 0.00025)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    # 0.3 s, applied to the SENSED force rather than by moving the fixture: a
    # hollow the wheel rides over unloads it without the wall going anywhere,
    # and moving a 2e4 N/m wall by hand would spend the whole test on the
    # re-contact transient instead of on the gate.
    hollow = range(300, 315)
    def on_cycle(cycle):
        if cycle in hollow:
            node.press_force = 0.0

    _, travel = _press_run(node, robot, cycles=500, on_cycle=on_cycle)

    assert len(travel) == 500, f"the sweep ended early: {node.pending_status}"
    before = travel[hollow.start - 50:hollow.start].mean()
    assert before > 0.5 * node.sweep_speed, "the base should be sweeping into the hollow"
    dip = travel[hollow.start:hollow.stop + 40].min()
    assert dip > 0.5 * before, f"a 0.3 s hollow cut the travel to {dip / before:.0%}"
    assert travel[-1] > 0.9 * before, "and the base should be back up to speed"


def _squeezed_press(force_alpha, weight_press_normal=1.0e4, cycles=400):
    """A press with an obstacle BEHIND the base, so the barrier pushes it in.

    The avoidance barrier is a perfectly good reason for the solver to drive the
    base at the wall, and it is the shape of the 13ed27e log with the signs
    reversed. ``weight_press_normal`` is exposed because it turns out to be the
    variable that decides whether anything happens at all — see the two tests
    below, which are meant to be read together.
    """
    node = _press_node(press_force_alpha=force_alpha, press_tare_seconds=0.0,
                       weight_press_normal=weight_press_normal)
    robot = _start_state_along_wall(node.chain, gap=PLATE_STANDOFF - 0.00025)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    # Half a metre behind the base (the wall is at +x, so behind is -x) and wide
    # across the sweep, so the only way out of it is toward the wall.
    base_x, base_y = robot.base_xy
    grid = _costmap([(base_x - 0.75, base_y - 2.0, base_x - 0.45, base_y + 2.0)])

    def on_cycle(cycle):
        if cycle >= 40:
            # Re-stamped every cycle: the fixture's clock runs on, and a costmap
            # older than costmap_max_age is dropped rather than trusted.
            node._on_costmap(grid)

    forces, _ = _press_run(node, robot, cycles=cycles, on_cycle=on_cycle)
    return node, forces


def test_the_press_normal_task_already_couples_the_base_to_the_arm():
    """Read this before the next test, and before touching weight_press_normal.

    The force barrier was proposed on the premise that the QP has no idea force
    exists, so a base driven at the wall has to be fought by the arm through the
    standoff task. That premise is WRONG at the shipped tuning, and it is worth
    having a test say so: weight_press_normal is 1e4 on ``m_hat @ J``, which is
    the whole-body normal row, so the solver already spends arm motion to hold
    the plate's normal velocity while the base moves. Measured here, the base
    travels 6 cm into the wall under an obstacle and the force does not move.

    So the barrier is a backstop, not the thing standing between the plate and
    the wall. What it adds is that it is a BOUND rather than a target — a task
    that loses is a silent residual, a constraint that loses is a reported
    slack — and that past the limit it demands retreat in proportion to the
    overshoot, where the press loop can never ask for more than gain * error
    clipped to press_v_max, which at the limit is 1.25 mm/s.
    """
    node, forces = _squeezed_press(force_alpha=0.0)

    assert node.pending_status is None, "no barrier, and still no overload"
    assert forces[-50:].mean() == pytest.approx(5.0, abs=1.0), "the target, held"


def test_the_force_barrier_bounds_the_squash_when_the_press_task_cannot():
    """What the barrier is actually worth, measured.

    The press normal task is a weighted least-squares term, so its authority is
    relative to everything else in the stack; the barrier's is not. Drop that
    weight until the task stops winning and the difference is the barrier:
    without it the wheel runs 44% past the limit before anything notices,
    with it the force is held AT the limit instead of sailing through it.

    Both still fail the sweep, and that is the honest result — the barrier
    bounds the overload, it does not make the situation survivable.
    """
    guarded, guarded_force = _squeezed_press(force_alpha=1.0, weight_press_normal=1.0)
    unguarded, unguarded_force = _squeezed_press(force_alpha=0.0, weight_press_normal=1.0)

    assert unguarded_force.max() > 40.0, (
        f"unbounded, the squash runs well past the limit "
        f"(peak {unguarded_force.max():.1f} N)")
    assert guarded_force.max() <= 30.5, (
        f"the barrier should hold it at the limit, not past it "
        f"(peak {guarded_force.max():.1f} N)")
    assert guarded_force.max() < 0.75 * unguarded_force.max()
    assert guarded.pending_status and unguarded.pending_status, "both still fail"


def test_the_force_barrier_does_not_slow_a_press_that_is_going_fine():
    """A barrier that binds when it should not is a press that never reaches its
    target force, which is the silent failure the whole press exists to avoid.
    With nothing pushing the plate in, the row must be slack throughout."""
    node = _press_node()
    robot = _start_state_along_wall(node.chain, gap=PLATE_STANDOFF + 0.03)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    forces, travel = _press_run(node, robot, cycles=700)

    assert node.press.in_contact, "it should have reached the wall"
    assert forces[-100:].mean() == pytest.approx(5.0, abs=1.5), "and be holding it"
    assert travel[-50:].mean() > 0.5 * node.sweep_speed, "and still be sweeping"


def test_a_press_that_never_reaches_the_wall_fails_instead_of_recording_air():
    """The first hardware run reported 'Sweep succeeded' having swept a segment
    it may never have pressed. A false success means nobody knows to rescan."""
    node = _press_node(press_contact_timeout=2.0)
    # Start beyond reach, so seeking can never close the gap.
    robot = _start_state_along_wall(node.chain, gap=0.60)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    _press_run(node, robot, cycles=1500)

    assert node.pending_status is not None
    assert node.pending_status.startswith("failed")
    assert "never reached the wall" in node.pending_status


def test_a_press_with_no_force_feedback_holds_instead_of_pushing_on():
    """An arm driving at a wall on a timer is the failure this must not become."""
    node = _press_node()
    robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    commands = {}
    _capture(node, commands)
    node.deadline = 1e12
    node.arm_stream.reset(robot.q)

    _wire(node, robot)
    node.wrench_stamp = None            # the broadcaster never came up
    node._control_step()

    assert commands["twist"] == [0.0, 0.0, 0.0], "a stale wrench must stop the base"


def test_an_overload_ends_the_sweep_rather_than_leaning_on_the_wall():
    # No tare: the overload is injected on the first cycle, and during a tare the
    # loop is deliberately holding still and believing nothing.
    node = _press_node(press_force_limit=25.0, press_filter_tau=0.0,
                       press_tare_seconds=0.0)
    robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    _capture(node, {})
    node.deadline = 1e12
    node.arm_stream.reset(robot.q)

    _wire(node, robot)
    node.press_force = 40.0             # the wheel is jammed against something
    node._control_step()

    # finish() withholds the terminal status until the plate has backed off the
    # wall, so pending_status is what says the sweep is over. Checking it rather
    # than driving the retreat keeps this about the overload.
    assert node.pending_status is not None
    assert node.pending_status.startswith("failed")
    assert "25.0 N limit" in node.pending_status
    assert node.phase == "retreat", "and it backs the plate off on the way out"


def test_pressing_stops_the_standoff_abort_firing():
    """The plate sits where the wall puts it, so a 20 cm target is not a thing to
    police any more. Without this the sweep would abort the moment it touched."""
    node = _press_node(standoff=0.20, max_standoff_error=0.05,
                       standoff_error_cycles=1)
    robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    _run(node, robot, cycles=20)

    assert node.status == "running"
    assert node.standoff_strikes == 0


def test_the_press_gain_can_be_dropped_while_the_wheel_is_loaded():
    """Tuning by restart means every attempt starts by driving the wheel back
    into the wall at the gain that just misbehaved. This is the way out."""
    node = _press_node()
    robot = _start_state_along_wall(node.chain, gap=APPROACH_GAP)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    _run(node, robot, cycles=600)
    assert node.press.in_contact
    assert node.press.gain == pytest.approx(5.0e-5)

    node.set_parameters([rclpy.parameter.Parameter("press_gain", value=1.0e-5)])
    _wire(node, robot)
    node._control_step()

    assert node.press.gain == pytest.approx(1.0e-5)
    assert node.press.in_contact, "retuning must not drop the press"


def test_the_safety_envelope_is_not_live_tunable():
    """min_distance and the filter are what stop a bad force reading and a bad
    gain respectively. Neither should be reachable mid-press."""
    node = _press_node()
    tunable = {name for name, _ in node.PRESS_TUNABLES}
    assert "press_min_distance" not in tunable
    assert "press_filter_alpha" not in tunable
    assert "press_stall_cycles" not in tunable


def test_an_obstacle_caps_the_travel_instead_of_freeing_it():
    """A barrier must be able to slow the base. It never needs to speed it up.

    The pin is a HARD bound and the obstacle rows are soft, so a two-sided pin
    would win against them and carry the base on into whatever they were
    avoiding. Releasing it outright fixed that and granted far more than the
    barrier asked for: measured over a Gazebo segment, the freed base ran
    anywhere from 0.002 to 0.063 m/s against a 0.030 reference. Keeping the
    upper half of the bound costs the barrier nothing at all.
    """
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    _travel_history(node, robot, cycles=60)
    assert node.base_travel_pinned, "it should be pinned before anything is in range"

    commands = {}
    _capture(node, commands)
    _wire(node, robot)
    node._control_step()
    free_speed = abs(commands["twist"][0])

    # Something across the path, 0.6 m ahead — the base travels in world -y here.
    base_y = robot.base_xy[1]
    node._on_costmap(_costmap([(robot.base_xy[0] - 1.0, base_y - 0.9,
                                robot.base_xy[0] + 1.0, base_y - 0.6)]))
    _wire(node, robot)
    commands.clear()
    node._control_step()
    blocked = commands["twist"][0]

    assert node.closest_obstacle < 1.0
    assert node.base_travel_capped, "an obstacle in range caps the travel"
    assert not node.base_travel_pinned, "... and the two-sided pin comes off"
    # The barrier keeps every bit of the authority it had: it slowed the base.
    assert abs(blocked) < free_speed
    # What it loses is the authority it never needed.
    assert abs(blocked) <= node.sweep_speed + 1e-9, "must not overspeed the scan"
    assert blocked >= -1e-9, "must not reverse into the segment already scanned"


def test_a_capped_barrier_can_still_stop_the_base_completely():
    """Zero stays reachable — the cap is a ceiling, not a floor.

    It takes a few cycles to get there, because the acceleration bound only lets
    the command fall by ``base_accel_max * dt`` each time. That ramp is the point:
    the barrier gets the base all the way to a stop, and does it smoothly.
    """
    node = _sweep_along_wall()
    robot = _start_state_along_wall(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    _travel_history(node, robot, cycles=60)

    commands = {}
    _capture(node, commands)
    # A slab right across the path, close enough that the barrier demands a stop.
    base_y = robot.base_xy[1]
    node._on_costmap(_costmap([(robot.base_xy[0] - 1.5, base_y - 0.5,
                                robot.base_xy[0] + 1.5, base_y - 0.2)]))

    clock = _install_clock(node)
    travel = []
    for _ in range(12):
        _wire(node, robot)
        commands.clear()
        node._control_step()
        travel.append(commands.get("twist", [0.0, 0.0, 0.0])[0])
        _stream(node, clock)
        robot.step(commands.get("twist", [0.0, 0.0, 0.0]),
                   commands.get("arm"), node.arm_stream.mode)

    travel = np.array(travel)
    assert node.base_travel_capped
    assert travel.min() == pytest.approx(0.0, abs=1e-6), "the barrier must reach a stop"
    # It got there by ramping, not by jumping — no step bigger than the bound.
    assert np.max(np.abs(np.diff(travel))) <= 0.3 / 50.0 + 1e-9
    # And the ceiling held the whole way down.
    assert travel.max() <= node.sweep_speed + 1e-9


# ----------------------------------------------------------------------
# Motion continuity: the solve itself, not the clip that follows it
# ----------------------------------------------------------------------

def _velocity_trace(node, robot, cycles):
    """Run the loop, keeping every velocity the SOLVER produced."""
    trace = []
    real_publish = node._publish

    def spy(u, n_arm):
        trace.append(np.asarray(u, dtype=float).copy())
        return real_publish(u, n_arm)

    node._publish = spy
    _run(node, robot, cycles)
    return np.array(trace)


def test_the_acceleration_bound_is_a_constraint_not_a_clip_after_the_fact():
    """The bound has to hold on the SOLVER's output, not just on what is
    published. Clipping afterwards yields a command the solve never checked:
    the base travel pin, the barriers and the joint limits were all satisfied
    by the number that came out, and by construction the clipped one satisfies
    none of them."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0),
                 base_accel_max=[0.3, 0.3, 0.6], arm_accel_max=2.0)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])

    trace = _velocity_trace(node, robot, 60)
    assert len(trace) > 20

    steps = np.abs(np.diff(trace, axis=0))
    allowed = np.concatenate((np.array([0.3, 0.3, 0.6]), np.full(6, 2.0)))
    # 3x the nominal period, matching the clamp _accel_bounds applies, plus a
    # little slack for the cycle where the bound is handed back unnarrowed.
    ceiling = allowed * 3.0 / 50.0 * 1.05
    worst = steps.max(axis=0)
    assert np.all(worst <= ceiling), (
        f"solver output stepped past the acceleration bound: {worst} > {ceiling}")


def test_the_smoothness_term_makes_consecutive_solutions_resemble_each_other():
    """Damping pulls toward stopping; this pulls toward carrying on. Turning it
    off must make the solution measurably rougher, or it is not doing anything.
    """
    rough = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0), weight_smoothness=0.0)
    robot = _start_state(rough.chain)
    rough.q_posture = robot.q.copy()
    rough.row_z = float(robot.tip()[2, 3])
    rough_trace = _velocity_trace(rough, robot, 60)

    smooth = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0), weight_smoothness=0.5)
    robot2 = _start_state(smooth.chain)
    smooth.q_posture = robot2.q.copy()
    smooth.row_z = float(robot2.tip()[2, 3])
    smooth_trace = _velocity_trace(smooth, robot2, 60)

    n = min(len(rough_trace), len(smooth_trace))
    # The WORST step, not the total path length. The two measure different
    # things and only one of them is the complaint: total travel is set by where
    # the arm has to get to and barely moves with this weight, while the peak
    # cycle-to-cycle change is exactly what is felt as a jolt. Measured across
    # this fixture, worst step falls 0.021 -> 0.012 -> 0.003 rad/s at weights of
    # 0, 0.5 and 5, while the standoff error stays at 0.048 m throughout.
    rough_step = np.abs(np.diff(rough_trace[:n, 3:], axis=0)).max()
    smooth_step = np.abs(np.diff(smooth_trace[:n, 3:], axis=0)).max()
    assert smooth_step < 0.75 * rough_step, (
        f"smoothness weight did not reduce the worst arm step: "
        f"{smooth_step:.5f} not meaningfully below {rough_step:.5f}")


def test_the_diagnostics_row_carries_all_three_velocities():
    """The whole point of the trace is comparing what was asked for, what was
    sent, and what the arm did. A row missing one of them cannot localise a
    fault, so check the layout rather than trusting the comment."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0), publish_diagnostics=True)
    robot = _start_state(node.chain)
    node.q_posture = robot.q.copy()
    node.row_z = float(robot.tip()[2, 3])
    node.joint_velocities = dict(zip(ARM_JOINTS, np.full(6, 0.123)))

    rows = []
    node.diag_pub.publish = lambda msg: rows.append(list(msg.data))
    _run(node, robot, 10)

    assert rows, "diagnostics were enabled but nothing was published"
    n = 3 + node.chain.n_joints
    header = node.DIAG_HEADER
    assert len(rows[-1]) == header + 2 * n + len(ARM_JOINTS) + 4
    row = rows[-1]
    assert row[1] == pytest.approx(1.0 / node.control_rate), "the SOLVE period"
    assert row[2] > 0.0, "solve duration should be recorded"
    # The measured block is the arm's own velocity, not a copy of the command.
    assert row[header + 2 * n:header + 2 * n + len(ARM_JOINTS)] == pytest.approx([0.123] * 6)
    # Appended rather than folded into the header, so every index a recorded bag
    # already knows keeps its meaning. Read beside [1] the stream period says the
    # two rates really are different numbers.
    assert row[-4] == pytest.approx(1.0 / node.stream_rate), "the STREAM period"
    # This node has no press, so nothing was gating the travel and no force
    # barrier was built — NaN, not zeros that would read as a closed gate and a
    # barrier holding the plate still.
    assert np.isnan(row[-3]), "the base travel authority"
    assert np.isnan(row[-2]), "the force barrier's approach cap"
    assert np.isnan(row[-1]), "the contact stiffness"


def test_diagnostics_stay_off_unless_asked_for():
    """A message per control cycle that nothing reads back is not a default."""
    node = _node((WALL_X, 0.0, 0.0), (WALL_X, 1.2, 0.0))
    assert node.diag_pub is None
