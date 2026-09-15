"""The hyperspectral bench run: sweep the wall in front of a hand-parked robot.

What `hyperspectral_wall_test` promises the operator is that the robot does
NOTHING but unfold, sweep one partition of whatever is ahead of it with the
camera sampling, and stop. Each piece of that promise is pinned here without a
ROS graph:

* the synthesised wall puts partition 1's scan pose exactly where the base is;
* `scan_wall_assume_parked` skips the transit and the costmap, so the base
  never moves;
* `fsm_stop_after` ends the run at ScanWall instead of driving home;
* `scan_world_frame` lets the whole thing run in `odom`, with no map and no
  localisation, because a parked base is fixed in any frame for a sweep;
* the tool composes the fsm_node flags that switch all of that on.

Run with:

    python3 -m pytest test/test_hyperspectral_wall_test.py -v
"""

import argparse
import math
import types

import pytest

from task_planner_fsm.hyperspectral_wall_test import (
    DEFAULT_SERVICE,
    build_fsm_argv,
)
from task_planner_fsm.machine import StateMachine
from task_planner_fsm.state import State
from task_planner_fsm.states.scan_wall import ScanWall
from task_planner_fsm.utils.costmap_utils import (
    DEFAULT_PARTITION_BASE_STANDOFF,
    DEFAULT_SCAN_LINE_OFFSET,
    partition_scan_pose,
    world_frame,
)
from task_planner_fsm.utils.hyperspectral_sampler import HyperspectralSampler
from task_planner_fsm.utils.wall_approach import unfolded_pose_name
from task_planner_fsm.utils.wall_geometry import (
    build_wall_in_front,
    left_scan_endpoint,
)
from task_planner_fsm.utils.wall_partitioning import partition_segment


# ---------------------------------------------------------------------------
# Doubles
# ---------------------------------------------------------------------------

class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg, **kw):
        self.lines.append((level, msg))

    def info(self, msg, **kw):
        self._log("info", msg)

    def warn(self, msg, **kw):
        self._log("warn", msg)

    def error(self, msg, **kw):
        self._log("error", msg)

    def debug(self, msg, **kw):
        self._log("debug", msg)


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


class _Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return _Clock()

    def create_publisher(self, *a, **kw):
        return _Publisher()

    def create_timer(self, period, cb):
        return types.SimpleNamespace(cancel=lambda: None)

    def destroy_timer(self, timer):
        pass


BASE = (2.0, 1.0)
YAW = math.pi / 2          # facing +Y
LENGTH = 0.8
LINES_Z = [1.0]


def wall():
    return build_wall_in_front(
        BASE, YAW, DEFAULT_PARTITION_BASE_STANDOFF, LENGTH,
        offset=DEFAULT_SCAN_LINE_OFFSET, scan_lines_z=LINES_Z,
    )


def make_ctx(**over):
    w = wall()
    ctx = {
        "node": _Node(),
        "current_wall_index": 0,
        "wall_inward_normals": [w["inward_normal"]],
        "walls_data": [w],
        "target_scan_wall": w["scan_line"],
        "target_scan_point": left_scan_endpoint(w["scan_line"], w["inward_normal"]),
        "current_wall_scan_lines": list(LINES_Z),
        "current_line_idx": 0,
        "sweep_use_arm": True,
        "scan_wall_assume_parked": True,
        "_cmd_vel_pub": _Publisher(),
        # No "global_costmap": the parked path must not need one.
    }
    ctx.update(over)
    return ctx


# ---------------------------------------------------------------------------
# The synthesised wall
# ---------------------------------------------------------------------------

def test_the_partition_scan_pose_is_the_base_pose():
    """The whole trick: with the face put partition_base_standoff_m ahead, the
    scan pose the partition geometry derives is where the robot already is, so
    there is nothing to drive to."""
    ctx = make_ctx()
    start, end = wall()["scan_line"]
    pose = partition_scan_pose(ctx, "test", start, end)
    assert pose is not None
    assert pose[0] == pytest.approx(BASE[0], abs=1e-9)
    assert pose[1] == pytest.approx(BASE[1], abs=1e-9)
    assert pose[2] == pytest.approx(YAW, abs=1e-9)


def test_the_scan_line_is_one_partition_long_and_centred_on_the_robot():
    start, end = wall()["scan_line"]
    assert math.dist(start[:2], end[:2]) == pytest.approx(LENGTH)
    mid = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
    # Centre lies straight ahead of the base, scan-line offset short of the face.
    ahead = DEFAULT_PARTITION_BASE_STANDOFF - DEFAULT_SCAN_LINE_OFFSET
    assert mid[0] == pytest.approx(BASE[0] + ahead * math.cos(YAW), abs=1e-9)
    assert mid[1] == pytest.approx(BASE[1] + ahead * math.sin(YAW), abs=1e-9)
    assert partition_segment(start, end, 0.8, 0.05) == [(start, end)]


def test_the_wall_normal_is_the_robot_heading():
    """inward_normal is what the EE z-axis looks along INTO the wall, and it is
    what wall_facing_yaw turns back into the parked heading. Anything but the
    heading would aim the plate off the wall."""
    n = wall()["inward_normal"]
    assert n[0] == pytest.approx(math.cos(YAW), abs=1e-9)
    assert n[1] == pytest.approx(math.sin(YAW), abs=1e-9)


def test_scan_line_starts_at_the_robots_left():
    """The sweep runs from scan_line[0]; ordering it left-to-right keeps it the
    same direction WallTargetSelection would choose."""
    w = wall()
    left = left_scan_endpoint(w["scan_line"], w["inward_normal"])
    assert left == w["scan_line"][0]
    # Facing +Y, the robot's left is -X.
    assert left[0] < w["scan_line"][1][0]


def test_heights_carry_through_sorted():
    w = build_wall_in_front(BASE, YAW, 1.15, 0.8, scan_lines_z=[1.4, 0.9])
    assert w["scan_lines_z"] == [0.9, 1.4]
    assert w["scan_line"][0][2] == pytest.approx(0.9)


@pytest.mark.parametrize("bad", [dict(face_distance=0.0), dict(scan_length=0.0)])
def test_degenerate_walls_are_refused(bad):
    kw = dict(face_distance=1.15, scan_length=0.8)
    kw.update(bad)
    with pytest.raises(ValueError):
        build_wall_in_front(BASE, YAW, **kw)


def test_the_front_pose_is_what_a_wall_facing_run_unfolds_to():
    """The bench pins scan_wall_unfolded_pose explicitly, but the policy must
    agree on its own: the arm sweep parks square to the wall, so the plate has
    to unfold to the FRONT."""
    assert unfolded_pose_name({"sweep_use_arm": True, "nav_face_wall": True}) \
        == "unfolded_front_fsm"


# ---------------------------------------------------------------------------
# scan_wall_assume_parked
# ---------------------------------------------------------------------------

@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s.started = True
    return s


def test_parked_requires_the_arm_sweep(state):
    """In the base-driven path the base IS the sweep; there is nothing to park."""
    assert state._assume_parked({"scan_wall_assume_parked": True, "sweep_use_arm": False}) is False
    assert state._assume_parked({"scan_wall_assume_parked": True}) is True
    assert state._assume_parked({}) is False


def test_parked_plan_needs_no_costmap(state):
    """Without the knob the arm-sweep plan polls for the global costmap; parked,
    the whole line is the one reachable segment and the plan comes back at
    once, with its scan pose on the base."""
    ctx = make_ctx()
    state._sweep_from, state._sweep_to = ctx["target_scan_wall"]
    plan = state._plan_line(ctx)
    assert plan is not None, "parked plan must not wait for a costmap"
    segments, poses = plan
    assert len(segments) == 1 and len(poses) == 1
    assert poses[0][0] == pytest.approx(BASE[0], abs=1e-9)
    assert poses[0][1] == pytest.approx(BASE[1], abs=1e-9)
    assert not ctx.get("error_triggered")


def test_unparked_plan_still_polls_for_the_costmap(state):
    """Regression guard for the normal mission: the knob off means the old
    behaviour, a None while the costmap has not arrived."""
    ctx = make_ctx(scan_wall_assume_parked=False)
    state._sweep_from, state._sweep_to = ctx["target_scan_wall"]
    assert state._plan_line(ctx) is None


def test_parked_transit_never_moves_the_base(state):
    """The transit phase hands straight to line_column: no Nav2 goal, no crawl,
    no /cmd_vel, whatever the geometric skip test would have said."""
    ctx = make_ctx()
    start, end = ctx["target_scan_wall"]
    state._segments = [(start, end)]
    # A scan pose deliberately far from the base, so only the knob can skip.
    state._scan_poses = [(BASE[0] + 3.0, BASE[1] + 3.0, YAW + 1.0)]
    state._seg_idx = 0
    state._seg_phase = "transit"
    state._base_xy_yaw_map = lambda _ctx: (BASE[0], BASE[1], YAW)
    state._start_sweep_crawl = lambda *a, **kw: pytest.fail("crawl started")
    state._send_base_goal = lambda *a, **kw: pytest.fail("Nav2 goal sent")

    state._run_scan(ctx)

    assert state._seg_phase == "line_column"
    assert state.column_commanded is False
    assert ctx["_cmd_vel_pub"].sent == []
    assert state._pending_transit is None


def test_unparked_transit_off_pose_still_transits(state):
    """Same setup with the knob off: the base is off the scan pose, so the
    ordinary path queues a transit (column first). Guards that the parked
    branch did not swallow the normal behaviour."""
    ctx = make_ctx(scan_wall_assume_parked=False)
    start, end = ctx["target_scan_wall"]
    state._segments = [(start, end)]
    state._scan_poses = [(BASE[0] + 3.0, BASE[1] + 3.0, YAW + 1.0)]
    state._seg_idx = 0
    state._seg_phase = "transit"
    state._base_xy_yaw_map = lambda _ctx: (BASE[0], BASE[1], YAW)

    state._run_scan(ctx)

    assert state._seg_phase == "transit_column"
    assert state._pending_transit is not None


# ---------------------------------------------------------------------------
# scan_world_frame: the sweep without a map
# ---------------------------------------------------------------------------

def test_world_frame_is_map_unless_told_otherwise():
    """A mission never sets it; the default must stay map or every recorded
    pose_map silently changes frame."""
    assert world_frame({}) == "map"
    assert world_frame({"scan_world_frame": "odom"}) == "odom"


def test_sweep_goal_and_axis_follow_the_world_frame(state):
    """The executor transforms goal.frame_id into arm_base itself, so the goal
    must name the frame the partition was computed in. And a sweep measured in
    that same frame needs no TF to express its axis."""
    ctx = make_ctx(scan_world_frame="odom", sweep_executor_proc=None)
    sent = {}
    state._sweep_client = types.SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda goal, feedback_callback: (
            sent.setdefault("goal", goal),
            types.SimpleNamespace(add_done_callback=lambda cb: None))[1],
    )
    state._segments = [ctx["target_scan_wall"]]
    state._seg_idx = 0
    state.current_line_z = 1.0
    assert state._send_sweep_goal(ctx, *ctx["target_scan_wall"]) is True
    assert sent["goal"].frame_id == "odom"
    # Identity when the sweep is measured in the world frame itself...
    assert state._axis_in_frame(ctx, "odom", (0.0, 1.0)) == (0.0, 1.0, 0.0)
    # ...and no silent fallback to map: without TF, another frame is a None.
    assert state._axis_in_frame(ctx, "arm_base", (0.0, 1.0)) is None
    assert state._gpr_trigger_frame(make_ctx(scan_world_frame="odom",
                                             sweep_use_arm=False)) == "odom"


def test_sampler_records_the_world_pose_in_the_configured_frame():
    """pose_map is looked up in the frame the sweep was configured with, so an
    odom run records odom poses next to arm_base ones -- and never asks TF for
    a map frame that does not exist."""
    asked = []

    def pose_fn(frame, timeout):
        asked.append(frame)
        return (1.0, 2.0, 3.0)

    s = HyperspectralSampler()
    s._ref, s._world, s._pose_fn = "arm_base", "odom", pose_fn
    assert s._map_pose((0.0, 0.0, 0.0)) == (1.0, 2.0, 3.0)
    assert asked == ["odom"]
    s._ref = "odom"
    assert s._map_pose((4.0, 5.0, 6.0)) == (4.0, 5.0, 6.0)
    assert asked == ["odom"]              # same frame: no lookup at all


def test_bootstrap_looks_the_base_up_in_the_world_frame():
    """The in-front wall is built from <world>->base; in an odom run that
    lookup must not touch map."""
    from task_planner_fsm.fsm_node import RobotFSMNode

    looked = []

    class _TF:
        def can_transform(self, target, source, when):
            looked.append((target, source))
            return target == "odom" and source == "turret_footprint"

        def lookup_transform(self, target, source, when):
            t = types.SimpleNamespace(x=2.0, y=1.0, z=0.0)
            q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(YAW / 2), w=math.cos(YAW / 2))
            return types.SimpleNamespace(transform=types.SimpleNamespace(translation=t, rotation=q))

    fake = types.SimpleNamespace(ctx={"scan_world_frame": "odom"}, tf_buffer=_TF(),
                                 get_logger=lambda: _Logger())
    import task_planner_fsm.fsm_node as fsm_node
    orig = fsm_node.rclpy.spin_once
    fsm_node.rclpy.spin_once = lambda node, timeout_sec: None
    try:
        found = RobotFSMNode._wait_for_base_pose(fake, 1.0)
    finally:
        fsm_node.rclpy.spin_once = orig
    assert found[0] == "turret_footprint"
    assert found[1][0] == pytest.approx(2.0) and found[1][2] == pytest.approx(YAW)
    assert all(target == "odom" for target, _ in looked)


# ---------------------------------------------------------------------------
# fsm_stop_after
# ---------------------------------------------------------------------------

class _Step(State):
    """A state that asks for `nxt` on its first tick."""

    def __init__(self, name, nxt):
        super().__init__(name)
        self.nxt = nxt
        self.entered = 0

    def on_enter(self, ctx):
        self.entered += 1

    def check_transition(self, ctx):
        return self.nxt


def machine(stop_after, states, initial):
    ctx = {"node": _Node(), "fsm_stop_after": stop_after,
           # Keep the process-wide atexit/signal hooks out of the test run.
           "_cleanup_installed": True}
    return StateMachine(states, initial_state=initial, ctx=ctx), ctx


def test_stop_after_ends_at_finished_instead_of_the_next_state():
    scan = _Step("ScanWall", "SensorDataProcessing")
    proc = _Step("SensorDataProcessing", "ArmFolding")
    fin = _Step("Finished", None)
    m, _ = machine("ScanWall", [scan, proc, fin], "ScanWall")
    m.step()
    assert m.current_state is fin
    assert proc.entered == 0


def test_stop_after_leaves_self_loops_alone():
    """ScanWall re-enters itself for the next line height; that is not
    'the onward transition' and must still happen."""
    scan = _Step("ScanWall", "ScanWall")
    fin = _Step("Finished", None)
    m, _ = machine("ScanWall", [scan, fin], "ScanWall")
    m.step()
    assert m.current_state is scan
    assert scan.entered == 2


def test_stop_after_leaves_the_error_path_alone():
    """A failing stop state still retries and still reaches Error; stopping
    early must never hide a failure as a clean finish."""
    scan = _Step("ScanWall", "Error")
    err = _Step("Error", None)
    fin = _Step("Finished", None)
    m, _ = machine("ScanWall", [scan, err, fin], "ScanWall")
    m.step()                       # retry once, in place
    assert m.current_state is scan
    m.step()                       # then Error
    assert m.current_state is err


def test_no_stop_after_runs_through():
    scan = _Step("ScanWall", "SensorDataProcessing")
    proc = _Step("SensorDataProcessing", None)
    fin = _Step("Finished", None)
    m, _ = machine(None, [scan, proc, fin], "ScanWall")
    m.step()
    assert m.current_state is proc


# ---------------------------------------------------------------------------
# The tool's argv
# ---------------------------------------------------------------------------

def parsed(**over):
    ns = dict(sim=False, line_z=[1.0], length=0.8, stop_after="ScanWall",
              no_launch_stack=False, spacing=None, min_period=None, speed=None,
              service=DEFAULT_SERVICE, world_frame="odom")
    ns.update(over)
    return argparse.Namespace(**ns)


def params_of(argv):
    """{name: value} of every `-p name:=value` in argv."""
    out = {}
    for i, tok in enumerate(argv):
        if tok == "-p":
            name, value = argv[i + 1].split(":=", 1)
            out[name] = value
    return out


def flag(argv, name):
    return argv[argv.index(name) + 1]


def test_the_run_enters_at_arm_unfolding_and_stops_at_scan_wall():
    argv = build_fsm_argv(parsed(), [])
    assert flag(argv, "--initial-state") == "ArmUnfolding"
    assert flag(argv, "--scan-phase") == "1"
    assert flag(argv, "--wall-source") == "in-front"
    assert flag(argv, "--stop-after") == "ScanWall"
    assert "--no-launch-stack" not in argv


def test_hyperspectral_on_gpr_off_front_pose_base_parked():
    p = params_of(build_fsm_argv(parsed(), []))
    assert p["hyperspectral_enabled"] == "true"
    assert p["gpr_enabled"] == "false"
    assert p["scan_wall_unfolded_pose"] == "unfolded_front_fsm"
    assert p["scan_wall_assume_parked"] == "true"
    assert p["sweep_use_arm"] == "true"
    assert p["nav_face_wall"] == "true"
    # Triggers are NOT switched off: the hyperspectral sampler shares their
    # sweep frame and axis, and they cost nothing but a topic.
    assert "gpr_trigger_enabled" not in p


def test_the_bench_runs_in_odom_and_does_not_wait_for_rtabmap():
    """No map, no localisation: the wall, the sweep goal and the recorded
    poses live in odom, and readiness is gated on what the sweep needs rather
    than on rtabmap's odometry topic."""
    p = params_of(build_fsm_argv(parsed(), []))
    assert p["scan_world_frame"] == "odom"
    assert p["stack_ready_topics"] == "[/tf,/joint_states]"
    p = params_of(build_fsm_argv(parsed(world_frame="map"), []))
    assert p["scan_world_frame"] == "map"


def test_wall_geometry_and_heights_are_forwarded():
    p = params_of(build_fsm_argv(parsed(line_z=[1.4, 0.9], length=0.6), []))
    assert p["bootstrap_wall_length_m"] == "0.600"
    assert p["bootstrap_wall_lines_z"] == "[1.400,0.900]"


def test_optional_knobs_only_appear_when_given():
    p = params_of(build_fsm_argv(parsed(), []))
    for name in ("hyperspectral_sample_spacing_m", "hyperspectral_min_sample_period_s",
                 "sweep_speed_mps", "hyperspectral_service"):
        assert name not in p
    p = params_of(build_fsm_argv(
        parsed(spacing=0.08, min_period=2.0, speed=0.04, service="/cam/measure"), []))
    assert p["hyperspectral_sample_spacing_m"] == "0.0800"
    assert p["hyperspectral_min_sample_period_s"] == "2.000"
    assert p["sweep_speed_mps"] == "0.0400"
    assert p["hyperspectral_service"] == "/cam/measure"


def test_operator_ros_args_join_the_same_group_and_win():
    """Whatever follows the tool's options reaches fsm_node; a leading
    --ros-args is absorbed so there is one parameter group, and because the
    operator's -p comes last it overrides ours."""
    argv = build_fsm_argv(parsed(), ["--ros-args", "-p", "sweep_speed_mps:=0.03",
                                     "-p", "scan_wall_assume_parked:=false"])
    assert argv.count("--ros-args") == 1
    assert argv.index("--ros-args") < argv.index("-p")
    p_idx = [i for i, t in enumerate(argv) if t == "-p"]
    assert argv[p_idx[-1] + 1] == "scan_wall_assume_parked:=false"
    assert argv[p_idx[-2] + 1] == "sweep_speed_mps:=0.03"


def test_sim_stop_after_and_attach_flags():
    argv = build_fsm_argv(parsed(sim=True, stop_after="SensorDataProcessing",
                                 no_launch_stack=True), [])
    assert flag(argv, "--sim") == "true"
    assert flag(argv, "--stop-after") == "SensorDataProcessing"
    assert "--no-launch-stack" in argv


def test_fsm_node_accepts_the_composed_argv():
    """The tool's flags must be flags fsm_node actually parses; a typo here
    would surface as an argparse error on the robot, after the preflight."""
    from task_planner_fsm import fsm_node
    argv = build_fsm_argv(parsed(no_launch_stack=True), ["-p", "x:=1"])
    # Reuse fsm_node.main's own parser definition by parsing the way it does.
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--sim", choices=["true", "false"])
    parser.add_argument("--initial-state", choices=fsm_node.FSM_STATE_ORDER)
    parser.add_argument("--scan-phase", type=int, choices=[1, 2])
    parser.add_argument("--wall-source", choices=list(fsm_node.WALL_SOURCES))
    parser.add_argument("--no-launch-stack", action="store_true")
    parser.add_argument("--stop-after", choices=fsm_node.FSM_STATE_ORDER)
    args, remaining = parser.parse_known_args(argv)
    assert args.wall_source == "in-front"
    assert args.no_launch_stack is True
    assert remaining[0] == "--ros-args"


# ---------------------------------------------------------------------------
# fsm_node's in-front bootstrap, off a stubbed TF
# ---------------------------------------------------------------------------

def test_in_front_bootstrap_builds_the_wall_from_the_base_pose_and_parks():
    from task_planner_fsm.fsm_node import RobotFSMNode

    fake = types.SimpleNamespace(
        ctx={"bootstrap_wall_lines_z": [1.2], "bootstrap_wall_length_m": 0.8},
        get_logger=lambda: _Logger(),
        _wait_for_base_pose=lambda timeout: ("turret_footprint", (BASE[0], BASE[1], YAW)),
        _prompt_wall_lines=lambda idx: pytest.fail("must not prompt when heights are given"),
    )
    fake._bootstrap_wall_lines_z = lambda: RobotFSMNode._bootstrap_wall_lines_z(fake)

    w = RobotFSMNode._wall_in_front_of_robot(fake)

    assert fake.ctx["scan_wall_assume_parked"] is True
    ctx = make_ctx(wall_inward_normals=[w["inward_normal"]])
    pose = partition_scan_pose(ctx, "test", *w["scan_line"])
    assert pose[0] == pytest.approx(BASE[0], abs=1e-9)
    assert pose[1] == pytest.approx(BASE[1], abs=1e-9)
    assert w["scan_lines_z"] == [1.2]


def test_in_front_bootstrap_honours_an_explicit_parked_override():
    """setdefault, not assignment: an operator who passes
    -p scan_wall_assume_parked:=false gets the geometric skip test instead."""
    from task_planner_fsm.fsm_node import RobotFSMNode

    fake = types.SimpleNamespace(
        ctx={"bootstrap_wall_lines_z": 1.0, "scan_wall_assume_parked": False},
        get_logger=lambda: _Logger(),
        _wait_for_base_pose=lambda timeout: ("turret_footprint", (0.0, 0.0, 0.0)),
    )
    fake._bootstrap_wall_lines_z = lambda: RobotFSMNode._bootstrap_wall_lines_z(fake)
    RobotFSMNode._wall_in_front_of_robot(fake)
    assert fake.ctx["scan_wall_assume_parked"] is False


def test_in_front_bootstrap_fails_loudly_without_tf():
    from task_planner_fsm.fsm_node import RobotFSMNode

    fake = types.SimpleNamespace(
        ctx={"bootstrap_wall_lines_z": [1.0], "bootstrap_tf_timeout_s": 0.0},
        get_logger=lambda: _Logger(),
        _wait_for_base_pose=lambda timeout: None,
    )
    with pytest.raises(RuntimeError):
        RobotFSMNode._wall_in_front_of_robot(fake)


def test_bootstrap_heights_accept_a_scalar_or_a_list_and_sort():
    from task_planner_fsm.fsm_node import RobotFSMNode
    fake = types.SimpleNamespace(ctx={"bootstrap_wall_lines_z": 1.3})
    assert RobotFSMNode._bootstrap_wall_lines_z(fake) == [1.3]
    fake.ctx["bootstrap_wall_lines_z"] = [1.4, 0.9]
    assert RobotFSMNode._bootstrap_wall_lines_z(fake) == [0.9, 1.4]


def test_a_failed_in_front_bootstrap_stops_the_stack_and_refuses_to_start(monkeypatch):
    """The first state's on_enter clears error_triggered, so a bootstrap that
    merely flagged the error would still unfold the arm with no wall to sweep.
    The in-front path raises instead -- after stopping whatever it launched."""
    from task_planner_fsm import fsm_node
    from task_planner_fsm.fsm_node import BootstrapError, RobotFSMNode

    stopped = []
    monkeypatch.setattr(fsm_node, "stop_all", lambda ctx: stopped.append(ctx))
    fake = types.SimpleNamespace(ctx={"node": _Node()}, get_logger=lambda: _Logger())

    with pytest.raises(BootstrapError):
        RobotFSMNode._abort_bootstrap(fake, "no TF")
    assert stopped == [fake.ctx]
