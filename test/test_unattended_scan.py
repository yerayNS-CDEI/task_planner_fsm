"""Unattended wall scanning: no wall prompts, and no motion before localization.

- scan_all_walls: ComputeWallPoints and the bootstrap take every detected wall
  at scan_all_walls_lines_z without reading stdin, and NavigateToTarget skips a
  wall it cannot reach instead of ending the run.
- A start that brings up the localization stack waits in Localize until
  rtabmap's fix (kept in ctx by fsm_node) is good; ObjectID uses the same gate.

Run with:

    python3 -m pytest test/test_unattended_scan.py -v
"""

import math
import types

import pytest

from task_planner_fsm import fsm_node
from task_planner_fsm.fsm_node import LOCALIZATION_GATED_START_STATES, RobotFSMNode
from task_planner_fsm.states.compute_wall_points import ComputeWallPoints
from task_planner_fsm.states.localize import Localize
from task_planner_fsm.states.navigate import NavigateToTarget
from task_planner_fsm.states.wall_target_selection import WallTargetSelection
from task_planner_fsm.telemetry import build_fsm_graph_payload
from task_planner_fsm.utils import localization
from task_planner_fsm.utils.base_pose import base_pose_map
from task_planner_fsm.utils.localization import FIX_KEY, LocalizationWait
from task_planner_fsm.utils.wall_selection import auto_lines_z, skip_current_wall


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(("info", msg))

    def warn(self, msg, **kw):
        self.lines.append(("warn", msg))

    def error(self, msg, **kw):
        self.lines.append(("error", msg))


def make_ctx(**extra):
    logger = _Logger()
    node = types.SimpleNamespace(get_logger=lambda: logger)
    ctx = {"node": node, "_fsm_status": {}}
    ctx.update(extra)
    return ctx


def no_stdin(monkeypatch):
    monkeypatch.setattr("builtins.input", lambda *a: pytest.fail("prompted on stdin"))


GOOD = {"x": 1.0, "y": 2.0, "xy_std": 0.05, "yaw_std": 0.01, "stamp": 0.0}
LOOSE = {"x": 1.0, "y": 2.0, "xy_std": 0.9, "yaw_std": 0.01, "stamp": 0.0}


# ---------------------------------------------------------------------------
# Localization fix and gate
# ---------------------------------------------------------------------------

def pose_msg(x, y, cov_xx, cov_yy, cov_yaw):
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = cov_xx, cov_yy, cov_yaw
    pos = types.SimpleNamespace(x=x, y=y, z=0.0)
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=pos), covariance=cov))


def test_record_fix_turns_the_covariance_into_std_devs():
    ctx = make_ctx()
    localization.record_fix(ctx, pose_msg(3.0, -1.0, 0.0016, 0.0009, 0.0004))
    fix = ctx[FIX_KEY]
    assert (fix["x"], fix["y"]) == (3.0, -1.0)
    assert fix["xy_std"] == pytest.approx(0.05)
    assert fix["yaw_std"] == pytest.approx(0.02)
    assert localization.is_localized(ctx)

    # rtabmap's "not localized yet" covariance
    localization.record_fix(ctx, pose_msg(0.0, 0.0, 9999, 9999, 9999))
    assert not localization.is_localized(ctx)
    localization.forget_fix(ctx)
    assert FIX_KEY not in ctx and not localization.is_localized(ctx)


def test_wait_holds_until_a_good_fix_then_goes():
    ctx = make_ctx()
    wait = LocalizationWait("T", "moving")
    wait.start()
    assert wait.poll(ctx) == "pending"
    ctx[FIX_KEY] = dict(LOOSE)
    assert wait.poll(ctx) == "pending"
    ctx[FIX_KEY] = dict(GOOD)
    assert wait.poll(ctx) == "ok"


def test_wait_times_out_to_bad_or_goes_when_not_required():
    wait = LocalizationWait("T", "moving")
    wait.started = -1e9     # long ago
    assert wait.poll(make_ctx(localization_timeout=1.0)) == "bad"
    assert wait.poll(make_ctx(localization_timeout=1.0, localization_required=False)) == "ok"
    assert wait.poll(make_ctx(localization_check=False)) == "ok"


def test_localize_state_enters_the_real_start_once_localized():
    ctx = make_ctx(localize_next_state="NavigateToTarget")
    state = Localize("Localize")
    state.on_enter(ctx)
    state.run(ctx)
    assert state.check_transition(ctx) is None
    ctx[FIX_KEY] = dict(GOOD)
    state.run(ctx)
    assert state.check_transition(ctx) == "NavigateToTarget"


def test_localize_state_fails_after_the_timeout():
    ctx = make_ctx(localize_next_state="ScanWall", localization_timeout=0.0)
    state = Localize("Localize")
    state.on_enter(ctx)
    state.run(ctx)
    assert state.check_transition(ctx) == "Error"
    assert "not localized" in ctx["error_reason"]


def gate_node(ctx):
    node = types.SimpleNamespace(ctx=ctx)
    node._gated_start = types.MethodType(RobotFSMNode._gated_start, node)
    return node


@pytest.mark.parametrize("state", ["GeometryReconstruction", "NavigateToTarget", "ScanWall"])
def test_starts_on_the_localization_stack_wait_in_localize(state):
    node = gate_node(make_ctx())
    assert node._gated_start(state) == "Localize"
    assert node.ctx["localize_next_state"] == state


@pytest.mark.parametrize("state", ["Initialization", "CreateMap", "ObjectID", "SensorDataProcessing"])
def test_mapping_objectid_and_offline_starts_are_not_gated(state):
    # CreateMap builds the map it stands in; ObjectID gates itself.
    assert state not in LOCALIZATION_GATED_START_STATES
    assert gate_node(make_ctx())._gated_start(state) == state


def test_no_gate_when_already_localized_or_switched_off():
    assert gate_node(make_ctx(**{FIX_KEY: dict(GOOD)}))._gated_start("ScanWall") == "ScanWall"
    assert gate_node(make_ctx(localization_check=False))._gated_start("ScanWall") == "ScanWall"


def test_graph_shows_localize():
    ids = [n["id"] for n in build_fsm_graph_payload(fsm_node.FSM_STATE_ORDER + ["Localize"])["states"]]
    assert "Localize" in ids
    # Not offered as a start or restart target: it needs a state to lead to.
    assert "Localize" not in fsm_node.FSM_STATE_ORDER


# ---------------------------------------------------------------------------
# scan_all_walls
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("raw, expected", [
    (None, [1.5]),
    (2.2, [2.2]),
    ([2.2, 1.5], [1.5, 2.2]),
    ("1.5, 2.2", [1.5, 2.2]),
])
def test_auto_lines_z(raw, expected):
    ctx = {} if raw is None else {"scan_all_walls_lines_z": raw}
    assert auto_lines_z(ctx) == expected


DETECTED = [((0.0, 0.0, 0.0), (4.0, 0.0, 0.0)), ((4.0, 0.0, 0.0), (4.0, 3.0, 0.0))]
NORMALS = [(0.0, -1.0), (1.0, 0.0)]


def test_compute_wall_points_takes_every_detected_wall_without_asking(monkeypatch):
    no_stdin(monkeypatch)
    ctx = make_ctx(scan_all_walls=True, scan_all_walls_lines_z=[1.5, 2.2],
                   detected_walls=DETECTED, detected_wall_normals=NORMALS)
    state = ComputeWallPoints("ComputeWallPoints")
    state.on_enter(ctx)
    state.run(ctx)
    assert ctx["walls_left"] == 2
    assert len(ctx["walls_data"]) == 2      # the predefined fallbacks are left out
    assert all(w["scan_lines_z"] == [1.5, 2.2] for w in ctx["walls_data"])
    assert state.check_transition(ctx) == "ArmFolding"


def test_compute_wall_points_fails_when_nothing_was_detected(monkeypatch):
    no_stdin(monkeypatch)
    ctx = make_ctx(scan_all_walls=True)
    state = ComputeWallPoints("ComputeWallPoints")
    state.on_enter(ctx)
    state.run(ctx)
    assert state.check_transition(ctx) == "Error"
    assert "no walls were detected" in ctx["error_reason"]


def bootstrap_node(ctx):
    node = types.SimpleNamespace(ctx=ctx, _stdin_warned=False)
    node.get_logger = ctx["node"].get_logger
    for name in ("_safe_input", "_prompt_int", "_prompt_wall_lines", "_prompt_walls_data",
                 "_build_wall_data", "_load_bootstrap_walls", "_load_walls_from_yaml",
                 "_abort_bootstrap"):
        setattr(node, name, types.MethodType(getattr(RobotFSMNode, name), node))
    return node


def write_walls(tmp_path, n):
    path = tmp_path / "detected_walls.yaml"
    walls = "".join(
        f"  - id: {i}\n    p1: [{i}.0, 0.0]\n    p2: [{i}.0, 3.0]\n    normal: [1.0, 0.0]\n"
        for i in range(n)
    )
    path.write_text("walls:\n" + walls if n else "walls: []\n")
    return str(path)


def test_bootstrap_takes_every_wall_without_asking(monkeypatch, tmp_path):
    no_stdin(monkeypatch)
    monkeypatch.setattr(fsm_node, "wall_detection_enabled", lambda ctx: False)
    ctx = make_ctx(scan_all_walls=True,
                   geometry_reconstruction_wall_file_path=write_walls(tmp_path, 3))
    walls = bootstrap_node(ctx)._prompt_walls_data()
    assert len(walls) == 3
    assert all(w["scan_lines_z"] == [1.5] for w in walls)


def test_bootstrap_refuses_the_hardcoded_walls(monkeypatch, tmp_path):
    no_stdin(monkeypatch)
    monkeypatch.setattr(fsm_node, "wall_detection_enabled", lambda ctx: False)
    monkeypatch.setattr(fsm_node, "stop_all", lambda ctx: None)
    ctx = make_ctx(scan_all_walls=True,
                   geometry_reconstruction_wall_file_path=write_walls(tmp_path, 0))
    with pytest.raises(fsm_node.BootstrapError, match="no detected walls"):
        bootstrap_node(ctx)._prompt_walls_data()


def test_skip_counts_the_wall_as_done():
    ctx = make_ctx(current_wall_index=0, walls_left=2)
    assert skip_current_wall(ctx, "T", "unreachable") is True
    assert ctx["walls_left"] == 1 and not ctx.get("scan_done")
    ctx["current_wall_index"] = 1
    assert skip_current_wall(ctx, "T", "unreachable") is False
    assert ctx["scan_done"] is True
    assert [w["wall_index"] for w in ctx["skipped_walls"]] == [0, 1]


def navigate(ctx):
    state = NavigateToTarget("NavigateToTarget")
    ctx.setdefault("current_wall_index", 0)
    state._skip_or_fail(ctx, "no portion of the wall is reachable")
    return state


def test_navigate_skips_an_unreachable_wall_and_moves_on():
    ctx = make_ctx(scan_all_walls=True, walls_left=2)
    assert navigate(ctx).check_transition(ctx) == "WallTargetSelection"
    assert not ctx.get("error_triggered")


def test_navigate_goes_home_when_the_skipped_wall_was_the_last():
    ctx = make_ctx(scan_all_walls=True, walls_left=1)
    assert navigate(ctx).check_transition(ctx) == "ArmFolding"
    assert ctx["scan_done"] is True


def test_navigate_still_fails_a_wall_the_operator_picked():
    ctx = make_ctx(walls_left=1)
    assert navigate(ctx).check_transition(ctx) == "Error"
    assert ctx["walls_left"] == 1


class FakeTF:
    """tf2 buffer stand-in: map -> <frame> at (x, y, yaw), for the frames given."""

    def __init__(self, x, y, yaw=0.0, frames=("turret_footprint",)):
        self.frames = frames
        self.asked = []
        q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        t = types.SimpleNamespace(x=x, y=y, z=0.0)
        self.tf = types.SimpleNamespace(transform=types.SimpleNamespace(translation=t, rotation=q))

    def can_transform(self, target, source, *a):
        self.asked.append(source)
        return target == "map" and source in self.frames

    def lookup_transform(self, target, source, *a):
        return self.tf


def test_base_pose_map_reads_the_nav2_frame_first():
    tf = FakeTF(1.0, 2.0, yaw=0.5, frames=("base_link", "turret_footprint"))
    x, y, yaw = base_pose_map({"tf_buffer": tf})
    assert (x, y, yaw) == (1.0, 2.0, pytest.approx(0.5))
    assert tf.asked == ["turret_footprint"]
    assert base_pose_map({"tf_buffer": FakeTF(0, 0, frames=())}) is None
    assert base_pose_map({}) is None


TWO_WALLS = [
    {"scan_line": ((0.0, 0.0, 1.5), (1.0, 0.0, 1.5)), "inward_normal": (0.0, 1.0, 0.0)},
    {"scan_line": ((9.0, 0.0, 1.5), (10.0, 0.0, 1.5)), "inward_normal": (0.0, 1.0, 0.0)},
]


def select(ctx):
    state = WallTargetSelection("WallTargetSelection")
    state.on_enter(ctx)
    state.run(ctx)
    return state


def test_wall_selection_picks_the_nearest_wall_in_the_map_frame():
    # odom still has the robot at its start (by wall 0); rtabmap has placed it
    # at wall 1 in the map.
    ctx = make_ctx(scan_phase=1, walls_data=TWO_WALLS,
                   base_position=types.SimpleNamespace(x=0.0, y=0.5),
                   tf_buffer=FakeTF(9.5, 0.5))
    select(ctx)
    assert ctx["current_wall_index"] == 1


def test_wall_selection_waits_for_the_map_pose_then_fails():
    ctx = make_ctx(scan_phase=1, walls_data=TWO_WALLS,
                   base_position=types.SimpleNamespace(x=0.0, y=0.5),
                   tf_buffer=FakeTF(0, 0, frames=()))
    state = select(ctx)
    assert state.check_transition(ctx) is None      # never the odom pose
    ctx["wall_target_selection_pose_timeout_s"] = 0.0
    state.run(ctx)
    assert state.check_transition(ctx) == "Error"
    assert "map -> base" in ctx["error_reason"]


def test_wall_selection_does_not_repick_the_bootstrap_wall():
    # Started at ScanWall on wall 0, which the robot is standing at.
    ctx = make_ctx(scan_phase=1, walls_data=TWO_WALLS, current_wall_index=0,
                   tf_buffer=FakeTF(0.0, 0.5))
    select(ctx)
    assert ctx["current_wall_index"] == 1
