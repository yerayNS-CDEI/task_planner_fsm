"""Restarting the FSM from Error/Finished without restarting the node or the stack.

`/fsm/restart` (a state name, or JSON with `state` and optionally `scan_phase`,
`wall_source`, `stop_after`) starts a new run at GeometryReconstruction or
later, or at ObjectID, while the robot stack stays up. The run context is reset
to the node's baseline, but live subscription data and ROS/process handles are
kept.

Run with:

    python3 -m pytest test/test_fsm_restart.py -v
"""

import json
import types

import pytest
from rclpy.publisher import Publisher

from task_planner_fsm import fsm_node
from task_planner_fsm.fsm_node import (
    RESTART_REDETECT_STATES,
    RESTART_TARGET_STATES,
    BootstrapError,
    RobotFSMNode,
)
from task_planner_fsm.machine import StateMachine
from task_planner_fsm.state import State
from task_planner_fsm.states.wall_target_selection import WallTargetSelection


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(("info", msg))

    def warn(self, msg, **kw):
        self.lines.append(("warn", msg))

    def error(self, msg, **kw):
        self.lines.append(("error", msg))


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


class _Rec(State):
    """Records what the machine calls on it."""

    def __init__(self, name, nxt=None):
        super().__init__(name)
        self.nxt = nxt
        self.calls = []

    def on_enter(self, ctx):
        self.calls.append("enter")

    def on_exit(self, ctx):
        self.calls.append("exit")

    def reset_run(self, ctx):
        self.calls.append("reset_run")

    def check_transition(self, ctx):
        return self.nxt


def machine(states, initial):
    events = []
    ctx = {
        "node": _Node(),
        # Keep the process-wide atexit/signal hooks out of the test run.
        "_cleanup_installed": True,
        "publish_fsm_event": lambda ev, **kw: events.append((ev, kw)),
    }
    return StateMachine(states, initial_state=initial, ctx=ctx), ctx, events


# ---------------------------------------------------------------------------
# StateMachine.restart
# ---------------------------------------------------------------------------

def test_restart_leaves_the_terminal_state_and_enters_the_target_fresh():
    geo, err = _Rec("GeometryReconstruction"), _Rec("Error")
    m, ctx, events = machine([geo, err], "Error")
    m._retried_current_state = True
    ctx["last_state"] = "ScanWall"

    m.restart("GeometryReconstruction", reason="test")

    assert m.current_state is geo
    assert err.calls == ["enter", "exit", "reset_run"]
    assert geo.calls == ["reset_run", "enter"]
    assert m._retried_current_state is False
    assert ctx["fsm_initial_state"] == "GeometryReconstruction"
    assert ctx["last_state"] is None
    assert ("restarted", "Error") in [(ev, kw["state_name"]) for ev, kw in events]


def test_restart_is_refused_while_running():
    geo, fin = _Rec("GeometryReconstruction"), _Rec("Finished")
    m, _, _ = machine([geo, fin], "GeometryReconstruction")
    with pytest.raises(RuntimeError):
        m.restart("Finished")
    assert m.current_state is geo


def test_restart_into_a_crashing_on_enter_takes_the_retry_path_not_the_node_down():
    class Boom(_Rec):
        def on_enter(self, ctx):
            super().on_enter(ctx)
            if self.calls.count("enter") == 1:
                raise RuntimeError("boom")

    boom, err = Boom("ScanWall"), _Rec("Error")
    m, ctx, events = machine([boom, err], "Error")
    m.restart("ScanWall")
    # First entry raised; the machine retried the state once, as step() would.
    assert m.current_state is boom
    assert boom.calls.count("enter") == 2
    assert "exception" in [ev for ev, _ in events]


def test_wall_target_selection_forgets_the_previous_runs_walls():
    wts = WallTargetSelection("WallTargetSelection")
    wts.scanned_walls_idx = [0, 2]
    wts.scanned_panels_idx = [1]
    wts.reset_run({})
    assert wts.scanned_walls_idx == [] and wts.scanned_panels_idx == []


# ---------------------------------------------------------------------------
# The node side
# ---------------------------------------------------------------------------

RESTART_METHODS = (
    "_parse_restart_request",
    "restart_callback",
    "_reject_restart",
    "_reset_ctx_for_restart",
    "_perform_restart",
    "_abort_bootstrap",
)


def fake_node(current="Error", ctx=None, baseline=None, bootstrap=None):
    node = types.SimpleNamespace()
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    node.events = []
    node.publish_fsm_event = lambda ev, **kw: node.events.append((ev, kw))
    node.publish_fsm_status = lambda snap: None
    node.set_fsm_status = lambda state, **kw: node.ctx["_fsm_status"].update(kw)
    node.graphs = []
    node.publish_fsm_graph = lambda: node.graphs.append(node.initial_state)
    node.RESTART_FIELDS = RobotFSMNode.RESTART_FIELDS
    node.restarts = []
    node.machine = types.SimpleNamespace(
        current_state=types.SimpleNamespace(name=current),
        restart=lambda target, reason: node.restarts.append(target),
    )
    node.ctx = ctx if ctx is not None else {}
    node._ctx_baseline = baseline if baseline is not None else {}
    node._pending_restart = None
    node._restarting = False
    node._run_index = 1
    node._stack_ensured = True
    node.wall_source = node._default_wall_source = "yaml"
    node.initial_state = "Initialization"
    node._bootstrap_context_for_initial_state = bootstrap or (lambda state, phase: None)
    for name in RESTART_METHODS:
        setattr(node, name, types.MethodType(getattr(RobotFSMNode, name), node))
    return node


def msg(data):
    return types.SimpleNamespace(data=data if isinstance(data, str) else json.dumps(data))


def test_targets_start_at_object_id_and_exclude_the_mapping_and_terminal_states():
    assert RESTART_TARGET_STATES[0] == "ObjectID"
    assert "GeometryReconstruction" in RESTART_TARGET_STATES
    for refused in ("Initialization", "CreateMap", "Error", "Finished"):
        assert refused not in RESTART_TARGET_STATES


@pytest.mark.parametrize("data, expected", [
    ("GeometryReconstruction", {"state": "GeometryReconstruction"}),
    (" ScanWall\n", {"state": "ScanWall"}),
    ({"state": "ScanWall", "wall_source": "in-front", "stop_after": "ScanWall"},
     {"state": "ScanWall", "wall_source": "in-front", "stop_after": "ScanWall"}),
    ({"state": "HomePosition", "scan_phase": 2, "stop_after": None},
     {"state": "HomePosition", "scan_phase": 2, "stop_after": None}),
])
def test_parse_accepts_a_bare_state_or_json(data, expected):
    node = fake_node()
    raw = data if isinstance(data, str) else json.dumps(data)
    assert node._parse_restart_request(raw) == expected


@pytest.mark.parametrize("data", [
    "CreateMap",
    "Initialization",
    "Error",
    "Nope",
    {"state": "ScanWall", "walls": [1]},
    {"state": "ScanWall", "scan_phase": 3},
    {"state": "ScanWall", "wall_source": "map"},
    {"state": "ScanWall", "stop_after": "Nope"},
    "[1, 2]",
])
def test_parse_refuses_bad_requests(data):
    node = fake_node()
    raw = data if isinstance(data, str) else json.dumps(data)
    with pytest.raises(ValueError):
        node._parse_restart_request(raw)


def test_callback_queues_from_a_terminal_state():
    node = fake_node("Finished")
    node.restart_callback(msg("GeometryReconstruction"))
    assert node._pending_restart == {"state": "GeometryReconstruction"}
    assert node.events == []


def test_callback_refuses_while_running_and_when_one_is_pending():
    node = fake_node("ScanWall")
    node.restart_callback(msg("GeometryReconstruction"))
    assert node._pending_restart is None
    assert node.events[-1][0] == "restart_rejected"

    node = fake_node("Error")
    node.restart_callback(msg("GeometryReconstruction"))
    node.restart_callback(msg("ScanWall"))
    assert node._pending_restart["state"] == "GeometryReconstruction"
    assert node.events[-1][0] == "restart_rejected"


def test_callback_reports_a_bad_request_instead_of_raising():
    node = fake_node("Error")
    node.restart_callback(msg("CreateMap"))
    assert node._pending_restart is None
    assert "cannot restart at 'CreateMap'" in node.events[-1][1]["summary"]


# ---------------------------------------------------------------------------
# Context reset
# ---------------------------------------------------------------------------

def run_ctx():
    baseline = {
        "start": False,
        "error_triggered": False,
        "last_state": None,
        "fsm_start_wall_time": 100.0,
        "fsm_stop_after": "ScanWall",
        "scan_line_offset": 0.6,      # a -p override
        "_fsm_status": {},
    }
    pub = Publisher.__new__(Publisher)
    ctx = dict(baseline)
    ctx.update({
        # the previous run
        "start": True,
        "error_triggered": True,
        "fatal": True,
        "last_state": "ScanWall",
        "walls_data": [{"scan_line": ((0, 0, 0), (1, 0, 0))}],
        "current_wall_index": 1,
        "walls_left": 0,
        "sensor_session_id": "20260925_101010",
        "fsm_error_summary": "ScanWall failed",
        "_partition_plan": object(),
        "_fsm_status": {"state": "Error", "progress": {"current": 2, "total": 3}},
        # live subscription data and handles
        "global_costmap": "latched-costmap",
        "base_position": "pose",
        "_procs": {"nav_sim": "proc"},
        "_cleanup_installed": True,
        "_cmd_vel_pub": pub,
    })
    return ctx, baseline, pub


def test_reset_drops_the_run_keeps_live_data_and_handles_in_place():
    ctx, baseline, pub = run_ctx()
    node = fake_node(ctx=ctx, baseline=baseline)
    node._run_index = 2
    same = node.ctx

    node._reset_ctx_for_restart("GeometryReconstruction")

    assert node.ctx is same
    for gone in ("fatal", "walls_data", "current_wall_index", "walls_left",
                 "sensor_session_id", "fsm_error_summary", "_partition_plan"):
        assert gone not in ctx, gone
    assert ctx["start"] is False and ctx["error_triggered"] is False
    assert ctx["last_state"] is None
    assert ctx["scan_line_offset"] == 0.6
    assert ctx["fsm_stop_after"] == "ScanWall"
    assert ctx["_fsm_status"] == {}
    assert ctx["global_costmap"] == "latched-costmap"
    assert ctx["base_position"] == "pose"
    assert ctx["_procs"] == {"nav_sim": "proc"}
    assert ctx["_cleanup_installed"] is True
    assert ctx["_cmd_vel_pub"] is pub
    assert ctx["fsm_run_index"] == 2


def test_reset_keeps_the_wall_clock_unless_the_detector_runs_again():
    ctx, baseline, _ = run_ctx()
    node = fake_node(ctx=ctx, baseline=baseline)
    node._reset_ctx_for_restart("GeometryReconstruction")
    # The detected_walls.yaml of this session must still pass the staleness guard.
    assert ctx["fsm_start_wall_time"] == 100.0

    assert "ObjectID" in RESTART_REDETECT_STATES
    node._reset_ctx_for_restart("ObjectID")
    assert ctx["fsm_start_wall_time"] > 100.0


# ---------------------------------------------------------------------------
# _perform_restart
# ---------------------------------------------------------------------------

def test_perform_restart_bootstraps_then_moves_the_machine():
    seen = []

    def bootstrap(state, phase):
        seen.append((state, phase, node.wall_source, node._restarting,
                     node._stack_ensured, node.ctx.get("fsm_stop_after")))

    ctx, baseline, _ = run_ctx()
    node = fake_node(ctx=ctx, baseline=baseline, bootstrap=bootstrap)
    node._perform_restart({"state": "ScanWall", "scan_phase": 1,
                           "wall_source": "in-front", "stop_after": None})

    assert seen == [("ScanWall", 1, "in-front", True, False, None)]
    assert node.restarts == ["ScanWall"]
    assert node.initial_state == "ScanWall"
    assert node.graphs == ["ScanWall"]
    assert node._restarting is False
    assert node._run_index == 2


def test_wall_source_override_is_for_one_run_only():
    node = fake_node(ctx={}, baseline={})
    node._perform_restart({"state": "ScanWall", "wall_source": "in-front"})
    assert node.wall_source == "in-front"
    node.machine.current_state.name = "Finished"
    node._perform_restart({"state": "ScanWall"})
    assert node.wall_source == "yaml"


def test_failed_bootstrap_keeps_the_machine_and_the_stack(monkeypatch):
    stopped = []
    monkeypatch.setattr(fsm_node, "stop_all", lambda ctx: stopped.append(True))

    def bootstrap(state, phase):
        node._abort_bootstrap("No map->base transform")

    node = fake_node(ctx={}, baseline={"_fsm_status": {}}, bootstrap=bootstrap)
    node._perform_restart({"state": "ScanWall"})

    assert stopped == []
    assert node.restarts == []
    assert node._restarting is False
    assert node.ctx["fsm_error_summary"].startswith("Restart at ScanWall failed")
    assert node.events[-1][0] == "restart_rejected"


def test_stack_not_ready_refuses_the_restart():
    def bootstrap(state, phase):
        node.ctx["error_triggered"] = True     # what _ensure_nav_sim_running does

    node = fake_node(ctx={}, baseline={"_fsm_status": {}}, bootstrap=bootstrap)
    node._perform_restart({"state": "GeometryReconstruction"})
    assert node.restarts == []
    assert node.ctx["error_triggered"] is False
    assert "robot stack is not ready" in node.events[-1][1]["summary"]


def test_abort_bootstrap_on_first_start_still_stops_everything(monkeypatch):
    stopped = []
    monkeypatch.setattr(fsm_node, "stop_all", lambda ctx: stopped.append(True))
    node = fake_node()
    with pytest.raises(BootstrapError):
        node._abort_bootstrap("nope")
    assert stopped == [True]


def test_restart_after_an_arm_state_error_warns():
    node = fake_node(ctx={"last_state": "ScanWall"}, baseline={})
    node._perform_restart({"state": "GeometryReconstruction"})
    assert any(level == "warn" and "arm/column" in text for level, text in node.logger.lines)
    assert node.restarts == ["GeometryReconstruction"]
