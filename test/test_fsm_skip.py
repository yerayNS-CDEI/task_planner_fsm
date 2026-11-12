"""Operator skip: leave the running state now and jump to a later one.

`/fsm/skip` (a state name, or JSON with `state`) is the UI's "Skip to" button.
The node first cancels every Nav2 goal and holds the machine for a tick, then
the machine runs the old state's on_exit, the node synthesizes the ctx the
skipped states would have left, and the target is entered. The run's ctx is
kept (unlike a restart).

Run with:

    python3 -m pytest test/test_fsm_skip.py -v
"""

import json
import types

import pytest

from task_planner_fsm.fsm_node import (
    SKIP_CANCEL_WAIT_TICKS,
    SKIP_TARGET_STATES,
    RobotFSMNode,
)
from task_planner_fsm.machine import StateMachine
from task_planner_fsm.state import State
from task_planner_fsm.utils.localization import FIX_KEY


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
    def __init__(self, name):
        super().__init__(name)
        self.calls = []

    def on_enter(self, ctx):
        self.calls.append("enter")

    def on_exit(self, ctx):
        self.calls.append("exit")

    def check_transition(self, ctx):
        return None


def machine(states, initial):
    events, transitions = [], []
    ctx = {
        "node": _Node(),
        "_cleanup_installed": True,
        "publish_fsm_event": lambda ev, **kw: events.append((ev, kw)),
        "publish_fsm_transition": lambda a, b, r: transitions.append((a, b, r)),
    }
    return StateMachine(states, initial_state=initial, ctx=ctx), ctx, events, transitions


# ---------------------------------------------------------------------------
# StateMachine.skip_to
# ---------------------------------------------------------------------------

def test_skip_exits_prepares_then_enters_and_keeps_the_run():
    oid, geo, err = _Rec("ObjectID"), _Rec("GeometryReconstruction"), _Rec("Error")
    m, ctx, events, transitions = machine([oid, geo, err], "ObjectID")
    ctx["walls_left"] = 3
    ctx["error_triggered"] = True
    order = []
    oid.on_exit = lambda c: order.append("exit")

    def prepare(target):
        order.append(("prepare", target))
        return target

    m.skip_to("GeometryReconstruction", prepare=prepare)

    assert order == ["exit", ("prepare", "GeometryReconstruction")]
    assert m.current_state is geo and geo.calls == ["enter"]
    assert ctx["walls_left"] == 3          # the run's ctx is kept
    assert ctx["last_state"] == "ObjectID"
    assert ctx["error_triggered"] is False
    assert transitions == [("ObjectID", "GeometryReconstruction", "operator_skip")]
    assert ("skipped", "ObjectID") in [(ev, kw["state_name"]) for ev, kw in events]


def test_prepare_may_redirect_through_localize():
    oid, loc, geo = _Rec("ObjectID"), _Rec("Localize"), _Rec("GeometryReconstruction")
    m, _, _, _ = machine([oid, loc, geo], "ObjectID")
    m.skip_to("GeometryReconstruction", prepare=lambda t: "Localize")
    assert m.current_state is loc


def test_failed_prepare_goes_to_error_without_reentering_the_old_state():
    oid, geo, err = _Rec("ObjectID"), _Rec("GeometryReconstruction"), _Rec("Error")
    m, ctx, _, _ = machine([oid, geo, err], "ObjectID")

    def prepare(target):
        raise RuntimeError("the robot stack is not ready")

    m.skip_to("GeometryReconstruction", prepare=prepare)
    assert m.current_state is err
    assert oid.calls == ["enter", "exit"]
    assert geo.calls == []
    assert "not ready" in ctx["fsm_error_summary"]


def test_skip_is_refused_from_a_terminal_state():
    geo, err = _Rec("GeometryReconstruction"), _Rec("Error")
    m, _, _, _ = machine([geo, err], "Error")
    with pytest.raises(RuntimeError):
        m.skip_to("GeometryReconstruction")
    assert m.current_state is err


# ---------------------------------------------------------------------------
# The node side
# ---------------------------------------------------------------------------

GOOD_FIX = {"x": 1.0, "y": 2.0, "xy_std": 0.05, "yaw_std": 0.01, "stamp": 100.0}

SKIP_METHODS = (
    "_parse_skip_request",
    "skip_callback",
    "_reject_skip",
    "_advance_skip",
    "_perform_skip",
    "_prepare_skip_target",
    "_gated_start",
)


class _Future:
    def __init__(self, done=False):
        self._done = done

    def done(self):
        return self._done


def fake_node(current="ObjectID", ctx=None, cancels=()):
    node = types.SimpleNamespace()
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    node.events = []
    node.publish_fsm_event = lambda ev, **kw: node.events.append((ev, kw))
    node.skips = []

    def skip_to(target, reason, prepare):
        node.skips.append(prepare(target))

    node.machine = types.SimpleNamespace(
        current_state=types.SimpleNamespace(name=current), skip_to=skip_to,
    )
    node.ctx = ctx if ctx is not None else {FIX_KEY: dict(GOOD_FIX)}
    node._pending_skip = None
    node._pending_restart = None
    node._skipping = False
    node._stack_ensured = True
    node.cancel_calls = 0

    def cancel():
        node.cancel_calls += 1
        return list(cancels)

    node._cancel_nav_goals = cancel
    node.calls = []
    node._ensure_nav_sim_running = lambda: node.calls.append("nav_sim")
    node._ensure_nav_client = lambda: node.calls.append("nav_client")
    node._prompt_phase1_target = lambda: node.calls.append("phase1_target")
    node._bootstrap_context_for_initial_state = (
        lambda state, phase: node.calls.append(("bootstrap", state)))
    for name in SKIP_METHODS:
        setattr(node, name, types.MethodType(getattr(RobotFSMNode, name), node))
    return node


def msg(data):
    return types.SimpleNamespace(data=data if isinstance(data, str) else json.dumps(data))


@pytest.mark.parametrize("data", ["GeometryReconstruction", {"state": "GeometryReconstruction"}])
def test_callback_queues_a_skip(data):
    node = fake_node("ObjectID")
    node.skip_callback(msg(data))
    assert node._pending_skip == {"from": "ObjectID", "state": "GeometryReconstruction"}
    assert node.events == []


@pytest.mark.parametrize("current, data", [
    ("Error", "GeometryReconstruction"),       # terminal: use a restart
    ("Finished", "GeometryReconstruction"),
    ("Localize", "GeometryReconstruction"),    # not localized yet
    ("ObjectID", "ObjectID"),                  # already there
    ("ObjectID", "CreateMap"),                 # not a target
    ("ObjectID", "Error"),
    ("ObjectID", {"state": "ScanWall", "scan_phase": 1}),
])
def test_callback_refuses(current, data):
    node = fake_node(current)
    node.skip_callback(msg(data))
    assert node._pending_skip is None
    assert node.events[-1][0] == "skip_rejected"


def test_only_one_skip_at_a_time():
    node = fake_node("ObjectID")
    node.skip_callback(msg("GeometryReconstruction"))
    node.skip_callback(msg("ScanWall"))
    assert node._pending_skip["state"] == "GeometryReconstruction"
    assert node.events[-1][0] == "skip_rejected"


def test_targets_are_the_restart_targets():
    assert "GeometryReconstruction" in SKIP_TARGET_STATES
    for refused in ("Initialization", "CreateMap", "Error", "Finished"):
        assert refused not in SKIP_TARGET_STATES


def test_holds_the_machine_until_nav2_answers_the_cancel():
    pending = _Future(done=False)
    node = fake_node("ObjectID", cancels=[pending])
    node.skip_callback(msg("GeometryReconstruction"))

    assert node._advance_skip() is False       # cancel sent, machine held
    assert node._advance_skip() is False       # no answer yet
    pending._done = True
    assert node._advance_skip() is True
    assert node.cancel_calls == 1
    assert node.skips == ["GeometryReconstruction"]
    assert node._pending_skip is None


def test_goes_on_when_nav2_never_answers():
    node = fake_node("ObjectID", cancels=[_Future(done=False)])
    node.skip_callback(msg("GeometryReconstruction"))
    held = 0
    while not node._advance_skip():
        held += 1
        assert held <= SKIP_CANCEL_WAIT_TICKS + 1
    assert node.skips == ["GeometryReconstruction"]


def test_no_nav2_means_no_hold():
    node = fake_node("ObjectID")
    node.skip_callback(msg("GeometryReconstruction"))
    assert node._advance_skip() is True
    assert node.skips == ["GeometryReconstruction"]


def test_dropped_when_the_state_moved_on_by_itself():
    node = fake_node("ObjectID")
    node.skip_callback(msg("GeometryReconstruction"))
    node.machine.current_state.name = "WallLinesComputation"
    assert node._advance_skip() is True
    assert node.skips == []
    assert node.events[-1][0] == "skip_rejected"


def test_skip_past_object_id_reads_the_walls_on_disk():
    node = fake_node("ObjectID")
    node.ctx["geometry_reconstruction_require_fresh_walls"] = True
    node.skip_callback(msg("GeometryReconstruction"))
    node._advance_skip()
    assert node.ctx["geometry_reconstruction_require_fresh_walls"] is False
    assert node.ctx["start"] is True
    assert node.calls == ["nav_sim"]           # reuses the stack ObjectID started
    assert node._stack_ensured is False


def test_later_skip_keeps_the_freshness_check():
    node = fake_node("ComputeWallPoints")
    node.ctx["geometry_reconstruction_require_fresh_walls"] = True
    node.ctx["walls_data"] = [{"scan_line": ((0, 0, 1), (1, 0, 1))}]
    node.skip_callback(msg("WallTargetSelection"))
    node._advance_skip()
    assert node.ctx["geometry_reconstruction_require_fresh_walls"] is True
    assert ("bootstrap", "WallTargetSelection") not in node.calls


def test_skipping_compute_wall_points_runs_the_wall_bootstrap():
    node = fake_node("GeometryReconstruction")
    node.ctx["fsm_initial_state"] = "Initialization"
    node.skip_callback(msg("WallTargetSelection"))
    node._advance_skip()
    assert node.calls == [("bootstrap", "WallTargetSelection")]
    assert node.ctx["fsm_initial_state"] == "Initialization"
    assert node._skipping is False


def test_skipping_wall_target_selection_asks_for_a_target():
    node = fake_node("WallTargetSelection")
    node.ctx["walls_data"] = [{"scan_line": ((0, 0, 1), (1, 0, 1))}]
    node.skip_callback(msg("NavigateToTarget"))
    node._advance_skip()
    assert "phase1_target" in node.calls


def test_unready_stack_fails_the_skip():
    node = fake_node("ObjectID")
    node._ensure_nav_sim_running = lambda: node.ctx.__setitem__("error_triggered", True)
    node.skip_callback(msg("GeometryReconstruction"))
    with pytest.raises(RuntimeError, match="not ready"):
        node._advance_skip()


def test_unlocalized_skip_enters_localize_first():
    node = fake_node("CreateMap", ctx={})
    node.skip_callback(msg("GeometryReconstruction"))
    node._advance_skip()
    assert node.skips == ["Localize"]
    assert node.ctx["localize_next_state"] == "GeometryReconstruction"
