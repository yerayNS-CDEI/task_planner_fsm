"""Starting the FSM at SensorDataProcessing: process what is on disk, alone.

`fsm_node --initial-state SensorDataProcessing` (the arm_control UI's
"Initial State" combo runs exactly that) must work with no robot at all: no
stack launched, the latest recorded session picked up unless one is named,
the legacy simulation mock kept out, and the run ending in Finished instead
of folding an arm that was never unfolded.

Run with:

    python3 -m pytest test/test_offline_processing_start.py -v
"""

import types

import pytest

from task_planner_fsm import fsm_node
from task_planner_fsm.fsm_node import (
    BootstrapError,
    NAV_CLIENT_BOOTSTRAP_STATES,
    NAV_SIM_REQUIRED_START_STATES,
    OFFLINE_INITIAL_STATES,
    RobotFSMNode,
    WALL_DATA_REQUIRED_INITIAL_STATES,
)
from task_planner_fsm.sensors import paths


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(("info", msg))

    def warn(self, msg, **kw):
        self.lines.append(("warn", msg))

    def error(self, msg, **kw):
        self.lines.append(("error", msg))


def sessions(tmp_path, *stamps):
    root = tmp_path / "raw" / "hyperspectral"
    for stamp in stamps:
        (root / f"session_{stamp}").mkdir(parents=True)
    (root / "raw_data").mkdir(exist_ok=True)        # the camera node's own CSVs: not a session
    (root / "notes.txt").write_text("")
    return root


def fake_node(tmp_path, **ctx):
    ctx.setdefault("sensor_data_dir", str(tmp_path))
    node = types.SimpleNamespace(ctx=ctx, get_logger=lambda: node.logger, logger=_Logger())
    node._bootstrap_sensor_processing = lambda: RobotFSMNode._bootstrap_sensor_processing(node)
    node._abort_bootstrap = lambda reason: RobotFSMNode._abort_bootstrap(node, reason)
    return node


# ---------------------------------------------------------------------------
# The latest session
# ---------------------------------------------------------------------------

def test_latest_session_is_the_newest_stamp_and_ignores_other_folders(tmp_path):
    root = sessions(tmp_path, "20260915_141344", "20260915_142215", "20260915_141752")
    ctx = {"sensor_data_dir": str(tmp_path)}
    assert [p.name for p in paths.raw_session_dirs(ctx)] == [
        "session_20260915_141344", "session_20260915_141752", "session_20260915_142215"]
    assert paths.latest_raw_session_dir(ctx) == root / "session_20260915_142215"


def test_no_sessions_means_none(tmp_path):
    assert paths.latest_raw_session_dir({"sensor_data_dir": str(tmp_path)}) is None
    (tmp_path / "raw" / "hyperspectral").mkdir(parents=True)
    assert paths.latest_raw_session_dir({"sensor_data_dir": str(tmp_path)}) is None


# ---------------------------------------------------------------------------
# The bootstrap
# ---------------------------------------------------------------------------

def test_offline_start_processes_the_latest_session_and_stops_there(tmp_path, monkeypatch):
    root = sessions(tmp_path, "20260915_141344", "20260915_142215")
    node = fake_node(tmp_path)
    node._bootstrap_sensor_processing()
    assert node.ctx["hyperspectral_session_dir"] == str(root / "session_20260915_142215")
    assert node.ctx["fsm_stop_after"] == "SensorDataProcessing"
    assert node.ctx["sensor_processing_mock"] is False
    assert node.ctx["current_wall_index"] is None
    # Results go next to the record, under the same stamp.
    assert paths.processed_dir(node.ctx).name == "session_20260915_142215"


def test_an_explicit_session_wins_over_the_latest(tmp_path):
    root = sessions(tmp_path, "20260915_141344", "20260915_142215")
    node = fake_node(tmp_path, hyperspectral_session_dir=str(root / "session_20260915_141344"))
    node._bootstrap_sensor_processing()
    assert node.ctx["hyperspectral_session_dir"].endswith("session_20260915_141344")


def test_a_missing_explicit_session_refuses_to_start(tmp_path, monkeypatch):
    monkeypatch.setattr(fsm_node, "stop_all", lambda ctx: None)
    node = fake_node(tmp_path, hyperspectral_session_dir=str(tmp_path / "nope"))
    with pytest.raises(BootstrapError):
        node._bootstrap_sensor_processing()


def test_no_session_at_all_still_runs_for_the_gpr_exports(tmp_path):
    node = fake_node(tmp_path)
    node._bootstrap_sensor_processing()
    assert "hyperspectral_session_dir" not in node.ctx
    assert any(level == "warn" and "GPR" in msg for level, msg in node.logger.lines)
    assert node.ctx["fsm_stop_after"] == "SensorDataProcessing"


def test_an_explicit_stop_after_is_respected(tmp_path):
    sessions(tmp_path, "20260915_142215")
    node = fake_node(tmp_path, fsm_stop_after="SendDataToPokeye")
    node._bootstrap_sensor_processing()
    assert node.ctx["fsm_stop_after"] == "SendDataToPokeye"


# ---------------------------------------------------------------------------
# Nothing else is dragged in
# ---------------------------------------------------------------------------

def test_the_offline_state_needs_no_stack_no_walls_no_nav_client():
    assert "SensorDataProcessing" in OFFLINE_INITIAL_STATES
    assert "SensorDataProcessing" not in NAV_SIM_REQUIRED_START_STATES
    assert "SensorDataProcessing" not in WALL_DATA_REQUIRED_INITIAL_STATES
    assert "SensorDataProcessing" not in NAV_CLIENT_BOOTSTRAP_STATES
    # ...while the states around it still get the stack, as before.
    assert "SendDataToPokeye" in NAV_SIM_REQUIRED_START_STATES
    assert "ArmFolding" in NAV_SIM_REQUIRED_START_STATES
