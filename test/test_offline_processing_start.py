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
    assert "sensor_session_id" not in node.ctx
    assert any(level == "warn" and "GPR" in msg for level, msg in node.logger.lines)
    assert node.ctx["fsm_stop_after"] == "SensorDataProcessing"


# ---------------------------------------------------------------------------
# GPR-only sessions (camera off): the record lives under data/raw/gpr only
# ---------------------------------------------------------------------------

def gpr_sessions(tmp_path, *names, manifest=True):
    root = tmp_path / "raw" / "gpr"
    (root / "incoming").mkdir(parents=True, exist_ok=True)      # the shared inbox: not a session
    dirs = []
    for name in names:
        d = root / f"session_{name}"
        d.mkdir()
        if manifest:
            (d / paths.GPR_MANIFEST_FILENAME).write_text('{"key": "w00_l00_s00"}\n')
        dirs.append(d)
    return root, dirs


def test_gpr_sessions_are_listed_and_the_latest_is_by_manifest_time(tmp_path):
    import os
    root, (older, newer) = gpr_sessions(tmp_path, "wheel", "20260917_121753")
    ctx = {"sensor_data_dir": str(tmp_path)}
    assert [p.name for p in paths.gpr_session_dirs(ctx)] == [
        "session_20260917_121753", "session_wheel"]
    # A hand-renamed session does not sort by name, so the manifest's mtime decides.
    os.utime(older / paths.GPR_MANIFEST_FILENAME, (2_000_000_000, 2_000_000_000))
    os.utime(newer / paths.GPR_MANIFEST_FILENAME, (1_000_000_000, 1_000_000_000))
    assert paths.latest_gpr_session_dir(ctx) == older
    assert paths.session_stamp_of(older) == "wheel"
    assert paths.session_stamp_of(newer) == "20260917_121753"


def test_a_gpr_folder_without_a_manifest_is_not_a_session(tmp_path):
    gpr_sessions(tmp_path, "empty", manifest=False)
    assert paths.latest_gpr_session_dir({"sensor_data_dir": str(tmp_path)}) is None


def test_offline_start_falls_back_to_the_latest_gpr_session(tmp_path):
    _, (session,) = gpr_sessions(tmp_path, "no_wheel")
    node = fake_node(tmp_path)
    node._bootstrap_sensor_processing()
    assert "hyperspectral_session_dir" not in node.ctx
    assert node.ctx["sensor_session_id"] == "no_wheel"
    assert paths.gpr_manifest_path(node.ctx) == session / paths.GPR_MANIFEST_FILENAME
    assert paths.processed_dir(node.ctx).name == "session_no_wheel"
    assert not any(level == "warn" for level, _ in node.logger.lines)


def test_a_hyperspectral_session_wins_over_a_gpr_only_one(tmp_path):
    root = sessions(tmp_path, "20260915_142215")
    gpr_sessions(tmp_path, "20260917_121753")
    node = fake_node(tmp_path)
    node._bootstrap_sensor_processing()
    assert node.ctx["hyperspectral_session_dir"] == str(root / "session_20260915_142215")
    assert paths.processed_dir(node.ctx).name == "session_20260915_142215"


def test_an_explicit_sensor_session_id_picks_that_gpr_session(tmp_path):
    _, (wheel, _no_wheel) = gpr_sessions(tmp_path, "wheel", "no_wheel")
    node = fake_node(tmp_path, sensor_session_id="wheel")
    node._bootstrap_sensor_processing()
    assert paths.gpr_session_dir(node.ctx) == wheel
    assert paths.processed_dir(node.ctx).name == "session_wheel"


def test_a_missing_explicit_sensor_session_id_refuses_to_start(tmp_path, monkeypatch):
    monkeypatch.setattr(fsm_node, "stop_all", lambda ctx: None)
    gpr_sessions(tmp_path, "wheel")
    node = fake_node(tmp_path, sensor_session_id="nope")
    with pytest.raises(BootstrapError):
        node._bootstrap_sensor_processing()


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


# ---------------------------------------------------------------------------
# The non-ROS CLI resolves the same sessions
# ---------------------------------------------------------------------------

def test_cli_resolves_a_stamp_under_the_hyperspectral_root_first(tmp_path):
    from task_planner_fsm.process_sensor_session import _resolve_session
    root = sessions(tmp_path, "20260915_142215")
    gpr_sessions(tmp_path, "20260915_142215")
    ctx = {"sensor_data_dir": str(tmp_path)}
    assert _resolve_session("20260915_142215", ctx) == (root / "session_20260915_142215", "20260915_142215")
    assert _resolve_session("session_20260915_142215", ctx) == (root / "session_20260915_142215", "20260915_142215")


def test_cli_resolves_a_gpr_only_session_without_a_hyperspectral_dir(tmp_path):
    from task_planner_fsm.process_sensor_session import _resolve_session
    _, (wheel,) = gpr_sessions(tmp_path, "wheel")
    ctx = {"sensor_data_dir": str(tmp_path)}
    assert _resolve_session("wheel", ctx) == (None, "wheel")
    assert _resolve_session(str(wheel), ctx) == (None, "wheel")
    with pytest.raises(SystemExit):
        _resolve_session("nope", ctx)
