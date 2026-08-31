"""Unit tests for GeometryReconstruction's stale-walls guard.

Regression cover for the bug where the detector wrote detected_walls.yaml into
the install tree: navi_wall installs rgb_detections/ from source, and that
directory is not cleared between runs or between environments. A read before the
detector had saved then loaded whatever the last run left there -- another
building, another Gazebo world -- silently, with plausible-looking markers.

Run with:

    python3 -m pytest test/test_wall_file_freshness.py -v
"""

import os
import time

import pytest

from task_planner_fsm.machine import WALL_RUN_DIR, wall_file_path
from task_planner_fsm.states.geometry_reconstruction import GeometryReconstruction


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        pass

    def error(self, msg):
        self.errors.append(msg)


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


@pytest.fixture
def state():
    s = GeometryReconstruction("GeometryReconstruction")
    s._walls_wait_start = None
    s._walls_wait_logged = False
    return s


def make_ctx(started_ago_s=10.0, **extra):
    ctx = {
        "node": _FakeNode(),
        "fsm_start_wall_time": time.time() - started_ago_s,
    }
    ctx.update(extra)
    return ctx


def write_walls(tmp_path, mtime_offset_s):
    """A walls file whose mtime is now + offset (negative = in the past)."""
    path = tmp_path / "detected_walls.yaml"
    path.write_text("walls: []\n")
    when = time.time() + mtime_offset_s
    os.utime(path, (when, when))
    return str(path)


# ---------------------------------------------------------------------------
# The guard itself
# ---------------------------------------------------------------------------

def test_file_written_during_this_run_is_fresh(state, tmp_path):
    ctx = make_ctx(started_ago_s=10.0)
    path = write_walls(tmp_path, -5.0)          # written 5s ago, run started 10s ago
    assert state._wall_file_is_fresh(ctx, path) is True


def test_file_from_a_previous_session_is_rejected(state, tmp_path):
    """The actual bug: a file left in the install tree by an earlier run is
    always older than the run that reads it."""
    ctx = make_ctx(started_ago_s=10.0)
    path = write_walls(tmp_path, -3600.0)       # an hour old
    assert state._wall_file_is_fresh(ctx, path) is False
    assert not ctx.get("error_triggered"), "should wait first, not fail immediately"


def test_missing_file_waits_rather_than_erroring(state, tmp_path):
    ctx = make_ctx()
    assert state._wall_file_is_fresh(ctx, str(tmp_path / "nope.yaml")) is False
    assert not ctx.get("error_triggered")


def test_stale_file_errors_after_the_timeout(state, tmp_path):
    ctx = make_ctx(started_ago_s=10.0, geometry_reconstruction_wall_file_timeout_s=0.0)
    path = write_walls(tmp_path, -3600.0)
    assert state._wall_file_is_fresh(ctx, path) is False
    assert ctx.get("error_triggered"), "must give up rather than load stale walls"
    assert state._walls_wait_start is not None


def test_error_message_names_the_path(state, tmp_path):
    ctx = make_ctx(geometry_reconstruction_wall_file_timeout_s=0.0)
    path = write_walls(tmp_path, -3600.0)
    state._wall_file_is_fresh(ctx, path)
    assert any(path in e for e in ctx["node"].get_logger().errors)


def test_guard_can_be_disabled_for_deliberate_replays(state, tmp_path):
    ctx = make_ctx(geometry_reconstruction_require_fresh_walls=False)
    path = write_walls(tmp_path, -999999.0)
    assert state._wall_file_is_fresh(ctx, path) is True


def test_wait_is_logged_once_not_every_tick(state, tmp_path):
    ctx = make_ctx()
    path = write_walls(tmp_path, -3600.0)
    for _ in range(5):
        state._wall_file_is_fresh(ctx, path)
    waiting = [m for m in ctx["node"].get_logger().infos if "Waiting" in m]
    assert len(waiting) == 1


def test_a_file_that_appears_while_waiting_is_accepted(state, tmp_path):
    """The normal sim start: the state polls, the detector saves, the next tick
    picks it up."""
    ctx = make_ctx(started_ago_s=10.0)
    path = str(tmp_path / "detected_walls.yaml")
    assert state._wall_file_is_fresh(ctx, path) is False
    write_walls(tmp_path, -1.0)
    assert state._wall_file_is_fresh(ctx, path) is True


# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------

def test_explicit_path_override_wins(state, tmp_path):
    target = str(tmp_path / "walls.yaml")
    ctx = make_ctx(geometry_reconstruction_wall_file_path=target)
    assert state._resolve_wall_file_path(ctx) == target


def test_override_expands_user(state):
    ctx = make_ctx(geometry_reconstruction_wall_file_path="~/walls.yaml")
    resolved = state._resolve_wall_file_path(ctx)
    assert not resolved.startswith("~")
    assert os.path.isabs(resolved)


# ---------------------------------------------------------------------------
# The run path must stay out of the install space -- that was the whole bug
# ---------------------------------------------------------------------------

def test_run_path_is_absolute_and_outside_the_install_space():
    for sim in (True, False):
        path = wall_file_path(sim)
        assert os.path.isabs(path)
        assert os.sep + "install" + os.sep not in path
        assert "rgb_detections" not in path


def test_sim_and_real_runs_use_different_files():
    assert wall_file_path(True) != wall_file_path(False)
    assert wall_file_path(True).startswith(WALL_RUN_DIR)
    assert wall_file_path(False).startswith(WALL_RUN_DIR)


# ---------------------------------------------------------------------------
# Diagnostics: the timeout message must say WHICH failure this is
# ---------------------------------------------------------------------------

class _DeadProc:
    pid = 4321

    def poll(self):
        return 1        # exited


class _LiveProc:
    pid = 1234

    def poll(self):
        return None     # still running


def test_timeout_says_nothing_is_running_when_no_proc_exists(state, tmp_path):
    ctx = make_ctx(geometry_reconstruction_wall_file_timeout_s=0.0)
    state._wall_file_is_fresh(ctx, str(tmp_path / "missing.yaml"))
    assert any("no process" in e for e in ctx["node"].get_logger().errors)


def test_timeout_says_nothing_is_running_when_the_proc_died(state, tmp_path):
    ctx = make_ctx(
        geometry_reconstruction_wall_file_timeout_s=0.0,
        _procs={"wall_detection": _DeadProc(), "nav_sim": _DeadProc()},
    )
    state._wall_file_is_fresh(ctx, str(tmp_path / "missing.yaml"))
    assert any("no process" in e for e in ctx["node"].get_logger().errors)


def test_nav_sim_counts_as_an_owner_of_the_detector(state, tmp_path):
    """move_robot.launch.py starts wall_detection_node itself, so there is no
    separate _procs["wall_detection"] in the bootstrap flow -- the hint must not
    claim the detector is absent."""
    ctx = make_ctx(
        geometry_reconstruction_wall_file_timeout_s=0.0,
        _procs={"nav_sim": _LiveProc()},
    )
    state._wall_file_is_fresh(ctx, str(tmp_path / "missing.yaml"))
    errors = ctx["node"].get_logger().errors
    assert not any("no process" in e for e in errors)
    assert any("nav_sim pid=1234" in e for e in errors)


def test_timeout_points_at_the_cloud_topic_when_the_detector_is_alive(state, tmp_path):
    """Different failure, different fix: the detector is up but has nothing to
    detect from, which means /rtabmap/cloud_map, not the launch wiring."""
    ctx = make_ctx(
        geometry_reconstruction_wall_file_timeout_s=0.0,
        _procs={"wall_detection": _LiveProc()},
    )
    state._wall_file_is_fresh(ctx, str(tmp_path / "missing.yaml"))
    errors = ctx["node"].get_logger().errors
    assert any("cloud_map" in e for e in errors)
    assert any("1234" in e for e in errors)


# ---------------------------------------------------------------------------
# The detector's path must be forwarded all the way down the launch chain
# ---------------------------------------------------------------------------

def test_nav_sim_launch_forwards_the_walls_path():
    """Regression: move_robot.launch.py starts wall_detection_node itself. Without
    forwarding wall_file_path it wrote the launch default (the REAL path) while
    the FSM read the sim path, so the guard timed out on every sim run."""
    src = open("task_planner_fsm/fsm_node.py").read()
    assert "wall_file_path:={self.ctx['geometry_reconstruction_wall_file_path']}" in src
