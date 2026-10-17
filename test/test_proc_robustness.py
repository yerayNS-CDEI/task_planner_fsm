"""Unit tests for the stack teardown and readiness checks in proc_utils.

Found while testing the Jetson/PC2 split:

- A node that ignores SIGINT and SIGTERM (a deadlocked rtabmap) outlived its
  launch and kept running, orphaned. stop_proc must kill what is left of the
  launch's process group.
- graceful_rtabmap_save reported "database saved" when rtabmap had been killed.
- The cleanup patterns did not match Nav2, so an orphaned lifecycle_manager
  survived the sweeps.
- CreateMap's readiness check passed while Nav2's bringup had been aborted.
- ScanWall's sensor reader outlived an FSM stopped mid-scan.

Real processes stand in for launches and rtabmap; no ROS graph is used.

Run with:

    python3 -m pytest test/test_proc_robustness.py -v
"""

import os
import re
import signal
import subprocess
import sys
import textwrap
import threading
import time
import uuid

import pytest

from task_planner_fsm.states import proc_utils


def _group_alive(pgid):
    return bool(proc_utils._group_members(pgid))


def test_stop_proc_kills_what_outlives_the_launch():
    # The "launch" (sh) dies on SIGINT, its child ignores SIGINT and SIGTERM:
    # the child is left in the group, reparented to init.
    ctx = {}
    cmd = ["sh", "-c", "(trap '' INT TERM; exec sleep 60) & wait"]
    proc_utils.start_proc(ctx, "stack", cmd)
    pgid = ctx["_procs"]["stack"].pid
    time.sleep(0.5)
    try:
        proc_utils.stop_proc(ctx, "stack", timeout=1.0)
        assert not _group_alive(pgid)
        assert "stack" not in ctx["_procs"]
    finally:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass


@pytest.mark.parametrize("cmdline", [
    "/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_navigation",
    "/opt/ros/humble/lib/nav2_controller/controller_server --ros-args",
    "/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator --ros-args",
    "/home/pc2/ros2_ws/install/navi_wall/lib/navi_wall/explore --ros-args",
    "/home/pc2/ros2_ws/install/navi_wall/lib/navi_wall/find_frontiers --ros-args",
])
def test_the_cleanup_patterns_match_nav2_and_the_explorer(cmdline):
    assert any(re.search(p, cmdline) for p in proc_utils.SIM_STACK_PATTERNS)


@pytest.mark.parametrize("cmdline", [
    "/usr/libexec/gvfsd-trash --spawner :1.3 /org/gtk/gvfs/exec_spaw/0",
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 run task_planner_fsm fsm_node",
    "/usr/bin/python3 /home/pc2/ros2_ws/install/task_planner_fsm/lib/task_planner_fsm/process_launcher",
])
def test_the_cleanup_patterns_leave_other_processes_alone(cmdline):
    assert not any(re.search(p, cmdline) for p in proc_utils.SIM_STACK_PATTERNS)


# --- graceful_rtabmap_save ------------------------------------------------

_FAKE_RTABMAP = textwrap.dedent("""
    import pathlib, signal, sys, time
    database, behaviour = sys.argv[4], sys.argv[5]
    def on_sigint(*_):
        if behaviour == "save":
            pathlib.Path(database).write_bytes(b"saved")
        sys.exit(0)
    signal.signal(signal.SIGINT, on_sigint if behaviour != "ignore" else signal.SIG_IGN)
    print("ready", flush=True)
    time.sleep(60)
""")


@pytest.fixture
def fake_rtabmap(tmp_path, monkeypatch):
    """Start a process that looks like rtabmap to graceful_rtabmap_save."""
    monkeypatch.setattr(proc_utils, "_REMOTE", None)
    started = []

    def start(behaviour):
        tag = f"fake_rtabmap_slam_{uuid.uuid4().hex}"
        database = tmp_path / "rtabmap.db"
        database.write_bytes(b"old")
        old = time.time() - 3600
        os.utime(database, (old, old))
        params = tmp_path / "launch_params_test"
        params.write_text(f"rtabmap:\n  ros__parameters:\n    database_path: {database}\n")
        p = subprocess.Popen(
            [sys.executable, "-c", _FAKE_RTABMAP, tag, "--params-file", str(params),
             str(database), behaviour],
            stdout=subprocess.PIPE, text=True)
        p.stdout.readline()  # handler installed
        started.append(p)
        return p, tag, database

    yield start
    for p in started:
        if p.poll() is None:
            p.kill()
        p.wait()


def test_a_clean_exit_that_wrote_the_database_counts_as_saved(fake_rtabmap):
    _p, tag, database = fake_rtabmap("save")
    assert proc_utils.graceful_rtabmap_save(timeout=10, poll_interval=0.1, pattern=tag)
    assert database.read_bytes() == b"saved"


def test_an_exit_without_writing_the_database_is_not_a_save(fake_rtabmap):
    _p, tag, _database = fake_rtabmap("exit")
    assert not proc_utils.graceful_rtabmap_save(timeout=10, poll_interval=0.1, pattern=tag)


def test_a_kill_during_the_save_is_not_a_save(fake_rtabmap):
    # rtabmap hangs on SIGINT and is killed while we wait for it: the old code
    # saw the process gone and reported the database saved.
    p, tag, _database = fake_rtabmap("ignore")
    killer = threading.Timer(0.5, p.kill)
    killer.start()
    try:
        assert not proc_utils.graceful_rtabmap_save(timeout=10, poll_interval=0.1, pattern=tag)
    finally:
        killer.cancel()


# --- wait_nav2_active ------------------------------------------------------

@pytest.fixture
def is_active_answers(monkeypatch):
    def install(*answers):
        replies = list(answers)

        def fake_run(cmd, **_kwargs):
            out = replies.pop(0) if len(replies) > 1 else replies[0]
            return subprocess.CompletedProcess(cmd, 0, stdout=out, stderr="")
        monkeypatch.setattr(proc_utils.subprocess, "run", fake_run)
    return install


ACTIVE = "response:\nstd_srvs.srv.Trigger_Response(success=True, message='')\n"
INACTIVE = "response:\nstd_srvs.srv.Trigger_Response(success=False, message='')\n"


def test_nav2_becoming_active_is_waited_for(is_active_answers):
    is_active_answers("", INACTIVE, ACTIVE)
    assert proc_utils.wait_nav2_active({}, timeout=10, poll_interval=0.01)


def test_an_aborted_nav2_bringup_fails_the_readiness_check(is_active_answers):
    is_active_answers(INACTIVE)
    start = time.time()
    assert not proc_utils.wait_nav2_active({}, timeout=0.2, poll_interval=0.01)
    assert time.time() - start < 5


# --- ScanWall's arm processes ---------------------------------------------

def test_scan_walls_arm_processes_are_stopped_with_the_rest(monkeypatch):
    # arduino_sensors(_sim) was left running after the FSM stopped mid-scan.
    from task_planner_fsm.states import scan_wall

    class _Log:
        def info(self, msg):
            pass

        def warn(self, msg):
            pass

        def error(self, msg):
            pass

    class _Node:
        def get_logger(self):
            return _Log()

    ctx = {"node": _Node()}
    state = scan_wall.ScanWall.__new__(scan_wall.ScanWall)
    state.name = "ScanWall"
    assert state._start_arm_procs(ctx, [("arduino_sensors_proc", ["sleep", "60"], "fake_sensors")])
    proc = ctx["arduino_sensors_proc"]
    try:
        assert ctx["_procs"]["arduino_sensors_proc"] is proc
        monkeypatch.setattr(proc_utils, "_REMOTE", None)
        monkeypatch.setattr(proc_utils, "graceful_rtabmap_save", lambda **_kw: True)
        proc_utils.stop_all(ctx)
        assert proc.wait(timeout=5) is not None
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
