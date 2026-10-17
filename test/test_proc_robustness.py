"""Unit tests for the FSM's stack teardown and readiness checks (proc_utils).

The process mechanics themselves (process groups, the verified rtabmap save,
the cleanup patterns) live in navi-wall's navi_wall_launcher and are tested
there (navi-wall/test/test_launcher_procs.py). These cover the FSM's side:

- stop_proc kills what is left of a launch's process group (through the
  shared code), e.g. a deadlocked rtabmap that outlived its launch.
- CreateMap's readiness check passed while Nav2's bringup had been aborted.
- ScanWall's sensor reader outlived an FSM stopped mid-scan.

Run with:

    python3 -m pytest test/test_proc_robustness.py -v
"""

import os
import signal
import subprocess
import time

import pytest

from navi_wall_launcher import procs as shared_procs
from task_planner_fsm.states import proc_utils


def _group_alive(pgid):
    return bool(shared_procs.group_members(pgid))


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
