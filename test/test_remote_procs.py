"""Unit tests for the Jetson/PC2 split's process handling (remote_procs + proc_utils).

The FSM runs the perception half of its stacks on PC2 through process_launcher
(``-p perception_launcher:=...``). These cover the failure modes found in
review:

- Ctrl+C while a state's poll() is inside a launcher call must not deadlock
  the cleanup on the client's own lock.
- A half-dead split pair must not leave its surviving control half running
  untracked next to a new one (two ros2_control_nodes on the same hardware).
- A launch left on PC2 by an earlier FSM session is stopped, not reused.
- graceful_rtabmap_save reports a failed save on PC2.
- The FSM imports with a navi_wall that lacks the launcher services.

No real launcher is used: the launcher client is replaced by fakes.

Run with:

    python3 -m pytest test/test_remote_procs.py -v
"""

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

from task_planner_fsm.states import proc_utils, remote_procs

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _run_child(code: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    """Run `code` in a fresh interpreter with this package importable.

    Used where a regression would hang or poison the interpreter (a deadlock,
    a stubbed module in sys.modules).
    """
    env = dict(os.environ)
    env["PYTHONPATH"] = f"{PACKAGE_ROOT}{os.pathsep}{env.get('PYTHONPATH', '')}"
    return subprocess.run([sys.executable, "-c", textwrap.dedent(code)], cwd=PACKAGE_ROOT,
                          env=env, capture_output=True, text=True, timeout=timeout)


class _FakeLogger:
    def __init__(self):
        self.errors = []

    def info(self, msg):
        pass

    def warn(self, msg):
        pass

    def error(self, msg):
        self.errors.append(msg)


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


class _FakeLauncher:
    """Stands in for RemoteLauncher; records the calls it gets."""

    launcher = "/process_launcher"

    def __init__(self, start_ok=False, save_ok=True):
        self.start_ok = start_ok
        self.save_ok = save_ok
        self.calls = []

    def start(self, key, cmd):
        self.calls.append(("start", key))
        if self.start_ok:
            return remote_procs.RemoteProc(self, key, pid=4242)
        return remote_procs.RemoteProc(self, key, pid=-1, failed=True)

    def stop(self, key, timeout, force_kill_patterns=None):
        self.calls.append(("stop", key))
        return True

    def status(self, key):
        return True, True, 0

    def save_rtabmap(self, timeout):
        self.calls.append(("save_rtabmap", None))
        return self.save_ok


@pytest.fixture
def fake_remote(monkeypatch):
    def install(**kwargs):
        launcher = _FakeLauncher(**kwargs)
        monkeypatch.setattr(proc_utils, "_REMOTE", launcher)
        return launcher
    return install


def test_ctrl_c_during_a_poll_does_not_deadlock_the_cleanup():
    # The main thread is inside status() (holding the client's lock, waiting
    # for the service) when SIGINT arrives. The cleanup handler then saves
    # rtabmap and stops the launch through the same client; before the fix it
    # blocked forever on the lock.
    result = _run_child("""
        import os, signal, threading, time
        from task_planner_fsm.states import proc_utils, remote_procs

        served = []

        class SlowClient:
            # Link down: wait_for_service runs out its timeout (capped at 1 s).
            def __init__(self, owner, name):
                self.owner, self.name = owner, name
            def wait_for_service(self, timeout_sec):
                served.append((self.name, self.owner is main))
                time.sleep(min(timeout_sec, 1.0))
                return False

        class OkContext:
            def ok(self):
                return True

        def fake_ensure(self):
            if not self._clients:
                self._context = OkContext()
                self._clients = {n: SlowClient(self, n) for n in
                                 ("start", "stop", "status", "cleanup", "save_rtabmap")}

        remote_procs.RemoteLauncher._ensure = fake_ensure
        main = remote_procs.RemoteLauncher("/process_launcher")
        proc_utils._REMOTE = main

        class Log:
            def info(self, m): pass
            warn = error = info
        class Node:
            def get_logger(self): return Log()

        ctx = {"node": Node(),
               "_procs": {"explore": remote_procs.RemoteProc(main, "explore", pid=123)}}
        proc_utils.install_global_cleanup(ctx)
        threading.Timer(0.3, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
        try:
            ctx["_procs"]["explore"].poll()
            print("poll returned")
        except KeyboardInterrupt:
            print("KeyboardInterrupt reached")
        print("served:", served)
    """)
    assert "KeyboardInterrupt reached" in result.stdout, result.stdout + result.stderr
    # The interrupted status call was the main client's; the cleanup's save and
    # stop went through the fallback client.
    assert "('status', True)" in result.stdout
    assert "('save_rtabmap', False)" in result.stdout
    assert "('stop', False)" in result.stdout


def test_a_half_dead_split_pair_is_stopped_before_a_new_one_starts(fake_remote):
    # PC2's launcher is down, so the remote half "exits" at once and the pair
    # counts as exited. Starting again must stop the surviving local control
    # half first instead of running a second one next to it.
    launcher = fake_remote(start_ok=False)
    ctx = {}
    cmd = ["sleep", "60"]  # stands in for move_robot.launch.py
    procs = []
    try:
        proc_utils.start_proc(ctx, "nav_sim", cmd)
        first = ctx["_procs"]["nav_sim"].local
        procs.append(first)
        proc_utils.start_proc(ctx, "nav_sim", cmd)
        second = ctx["_procs"]["nav_sim"].local
        procs.append(second)

        assert second is not first
        assert first.wait(timeout=5) is not None
        assert second.poll() is None
        # Only the fresh session's start cleared a leftover on PC2; the restart
        # stopped the old pair through stop_proc, whose remote half had exited.
        assert launcher.calls == [("stop", "nav_sim"), ("start", "nav_sim"), ("start", "nav_sim")]
    finally:
        for p in procs:
            if p.poll() is None:
                p.kill()
                p.wait()


def test_a_running_split_pair_is_reused(fake_remote):
    launcher = fake_remote(start_ok=True)
    ctx = {}
    procs = []
    try:
        proc_utils.start_proc(ctx, "nav_sim", ["sleep", "60"])
        pair = ctx["_procs"]["nav_sim"]
        procs.append(pair.local)
        proc_utils.start_proc(ctx, "nav_sim", ["sleep", "60"])
        assert ctx["_procs"]["nav_sim"] is pair
        assert launcher.calls == [("stop", "nav_sim"), ("start", "nav_sim")]
    finally:
        for p in procs:
            p.kill()
            p.wait()


def test_a_launch_left_on_pc2_by_an_earlier_session_is_stopped_first(fake_remote):
    launcher = fake_remote(start_ok=True)
    ctx = {}
    proc_utils.start_proc(ctx, "explore", ["ros2", "launch", "navi_wall", "exploration.launch.py"])
    assert isinstance(ctx["_procs"]["explore"], remote_procs.RemoteProc)
    assert launcher.calls == [("stop", "explore"), ("start", "explore")]


@pytest.mark.parametrize("save_ok", [True, False])
def test_rtabmap_save_reports_the_result_on_pc2(fake_remote, save_ok):
    fake_remote(save_ok=save_ok)
    # No rtabmap runs locally (pattern matches nothing), as on the Jetson.
    result = proc_utils.graceful_rtabmap_save(node=_FakeNode(), timeout=1.0,
                                              pattern="no_such_rtabmap_process_xyz")
    assert result is save_ok


def test_status_waits_briefly_for_the_launcher():
    # The states poll every 1 Hz tick; with PC2 unreachable a status call must
    # not hold the tick for the full service wait of the other calls.
    launcher = remote_procs.RemoteLauncher("/process_launcher")
    waits = []

    class Unreachable:
        def wait_for_service(self, timeout_sec):
            waits.append(timeout_sec)
            return False

    class OkContext:
        def ok(self):
            return True

    launcher._context = OkContext()
    launcher._clients = {n: Unreachable() for n in ("start", "stop", "status", "cleanup", "save_rtabmap")}
    proc = remote_procs.RemoteProc(launcher, "explore", pid=1)
    assert proc.poll() is None  # unknown, not dead
    launcher.stop("explore", timeout=1.0)
    assert waits == [remote_procs.STATUS_SERVICE_WAIT_S, remote_procs.SERVICE_WAIT_S]
    assert remote_procs.STATUS_SERVICE_WAIT_S < remote_procs.SERVICE_WAIT_S


def test_the_fsm_imports_without_the_launcher_services():
    # A navi_wall built before the split has no StartLaunch & co. With the
    # split off the FSM must still import.
    result = _run_child("""
        import sys, types
        sys.modules["navi_wall.srv"] = types.ModuleType("navi_wall.srv")
        from task_planner_fsm.states import proc_utils  # noqa: F401
        from task_planner_fsm import machine  # noqa: F401
        print("imported")
    """)
    assert "imported" in result.stdout, result.stdout + result.stderr
