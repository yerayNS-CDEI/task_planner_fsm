"""Launches on the other computer, through its process_launcher node.

With ``-p perception_launcher:=/process_launcher`` the FSM runs the perception
half of the robot stack (lidars, cameras, rtabmap, Nav2) on PC2 instead of
locally; see task_planner_fsm/process_launcher.py for the PC2 side and
proc_utils.PLACEMENT for which launches go where.

The objects here stand in for ``subprocess.Popen`` in ``ctx["_procs"]``, so the
states keep checking ``ctx["_procs"][key].poll()`` as before:

    RemoteProc  a launch running on PC2 only (e.g. the frontier explorer)
    SplitProc   one FSM launch run as two: stack:=control locally and
                stack:=perception on PC2 (mapping, nav_sim)

The service calls go through a private node in its own rclpy context, spun
only for the length of each call. The FSM calls proc_utils while its own
executor is blocked, and on Ctrl+C after rclpy's signal handler has shut the
default context down, so the client must not depend on either.
"""
import threading
from typing import Iterable, List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from navi_wall.srv import CleanupProcesses, LaunchStatus, SaveRtabmap, StartLaunch, StopLaunch

# How long to wait for the launcher's services to appear before a call fails.
SERVICE_WAIT_S = 5.0
# Plain queries (start, status) return at once on the launcher side.
QUERY_TIMEOUT_S = 15.0
# stop_proc on the launcher escalates SIGINT (timeout) -> SIGTERM (+5 s) ->
# SIGKILL (+2 s); give the call room for the whole sequence.
STOP_MARGIN_S = 30.0


class RemoteLauncherError(RuntimeError):
    """The launcher did not answer (not running, or the link is down)."""


class RemoteLauncher:
    """Client for one process_launcher node, e.g. ``/process_launcher``."""

    def __init__(self, launcher: str, logger=None):
        self.launcher = launcher.rstrip("/")
        self._logger = logger
        self._lock = threading.Lock()
        self._context = None
        self._node = None
        self._executor = None
        self._clients = {}

    def _ensure(self):
        """(Re)create the private context, node and clients if needed."""
        if self._context is not None and self._context.ok():
            return
        self._context = rclpy.Context()
        rclpy.init(context=self._context)
        self._node = rclpy.create_node("fsm_remote_launcher_client", context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._clients = {
            name: self._node.create_client(srv_type, f"{self.launcher}/{name}")
            for name, srv_type in (
                ("start", StartLaunch),
                ("stop", StopLaunch),
                ("status", LaunchStatus),
                ("cleanup", CleanupProcesses),
                ("save_rtabmap", SaveRtabmap),
            )
        }

    def _call(self, name: str, request, timeout: float):
        with self._lock:
            self._ensure()
            client = self._clients[name]
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
                raise RemoteLauncherError(f"{self.launcher}/{name} is not available")
            future = client.call_async(request)
            self._executor.spin_until_future_complete(future, timeout_sec=timeout)
            if not future.done():
                future.cancel()
                raise RemoteLauncherError(f"{self.launcher}/{name} did not answer within {timeout:.0f} s")
            return future.result()

    def _log(self, level: str, msg: str):
        # One call per severity: rclpy rejects a log call site whose severity
        # changes between calls.
        if self._logger is None:
            return
        msg = f"[remote {self.launcher}] {msg}"
        if level == "error":
            self._logger.error(msg)
        elif level == "warn":
            self._logger.warn(msg)
        else:
            self._logger.info(msg)

    def start(self, key: str, cmd: List[str]) -> "RemoteProc":
        req = StartLaunch.Request(key=key, cmd=list(cmd))
        try:
            res = self._call("start", req, QUERY_TIMEOUT_S)
        except RemoteLauncherError as exc:
            self._log("error", f"start '{key}' failed: {exc}")
            return RemoteProc(self, key, pid=-1, failed=True)
        if not res.success:
            self._log("error", f"start '{key}' rejected: {res.message}")
            return RemoteProc(self, key, pid=-1, failed=True)
        self._log("info", f"started '{key}' (pid {res.pid} there): {' '.join(cmd)}")
        return RemoteProc(self, key, pid=res.pid)

    def stop(self, key: str, timeout: float, force_kill_patterns: Optional[Iterable[str]] = None) -> bool:
        req = StopLaunch.Request(key=key, timeout=float(timeout),
                                 force_kill_patterns=list(force_kill_patterns or []))
        try:
            res = self._call("stop", req, float(timeout) + STOP_MARGIN_S)
        except RemoteLauncherError as exc:
            self._log("error", f"stop '{key}' failed: {exc}")
            return False
        self._log("info", res.message)
        return res.success

    def status(self, key: str):
        """(known, running, return_code); raises RemoteLauncherError."""
        res = self._call("status", LaunchStatus.Request(key=key), QUERY_TIMEOUT_S)
        return res.known, res.running, res.return_code

    def cleanup(self, patterns: Iterable[str], kill: bool, wait_timeout: float = 0.0) -> bool:
        req = CleanupProcesses.Request(patterns=list(patterns), kill=kill, wait_timeout=float(wait_timeout))
        try:
            res = self._call("cleanup", req, float(wait_timeout) + QUERY_TIMEOUT_S)
        except RemoteLauncherError as exc:
            self._log("warn", f"cleanup failed: {exc}")
            return False
        if res.killed_pids:
            self._log("warn", f"force-killed stale processes there: {list(res.killed_pids)}")
        return res.success

    def save_rtabmap(self, timeout: float) -> bool:
        try:
            res = self._call("save_rtabmap", SaveRtabmap.Request(timeout=float(timeout)),
                             float(timeout) + QUERY_TIMEOUT_S)
        except RemoteLauncherError as exc:
            self._log("error", f"rtabmap save failed: {exc}")
            return False
        self._log("info", res.message)
        return res.success

    def close(self):
        with self._lock:
            if self._context is not None and self._context.ok():
                self._node.destroy_node()
                rclpy.shutdown(context=self._context)
            self._context = None


class RemoteProc:
    """Popen stand-in for a launch running on the launcher's machine."""

    def __init__(self, launcher: RemoteLauncher, key: str, pid: int, failed: bool = False):
        self.launcher = launcher
        self.key = key
        self.pid = pid
        # A launch that never started counts as exited with an error, so the
        # states' "exited early" checks catch it.
        self.returncode: Optional[int] = 1 if failed else None
        self._link_warned = False

    def poll(self) -> Optional[int]:
        if self.returncode is not None:
            return self.returncode
        try:
            known, running, rc = self.launcher.status(self.key)
        except RemoteLauncherError as exc:
            # Unknown is not the same as dead: report it running and say so
            # once, rather than tear the stack down over a slow link.
            if not self._link_warned:
                self.launcher._log("warn", f"status of '{self.key}' unknown: {exc}")
                self._link_warned = True
            return None
        self._link_warned = False
        if not known:
            self.returncode = -1
        elif not running:
            self.returncode = rc
        return self.returncode


class SplitProc:
    """One FSM launch run as two halves: control here, perception there."""

    def __init__(self, local, remote: RemoteProc):
        self.local = local
        self.remote = remote

    @property
    def pid(self):
        return self.local.pid

    def poll(self) -> Optional[int]:
        """Exited as soon as either half has: the stack is incomplete then."""
        rc = self.local.poll()
        if rc is not None:
            return rc
        return self.remote.poll()
