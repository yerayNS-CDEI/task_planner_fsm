"""Launches on the other computer, through its process_launcher node.

With ``-p perception_launcher:=/process_launcher`` the FSM runs the perception
half of the robot stack (lidars, cameras, rtabmap, Nav2) on PC2 instead of
locally. The PC2 side is navi-wall's process_launcher (navi_wall_launcher.node);
proc_utils.PLACEMENT says which launches go where.

The objects here stand in for ``subprocess.Popen`` in ``ctx["_procs"]``, so the
states keep checking ``ctx["_procs"][key].poll()`` as before:

    RemoteProc  a launch running on PC2 only (e.g. the frontier explorer)
    SplitProc   one FSM launch run as two: stack:=control locally and
                stack:=perception on PC2 (mapping, nav_sim)

The service calls go through a private node in its own rclpy context, spun
only for the length of each call. The FSM calls proc_utils while its own
executor is blocked, and on Ctrl+C after rclpy's signal handler has shut the
default context down, so the client must not depend on either.

The navi_wall service types are imported only once a launcher is used, so the
FSM still imports with a navi_wall built before they existed, as long as the
split is off.
"""
import importlib
import threading
from typing import Iterable, List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

# How long to wait for the launcher's services to appear before a call fails.
SERVICE_WAIT_S = 5.0
# The same for the status query, which the states make on every 1 Hz tick:
# with PC2 unreachable a longer wait would stall each tick.
STATUS_SERVICE_WAIT_S = 1.0
# Plain queries (start, status) return at once on the launcher side.
QUERY_TIMEOUT_S = 15.0
# stop_proc on the launcher escalates SIGINT (timeout) -> SIGTERM (+5 s) ->
# SIGKILL (+2 s); give the call room for the whole sequence.
STOP_MARGIN_S = 30.0


def _srv():
    """The navi_wall service types, imported on first use."""
    return importlib.import_module("navi_wall.srv")


class RemoteLauncherError(RuntimeError):
    """The launcher did not answer (not running, or the link is down)."""


class RemoteLauncher:
    """Client for one process_launcher node, e.g. ``/process_launcher``."""

    def __init__(self, launcher: str, logger=None):
        self.launcher = launcher.rstrip("/")
        self._logger = logger
        self._lock = threading.Lock()
        # Thread inside _call, and the client used when that thread re-enters:
        # a SIGINT/SIGTERM handler runs on the main thread, possibly while the
        # main thread is inside a call (a state's poll() on the 1 Hz tick), and
        # its cleanup (stop_all -> save_rtabmap) must not wait on our own lock.
        self._owner = None
        self._fallback: Optional["RemoteLauncher"] = None
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
        srv = _srv()
        self._clients = {
            name: self._node.create_client(srv_type, f"{self.launcher}/{name}")
            for name, srv_type in (
                ("start", srv.StartLaunch),
                ("stop", srv.StopLaunch),
                ("status", srv.LaunchStatus),
                ("cleanup", srv.CleanupProcesses),
                ("save_rtabmap", srv.SaveRtabmap),
            )
        }

    def _call(self, name: str, request, timeout: float, service_wait: float = SERVICE_WAIT_S):
        if self._owner == threading.get_ident():
            # Re-entered from a signal handler on the thread already inside a
            # call: use an independent client (own context, node, executor)
            # instead of deadlocking on our lock, so the cleanup still reaches
            # the launcher.
            if self._fallback is None:
                self._fallback = RemoteLauncher(self.launcher, self._logger)
            return self._fallback._call(name, request, timeout, service_wait)
        with self._lock:
            self._owner = threading.get_ident()
            try:
                self._ensure()
                client = self._clients[name]
                if not client.wait_for_service(timeout_sec=service_wait):
                    raise RemoteLauncherError(f"{self.launcher}/{name} is not available")
                future = client.call_async(request)
                self._executor.spin_until_future_complete(future, timeout_sec=timeout)
                if not future.done():
                    future.cancel()
                    raise RemoteLauncherError(f"{self.launcher}/{name} did not answer within {timeout:.0f} s")
                return future.result()
            finally:
                self._owner = None

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
        req = _srv().StartLaunch.Request(key=key, cmd=list(cmd))
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
        req = _srv().StopLaunch.Request(key=key, timeout=float(timeout),
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
        res = self._call("status", _srv().LaunchStatus.Request(key=key), QUERY_TIMEOUT_S,
                         service_wait=STATUS_SERVICE_WAIT_S)
        return res.known, res.running, res.return_code

    def cleanup(self, patterns: Iterable[str], kill: bool, wait_timeout: float = 0.0) -> bool:
        req = _srv().CleanupProcesses.Request(patterns=list(patterns), kill=kill,
                                              wait_timeout=float(wait_timeout))
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
            res = self._call("save_rtabmap", _srv().SaveRtabmap.Request(timeout=float(timeout)),
                             float(timeout) + QUERY_TIMEOUT_S)
        except RemoteLauncherError as exc:
            self._log("error", f"rtabmap save failed: {exc}")
            return False
        self._log("info", res.message)
        return res.success

    def close(self):
        if self._fallback is not None:
            self._fallback.close()
            self._fallback = None
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
