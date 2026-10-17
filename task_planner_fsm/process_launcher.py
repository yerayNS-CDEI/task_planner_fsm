#!/usr/bin/env python3
"""Starts and stops launches on this machine for an FSM running on another one.

The robot's load is split between two computers: the Jetson keeps the base,
arm, column and this FSM, and PC2 runs perception and navigation (lidars,
cameras, rtabmap, Nav2), brought up with navi-wall's ``stack:=perception``.
The FSM still decides when those launches start and stop, so this node runs on
PC2 (started at boot) and does it for the FSM, over ROS services::

    ~/start           navi_wall/srv/StartLaunch       ros2 launch <pkg> ... under a key
    ~/stop            navi_wall/srv/StopLaunch        SIGINT the group, escalate after timeout
    ~/status          navi_wall/srv/LaunchStatus      still running? exit code?
    ~/cleanup         navi_wall/srv/CleanupProcesses  kill stale drivers / wait until gone
    ~/save_rtabmap    navi_wall/srv/SaveRtabmap       SIGINT rtabmap alone, wait for the save

and publishes the console output of its launches, one line per message as
``<key>| <line>``, so the UI on the FSM's computer can show it::

    ~/output          std_msgs/String

Everything is done here with the FSM's own proc_utils, exactly as the FSM does
it for local launches: process groups, the SIGINT -> SIGTERM -> SIGKILL
escalation, and the SIGINT-only rtabmap save. Signals never cross the network,
so a dropped link cannot SIGKILL rtabmap mid-save.

The FSM side is ``states/remote_procs.py``; it is enabled by giving the FSM
``-p perception_launcher:=/process_launcher``.

Only ``ros2 launch`` of the packages in ``allowed_packages`` (default
navi_wall) is accepted, so the node cannot be used to run arbitrary commands.

Run it with::

    ros2 run task_planner_fsm process_launcher
"""
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from navi_wall.srv import CleanupProcesses, LaunchStatus, SaveRtabmap, StartLaunch, StopLaunch
from task_planner_fsm.states import proc_utils


class ProcessLauncher(Node):
    def __init__(self):
        super().__init__("process_launcher")
        self.declare_parameter("allowed_packages", ["navi_wall"])
        # proc_utils keeps its Popen handles in ctx["_procs"] and logs through
        # ctx["node"], the same layout the FSM uses.
        self._ctx = {"node": self}
        # Each launch's output, still echoed to this node's own stdout.
        self._output_pub = self.create_publisher(String, "~/output", QoSProfile(depth=200))
        self._ctx["_output_sink"] = self._publish_output
        # One start/stop at a time; status queries go through while a long stop
        # (column retraction, rtabmap save) is in progress.
        self._lock = threading.Lock()

        group = ReentrantCallbackGroup()
        self.create_service(StartLaunch, "~/start", self._start, callback_group=group)
        self.create_service(StopLaunch, "~/stop", self._stop, callback_group=group)
        self.create_service(LaunchStatus, "~/status", self._status, callback_group=group)
        self.create_service(CleanupProcesses, "~/cleanup", self._cleanup, callback_group=group)
        self.create_service(SaveRtabmap, "~/save_rtabmap", self._save_rtabmap, callback_group=group)
        self.get_logger().info(
            f"Ready. Accepting ros2 launch of: {self._allowed_packages()}")

    def _publish_output(self, key, line):
        self._output_pub.publish(String(data=f"{key}| {line}"))

    def _allowed_packages(self):
        return list(self.get_parameter("allowed_packages").value)

    def _start(self, request, response):
        cmd = list(request.cmd)
        if len(cmd) < 4 or cmd[:2] != ["ros2", "launch"] or cmd[2] not in self._allowed_packages():
            response.success = False
            response.message = (
                f"Rejected {cmd[:3]}: only 'ros2 launch <pkg> <file>' with pkg in "
                f"{self._allowed_packages()} is allowed")
            self.get_logger().error(response.message)
            return response
        with self._lock:
            self.get_logger().info(f"Starting '{request.key}': {' '.join(cmd)}")
            try:
                proc_utils.start_proc(self._ctx, request.key, cmd)
            except OSError as exc:
                response.success = False
                response.message = f"Could not start '{request.key}': {exc}"
                self.get_logger().error(response.message)
                return response
            proc = self._ctx["_procs"][request.key]
        response.success = True
        response.pid = proc.pid
        response.message = f"'{request.key}' running, pid {proc.pid}"
        return response

    def _stop(self, request, response):
        with self._lock:
            known = request.key in self._ctx.get("_procs", {})
            proc_utils.stop_proc(
                self._ctx, request.key,
                timeout=request.timeout if request.timeout > 0 else 10.0,
                force_kill_patterns=list(request.force_kill_patterns) or None)
        response.success = True
        response.message = f"'{request.key}' stopped" if known else f"'{request.key}' was not running"
        return response

    def _status(self, request, response):
        proc = self._ctx.get("_procs", {}).get(request.key)
        response.known = proc is not None
        if proc is not None:
            rc = proc.poll()
            response.running = rc is None
            response.return_code = rc if rc is not None else 0
        return response

    def _cleanup(self, request, response):
        patterns = list(request.patterns)
        killed = []
        if request.kill:
            killed = proc_utils.kill_stale_stack(node=self, patterns=patterns)
        gone = True
        if request.wait_timeout > 0:
            gone = proc_utils.wait_processes_gone(patterns, timeout=request.wait_timeout, node=self)
        response.success = gone
        response.killed_pids = killed
        response.message = f"killed {len(killed)}, {'none left' if gone else 'some still running'}"
        return response

    def _save_rtabmap(self, request, response):
        timeout = request.timeout if request.timeout > 0 else 180.0
        response.success = proc_utils.graceful_rtabmap_save(node=self, timeout=timeout)
        response.message = ("rtabmap saved and exited" if response.success
                            else "rtabmap did not save (still running, killed or crashed)")
        return response

    def stop_all(self):
        """Stop every launch still running, saving rtabmap first."""
        proc_utils.stop_all(self._ctx)


def main():
    rclpy.init()
    node = ProcessLauncher()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
