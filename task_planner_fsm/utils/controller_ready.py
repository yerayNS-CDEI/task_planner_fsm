"""Gate that waits until the arm's trajectory controller is actually running.

The dashboard ``play`` command returns as soon as the UR accepts it, but that
is only the start of the sequence: the External Control program still has to
connect back to the driver, ``robot_program_running`` then flips to true, and
only then does ur_robot_driver's ``controller_stopper`` re-activate the
controllers it deactivated while the program was stopped.  That whole round
trip takes roughly a second.

A trajectory sent inside that window is rejected by the controller with
"Can't accept new trajectories. Controller is not running.", so every state
that presses play must wait for the controller to come back before sending a
goal.
"""

from controller_manager_msgs.srv import ListControllers

#: Any of these being active means the arm can accept a trajectory.  Which one
#: is used depends on the launch (passthrough on the real robot, plain JTC under
#: Gazebo), so treat them as interchangeable here.
TRAJECTORY_CONTROLLERS = (
    "passthrough_trajectory_controller",
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
)

DEFAULT_SERVICE = "/controller_manager/list_controllers"


class TrajectoryControllerGate:
    """Poll ``list_controllers`` until a trajectory controller reports active.

    Designed for the FSM's tick loop: ``ready()`` never blocks, it returns
    False while still waiting and True once the controller is up.  On timeout
    it gives up and returns True so the goal is still attempted -- failing open
    keeps the old behaviour rather than stalling the mission.
    """

    def __init__(self, timeout_s=15.0, service=DEFAULT_SERVICE):
        self.timeout_s = timeout_s
        self.service = service
        self._client = None
        self._future = None
        self._deadline = None
        self._ready = False
        self._announced = False

    def reset(self):
        """Re-arm the gate, e.g. on state entry."""
        self._future = None
        self._deadline = None
        self._ready = False
        self._announced = False

    def ready(self, node):
        # Latched: once the controller has come up, stay out of the way.
        if self._ready:
            return True

        now = node.get_clock().now().nanoseconds / 1e9
        if self._deadline is None:
            self._deadline = now + self.timeout_s

        if self._client is None:
            self._client = node.create_client(ListControllers, self.service)

        if self._future is None:
            if not self._client.service_is_ready():
                return self._give_up_on_timeout(node, now, "controller_manager is not reachable")
            self._future = self._client.call_async(ListControllers.Request())
            return self._still_waiting(node, now)

        if not self._future.done():
            return self._give_up_on_timeout(node, now, "list_controllers did not answer")

        future, self._future = self._future, None
        try:
            controllers = future.result().controller
        except Exception as exc:  # noqa: BLE001 - report and retry next tick
            node.get_logger().warn(f"list_controllers call failed: {exc}")
            return self._give_up_on_timeout(node, now, "list_controllers call failed")

        for controller in controllers:
            if controller.name in TRAJECTORY_CONTROLLERS and controller.state == "active":
                node.get_logger().info(f"{controller.name} is active; arm can accept trajectories.")
                self._ready = True
                return True

        return self._still_waiting(node, now)

    def _still_waiting(self, node, now):
        if not self._announced:
            node.get_logger().info(
                "Waiting for the arm trajectory controller to be re-activated "
                "after the External Control program start..."
            )
            self._announced = True
        return self._give_up_on_timeout(node, now, "no trajectory controller became active")

    def _give_up_on_timeout(self, node, now, reason):
        if now > self._deadline:
            node.get_logger().warn(
                f"Gave up waiting for the arm trajectory controller ({reason}); "
                f"sending the goal anyway."
            )
            self._ready = True
            return True
        return False
