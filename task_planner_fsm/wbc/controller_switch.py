"""Swap the arm between trajectory control and streaming velocity control.

Planned arm moves (the FSM's named poses, the Z approach) go through a
``FollowJointTrajectory`` controller; the whole-body sweep instead streams joint
velocities, and ros2_control allows only one controller per command interface.
So the sweep has to claim ``forward_velocity_controller`` for its duration and
give it back afterwards.

Ownership sits with the sweep node rather than the FSM on purpose: whoever holds
the controller should be the process that dies if the sweep dies, so a crash
cannot leave the arm parked on a velocity controller nobody is feeding. The FSM
still checks on the way out (see ``ScanWall._ensure_arm_trajectory_controller``)
in case this node was killed hard enough to skip its own teardown.

Real-robot note: the UR ``force_mode_controller`` only runs alongside
``passthrough_trajectory_controller``, never alongside a streaming controller —
which is why the whole-body sweep is sim-first and the FSM gates it on
``ctx["sim"]``.
"""

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)

# Controllers that own the arm's command interfaces when the sweep is not
# running. Whichever of these is active gets deactivated for the sweep and
# reactivated afterwards.
TRAJECTORY_CONTROLLERS = [
    "joint_trajectory_controller",
    "scaled_joint_trajectory_controller",
    "passthrough_trajectory_controller",
    "forward_position_controller",
]


class ArmControllerSwitch:
    """Claim the velocity controller for a sweep, then put things back."""

    def __init__(self, node):
        self.node = node
        self.velocity_controller = _param(node, "arm_velocity_controller",
                                          "forward_velocity_controller")
        self.trajectory_controllers = list(
            _param(node, "arm_trajectory_controllers", TRAJECTORY_CONTROLLERS))
        self.controller_manager = _param(node, "controller_manager", "/controller_manager")
        self.enabled = bool(_param(node, "manage_controllers", True))
        self.service_timeout = float(_param(node, "controller_service_timeout", 10.0))
        self.deactivated = []
        self.activated = False

    # ------------------------------------------------------------------
    def to_velocity_control(self):
        """Activate the velocity controller, deactivating whoever holds the arm.

        Returns True when the arm is ready to take streamed velocities (also when
        management is disabled — then the caller has arranged it themselves).
        """
        if not self.enabled:
            self.node.get_logger().info("manage_controllers=false: leaving the controller set alone.")
            return True

        states = self._controller_states()
        if states is None:
            return False
        if states.get(self.velocity_controller) == "active":
            self.node.get_logger().info(
                f"'{self.velocity_controller}' is already active.")
            return True
        if not self._ensure_loaded(states):
            return False

        self.deactivated = [name for name in self.trajectory_controllers
                            if states.get(name) == "active"]
        self.node.get_logger().info(
            f"Switching the arm to '{self.velocity_controller}'"
            + (f" (deactivating {', '.join(self.deactivated)})" if self.deactivated else "")
        )
        if not self._switch(activate=[self.velocity_controller], deactivate=self.deactivated):
            self.deactivated = []
            return False
        self.activated = True
        return True

    def restore(self):
        """Undo :meth:`to_velocity_control`. Safe to call when it never ran."""
        if not self.enabled or not self.activated:
            return
        self.node.get_logger().info(
            f"Restoring the arm to {', '.join(self.deactivated) or 'no trajectory controller'}.")
        self._switch(activate=self.deactivated, deactivate=[self.velocity_controller])
        self.activated = False
        self.deactivated = []

    # ------------------------------------------------------------------
    def _controller_states(self):
        """{name: state} for every controller the manager knows about."""
        response = self._call(ListControllers, f"{self.controller_manager}/list_controllers",
                              ListControllers.Request())
        if response is None:
            return None
        return {c.name: c.state for c in response.controller}

    def _ensure_loaded(self, states):
        """Load and configure the velocity controller if it is not there yet.

        Launch files spawn the controllers a robot normally uses, and a streaming
        velocity controller is not one of them — in the Gazebo stack the arm's
        velocity interfaces sit available and unclaimed with no controller for
        them. Loading it here (its type and joints are already declared in the
        controller_manager's parameters) means a whole-body sweep needs no launch
        file change to try.
        """
        state = states.get(self.velocity_controller)
        if state is None:
            self.node.get_logger().info(f"Loading '{self.velocity_controller}'...")
            request = LoadController.Request()
            request.name = self.velocity_controller
            response = self._call(LoadController,
                                  f"{self.controller_manager}/load_controller", request)
            if response is None or not response.ok:
                self.node.get_logger().error(
                    f"Could not load '{self.velocity_controller}'. Is its type declared in "
                    f"the controller_manager's parameters?"
                )
                return False
            state = "unconfigured"
        if state == "unconfigured":
            request = ConfigureController.Request()
            request.name = self.velocity_controller
            response = self._call(ConfigureController,
                                  f"{self.controller_manager}/configure_controller", request)
            if response is None or not response.ok:
                self.node.get_logger().error(
                    f"Could not configure '{self.velocity_controller}'.")
                return False
        return True

    def _switch(self, activate, deactivate):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        # BEST_EFFORT: a controller that is already in the requested state (or
        # simply absent from this robot's set) must not abort the switch.
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        response = self._call(SwitchController, f"{self.controller_manager}/switch_controller",
                              request)
        if response is None or not response.ok:
            self.node.get_logger().error(
                f"switch_controller failed (activate={activate}, deactivate={deactivate}).")
            return False
        return True

    def _call(self, srv_type, service, request):
        """Blocking service call. Only ever used outside the control loop."""
        client = self.node.create_client(srv_type, service)
        try:
            if not client.wait_for_service(timeout_sec=self.service_timeout):
                self.node.get_logger().error(f"'{service}' unavailable.")
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.service_timeout)
            if not future.done():
                self.node.get_logger().error(f"'{service}' timed out.")
                return None
            return future.result()
        finally:
            self.node.destroy_client(client)


def _param(node, name, default):
    """Declare-or-read, so the switch can be dropped into any node."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value
