"""Chassis parking: align the diff-drive chassis with the turret.

Nav2 steers ``turret_footprint``, so after any navigation the turret has the
commanded heading but the chassis wheels sit at an arbitrary angle to it.
sim_controller's ``/sim_controller/park_now`` service rotates the chassis to the
turret's current world heading while the turret joint compensates to hold the
turret (and the arm on it) world-stationary.

Used by ScanWall (square the whole robot to the wall before each sweep) and by
HomePosition (end the mission aligned, so the next run -- a restart included --
starts from the same chassis/turret angle as a fresh one; the controller follows
commands poorly once that angle passes ~90 deg).

``park_now`` is live-gated by the controller's ``enable_park_service`` parameter
(kept false by default), so one maneuver is a small non-blocking sequence driven
from the owner state's tick::

    enable -> request -> settle -> disable

Best-effort at every step: a missing service or a failed/rejected call skips
gracefully, and ``done`` is set when the maneuver finished or was skipped.

ctx knobs, with ``<prefix>`` the owner's (``scan_wall``, ``home_position``):
    <prefix>_park_base       false skips parking (benches without the base controller)
    <prefix>_park_grace_s    wait this long for parking_active to go true (5 s);
                             if it never does, the chassis was already aligned
    <prefix>_park_timeout_s  give up waiting for the maneuver (120 s)
    park_service             Trigger service (/sim_controller/park_now)
    park_set_param_service   /sim_controller/set_parameters
The settle phase reads ``ctx["parking_active"]``, which fsm_node keeps from the
latched ``/sim_controller/parking_active`` topic.
"""

import time

from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger


class ChassisParker:
    def __init__(self, owner_name, knob_prefix):
        self.owner_name = owner_name
        self.knob_prefix = knob_prefix
        self.park_client = None            # ~/park_now Trigger client (created once)
        self.park_enable_client = None     # /set_parameters client (created once)
        self.reset()

    def reset(self):
        """Start a fresh enable->request->settle->disable cycle on the next step()."""
        self.done = False
        self._phase = "enable"
        self._park_future = None
        self._param_future = None
        self._saw_active = False
        self._wait_start = None

    def _knob(self, ctx, name, default):
        return ctx.get(f"{self.knob_prefix}_{name}", default)

    def _send_park_enabled(self, ctx, value):
        """Flip the controller's enable_park_service parameter via /set_parameters.

        Returns the call future, or None if the parameter service is unavailable.
        """
        node = ctx["node"]
        set_param_srv = ctx.get("park_set_param_service", "/sim_controller/set_parameters")
        if self.park_enable_client is None:
            self.park_enable_client = node.create_client(SetParameters, set_param_srv)
        if not self.park_enable_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().warn(
                f"[{self.owner_name}] Parameter service '{set_param_srv}' unavailable; "
                f"cannot toggle enable_park_service."
            )
            return None
        req = SetParameters.Request()
        pmsg = ParameterMsg()
        pmsg.name = "enable_park_service"
        pmsg.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
        req.parameters = [pmsg]
        return self.park_enable_client.call_async(req)

    def _param_set_ok(self, ctx, future, label):
        """True if a /set_parameters future succeeded; logs and returns False on an
        exception or a rejected result."""
        node = ctx["node"]
        try:
            results = future.result().results
        except Exception as e:
            node.get_logger().warn(
                f"[{self.owner_name}] set enable_park_service ({label}) call failed: {e}"
            )
            return False
        ok = bool(results) and all(r.successful for r in results)
        if not ok:
            reason = results[0].reason if results else "no result"
            node.get_logger().warn(
                f"[{self.owner_name}] set enable_park_service ({label}) rejected: {reason}"
            )
        return ok

    def step(self, ctx):
        """Advance the maneuver by one phase. Returns ``done``; call every tick."""
        if not self.done:
            self._step(ctx)
        return self.done

    def _step(self, ctx):
        node = ctx["node"]

        # Opt-out hook (e.g. benches without the base controller).
        if not bool(self._knob(ctx, "park_base", True)):
            self.done = True
            return

        # --- enable the live-gated park service for this maneuver. ---
        if self._phase == "enable":
            self._param_future = self._send_park_enabled(ctx, True)
            if self._param_future is None:
                self.done = True  # no param service -> skip (parameter stays false)
                return
            node.get_logger().info(
                f"[{self.owner_name}] Parking: enabling park service before aligning the base."
            )
            self._phase = "enable_wait"
            return

        if self._phase == "enable_wait":
            if not self._param_future.done():
                return
            if not self._param_set_ok(ctx, self._param_future, "enable"):
                # Couldn't enable -> park_now would refuse; skip. The parameter is
                # unchanged (still false), so no revert is needed.
                self.done = True
                self._param_future = None
                return
            self._param_future = None
            self._phase = "request"
            return

        # --- request the maneuver. ---
        if self._phase == "request":
            park_service = ctx.get("park_service", "/sim_controller/park_now")
            if self.park_client is None:
                self.park_client = node.create_client(Trigger, park_service)
            if not self.park_client.wait_for_service(timeout_sec=2.0):
                node.get_logger().warn(
                    f"[{self.owner_name}] Park service '{park_service}' unavailable; "
                    f"skipping base alignment."
                )
                self._phase = "disable"  # revert the parameter we just enabled
                return
            node.get_logger().info(
                f"[{self.owner_name}] Parking: aligning the chassis to the turret heading."
            )
            self._park_future = self.park_client.call_async(Trigger.Request())
            self._saw_active = False
            self._wait_start = time.time()
            self._phase = "accept_wait"
            return

        if self._phase == "accept_wait":
            if not self._park_future.done():
                return
            try:
                resp = self._park_future.result()
                if not resp.success:
                    node.get_logger().warn(
                        f"[{self.owner_name}] Park request not accepted ({resp.message}); "
                        f"proceeding without base alignment."
                    )
                    self._phase = "disable"
                    self._park_future = None
                    return
                node.get_logger().info(f"[{self.owner_name}] Park started: {resp.message}")
            except Exception as e:
                node.get_logger().warn(
                    f"[{self.owner_name}] Park service call failed ({e}); proceeding "
                    f"without base alignment."
                )
                self._phase = "disable"
                self._park_future = None
                return
            self._park_future = None
            self._phase = "settle"
            return

        # --- wait for the maneuver to finish (latched parking_active). ---
        if self._phase == "settle":
            grace_s = float(self._knob(ctx, "park_grace_s", 5.0))
            timeout_s = float(self._knob(ctx, "park_timeout_s", 120.0))
            active = ctx.get("parking_active")
            elapsed = time.time() - self._wait_start

            if elapsed > timeout_s:
                node.get_logger().warn(
                    f"[{self.owner_name}] Parking did not confirm after {timeout_s:.0f}s; "
                    f"proceeding anyway."
                )
                self._phase = "disable"
                return
            if active:
                self._saw_active = True
                return
            # active is False or None here.
            if self._saw_active:
                node.get_logger().info(f"[{self.owner_name}] Chassis aligned to the turret.")
                self._phase = "disable"
                return
            if elapsed >= grace_s:
                node.get_logger().info(
                    f"[{self.owner_name}] Chassis already aligned (parking never went active)."
                )
                self._phase = "disable"
            return

        # --- disable the park service again, then finish. ---
        if self._phase == "disable":
            self._param_future = self._send_park_enabled(ctx, False)
            if self._param_future is None:
                node.get_logger().warn(
                    f"[{self.owner_name}] Could not disable park service (param service "
                    f"gone); leaving enable_park_service as-is."
                )
                self.done = True
                return
            self._phase = "disable_wait"
            return

        if self._phase == "disable_wait":
            if not self._param_future.done():
                return
            self._param_set_ok(ctx, self._param_future, "disable")  # best-effort log
            self._param_future = None
            node.get_logger().info(f"[{self.owner_name}] Parking done; park service disabled.")
            self.done = True
            return
