"""Hand the wall's drilling targets to POKEYE and wait for its acknowledgement.

Entered only when SensorDataProcessing produced at least one target for the
wall just scanned (``ctx["pokeye_targets"]``, see ``sensors/pokeye.py``). The
state writes the full request as JSON next to the decisions, then calls the
``/send_data_to_pokeye`` service (``arm_control/SendPokeyeTargets``) with the
compact form -- positions, ids, reasons -- and the path of that JSON for
anything richer. A service, not a topic, because the whole point of this state
is knowing the data arrived: the response says how many targets POKEYE accepted.

The service call is made from run(), not on_enter(): waiting for the server
inside on_enter blocks the FSM tick, and a failure there used to leave the
state ticking on a None future until the retry. Here a missing server is a
timed wait that ends in fail(), like the other service-backed states.
"""

import json
import os
import time

from geometry_msgs.msg import Point

from arm_control.srv import SendPokeyeTargets

from ..sensors import paths
from ..state import State

REQUEST_FILENAME = "pokeye_request.json"


class SendDataToPokeye(State):
    SERVICE_NAME = "/send_data_to_pokeye"
    # How long to wait for the service to appear / answer before failing the
    # state (and taking the machine's one retry).
    SERVICE_TIMEOUT_S = 10.0

    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None
        self._wait_started = None
        self._request = None

    def on_enter(self, ctx):
        node = ctx["node"]
        ctx["data_sent"] = False
        ctx["error_triggered"] = False
        self.future = None
        self._wait_started = None
        self._request = None

        targets = list(ctx.get("pokeye_targets") or [])
        if not targets:
            # The transition guard should make this unreachable; be explicit
            # rather than send an empty request.
            node.get_logger().warn(
                f"[{self.name}] entered with no POKEYE targets; nothing to send.")
            ctx["data_sent"] = True
            return

        request_path = self._write_request_json(ctx, targets)
        ctx["pokeye_request_json"] = request_path
        self._request = self._build_request(ctx, targets, request_path)
        node.get_logger().info(
            f"[{self.name}] {len(targets)} POKEYE target(s) for wall "
            f"{ctx.get('current_wall_index')} -> {request_path}"
        )

    def run(self, ctx):
        node = ctx["node"]
        if self._request is None or ctx.get("data_sent"):
            return
        n = len(self._request.target_ids)
        self.set_activity(ctx, f"Sending {n} target(s) to Pokeye")

        if self.future is None:
            if self.client is None:
                self.client = node.create_client(
                    SendPokeyeTargets, str(ctx.get("pokeye_service", self.SERVICE_NAME)))
            if not self.client.service_is_ready():
                if self._wait_started is None:
                    self._wait_started = time.monotonic()
                    node.get_logger().info(
                        f"[{self.name}] waiting for {self.client.srv_name} ...")
                if time.monotonic() - self._wait_started > float(
                        ctx.get("pokeye_service_timeout_s", self.SERVICE_TIMEOUT_S)):
                    self.fail(ctx, f"service {self.client.srv_name} not available")
                return
            self.future = self.client.call_async(self._request)
            self._wait_started = time.monotonic()
            return

        if not self.future.done():
            if time.monotonic() - self._wait_started > float(
                    ctx.get("pokeye_service_timeout_s", self.SERVICE_TIMEOUT_S)):
                self.client.remove_pending_request(self.future)
                self.future = None
                self.fail(ctx, "Pokeye did not answer in time")
            return

        result = self.future.result()
        self.future = None
        if result is None or not result.success:
            detail = getattr(result, "message", "") if result is not None else "no response"
            self.fail(ctx, f"Pokeye refused the targets: {detail}")
            return
        node.get_logger().info(
            f"[{self.name}] Pokeye accepted {result.accepted_count}/{n} target(s)"
            + (f": {result.message}" if result.message else "")
        )
        ctx["pokeye_accepted_count"] = int(result.accepted_count)
        ctx["data_sent"] = True

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        if ctx.get("data_sent"):
            return "ArmFolding"
        return None

    # ------------------------------------------------------------------
    def _write_request_json(self, ctx, targets):
        out_dir = paths.pokeye_results_dir(ctx)
        os.makedirs(out_dir, exist_ok=True)
        path = out_dir / REQUEST_FILENAME
        payload = {
            "session_id": paths.session_id(ctx),
            "wall_index": ctx.get("current_wall_index"),
            "frame_id": "map",
            "n_targets": len(targets),
            "targets": targets,
            "sources": {
                "hsi_result_json": ctx.get("hsi_result_json"),
                "hsi_samples_csv": ctx.get("hsi_samples_csv"),
                "decisions_json": ctx.get("pokeye_decisions_json"),
                "targets_json": ctx.get("pokeye_targets_json"),
                "gpr_summary_json": ctx.get("gpr_summary_json"),
            },
        }
        with open(path, "w") as handle:
            json.dump(payload, handle, indent=2)
        return str(path)

    @staticmethod
    def _build_request(ctx, targets, request_path):
        req = SendPokeyeTargets.Request()
        req.session_id = str(paths.session_id(ctx))
        wall = ctx.get("current_wall_index")
        req.wall_index = int(wall) if wall is not None else -1
        req.frame_id = "map"
        req.request_json_path = str(request_path)
        for t in targets:
            p = Point()
            p.x, p.y, p.z = (float(v) for v in t["position"])
            req.positions.append(p)
            req.target_ids.append(str(t.get("target_id", "")))
            req.requested_actions.append(str(t.get("requested_action", "")))
            req.reasons.append(str(t.get("reason", "")))
            req.sample_counts.append(int(t.get("n_samples", 0)))
        return req
