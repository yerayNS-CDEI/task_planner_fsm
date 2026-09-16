"""Hand the wall's drilling targets to POKEYE and wait for its acknowledgement.

Entered only when SensorDataProcessing produced at least one target for the
wall just scanned (``ctx["pokeye_targets"]``, see ``sensors/pokeye.py``). The
state writes the full request as JSON next to the decisions, then calls the
``/send_data_to_pokeye`` service (``arm_control/SendPokeyeTargets``) with the
compact form -- positions, ids, reasons -- and the path of that JSON for
anything richer. A service, not a topic, because the whole point of this state
is knowing the data arrived: the response says how many targets POKEYE accepted.

The request also carries what GPR allows on this wall (POKEYE policy v2): the
``NO_DRILL`` zones around detected hyperbolae, and the scanned lines that are
the only drillable region. The targets in it have already been screened against
both, so they are not a filter POKEYE has to re-apply to what it was sent --
they are there because POKEYE drills for reasons the FSM did not choose, RANDOM
locations included, and those have to obey the same two rules: on a scanned
line, off every hyperbola. Sending nothing would leave POKEYE to pick a spot
over a pipe, or over a stretch of wall nobody has looked behind, with no way of
knowing either.

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

from ..sensors import no_drill, paths
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
        zones = list(ctx.get("pokeye_no_drill_zones") or [])
        lines = list(ctx.get("pokeye_scanned_lines") or [])
        if not targets:
            # The transition guard should make this unreachable; be explicit
            # rather than send an empty request.
            node.get_logger().warn(
                f"[{self.name}] entered with no POKEYE targets; nothing to send.")
            ctx["data_sent"] = True
            return

        request_path = self._write_request_json(ctx, targets, zones, lines)
        ctx["pokeye_request_json"] = request_path
        self._request = self._build_request(ctx, targets, zones, lines, request_path)
        node.get_logger().info(
            f"[{self.name}] {len(targets)} POKEYE target(s) for wall "
            f"{ctx.get('current_wall_index')} -> {request_path}"
            + f" (+{len(zones)} NO_DRILL zone(s), {len(lines)} scanned line(s))"
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
    def _write_request_json(self, ctx, targets, zones, lines):
        out_dir = paths.pokeye_results_dir(ctx)
        os.makedirs(out_dir, exist_ok=True)
        path = out_dir / REQUEST_FILENAME
        stats = ctx.get("pokeye_no_drill_stats") or {}
        payload = {
            "session_id": paths.session_id(ctx),
            "wall_index": ctx.get("current_wall_index"),
            "frame_id": "map",
            "n_targets": len(targets),
            "targets": targets,
            "drilling_constraints": {
                "coordinate_frame": "map",
                "policy": "A detected GPR hyperbola forbids drilling at its "
                          "position, whatever sent POKEYE there (RANDOM included).",
                "targets_already_screened": True,
                "exclusion_tolerance_m": stats.get("tolerance_m"),
                "exclusion_tolerance_source": (
                    "ctx:pokeye_no_drill_tolerance_m"
                    if ctx.get("pokeye_no_drill_tolerance_m") not in (None, "")
                    else "fsm_placeholder_pending_project_approval"),
                "n_no_drill_zones": len(zones),
                "no_drill_zones": zones,
                "n_unlocated": stats.get("n_unlocated", 0),
            },
            "drillable_region": {
                "coordinate_frame": "map",
                "policy": "Only the GPR lines scanned and analysed on this wall may "
                          "be drilled, RANDOM locations included. Nowhere else has "
                          "been looked behind.",
                "enforced": bool(ctx.get("pokeye_require_gpr_coverage", True)),
                "line_tolerance_m": stats.get("line_tolerance_m"),
                "n_scanned_lines": len(lines),
                "scanned_lines": lines,
            },
            "blocked_targets": ctx.get("pokeye_blocked_targets") or [],
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
    def _build_request(ctx, targets, zones, lines, request_path):
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
        for z in zones:
            p = Point()
            p.x, p.y, p.z = (float(v) for v in z["position"])
            req.no_drill_positions.append(p)
            req.no_drill_radii.append(float(z.get("radius_m", 0.0)))
            req.no_drill_ids.append(str(z.get("zone_id", "")))
        for ln in lines:
            start, end = Point(), Point()
            start.x, start.y, start.z = (float(v) for v in ln["seg_start"])
            end.x, end.y, end.z = (float(v) for v in ln["seg_end"])
            req.scanned_line_starts.append(start)
            req.scanned_line_ends.append(end)
            req.scanned_line_ids.append(str(ln.get("line_id", "")))
        stats = ctx.get("pokeye_no_drill_stats") or {}
        req.n_no_drill_unlocated = int(stats.get("n_unlocated", 0))
        req.scanned_line_tolerance = float(
            stats.get("line_tolerance_m") or no_drill.DEFAULT_LINE_TOLERANCE_M)
        return req
