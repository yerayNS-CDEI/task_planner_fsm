"""CreateMap: build the map in two structured phases.

Phase 1 (EXPLORE): launch the persistent mapping + nav2 stack (``mapping_cmd``)
and, once it is producing a costmap, launch the frontier-exploration nodes
(``explore_cmd``) as a SEPARATE process. Exploration drives the robot to every
frontier until none remain, then shuts itself down — its launch process exits,
which is how this state detects "exploration finished". Crucially the mapping +
nav2 stack is left RUNNING (it is a different process), so the robot is still
localized, mapping, and navigable for the next phase.

Phase 2 (DENSIFY, optional): drive a uniform free-cell coverage of the
already-explored space — the reachable free area is tiled at a fixed spacing,
every tile gets one waypoint (so all free space is covered, nothing skipped), and
the points are visited in a wall-aware geodesic TSP order (no revisiting) — so
rtabmap keeps integrating scans and the map gets denser/more accurate. The
geometry + navigation are handled by the ``CoverageDriver``, configured from this
state's ``create_map_*`` parameters. Gate it off with
``create_map_densify_enabled:=False``.

The whole stack is torn down in ``on_exit`` before the FSM moves on.
"""

import shlex
import subprocess
import time

from ..state import State
from ..utils.coverage_driver import CoverageConfig, CoverageDriver

from task_planner_fsm.states.proc_utils import (
    start_proc,
    stop_proc,
    wait_stack_ready,
    wait_processes_gone,
    kill_stale_stack,
    SIM_STACK_PATTERNS,
)


class CreateMap(State):
    # Phases.
    STARTING = "starting"     # stack launched, waiting for it to be ready
    EXPLORING = "exploring"   # frontier exploration running
    DENSIFY = "densify"       # structured coverage sweep running
    DONE = "done"
    ERROR = "error"

    def __init__(self, name):
        super().__init__(name)
        self.phase = self.STARTING
        self.driver = None

    def on_enter(self, ctx):
        node = ctx["node"]
        self.phase = self.STARTING
        self.driver = None
        ctx["map_ready"] = False
        ctx["error_triggered"] = False

        # Free hardware resources held by any orphaned driver from a previous
        # run (notably the SICK UDP ports 2115/2116) before relaunching, so the
        # new stack's drivers can bind instead of failing with "Address already
        # in use" and silently breaking odometry/localization.
        kill_stale_stack(node=node)

        cmd = ctx.get("mapping_cmd")
        if isinstance(cmd, str):
            cmd = shlex.split(cmd)
        start_proc(ctx, "mapping", cmd)
        p = ctx.get("_procs", {}).get("mapping")
        node.get_logger().info(
            f"[{self.name}] Mapping + nav2 stack started (pid={p.pid if p else 'unknown'})."
        )

    def run(self, ctx):
        if self.phase == self.STARTING:
            self._run_starting(ctx)
        elif self.phase == self.EXPLORING:
            self._run_exploring(ctx)
        elif self.phase == self.DENSIFY:
            self._run_densify(ctx)

    # ------------------------------------------------------------------
    # Phase STARTING: wait for the stack, then launch exploration.
    # ------------------------------------------------------------------
    def _run_starting(self, ctx):
        node = ctx["node"]

        # Make sure the stack launch did not die immediately.
        proc = ctx.get("_procs", {}).get("mapping")
        if proc and proc.poll() is not None:
            node.get_logger().error(
                f"[{self.name}] Mapping stack exited early (code {proc.poll()})."
            )
            ctx["error_triggered"] = True
            self.phase = self.ERROR
            return

        # Gate exploration on the stack actually producing a costmap, so the
        # frontier explorer has nav2 available when it starts.
        ready_topics = ctx.get("create_map_stack_ready_topics", ["/global_costmap/costmap"])
        ready_timeout = float(ctx.get("create_map_stack_ready_timeout", 120.0))
        node.get_logger().info(
            f"[{self.name}] Waiting up to {ready_timeout:.0f}s for mapping + nav2 stack "
            f"(topics: {ready_topics})..."
        )
        if not wait_stack_ready(ctx, ready_topics, timeout=ready_timeout):
            node.get_logger().error(f"[{self.name}] Mapping + nav2 stack did not become ready.")
            ctx["error_triggered"] = True
            self.phase = self.ERROR
            return

        explore_cmd = ctx.get("explore_cmd")
        if isinstance(explore_cmd, str):
            explore_cmd = shlex.split(explore_cmd)
        start_proc(ctx, "explore", explore_cmd)
        ep = ctx.get("_procs", {}).get("explore")
        node.get_logger().info(
            f"[{self.name}] Frontier exploration started (pid={ep.pid if ep else 'unknown'})."
        )
        self.phase = self.EXPLORING

    # ------------------------------------------------------------------
    # Phase EXPLORING: wait for the explorer to finish (its launch exits).
    # ------------------------------------------------------------------
    def _run_exploring(self, ctx):
        node = ctx["node"]
        proc = ctx.get("_procs", {}).get("explore")
        if proc is None:
            node.get_logger().error(f"[{self.name}] Exploration process missing.")
            ctx["error_triggered"] = True
            self.phase = self.ERROR
            return

        rc = proc.poll()
        if rc is None:
            return  # still exploring

        node.get_logger().info(f"[{self.name}] Exploration finished (launch exited with code {rc}).")
        # Reap the proc handle; the mapping stack keeps running.
        ctx.get("_procs", {}).pop("explore", None)

        if rc != 0:
            ctx["error_triggered"] = True
            self.phase = self.ERROR
            return

        if not bool(ctx.get("create_map_densify_enabled", True)):
            node.get_logger().info(
                f"[{self.name}] create_map_densify_enabled=False: skipping densification sweep."
            )
            ctx["map_ready"] = True
            self.phase = self.DONE
            return

        if not self._start_densify(ctx):
            ctx["error_triggered"] = True
            self.phase = self.ERROR
            return
        self.phase = self.DENSIFY

    # ------------------------------------------------------------------
    # Phase DENSIFY: structured coverage sweep while rtabmap keeps mapping.
    # ------------------------------------------------------------------
    def _start_densify(self, ctx) -> bool:
        node = ctx["node"]
        cfg = self._build_coverage_config(ctx)
        self.driver = CoverageDriver(node, self.name, cfg)
        if not self.driver.ensure_io(ctx, wait_for_server_timeout=10.0):
            return False
        node.get_logger().info(
            f"[{self.name}] Starting densification sweep. Preferred grid: {cfg.costmap_topic} "
            f"(fallback {cfg.map_topic})."
        )
        self.driver.begin(ctx)
        return True

    def _run_densify(self, ctx):
        if self.driver is None:
            ctx["error_triggered"] = True
            self.phase = self.ERROR
            return

        status = self.driver.tick(ctx)
        if self.driver.generated_waypoints:
            ctx["create_map_waypoints"] = self.driver.generated_waypoints
            ctx["create_map_plan_debug"] = self.driver.last_plan_debug
        if status == CoverageDriver.DONE:
            ctx["map_ready"] = True
            self.phase = self.DONE
        elif status == CoverageDriver.FAILED:
            ctx["error_triggered"] = True
            self.phase = self.ERROR

    def _build_coverage_config(self, ctx) -> CoverageConfig:
        return CoverageConfig(
            costmap_topic=str(ctx.get("create_map_costmap_topic", "/global_costmap/costmap")),
            map_topic=str(ctx.get("create_map_map_topic", "/map")),
            allow_map_fallback=bool(ctx.get("create_map_allow_map_fallback", True)),
            costmap_wait_timeout=float(ctx.get("create_map_costmap_wait_timeout", 10.0)),
            # Waypoint spacing (tile size) for the uniform free-cell coverage grid.
            coverage_step_m=float(ctx.get("create_map_coverage_step_m", 3.0)),
            # Permissive connectivity threshold for the reachability flood-fill: a
            # free area joined to the rest only through a narrow gap (1.0-1.8 m,
            # whose centre the costmap inflates to a high-but-not-lethal cost)
            # still flood-fills as reachable, so it gets a waypoint. Keep it just
            # below the inscribed-obstacle cost (~99) -- above that the base could
            # not safely drive there anyway.
            max_traversable_cost=int(ctx.get("create_map_max_traversable_cost", 95)),
            map_occupied_threshold=int(ctx.get("create_map_map_occupied_threshold", 50)),
            # Strict placement threshold (0 = genuine free space): waypoints stay
            # off the inflation band while connectivity above stays permissive.
            waypoint_max_cost=int(ctx.get("create_map_waypoint_max_cost", 0)),
            # Optional standoff eroded from the free/inflation seam, lifting points
            # a little clear of the inflation band.
            wall_clearance_m=float(ctx.get("create_map_wall_clearance_m", 0.1)),
            # Coarsening factor for the geodesic floods used only to ORDER points
            # (not the driven path). Higher = faster planning on a big map. Raise
            # to 4-5 if planning is slow.
            geodesic_downsample=int(ctx.get("create_map_geodesic_downsample", 3)),
            dry_run=bool(ctx.get("create_map_dry_run", False)),
            use_through_poses=bool(ctx.get("create_map_use_nav_through_poses", False)),
            marker_topic="/create_map/coverage_markers",
            marker_namespace="create_map",
            show_heading_arrows=bool(ctx.get("create_map_show_heading_arrows", False)),
        )

    # ------------------------------------------------------------------
    def on_exit(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Exiting state; tearing down the mapping stack cleanly...")

        if self.driver is not None and bool(ctx.get("create_map_clear_markers_on_exit", False)):
            self.driver.clear_markers()

        # Stop the exploration launch first (normally already self-exited), then
        # the persistent mapping + nav2 stack.
        stop_proc(ctx, "explore", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)

        # Graceful, whole-process-group shutdown of the mapping launch
        # (SIGINT -> SIGTERM -> SIGKILL). This lets ros2 launch shut its children
        # down cleanly so DDS participants deregister and rtabmap releases its
        # sqlite database, instead of -9'ing Gazebo only and orphaning
        # robot_state_publisher / controllers / rtabmap / rviz.
        stop_proc(ctx, "mapping", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)

        # Wait until every node from the mapping launch is actually gone, so the
        # next launch (ObjectID) does not collide with orphaned nodes or a locked
        # rtabmap DB. Fixed sleeps are replaced by this real check.
        if not wait_processes_gone(SIM_STACK_PATTERNS, timeout=3.0, node=node):
            node.get_logger().warn(
                f"[{self.name}] Sim-stack processes still present after teardown; force-killing stragglers..."
            )
            for pattern in SIM_STACK_PATTERNS:
                try:
                    subprocess.run(['pkill', '-9', '-f', pattern], timeout=2, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
            wait_processes_gone(SIM_STACK_PATTERNS, timeout=10.0, node=node)

        # Small settle so DDS discovery clears any stale endpoints before the new
        # stack starts publishing.
        time.sleep(3)
        node.get_logger().info(f"[{self.name}] Mapping stack fully stopped.")

    def check_transition(self, ctx):
        if ctx.get("map_ready"):
            return "ObjectID"
        if ctx.get("error_triggered"):
            return "Error"
        return None
