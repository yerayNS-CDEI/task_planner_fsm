"""ObjectID: navigate the base over the whole space while YOLO runs.

This state plans a single-direction boustrophedon (lawnmower) coverage route
over the already-inflated nav2 global costmap, so the omnidirectional base
visits *all* reachable free space once — in clean wall-aligned lanes, without
doubling back or revisiting — giving YOLO a chance to see objects from short
range anywhere in the room. The route is published as RViz markers for
debugging and executed with Nav2's NavigateToPose action.

The coverage geometry and execution are handled by the shared
``utils/coverage_driver.CoverageDriver`` (it wraps ``utils/floor_coverage``):
it aligns lanes to the dominant wall angle, splits each lane around obstacles
and sweeps room-by-room, so the plan is structured and revisit-free by
construction. ``CreateMap`` uses the same driver for its densification sweep,
with its OWN parameters, so the two sweeps are tuned independently.

Config is read from ``ctx`` (all keys optional):
  object_id_sweep_enabled         run the coverage sweep at all (default True)
  object_id_line_spacing_m        lane spacing in metres (default 1.5)
  object_id_max_traversable_cost  costmap cost <= this is drivable (default 65)
  object_id_min_segment_length_m  drop lane runs shorter than this (default 0.5)
  object_id_wall_clearance_m      extra margin on top of the costmap (default 0.4)
  object_id_sweep_axis            "auto" | "dominant" | "perpendicular"
  object_id_room_split_erosion_m  room-by-room split threshold (default 1.3)
  object_id_costmap_topic         default "/global_costmap/costmap"
  object_id_map_topic             raw-map fallback, default "/map"
  object_id_dry_run               plan + markers only, skip base motion
  object_id_use_mock              legacy /object_id_sim service path
  object_id_yolo_cmd              optional command to launch YOLO
"""

import shlex
import socket
import time

from example_interfaces.srv import SetBool

from ..state import State
from ..utils.coverage_driver import CoverageConfig, CoverageDriver
from task_planner_fsm.states.proc_utils import (
    start_proc,
    stop_proc,
    wait_stack_ready,
    wait_services_ready,
    SIM_STACK_PATTERNS,
)


class ObjectID(State):
    def __init__(self, name):
        super().__init__(name)

        # Legacy/mock service fields.
        self.client = None
        self.future = None

        # Shared coverage driver (planning + markers + navigation).
        self.driver = None

        self.sim_value = None
        self.phase = "idle"

    # ------------------------------------------------------------------
    # FSM lifecycle
    # ------------------------------------------------------------------
    def on_enter(self, ctx):
        node = ctx["node"]

        ctx["object_id_ready"] = False
        ctx["error_triggered"] = False
        self.phase = "starting"
        self.driver = None

        if not self._start_robot_stack(ctx):
            self.phase = "error"
            return

        self.sim_value = "true" if ctx.get("sim", False) else "false"

        # Legacy mock path for quick FSM-only tests.
        if bool(ctx.get("object_id_use_mock", False)):
            self._start_mock_object_id(ctx)
            return

        self._start_object_id_detection(ctx)
        self._start_optional_yolo_process(ctx)

        # Optionally skip the coverage sweep entirely (e.g. to avoid driving the
        # whole room twice when CreateMap already ran a densification sweep). YOLO
        # and the nav stack still come up; the state just transitions through.
        if not bool(ctx.get("object_id_sweep_enabled", True)):
            node.get_logger().info(
                f"[{self.name}] object_id_sweep_enabled=False: skipping coverage sweep."
            )
            ctx["object_id_ready"] = True
            ctx["object_id_data"] = ctx.get("walls_data")
            self.phase = "done"
            return

        cfg = self._build_coverage_config(ctx)
        self.driver = CoverageDriver(node, self.name, cfg)
        if not self.driver.ensure_io(ctx, wait_for_server_timeout=10.0):
            ctx["error_triggered"] = True
            self.phase = "error"
            return

        node.get_logger().info(
            f"[{self.name}] Coverage planner ready. Preferred grid: {cfg.costmap_topic} "
            f"(fallback {cfg.map_topic})."
        )
        self.driver.begin(ctx)
        self.phase = "running"

    def run(self, ctx):
        if self.phase == "mock_waiting":
            self._run_mock_object_id(ctx)
            return

        if self.phase in {"idle", "error", "done", "starting"}:
            return

        if self.phase == "running" and self.driver is not None:
            status = self.driver.tick(ctx)
            # Surface plan results for debugging / downstream once computed.
            if self.driver.generated_waypoints:
                ctx["object_id_waypoints"] = self.driver.generated_waypoints
                ctx["object_id_plan_debug"] = self.driver.last_plan_debug
            if status == CoverageDriver.DONE:
                ctx["object_id_ready"] = True
                ctx["object_id_data"] = ctx.get("walls_data")
                self.phase = "done"
            elif status == CoverageDriver.FAILED:
                ctx["error_triggered"] = True
                self.phase = "error"

    def on_exit(self, ctx):
        if self.driver is not None and bool(ctx.get("object_id_clear_markers_on_exit", False)):
            self.driver.clear_markers()
        # Stop the YOLO object-detection launch when leaving the state. The
        # navigation stack ("nav_sim") is intentionally left running for the
        # following states; only the object-ID process is torn down here. This
        # is a no-op in simulation (the process was never started).
        stop_proc(ctx, "object_id", timeout=10.0)
        if bool(ctx.get("object_id_stop_yolo_on_exit", False)):
            stop_proc(ctx, "object_id_yolo")

    def check_transition(self, ctx):
        if ctx.get("object_id_ready"):
            return "WallLinesComputation"
        if ctx.get("error_triggered"):
            return "Error"
        return None

    # ------------------------------------------------------------------
    # Coverage configuration
    # ------------------------------------------------------------------
    def _build_coverage_config(self, ctx) -> CoverageConfig:
        return CoverageConfig(
            costmap_topic=str(ctx.get("object_id_costmap_topic", "/global_costmap/costmap")),
            map_topic=str(ctx.get("object_id_map_topic", "/map")),
            allow_map_fallback=bool(ctx.get("object_id_allow_map_fallback", True)),
            costmap_wait_timeout=float(ctx.get("object_id_costmap_wait_timeout", 10.0)),
            line_spacing_m=float(ctx.get("object_id_line_spacing_m", 1.5)),
            min_segment_length_m=float(ctx.get("object_id_min_segment_length_m", 0.5)),
            wall_clearance_m=float(ctx.get("object_id_wall_clearance_m", 0.4)),
            axis=str(ctx.get("object_id_sweep_axis", "auto")),
            room_split_erosion_m=float(ctx.get("object_id_room_split_erosion_m", 1.3)),
            max_traversable_cost=int(ctx.get("object_id_max_traversable_cost", 65)),
            map_occupied_threshold=int(ctx.get("object_id_map_occupied_threshold", 50)),
            dry_run=bool(ctx.get("object_id_dry_run", False)),
            # Default to sequential NavigateToPose: it is robust to a single
            # hard-to-reach waypoint (it just retries/skips that one), whereas
            # NavigateThroughPoses fails the whole plan if any pose is
            # unreachable. Opt back in with object_id_use_nav_through_poses:=True.
            use_through_poses=bool(ctx.get("object_id_use_nav_through_poses", False)),
            marker_topic="/object_id/coverage_markers",
            marker_namespace="object_id",
            show_heading_arrows=bool(ctx.get("object_id_show_heading_arrows", False)),
        )

    # ------------------------------------------------------------------
    # Stack and optional process setup
    # ------------------------------------------------------------------
    def _start_robot_stack(self, ctx):
        """Launch navigation/localization stack. Returns True when ready."""
        node = ctx["node"]
        requested_sim = bool(ctx.get("sim", False))
        planner_backend = str(ctx.get("planner_backend", "legacy")).strip().lower()
        # Honour the sim value from the context, same as CreateMap / machine.py
        # (which build mapping_cmd from ctx.get('sim')). No more forcing sim:=true.
        sim_value = "true" if requested_sim else "false"
        p = ctx.get("_procs", {}).get("nav_sim")
        if p and p.poll() is None:
            node.get_logger().info(f"[{self.name}] Navigation already running (pid={p.pid}).")
            return True

        try:
            robot_ip = str(ctx.get("robot_ip", "192.168.1.102"))
            robot_port = int(ctx.get("robot_port", 29999))

            if not requested_sim:
                node.get_logger().info(f"[{self.name}] Checking UR robot connectivity at {robot_ip}:{robot_port}...")
                robot_ready = False
                for attempt in range(5):
                    try:
                        sock = socket.create_connection((robot_ip, robot_port), timeout=3)
                        sock.close()
                        robot_ready = True
                        node.get_logger().info(f"[{self.name}] UR robot is reachable at {robot_ip}")
                        time.sleep(2)
                        break
                    except (socket.timeout, socket.error) as e:
                        node.get_logger().warn(
                            f"[{self.name}] UR robot not reachable (attempt {attempt + 1}/5): {e}"
                        )
                        if attempt < 4:
                            time.sleep(3)

                if not robot_ready:
                    node.get_logger().error(
                        f"[{self.name}] UR robot not reachable at {robot_ip}:{robot_port} after 5 attempts."
                    )
                    ctx["error_triggered"] = True
                    return False

            node.get_logger().info(
                f"[{self.name}] Launching navigation stack with Gazebo/Nav2 (sim:={sim_value}, hybrid_sim:=false)..."
            )
            start_proc(
                ctx,
                "nav_sim",
                [
                    "ros2", "launch", "navi_wall", "move_robot.launch.py",
                    f"sim:={sim_value}", "mode:=full", "controller_type:=omni", "hybrid_sim:=false",
                    f"robot_ip:={robot_ip}", "database_name:=rtabmap_fsm", "headless:=true",
                    f"planner_backend:={planner_backend}",
                    f"use_sim_time:={sim_value}",
                ],
            )

            # Wait until the stack is actually producing data instead of a blind
            # sleep. We gate on topics that are known to exist in this system and
            # that map directly to the symptoms we want to avoid:
            #   /clock          -> sim time is running (SIMULATION ONLY)
            #   /tf             -> robot_state_publisher / TF is up (robot model)
            #   /joint_states   -> ros2_control controllers are active
            #   /rtabmap/odom   -> rtabmap odometry is initialized and receiving data
            #
            # /clock is only published when sim time is running, so it must NOT be
            # required on the real robot — otherwise wait_stack_ready always times
            # out, _start_robot_stack returns False, and on_enter bails out before
            # ever launching the object-ID (YOLO) stack.
            default_ready_topics = ["/tf", "/joint_states", "/rtabmap/odom"]
            if requested_sim:
                default_ready_topics = ["/clock"] + default_ready_topics
            required_topics = ctx.get("stack_ready_topics", default_ready_topics)
            ready_timeout = float(ctx.get("stack_ready_timeout", 90.0))
            node.get_logger().info(
                f"[{self.name}] Waiting up to {ready_timeout:.0f}s for navigation + localization stack..."
            )
            if not wait_stack_ready(ctx, required_topics, timeout=ready_timeout):
                node.get_logger().error(f"[{self.name}] Navigation + localization stack did not become ready.")
                stop_proc(ctx, "nav_sim", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)
                ctx["error_triggered"] = True
                return False

            if planner_backend == "legacy":
                collision_services = ctx.get(
                    "collision_ready_services",
                    ["/collision/check_collision_pose"],
                )
                collision_timeout = float(ctx.get("collision_ready_timeout", 60.0))
                node.get_logger().info(
                    f"[{self.name}] Waiting up to {collision_timeout:.0f}s for collision checking service..."
                )
                if not wait_services_ready(ctx, collision_services, timeout=collision_timeout):
                    node.get_logger().error(f"[{self.name}] Collision checking service did not come up.")
                    stop_proc(ctx, "nav_sim", timeout=10.0, force_kill_patterns=SIM_STACK_PATTERNS)
                    ctx["error_triggered"] = True
                    return False

            node.get_logger().info(f"[{self.name}] Navigation + localization stack is ready.")
            return True

        except Exception as e:
            node.get_logger().error(f"[{self.name}] Could not start navigation + localization process: {e}")
            ctx["error_triggered"] = True
            return False

    def _start_object_id_detection(self, ctx):
        """Launch the YOLO object-detection stack (real robot only).

        The camera + YOLO model only run on the physical robot; in simulation
        there is no real camera feed, so the launch is skipped and only the
        space-coverage route is executed. The navigation stack ("nav_sim") is
        kept running for the following states; this process is torn down in
        ``on_exit``.
        """
        node = ctx["node"]
        if bool(ctx.get("sim", False)):
            node.get_logger().info(
                f"[{self.name}] Simulation: skipping YOLO object-detection launch "
                f"(running space coverage only)."
            )
            return

        proc = ctx.get("_procs", {}).get("object_id")
        if proc and proc.poll() is None:
            node.get_logger().info(f"[{self.name}] Object-ID detection already running (pid={proc.pid}).")
            return

        cmd = ctx.get(
            "object_id_detection_cmd",
            ["ros2", "launch", "navi_wall", "yolo_object_detection.launch.py"],
        )
        if isinstance(cmd, str):
            cmd = shlex.split(cmd)
        node.get_logger().info(
            f"[{self.name}] Starting YOLO object detection for real robot: {' '.join(cmd)}"
        )
        start_proc(ctx, "object_id", cmd)

    def _start_optional_yolo_process(self, ctx):
        node = ctx["node"]
        cmd = ctx.get("object_id_yolo_cmd")
        if not cmd:
            return
        proc = ctx.get("_procs", {}).get("object_id_yolo")
        if proc and proc.poll() is None:
            node.get_logger().info(f"[{self.name}] YOLO process already running (pid={proc.pid}).")
            return
        if isinstance(cmd, str):
            cmd = shlex.split(cmd)
        node.get_logger().info(f"[{self.name}] Starting YOLO process: {' '.join(cmd)}")
        start_proc(ctx, "object_id_yolo", cmd)

    # ------------------------------------------------------------------
    # Legacy mock mode
    # ------------------------------------------------------------------
    def _start_mock_object_id(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Mock mode: calling /object_id_sim")
        self.client = node.create_client(SetBool, "/object_id_sim")
        request = SetBool.Request()
        request.data = True
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /object_id_sim not available.")
            ctx["error_triggered"] = True
            self.phase = "error"
            return
        self.future = self.client.call_async(request)
        self.phase = "mock_waiting"

    def _run_mock_object_id(self, ctx):
        node = ctx["node"]
        if self.future is None:
            return
        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Mock ObjectID completed correctly.")
                ctx["object_id_ready"] = True
                ctx["object_id_data"] = ctx.get("walls_data")
                self.phase = "done"
            else:
                node.get_logger().error(f"[{self.name}] Error while computing mock ObjectID.")
                ctx["error_triggered"] = True
                self.phase = "error"
            self.future = None
