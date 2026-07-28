import os, signal, subprocess, atexit, signal as pysignal, time
from typing import Dict, Iterable, List, Optional

# Gazebo/Ignition server processes. Kept separate because they are the most
# stubborn to kill and are reused as a fallback in several places.
GAZEBO_PATTERNS = [
    'gz sim',
    'ign gazebo',
    'ruby.*gz',
    'gzserver',
    'gz-sim',
]

# Hardware sensor drivers brought up by the mapping / navigation stack. These
# are the most dangerous to leave orphaned: they hold exclusive OS resources
# (the SICK multiscan UDP receiver ports 2115/2116, the Ouster TCP/UDP session,
# the RealSense USB device). A survivor from a previous run makes the next
# launch's driver fail to bind ("Address already in use, error 98"), which
# silently breaks the point-cloud -> icp_odometry -> rtabmap localization chain
# (no /rtabmap/odom, no /map, empty costmap). SIGINT is routinely ignored by
# sick_generic_caller, so these MUST be reachable by the force-kill paths.
SENSOR_DRIVER_PATTERNS = [
    'sick_generic_caller',
    'os_driver',
    'pointcloud_concatenate_node',
    'robot_body_filter_node',
    'icp_odometry',
    'rgbd_sync',
    'realsense2_camera_node',
]

# Every process that a single move_robot.launch.py / mapping launch brings up.
# When we tear a launch down we must remove ALL of these, otherwise orphaned
# nodes (a second robot_state_publisher, controller_manager, rtabmap, rviz, ...)
# collide with the next launch and the new stack never comes up cleanly.
SIM_STACK_PATTERNS = GAZEBO_PATTERNS + SENSOR_DRIVER_PATTERNS + [
    'move_robot.launch.py',
    'mapping_stack.launch.py',
    'exploration.launch.py',
    'global_exploration.launch.py',
    'robot_state_publisher',
    'ros2_control_node',
    'controller_manager',
    'spawner',
    'rtabmap',
    'rgbd_odometry',
    'parameter_bridge',
    'ros_gz_bridge',
    'rviz2',
]

# Launch keys whose process tree contains ros2_control -- and therefore the
# column hardware interface, which retracts the column inside its on_deactivate.
ROBOT_STACK_KEYS = {"nav_sim"}

# Teardown budget for those launches. The column retraction keeps pumping the
# Modbus heartbeat for up to ~15 s while the column travels (see
# ColumnHardwareInterface::on_deactivate in navi-wall); with the usual 10 s grace
# our own escalation cuts the drive off mid-travel and the column is left
# extended, which then also breaks the next run.
ROBOT_STACK_STOP_TIMEOUT = 25.0

# ``ros2 launch`` runs its own SIGINT -> SIGTERM -> SIGKILL escalation on every
# node it spawned (5 s + 5 s by default), so a default-configured launch SIGKILLs
# ros2_control roughly 10 s into shutdown -- before the column has finished
# retracting, and a SIGKILLed hardware interface never gets to run on_deactivate
# at all. rclcpp treats SIGTERM as just another shutdown request, so only the
# SIGKILL deadline needs to move; pass this to every launch owning the column.
# Our own escalation above (SIGTERM at 25 s, SIGKILL at 30 s) still bounds the
# teardown, so this cannot hang the FSM.
ROBOT_STACK_LAUNCH_SHUTDOWN_ARGS = ["sigkill_timeout:=30"]


def stop_timeout_for(key: str) -> float:
    """Graceful-shutdown budget to give the launch registered under ``key``."""
    return ROBOT_STACK_STOP_TIMEOUT if key in ROBOT_STACK_KEYS else 10.0


def start_proc(ctx: dict, key: str, cmd: list[str]) -> None:
    procs: Dict[str, subprocess.Popen] = ctx.setdefault("_procs", {})
    if key in procs and procs[key] and procs[key].poll() is None:
        return
    procs[key] = subprocess.Popen(
        cmd,
        start_new_session=True,     # = setsid, mejor que preexec_fn en Py>=3.8
        stdout=None, stderr=None,   # deja logs visibles mientras depuras
        close_fds=True,
    )

def _force_kill_gazebo():
    """Force kill all Gazebo/Ignition processes using pkill."""
    gz_patterns = [
        'gz sim',
        'ign gazebo',
        'ruby.*gz',
        'gzserver',
        'gz-sim',
    ]
    
    for pattern in gz_patterns:
        try:
            subprocess.run(['pkill', '-9', '-f', pattern], timeout=2, stderr=subprocess.DEVNULL)
        except:
            pass

def _killpg(pid: int, sig) -> bool:
    """Signal the whole process group of `pid`. Returns False if it is gone."""
    try:
        os.killpg(os.getpgid(pid), sig)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def stop_proc(ctx: dict, key: str, sig=signal.SIGINT, timeout=10.0, force_kill_patterns=None) -> None:
    """
    Stop a launch process *and all of its children* gracefully, escalating to
    SIGKILL only if needed.

    Because launches are started with ``start_new_session=True`` the Popen child
    is the leader of its own process group, so signalling the group reaches every
    node the launch spawned (Gazebo, controllers, rtabmap, rviz, ...). This is the
    key difference from the previous version, which only signalled the launch
    parent and left its children orphaned.

    Args:
        ctx: Context dictionary
        key: Process identifier
        sig: Initial (graceful) signal (default: SIGINT)
        timeout: Timeout for graceful shutdown (default: 10s)
        force_kill_patterns: Extra command patterns to ``pkill -9 -f`` as a last
            resort if the group still has not exited.
    """
    procs = ctx.get("_procs", {})
    p = procs.get(key)
    if not p:
        return
    node = ctx.get("node")
    if p.poll() is None:
        # 1) Graceful: SIGINT to the whole group -> ros2 launch shuts its
        #    children down cleanly (DDS participants deregister, rtabmap closes
        #    its sqlite DB, controllers unload).
        try:
            if node:
                node.get_logger().info(
                    f"Stopping process group '{key}' (pid={p.pid}) with {sig.name}, waiting up to {timeout}s..."
                )
            if not _killpg(p.pid, sig):
                p.send_signal(sig)
            p.wait(timeout=timeout)
            if node:
                node.get_logger().info(f"Process '{key}' stopped cleanly")
        except subprocess.TimeoutExpired:
            # 2) Escalate to SIGTERM on the group.
            if node:
                node.get_logger().warn(
                    f"Process '{key}' did not respond to {sig.name} after {timeout}s, sending SIGTERM to group..."
                )
            _killpg(p.pid, signal.SIGTERM)
            try:
                p.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                # 3) Last resort: SIGKILL the group, then pattern-kill stragglers.
                if node:
                    node.get_logger().warn(f"Process '{key}' still alive, SIGKILLing group and stragglers...")
                _killpg(p.pid, signal.SIGKILL)
                if force_kill_patterns:
                    for pattern in force_kill_patterns:
                        try:
                            result = subprocess.run(['pkill', '-9', '-f', pattern], timeout=2,
                                                    capture_output=True, text=True)
                            if node:
                                node.get_logger().info(f"pkill -9 -f '{pattern}' executed (rc={result.returncode})")
                        except Exception as e:
                            if node:
                                node.get_logger().error(f"pkill failed for '{pattern}': {e}")
                try:
                    p.kill()
                except Exception:
                    pass
                try:
                    p.wait(timeout=2)
                except Exception:
                    pass
                if node:
                    node.get_logger().warn(f"Force kill sequence completed for '{key}'")
        except ProcessLookupError:
            pass
    procs.pop(key, None)


def _pgrep(pattern: str) -> List[int]:
    try:
        r = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True, timeout=3)
        return [int(x) for x in r.stdout.split()]
    except Exception:
        return []


def graceful_rtabmap_save(
    node=None,
    timeout: float = 180.0,
    poll_interval: float = 1.0,
    pattern: str = 'rtabmap_slam',
) -> bool:
    """SIGINT the rtabmap SLAM node alone and block until it has exited.

    rtabmap only persists its map database (the visual-word dictionary + the
    optimized graph) when its process receives **SIGINT** and shuts down
    cleanly -- SIGTERM/SIGKILL skip the save entirely, leaving a database with
    nodes/features but an empty Word table (``VWDictionary ... dict size=0`` on
    reload), or a truncated/unopenable file. On a large (>1 GB) map this save
    can take well over a minute on the Jetson, far longer than the ~10 s
    escalation window used when we tear the whole launch down.

    So before stopping the launch we signal *only* the SLAM node (matched by
    ``rtabmap_slam`` in its command line -- the odometry/sync/viz nodes live in
    ``rtabmap_odom``/``rtabmap_sync``/``rtabmap_viz`` and are left for the normal
    teardown) and wait for the process to disappear, which is our proof the
    database has been flushed to disk. Signalling just this pid means neither
    ``stop_proc``'s escalation nor ``ros2 launch``'s own SIGKILL timer can kill
    rtabmap before the save completes.

    Returns True if rtabmap exited within ``timeout`` (or was not running),
    False if it was still alive when the timeout elapsed.
    """
    pids = _pgrep(pattern)
    if not pids:
        if node:
            node.get_logger().info(
                "[shutdown] No rtabmap SLAM node found; nothing to save."
            )
        return True

    if node:
        node.get_logger().info(
            f"[shutdown] Sending SIGINT to rtabmap SLAM (pids={pids}) and waiting "
            f"up to {timeout:.0f}s for the database save to finish..."
        )
    for pid in pids:
        try:
            os.kill(pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass

    deadline = time.time() + timeout
    while time.time() < deadline:
        if not _pgrep(pattern):
            if node:
                node.get_logger().info(
                    "[shutdown] rtabmap SLAM exited cleanly; database saved."
                )
            return True
        time.sleep(poll_interval)

    if node:
        node.get_logger().warn(
            f"[shutdown] rtabmap SLAM still alive after {timeout:.0f}s; proceeding "
            f"with teardown -- the database may be incomplete."
        )
    return False


def wait_processes_gone(
    patterns: Iterable[str],
    timeout: float = 25.0,
    node=None,
    exclude_pids: Optional[Iterable[int]] = None,
    poll_interval: float = 0.5,
) -> bool:
    """Block until no process matching any of `patterns` remains.

    Used after a teardown to guarantee the previous launch is fully gone before
    the next one starts, so the new stack does not collide with orphaned nodes or
    a still-locked rtabmap database. Returns True if everything exited within
    `timeout`, False otherwise.
    """
    exclude = set(exclude_pids or [])
    exclude.add(os.getpid())
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = sorted({
            pid
            for pattern in patterns
            for pid in _pgrep(pattern)
            if pid not in exclude
        })
        if not remaining:
            return True
        if node:
            node.get_logger().info(f"Waiting for {len(remaining)} sim-stack process(es) to exit: {remaining}")
        time.sleep(poll_interval)
    if node:
        node.get_logger().warn(f"Timed out after {timeout}s waiting for processes to exit.")
    return False


def kill_stale_stack(
    node=None,
    patterns: Iterable[str] = SENSOR_DRIVER_PATTERNS,
    exclude_pids: Optional[Iterable[int]] = None,
    settle: float = 1.0,
) -> List[int]:
    """Force-kill lingering stack/sensor processes BEFORE launching a new stack.

    Orphans from a previous run (crashed, or one whose ``ros2 launch`` was
    Ctrl-C'd while ``sick_generic_caller`` ignored the SIGINT) keep holding the
    SICK UDP ports 2115/2116 and the RealSense USB device. The next launch's
    drivers then fail to bind ("Address already in use") and odometry /
    localization silently never come up. Calling this at the start of the launch
    guarantees a clean slate. It is a no-op when nothing matches.

    Defaults to ``SENSOR_DRIVER_PATTERNS`` (the resource-holding drivers); pass
    ``SIM_STACK_PATTERNS`` for a full-stack sweep. Returns the PIDs it killed.
    """
    exclude = set(exclude_pids or [])
    exclude.add(os.getpid())
    killed: List[int] = []
    for pattern in patterns:
        for pid in _pgrep(pattern):
            if pid in exclude:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
                killed.append(pid)
                if node:
                    node.get_logger().warn(
                        f"[cleanup] Force-killed stale process pid={pid} matching '{pattern}'."
                    )
            except (ProcessLookupError, PermissionError):
                pass
    if killed and settle > 0:
        # Give the kernel a moment to release the bound UDP sockets / USB handle
        # before the new drivers try to claim them.
        time.sleep(settle)
    return killed


def _topic_publisher_count(topic: str, timeout: float = 4.0) -> int:
    """Number of publishers on `topic`, via the ros2 CLI (no node spin required)."""
    try:
        r = subprocess.run(['ros2', 'topic', 'info', topic],
                           capture_output=True, text=True, timeout=timeout)
        for line in r.stdout.splitlines():
            if 'Publisher count' in line:
                try:
                    return int(line.split(':', 1)[1].strip())
                except ValueError:
                    return 0
    except Exception:
        pass
    return 0


def wait_stack_ready(
    ctx: dict,
    required_topics: Iterable[str],
    timeout: float = 90.0,
    poll_interval: float = 2.0,
) -> bool:
    """Block until every topic in `required_topics` has at least one publisher.

    This replaces blind ``time.sleep()`` calls after launching the navigation /
    localization stack: it confirms the stack is actually producing data (clock,
    TF, joint states, rtabmap odom, ...) before the FSM proceeds. Returns True if
    all topics had a publisher within `timeout`, False otherwise.

    The check uses the ``ros2 topic info`` CLI so it works even while the FSM
    node's own executor is blocked inside this call.
    """
    node = ctx.get("node")
    pending = list(required_topics)
    deadline = time.time() + timeout
    while time.time() < deadline:
        still_pending = []
        for topic in pending:
            if _topic_publisher_count(topic) > 0:
                if node:
                    node.get_logger().info(f"[stack-ready] '{topic}' has a publisher.")
            else:
                still_pending.append(topic)
        pending = still_pending
        if not pending:
            return True
        if node:
            node.get_logger().info(f"[stack-ready] Still waiting for: {pending}")
        time.sleep(poll_interval)
    if node:
        node.get_logger().error(f"[stack-ready] Timed out after {timeout}s; no publisher on: {pending}")
    return False

def _list_services(timeout: float = 4.0) -> set:
    """Set of service names currently visible in the ROS graph, via the ros2 CLI."""
    try:
        r = subprocess.run(['ros2', 'service', 'list'],
                           capture_output=True, text=True, timeout=timeout)
        return {line.strip() for line in r.stdout.splitlines() if line.strip()}
    except Exception:
        return set()


def wait_services_ready(
    ctx: dict,
    required_services: Iterable[str],
    timeout: float = 60.0,
    poll_interval: float = 2.0,
) -> bool:
    """Block until every service in `required_services` is present in the graph.

    Complements ``wait_stack_ready`` for parts of the stack that expose no topic
    to gate on — notably the collision-checking service, which loses a startup
    race under load often enough that the FSM must wait for it explicitly instead
    of proceeding while the planner silently runs without collision validation.

    Uses the ``ros2 service list`` CLI so it works even while the FSM node's own
    executor is blocked inside this call. Returns True if all services appeared
    within `timeout`, False otherwise.
    """
    node = ctx.get("node")
    pending = list(required_services)
    deadline = time.time() + timeout
    while time.time() < deadline:
        available = _list_services()
        still_pending = []
        for svc in pending:
            if svc in available:
                if node:
                    node.get_logger().info(f"[stack-ready] service '{svc}' is available.")
            else:
                still_pending.append(svc)
        pending = still_pending
        if not pending:
            return True
        if node:
            node.get_logger().info(f"[stack-ready] Still waiting for service(s): {pending}")
        time.sleep(poll_interval)
    if node:
        node.get_logger().error(f"[stack-ready] Timed out after {timeout}s; missing service(s): {pending}")
    return False


def stop_all(ctx: dict) -> None:
    node = ctx.get("node")
    node.get_logger().info(f"Stopping all managed processes. Keys:{ctx.get('_procs', {}).keys()}")
    # Save rtabmap's database BEFORE force-killing anything. This path runs on
    # Ctrl+C / atexit (the FSM equivalent of pressing the UI's "Stop Mapping"
    # button), and the stop_proc loop below would otherwise SIGKILL / pkill the
    # SLAM node mid-save on a large map, leaving a database with 0 words
    # ("VWDictionary dict size=0"). SIGINT the SLAM node alone and wait for it to
    # flush first; it is a no-op when no rtabmap node is running.
    save_timeout = float(ctx.get("create_map_rtabmap_save_timeout", 180.0))
    graceful_rtabmap_save(node=node, timeout=save_timeout)
    for k in list(ctx.get("_procs", {}).keys()):
        # Launches that own ros2_control get a longer grace so the column
        # hardware interface can finish retracting the column before we escalate.
        stop_proc(ctx, k, timeout=stop_timeout_for(k), force_kill_patterns=SIM_STACK_PATTERNS)

def install_global_cleanup(ctx: dict):
    # Solo instalar una vez
    if ctx.get("_cleanup_installed"):
        return
    ctx["_cleanup_installed"] = True
    ctx["_cleanup_done"] = False

    def _cleanup():
        # Garantiza limpiar una sola vez
        if ctx.get("_cleanup_done"):
            return
        stop_all(ctx)
        ctx["_cleanup_done"] = True

    # atexit siempre ayuda si el proceso sale "bien"
    atexit.register(_cleanup)

    # Encadena con el handler previo (muy importante para que SIGINT siga
    # levantando KeyboardInterrupt y rclpy pare el spin)
    for sig in (pysignal.SIGINT, pysignal.SIGTERM):
        prev = pysignal.getsignal(sig)
        def _chain_handler(signum, frame, _prev=prev):
            try:
                _cleanup()
            finally:
                if callable(_prev):
                    _prev(signum, frame)  # deja que el handler original haga lo suyo
        pysignal.signal(sig, _chain_handler)
