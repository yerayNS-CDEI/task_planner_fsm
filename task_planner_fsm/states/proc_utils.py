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

# Every process that a single move_robot.launch.py / mapping launch brings up.
# When we tear a launch down we must remove ALL of these, otherwise orphaned
# nodes (a second robot_state_publisher, controller_manager, rtabmap, rviz, ...)
# collide with the next launch and the new stack never comes up cleanly.
SIM_STACK_PATTERNS = GAZEBO_PATTERNS + [
    'move_robot.launch.py',
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
    for k in list(ctx.get("_procs", {}).keys()):
        stop_proc(ctx, k, force_kill_patterns=SIM_STACK_PATTERNS)

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
