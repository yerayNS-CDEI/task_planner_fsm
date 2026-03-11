import os, signal, subprocess, atexit, signal as pysignal
from typing import Dict

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

def stop_proc(ctx: dict, key: str, sig=signal.SIGINT, timeout=10.0, force_kill_patterns=None) -> None:
    """
    Stop a process gracefully, with force-kill fallback for stubborn processes.
    
    Args:
        ctx: Context dictionary
        key: Process identifier
        sig: Initial signal (default: SIGINT)
        timeout: Timeout for graceful shutdown (default: 10s)
        force_kill_patterns: List of command patterns to force kill if graceful fails
    """
    procs = ctx.get("_procs", {})
    p = procs.get(key)
    if not p: return
    if p.poll() is None:
        node = ctx.get("node")
        try:
            if node:
                node.get_logger().info(f"Stopping process '{key}' (pid={p.pid}) with {sig.name}, waiting up to {timeout}s...")
            p.send_signal(sig)
            p.wait(timeout=timeout)
            if node:
                node.get_logger().info(f"Process '{key}' stopped cleanly")
        except subprocess.TimeoutExpired:
            # Graceful shutdown failed, use force kill
            if node:
                node.get_logger().warn(f"Process '{key}' did not respond after {timeout}s, FORCE KILLING NOW...")
            
            # If force_kill_patterns provided, use pkill -9 -f
            if force_kill_patterns:
                if node:
                    node.get_logger().warn(f"Executing pkill -9 -f for patterns: {force_kill_patterns}")
                for pattern in force_kill_patterns:
                    try:
                        result = subprocess.run(['pkill', '-9', '-f', pattern], timeout=2, 
                                              capture_output=True, text=True)
                        if node:
                            node.get_logger().info(f"pkill -9 -f '{pattern}' executed (rc={result.returncode})")
                    except Exception as e:
                        if node:
                            node.get_logger().error(f"pkill failed for '{pattern}': {e}")
            
            # Also try direct process group kill
            try:
                if node:
                    node.get_logger().warn(f"Sending SIGKILL to process group...")
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            except Exception as e:
                if node:
                    node.get_logger().error(f"Process group kill failed: {e}")
            
            # Final fallback: direct kill
            try:
                p.kill()
            except:
                pass
            
            # Don't wait indefinitely
            try:
                p.wait(timeout=2)
            except:
                pass
            
            if node:
                node.get_logger().warn(f"Force kill sequence completed for '{key}'")
                
        except ProcessLookupError:
            pass
    procs.pop(key, None)

def stop_all(ctx: dict) -> None:
    node = ctx.get("node")
    node.get_logger().info(f"Stopping all managed processes. Keys:{ctx.get('_procs', {}).keys()}")
    for k in list(ctx.get("_procs", {}).keys()):
        stop_proc(ctx, k, force_kill_patterns=['gz sim', 'ign gazebo', 'ruby.*gz', 'gzserver', 'gz-sim'])

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
