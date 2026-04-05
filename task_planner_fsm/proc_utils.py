import atexit
import os
import signal
import subprocess
from typing import Dict, List


def start_proc(ctx: dict, key: str, cmd: List[str]) -> None:
    procs: Dict[str, subprocess.Popen] = ctx.setdefault("_procs", {})
    if key in procs and procs[key] and procs[key].poll() is None:
        return
    procs[key] = subprocess.Popen(
        cmd,
        start_new_session=True,
        stdout=None,
        stderr=None,
        close_fds=True,
    )


def stop_proc(ctx: dict, key: str, sig=signal.SIGINT, timeout=10.0, force_kill_patterns=None) -> None:
    procs = ctx.get("_procs", {})
    p = procs.get(key)
    if not p:
        return
    if p.poll() is None:
        node = ctx.get("node")
        try:
            if node:
                node.get_logger().info(
                    f"Stopping process '{key}' (pid={p.pid}) with {sig.name}, waiting up to {timeout}s..."
                )
            p.send_signal(sig)
            p.wait(timeout=timeout)
            if node:
                node.get_logger().info(f"Process '{key}' stopped cleanly")
        except subprocess.TimeoutExpired:
            if node:
                node.get_logger().warn(
                    f"Process '{key}' did not respond after {timeout}s, FORCE KILLING NOW..."
                )
            if force_kill_patterns:
                for pattern in force_kill_patterns:
                    try:
                        subprocess.run(
                            ["pkill", "-9", "-f", pattern], timeout=2, capture_output=True, text=True
                        )
                    except Exception:
                        pass
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
            try:
                p.kill()
            except Exception:
                pass
            try:
                p.wait(timeout=2)
            except Exception:
                pass
    procs.pop(key, None)


def stop_all(ctx: dict) -> None:
    node = ctx.get("node")
    if node:
        node.get_logger().info(f"Stopping all managed processes. Keys:{ctx.get('_procs', {}).keys()}")
    for k in list(ctx.get("_procs", {}).keys()):
        stop_proc(ctx, k, force_kill_patterns=["gz sim", "ign gazebo", "ruby.*gz", "gzserver", "gz-sim"])


def install_global_cleanup(ctx: dict):
    if ctx.get("_cleanup_installed"):
        return
    ctx["_cleanup_installed"] = True
    ctx["_cleanup_done"] = False

    def _cleanup():
        if ctx.get("_cleanup_done"):
            return
        stop_all(ctx)
        ctx["_cleanup_done"] = True

    atexit.register(_cleanup)

    for sig in (signal.SIGINT, signal.SIGTERM):
        prev = signal.getsignal(sig)

        def _chain_handler(signum, frame, _prev=prev):
            try:
                _cleanup()
            finally:
                if callable(_prev):
                    _prev(signum, frame)

        signal.signal(sig, _chain_handler)
