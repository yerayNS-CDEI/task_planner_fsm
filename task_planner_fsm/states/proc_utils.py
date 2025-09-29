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

def stop_proc(ctx: dict, key: str, sig=signal.SIGINT, timeout=5.0) -> None:
    procs = ctx.get("_procs", {})
    p = procs.get(key)
    if not p: return
    if p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), sig)
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    procs.pop(key, None)

def stop_all(ctx: dict) -> None:
    for k in list(ctx.get("_procs", {}).keys()):
        stop_proc(ctx, k)

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
