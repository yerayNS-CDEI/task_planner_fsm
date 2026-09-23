"""Run a pipeline function off the FSM tick.

The delivered pipelines are single blocking calls -- Mask R-CNN over a B-scan
takes minutes on a CPU, and XGBoost over a few thousand spectra several seconds.
A state cannot afford either inside ``run()``: the machine would stop ticking,
the RViz panel would freeze, and nothing could interrupt it. So the call is
handed to a thread and ``run()`` polls :attr:`done` once per tick, exactly the
way the service-based phases poll their futures.

One job at a time per phase; the states never start a second while one runs.
"""

import threading
import time
import traceback


class BackgroundJob:
    """A function running in a daemon thread, polled from the FSM tick."""

    def __init__(self, fn, *args, name="sensor-job", **kwargs):
        self.name = name
        self.result = None
        self.error = None
        self.traceback = None
        self._started = time.monotonic()
        self._done = threading.Event()
        self._thread = threading.Thread(
            target=self._run, args=(fn, args, kwargs), name=name, daemon=True)
        self._thread.start()

    def _run(self, fn, args, kwargs):
        try:
            self.result = fn(*args, **kwargs)
        except BaseException as exc:          # noqa: BLE001 -- reported, not swallowed
            self.error = exc
            self.traceback = traceback.format_exc()
        finally:
            self._done.set()

    @property
    def done(self):
        return self._done.is_set()

    @property
    def failed(self):
        return self.done and self.error is not None

    @property
    def elapsed_s(self):
        return time.monotonic() - self._started

    def wait(self, timeout=None):
        """Block until finished (offline tools only; the FSM polls instead)."""
        self._done.wait(timeout)
        return self.done
