"""Why a state failed: the warnings and errors it logged before it gave up.

Most failure paths in the states set ``ctx["error_triggered"] = True`` right
after logging the cause with ``node.get_logger().error(...)``, without passing
that cause to ``State.fail``. The panel then only said "<State> failed". The
node's logger is a :class:`RecordingLogger` writing every WARN and above into
a :class:`StateLogBook`; the state machine marks the book when it enters a
state, and on a failure takes the state's last error as the cause and its
recent lines as context for the ``/fsm/event`` failure event.
"""

import os
import re
import threading
import time
from collections import deque
from typing import Dict, List, Optional

from rclpy.impl import rcutils_logger
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import LoggingSeverity

# rclpy names the code that logged a line (and keys throttle_duration_sec /
# once per call site) by walking up the stack past its own files. Listing this
# module there keeps it pointing at the state that called .error(), not at
# RecordingLogger.log. The list is meant to be extended (see _find_caller).
_THIS_FILE = os.path.realpath(__file__)
if _THIS_FILE not in rcutils_logger._internal_callers:
    rcutils_logger._internal_callers.append(_THIS_FILE)

_LEVEL_NAMES = {
    LoggingSeverity.WARN: "WARN",
    LoggingSeverity.ERROR: "ERROR",
    LoggingSeverity.FATAL: "FATAL",
}

# Longest text kept per line, and the longest cause put in a summary.
MAX_LINE_CHARS = 2000
MAX_CAUSE_CHARS = 300

# "[ScanWall] ", "[FSM] " ... at the start of a message: the state is already
# named in "<State> failed due to <cause>".
_TAG_PREFIX = re.compile(r"^(\s*\[[^\]\n]{1,40}\]\s*)+")


class StateLogBook:
    """The node's recent WARN/ERROR/FATAL lines, split at state entries."""

    def __init__(self, maxlen: int = 300):
        self._lock = threading.Lock()
        self._entries = deque(maxlen=maxlen)
        self._seq = 0
        self._mark = 0

    def record(self, level: str, text) -> None:
        text = str(text)
        if len(text) > MAX_LINE_CHARS:
            text = text[:MAX_LINE_CHARS] + " ..."
        with self._lock:
            self._seq += 1
            self._entries.append(
                {"seq": self._seq, "stamp": time.time(), "level": level, "text": text}
            )

    def mark(self) -> None:
        """Start a new state: later queries only see lines logged after this."""
        with self._lock:
            self._mark = self._seq

    def since_mark(self, limit: Optional[int] = None) -> List[Dict]:
        """Lines logged since the last mark, oldest first (the last ``limit``)."""
        with self._lock:
            lines = [dict(e) for e in self._entries if e["seq"] > self._mark]
        if limit is not None:
            lines = lines[-limit:] if limit > 0 else []
        for line in lines:
            line.pop("seq", None)
        return lines

    def last_error(self) -> Optional[str]:
        """The last ERROR/FATAL since the mark, as a one-line cause, or None."""
        for line in reversed(self.since_mark()):
            if line["level"] in ("ERROR", "FATAL"):
                return cause_from_log_line(line["text"])
        return None


def cause_from_log_line(text: str) -> Optional[str]:
    """Turn a logged error into the "<cause>" of "<State> failed due to <cause>".

    Keeps the first non-empty line, drops leading "[Tag]" prefixes and a
    trailing full stop, and caps the length.
    """
    for raw in str(text).splitlines():
        line = _TAG_PREFIX.sub("", raw).strip()
        if line:
            line = line.rstrip(".").rstrip()
            if len(line) > MAX_CAUSE_CHARS:
                line = line[: MAX_CAUSE_CHARS - 4].rstrip() + " ..."
            return line or None
    return None


class RecordingLogger(RcutilsLogger):
    """An rclpy logger that also writes WARN and above into a StateLogBook."""

    def __init__(self, name: str, logbook: StateLogBook):
        super().__init__(name)
        self.logbook = logbook

    def get_child(self, name):
        return RecordingLogger(super().get_child(name).name, self.logbook)

    def log(self, message, severity, **kwargs):
        logged = super().log(message, severity, **kwargs)
        # Only what was actually printed: a throttled or filtered-out message
        # never reached the console either.
        if logged:
            level = _LEVEL_NAMES.get(LoggingSeverity(severity))
            if level is not None:
                try:
                    self.logbook.record(level, message)
                except Exception:
                    pass
        return logged
