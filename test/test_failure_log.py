"""Why a state failed, carried to /fsm/event and the panel.

Most failure paths only log their cause and set ``error_triggered``. The node's
RecordingLogger keeps the WARN/ERROR lines in a StateLogBook; the state machine
takes the failed state's last error as the cause and its recent lines as the
context of the retry/error event.

Run with:

    python3 -m pytest test/test_failure_log.py -v
"""

import pytest
from rclpy.impl import rcutils_logger

from task_planner_fsm.failure_log import (
    MAX_CAUSE_CHARS,
    RecordingLogger,
    StateLogBook,
    cause_from_log_line,
)
from task_planner_fsm.machine import StateMachine
from task_planner_fsm.state import State


@pytest.fixture
def emitted(monkeypatch):
    """What rclpy hands to rcutils: (severity, name, message, function, file, line)."""
    calls = []

    def fake_log(severity, name, message, function_name, file_path, line_number):
        calls.append((severity, name, message, function_name, file_path, line_number))

    monkeypatch.setattr(rcutils_logger._rclpy, "rclpy_logging_rcutils_log", fake_log)
    return calls


class _Node:
    def __init__(self, logbook):
        self.logger = RecordingLogger("test_fsm_failure_log", logbook)

    def get_logger(self):
        return self.logger


class _Err(State):
    pass


class _LogsThenFails(State):
    """Logs its cause the way most states do, then sets the bare flag."""

    def __init__(self, name, messages):
        super().__init__(name)
        self.messages = list(messages)
        self.entries = 0

    def on_enter(self, ctx):
        self.entries += 1

    def run(self, ctx):
        log = ctx["node"].get_logger()
        log.warn(f"[{self.name}] slow start (attempt {self.entries})")
        log.error(self.messages[min(self.entries, len(self.messages)) - 1])
        ctx["error_triggered"] = True

    def check_transition(self, ctx):
        return "Error" if ctx.get("error_triggered") else None


def machine(states, initial):
    logbook = StateLogBook()
    events = []
    ctx = {
        "node": _Node(logbook),
        "_cleanup_installed": True,
        "fsm_logbook": logbook,
        "publish_fsm_event": lambda ev, **kw: events.append((ev, kw)),
    }
    return StateMachine(states, initial_state=initial, ctx=ctx), ctx, events


def failures(events):
    return [(ev, kw) for ev, kw in events if ev in ("retry", "error")]


# ---------------------------------------------------------------------------
# RecordingLogger
# ---------------------------------------------------------------------------

def test_logger_records_warn_and_above_only(emitted):
    book = StateLogBook()
    log = RecordingLogger("test_fsm_levels", book)
    log.info("fine")
    log.warn("careful")
    log.error("broken")
    log.fatal("dead")
    assert [(e["level"], e["text"]) for e in book.since_mark()] == [
        ("WARN", "careful"), ("ERROR", "broken"), ("FATAL", "dead"),
    ]
    assert len(emitted) == 4  # every line still reaches the console


def test_logger_reports_the_real_call_site(emitted):
    log = RecordingLogger("test_fsm_site", StateLogBook())
    log.error("here")
    _, _, _, function_name, file_path, _ = emitted[-1]
    assert function_name == "test_logger_reports_the_real_call_site"
    assert file_path.endswith("test_failure_log.py")


def test_throttle_is_still_per_call_site(emitted):
    book = StateLogBook()
    log = RecordingLogger("test_fsm_throttle", book)
    for _ in range(3):
        log.error("site A", throttle_duration_sec=60.0)
        log.error("site B", throttle_duration_sec=60.0)
    # Each site printed once; had the wrapper hidden the caller, both would
    # share one throttle and site B would never print.
    assert [c[2] for c in emitted] == ["site A", "site B"]
    assert [e["text"] for e in book.since_mark()] == ["site A", "site B"]


def test_child_logger_records_into_the_same_book(emitted):
    book = StateLogBook()
    child = RecordingLogger("test_fsm_parent", book).get_child("child")
    assert child.name == "test_fsm_parent.child"
    child.error("from child")
    assert book.since_mark()[-1]["text"] == "from child"


# ---------------------------------------------------------------------------
# StateLogBook
# ---------------------------------------------------------------------------

def test_mark_hides_older_lines():
    book = StateLogBook()
    book.record("ERROR", "previous state")
    book.mark()
    assert book.since_mark() == []
    assert book.last_error() is None
    book.record("WARN", "a warning")
    assert book.last_error() is None
    book.record("ERROR", "[ScanWall] Column did not reach 1.20 m.")
    assert book.last_error() == "Column did not reach 1.20 m"
    assert [e["text"] for e in book.since_mark(limit=1)] == ["[ScanWall] Column did not reach 1.20 m."]


@pytest.mark.parametrize("text, cause", [
    ("[CreateMap] Nav2 did not become active.", "Nav2 did not become active"),
    ("[FSM] [ScanWall] two tags", "two tags"),
    ("\n\n[ObjectID] first line\nsecond line", "first line"),
    ("no tag at all", "no tag at all"),
    ("[Tag]   ", None),
])
def test_cause_from_log_line(text, cause):
    assert cause_from_log_line(text) == cause


def test_cause_is_capped():
    cause = cause_from_log_line("x" * 1000)
    assert len(cause) <= MAX_CAUSE_CHARS
    assert cause.endswith("...")


# ---------------------------------------------------------------------------
# StateMachine: the cause and context of a failure
# ---------------------------------------------------------------------------

def test_logged_error_becomes_the_cause_and_each_attempt_has_its_own(emitted):
    state = _LogsThenFails("CreateMap", [
        "[CreateMap] Nav2 did not become active.",
        "[CreateMap] Mapping stack exited early (code -9).",
    ])
    m, ctx, events = machine([state, _Err("Error")], "CreateMap")

    m.step()  # first attempt fails -> retry
    m.step()  # second attempt fails -> Error

    (ev1, retry), (ev2, error) = failures(events)
    assert ev1 == "retry" and ev2 == "error"
    assert retry["summary"] == "CreateMap failed due to Nav2 did not become active; retrying once"
    assert retry["details"]["cause_source"] == "log"
    assert retry["details"]["attempt"] == 1
    assert error["details"]["cause"] == "Mapping stack exited early (code -9)"
    assert error["details"]["attempt"] == 2
    assert m.current_state.name == "Error"
    assert ctx["fsm_error_summary"].startswith("CreateMap failed due to Mapping stack exited early")

    # The retry's context is its own attempt: not the first attempt's lines,
    # nor the machine's own "Retrying this state once" warning.
    texts = [line["text"] for line in error["details"]["recent_logs"]]
    assert texts == [
        "[CreateMap] slow start (attempt 2)",
        "[CreateMap] Mapping stack exited early (code -9).",
    ]
    assert all({"stamp", "level", "text"} <= set(line) for line in error["details"]["recent_logs"])


def test_explicit_fail_reason_wins_over_the_log(emitted):
    class Fails(_LogsThenFails):
        def run(self, ctx):
            ctx["node"].get_logger().error("[ScanWall] some detail")
            self.fail(ctx, "column height inputs never arrived")

    m, _, events = machine([Fails("ScanWall", []), _Err("Error")], "ScanWall")
    m.step()
    (_, retry), = failures(events)
    assert retry["details"]["cause"] == "column height inputs never arrived"
    assert retry["details"]["cause_source"] == "fail"
    assert [l["text"] for l in retry["details"]["recent_logs"]] == ["[ScanWall] some detail"]


def test_exception_names_itself_and_carries_the_traceback(emitted):
    class Crashes(State):
        def run(self, ctx):
            raise KeyError("walls_data")

    m, _, events = machine([Crashes("GeometryReconstruction"), _Err("Error")], "GeometryReconstruction")
    m.step()
    (_, retry), = failures(events)
    assert retry["details"]["cause"] == "KeyError: 'walls_data'"
    assert retry["details"]["cause_source"] == "exception"
    assert "Traceback" in retry["details"]["traceback"]
    assert "raise KeyError" in retry["details"]["traceback"]


def test_a_silent_failure_says_so(emitted):
    class Silent(State):
        def run(self, ctx):
            ctx["error_triggered"] = True

        def check_transition(self, ctx):
            return "Error" if ctx.get("error_triggered") else None

    m, _, events = machine([Silent("ArmFolding"), _Err("Error")], "ArmFolding")
    m.step()
    (_, retry), = failures(events)
    assert retry["summary"] == "ArmFolding failed; retrying once"
    assert retry["details"]["cause"] == ""
    assert retry["details"]["cause_source"] == ""


def test_machine_without_a_logbook_still_works():
    """Older callers (and other tests) build ctx without one."""
    class Fails(State):
        def run(self, ctx):
            self.fail(ctx, "nope")

        def check_transition(self, ctx):
            return "Error"

    events = []

    class _PlainLogger:
        def __getattr__(self, name):
            return lambda *a, **k: None

    class _PlainNode:
        def get_logger(self):
            return _PlainLogger()

    ctx = {"node": _PlainNode(), "_cleanup_installed": True,
           "publish_fsm_event": lambda ev, **kw: events.append((ev, kw))}
    m = StateMachine([Fails("ScanWall"), _Err("Error")], initial_state="ScanWall", ctx=ctx)
    m.step()
    (_, retry), = failures(events)
    assert retry["details"]["cause"] == "nope"
    assert retry["details"]["recent_logs"] == []
