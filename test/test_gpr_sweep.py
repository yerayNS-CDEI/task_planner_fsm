"""The GPR under the whole-body sweep: the GP API flow GprSweep drives, the
plate-travel trigger grid, the unseated stretches it records, ScanWall arming
it on the sweep node's first "running: seated", and the node reporting that.

No rclpy graph: the node, TF buffer and HTTP are fakes.

Run with:

    python3 -m pytest test/test_gpr_sweep.py -v
"""

import io
import json
import zipfile
from types import SimpleNamespace

import pytest

from task_planner_fsm.utils import gpr_sweep as gpr_sweep_mod
from task_planner_fsm.utils.gpr_sweep import GprSweep


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(msg)

    warn = error = debug = info


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


class _Timer:
    def cancel(self):
        pass


class _Node:
    def __init__(self):
        self.logger = _Logger()
        self.pub = _Pub()
        self.timers = []

    def get_logger(self):
        return self.logger

    def create_publisher(self, *a, **kw):
        return self.pub

    def create_timer(self, period, cb):
        self.timers.append(cb)
        return _Timer()

    def destroy_timer(self, timer):
        pass


class _Tf:
    """Plate at ``self.xyz`` in every frame."""

    def __init__(self):
        self.xyz = [0.0, 0.0, 1.0]

    def can_transform(self, *a, **kw):
        return True

    def lookup_transform(self, *a, **kw):
        t = SimpleNamespace(x=self.xyz[0], y=self.xyz[1], z=self.xyz[2])
        return SimpleNamespace(transform=SimpleNamespace(translation=t))


class _Resp:
    def __init__(self, status=200, body=None, content=None):
        self.status_code = status
        if content is not None:
            self.content = content
            self.headers = {"Content-Type": "application/zip"}
        else:
            self.content = b"" if body is None else json.dumps(body).encode()
            self.headers = {"Content-Type": "application/json"}

    @property
    def text(self):
        return self.content.decode(errors="replace")

    def json(self):
        return json.loads(self.content)


def _export_zip():
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr("m_20260923/m.csv", "x")
        zf.writestr("m_20260923/m.sgy", "traces")
    return buf.getvalue()


class _App:
    """The GP App's measurement/line state, as far as the GP API shows it.

    ``start_stamps`` feeds line/start's clientStartTimestamp, one per call
    (the last one repeats); 0 is a start the probe never acknowledged -- the
    app still marks the line started, as it did on 09-30."""

    def __init__(self, start_stamps=(1790764725139,), line_finishes=True):
        self.calls = []
        self.timeouts = {}
        self.measurement = False
        self.started = False
        self.finished = False
        self.start_stamps = list(start_stamps)
        self.line_finishes = line_finishes

    def __call__(self, method, url, json=None, timeout=None):
        self.calls.append((method, url, json))
        path = url.split(":9000", 1)[-1]
        self.timeouts[path] = timeout
        if path == "/measurement/line" and method == "GET":
            return _Resp(200, {"started": self.started, "finished": self.finished,
                               "scans": 200, "length": 1.0,
                               "index": 1 if self.measurement else 0})
        if path == "/measurement/start":
            self.measurement, self.started, self.finished = True, False, False
            return _Resp(200, {"data": {"name": json["name"], "type": json["type"]}})
        if path == "/measurement/line/start":
            if not self.measurement or self.started:
                return _Resp(400, {"error": {"message": "cannot start a line"}})
            stamp = self.start_stamps.pop(0) if len(self.start_stamps) > 1 else self.start_stamps[0]
            self.started = True
            return _Resp(200, {"clientStartTimestamp": stamp})
        if path == "/measurement/line/stop":
            self.finished = self.line_finishes
            return _Resp(204)
        if path == "/measurement/export/raw":
            return _Resp(200, content=_export_zip())
        if path == "/measurement/stop":
            self.measurement = self.started = self.finished = False
            return _Resp(204)
        return _Resp(200)

    def paths(self):
        return [(m, u.split(":9000", 1)[1]) for m, u, _ in self.calls]


@pytest.fixture
def app(monkeypatch):
    fake = _App()
    monkeypatch.setattr(gpr_sweep_mod.requests, "request", fake)
    return fake


@pytest.fixture
def http(app):
    """Every GP API call, against a healthy app."""
    return app.calls


def make_ctx(tmp_path, **over):
    ctx = {"node": _Node(), "tf_buffer": _Tf(), "gpr_enabled": True,
           "sensor_data_dir": str(tmp_path), "sensor_session_id": "test",
           "gpr_export_delay_s": 0.0, "gpr_line_start_delay_s": 0.0,
           "gpr_line_start_confirm_s": 0.0, "gpr_line_finish_timeout_s": 0.0}
    ctx.update(over)
    return ctx


SEG = ((0.0, 0.0, 1.0), (2.0, 0.0, 1.0))


def _begin(gpr, ctx, wall_index, line_idx, seg_idx):
    """What ScanWall does per segment: the measurement at sweep_setup, the line
    at press_settle. The first error, or None."""
    return (gpr.open_measurement(ctx, wall_index, line_idx, seg_idx)
            or gpr.start_line(ctx, *SEG, "map"))


def _manifest(tmp_path):
    path = tmp_path / "raw" / "gpr" / "session_test" / "gpr_lines.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines()]


# ----------------------------------------------------------------------
# GP API flow
# ----------------------------------------------------------------------
def test_segment_runs_the_gp_api_flow_in_order_against_the_static_probe(tmp_path, http):
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    assert _begin(gpr, ctx, 2, 1, 0) is None
    assert gpr.line_active and gpr.measurement_active
    gpr.end_segment(ctx)

    paths = [(m, u.split(":9000", 1)[1]) for m, u, _ in http]
    assert paths == [
        ("POST", "/probe/connect"),
        ("GET", "/measurement/line"),          # anything left open? no
        ("POST", "/measurement/start"),
        ("GET", "/measurement/line"),          # fresh: no line yet
        ("POST", "/measurement/line/start"),
        ("GET", "/measurement/line"),          # running
        ("POST", "/measurement/line/stop"),
        ("GET", "/measurement/line"),          # finished
        ("POST", "/measurement/export/raw"),
        ("POST", "/measurement/stop"),
    ]
    assert all(u.startswith("http://192.168.1.239:9000/") for _, u, _ in http)
    assert http[0][2] == {"serialNumber": "GP88-007-0081", "ip": "192.168.1.99"}
    assert http[2][2] == {"type": "LINE_SCAN", "name": "scan_wall line 2 seg 1"}

    (row,) = _manifest(tmp_path)
    assert row["key"] == "w02_l01_s00" and row["sweep"] == "wbc"
    assert row["probe_active"] and row["export"]["ok"]
    assert row["probe_line"]["scans"] == 200
    assert row["client_start_timestamp_ms"] == 1790764725139
    assert row["line_start_attempts"] == 1
    incoming = tmp_path / "raw" / "gpr" / "session_test" / "incoming"
    assert (incoming / "w02_l01_s00_m.sgy").is_file()


def test_a_failed_connect_refuses_the_segment_and_starts_nothing(tmp_path, monkeypatch):
    monkeypatch.setattr(gpr_sweep_mod.requests, "request",
                        lambda *a, **kw: _Resp(500, {"error": {"message": "no"}}))
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    assert _begin(gpr, ctx, 0, 0, 0) == "GPR probe connection failed"
    assert not gpr.line_active and not gpr.measurement_active


def test_an_unacknowledged_line_start_is_retried_from_a_clean_measurement(tmp_path, monkeypatch):
    """09-30: line/start answered 200 with clientStartTimestamp 0. The line is
    closed (it did start on the app) without an export, the measurement
    stopped, and the whole start sequence runs again."""
    app = _App(start_stamps=(0, 1790764975634))
    monkeypatch.setattr(gpr_sweep_mod.requests, "request", app)
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    assert _begin(gpr, ctx, 0, 0, 0) is None
    posts = [p for m, p in app.paths() if m == "POST"]
    assert posts == [
        "/probe/connect", "/measurement/start", "/measurement/line/start",
        "/measurement/line/stop", "/measurement/stop",            # attempt 1, no export
        "/probe/connect", "/measurement/start", "/measurement/line/start",
    ]
    assert gpr.line_active and gpr.measurement_active
    gpr.end_segment(ctx)
    (row,) = _manifest(tmp_path)
    assert row["line_start_attempts"] == 2 and row["export"]["ok"]


def test_a_line_that_never_acknowledges_aborts_and_leaves_nothing_open(tmp_path, monkeypatch):
    app = _App(start_stamps=(0,))
    monkeypatch.setattr(gpr_sweep_mod.requests, "request", app)
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    err = _begin(gpr, ctx, 0, 0, 0)
    assert err and "clientStartTimestamp" in err
    assert not gpr.line_active and not gpr.measurement_active
    assert not app.measurement
    assert ("POST", "/measurement/export/raw") not in app.paths()


class _RefusingApp(_App):
    """An app that answers line/start with an error."""

    def __call__(self, method, url, json=None, timeout=None):
        if url.endswith("/measurement/line/start"):
            self.calls.append((method, url, json))
            return _Resp(400, {"error": {"message": "no"}})
        return super().__call__(method, url, json, timeout)


def test_a_rejected_line_start_is_never_stopped(tmp_path, monkeypatch):
    """Stopping a line that did not start makes no sense: only the measurement
    is closed."""
    app = _RefusingApp()
    monkeypatch.setattr(gpr_sweep_mod.requests, "request", app)
    ctx = make_ctx(tmp_path, gpr_line_start_attempts=1)
    gpr = GprSweep("ScanWall", None)
    assert _begin(gpr, ctx, 0, 0, 0) == "GPR failed to start the scan line"
    posts = [p for m, p in app.paths() if m == "POST"]
    assert "/measurement/line/stop" not in posts
    assert posts[-1] == "/measurement/stop"


def test_a_measurement_left_open_by_an_earlier_attempt_is_closed_first(tmp_path, app):
    """A state retry after the app went unreachable (or an FSM restart) leaves
    a running line on the app that this object knows nothing about."""
    app.measurement, app.started, app.finished = True, True, False
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    assert _begin(gpr, ctx, 0, 0, 0) is None
    posts = [p for m, p in app.paths() if m == "POST"]
    assert posts[:4] == ["/probe/connect", "/measurement/line/stop",
                         "/measurement/stop", "/measurement/start"]


def test_a_line_that_never_finishes_is_not_exported_but_the_measurement_closes(
        tmp_path, monkeypatch):
    app = _App(line_finishes=False)
    monkeypatch.setattr(gpr_sweep_mod.requests, "request", app)
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    assert _begin(gpr, ctx, 0, 0, 0) is None
    gpr.end_segment(ctx)
    posts = [p for m, p in app.paths() if m == "POST"]
    assert "/measurement/export/raw" not in posts
    assert posts[-1] == "/measurement/stop"
    (row,) = _manifest(tmp_path)
    assert row["export"]["skipped"] == "line not finished"


def test_the_export_gets_its_own_long_timeout(tmp_path, app):
    """A timed-out export followed by measurement/stop would stop the
    measurement under an export the app is still writing."""
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    _begin(gpr, ctx, 0, 0, 0)
    gpr.end_segment(ctx)
    assert app.timeouts["/measurement/export/raw"] == GprSweep.EXPORT_TIMEOUT_S
    assert app.timeouts["/measurement/start"] == GprSweep.TIMEOUT_S


def test_a_line_is_never_started_without_an_open_measurement(tmp_path, app):
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    gpr._segment = (0, 0, 0)
    assert gpr.start_line(ctx, *SEG, "map") == \
        "GPR line start requested without an open measurement"
    assert app.calls == []


def test_the_line_waits_out_only_what_is_left_of_the_gap(tmp_path, app, monkeypatch):
    """The arm approach normally spans the gap; a line started sooner waits the
    remainder, and one started later waits nothing."""
    slept = []
    monkeypatch.setattr(gpr_sweep_mod.time, "sleep", slept.append)
    ctx = make_ctx(tmp_path, gpr_line_start_delay_s=2.0)
    gpr = GprSweep("ScanWall", None)
    assert gpr.open_measurement(ctx, 0, 0, 0) is None
    gpr._measurement_t0 -= 0.5                   # 0.5 s of approach
    assert gpr.start_line(ctx, *SEG, "map") is None
    assert len(slept) == 1 and 1.4 < slept[0] <= 1.5
    gpr.end_segment(ctx)

    slept.clear()
    assert gpr.open_measurement(ctx, 0, 0, 1) is None
    gpr._measurement_t0 -= 10.0                  # a long approach
    assert gpr.start_line(ctx, *SEG, "map") is None
    assert slept == []


def test_a_segment_skipped_before_its_line_only_closes_the_measurement(tmp_path, app):
    """press_prepare can skip the segment after sweep_setup opened the
    measurement: nothing to stop, export or record -- just close it."""
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    assert gpr.open_measurement(ctx, 0, 0, 0) is None
    app.calls.clear()
    gpr.abort(ctx)
    assert app.paths() == [("POST", "/measurement/stop")]
    assert not gpr.measurement_active and not app.measurement
    assert not (tmp_path / "raw" / "gpr" / "session_test" / "gpr_lines.jsonl").exists()


def test_probe_off_in_sim_still_writes_the_line_record(tmp_path, http):
    ctx = make_ctx(tmp_path, sim=True)
    del ctx["gpr_enabled"]
    gpr = GprSweep("ScanWall", None)
    assert _begin(gpr, ctx, 0, 0, 0) is None
    gpr.end_segment(ctx)
    assert http == []
    (row,) = _manifest(tmp_path)
    assert not row["probe_active"]


# ----------------------------------------------------------------------
# Triggers
# ----------------------------------------------------------------------
def _armed(tmp_path, http):
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    _begin(gpr, ctx, 0, 0, 0)
    ctx["tf_buffer"].xyz = [0.30, 0.0, 1.0]      # pre-roll travel: not counted
    gpr.arm_triggers(ctx, "map", *SEG, 0.045)
    return ctx, gpr


def _move_to(ctx, x):
    ctx["tf_buffer"].xyz = [x, 0.0, 1.0]
    ctx["node"].timers[-1]()


def test_first_trigger_is_where_the_plate_seated(tmp_path, http):
    ctx, gpr = _armed(tmp_path, http)
    assert ctx["node"].pub.sent == [1]           # d = 0 at x = 0.30
    for x in (0.302, 0.3049, 0.3056, 0.3149, 0.3156):
        _move_to(ctx, x)
    assert ctx["node"].pub.sent == [1, 2, 3, 4]  # 0.5 / 1.0 / 1.5 cm


def test_normal_motion_and_jumps_fire_nothing(tmp_path, http):
    ctx, gpr = _armed(tmp_path, http)
    ctx["tf_buffer"].xyz = [0.30, 0.02, 1.0]     # press along the wall normal
    ctx["node"].timers[-1]()
    _move_to(ctx, 0.40)                          # 10 cm in one sample: TF jump
    assert ctx["node"].pub.sent == [1]


def test_unseated_stretch_keeps_firing_and_is_recorded(tmp_path, http):
    ctx, gpr = _armed(tmp_path, http)
    gpr.note_contact(ctx, True)
    _move_to(ctx, 0.31)
    gpr.note_contact(ctx, False)
    _move_to(ctx, 0.33)
    gpr.note_contact(ctx, True)
    _move_to(ctx, 0.35)
    assert ctx["node"].pub.sent[-1] == 11        # 5 cm at 0.5 cm, plus d = 0
    gpr.end_segment(ctx)
    (row,) = _manifest(tmp_path)
    (stretch,) = row["unseated"]
    assert (stretch["from_m"], stretch["to_m"]) == (0.01, 0.03)
    assert row["trigger_count"] == 11 and row["travel_m"] == 0.05


def test_contact_notes_before_arming_are_ignored(tmp_path, http):
    ctx = make_ctx(tmp_path)
    gpr = GprSweep("ScanWall", None)
    _begin(gpr, ctx, 0, 0, 0)
    gpr.note_contact(ctx, False)
    gpr.end_segment(ctx)
    assert "unseated" not in _manifest(tmp_path)[0]


# ----------------------------------------------------------------------
# ScanWall: arm on the sweep node's first "running: seated"
# ----------------------------------------------------------------------
class _GprStub:
    def __init__(self):
        self.armed = False
        self.calls = []

    def arm_triggers(self, ctx, ref, seg_start, seg_end, speed):
        self.armed = True
        self.calls.append(("arm", ref, seg_start, speed))

    def note_contact(self, ctx, seated):
        self.calls.append(("contact", seated))


@pytest.fixture
def scan_wall():
    from task_planner_fsm.states.scan_wall import ScanWall
    state = ScanWall("ScanWall")
    state._gpr = _GprStub()
    state._segments = [SEG]
    state._seg_idx = 0
    state._seg_phase = "sweep_wait"
    return state


def _status(state, ctx, text):
    state._on_wbc_status(SimpleNamespace(data=text), ctx)


def test_scan_wall_arms_on_first_seated_not_on_approach(scan_wall, tmp_path):
    ctx = make_ctx(tmp_path, sweep_use_wbc=True, sim=True)
    _status(scan_wall, ctx, "running: approach")
    assert scan_wall._gpr.calls == []
    _status(scan_wall, ctx, "running: seated")
    _status(scan_wall, ctx, "running: unseated")
    _status(scan_wall, ctx, "running: seated")
    assert scan_wall._gpr.calls == [
        ("arm", "map", SEG[0], 0.045),
        ("contact", True), ("contact", False), ("contact", True),
    ]
    assert scan_wall._nav_status is None


def test_scan_wall_still_maps_terminal_statuses(scan_wall, tmp_path):
    ctx = make_ctx(tmp_path, sweep_use_wbc=True, sim=True)
    _status(scan_wall, ctx, "failed: the press never reached the wall")
    assert scan_wall._nav_status == -1
    assert scan_wall._gpr.calls == []


# ----------------------------------------------------------------------
# Sweep node: the contact suffix on the running status
# ----------------------------------------------------------------------
def test_sweep_node_reports_contact_on_the_running_status():
    from task_planner_fsm.wbc.sweep_node import WholeBodySweepNode
    node = WholeBodySweepNode.__new__(WholeBodySweepNode)
    node.status_pub = _Pub()
    node.status = "running"
    node.contact = "approach"
    node._set_contact(False)                     # dwell lost on the way in
    node._set_contact(True)
    node._set_contact(True)                      # no change, no publish
    node._set_contact(False)
    node.status = "succeeded"
    node._publish_status()
    assert node.status_pub.sent == [
        "running: seated", "running: unseated", "succeeded"]
