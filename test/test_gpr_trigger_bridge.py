"""ScanWall's gate on the GPR trigger bridge, and the bridge's own ACK/PONG
bookkeeping (the part that decides what ``~/status`` reports).

The bridge is exercised without rclpy: the object is built with ``__new__``
and only the fields ``_handle_line``/``_snapshot`` touch are set.

Run with:

    python3 -m pytest test/test_gpr_trigger_bridge.py -v
"""

import threading
import time

import pytest

from task_planner_fsm.states.scan_wall import ScanWall
from task_planner_fsm.gpr_trigger_bridge import GprTriggerBridge


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(msg)

    warn = error = debug = info


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


# ----------------------------------------------------------------------
# ScanWall gate
# ----------------------------------------------------------------------
def make_ctx(**over):
    ctx = {"node": _Node(), "gpr_trigger_enabled": True}
    ctx.update(over)
    return ctx


@pytest.fixture
def state():
    return ScanWall("ScanWall")


def test_gate_is_open_when_not_required(state):
    # Default: sim / bench without the board. No status at all must still pass.
    assert state._gpr_trigger_bridge_ready(make_ctx()) == (True, "")


def test_gate_is_open_when_triggers_disabled(state):
    ctx = make_ctx(gpr_trigger_enabled=False, gpr_trigger_bridge_required=True)
    assert state._gpr_trigger_bridge_ready(ctx) == (True, "")


def test_gate_needs_a_status(state):
    ctx = make_ctx(gpr_trigger_bridge_required=True)
    ok, reason = state._gpr_trigger_bridge_ready(ctx)
    assert not ok and "gpr_trigger_bridge running" in reason


def test_gate_rejects_a_stale_status(state):
    ctx = make_ctx(
        gpr_trigger_bridge_required=True,
        gpr_trigger_bridge_status={"alive": True},
        gpr_trigger_bridge_status_stamp=time.time() - 10.0,
    )
    ok, reason = state._gpr_trigger_bridge_ready(ctx)
    assert not ok and "old" in reason


def test_gate_rejects_a_dead_receiver(state):
    ctx = make_ctx(
        gpr_trigger_bridge_required=True,
        gpr_trigger_bridge_status={"alive": False, "receiver": "10.0.0.5:5005", "last_rx_age_s": 7.5},
        gpr_trigger_bridge_status_stamp=time.time(),
    )
    ok, reason = state._gpr_trigger_bridge_ready(ctx)
    assert not ok and "10.0.0.5:5005" in reason


def test_gate_passes_a_live_receiver(state):
    ctx = make_ctx(
        gpr_trigger_bridge_required=True,
        gpr_trigger_bridge_status={"alive": True},
        gpr_trigger_bridge_status_stamp=time.time(),
    )
    assert state._gpr_trigger_bridge_ready(ctx) == (True, "")


def test_arming_fails_the_state_on_a_dead_bridge(state):
    ctx = make_ctx(
        gpr_trigger_bridge_required=True,
        gpr_trigger_bridge_status={"alive": False, "receiver": "x", "last_rx_age_s": 9},
        gpr_trigger_bridge_status_stamp=time.time(),
    )
    state._seg_idx = 0
    state._open_gpr_line_record = lambda *a, **kw: None
    state._start_gpr_triggers(ctx, (0.0, 0.0, 1.0), (1.0, 0.0, 1.0))
    assert ctx["error_triggered"] is True
    assert "not answering" in ctx["error_reason"]
    assert state._gpr_trigger_timer is None


# ----------------------------------------------------------------------
# Bridge bookkeeping
# ----------------------------------------------------------------------
def make_bridge():
    b = GprTriggerBridge.__new__(GprTriggerBridge)
    b.receiver = ("10.0.0.5", 5005)
    b.alive_timeout = 3.0
    b.ack_timeout = 0.5
    b._lock = threading.Lock()
    b._seq = 0
    b._acked = 0
    b._pending = {}
    b._expired = set()
    b._lost = 0
    b._caught_up = 0
    b._sent = 0
    b._last_rx = None
    b._ping_n = 0
    b._ping_sent = {}
    b._rtt_ms = None
    b._rssi = None
    b._receiver_seq = None
    b._receiver_fired = None
    b._pong_waiters = {}
    b._errors = 0
    b._log = _Logger()
    b.get_logger = lambda: b._log
    return b


def _pretend_sent(b, *seqs):
    for s in seqs:
        b._pending[s] = time.time()
        b._seq = max(b._seq, s)
        b._sent += 1


def test_ack_clears_pending():
    b = make_bridge()
    _pretend_sent(b, 1, 2)
    b._handle_line("ACK 1 1", ("10.0.0.5", 5005))
    snap = b._snapshot()
    assert snap["alive"] and snap["acked"] == 1 and snap["pending"] == 1
    assert snap["lost"] == 0 and snap["caught_up"] == 0


def test_catch_up_ack_covers_the_lost_triggers():
    # TRIG 2 and 3 never reached the board; TRIG 4 made it fire three points.
    b = make_bridge()
    _pretend_sent(b, 1, 2, 3, 4)
    b._handle_line("ACK 1 1", ("10.0.0.5", 5005))
    b._handle_line("ACK 4 3", ("10.0.0.5", 5005))
    snap = b._snapshot()
    assert snap["acked"] == 4
    assert snap["pending"] == 0          # 2 and 3 were fired by the catch-up
    assert snap["caught_up"] == 2
    assert snap["lost"] == 0


def test_unacked_trigger_counts_as_lost_after_timeout():
    b = make_bridge()
    _pretend_sent(b, 1)
    b._pending[1] = time.time() - 1.0     # older than ack_timeout
    b._expire_pending()
    snap = b._snapshot()
    assert snap["lost"] == 1 and snap["pending"] == 0
    assert any("unacked" in line for line in b._log.lines)


def test_late_catch_up_reclaims_an_expired_trigger():
    # TRIG 2 timed out (counted lost), then TRIG 3's ACK says it fired both.
    b = make_bridge()
    _pretend_sent(b, 2, 3)
    b._pending[2] = time.time() - 1.0
    b._expire_pending()
    assert b._snapshot()["lost"] == 1
    b._handle_line("ACK 3 2", ("10.0.0.5", 5005))
    snap = b._snapshot()
    assert snap["lost"] == 0 and snap["caught_up"] == 1 and snap["pending"] == 0


def test_pong_updates_link_stats_and_wakes_the_ping_service():
    b = make_bridge()
    ev = threading.Event()
    b._ping_sent[7] = time.time() - 0.012
    b._pong_waiters[7] = ev
    b._handle_line("PONG 7 41 -58 123456 41", ("10.0.0.5", 5005))
    snap = b._snapshot()
    assert ev.is_set()
    assert snap["receiver_seq"] == 41 and snap["rssi_dbm"] == -58
    assert 10.0 <= snap["rtt_ms"] <= 200.0


def test_unfired_is_sent_minus_what_the_board_reports():
    # 5 sent; the board says it fired 4 in total: one trace really is missing,
    # whatever the ACK bookkeeping thinks.
    b = make_bridge()
    _pretend_sent(b, 1, 2, 3, 4, 5)
    b._handle_line("ACK 5 1 4", ("10.0.0.5", 5005))
    snap = b._snapshot()
    assert snap["receiver_fired"] == 4 and snap["unfired"] == 1
    # ...and a lost ACK is not a lost trace: PONG carries the same total.
    b._handle_line("PONG 3 5 -60 999 5", ("10.0.0.5", 5005))
    assert b._snapshot()["unfired"] == 0


def test_not_alive_until_the_receiver_answers():
    b = make_bridge()
    assert b._snapshot()["alive"] is False
    b._handle_line("ACK RESET", ("10.0.0.5", 5005))
    assert b._snapshot()["alive"] is True


class _Param:
    def __init__(self, v):
        self.value = v


def _with_params(b, **params):
    b.get_parameter = lambda name: _Param(params[name])
    return b


@pytest.mark.parametrize("spacing_m, expected", [
    (0.005, 8),     # 2 scans/cm  -> round(16 / 2)
    (0.01, 16),     # 1 scan/cm   -> round(16 / 1)
    (0.02, 32),     # 0.5 scans/cm-> round(16 / 0.5)
])
def test_pulses_per_scan_follow_the_handheld_calibration(spacing_m, expected):
    b = _with_params(make_bridge(), pulses_per_scan=0, encoder_cycles_per_cm=16.0,
                     trigger_distance_m=spacing_m)
    assert b._pulses_per_scan() == expected


def test_explicit_pulses_per_scan_wins():
    b = _with_params(make_bridge(), pulses_per_scan=5, encoder_cycles_per_cm=16.0,
                     trigger_distance_m=0.005)
    assert b._pulses_per_scan() == 5


def test_trig_line_carries_the_encoder_burst():
    b = _with_params(make_bridge(), mode=1, pulses_per_scan=0, encoder_cycles_per_cm=16.0,
                     trigger_distance_m=0.005, half_period_us=800, trigger_pattern=0,
                     trigger_hold_ms=50)
    assert b._trig_line(37) == "TRIG 37 1 8 800 0 50"


def test_err_lines_are_counted_and_logged():
    b = make_bridge()
    b._handle_line("ERR unknown command", ("10.0.0.5", 5005))
    assert b._snapshot()["errors"] == 1
    assert any("unknown command" in line for line in b._log.lines)
