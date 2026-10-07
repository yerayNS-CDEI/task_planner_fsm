"""The arm setpoint stream in its own process: what it decides, and that it
leaves when it should.

Most of this drives :class:`StreamCore` directly on a hand-advanced clock — the
decisions are the in-process stream's, plus the follow guard. The last tests
start the real process (on the suite's private ROS domain, see conftest.py, and
on topics nothing else uses) to check the parts a stub cannot: that it comes
up, ticks, confirms a hold, and exits both when told and when the sweep
controller's end of the pipe disappears.
"""

import time

import numpy as np
import pytest

pytest.importorskip("std_msgs")

from task_planner_fsm.wbc import arm_streamer as streamer  # noqa: E402
from task_planner_fsm.wbc.arm_streamer import (  # noqa: E402
    EXIT,
    HOLD,
    LIMITS,
    VELOCITY,
    RemoteArmStream,
    StreamCore,
)

JOINTS = ["j1", "j2", "j3"]
RATE = 100.0
DT = 1.0 / RATE


class _Recorder:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))

    @property
    def last(self):
        return self.sent[-1] if self.sent else None


class _StubNode:
    def __init__(self):
        self.publisher = _Recorder()
        self.logs = []

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher

    def get_logger(self):
        return self

    def error(self, message, **_kwargs):
        self.logs.append(message)

    warn = info = error


def _core(**kwargs):
    node = _StubNode()
    core = StreamCore(node, JOINTS, stream_rate=RATE, **kwargs)
    return node, core


class _Seq:
    def __init__(self):
        self.n = 0

    def __call__(self, kind, *payload):
        self.n += 1
        return (kind, self.n) + payload


def _seeded(q=(0.1, -1.0, 1.5), **kwargs):
    node, core = _core(**kwargs)
    seq = _Seq()
    q = np.array(q)
    core.measured(q, np.zeros(3))
    core.handle(seq(HOLD, list(q)), 0.0)        # initial_command
    return node, core, seq, q


# ----------------------------------------------------------------------
# What the in-process stream did, unchanged
# ----------------------------------------------------------------------
def test_a_velocity_is_integrated_over_the_measured_tick():
    node, core, seq, q = _seeded()
    assert node.publisher.last == pytest.approx(list(q)), "the seed goes out at once"
    core.handle(seq(VELOCITY, [0.1, 0.0, -0.2], 0.5, True), 0.0)
    core.tick(0.01)
    core.tick(0.03)                                  # a late tick: 20 ms of motion
    expect = q + np.array([0.1, 0.0, -0.2]) * (DT + 0.02)
    assert node.publisher.last == pytest.approx(list(expect))


def test_a_stale_velocity_holds_once_at_the_arm_and_resumes_on_a_fresh_one():
    node, core, seq, q = _seeded()
    core.handle(seq(VELOCITY, [0.2, 0.0, 0.0], 0.1, True), 0.0)
    core.tick(0.01)
    moved = len(node.publisher.sent)
    arm = q + np.array([0.001, 0.0, 0.0])
    core.measured(arm, np.zeros(3))
    for t in (0.2, 0.21, 0.22):
        core.tick(t)
    assert len(node.publisher.sent) == moved + 1, "one hold, then quiet"
    assert node.publisher.last == pytest.approx(list(arm)), "held at the measurement"
    assert any("No control solution" in m for m in node.logs)

    core.handle(seq(VELOCITY, [0.2, 0.0, 0.0], 0.1, True), 0.3)
    core.tick(0.31)
    assert node.publisher.last[0] > arm[0], "moving again from where it was held"


def test_a_hold_stops_the_integration_immediately():
    node, core, seq, q = _seeded()
    core.handle(seq(VELOCITY, [0.3, 0.0, 0.0], 0.5, True), 0.0)
    core.tick(0.01)
    here = np.array(node.publisher.last)
    core.handle(seq(HOLD, None), 0.015)              # hold the last setpoint
    assert node.publisher.last == pytest.approx(list(here))
    n = len(node.publisher.sent)
    core.tick(0.02)
    core.tick(0.03)
    assert len(node.publisher.sent) == n, "nothing advances the setpoint after a hold"
    assert core.ack == seq.n


def test_the_setpoint_is_still_clamped_to_the_joint_limits_and_the_lead():
    node, core, seq, q = _seeded(max_lead=0.05)
    core.handle(seq(LIMITS, [-1.0, -1.05, -2.0], [1.0, 1.0, 2.0]), 0.0)
    core.handle(seq(VELOCITY, [0.0, -1.0, 2.0], 5.0, False), 0.0)
    for k in range(1, 30):
        core.tick(k * DT)
    out = np.array(node.publisher.last)
    assert out[1] >= -1.05 - 1e-12, "joint limit"
    assert out[2] <= q[2] + 0.05 + 1e-12, "lead clamp against an arm that is not moving"


# ----------------------------------------------------------------------
# The follow guard (16:10:28 on 2026-10-07)
# ----------------------------------------------------------------------
def _stalled_arm(core, seq, q, in_air, seconds, still=True):
    """Command 0.2 rad/s on j1 while the arm stays at ``q`` for ``seconds``."""
    core.handle(seq(VELOCITY, [0.2, 0.0, 0.0], 1.0, in_air), 0.0)
    qd = np.zeros(3) if still else np.array([0.2, 0.0, 0.0])
    t = 0.0
    while t < seconds:
        t += DT
        core.measured(q, qd)
        core.tick(t)
    return t


def test_an_arm_that_stops_following_in_the_air_is_re_anchored():
    node, core, seq, q = _seeded()
    _stalled_arm(core, seq, q, in_air=True, seconds=0.6)
    assert core.reanchors >= 1
    assert core.lead() < 0.05 + 0.2 * DT * 2, "back near the arm, not wound up to max_lead"
    assert any("stopped following" in m for m in node.logs)


def test_the_guard_waits_out_the_normal_start_of_a_motion():
    """From rest the arm lags its setpoint for a moment; that is not a stall."""
    _, core, seq, q = _seeded()
    _stalled_arm(core, seq, q, in_air=True, seconds=0.2)
    assert core.reanchors == 0


def test_the_guard_leaves_a_press_alone():
    """On the wall the plate is held still by the wall against a setpoint that
    leads it by design; re-anchoring there would undo the press."""
    _, core, seq, q = _seeded()
    _stalled_arm(core, seq, q, in_air=False, seconds=0.6)
    assert core.reanchors == 0


def test_the_guard_leaves_an_arm_that_is_moving_alone():
    _, core, seq, q = _seeded()
    _stalled_arm(core, seq, q, in_air=True, seconds=0.6, still=False)
    assert core.reanchors == 0


def test_losing_the_sweep_controller_holds_at_the_arm_and_exits():
    node, core, seq, q = _seeded()
    core.handle(seq(VELOCITY, [0.2, 0.0, 0.0], 1.0, True), 0.0)
    core.tick(0.01)
    arm = q + np.array([0.002, 0.0, 0.0])
    core.measured(arm, np.zeros(3))
    core.parent_gone()
    assert node.publisher.last == pytest.approx(list(arm))
    assert core.exit_requested
    n = len(node.publisher.sent)
    core.tick(0.02)
    assert len(node.publisher.sent) == n


def test_exit_is_a_request_the_loop_acts_on():
    _, core, seq, _ = _seeded()
    core.handle(seq(EXIT), 0.0)
    assert core.exit_requested and core.ack == seq.n


# ----------------------------------------------------------------------
# The real process
# ----------------------------------------------------------------------
class _LogNode:
    def __init__(self):
        self.logs = []

    def get_logger(self):
        return self

    def error(self, message, **_kwargs):
        self.logs.append(message)

    warn = info = error


def _remote():
    config = dict(node_name="test_arm_streamer", use_sim_time=False,
                  joint_states_topic="/test_arm_streamer/joint_states",
                  stream_rate=RATE, stream_period_max_factor=50.0, follow_lead=0.05,
                  follow_moving_speed=0.01, follow_still_speed=0.005, follow_seconds=0.25)
    return RemoteArmStream(_LogNode(), JOINTS, topic="/test_arm_streamer/commands",
                           max_lead=0.2, config=config)


def _wait(predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_the_streamer_process_ticks_confirms_a_hold_and_exits_when_told():
    remote = _remote()
    try:
        assert _wait(remote.ready, 30.0), "the process never came up"
        beats = remote._status[streamer.HEARTBEAT]
        time.sleep(0.5)
        ticks = remote._status[streamer.HEARTBEAT] - beats
        assert ticks > 0.5 * RATE * 0.5, f"only {ticks:.0f} ticks in 0.5 s"
        remote.initial_command([0.1, -1.0, 1.5])
        remote.velocity([0.1, 0.0, 0.0], 0.5, True)
        remote.hold(None)
        assert remote.flush(2.0), "the hold was never confirmed"
    finally:
        assert remote.close(2.0)
    assert not remote._alive()


def test_the_streamer_process_exits_when_the_sweep_controller_is_gone():
    """SIGKILL of the sweep controller closes its end of the pipe; the streamer
    must not outlive it, or a later sweep would find it still publishing."""
    remote = _remote()
    try:
        assert _wait(remote.ready, 30.0)
        remote._conn.close()                         # what the death of the parent does
        assert _wait(lambda: not remote._alive(), 5.0), "orphan streamer"
    finally:
        if remote._alive():
            remote._proc.kill()
        remote._status = None
        remote._shm.close()
        remote._shm.unlink()
