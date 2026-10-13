r"""The arm setpoint stream, in a process of its own.

``sweep_node`` decides what velocity the arm should have; something then has to
turn that into a position setpoint on ``forward_position_controller`` at a
steady rate, because the UR's servoj chases every setpoint it is given. When
that something was a timer inside ``sweep_node`` it shared one Python
interpreter with the solve, the sensor callbacks and the executor, and only one
of them runs at a time. On the robot (2026-10-07, bags 13_57 and 16_05) the
100 Hz stream delivered a setpoint every 24 ms at the median, 110-180 ms at the
99th percentile and up to 395 ms at worst, while the driver's own
``/joint_states`` held 10 ms throughout. At 100 % on the teach pendant the arm
reached each late setpoint, stopped on it, and jumped when the next one came:
10-40 rad/s^2 measured in the air against the 2 commanded. It was never ahead of
its setpoint; it was being stopped and started. Closing RViz did not change it,
and bounding the solve's jerk (8e8cb24) could not, because the solve was never
the part arriving late.

So the stream moves out. This module is three pieces:

* :class:`StreamCore` — the tick itself: integrate the last trusted velocity,
  reaching each new one on a ramp at ``accel`` and ramping to rest when it goes
  stale (see :meth:`StreamCore.tick`), plus one new guard, below. Plain Python,
  so it is tested without a process.
* :func:`run` — the process: its own interpreter, its own rclpy context, one
  timer and one ``/joint_states`` subscription on a single-threaded executor,
  and a thread reading the pipe from ``sweep_node``. Both run SCHED_FIFO when
  allowed (see :func:`_realtime`): a process of its own fixed the interpreter
  contention, not the CPU contention.
* :class:`RemoteArmStream` — ``sweep_node``'s end, with the parts of
  :class:`~.streaming.ArmStream`'s interface the node uses.

**Why a pipe and shared memory, not topics.** Every safety property of the
stream is about ORDER: a hold must land after the last velocity and before the
arm is handed back to the trajectory controller, and nothing may advance the
setpoint after it. A pipe is ordered and reliable; the hold's acknowledgement
comes back through shared memory, so ``sweep_node`` can wait for it without
spinning an executor at shutdown, when it has none. And the pipe closing IS the
death of ``sweep_node`` — however it died — which the streamer turns into a
hold at the measurement and an exit. The streamer never writes to the pipe, so
nothing it does can block ``sweep_node``.

**Signals.** The FSM stops a sweep with SIGINT to the whole process group
(``proc_utils.stop_proc``), which includes this process. Exiting on that would
pull the stream out from under the hold ``sweep_node`` is about to send, so
SIGINT and SIGTERM are ignored here: the streamer leaves when ``sweep_node``
says so, or when the pipe closes. The FSM's SIGKILL escalation still ends it;
the controller is then left holding the last setpoint, a pose.

**The guard: an arm that has stopped following.** 16_05, 16:10:28-31: a 265 ms
gap in the stream let the arm stop on the last setpoint, and when setpoints
resumed — already 55 mrad ahead — the UR did not follow them. No log on the
pendant, speed scaling at 100 %, the F/T reading free air. The setpoint wound
up to the full ``max_lead`` (0.2 rad) over 2.6 s, until a hold re-seeded it at
the measurement and the arm moved off again at once. A setpoint wound 0.2 rad
ahead of an arm that might start following it at any moment is a lurch waiting
to happen. So in the air, when the setpoint is moving, the arm makes less than
``follow_ratio`` of that progress, and the gap has grown past ``follow_lead``
for ``follow_seconds``, the integrator is re-anchored at the measurement and
ramps up again from the arm's own velocity; ``air_max_lead`` bounds how far it
can wind meanwhile. It happened again on 2026-10-08 11:03:33, with setpoints
arriving on time. Off on the wall: a press holds the plate still against a
setpoint that leads it by design.
"""

import os
import signal
import threading

import numpy as np

from .streaming import DEFAULT_CONTROLLER, DEFAULT_TOPIC, POSITION, ArmStream

# Pipe messages, sweep_node -> streamer. Tuples of (kind, seq, *payload); the
# sequence number is what the acknowledgement echoes.
VELOCITY = "velocity"   # (kind, seq, qdot, max_age, in_air)
HOLD = "hold"           # (kind, seq, q or None) — None holds the last setpoint
RESET = "reset"         # (kind, seq, q) — re-seed without publishing
LIMITS = "limits"       # (kind, seq, lower, upper)
EXIT = "exit"           # (kind, seq)

# Shared status, streamer -> sweep_node: doubles, each written only by the
# streamer, read whenever sweep_node likes.
HEARTBEAT = 0     # stream ticks so far
ACK = 1           # sequence number of the last message handled
LEAD = 2          # worst-joint setpoint lead over the arm, rad
REANCHORS = 3     # times the follow guard has re-anchored
TICK_PERIOD = 4   # measured interval of the last tick, s
HOLDING = 5       # 1 while no velocity is being integrated
READY = 6         # 1 once the timer and subscription exist
RT_PRIORITY = 7   # SCHED_FIFO priority the tick runs at, 0 if it could not get one
STATUS_SIZE = 8


class StreamCore:
    """One stream's state and decisions; no threads, no clock of its own.

    ``node`` only needs ``create_publisher`` and ``get_logger`` — it is handed
    straight to :class:`~.streaming.ArmStream`. ``now`` is always passed in.
    """

    def __init__(self, node, joint_names, topic=None, max_lead=0.2, stream_rate=100.0,
                 stream_period_max_factor=50.0, follow_lead=0.05,
                 follow_moving_speed=0.01, follow_ratio=0.3, follow_seconds=0.25,
                 accel=2.0, air_max_lead=0.1):
        self.stream = ArmStream(node, joint_names, mode=POSITION, topic=topic,
                                max_lead=max_lead)
        self.logger = node.get_logger()
        self.n = len(joint_names)
        self.stream_rate = float(stream_rate)
        self.stream_period_max_factor = float(stream_period_max_factor)
        self.follow_lead = float(follow_lead)
        self.follow_moving_speed = float(follow_moving_speed)
        self.follow_ratio = float(follow_ratio)
        self.follow_seconds = float(follow_seconds)
        # rad/s^2, worst joint: how fast the integrated velocity may change.
        # 0 = no ramp: velocities are stepped and a stale one is a hard hold.
        self.accel = float(accel)
        # The lead clamp in the air, rad (0 = use max_lead everywhere). On the
        # wall the setpoint leads the plate by design and keeps max_lead.
        self.max_lead = float(max_lead)
        self.air_max_lead = float(air_max_lead)

        self.qdot = None            # the velocity being integrated, or None
        self.qdot_stamp = None      # when it arrived, on the streamer's clock
        self.max_age = None         # how long it stays trusted, s
        self.in_air = False
        self.stale = False          # latched while the velocity is stale
        # The velocity actually being integrated: qdot reached at ``accel``.
        # None = at rest (after a seed or a hold).
        self.v_out = None
        self.stream_stamp = None
        self.tick_period = 0.0
        self.q = None               # measured, controller order
        self.qd = None
        self.still_since = None
        self.reanchors = 0
        self.ack = 0
        self.exit_requested = False

    # ------------------------------------------------------------------
    def measured(self, q, qd):
        self.q = None if q is None else np.asarray(q, dtype=float)
        self.qd = None if qd is None else np.asarray(qd, dtype=float)

    def handle(self, msg, now):
        """Apply one pipe message. Holds and seeds publish immediately."""
        kind, seq = msg[0], int(msg[1])
        if kind == VELOCITY:
            qdot, max_age, in_air = msg[2], msg[3], msg[4]
            self.qdot = np.asarray(qdot, dtype=float)
            self.qdot_stamp = now
            self.max_age = float(max_age)
            self.in_air = bool(in_air)
        elif kind == HOLD:
            self._stop()
            self.stream.hold(None if msg[2] is None else np.asarray(msg[2], dtype=float))
        elif kind == RESET:
            self._stop()
            self.stream.reset(np.asarray(msg[2], dtype=float))
        elif kind == LIMITS:
            self.stream.set_position_limits(msg[2], msg[3])
        elif kind == EXIT:
            self.exit_requested = True
        else:
            self.logger.error(f"Arm streamer: unknown message '{kind}'; ignored.")
        self.ack = seq

    def parent_gone(self):
        """sweep_node is gone: stop where the arm is, and leave."""
        self._stop()
        self.stream.hold(self.q)
        self.exit_requested = True
        self.logger.error(
            "The sweep controller is gone; holding the arm where it is and exiting.")

    def _stop(self):
        self.qdot = None
        self.qdot_stamp = None
        self.stream_stamp = None
        self.stale = False
        self.v_out = None
        self.still_since = None

    # ------------------------------------------------------------------
    def tick(self, now):
        """One stream tick: integrate the solve's velocity, reached at ``accel``.

        Every change of velocity is a ramp, not only the stops. 2026-10-08
        10:59: the solve ran at ~6 Hz and each answer could move the velocity
        by up to ~0.16 rad/s (the solve's own bound is clamped at 4 nominal
        periods), which the stream used to apply in one 10 ms tick — 14-20
        rad/s^2 in the air against the 2 commanded. A ramp at accel turns each
        into a 10-80 ms slope; at pressing speeds (mrad/s) it is invisible.

        A stale velocity — the solve has stopped, or its messages have — is a
        ramp to zero along the same path, then quiet: the controller holds the
        last setpoint. It used to be a hold at the measurement on the spot, a
        stop from full speed in one tick (16:57 on 2026-10-07: up to 60
        rad/s^2). At 0.4 rad/s and 2 rad/s^2 it is 0.04 rad more travel.
        """
        if self.qdot is None or self.qdot_stamp is None:
            return
        nominal = 1.0 / self.stream_rate
        elapsed = (now - self.stream_stamp) if self.stream_stamp else nominal
        dt = float(min(max(elapsed, nominal), self.stream_period_max_factor * nominal,
                       self.max_age))
        self.tick_period = elapsed
        self.stream_stamp = now

        stale = now - self.qdot_stamp > self.max_age
        if stale and not self.stale:
            self.stale = True
            self.logger.error(
                f"No control solution for {now - self.qdot_stamp:.2f}s: bringing the "
                f"arm to rest rather than streaming on with the last velocity it "
                f"was given.")
            if self.accel <= 0.0:
                self.v_out = None
                self.stream.hold(self.q)
                return
        elif not stale and self.stale:
            self.stale = False
            self.logger.warn("Control solutions are arriving again; resuming.")
        if stale and self.accel <= 0.0:
            return

        self._ramp(np.zeros(self.n) if stale else self.qdot, dt)
        if stale and float(np.max(np.abs(self.v_out))) <= 1e-9:
            return                                   # at rest: leave it quiet
        self.stream.max_lead = (self.air_max_lead if self.in_air and self.air_max_lead > 0.0
                                else self.max_lead)
        if not stale:
            self._follow_guard(now)
        self.stream.send(self.v_out, dt, self.q)

    def _ramp(self, target, dt):
        """Move v_out toward ``target`` by at most accel*dt on the worst joint.

        The whole change is scaled, not clipped per joint, so a ramp keeps the
        direction the arm is moving in — a stop is a stop along the path.
        """
        v0 = np.zeros(self.n) if self.v_out is None else self.v_out
        target = np.asarray(target, dtype=float)
        if self.accel <= 0.0:
            self.v_out = target.copy()
            return
        delta = target - v0
        peak = float(np.max(np.abs(delta)))
        step = self.accel * dt
        if peak > step:
            delta = delta * (step / peak)
        self.v_out = v0 + delta

    def _follow_guard(self, now):
        """Re-anchor at the arm when, in the air, it has stopped following.

        "Stopped following" is the arm making less than ``follow_ratio`` of the
        commanded progress, not the arm standing still: 10:59 on 2026-10-08 it
        crept at 0.02-0.05 rad/s under a 0.36 rad/s setpoint, which the old
        still-below-5-mrad/s test missed until the setpoint had wound up to the
        full 0.2 rad. After a re-anchor the stream restarts from the velocity
        the arm actually has and ramps up from there, instead of handing a
        stationary arm the full velocity in one tick (the 30 rad/s^2 that run
        ended its return with).
        """
        if (not self.in_air or self.q is None or self.qd is None
                or self.stream.command is None):
            self.still_since = None
            return
        lead = float(np.max(np.abs(self.stream.command - self.q)))
        speed = float(np.linalg.norm(self.v_out))
        moving = float(np.max(np.abs(self.v_out))) > self.follow_moving_speed
        progress = float(self.qd @ self.v_out) / speed if speed > 1e-12 else 0.0
        lagging = progress < self.follow_ratio * speed
        if not (moving and lagging and lead > self.follow_lead):
            self.still_since = None
            return
        if self.still_since is None:
            self.still_since = now
            return
        if now - self.still_since < self.follow_seconds:
            return
        self.stream.reset(self.q)
        self.v_out = self.qd.copy()
        self.still_since = None
        self.reanchors += 1
        self.logger.warn(
            f"The arm stopped following its setpoint ({lead * 1000:.0f} mrad behind, "
            f"{progress:.3f} of {speed:.3f} rad/s for {self.follow_seconds:.2f}s): "
            f"re-anchoring the stream at the arm (#{self.reanchors}).")

    def lead(self):
        if self.stream.command is None or self.q is None:
            return 0.0
        return float(np.max(np.abs(self.stream.command - self.q)))


# ----------------------------------------------------------------------
# The process.
# ----------------------------------------------------------------------
def run(conn, status, config):
    """The streamer's loop. ``conn`` is the read end of the pipe, ``status`` a
    float64 array over the shared status memory."""
    # See the module docstring: the FSM's SIGINT to the group is not for us.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(
        config.get("node_name", "wbc_arm_streamer"),
        parameter_overrides=[Parameter("use_sim_time",
                                       value=bool(config.get("use_sim_time", False)))])
    joints = list(config["joint_names"])
    core = StreamCore(
        node, joints, topic=config.get("topic"), max_lead=config["max_lead"],
        stream_rate=config["stream_rate"],
        stream_period_max_factor=config["stream_period_max_factor"],
        follow_lead=config["follow_lead"],
        follow_moving_speed=config["follow_moving_speed"],
        follow_ratio=config.get("follow_ratio", 0.3),
        follow_seconds=config["follow_seconds"],
        accel=config.get("accel", 2.0),
        air_max_lead=config.get("air_max_lead", 0.1))
    lock = threading.Lock()
    priority = int(config.get("rt_priority", 0))
    parent = os.getppid()

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def publish_status():
        status[ACK] = float(core.ack)
        status[LEAD] = core.lead()
        status[REANCHORS] = float(core.reanchors)
        status[TICK_PERIOD] = core.tick_period
        status[HOLDING] = 0.0 if core.qdot is not None and not core.stale else 1.0

    def reader():
        _realtime(priority, node.get_logger(), "Arm streamer pipe reader", quiet=True)
        while True:
            try:
                msg = conn.recv()
            except (EOFError, OSError):
                msg = None
            with lock:
                if msg is None:
                    core.parent_gone()
                else:
                    core.handle(msg, now())
                publish_status()
                if core.exit_requested:
                    return

    def on_joint_states(msg):
        try:
            index = [msg.name.index(name) for name in joints]
        except ValueError:
            return
        q = [msg.position[i] for i in index]
        qd = [msg.velocity[i] for i in index] if len(msg.velocity) == len(msg.name) else None
        with lock:
            core.measured(q, qd)

    def on_tick():
        with lock:
            if os.getppid() != parent and not core.exit_requested:
                core.parent_gone()
            core.tick(now())
            status[HEARTBEAT] += 1.0
            publish_status()

    node.create_subscription(JointState, config.get("joint_states_topic", "/joint_states"),
                             on_joint_states, 10)
    node.create_timer(1.0 / float(config["stream_rate"]), on_tick)
    # The tick thread is this one: the executor spins here. Raised before the
    # reader starts, which inherits nothing (it raises itself). The DDS threads
    # rclpy.init made stay where they are.
    status[RT_PRIORITY] = float(_realtime(priority, node.get_logger(), "Arm streamer stream tick"))
    threading.Thread(target=reader, name="arm_streamer_pipe", daemon=True).start()
    status[READY] = 1.0

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            with lock:
                if core.exit_requested:
                    break
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def _realtime(priority, logger, what, quiet=False):
    """Put the CALLING thread on SCHED_FIFO at ``priority``; return what it got.

    16:57 on 2026-10-07: with the stream in its own process the setpoints came
    10 ms apart at the median, but still 50-130 ms apart now and then, and those
    were the jolts. Not the interpreter any more — the CPU: load 45 on 12 cores,
    and this whole process tree at nice 19 behind rtabmap and icp_odometry at
    70 % of a core each. The UR driver's own loops run SCHED_FIFO (controller
    manager 50, EtherCAT 80, RTDE 99) for exactly this reason, and the user is
    in the realtime group (RLIMIT_RTPRIO 99) on an RT kernel, so this may too.
    Below the controller manager, which has to run first; a tick is
    microseconds, and the kernel's RT throttling (95 %) is the backstop.
    """
    if priority <= 0:
        return 0
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
    except (PermissionError, OSError) as exc:
        logger.warn(
            f"{what}: could not get SCHED_FIFO {priority} ({exc}); running at normal "
            f"priority, where a loaded CPU can delay it.")
        return 0
    if not quiet:
        logger.info(f"{what} running SCHED_FIFO priority {priority}.")
    return priority


def main(argv=None):
    """``python3 -m task_planner_fsm.wbc.arm_streamer --fd N --shm NAME --config JSON``.

    A plain subprocess rather than multiprocessing: 'spawn' re-imports the
    PARENT's main script in the child, so the streamer would depend on how
    sweep_node happened to be started. Run as a module, it depends on nothing
    but this file.
    """
    import argparse
    import json
    from multiprocessing import resource_tracker, shared_memory
    from multiprocessing.connection import Connection

    parser = argparse.ArgumentParser()
    parser.add_argument("--fd", type=int, required=True)
    parser.add_argument("--shm", required=True)
    parser.add_argument("--config", required=True)
    args = parser.parse_args(argv)

    shm = shared_memory.SharedMemory(name=args.shm)
    # The parent created it and unlinks it; without this the child's resource
    # tracker would unlink it too, on exit, under the parent's feet.
    resource_tracker.unregister(shm._name, "shared_memory")
    status = np.ndarray((STATUS_SIZE,), dtype=np.float64, buffer=shm.buf)
    try:
        run(Connection(args.fd, writable=False), status, json.loads(args.config))
    finally:
        del status
        shm.close()


class RemoteArmStream:
    """sweep_node's end of the stream: the ArmStream calls the node makes.

    ``send`` is absent on purpose: the node hands velocities over with
    :meth:`velocity` from its solve, and the streamer decides when they reach
    the wire.
    """

    def __init__(self, node, joint_names, topic=None, max_lead=0.2, config=None):
        import json
        import subprocess
        import sys
        from multiprocessing import shared_memory
        from multiprocessing.connection import Connection

        self.node = node
        self.joint_names = list(joint_names)
        self.mode = POSITION
        self.topic = topic or DEFAULT_TOPIC[POSITION]
        self.controller = DEFAULT_CONTROLLER[POSITION]
        self.max_lead = float(max_lead)
        config = dict(config or {})
        config.update(joint_names=self.joint_names, topic=self.topic, max_lead=self.max_lead)

        self._shm = shared_memory.SharedMemory(create=True, size=8 * STATUS_SIZE)
        self._status = np.ndarray((STATUS_SIZE,), dtype=np.float64, buffer=self._shm.buf)
        self._status[:] = 0.0
        # Not inheritable by default; pass_fds makes the read end the streamer's
        # alone, so the pipe closes exactly when THIS process lets go of the
        # write end — by close(), or by dying.
        read_fd, write_fd = os.pipe()
        self._proc = subprocess.Popen(
            [sys.executable, "-m", "task_planner_fsm.wbc.arm_streamer",
             "--fd", str(read_fd), "--shm", self._shm.name, "--config", json.dumps(config)],
            pass_fds=(read_fd,))
        os.close(read_fd)
        self._conn = Connection(write_fd, readable=False)
        self._seq = 0
        self._lock = threading.Lock()
        self.dead = False
        self._heartbeat = (-1.0, None)

    def _alive(self):
        return self._proc.poll() is None

    # ------------------------------------------------------------------
    def _send(self, kind, *payload):
        with self._lock:
            self._seq += 1
            if self.dead:
                return self._seq
            try:
                self._conn.send((kind, self._seq) + payload)
            except (BrokenPipeError, EOFError, OSError):
                self.dead = True
                self.node.get_logger().error(
                    "The arm streamer process is gone; the arm is left on its last setpoint.")
            return self._seq

    @staticmethod
    def _floats(values):
        return None if values is None else [float(v) for v in values]

    def ready(self):
        return not self.dead and self._alive() and self._status[READY] > 0.0

    def alive_since(self, now, max_age):
        """False once the streamer has not ticked for ``max_age`` seconds."""
        beat = float(self._status[HEARTBEAT])
        last, stamp = self._heartbeat
        if beat != last or stamp is None:
            self._heartbeat = (beat, now)
            return self.ready()
        return self.ready() and now - stamp <= max_age

    def set_position_limits(self, lower, upper):
        self._send(LIMITS, self._floats(lower), self._floats(upper))

    def reset(self, q_measured):
        if q_measured is not None:
            self._send(RESET, self._floats(q_measured))

    def initial_command(self, q_measured):
        return self.hold(q_measured)

    def velocity(self, qdot, max_age, in_air):
        self._send(VELOCITY, self._floats(qdot), float(max_age), bool(in_air))

    def hold(self, q_measured=None):
        self._send(HOLD, self._floats(q_measured))
        return None

    def flush(self, timeout=1.0):
        """Wait until the streamer has handled everything sent so far."""
        import time

        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.dead or not self._alive():
                return False
            if self._status[ACK] >= self._seq:
                return True
            time.sleep(0.002)
        return False

    def lead(self, q_measured=None):
        return float(self._status[LEAD])

    def reanchors(self):
        return int(self._status[REANCHORS])

    def tick_period(self):
        return float(self._status[TICK_PERIOD])

    def close(self, timeout=1.0):
        """Make sure the last hold is on the wire, then end the process."""
        import subprocess

        flushed = self.flush(timeout)
        if not flushed and not self.dead:
            self.node.get_logger().error(
                "The arm streamer did not confirm the last command before shutdown.")
        self._send(EXIT)
        try:
            self._conn.close()
        except OSError:
            pass
        try:
            self._proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait(1.0)
        try:
            self._status = None
            self._shm.close()
            self._shm.unlink()
        except (FileNotFoundError, BufferError):
            pass
        return flushed


if __name__ == "__main__":
    main()
