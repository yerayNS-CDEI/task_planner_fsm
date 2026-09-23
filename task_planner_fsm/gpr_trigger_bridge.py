#!/usr/bin/env python3
"""UDP bridge between the FSM's GPR triggers and the ESP32 fake-encoder receiver.

ScanWall fires one ``std_msgs/UInt32`` on ``/gpr/trigger`` per
``gpr_trigger_distance_m`` of sensor-plate travel (the message is the trigger
index within the current segment). This node turns each one into a ``TRIG``
datagram for the ESP32 on the robot Wi-Fi (``ESP32/GPR_RX_FINALE.ino``), which
clocks the GPR's encoder input. The wire protocol is documented at the top of
that sketch.

Delivery is verified out of band rather than in the trigger path: every TRIG
carries a bridge-wide monotonic sequence number, the receiver ACKs it after
the pulses went out, and a lost datagram is caught up by the receiver on the
next one (it fires ``seq - last_seq`` scan points). The subscriber therefore
never blocks the FSM's 50 Hz trigger timer on a Wi-Fi round trip; what it
knows about the link goes out on ``~/status`` (JSON) and ``~/alive`` once a
second, and ScanWall gates the sweep on that (``gpr_trigger_bridge_required``).

Services, for the bench and for launch checks::

    ros2 service call /gpr_trigger_bridge/ping  std_srvs/srv/Trigger
    ros2 service call /gpr_trigger_bridge/fire  std_srvs/srv/Trigger
    ros2 service call /gpr_trigger_bridge/reset std_srvs/srv/Trigger

Parameters (all overridable at launch; ``receiver_ip`` is the one you must set):

    receiver_ip        IP of the ESP32 on Oliwall_2G (DHCP reservation or static)
    receiver_port      5005
    trigger_topic      /gpr/trigger
    mode               1 = fake wheel encoder (default; the calibrated handheld
                       GPR_TX_01 used this), 2 = external trigger (GPR_TX_02,
                       never made the probe record with our wiring)
    encoder_cycles_per_cm   probe wheel calibration: quadrature cycles per cm
                       of travel. 16.0 measured with the handheld ("1 cm
                       corrisponde a circa 16 cicli encoder").
    trigger_distance_m plate travel per trigger; MUST equal the FSM's
                       ``gpr_trigger_distance_m`` (0.005 m default) and the
                       trace interval set in the GP app.
    pulses_per_scan    quadrature cycles per trigger. 0 (default) derives it:
                       round(encoder_cycles_per_cm * trigger_distance_m * 100),
                       i.e. 8 cycles per 0.5 cm -- the handheld's
                       round(16 / scans_per_cm). Set > 0 to force a value.
    half_period_us     quadrature half period (mode 1), handheld: 800
    trigger_pattern    0..4, see generateExternalTrigger in the sketch (mode 2)
    trigger_hold_ms    hold time of the external trigger (mode 2)

In encoder mode the probe records by DISTANCE: every trigger advances its
odometer by trigger_distance_m and the app records a trace each time the
odometer crosses its trace interval. So the trace count is right even if
the two intervals differ; matching them just makes it one trace per
trigger. A burst takes pulses * 4 * half_period_us (8 cycles: 25.6 ms),
well inside the 100 ms between triggers at 0.05 m/s and 0.5 cm.
    ping_period_s      link check interval
    alive_timeout_s    no PONG/ACK for this long -> alive=false
    ack_timeout_s      a TRIG unacked for this long counts as lost
"""

import json
import select
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger


class GprTriggerBridge(Node):

    def __init__(self):
        super().__init__("gpr_trigger_bridge")

        self.declare_parameter("receiver_ip", "")
        self.declare_parameter("receiver_port", 5005)
        self.declare_parameter("trigger_topic", "/gpr/trigger")
        self.declare_parameter("mode", 1)
        self.declare_parameter("encoder_cycles_per_cm", 16.0)
        self.declare_parameter("trigger_distance_m", 0.005)
        self.declare_parameter("pulses_per_scan", 0)      # 0 = derive, see docstring
        self.declare_parameter("half_period_us", 800)
        self.declare_parameter("trigger_pattern", 0)
        self.declare_parameter("trigger_hold_ms", 50)
        self.declare_parameter("ping_period_s", 1.0)
        self.declare_parameter("alive_timeout_s", 3.0)
        self.declare_parameter("ack_timeout_s", 0.5)

        ip = str(self.get_parameter("receiver_ip").value).strip()
        if not ip:
            raise RuntimeError(
                "receiver_ip is not set: ros2 run task_planner_fsm gpr_trigger_bridge "
                "--ros-args -p receiver_ip:=<ESP32 IP on Oliwall_2G>"
            )
        self.receiver = (ip, int(self.get_parameter("receiver_port").value))
        self.ping_period = float(self.get_parameter("ping_period_s").value)
        self.alive_timeout = float(self.get_parameter("alive_timeout_s").value)
        self.ack_timeout = float(self.get_parameter("ack_timeout_s").value)

        # One socket for both directions; the receiver replies to the source
        # port of whatever it got, so an ephemeral bind is enough.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.setblocking(False)

        # Link bookkeeping, shared between the executor thread and the RX thread.
        self._lock = threading.Lock()
        self._seq = 0                  # last TRIG sequence sent
        self._acked = 0                # highest sequence ACKed
        self._pending = {}             # seq -> send time, for the ACK timeout
        self._expired = set()          # timed-out seqs, until a catch-up ACK covers them
        self._lost = 0                 # TRIGs never ACKed nor caught up
        self._caught_up = 0            # extra scan points the receiver fired for us
        self._sent = 0
        self._last_rx = None           # wall time of the last ACK/PONG
        self._ping_n = 0
        self._ping_sent = {}           # n -> send time, for the RTT
        self._rtt_ms = None
        self._rssi = None
        self._receiver_seq = None      # lastSequence as reported by the receiver
        self._receiver_fired = None    # scan points the receiver says it fired since RESET
        self._pong_waiters = {}        # n -> threading.Event (the ~/ping service)
        self._errors = 0

        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            UInt32, str(self.get_parameter("trigger_topic").value), self._on_trigger, qos
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.alive_pub = self.create_publisher(Bool, "~/alive", 10)
        self.create_service(Trigger, "~/ping", self._srv_ping)
        self.create_service(Trigger, "~/fire", self._srv_fire)
        self.create_service(Trigger, "~/reset", self._srv_reset)

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.create_timer(self.ping_period, self._on_ping_timer)

        # Start the receiver from a known state: outputs low, sequence 0.
        self._send("RESET")
        mode = int(self.get_parameter("mode").value)
        if mode == 1:
            how = (
                f"fake encoder: {self._pulses_per_scan()} quadrature cycles per trigger "
                f"({float(self.get_parameter('encoder_cycles_per_cm').value):g} cycles/cm x "
                f"{float(self.get_parameter('trigger_distance_m').value) * 100.0:g} cm), "
                f"{int(self.get_parameter('half_period_us').value)} us half period"
            )
        else:
            how = (
                f"external trigger: pattern {int(self.get_parameter('trigger_pattern').value)}, "
                f"{int(self.get_parameter('trigger_hold_ms').value)} ms hold"
            )
        self.get_logger().info(
            f"GPR trigger bridge -> udp://{ip}:{self.receiver[1]} ({how}); "
            f"listening on '{self.get_parameter('trigger_topic').value}'."
        )

    # ------------------------------------------------------------------
    # TX
    # ------------------------------------------------------------------
    def _send(self, text):
        try:
            self.sock.sendto((text + "\n").encode("ascii"), self.receiver)
            return True
        except OSError as e:
            self.get_logger().error(f"UDP send failed: {e}")
            return False

    def _pulses_per_scan(self):
        """Quadrature cycles per trigger: the explicit parameter if set, else
        the wheel calibration times the trigger spacing (the handheld's
        round(ENCODER_CYCLES_PER_CM / scans_per_cm))."""
        forced = int(self.get_parameter("pulses_per_scan").value)
        if forced > 0:
            return forced
        cycles_per_cm = float(self.get_parameter("encoder_cycles_per_cm").value)
        spacing_cm = float(self.get_parameter("trigger_distance_m").value) * 100.0
        return max(1, int(round(cycles_per_cm * spacing_cm)))

    def _trig_line(self, seq):
        p = self.get_parameter
        return (
            f"TRIG {seq} {int(p('mode').value)} {self._pulses_per_scan()} "
            f"{int(p('half_period_us').value)} {int(p('trigger_pattern').value)} "
            f"{int(p('trigger_hold_ms').value)}"
        )

    def _fire(self):
        """Send one TRIG; returns its sequence number."""
        with self._lock:
            self._seq += 1
            seq = self._seq
            self._pending[seq] = time.time()
            self._sent += 1
        self._send(self._trig_line(seq))
        return seq

    def _on_trigger(self, msg):
        seq = self._fire()
        self.get_logger().debug(f"trigger #{msg.data} -> TRIG {seq}")

    def _on_ping_timer(self):
        with self._lock:
            self._ping_n += 1
            n = self._ping_n
            self._ping_sent[n] = time.time()
            # Forget pings that never came back so the dict does not grow.
            for k in [k for k, t in self._ping_sent.items() if time.time() - t > 10.0]:
                del self._ping_sent[k]
        self._send(f"PING {n}")
        self._expire_pending()
        self._publish_status()

    def _expire_pending(self):
        now = time.time()
        with self._lock:
            late = [s for s, t in self._pending.items() if now - t > self.ack_timeout]
            for s in late:
                del self._pending[s]
                self._expired.add(s)
                self._lost += 1
            # A catch-up never reaches further back than MAX_CATCH_UP on the
            # board; anything older stays lost for good, so stop tracking it.
            self._expired = {s for s in self._expired if s > self._seq - 100}
        if late:
            self.get_logger().warn(
                f"{len(late)} TRIG(s) unacked after {self.ack_timeout:.1f}s "
                f"(seq {min(late)}..{max(late)}); compare sent vs receiver_fired "
                f"in ~/status to see whether the board fired them."
            )

    # ------------------------------------------------------------------
    # RX
    # ------------------------------------------------------------------
    def _rx_loop(self):
        while rclpy.ok():
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.2)
            except (OSError, ValueError):
                return
            if not ready:
                continue
            try:
                data, addr = self.sock.recvfrom(256)
            except OSError:
                continue
            for line in data.decode("ascii", errors="replace").splitlines():
                self._handle_line(line.strip(), addr)

    def _handle_line(self, line, addr):
        if not line:
            return
        tok = line.split()
        now = time.time()
        with self._lock:
            self._last_rx = now
            if tok[0] == "ACK" and len(tok) >= 2 and tok[1].isdigit():
                seq = int(tok[1])
                fired = int(tok[2]) if len(tok) >= 3 and tok[2].isdigit() else 1
                if len(tok) >= 4 and tok[3].isdigit():
                    self._receiver_fired = int(tok[3])
                self._pending.pop(seq, None)
                if seq > self._acked:
                    self._acked = seq
                if fired > 1:
                    self._caught_up += fired - 1
                    # Those were TRIGs we had pending (or already gave up on):
                    # the receiver has fired them, so they are not lost data.
                    for s in range(seq - fired + 1, seq):
                        self._pending.pop(s, None)
                        if s in self._expired:
                            self._expired.discard(s)
                            self._lost -= 1
                return
            if tok[0] == "PONG" and len(tok) >= 5:
                n = int(tok[1]) if tok[1].isdigit() else -1
                t0 = self._ping_sent.pop(n, None)
                if t0 is not None:
                    self._rtt_ms = (now - t0) * 1000.0
                self._receiver_seq = int(tok[2]) if tok[2].isdigit() else None
                try:
                    self._rssi = int(tok[3])
                except ValueError:
                    self._rssi = None
                if len(tok) >= 6 and tok[5].isdigit():
                    self._receiver_fired = int(tok[5])
                ev = self._pong_waiters.get(n)
                if ev is not None:
                    ev.set()
                return
            if tok[0] == "ACK":      # "ACK RESET"
                return
            self._errors += 1
        self.get_logger().warn(f"receiver {addr[0]}: {line}")

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def _snapshot(self):
        with self._lock:
            alive = (
                self._last_rx is not None
                and time.time() - self._last_rx < self.alive_timeout
            )
            # The one number that says whether the GPR got every trace: what
            # the board fired versus what we sent since the last RESET. The
            # ack/lost counters below only describe the Wi-Fi round trip.
            unfired = (
                None if self._receiver_fired is None
                else max(0, self._sent - self._receiver_fired)
            )
            return {
                "alive": bool(alive),
                "receiver": f"{self.receiver[0]}:{self.receiver[1]}",
                "sent": self._sent,
                "receiver_fired": self._receiver_fired,
                "unfired": unfired,
                "seq": self._seq,
                "acked": self._acked,
                "pending": len(self._pending),
                "lost": self._lost,
                "caught_up": self._caught_up,
                "errors": self._errors,
                "rtt_ms": None if self._rtt_ms is None else round(self._rtt_ms, 1),
                "rssi_dbm": self._rssi,
                "receiver_seq": self._receiver_seq,
                "last_rx_age_s": (
                    None if self._last_rx is None else round(time.time() - self._last_rx, 2)
                ),
                "stamp": round(time.time(), 3),
            }

    def _publish_status(self):
        snap = self._snapshot()
        self.status_pub.publish(String(data=json.dumps(snap)))
        self.alive_pub.publish(Bool(data=snap["alive"]))

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _srv_ping(self, request, response):
        ev = threading.Event()
        with self._lock:
            self._ping_n += 1
            n = self._ping_n
            self._ping_sent[n] = time.time()
            self._pong_waiters[n] = ev
        self._send(f"PING {n}")
        ok = ev.wait(timeout=1.0)
        with self._lock:
            self._pong_waiters.pop(n, None)
        snap = self._snapshot()
        response.success = bool(ok)
        response.message = (
            f"PONG in {snap['rtt_ms']} ms, RSSI {snap['rssi_dbm']} dBm, "
            f"receiver seq {snap['receiver_seq']}"
            if ok else f"no PONG from {snap['receiver']} within 1 s"
        )
        return response

    def _srv_fire(self, request, response):
        seq = self._fire()
        # The ACK comes after the pulse routine returns, which for the
        # two-pulse patterns is 2*hold + 5 ms; wait for that plus the round trip.
        if int(self.get_parameter("mode").value) == 1:
            burst_s = self._pulses_per_scan() * 4 * int(self.get_parameter("half_period_us").value) / 1e6
            wait = max(1.0, burst_s + 0.5)
        else:
            hold_s = int(self.get_parameter("trigger_hold_ms").value) / 1000.0
            wait = max(1.0, 2.0 * hold_s + 0.5)
        deadline = time.time() + wait
        while time.time() < deadline:
            with self._lock:
                if seq not in self._pending:
                    response.success = True
                    response.message = f"TRIG {seq} acked"
                    return response
            time.sleep(0.02)
        response.success = False
        response.message = f"TRIG {seq} not acked within {wait:.1f} s"
        return response

    def _srv_reset(self, request, response):
        # Both ends restart their counters here, so sent vs receiver_fired
        # stays comparable after a reset mid-session.
        with self._lock:
            self._pending.clear()
            self._expired.clear()
            self._sent = 0
            self._receiver_fired = None
        response.success = self._send("RESET")
        response.message = "RESET sent" if response.success else "send failed"
        return response

    def destroy_node(self):
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GprTriggerBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
