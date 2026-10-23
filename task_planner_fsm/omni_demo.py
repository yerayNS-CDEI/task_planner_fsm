"""Omnidirectional base demo: a fixed choreography for expos.

The robot is taken to stand in the middle of a clear area (2 x 2 m by default).
It runs the selected behaviours in this order and returns to the start pose
between them:

    cross    forward and back, left and right, then a diagonal out to the
             forward-left corner and back (turret heading fixed)
    square   clockwise square centred on the start, ``square_laps`` laps, with
             a stop at every corner (turret heading fixed)
    figure8  figure-of-eight centred on the start, lobes to the left and right
             (turret heading fixed)
    orbit    back off ``orbit_radius``, circle the start clockwise with the
             turret always facing it, come back

It parks before the first behaviour, so every run starts from the same
chassis angle, and again after returning to the start at the end: the
chassis turns under the turret back into line (sim_controller's park_now, see
utils/chassis_parking.py). The start pose is taken after the first park.

HOW IT DRIVES
-------------
Every behaviour is a list of legs: densely sampled paths of the
``turret_footprint`` point (the point Nav2 steers and the costmap's
``robot_radius`` is centred on) with a turret heading per sample, all in a
local frame at the start pose (x forward, y left). A leg is followed with a
reference point that moves along it at the leg's speed profile, plus a P term
on the position error, sent to twist_mux's navigation channel (``/cmd_vel``,
the channel the FSM's fine correction uses). The reference only advances while
the robot keeps up, which matters here: the base is a diff-drive chassis under
a turret, and sim_controller scales translation down, keeping its direction,
while the chassis swings round to a new direction of travel (0.2 rad/s, about
8 s for a square corner). So a leg can start slowly, but it stays on its line.

Speed along curves is capped so the chassis can follow
(``chassis_rate_limit / curvature``) and so the turret's rotation stays inside
``turret_rate_limit`` (the orbit). Every leg starts and ends at rest.

Driving backwards (the chassis pulling), sim_controller's pull compensation
mirrors the sideways part of the command. A position servo's sideways
corrections then push the wrong way and the robot circles its target, so the
commands are passed through the inverse of that map first
(``undo_pull_compensation``, needs the ``base_footprint`` -> turret TF): the
turret point goes where it is sent and, in pull, the chassis turns round
under the turret instead.

SAFETY
------
* The arm must be folded (joint check against ``folded_fsm``/``folded``).
* Before moving, the local costmap must show no obstacle within
  ``robot_radius + safety_margin`` of anywhere the path goes.
* While moving, an obstacle within ``robot_radius + runtime_stop_margin`` of
  the robot pauses the demo until it has been clear for ``runtime_clear_s``.
* Joystick input (``/cmd_vel_joy``), a latched ``/emergency_stop`` true or a
  message on ``~/abort`` stops the robot and ends the demo. So does Ctrl+C.
  An aborted demo does not return or park: drive it back by hand.

RVIZ
----
``~/planned_path`` and ``~/trail`` (nav_msgs/Path, odom), ``~/markers``
(start pose, path box, required clear area, reference point, blocking
obstacles) and a static ``omni_demo_start`` frame at the start pose.
``rviz/omni_demo.rviz`` shows them all.

    ros2 run task_planner_fsm omni_demo
    ros2 run task_planner_fsm omni_demo --ros-args -p behaviours:="[square, orbit]" -p speed:=0.1
    ros2 run task_planner_fsm omni_demo --ros-args -p preflight_only:=true

Exit codes: 0 finished, 1 refused or failed, 2 aborted.
"""

import functools
import json
import math
import signal
import sys
import time
from dataclasses import dataclass, field

import numpy as np

BEHAVIOURS = ("cross", "square", "figure8", "orbit")
BEHAVIOUR_LABELS = {
    "cross": "Cross + diagonals",
    "square": "Square",
    "figure8": "Figure-8",
    "orbit": "Orbit",
}

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_ABORTED = 2

# Joint vectors the arm is considered folded at (arm_control's position_sender_node:
# folded_fsm, which ArmFolding sends, and the older 'folded').
ARM_JOINT_NAMES = (
    "arm_shoulder_pan_joint",
    "arm_shoulder_lift_joint",
    "arm_elbow_joint",
    "arm_wrist_1_joint",
    "arm_wrist_2_joint",
    "arm_wrist_3_joint",
)
FOLDED_JOINT_SETS = {
    "folded_fsm": (-1.273, -1.840, 2.028, 0.522, 1.251, -0.351),
    "folded": (-3.619, -1.298, -2.243, -1.177, 1.564, -4.373),
}

SAMPLE_STEP_M = 0.01


def wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

@dataclass
class Leg:
    """One stretch driven from rest to rest.

    ``xy`` (N, 2) and ``yaw`` (N,) are the turret_footprint position and the
    turret heading in the start frame, ``yaw`` unwrapped. ``profile()`` fills
    the arc length, tangents and the speed profile.
    """
    name: str
    xy: np.ndarray
    yaw: np.ndarray
    pause_after_s: float = 0.0
    s: np.ndarray = field(default=None, repr=False)
    tangent: np.ndarray = field(default=None, repr=False)
    dyaw_ds: np.ndarray = field(default=None, repr=False)
    curvature: np.ndarray = field(default=None, repr=False)
    v: np.ndarray = field(default=None, repr=False)

    @property
    def length(self):
        return float(self.s[-1]) if self.s is not None and len(self.s) else 0.0

    def point_at(self, s):
        if self.length < 1e-6:
            return self.xy[-1].copy()
        return np.array([np.interp(s, self.s, self.xy[:, 0]), np.interp(s, self.s, self.xy[:, 1])])

    def yaw_at(self, s):
        if self.length < 1e-6:
            return float(self.yaw[-1])
        return float(np.interp(s, self.s, self.yaw))

    def tangent_at(self, s):
        if self.length < 1e-6:
            return np.zeros(2)
        t = np.array([np.interp(s, self.s, self.tangent[:, 0]),
                      np.interp(s, self.s, self.tangent[:, 1])])
        n = np.linalg.norm(t)
        return t / n if n > 1e-9 else np.zeros(2)

    def dyaw_ds_at(self, s):
        if self.length < 1e-6:
            return 0.0
        return float(np.interp(s, self.s, self.dyaw_ds))

    def speed_at(self, s):
        if self.length < 1e-6:
            return 0.0
        return float(np.interp(s, self.s, self.v))

    def profile(self, speed, accel, chassis_rate_limit, turret_rate_limit):
        """Arc length, tangents, curvature and a rest-to-rest speed profile.

        The cap at each sample is the smallest of ``speed``, what the chassis
        can follow at the path's curvature, and what keeps the turret rotation
        under ``turret_rate_limit``; a forward and a backward pass then bound
        the acceleration from rest and the braking to rest.
        """
        xy = self.xy
        if len(xy) < 2:
            xy = np.vstack([xy, xy])
            self.xy = xy
            self.yaw = np.array([self.yaw[0], self.yaw[-1]])
        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        n = len(self.s)
        if self.length < 1e-6:
            self.tangent = np.zeros((n, 2))
            self.dyaw_ds = np.zeros(n)
            self.curvature = np.zeros(n)
            self.v = np.zeros(n)
            return self
        t = np.gradient(xy, self.s, axis=0)
        norms = np.linalg.norm(t, axis=1)
        norms[norms < 1e-9] = 1.0
        self.tangent = t / norms[:, None]
        heading = np.unwrap(np.arctan2(self.tangent[:, 1], self.tangent[:, 0]))
        self.curvature = np.abs(np.gradient(heading, self.s)) if n > 2 else np.zeros(n)
        self.dyaw_ds = np.gradient(self.yaw, self.s) if n > 2 else np.zeros(n)

        cap = np.full(n, float(speed))
        with np.errstate(divide="ignore"):
            if chassis_rate_limit > 0.0:
                cap = np.minimum(cap, np.where(self.curvature > 1e-6,
                                               chassis_rate_limit / self.curvature, np.inf))
            if turret_rate_limit > 0.0:
                cap = np.minimum(cap, np.where(np.abs(self.dyaw_ds) > 1e-6,
                                               turret_rate_limit / np.abs(self.dyaw_ds), np.inf))
        v = cap.copy()
        v[0] = 0.0
        ds = np.diff(self.s)
        for i in range(1, n):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * accel * ds[i - 1]))
        v[-1] = 0.0
        for i in range(n - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * accel * ds[i]))
        self.v = v
        return self

    def nominal_duration(self, min_speed):
        """Time to drive the profile, each sample at least at ``min_speed``."""
        if self.length < 1e-6:
            return 0.0
        v = np.maximum(self.v, min_speed)
        vm = 0.5 * (v[1:] + v[:-1])
        return float(np.sum(np.diff(self.s) / vm))


def _resample(xy, yaw, step=SAMPLE_STEP_M):
    """Resample a polyline (and its headings) at a uniform arc-length step."""
    xy = np.asarray(xy, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-9:
        return xy[[0, -1]], yaw[[0, -1]]
    n = max(2, int(math.ceil(s[-1] / step)) + 1)
    su = np.linspace(0.0, s[-1], n)
    out = np.column_stack([np.interp(su, s, xy[:, 0]), np.interp(su, s, xy[:, 1])])
    return out, np.interp(su, s, yaw)


def line_leg(name, p0, p1, yaw0=0.0, yaw1=None, pause_after_s=0.0):
    """Straight leg from ``p0`` to ``p1``; the heading ramps from yaw0 to yaw1."""
    yaw1 = yaw0 if yaw1 is None else yaw1
    xy, yaw = _resample(np.array([p0, p1], dtype=float), [yaw0, yaw1])
    if np.linalg.norm(np.subtract(p1, p0)) < 1e-9:
        xy = np.array([p0, p1], dtype=float)
        yaw = np.array([yaw0, yaw1], dtype=float)
    return Leg(name, xy, yaw, pause_after_s)


def build_cross(distance, pause_s):
    """Forward/back, left/right, forward-left diagonal out and back."""
    d = float(distance)
    o = (0.0, 0.0)
    return [
        line_leg("forward", o, (d, 0.0), pause_after_s=pause_s),
        line_leg("backward", (d, 0.0), o, pause_after_s=pause_s),
        line_leg("left", o, (0.0, d), pause_after_s=pause_s),
        line_leg("right", (0.0, d), o, pause_after_s=pause_s),
        line_leg("diagonal forward-left", o, (d, d), pause_after_s=pause_s),
        line_leg("diagonal backward-right", (d, d), o, pause_after_s=pause_s),
    ]


def build_square(side, laps, pause_s):
    """Clockwise square centred on the start, entered and left along the
    diagonal to its forward-left corner. Clockwise seen from above with x
    forward: forward-left -> forward-right -> back-right -> back-left."""
    h = 0.5 * float(side)
    corners = [(h, h), (h, -h), (-h, -h), (-h, h)]
    names = ["front side", "right side", "back side", "left side"]
    legs = [line_leg("to forward-left corner", (0.0, 0.0), corners[0], pause_after_s=pause_s)]
    for lap in range(int(laps)):
        for i in range(4):
            legs.append(line_leg(f"lap {lap + 1}: {names[i]}", corners[i], corners[(i + 1) % 4],
                                 pause_after_s=pause_s))
    legs.append(line_leg("back to centre", corners[0], (0.0, 0.0), pause_after_s=pause_s))
    return legs


def build_figure8(length, width, laps, samples_per_lap=2000):
    """Figure-of-eight through the start: y = L/2 sin t, x = W/2 sin 2t.

    Lobes to the left and right (``length`` across them), ``width`` along the
    robot's forward axis. It leaves the start towards forward-left and goes
    round the left lobe counter-clockwise, then the right lobe clockwise.
    """
    laps = max(1, int(laps))
    t = np.linspace(0.0, 2.0 * math.pi * laps, samples_per_lap * laps + 1)
    xy = np.column_stack([0.5 * width * np.sin(2.0 * t), 0.5 * length * np.sin(t)])
    xy, yaw = _resample(xy, np.zeros(len(t)))
    return [Leg(f"figure-8 ({laps} lap{'s' if laps > 1 else ''})", xy, yaw)]


def build_orbit(radius, laps, pause_s, samples_per_lap=2000):
    """Back off ``radius`` (the turret already faces the start), circle the
    start clockwise with the turret facing it, then drive back in."""
    r = float(radius)
    laps = max(1, int(laps))
    theta = np.linspace(math.pi, math.pi - 2.0 * math.pi * laps, samples_per_lap * laps + 1)
    xy = np.column_stack([r * np.cos(theta), r * np.sin(theta)])
    # Facing the centre: heading = theta + pi, which is 0 (the start heading) at theta = pi.
    xy, yaw = _resample(xy, theta - math.pi)
    end_yaw = float(yaw[-1])
    return [
        line_leg("back off to the orbit", (0.0, 0.0), (-r, 0.0), pause_after_s=pause_s),
        Leg(f"orbit ({laps} lap{'s' if laps > 1 else ''})", xy, yaw, pause_after_s=pause_s),
        line_leg("back to centre", (-r, 0.0), (0.0, 0.0), yaw0=end_yaw, pause_after_s=pause_s),
    ]


@dataclass
class DemoConfig:
    behaviours: tuple = ("cross", "square")
    speed: float = 0.15
    accel: float = 0.15
    move_distance: float = 0.35
    square_side: float = 0.70
    square_laps: int = 2
    figure8_length: float = 0.70
    figure8_width: float = 0.50
    figure8_laps: int = 1
    orbit_radius: float = 0.35
    orbit_laps: int = 1
    corner_pause_s: float = 0.5
    behaviour_pause_s: float = 2.0
    chassis_rate_limit: float = 0.18
    turret_rate_limit: float = 0.30
    robot_radius: float = 0.60
    safety_margin: float = 0.05


def normalise_behaviours(names):
    """Validated behaviour list in the fixed demo order (duplicates dropped)."""
    wanted = {str(n).strip().lower().replace("-", "").replace("_", "") for n in names}
    wanted.discard("")
    aliases = {"figure8": "figure8", "eight": "figure8", "8": "figure8"}
    wanted = {aliases.get(n, n) for n in wanted}
    unknown = wanted - set(BEHAVIOURS)
    if unknown:
        raise ValueError(f"unknown behaviour(s) {sorted(unknown)}; choose from {list(BEHAVIOURS)}")
    return tuple(b for b in BEHAVIOURS if b in wanted)


def build_behaviour(name, cfg):
    if name == "cross":
        return build_cross(cfg.move_distance, cfg.corner_pause_s)
    if name == "square":
        return build_square(cfg.square_side, cfg.square_laps, cfg.corner_pause_s)
    if name == "figure8":
        return build_figure8(cfg.figure8_length, cfg.figure8_width, cfg.figure8_laps)
    if name == "orbit":
        return build_orbit(cfg.orbit_radius, cfg.orbit_laps, cfg.corner_pause_s)
    raise ValueError(name)


def build_plan(cfg):
    """[(behaviour, [profiled legs])] in demo order."""
    plan = []
    for name in normalise_behaviours(cfg.behaviours):
        legs = [leg.profile(cfg.speed, cfg.accel, cfg.chassis_rate_limit, cfg.turret_rate_limit)
                for leg in build_behaviour(name, cfg)]
        plan.append((name, legs))
    return plan


def plan_box(plan):
    """Axis-aligned box (xmin, xmax, ymin, ymax) of every path point and the start."""
    pts = [np.zeros((1, 2))] + [leg.xy for _, legs in plan for leg in legs]
    pts = np.vstack(pts)
    return (float(pts[:, 0].min()), float(pts[:, 0].max()),
            float(pts[:, 1].min()), float(pts[:, 1].max()))


def required_area(cfg):
    """(depth, width) in metres of the clear floor the demo needs: the path box
    grown by the robot radius plus the safety margin on every side."""
    xmin, xmax, ymin, ymax = plan_box(build_plan(cfg))
    grow = 2.0 * (cfg.robot_radius + cfg.safety_margin)
    return xmax - xmin + grow, ymax - ymin + grow


def plan_duration(plan, min_speed=0.03):
    """Rough running time in seconds (pauses included, chassis swings not)."""
    total = 0.0
    for _, legs in plan:
        for leg in legs:
            total += leg.nominal_duration(min_speed) + leg.pause_after_s
    return total


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def box_distance(points, box):
    """Distance from each (N, 2) point to the box (0 inside it)."""
    xmin, xmax, ymin, ymax = box
    p = np.atleast_2d(points)
    dx = np.maximum(np.maximum(xmin - p[:, 0], 0.0), p[:, 0] - xmax)
    dy = np.maximum(np.maximum(ymin - p[:, 1], 0.0), p[:, 1] - ymax)
    return np.hypot(dx, dy)


def costmap_obstacles(info, data, threshold):
    """Cell centres (N, 2), in the costmap's frame, of cells at or above
    ``threshold`` (OccupancyGrid values: 100 lethal, 99 inscribed, -1 unknown)."""
    grid = np.asarray(data, dtype=np.int16).reshape(info.height, info.width)
    rows, cols = np.nonzero(grid >= threshold)
    if rows.size == 0:
        return np.zeros((0, 2))
    res = info.resolution
    lx = (cols + 0.5) * res
    ly = (rows + 0.5) * res
    o = info.origin
    oyaw = 2.0 * math.atan2(o.orientation.z, o.orientation.w)
    c, s = math.cos(oyaw), math.sin(oyaw)
    return np.column_stack([o.position.x + c * lx - s * ly, o.position.y + s * lx + c * ly])


def costmap_bounds(info):
    """Axis-aligned box of the costmap in its frame (assumes no origin rotation)."""
    x0, y0 = info.origin.position.x, info.origin.position.y
    return (x0, x0 + info.width * info.resolution, y0, y0 + info.height * info.resolution)


def to_local(points, origin):
    """World (N, 2) points into the frame at origin = (x, y, yaw)."""
    x0, y0, yaw0 = origin
    c, s = math.cos(yaw0), math.sin(yaw0)
    p = np.atleast_2d(points) - np.array([x0, y0])
    return np.column_stack([c * p[:, 0] + s * p[:, 1], -s * p[:, 0] + c * p[:, 1]])


def to_world(points, origin):
    x0, y0, yaw0 = origin
    c, s = math.cos(yaw0), math.sin(yaw0)
    p = np.atleast_2d(points)
    return np.column_stack([x0 + c * p[:, 0] - s * p[:, 1], y0 + s * p[:, 0] + c * p[:, 1]])


def clearance_check(obstacles_local, box, clearance):
    """Obstacles closer than ``clearance`` to the path box.

    Returns (blocking (M, 2), nearest distance or inf).
    """
    if len(obstacles_local) == 0:
        return np.zeros((0, 2)), math.inf
    d = box_distance(obstacles_local, box)
    return obstacles_local[d < clearance], float(d.min())


def arm_fold_error(positions, references=FOLDED_JOINT_SETS, joint_names=ARM_JOINT_NAMES):
    """(name, largest joint error in rad) of the closest folded reference.

    Joint differences are taken modulo 2 pi. ``positions`` maps joint name to
    position; a missing joint returns (None, inf).
    """
    if any(j not in positions for j in joint_names):
        return None, math.inf
    q = np.array([positions[j] for j in joint_names])
    best = (None, math.inf)
    for name, ref in references.items():
        err = float(np.max(np.abs(np.arctan2(np.sin(q - ref), np.cos(q - ref)))))
        if err < best[1]:
            best = (name, err)
    return best


# ---------------------------------------------------------------------------
# sim_controller's pull compensation
# ---------------------------------------------------------------------------

@dataclass
class PullCompensation:
    """sim_controller's pull compensation parameters (Sim_odometry::applyPullCompensation)."""
    center_distance: float = 0.167   # d1, turret axis to wheel axle
    min_speed: float = 0.02          # pull_compensation_min_speed
    deadband: float = 0.05           # pull_compensation_deadband


def pull_compensate(u, thetap_dot, pc):
    """What sim_controller does to chassis-frame velocities ``u`` (N, 2) in pull
    (u_x < 0): blend each towards its reflection about the axis at gamma."""
    u = np.atleast_2d(np.asarray(u, dtype=float))
    x, y = u[:, 0], u[:, 1]
    speed = np.hypot(x, y)
    blend = np.ones(len(u))
    if pc.deadband > 0.0:
        blend *= np.clip(-x / pc.deadband, 0.0, 1.0)
    if pc.min_speed > 0.0:
        blend *= np.clip((speed - pc.min_speed) / pc.min_speed, 0.0, 1.0)
    blend[(x >= 0.0) | (speed <= 0.0)] = 0.0
    with np.errstate(divide="ignore", invalid="ignore"):
        nc = np.clip(np.where(speed > 0.0, pc.center_distance * thetap_dot / speed, 0.0), -1.0, 1.0)
    g2 = 2.0 * (math.pi - np.arcsin(nc))
    cx = np.cos(g2) * x + np.sin(g2) * y
    cy = np.sin(g2) * x - np.cos(g2) * y
    return np.column_stack([(1.0 - blend) * x + blend * cx, (1.0 - blend) * y + blend * cy])


def undo_pull_compensation(v_turret, thetap_dot, phi, pc, prev=None, max_gain=1.1, samples=601):
    """Turret-frame command that sim_controller's pull compensation turns into
    ``v_turret``.

    In pull the controller mirrors the lateral part of the command (so a
    chassis driving backwards straightens rather than turning round). Fine for
    a joystick, but it flips the sign of every sideways correction a position
    servo makes, and the robot circles its target instead of reaching it. This
    inverts that map so the turret point moves as asked; in pull the chassis
    then turns round under the turret, the natural motion of the base.

    ``phi`` is the turret angle relative to the chassis. The reflection keeps
    the component along its axis, so the pre-image is searched along the
    perpendicular. Several can be (near) exact; the one closest to ``prev``, the
    last command sent (turret frame), is taken so the command does not jump
    between them from one tick to the next (the controller's acceleration
    limiter would blend such a jump into a wrong motion). Without ``prev``, the
    one closest to the mirrored command. Where the blend is close to 0.5 the
    controller collapses the sideways part and no pre-image exists; the best
    one no larger than ``max_gain`` times the request is returned.
    """
    c, s = math.cos(phi), math.sin(phi)
    d = np.array([c * v_turret[0] - s * v_turret[1], s * v_turret[0] + c * v_turret[1]])
    speed = float(np.hypot(*d))
    if d[0] >= 0.0 or speed <= 0.0:
        return np.asarray(v_turret, dtype=float).copy()
    nc = min(1.0, max(-1.0, pc.center_distance * thetap_dot / speed))
    g = math.pi - math.asin(nc)
    axis = np.array([math.cos(g), math.sin(g)])
    perp = np.array([-math.sin(g), math.cos(g)])
    da, dp = float(axis @ d), float(perp @ d)
    # Bounded: near the singular blend the exact pre-image grows without limit,
    # and a command the controller then clips is worse than an inexact one.
    lim = max_gain * speed
    span = math.sqrt(max(0.0, lim * lim - da * da))
    cand = np.concatenate([np.linspace(-span, span, samples), [dp, -dp]])
    cand = cand[np.abs(cand) <= span + 1e-12]
    u = da * axis + cand[:, None] * perp
    res = np.linalg.norm(pull_compensate(u, thetap_dot, pc) - d, axis=1)
    ok = res <= res.min() + 1e-3
    if prev is None:
        ref = da * axis - dp * perp
    else:
        ref = np.array([c * prev[0] - s * prev[1], s * prev[0] + c * prev[1]])
    best = u[ok][int(np.argmin(np.linalg.norm(u[ok] - ref, axis=1)))]
    return np.array([c * best[0] + s * best[1], -s * best[0] + c * best[1]])


# ---------------------------------------------------------------------------
# Leg tracking
# ---------------------------------------------------------------------------

@dataclass
class TrackerGains:
    kp_pos: float = 1.2           # 1/s on the position error
    kp_yaw: float = 1.0           # 1/s on the heading error
    track_window: float = 0.10    # m; the reference stops advancing at this error
    min_progress_speed: float = 0.03   # m/s; reference crawl from/to rest
    pos_tol: float = 0.03
    yaw_tol: float = 0.05
    settle_timeout_s: float = 8.0  # after the reference reaches the end
    accept_pos_tol: float = 0.08   # ... after which this much error is accepted
    accept_yaw_tol: float = 0.12
    max_tracking_error: float = 0.40   # m; beyond this the leg fails
    final_deadband: float = 0.008  # m/s; smaller end corrections are not sent


class LegTracker:
    """Follows one Leg. ``step`` returns the start-frame command and a status:
    'running', 'done', 'accepted' (settled within the looser tolerance after
    ``settle_timeout_s``) or 'failed'."""

    def __init__(self, leg, max_speed, accel, turret_rate_limit, gains=None):
        self.leg = leg
        self.max_speed = float(max_speed)
        self.accel = float(accel)
        self.turret_rate_limit = float(turret_rate_limit)
        self.g = gains or TrackerGains()
        self.s_ref = 0.0
        self.end_reached_t = None
        self.error = 0.0
        self.yaw_error = 0.0
        self.message = ""

    @property
    def progress(self):
        return 1.0 if self.leg.length < 1e-6 else self.s_ref / self.leg.length

    def reference(self):
        return self.leg.point_at(self.s_ref), self.leg.yaw_at(self.s_ref)

    def step(self, pos, yaw, dt, now):
        g, leg = self.g, self.leg
        pos = np.asarray(pos, dtype=float)
        length = leg.length

        ref = leg.point_at(self.s_ref)
        err = float(np.linalg.norm(ref - pos))
        gate = min(1.0, max(0.0, 1.0 - err / g.track_window))
        at_end = self.s_ref >= length - 1e-9
        v_adv = 0.0 if at_end else max(leg.speed_at(self.s_ref), g.min_progress_speed)
        if not at_end:
            self.s_ref = min(length, self.s_ref + v_adv * gate * dt)
            at_end = self.s_ref >= length - 1e-9

        ref = leg.point_at(self.s_ref)
        e_vec = ref - pos
        self.error = float(np.linalg.norm(e_vec))
        yaw_ref = leg.yaw_at(self.s_ref)
        self.yaw_error = wrap(yaw_ref - yaw)

        if self.error > g.max_tracking_error:
            self.message = (f"tracking error {self.error:.2f} m > {g.max_tracking_error:.2f} m "
                            f"on '{leg.name}'")
            return np.zeros(2), 0.0, "failed"

        v = leg.tangent_at(self.s_ref) * (0.0 if at_end else v_adv * gate) + g.kp_pos * e_vec
        cap = self.max_speed
        if at_end:
            cap = min(cap, math.sqrt(2.0 * self.accel * self.error))
        n = float(np.linalg.norm(v))
        if n > cap:
            v = v * (cap / n) if n > 1e-9 else v
        wz = (0.0 if at_end else leg.dyaw_ds_at(self.s_ref) * v_adv * gate) + g.kp_yaw * self.yaw_error
        wz = max(-self.turret_rate_limit, min(self.turret_rate_limit, wz))

        if not at_end:
            return v, wz, "running"

        if self.end_reached_t is None:
            self.end_reached_t = now
        if self.error <= g.pos_tol and abs(self.yaw_error) <= g.yaw_tol:
            return np.zeros(2), 0.0, "done"
        if now - self.end_reached_t > g.settle_timeout_s:
            if self.error <= g.accept_pos_tol and abs(self.yaw_error) <= g.accept_yaw_tol:
                self.message = (f"'{leg.name}' settled {self.error * 100:.1f} cm / "
                                f"{math.degrees(self.yaw_error):.1f} deg off; carrying on")
                return np.zeros(2), 0.0, "accepted"
            self.message = (f"'{leg.name}' did not settle: {self.error * 100:.1f} cm / "
                            f"{math.degrees(self.yaw_error):.1f} deg off after "
                            f"{g.settle_timeout_s:.0f} s")
            return np.zeros(2), 0.0, "failed"
        if np.linalg.norm(v) < g.final_deadband:
            v = np.zeros(2)
        if abs(self.yaw_error) <= g.yaw_tol:
            wz = 0.0
        return v, wz, "running"


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

def main(args=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # Own SIGINT/SIGTERM handling: the robot gets a zero twist before the
    # context goes away (rclpy's default handler shuts it down first).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = _node_class()()

    def _on_signal(signum, _frame):
        node.request_abort(f"signal {signal.Signals(signum).name}")

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.stop_robot(repeat=3)
        code = EXIT_FAILED if node.exit_code is None else node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


@functools.lru_cache(maxsize=None)
def _node_class():
    """OmniDemoNode, built on first use so the helpers above import without ROS
    (the tests and the UI's area estimate)."""
    from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, ColorRGBA, Empty, String
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    from task_planner_fsm.utils.chassis_parking import ChassisParker

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def quat(yaw):
        return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)

    class OmniDemoNode(Node):
        TICK_HZ = 20.0

        def __init__(self):
            super().__init__("omni_demo")
            p = self.declare_parameter
            d = DemoConfig()
            p("behaviours", list(d.behaviours))
            p("speed", d.speed)
            p("accel", d.accel)
            p("move_distance", d.move_distance)
            p("square_side", d.square_side)
            p("square_laps", d.square_laps)
            p("figure8_length", d.figure8_length)
            p("figure8_width", d.figure8_width)
            p("figure8_laps", d.figure8_laps)
            p("orbit_radius", d.orbit_radius)
            p("orbit_laps", d.orbit_laps)
            p("corner_pause_s", d.corner_pause_s)
            p("behaviour_pause_s", d.behaviour_pause_s)
            p("chassis_rate_limit", d.chassis_rate_limit)
            p("turret_rate_limit", d.turret_rate_limit)
            p("robot_radius", d.robot_radius)
            p("safety_margin", d.safety_margin)
            p("check_arm_folded", True)
            p("arm_fold_tolerance", 0.20)
            p("check_clearance", True)
            p("runtime_guard", True)
            p("runtime_stop_margin", 0.15)
            p("runtime_clear_s", 1.5)
            p("costmap_threshold", 100)
            p("park_at_start", True)
            p("park_at_end", True)
            p("undo_pull_compensation", True)
            p("pull_center_distance", PullCompensation.center_distance)
            p("pull_min_speed", PullCompensation.min_speed)
            p("pull_deadband", PullCompensation.deadband)
            p("preflight_only", False)
            p("odom_frame", "odom")
            p("robot_frame", "turret_footprint")
            p("chassis_frame", "base_footprint")
            p("cmd_vel_topic", "/cmd_vel")
            p("joy_topic", "/cmd_vel_joy")
            p("estop_topic", "/emergency_stop")
            p("costmap_topic", "/local_costmap/costmap")
            p("joint_states_topic", "/joint_states")
            p("startup_timeout_s", 10.0)

            gp = lambda name: self.get_parameter(name).value  # noqa: E731
            self.cfg = DemoConfig(
                behaviours=tuple(gp("behaviours")),
                speed=min(0.2, max(0.02, float(gp("speed")))),
                accel=max(0.02, float(gp("accel"))),
                move_distance=float(gp("move_distance")),
                square_side=float(gp("square_side")),
                square_laps=max(1, int(gp("square_laps"))),
                figure8_length=float(gp("figure8_length")),
                figure8_width=float(gp("figure8_width")),
                figure8_laps=max(1, int(gp("figure8_laps"))),
                orbit_radius=float(gp("orbit_radius")),
                orbit_laps=max(1, int(gp("orbit_laps"))),
                corner_pause_s=max(0.0, float(gp("corner_pause_s"))),
                behaviour_pause_s=max(0.0, float(gp("behaviour_pause_s"))),
                chassis_rate_limit=float(gp("chassis_rate_limit")),
                turret_rate_limit=float(gp("turret_rate_limit")),
                robot_radius=float(gp("robot_radius")),
                safety_margin=float(gp("safety_margin")),
            )
            self.odom_frame = str(gp("odom_frame"))
            self.robot_frame = str(gp("robot_frame"))
            self.chassis_frame = str(gp("chassis_frame"))
            self.pull = PullCompensation(float(gp("pull_center_distance")), float(gp("pull_min_speed")),
                                         float(gp("pull_deadband"))) \
                if bool(gp("undo_pull_compensation")) else None
            self._warned_chassis_tf = False
            self.last_cmd = np.zeros(2)    # turret-frame linear command last sent
            self.initial_parked = not bool(gp("park_at_start"))
            self.exit_code = None
            self.gains = TrackerGains()

            self.cmd_pub = self.create_publisher(Twist, str(gp("cmd_vel_topic")), 10)
            self.status_pub = self.create_publisher(String, "~/status", latched)
            self.plan_pub = self.create_publisher(Path, "~/planned_path", latched)
            self.trail_pub = self.create_publisher(Path, "~/trail", 10)
            self.marker_pub = self.create_publisher(MarkerArray, "~/markers", latched)
            self.static_tf = StaticTransformBroadcaster(self)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.joint_positions = {}
            self.costmap = None
            self.estop = False
            self.parking_active = None
            self.create_subscription(JointState, str(gp("joint_states_topic")), self._on_joints, 10)
            self.create_subscription(OccupancyGrid, str(gp("costmap_topic")), self._on_costmap, latched)
            self.create_subscription(Twist, str(gp("joy_topic")), self._on_joy, 10)
            self.create_subscription(Bool, str(gp("estop_topic")), self._on_estop, latched)
            self.create_subscription(Empty, "~/abort", lambda _m: self.request_abort("abort requested"), 10)
            self.create_subscription(Bool, "/sim_controller/parking_active", self._on_parking_active,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

            try:
                self.plan = build_plan(self.cfg)
            except ValueError as e:
                self._finish(EXIT_FAILED, f"Bad parameters: {e}")
                return
            if not self.plan:
                self._finish(EXIT_FAILED, "No behaviour selected (parameter 'behaviours').")
                return
            self.box = plan_box(self.plan)
            depth, width = required_area(self.cfg)
            names = ", ".join(BEHAVIOUR_LABELS[b] for b, _ in self.plan)
            self.get_logger().info(
                f"Omni demo: {names}. Speed {self.cfg.speed:.2f} m/s, about "
                f"{plan_duration(self.plan) / 60.0:.1f} min plus chassis swings. Needs a clear area "
                f"of {depth:.2f} m (forward) x {width:.2f} m (sideways) centred on the robot.")

            self.steps = self._build_steps()
            self.step_index = -1
            self.phase = "wait_tf"
            self.phase_start = time.monotonic()
            self.start = None          # (x, y, yaw) in odom
            self.tracker = None
            self.pause_until = None
            self.guard_blocked_since = None
            self.guard_clear_since = None
            self.guard_paused = False
            self.parker = ChassisParker("OmniDemo", "omni_demo")
            self.park_ctx = None
            self.trail = Path()
            self.trail.header.frame_id = self.odom_frame
            self.last_trail_t = 0.0
            self.last_tick = time.monotonic()
            self.blocking_world = np.zeros((0, 2))
            self.timer = self.create_timer(1.0 / self.TICK_HZ, self._tick)
            self._status("Waiting for the robot pose")

        # ---------------- inputs ----------------

        def _on_joints(self, msg):
            for name, pos in zip(msg.name, msg.position):
                self.joint_positions[name] = pos

        def _on_costmap(self, msg):
            self.costmap = msg

        def _on_joy(self, msg):
            if self.phase in ("run", "pause", "return", "park", "park_start") and (
                    abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3):
                self.request_abort("joystick took over")

        def _on_estop(self, msg):
            self.estop = bool(msg.data)
            if self.estop and self.phase not in ("wait_tf", "done"):
                self.request_abort("emergency stop active")

        def _on_parking_active(self, msg):
            self.parking_active = bool(msg.data)
            if self.park_ctx is not None:
                self.park_ctx["parking_active"] = self.parking_active

        def request_abort(self, reason):
            if self.exit_code is not None:
                return
            self.stop_robot(repeat=3)
            msg = f"ABORTED ({reason}). The robot was stopped where it is; it does not return or park."
            if self.phase in ("park", "park_start"):
                msg += " Parking was already running inside the controller and finishes on its own."
            self._finish(EXIT_ABORTED, msg, level="warn")

        # ---------------- helpers ----------------

        def stop_robot(self, repeat=1):
            self.last_cmd = np.zeros(2)
            for _ in range(repeat):
                try:
                    self.cmd_pub.publish(Twist())
                except Exception:
                    return

        def _finish(self, code, message, level="info"):
            if self.exit_code is not None:
                return
            {"info": self.get_logger().info, "warn": self.get_logger().warn,
             "error": self.get_logger().error}[level](message)
            self._status(message, phase={EXIT_OK: "done", EXIT_ABORTED: "aborted"}.get(code, "failed"))
            self.exit_code = code

        def _status(self, message, phase=None):
            step = None
            if 0 <= getattr(self, "step_index", -1) < len(getattr(self, "steps", [])):
                step = self.steps[self.step_index]
            payload = {
                "phase": phase or getattr(self, "phase", "init"),
                "behaviour": BEHAVIOUR_LABELS.get(step[1], "") if step else "",
                "step": (self.step_index + 1) if step else 0,
                "steps": len(getattr(self, "steps", [])),
                "message": message,
            }
            self.status_pub.publish(String(data=json.dumps(payload)))

        def _pose(self):
            try:
                tf = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, Time(),
                                                     timeout=Duration(seconds=0.0))
            except TransformException:
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            return t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        def _frame_pose_2d(self, target, source):
            try:
                tf = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.0))
            except TransformException:
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            return t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        def _turret_angle(self):
            """Turret heading relative to the chassis (sim_controller's phi), or None."""
            tf = self._frame_pose_2d(self.chassis_frame, self.robot_frame)
            if tf is None:
                if not self._warned_chassis_tf:
                    self._warned_chassis_tf = True
                    self.get_logger().warn(
                        f"No transform {self.chassis_frame} -> {self.robot_frame}: driving without "
                        "undoing the controller's pull compensation (sideways corrections while "
                        "reversing may circle).")
                return None
            return tf[2]

        def _local_pose(self):
            pose = self._pose()
            if pose is None:
                return None
            p = to_local([pose[:2]], self.start)[0]
            return p, wrap(pose[2] - self.start[2])

        def _build_steps(self):
            """[(kind, behaviour, payload)]: 'leg' (Leg), 'return', 'pause' (s), 'park'."""
            steps = []
            for i, (name, legs) in enumerate(self.plan):
                steps.extend(("leg", name, leg) for leg in legs)
                steps.append(("return", name, None))
                if i < len(self.plan) - 1 and self.cfg.behaviour_pause_s > 0.0:
                    steps.append(("pause", name, self.cfg.behaviour_pause_s))
            if bool(self.get_parameter("park_at_end").value):
                steps.append(("park", self.plan[-1][0], None))
            return steps

        # ---------------- main loop ----------------

        def _tick(self):
            if self.exit_code is not None:
                return
            now = time.monotonic()
            dt = min(0.2, max(1e-3, now - self.last_tick))
            self.last_tick = now
            try:
                getattr(self, f"_tick_{self.phase}")(now, dt)
            except Exception as e:  # never leave the base driving on a bug
                self.stop_robot(repeat=3)
                self._finish(EXIT_FAILED, f"Internal error in phase '{self.phase}': {e!r}", level="error")

        def _set_phase(self, phase, message=None):
            self.phase = phase
            self.phase_start = time.monotonic()
            if message:
                self.get_logger().info(message)
                self._status(message)

        def _tick_wait_tf(self, now, _dt):
            pose = self._pose()
            if pose is None:
                if now - self.phase_start > float(self.get_parameter("startup_timeout_s").value):
                    self._finish(EXIT_FAILED, f"No transform {self.odom_frame} -> {self.robot_frame}: "
                                 "is the robot stack running?", level="error")
                return
            if self.estop:
                self._finish(EXIT_FAILED, "Emergency stop is active; release it first.", level="error")
                return
            self.start = pose
            self.get_logger().info(
                f"Start pose ({self.odom_frame}): x {pose[0]:.2f} m, y {pose[1]:.2f} m, "
                f"heading {math.degrees(pose[2]):.0f} deg.")
            self._publish_start_frame()
            self._publish_plan()
            self._publish_markers()
            self._set_phase("check_arm", "Checking the arm is folded")

        def _tick_check_arm(self, now, _dt):
            if not bool(self.get_parameter("check_arm_folded").value):
                self.get_logger().warn("Arm fold check disabled.")
                self._set_phase("check_clearance", "Checking the area is clear")
                return
            ref, err = arm_fold_error(self.joint_positions)
            if ref is None:
                if now - self.phase_start > float(self.get_parameter("startup_timeout_s").value):
                    missing = [j for j in ARM_JOINT_NAMES if j not in self.joint_positions]
                    self._finish(EXIT_FAILED, f"No arm joint states ({', '.join(missing)} missing); "
                                 "cannot confirm the arm is folded. Untick 'Require folded arm' "
                                 "for a base-only stack.", level="error")
                return
            tol = float(self.get_parameter("arm_fold_tolerance").value)
            if err > tol:
                q = ", ".join(f"{self.joint_positions[j]:.2f}" for j in ARM_JOINT_NAMES)
                self._finish(EXIT_FAILED, f"The arm is not folded: closest folded pose '{ref}' is "
                             f"{math.degrees(err):.0f} deg away on its worst joint (tolerance "
                             f"{math.degrees(tol):.0f} deg). Joints: [{q}]. Fold it (folded_fsm) "
                             "and start again.", level="error")
                return
            self.get_logger().info(f"Arm folded ('{ref}', worst joint {math.degrees(err):.1f} deg off).")
            self._set_phase("check_clearance", "Checking the area is clear")

        def _obstacles_local(self):
            """(obstacles in the start frame, costmap) or (None, reason)."""
            cm = self.costmap
            if cm is None:
                return None, "no costmap"
            thr = int(self.get_parameter("costmap_threshold").value)
            pts = costmap_obstacles(cm.info, cm.data, thr)
            frame = cm.header.frame_id or self.odom_frame
            if frame != self.odom_frame and len(pts):
                tf = self._frame_pose_2d(self.odom_frame, frame)
                if tf is None:
                    return None, f"no transform {self.odom_frame} -> {frame}"
                pts = to_world(pts, tf)
            return to_local(pts, self.start), cm

        def _tick_check_clearance(self, now, _dt):
            self._record_trail(now)
            if not bool(self.get_parameter("check_clearance").value):
                self.get_logger().warn("Clearance check disabled: nothing checks the area before moving.")
                self._ready_to_run()
                return
            obstacles, cm = self._obstacles_local()
            if obstacles is None:
                if now - self.phase_start > float(self.get_parameter("startup_timeout_s").value):
                    self._finish(EXIT_FAILED, f"Clearance check impossible ({cm}) on "
                                 f"'{self.get_parameter('costmap_topic').value}': is Nav2 running? "
                                 "Untick 'Pre-start clearance check' to skip it.", level="error")
                return
            clearance = self.cfg.robot_radius + self.cfg.safety_margin
            # The costmap is a rolling window: an area that sticks out of it is not seen.
            frame = cm.header.frame_id or self.odom_frame
            corners = np.array([[self.box[0] - clearance, self.box[2] - clearance],
                                [self.box[0] - clearance, self.box[3] + clearance],
                                [self.box[1] + clearance, self.box[2] - clearance],
                                [self.box[1] + clearance, self.box[3] + clearance]])
            if frame == self.odom_frame:
                cw = to_world(corners, self.start)
                bx = costmap_bounds(cm.info)
                if (cw[:, 0].min() < bx[0] or cw[:, 0].max() > bx[1]
                        or cw[:, 1].min() < bx[2] or cw[:, 1].max() > bx[3]):
                    self.get_logger().warn("Part of the demo area lies outside the local costmap "
                                           "window and is not checked.")
            blocking, nearest = clearance_check(obstacles, self.box, clearance)
            self.blocking_world = to_world(blocking, self.start) if len(blocking) else np.zeros((0, 2))
            self._publish_markers()
            if len(blocking):
                d = box_distance(blocking, self.box)
                worst = blocking[int(np.argmin(d))]
                self._finish(EXIT_FAILED, (
                    f"Area NOT clear: {len(blocking)} obstacle cell(s) within {clearance:.2f} m of the "
                    f"path, the nearest {float(d.min()):.2f} m from it at {worst[0]:+.2f} m forward, "
                    f"{worst[1]:+.2f} m left of the robot. Clear the area, move the robot or "
                    "reduce the distances (red cubes in RViz)."), level="error")
                return
            self.get_logger().info(
                "Area clear" + (f" (nearest obstacle {nearest:.2f} m from the path)."
                                if math.isfinite(nearest) else " (no obstacles in the costmap)."))
            self._ready_to_run()

        def _ready_to_run(self):
            if bool(self.get_parameter("preflight_only").value):
                self._finish(EXIT_OK, "Preflight passed (preflight_only: the robot does not move).")
                return
            if not self.initial_parked:
                # Start from a known chassis angle: in line with the turret.
                self.park_ctx = {"node": self, "parking_active": self.parking_active,
                                 "omni_demo_park_base": True}
                self.parker.reset()
                self._set_phase("park_start", "Parking first: chassis in line with the turret")
                return
            self._next_step()

        def _tick_park_start(self, now, _dt):
            if not self.parker.step(self.park_ctx):
                return
            self.initial_parked = True
            pose = self._pose()
            if pose is None:
                self._fail_pose()
                return
            # Parking turns the chassis about its own centre, which moves the
            # turret point: the demo is centred where it ended up, re-checked.
            moved = math.hypot(pose[0] - self.start[0], pose[1] - self.start[1])
            self.start = pose
            self._publish_start_frame()
            self._publish_plan()
            self._set_phase("check_clearance", f"Parked (turret point moved {moved * 100:.0f} cm); "
                            "re-checking the area from here")

        def _next_step(self):
            self.step_index += 1
            self.tracker = None
            if self.step_index >= len(self.steps):
                self.stop_robot()
                self._finish(EXIT_OK, "Omni demo finished.")
                return
            kind, behaviour, payload = self.steps[self.step_index]
            label = BEHAVIOUR_LABELS[behaviour]
            if kind == "leg":
                self.tracker = LegTracker(payload, self.cfg.speed, self.cfg.accel,
                                          self.cfg.turret_rate_limit, self.gains)
                self.leg_deadline = time.monotonic() + 2.5 * payload.nominal_duration(
                    self.gains.min_progress_speed) + 45.0
                self._set_phase("run", f"[{label}] {payload.name} ({payload.length:.2f} m)")
            elif kind == "return":
                pos, yaw = self._local_pose() or (None, None)
                if pos is None:
                    self._fail_pose()
                    return
                leg = line_leg("return to the start pose", pos, (0.0, 0.0), yaw0=yaw, yaw1=0.0)
                leg.profile(self.cfg.speed, self.cfg.accel, self.cfg.chassis_rate_limit,
                            self.cfg.turret_rate_limit)
                self.tracker = LegTracker(leg, self.cfg.speed, self.cfg.accel,
                                          self.cfg.turret_rate_limit, self.gains)
                self.leg_deadline = time.monotonic() + 2.5 * leg.nominal_duration(
                    self.gains.min_progress_speed) + 45.0
                self._set_phase("return", f"[{label}] done; returning to the start pose "
                                f"({np.linalg.norm(pos) * 100:.1f} cm, {math.degrees(yaw):.1f} deg off)")
            elif kind == "pause":
                self.pause_until = time.monotonic() + float(payload)
                self._set_phase("pause", f"Pausing {payload:.1f} s before the next behaviour")
            elif kind == "park":
                self.stop_robot()
                self.park_ctx = {"node": self, "parking_active": self.parking_active,
                                 "omni_demo_park_base": True}
                self.parker.reset()
                self._set_phase("park", "Parking: turning the chassis back in line with the turret")

        def _fail_pose(self):
            self.stop_robot(repeat=3)
            self._finish(EXIT_FAILED, f"Lost the robot pose ({self.odom_frame} -> {self.robot_frame}).",
                         level="error")

        def _guard_blocked(self, now):
            """Runtime guard: True while the demo must hold still for an obstacle."""
            if not bool(self.get_parameter("runtime_guard").value):
                return False
            obstacles, _cm = self._obstacles_local()
            if obstacles is None or len(obstacles) == 0:
                near = False
            else:
                pos, _yaw = self._local_pose() or (None, None)
                if pos is None:
                    return False
                stop_r = self.cfg.robot_radius + float(self.get_parameter("runtime_stop_margin").value)
                near = bool(np.any(np.hypot(*(obstacles - pos).T) < stop_r))
            if near:
                self.guard_clear_since = None
                if not self.guard_paused:
                    self.guard_paused = True
                    self.get_logger().warn("Obstacle close to the robot: holding still until it clears.")
                    self._status("Paused: obstacle close to the robot")
                return True
            if self.guard_paused:
                if self.guard_clear_since is None:
                    self.guard_clear_since = now
                if now - self.guard_clear_since < float(self.get_parameter("runtime_clear_s").value):
                    return True
                self.guard_paused = False
                self.get_logger().info("Clear again: resuming.")
                self._status("Resumed")
            return False

        def _tick_run(self, now, dt):
            self._drive(now, dt)

        def _tick_return(self, now, dt):
            self._drive(now, dt)

        def _drive(self, now, dt):
            local = self._local_pose()
            if local is None:
                self._fail_pose()
                return
            pos, yaw = local
            self._record_trail(now)
            if self._guard_blocked(now):
                self.stop_robot()
                # Do not count the hold against the leg.
                self.leg_deadline += dt
                if self.tracker.end_reached_t is not None:
                    self.tracker.end_reached_t += dt
                return
            if now > self.leg_deadline:
                self.stop_robot(repeat=3)
                self._finish(EXIT_FAILED, f"'{self.tracker.leg.name}' took too long; stopped.",
                             level="error")
                return
            v, wz, status = self.tracker.step(pos, yaw, dt, now)
            if status == "failed":
                self.stop_robot(repeat=3)
                self._finish(EXIT_FAILED, f"Stopped: {self.tracker.message}.", level="error")
                return
            if status in ("done", "accepted"):
                self.stop_robot()
                if status == "accepted":
                    self.get_logger().warn(self.tracker.message)
                pause = self.tracker.leg.pause_after_s
                if pause > 0.0:
                    self.pause_until = now + pause
                    self.phase = "pause"
                else:
                    self._next_step()
                self._publish_markers()
                return
            # Start-frame velocity into the turret frame (sim_controller cmd_type 'relative').
            c, s = math.cos(yaw), math.sin(yaw)
            vt = np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])
            if self.pull is not None and float(np.linalg.norm(vt)) > 1e-6:
                phi = self._turret_angle()
                if phi is not None:
                    vt = undo_pull_compensation(vt, wz, phi, self.pull, prev=self.last_cmd)
            self.last_cmd = vt
            cmd = Twist()
            cmd.linear.x = float(vt[0])
            cmd.linear.y = float(vt[1])
            cmd.angular.z = float(wz)
            self.cmd_pub.publish(cmd)
            if int(now * 4) != int((now - dt) * 4):
                self._publish_markers()

        def _tick_pause(self, now, _dt):
            self.stop_robot()
            self._record_trail(now)
            if now >= self.pause_until:
                self._next_step()

        def _tick_park(self, now, _dt):
            self._record_trail(now)
            if self.parker.step(self.park_ctx):
                self._next_step()

        # ---------------- visualisation ----------------

        def _pose_stamped(self, x, y, yaw, stamp):
            ps = PoseStamped()
            ps.header.frame_id = self.odom_frame
            ps.header.stamp = stamp
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            q = quat(yaw)
            ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
            ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
            return ps

        def _publish_start_frame(self):
            t = TransformStamped()
            t.header.frame_id = self.odom_frame
            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = "omni_demo_start"
            t.transform.translation.x, t.transform.translation.y = self.start[0], self.start[1]
            q = quat(self.start[2])
            t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
            self.static_tf.sendTransform(t)

        def _publish_plan(self):
            path = Path()
            path.header.frame_id = self.odom_frame
            path.header.stamp = self.get_clock().now().to_msg()
            for _name, legs in self.plan:
                for leg in legs:
                    idx = np.unique(np.linspace(0, len(leg.xy) - 1, max(2, len(leg.xy) // 3)).astype(int))
                    world = to_world(leg.xy[idx], self.start)
                    for (x, y), k in zip(world, idx):
                        path.poses.append(self._pose_stamped(x, y, self.start[2] + leg.yaw[k],
                                                             path.header.stamp))
            self.plan_pub.publish(path)

        def _record_trail(self, now):
            if now - self.last_trail_t < 0.2:
                return
            pose = self._pose()
            if pose is None:
                return
            self.last_trail_t = now
            stamp = self.get_clock().now().to_msg()
            self.trail.header.stamp = stamp
            self.trail.poses.append(self._pose_stamped(pose[0], pose[1], pose[2], stamp))
            if len(self.trail.poses) > 6000:
                del self.trail.poses[:1000]
            self.trail_pub.publish(self.trail)

        def _publish_markers(self):
            if self.start is None:
                return
            stamp = self.get_clock().now().to_msg()
            arr = MarkerArray()

            def marker(mid, mtype, color, scale):
                m = Marker()
                m.header.frame_id = self.odom_frame
                m.header.stamp = stamp
                m.ns = "omni_demo"
                m.id = mid
                m.type = mtype
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = scale
                m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
                return m

            def outline(points_local):
                pts = to_world(np.vstack([points_local, points_local[:1]]), self.start)
                return [Point(x=float(x), y=float(y), z=0.02) for x, y in pts]

            # Start pose arrow.
            m = marker(0, Marker.ARROW, (0.2, 0.8, 0.3, 1.0), (0.4, 0.06, 0.06))
            m.pose = self._pose_stamped(self.start[0], self.start[1], self.start[2], stamp).pose
            m.pose.position.z = 0.05
            arr.markers.append(m)

            # Path box and the clear area the check needs around it.
            xmin, xmax, ymin, ymax = self.box
            m = marker(1, Marker.LINE_STRIP, (0.3, 0.6, 1.0, 0.8), (0.015, 0.0, 0.0))
            m.points = outline(np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]]))
            arr.markers.append(m)
            r = self.cfg.robot_radius + self.cfg.safety_margin
            a = np.linspace(0.0, 0.5 * math.pi, 12)
            corner_arcs = []
            for (cx, cy), a0 in (((xmax, ymax), 0.0), ((xmin, ymax), 0.5 * math.pi),
                                 ((xmin, ymin), math.pi), ((xmax, ymin), 1.5 * math.pi)):
                corner_arcs.append(np.column_stack([cx + r * np.cos(a + a0), cy + r * np.sin(a + a0)]))
            m = marker(2, Marker.LINE_STRIP, (1.0, 0.65, 0.0, 0.9), (0.025, 0.0, 0.0))
            m.points = outline(np.vstack(corner_arcs))
            arr.markers.append(m)

            # Reference point being chased.
            m = marker(3, Marker.SPHERE, (1.0, 0.2, 0.8, 0.9), (0.08, 0.08, 0.08))
            if self.tracker is not None:
                ref, _ = self.tracker.reference()
                w = to_world([ref], self.start)[0]
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(w[0]), float(w[1]), 0.05
            else:
                m.action = Marker.DELETE
            arr.markers.append(m)

            # Current step label.
            m = marker(4, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (0.0, 0.0, 0.15))
            if 0 <= self.step_index < len(self.steps):
                kind, behaviour, payload = self.steps[self.step_index]
                m.text = BEHAVIOUR_LABELS[behaviour] + (f": {payload.name}" if kind == "leg" else
                                                        "" if kind == "pause" else f": {kind}")
            else:
                m.text = "Omni demo"
            w = to_world([[xmax + r + 0.2, 0.0]], self.start)[0]
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(w[0]), float(w[1]), 0.6
            arr.markers.append(m)

            # Obstacles that failed the clearance check.
            m = marker(5, Marker.CUBE_LIST, (1.0, 0.1, 0.1, 0.9), (0.05, 0.05, 0.1))
            m.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in self.blocking_world]
            if not m.points:
                m.action = Marker.DELETE
            arr.markers.append(m)
            self.marker_pub.publish(arr)

    return OmniDemoNode


if __name__ == "__main__":
    sys.exit(main())
