"""Omni base demo (task_planner_fsm/omni_demo.py): paths, checks and tracking.

Run with:

    python3 -m pytest test/test_omni_demo.py -v
"""

import math
import types

import numpy as np
import pytest

from task_planner_fsm import omni_demo as od


def _plan(*behaviours, **kw):
    return od.build_plan(od.DemoConfig(behaviours=behaviours, **kw))


def _ends(legs):
    return [(tuple(np.round(leg.xy[0], 3)), tuple(np.round(leg.xy[-1], 3))) for leg in legs]


# ---------------------------------------------------------------- behaviours

def test_behaviours_run_in_demo_order_whatever_the_selection_order():
    assert od.normalise_behaviours(["orbit", "Figure-8", "cross"]) == ("cross", "figure8", "orbit")
    assert od.normalise_behaviours(["square", "square"]) == ("square",)
    with pytest.raises(ValueError):
        od.normalise_behaviours(["spin"])


def test_cross_is_forward_back_left_right_then_diagonal_out_and_back():
    (_, legs), = _plan("cross", move_distance=0.4)
    d = 0.4
    assert _ends(legs) == [
        ((0, 0), (d, 0)), ((d, 0), (0, 0)),
        ((0, 0), (0, d)), ((0, d), (0, 0)),
        ((0, 0), (d, d)), ((d, d), (0, 0)),
    ]
    assert all(np.allclose(leg.yaw, 0.0) for leg in legs)


def test_square_is_centred_clockwise_and_laps_twice():
    (_, legs), = _plan("square", square_side=0.8, square_laps=2)
    h = 0.4
    corners = [(h, h), (h, -h), (-h, -h), (-h, h)]
    ends = _ends(legs)
    assert ends[0] == ((0, 0), corners[0])
    assert ends[-1] == (corners[0], (0, 0))
    sides = ends[1:-1]
    assert len(sides) == 8
    for i, (a, b) in enumerate(sides):
        assert a == corners[i % 4] and b == corners[(i + 1) % 4]
    # Clockwise seen from above (z up): negative signed area.
    pts = np.array(corners)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    assert area < 0
    assert all(np.allclose(leg.yaw, 0.0) for leg in legs)
    assert all(leg.pause_after_s > 0 for leg in legs)  # sharp corners: stop at each


def test_figure8_crosses_the_start_with_lobes_left_and_right():
    (_, legs), = _plan("figure8", figure8_length=0.7, figure8_width=0.5)
    leg, = legs
    assert np.allclose(leg.xy[0], 0.0, atol=1e-6) and np.allclose(leg.xy[-1], 0.0, atol=1e-6)
    assert leg.xy[:, 1].max() == pytest.approx(0.35, abs=0.01)
    assert leg.xy[:, 1].min() == pytest.approx(-0.35, abs=0.01)
    assert np.abs(leg.xy[:, 0]).max() == pytest.approx(0.25, abs=0.01)
    assert np.allclose(leg.yaw, 0.0)


def test_orbit_faces_the_centre_all_the_way_round_clockwise():
    (_, legs), = _plan("orbit", orbit_radius=0.4)
    back, arc, home = legs
    assert _ends([back]) == [((0, 0), (-0.4, 0))]
    assert np.allclose(np.linalg.norm(arc.xy, axis=1), 0.4, atol=1e-3)
    for p, yaw in zip(arc.xy[::50], arc.yaw[::50]):
        to_centre = math.atan2(-p[1], -p[0])
        assert abs(od.wrap(yaw - to_centre)) < 0.02
    # Clockwise: the polar angle decreases.
    ang = np.unwrap(np.arctan2(arc.xy[:, 1], arc.xy[:, 0]))
    assert ang[-1] - ang[0] == pytest.approx(-2.0 * math.pi, abs=0.01)
    assert abs(od.wrap(home.yaw[-1])) < 1e-6
    assert _ends([home]) == [((-0.4, 0), (0, 0))]


def test_default_shapes_fit_a_2_by_2_m_area():
    cfg = od.DemoConfig(behaviours=od.BEHAVIOURS)
    depth, width = od.required_area(cfg)
    assert depth <= 2.0 + 1e-6 and width <= 2.0 + 1e-6


# ---------------------------------------------------------------- speed profile

def test_profile_is_rest_to_rest_and_within_the_speed():
    for _, legs in _plan(*od.BEHAVIOURS, speed=0.12):
        for leg in legs:
            assert leg.v[0] == 0.0 and leg.v[-1] == 0.0
            assert leg.v.max() <= 0.12 + 1e-9


def test_profile_respects_acceleration():
    (_, legs), = _plan("cross", speed=0.2, accel=0.1, move_distance=1.0)
    leg = legs[0]
    dv2 = np.abs(np.diff(leg.v ** 2))
    assert np.all(dv2 <= 2.0 * 0.1 * np.diff(leg.s) + 1e-9)


def test_orbit_speed_capped_by_chassis_and_turret_rates():
    (_, legs), = _plan("orbit", orbit_radius=0.35, speed=0.2, chassis_rate_limit=0.18,
                       turret_rate_limit=0.3)
    arc = legs[1]
    assert arc.v.max() <= 0.18 * 0.35 + 1e-3


# ---------------------------------------------------------------- checks

def test_clearance_check_against_the_path_box():
    box = (-0.35, 0.35, -0.35, 0.35)
    obstacles = np.array([[1.2, 0.0], [0.9, 0.9], [-0.35, 1.0]])
    blocking, nearest = od.clearance_check(obstacles, box, 0.65)
    assert nearest == pytest.approx(0.65, abs=1e-6)  # (-0.35, 1.0) is 0.65 above the box
    assert len(blocking) == 0
    blocking, _ = od.clearance_check(np.array([[0.8, 0.0], [2.0, 2.0]]), box, 0.65)
    assert blocking.tolist() == [[0.8, 0.0]]
    assert od.clearance_check(np.zeros((0, 2)), box, 0.65)[1] == math.inf


def test_costmap_cells_to_points_and_frames():
    info = types.SimpleNamespace(
        width=4, height=3, resolution=0.5,
        origin=types.SimpleNamespace(position=types.SimpleNamespace(x=-1.0, y=-1.0),
                                     orientation=types.SimpleNamespace(z=0.0, w=1.0)))
    data = [0] * 12
    data[1 * 4 + 2] = 100   # row 1, col 2
    data[0] = 99            # inscribed, below the threshold
    pts = od.costmap_obstacles(info, data, 100)
    assert pts.tolist() == [[0.25, -0.25]]
    origin = (1.0, 2.0, math.pi / 2)
    local = od.to_local([[1.0, 3.0]], origin)
    assert np.allclose(local, [[1.0, 0.0]])
    assert np.allclose(od.to_world(local, origin), [[1.0, 3.0]])


def test_arm_fold_check_wraps_joints_and_needs_every_joint():
    q = dict(zip(od.ARM_JOINT_NAMES, od.FOLDED_JOINT_SETS["folded_fsm"]))
    assert od.arm_fold_error(q) == ("folded_fsm", pytest.approx(0.0, abs=1e-9))
    q["arm_wrist_3_joint"] += 2.0 * math.pi
    assert od.arm_fold_error(q)[1] == pytest.approx(0.0, abs=1e-9)
    q["arm_elbow_joint"] += 0.3
    assert od.arm_fold_error(q)[1] == pytest.approx(0.3, abs=1e-9)
    del q["arm_elbow_joint"]
    assert od.arm_fold_error(q) == (None, math.inf)


# ---------------------------------------------------------------- pull compensation

def _controller_output(u_turret, wz, phi, pc):
    """Chassis-frame velocity sim_controller makes of a turret-frame command."""
    c, s = math.cos(phi), math.sin(phi)
    uc = np.array([c * u_turret[0] - s * u_turret[1], s * u_turret[0] + c * u_turret[1]])
    return od.pull_compensate(uc, wz, pc)[0]


@pytest.mark.parametrize("phi", [0.0, 0.7, math.pi, -2.0, 2.5])
@pytest.mark.parametrize("v", [(-0.15, 0.03), (0.0, -0.12), (0.1, 0.1), (-0.05, -0.05), (0.02, -0.03)])
@pytest.mark.parametrize("wz", [0.0, -0.2])
def test_undo_pull_compensation_inverts_the_controller(phi, v, wz):
    pc = od.PullCompensation()
    v = np.array(v)
    c, s = math.cos(phi), math.sin(phi)
    want = np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
    u = od.undo_pull_compensation(v, wz, phi, pc)
    out = _controller_output(u, wz, phi, pc)
    assert np.linalg.norm(u) <= 1.1 * np.linalg.norm(v) + 1e-9
    if want[0] <= -pc.deadband and np.linalg.norm(want) >= 2.0 * pc.min_speed:
        # Fully mirrored by the controller: undone exactly.
        assert np.allclose(out, want, atol=2e-3)
    else:
        # Pushing (identity) or the partial blend, which can collapse the sideways
        # part: never worse than sending the request as it is.
        plain = _controller_output(v, wz, phi, pc)
        assert np.linalg.norm(out - want) <= np.linalg.norm(plain - want) + 2e-3


def test_undo_pull_compensation_is_identity_when_pushing():
    pc = od.PullCompensation()
    v = np.array([0.1, 0.05])
    assert np.allclose(od.undo_pull_compensation(v, 0.0, 0.0, pc), v)


# ---------------------------------------------------------------- tracking

def _track(leg, speed=0.15, lag=1.0, dt=0.05, t_max=120.0):
    """Drive a holonomic point (first-order lag on the command) along ``leg``."""
    tr = od.LegTracker(leg, speed, 0.15, 0.3)
    pos, yaw, vel, t = leg.xy[0].astype(float).copy(), float(leg.yaw[0]), np.zeros(2), 0.0
    dev = 0.0
    while t < t_max:
        v, wz, status = tr.step(pos, yaw, dt, t)
        if status != "running":
            return status, t, dev, tr
        vel += (v - vel) * min(1.0, dt / lag)
        pos = pos + vel * dt
        yaw = od.wrap(yaw + wz * dt)
        dev = max(dev, float(np.min(np.linalg.norm(leg.xy - pos, axis=1))))
        t += dt
    return "timeout", t, dev, tr


def test_tracker_follows_every_leg_and_stops_on_target():
    for _, legs in _plan(*od.BEHAVIOURS):
        for leg in legs:
            status, _t, dev, tr = _track(leg, lag=0.3)
            assert status == "done", (leg.name, tr.message)
            assert dev < 0.05, leg.name
            assert tr.error <= tr.g.pos_tol


def test_tracker_waits_for_a_slow_robot_instead_of_running_away():
    (_, legs), = _plan("cross")
    status, _t, dev, _tr = _track(legs[2], lag=3.0)   # a chassis swinging round
    assert status == "done"
    assert dev < 0.05


def test_tracker_fails_when_pushed_off_the_path():
    (_, legs), = _plan("cross")
    tr = od.LegTracker(legs[0], 0.15, 0.15, 0.3)
    _v, _wz, status = tr.step(np.array([0.0, 0.6]), 0.0, 0.05, 0.0)
    assert status == "failed"


# ---------------------------------------------------------------- closed loop

class _OmniBase:
    """Kinematic stand-in for sim_controller in cmd_type 'relative' (Sim_odometry::
    InverseKine_relative_speed): acceleration limits, per-axis clamps, pull
    compensation and the direction-preserving chassis/wheel/turret limits, on
    the navi_wall parameters (mobile_manipulator_controllers.yaml)."""
    R, D1, D2 = 0.125, 0.167, 0.3655
    V, A, WT, AT, WB, WM, TM = 0.2, 0.2, 0.35, 0.2, 0.2, 3.5, 0.5

    def __init__(self, chassis_heading=0.0):
        self.p = np.zeros(2)
        self.thc = chassis_heading
        self.thp = 0.0
        self.last = np.zeros(3)

    def step(self, vx, vy, wz, dt):
        cmd = np.array([vx, vy, wz])
        for i, a in enumerate((self.A, self.A, self.AT)):
            dec = abs(cmd[i]) < abs(self.last[i]) or self.last[i] * cmd[i] < 0
            m = a * dt * (2 if dec else 1)
            cmd[i] = self.last[i] + np.clip(cmd[i] - self.last[i], -m, m)
        self.last = cmd.copy()
        vx, vy = np.clip(cmd[:2], -self.V, self.V)
        wz = float(np.clip(cmd[2], -self.WT, self.WT))
        phi = self.thp - self.thc
        x = math.cos(phi) * vx - math.sin(phi) * vy
        y = math.sin(phi) * vx + math.cos(phi) * vy
        x, y = od.pull_compensate([[x, y]], wz, od.PullCompensation())[0]
        lo, hi = 0.0, 1.0
        for coeff, low, up in ((y / self.D1, -self.WB, self.WB),
                               (-y / self.D1, -self.TM - wz, self.TM - wz),
                               (x / self.R - self.D2 * y / (2 * self.R * self.D1), -self.WM, self.WM),
                               (x / self.R + self.D2 * y / (2 * self.R * self.D1), -self.WM, self.WM)):
            if abs(coeff) > 1e-9:
                a, b = sorted((low / coeff, up / coeff))
                lo, hi = max(lo, a), min(hi, b)
        scale = max(lo, min(1.0, hi)) if lo <= hi else 0.0
        x, y = x * scale, y * scale
        c, s = math.cos(self.thc), math.sin(self.thc)
        self.p = self.p + dt * np.array([c * x - s * y, s * x + c * y])
        self.thc += y / self.D1 * dt
        self.thp += wz * dt


def _run_on_base(behaviours, chassis_heading, undo=True, dt=0.05):
    cfg = od.DemoConfig(behaviours=behaviours)
    base = _OmniBase(chassis_heading)
    worst = 0.0
    for _name, legs in od.build_plan(cfg):
        for leg in legs:
            tr = od.LegTracker(leg, cfg.speed, cfg.accel, cfg.turret_rate_limit)
            t = 0.0
            while True:
                v, wz, status = tr.step(base.p, od.wrap(base.thp), dt, t)
                if status != "running" or t > 120.0:
                    break
                c, s = math.cos(base.thp), math.sin(base.thp)
                vt = np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])
                if undo:
                    vt = od.undo_pull_compensation(vt, wz, base.thp - base.thc, od.PullCompensation(),
                                                   prev=base.last[:2])
                base.step(vt[0], vt[1], wz, dt)
                worst = max(worst, float(np.min(np.linalg.norm(leg.xy - base.p, axis=1))))
                t += dt
            if status != "done":
                return False, worst
    return True, worst


@pytest.mark.parametrize("chassis_deg", [0, 180, -120])
def test_demo_runs_on_the_controller_model(chassis_deg):
    ok, worst = _run_on_base(("cross", "square"), math.radians(chassis_deg))
    assert ok
    assert worst < 0.06


def test_without_undoing_pull_compensation_reversing_legs_circle():
    # The reason undo_pull_compensation exists: 'right' after 'left' pulls with
    # the chassis a little off line, and the servo's sideways corrections get
    # mirrored into a circle round the target.
    ok, _worst = _run_on_base(("cross",), 0.0, undo=False)
    assert not ok
