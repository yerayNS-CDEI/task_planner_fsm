"""The base's actuator limits, as QP constraints on the commanded Twist.

The whole-body Jacobian treats the base as a 3-DOF holonomic planar joint, which
is legitimate because ``sim_controller`` feedback-linearizes the diff-drive
chassis at the turret axis: it takes a Twist (vx, vy, wz) *in the turret frame*
and solves for wheel speeds plus a compensating turret rate. The commanded
(vx, vy) is therefore the velocity of the TURRET AXIS — not of the arm, which
rides ~0.29 m forward of it on the column — so ``kinematics`` takes the lever arm
from that axis. The abstraction is
leaky in exactly one way — the actuators underneath have limits that are NOT a
box on (vx, vy, wz) — and this module encodes that leak.

With ``phi`` the turret joint angle (turret heading relative to the chassis),
sim_controller's ``InverseKine_relative_speed`` rotates the commanded twist into
the chassis frame and solves::

    x_c = cos(phi) vx - sin(phi) vy          (chassis forward)
    y_c = sin(phi) vx + cos(phi) vy          (chassis lateral)
    w_chassis = y_c / d1                     (lateral motion YAWS the chassis)
    phi_dot   = wz - y_c / d1                (turret counter-rotates to hold heading)
    w_L = x_c / R - d2 y_c / (2 R d1)
    w_R = x_c / R + d2 y_c / (2 R d1)

so the real constraints are on ``y_c`` and ``x_c``, not on vy and vx. It also
clamps the twist itself (box) and, when a limit is violated, scales the whole
translation along its direction. That scaling is why the QP should respect the
limits itself: a command the controller has to rescale is a command whose
DIRECTION survives but whose speed does not, and the sweep quietly slows down.

The binding one for a wall sweep: sweeping along the wall is lateral motion,
which costs chassis yaw rate, so

    |v_along| <= d1 * w_chassis_max = 0.167 * 0.2 ~ 0.033 m/s

with the deployed navi-wall values. (The design note derives the cap from the
TURRET rate, 0.167 * 0.35 ~ 0.058 m/s; the controller applies the chassis limit
too, and it is the tighter of the two — see ``max_lateral_speed``.) Either way
the along-wall speed is set by the base geometry, not chosen freely: the only
ways to sweep faster are a longer ``center_distance`` or faster actuators.

Defaults below mirror ``navi-wall/config/diffdrive_controllers.yaml``
(``sim_controller`` block). Keep them in sync, or pass the live values in.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class BaseLimits:
    """sim_controller's limits for one robot. All rates in SI units."""

    center_distance: float = 0.167      # d1: turret axis ahead of the wheel axle (m)
    wheel_radius: float = 0.125         # R (m)
    wheel_separation: float = 0.3655    # d2 (m)
    vx_max: float = 0.2                 # max_x_velocity (m/s)
    vy_max: float = 0.2                 # max_y_velocity (m/s)
    wz_max: float = 0.35                # max_angular_turret: commanded heading rate (rad/s)
    w_chassis_max: float = 0.2          # max_angular_base: chassis yaw rate (rad/s)
    w_turret_motor_max: float = 0.5     # max_turret_motor_speed: turret JOINT rate (rad/s)
    wheel_speed_max: float = 0.0        # per-wheel rate (rad/s); 0 disables the rows

    def max_speed_along(self, turret_angle, chassis_angle):
        """Largest sustainable straight-line speed in a given heading.

        The cap is not one number: it depends on where the robot is going
        relative to its own body. Driving straight ahead costs only wheel speed
        and is limited by ``vx_max``; driving sideways has to yaw the chassis and
        collapses to ``max_lateral_speed``, six times slower. A wall swept with
        the chassis parked square is the sideways case, which is why that is the
        one usually quoted — but a chassis parked at an angle can sweep faster,
        and the reference should say so rather than assume the worst.

        ``turret_angle`` is the travel direction in the TURRET frame (where the
        commanded twist lives, so it meets the box limits) and ``chassis_angle``
        the same direction in the CHASSIS frame (which meets the yaw-rate
        limits). They differ by the turret joint angle. Assumes pure translation:
        a simultaneous turn eats into the same budget, and the QP's own rows,
        not this, are what enforce that.
        """
        caps = []
        for angle, along, across in (
            (turret_angle, self.vx_max, self.vy_max),
            (chassis_angle, None, self.center_distance * self.w_chassis_max),
        ):
            forward, lateral = abs(np.cos(angle)), abs(np.sin(angle))
            if along is not None and forward > 1e-6:
                caps.append(along / forward)
            if lateral > 1e-6:
                caps.append(across / lateral)
        if self.w_turret_motor_max > 0.0:
            lateral = abs(np.sin(chassis_angle))
            if lateral > 1e-6:
                caps.append(self.center_distance * self.w_turret_motor_max / lateral)
        return float(min(caps)) if caps else float("inf")

    def max_lateral_speed(self):
        """Largest sustainable chassis-lateral speed (m/s) — the sweep-speed cap.

        This is the number that decides how fast a wall can be swept. Both the
        chassis-yaw limit and the turret motor limit bound ``y_c``; the chassis
        one binds first with stock values.
        """
        caps = [self.center_distance * self.w_chassis_max]
        if self.w_turret_motor_max > 0.0:
            # phi_dot = wz - y_c/d1; with wz free up to wz_max the turret motor
            # allows |y_c|/d1 <= w_turret_motor_max + wz_max, so it only binds on
            # a robot with a slow turret. Take the honest worst case wz = 0.
            caps.append(self.center_distance * self.w_turret_motor_max)
        return float(min(caps))


def box_bounds(limits):
    """(lower, upper) box on the commanded base twist ``[vx, vy, wz]``."""
    lo = np.array([-limits.vx_max, -limits.vy_max, -limits.wz_max])
    return lo, -lo


def constraint_rows(limits, phi):
    """Linear actuator constraints on ``[vx, vy, wz]`` at turret angle ``phi``.

    Returns ``(A, lower, upper)`` with ``lower <= A u <= upper``, ``u`` the base
    twist in the turret frame. The rows are the chassis yaw rate, the turret
    motor rate and (when ``wheel_speed_max`` is set) the two wheel speeds — the
    quantities ``sim_controller`` would otherwise silently rescale.

    ``phi`` is the live ``turret_joint`` position. It matters: the sweep starts
    with the chassis parked square to the wall (``phi ~ 0``, turret frame ==
    chassis frame) and winds up as the base strafes, which rotates how much of
    the commanded (vx, vy) lands on the chassis-lateral axis.
    """
    d1 = limits.center_distance
    c, s = np.cos(phi), np.sin(phi)
    # Chassis-frame components of the commanded translation.
    row_xc = np.array([c, -s, 0.0])
    row_yc = np.array([s, c, 0.0])

    rows, lower, upper = [], [], []

    # Chassis yaw rate: w_chassis = y_c / d1.
    rows.append(row_yc / d1)
    lower.append(-limits.w_chassis_max)
    upper.append(limits.w_chassis_max)

    # Turret joint rate: phi_dot = wz - y_c / d1.
    if limits.w_turret_motor_max > 0.0:
        rows.append(np.array([0.0, 0.0, 1.0]) - row_yc / d1)
        lower.append(-limits.w_turret_motor_max)
        upper.append(limits.w_turret_motor_max)

    # Wheel speeds: w_{L,R} = x_c / R -+ d2 y_c / (2 R d1).
    if limits.wheel_speed_max > 0.0:
        R, d2 = limits.wheel_radius, limits.wheel_separation
        rows.append(row_xc / R - (d2 / (2.0 * R * d1)) * row_yc)
        lower.append(-limits.wheel_speed_max)
        upper.append(limits.wheel_speed_max)
        rows.append(row_xc / R + (d2 / (2.0 * R * d1)) * row_yc)
        lower.append(-limits.wheel_speed_max)
        upper.append(limits.wheel_speed_max)

    return np.vstack(rows), np.array(lower), np.array(upper)


def wheel_and_turret_rates(limits, u_base, phi):
    """What the base actuators will actually do for a commanded twist.

    Returns ``(w_left, w_right, phi_dot, w_chassis)`` — the same quantities
    sim_controller computes — so a controller can log or assert on them without
    re-deriving the model. Diagnostics only; the QP uses ``constraint_rows``.
    """
    vx, vy, wz = (float(u_base[0]), float(u_base[1]), float(u_base[2]))
    d1, R, d2 = limits.center_distance, limits.wheel_radius, limits.wheel_separation
    x_c = np.cos(phi) * vx - np.sin(phi) * vy
    y_c = np.sin(phi) * vx + np.cos(phi) * vy
    w_chassis = y_c / d1
    return (x_c / R - d2 * y_c / (2.0 * R * d1),
            x_c / R + d2 * y_c / (2.0 * R * d1),
            wz - w_chassis,
            w_chassis)
