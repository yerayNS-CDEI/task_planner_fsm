"""Surface-relative sensing: the six plate ranges -> the local wall frame.

The sweep should follow the wall it *measures*, not the wall the map remembers:
that is what makes it robust to map error, non-planar walls and protrusions. Each
control cycle the six distance sensors are fitted to a plane, giving

  * ``m_hat`` -- the unit direction from the plate toward the wall (the surface
    normal, pointing INTO the surface), and
  * ``distance`` -- the mean measured standoff along it,

which the controller turns into a normal-direction correction and a plate
orientation target. The in-surface sweep tangent then follows by projecting the
scan-line direction onto the plane.

The plane fit is the one ``arm_control``'s ``wall_parallel_controller`` already
uses (weighted IRLS/Huber over the valid ranges, EMA-filtered), reproduced here
so the whole-body loop does not depend on that node being alive — during a
whole-body sweep it must NOT be running, since it streams its own IK setpoints.
The sensor layout and validity windows are copied from it: keep them in sync
(they are also duplicated in ``states/scan_wall.py`` for the arm's Z approach).

Sensor array convention (matches ``arduino_sensors[_sim].py`` ``distance_sensors``)::

    data = [C/U1, A/U2, B/U3, S1, S2, S3]   (metres, along the plate +Z axis)
    ultrasonic: C(0, .172)  A(-.155, -.17)  B(.155, -.17)
    ToF:        S1(-.152, .17)  S2(.152, .17)  S3(0, -.172)
"""

import numpy as np

# Sensor (x, y) in the plate frame, metres, in publish order.
SENSOR_XY = np.array([
    [0.000,  0.172],    # C/U1 ultrasonic, top-mid
    [-0.155, -0.170],   # A/U2 ultrasonic, bottom-left
    [0.155, -0.170],    # B/U3 ultrasonic, bottom-right
    [-0.152, 0.170],    # S1 ToF, top-left
    [0.152,  0.170],    # S2 ToF, top-right
    [0.000, -0.172],    # S3 ToF, bottom-mid
])
SENSOR_SIGMA = np.array([0.010, 0.010, 0.010, 0.010, 0.010, 0.010])
# Validity window per sensor. Mind the ToF ceiling: beyond 0.258 m only the
# ultrasonics report, so a fit taken far from the wall rests on three points.
VALID_LO = np.array([0.02, 0.02, 0.02, 0.011, 0.011, 0.011])
VALID_HI = np.array([3.90, 3.90, 3.90, 0.258, 0.258, 0.258])

MIN_VALID_SENSORS = 3   # a plane needs three points


def fit_wall_plane(distances, huber_k=1.5, iterations=3):
    """Robust weighted plane fit over the valid ranges.

    Fits ``d = a x + b y + c`` in the plate frame, so the surface normal is
    ``[-a, -b, 1]`` normalized — a unit vector pointing from the plate toward the
    wall, equal to plate +Z when the plate is parallel to the surface.

    Returns ``(normal, distance, n_valid)``, or ``(None, None, n_valid)`` when
    fewer than three sensors report a usable range. ``distance`` is the
    PERPENDICULAR gap from the plate centre to the fitted plane (``c * n_z``),
    which is what a correction along the normal has to close. It equals the mean
    range only when the plate is parallel; taking the plane at the plate centre
    also avoids the bias a plain mean picks up when the dropped sensors sit on
    one side (e.g. past 0.258 m only the three ultrasonics survive, and they are
    not symmetric about the centre).
    """
    d = np.asarray(distances, dtype=float)
    valid = np.isfinite(d) & (d > VALID_LO) & (d < VALID_HI)
    n_valid = int(valid.sum())
    if n_valid < MIN_VALID_SENSORS:
        return None, None, n_valid

    x, y, dv = SENSOR_XY[valid, 0], SENSOR_XY[valid, 1], d[valid]
    A = np.column_stack((x, y, np.ones_like(x)))
    w_base = 1.0 / (SENSOR_SIGMA[valid] ** 2)
    w = w_base.copy()

    theta = None
    for _ in range(iterations):
        W = np.diag(w)
        try:
            theta = np.linalg.solve(A.T @ W @ A, A.T @ W @ dv)
        except np.linalg.LinAlgError:
            return None, None, n_valid
        residual = dv - A @ theta
        mad = np.median(np.abs(residual - np.median(residual)))
        scale = 1.4826 * mad if mad > 1e-6 else (np.std(residual) + 1e-6)
        t = np.abs(residual) / (huber_k * scale)
        w = w_base * np.where(t <= 1.0, 1.0, 1.0 / np.maximum(t, 1e-9))

    a, b, c = theta
    normal = np.array([-a, -b, 1.0])
    norm = np.linalg.norm(normal)
    return normal / norm, float(c / norm), n_valid


class SurfaceEstimator:
    """Per-cycle surface estimate with an EMA low-pass on the normal.

    Filtering the FIT (not the raw ranges) is what keeps the plate from chasing
    ultrasonic noise; ``ema_alpha`` near 0 is smooth and laggy, near 1 is
    responsive and twitchy.
    """

    def __init__(self, ema_alpha=0.3):
        self.ema_alpha = float(ema_alpha)
        self.normal_plate = np.array([0.0, 0.0, 1.0])
        self.distance = None
        self.n_valid = 0

    def reset(self):
        self.normal_plate = np.array([0.0, 0.0, 1.0])
        self.distance = None
        self.n_valid = 0

    def update(self, distances):
        """Fold one ``/distance_sensors`` frame in. True when the estimate is usable."""
        normal, distance, n_valid = fit_wall_plane(distances)
        self.n_valid = n_valid
        if normal is None:
            return False
        filtered = self.ema_alpha * normal + (1.0 - self.ema_alpha) * self.normal_plate
        self.normal_plate = filtered / np.linalg.norm(filtered)
        self.distance = distance
        return True

    def tilt(self):
        """Angle (rad) between the plate and the sensed surface. 0 = parallel."""
        return float(np.arccos(np.clip(self.normal_plate[2], -1.0, 1.0)))


def sweep_tangent(m_hat, sweep_dir):
    """In-surface sweep direction: ``sweep_dir`` projected onto the sensed plane.

    ``sweep_dir`` is the scan segment's direction in world coordinates (from the
    wall geometry). Removing its component along the surface normal keeps the
    plate from drifting into or away from the wall as the segment direction and
    the sensed wall disagree — the standoff loop then owns the normal axis
    alone. Returns ``None`` if the segment is (degenerately) normal to the wall.
    """
    m_hat = np.asarray(m_hat, dtype=float)
    t = np.asarray(sweep_dir, dtype=float)
    t = t - np.dot(t, m_hat) * m_hat
    norm = np.linalg.norm(t)
    if norm < 1e-6:
        return None
    return t / norm


def plate_orientation_target(m_hat, up=(0.0, 0.0, 1.0)):
    """Desired plate rotation: +Z into the wall, +Y up, +X = Y x Z.

    Mirrors ``wall_parallel_controller``'s parallelism + roll compensation, but
    as an absolute target the whole-body QP can servo to, rather than an
    incremental IK setpoint. Returns a 3x3 world rotation matrix, or ``None``
    when the surface normal is (degenerately) vertical.
    """
    z = np.asarray(m_hat, dtype=float)
    z = z / np.linalg.norm(z)
    y = np.asarray(up, dtype=float) - np.dot(up, z) * z
    norm = np.linalg.norm(y)
    if norm < 1e-6:
        return None
    y = y / norm
    x = np.cross(y, z)
    return np.column_stack((x / np.linalg.norm(x), y, z))
