r"""Keeping the base off obstacles during a sweep, as QP constraints.

The whole-body sweep drives the base directly over ``/cmd_vel``, which means it
bypasses Nav2 — and with it, the only thing that was watching for obstacles
while the base moved. A sweep is minutes of slow motion alongside a wall, so
that gap has to be closed inside the control law rather than around it.

The mechanism is a control barrier: let :math:`d(\mathbf{p})` be the distance
from a point on the base to the nearest obstacle, and require

.. math::  \nabla d \cdot \mathbf{v} \;\ge\; -\alpha\,(d - d_{\text{safe}}),

which permits approaching an obstacle quickly while far away, slows the approach
as the margin closes, and demands retreat once inside it. It is one linear row in
the base's velocity variables, so it drops straight into the QP that already
carries the actuator limits — no separate checker, no veto layer, and the
solver trades it off against the sweep task instead of overriding it.

Two details make it fit this robot:

**The footprint rotates.** Yawing the base moves the chassis without moving the
turret axis, so guarding a single point would miss a swing into a wall. Each
sample point on the footprint gets its own row, and the velocity of a point at
lever arm :math:`\mathbf{r}` is exactly what ``base_jacobian_block`` already
computes — the rows are that block's linear part, projected on
:math:`\nabla d`.

**The wall being scanned is an obstacle.** The whole point of the sweep is to
hold the base ~1 m off a wall that is lethal in the costmap, with its inflation
reaching further still. Left alone, the barrier would push the base away from
the very surface it is scanning and the arm would stretch to compensate until it
ran out of reach. So the scan corridor is masked out of the grid before the
distance transform (:func:`ObstacleField.from_grid`), and what remains is
everything the sweep did *not* plan for. The cost of that mask is real and worth
stating: obstacles ON the scan line — a pilaster, a fixture — become invisible
here. They are covered elsewhere, by ``reachable_wall_segments`` (which refuses
to sweep where the base cannot stand) and by the sweep's own standoff-jump abort
(which stops when the plate meets something the wall plane does not explain).

Pure numpy/scipy: the ROS grid is unpacked by the caller, so all of this is
testable without a costmap publisher.
"""

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from .kinematics import base_jacobian_block


@dataclass
class AvoidanceConfig:
    """Barrier tuning. Distances in metres, ``alpha`` in 1/s."""

    safety_margin: float = 0.15   # clearance demanded beyond the footprint
    influence: float = 1.0        # ignore obstacles further away than this
    alpha: float = 1.0            # how hard the barrier pushes back
    # Nav2 publishes its costmap on the 0..100 OccupancyGrid scale (-1 unknown),
    # where 100 is LETHAL (a real observed obstacle), 99 is INSCRIBED ("a disc of
    # robot_radius centred here touches something") and everything below is the
    # inflation tail. Count LETHAL ONLY. The inscribed ring and the inflation are
    # the planner's own robot model, and this module brings its own: it samples
    # the footprint perimeter and adds ``safety_margin``. Counting them too
    # double-counts the robot's size — with a 0.6 m robot_radius and a 1.0 m
    # inflation radius, a base parked 0.56 m off the wall it is scanning reads as
    # permanently in collision, and the barrier throttles the sweep to a crawl
    # against the very surface it is supposed to follow. (Observed in Gazebo:
    # a constant 0.15 m "clearance" and a sweep at a fifth of its commanded
    # speed, from a costmap whose only real obstacle was that wall.)
    obstacle_cost: int = 100
    # Unknown cells are NOT obstacles by default. A rolling local costmap always
    # has unknown cells at its rim, and treating them as lethal would stall the
    # sweep for something no sensor has actually seen. The plate's own ranges and
    # the standoff-jump abort cover what is directly ahead of the tool.
    unknown_is_obstacle: bool = False
    footprint_radius: float = 0.45
    footprint_offset: tuple = (0.0, 0.0)   # footprint centre in the turret frame
    n_samples: int = 8            # points around the footprint perimeter
    max_rows: int = 6             # keep only the most threatening samples
    # The corridor masked out of the grid, around the scan segment. It has to
    # cover the wall's marked cells plus however far the mapped wall sits from
    # the scan line the FSM computed — the segment comes from detected geometry,
    # the costmap from live lidar, and the two disagree by centimetres. It does
    # NOT need to cover the inflation radius, since only LETHAL cells count.
    mask_halfwidth: float = 0.7
    # How far the corridor runs PAST each end of the segment. Generous on
    # purpose. The wall does not stop where the segment does, and the base spends
    # the whole sweep within a metre of it — so a short corridor leaves the same
    # wall unmasked just beyond the segment ends, where its gradient points back
    # ALONG the wall. The barrier then pushes the base out of the masked window,
    # which unmasks more wall, which pushes harder: a runaway that drives the
    # sweep backwards. (Observed in Gazebo on a 0.6 m segment with a 0.5 m
    # extension: the sweep ran 0.87 m in reverse before settling against the
    # margin.) This must comfortably exceed how far the base can travel during
    # one sweep, including any retreat.
    mask_extension: float = 3.0


class ObstacleField:
    """Distance-to-obstacle over a costmap, with the scan corridor removed.

    The distance transform is computed once per costmap message (they arrive at
    a few Hz, the control loop runs at 50), then sampled cheaply each cycle.
    """

    def __init__(self, distance, resolution, origin, gradient=None):
        self.distance = distance                  # (h, w) metres to nearest obstacle
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)[:2]
        if gradient is None:
            # np.gradient returns d/drow, d/dcol; convert to metres and (x, y).
            grad_row, grad_col = np.gradient(distance, self.resolution)
            gradient = (grad_col, grad_row)
        self.grad_x, self.grad_y = gradient

    @classmethod
    def from_grid(cls, data, resolution, origin, config, mask_segment=None):
        """Build the field from a costmap array (row-major, shape ``(h, w)``).

        ``mask_segment`` is ``((x0, y0), (x1, y1))`` in the same frame as
        ``origin`` — the wall being swept, whose corridor is cleared before the
        transform.
        """
        data = np.asarray(data)
        obstacle = data >= config.obstacle_cost
        if config.unknown_is_obstacle:
            obstacle |= data < 0
        if mask_segment is not None:
            obstacle &= ~cls._corridor_mask(
                data.shape, resolution, origin, mask_segment, config)

        if not obstacle.any():
            # Nothing to avoid: a uniform "far away" field, so every row is
            # slack and the QP is unaffected.
            distance = np.full(data.shape, float(config.influence) * 10.0)
            return cls(distance, resolution, origin,
                       gradient=(np.zeros_like(distance), np.zeros_like(distance)))

        distance = ndimage.distance_transform_edt(~obstacle, sampling=resolution)
        return cls(distance, resolution, origin)

    @staticmethod
    def _corridor_mask(shape, resolution, origin, segment, config):
        """Cells within ``mask_halfwidth`` of the scan segment (the wall we sweep)."""
        height, width = shape
        origin = np.asarray(origin, dtype=float)
        xs = origin[0] + (np.arange(width) + 0.5) * resolution
        ys = origin[1] + (np.arange(height) + 0.5) * resolution
        grid_x, grid_y = np.meshgrid(xs, ys)

        (x0, y0), (x1, y1) = segment
        seg = np.array([x1 - x0, y1 - y0], dtype=float)
        length = float(np.linalg.norm(seg))
        if length < 1e-9:
            return np.zeros(shape, dtype=bool)
        unit = seg / length
        rel_x, rel_y = grid_x - x0, grid_y - y0
        # Projection along the segment, allowed to run past both ends so the
        # corridor covers where the sweep starts and finishes.
        t = np.clip(rel_x * unit[0] + rel_y * unit[1],
                    -config.mask_extension, length + config.mask_extension)
        perp = np.hypot(rel_x - t * unit[0], rel_y - t * unit[1])
        return perp <= config.mask_halfwidth

    def sample(self, x, y):
        """``(distance, grad_x, grad_y)`` at a world point (nearest-cell lookup).

        Off-grid points report a large distance and a zero gradient, i.e. "no
        obstacle here" — the local costmap is a rolling window and the base can
        legitimately sit near its edge.
        """
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        height, width = self.distance.shape
        if col < 0 or row < 0 or col >= width or row >= height:
            return float("inf"), 0.0, 0.0
        return (float(self.distance[row, col]),
                float(self.grad_x[row, col]), float(self.grad_y[row, col]))


def footprint_samples(config):
    """Points around the base footprint, in the TURRET frame.

    The turret frame is where the commanded twist lives, so keeping the samples
    there means the rows need no extra frame juggling — only the yaw that
    ``base_jacobian_block`` already takes.
    """
    centre = np.asarray(config.footprint_offset, dtype=float)
    angles = np.linspace(0.0, 2.0 * np.pi, config.n_samples, endpoint=False)
    return [centre + config.footprint_radius * np.array([np.cos(a), np.sin(a)])
            for a in angles]


def avoidance_rows(field, yaw, p_turret, config, n_arm=0):
    """Barrier rows on ``[vx, vy, wz]`` (plus zero columns for the arm).

    Returns ``(A, lower, upper, closest)`` with ``lower <= A u`` (upper is
    ``+inf``), and ``closest`` the smallest clearance seen — for logging. ``A``
    is empty when nothing is within the influence radius, which is the normal
    case mid-wall.

    Only the ``max_rows`` tightest samples are kept: past a handful the rest are
    slack or duplicates of the same obstacle, and every row costs solve time in a
    loop with a 20 ms budget.
    """
    rows, lowers = [], []
    closest = float("inf")
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    p_turret = np.asarray(p_turret, dtype=float)[:2]

    candidates = []
    for offset in footprint_samples(config):
        lever = rotation @ offset                      # turret -> world, still a lever arm
        point = p_turret + lever
        distance, grad_x, grad_y = field.sample(point[0], point[1])
        closest = min(closest, distance)
        if not np.isfinite(distance) or distance > config.influence:
            continue
        gradient = np.array([grad_x, grad_y])
        norm = np.linalg.norm(gradient)
        if norm < 1e-6:
            # No usable direction (uniform field, or the sample sits inside an
            # obstacle where the transform is flat). Nothing to constrain along.
            continue
        gradient /= norm
        # Velocity of this footprint point = the planar base Jacobian at its
        # lever arm; the barrier acts on its component along the gradient. Rows
        # 0:2 only — the distance field is planar, and a planar base has no
        # world-z velocity to constrain.
        point_jacobian = base_jacobian_block(yaw, np.array([lever[0], lever[1], 0.0]))[:2, :]
        candidates.append((distance,
                           gradient @ point_jacobian,
                           -config.alpha * (distance - config.safety_margin)))

    if not candidates:
        return np.zeros((0, 3 + n_arm)), np.zeros(0), np.zeros(0), closest

    candidates.sort(key=lambda c: c[0])
    for _, row, lower in candidates[:config.max_rows]:
        rows.append(row)
        lowers.append(lower)

    A = np.vstack(rows)
    if n_arm:
        A = np.hstack((A, np.zeros((A.shape[0], n_arm))))
    return A, np.array(lowers), np.full(len(lowers), np.inf), closest
