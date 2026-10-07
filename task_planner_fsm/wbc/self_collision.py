r"""Keeping the arm off itself and off the robot it is mounted on, as QP rows.

The whole-body sweep streams joint velocities straight to the arm, so none of
the collision checking the FSM's planner does ever sees these motions. In
contact that is fine — the plate is on the wall and the arm barely moves. In
the air it is not: the retreat folds the elbow, and the return swings the
joints back to the unfolded pose, both through space nothing is watching. On
2026-09-18 the retreat took the elbow from 147 to 164 deg in six seconds.

This is the same control barrier ``avoidance.py`` puts on the base, applied to
pairs of points on the robot. For a pair at distance :math:`d`, require

.. math::  \dot d \;\ge\; -\alpha\,(d - d_{\text{safe}}),

which lets the pair close quickly while far apart, slows the approach as the
margin closes, and pushes them apart once inside it. :math:`\dot d` is linear in
the joint rates, so each pair is one row in the QP, traded off against the task
by the solver rather than vetoing its answer afterwards.

**Geometry is spheres.** Each arm link is covered by a few spheres fixed in the
link's frame; the static parts of the robot (the column the arm stands on, the
turret body under it) are boxes and vertical cylinders. Sphere-to-anything
distances are closed form and their gradient is the sphere centre's velocity,
which is one point Jacobian — so the rows cost microseconds against a 1 ms
solve. A sphere model is conservative on thin links and a little generous at
the corners of boxes; ``safety_margin`` is what covers the difference.

**Everything is in the arm's root frame.** The base carries the arm and the
robot's body together, so base motion never changes a self-distance and the
rows have zero base columns. Obstacles fixed to the ARM ROOT (the column top,
the UR's own base) are given in that frame directly; obstacles fixed to the
TURRET (its body) are given in a second frame whose pose in the root frame the
caller supplies each cycle — from TF, so it carries the column's live height.

Pure numpy: the caller parses nothing and looks nothing up, so all of this is
testable without ROS.
"""

import json
from dataclasses import dataclass

import numpy as np

from .kinematics import SerialChain, axis_angle_to_matrix

ROOT = "root"   # obstacle frame tag: fixed to the arm's root link
BODY = "body"   # obstacle frame tag: fixed to the frame the caller poses each cycle


@dataclass
class SelfCollisionConfig:
    """Barrier tuning. Distances in metres, ``alpha`` in 1/s."""

    safety_margin: float = 0.04   # clearance demanded between sphere surfaces
    influence: float = 0.20       # ignore pairs further apart than this
    alpha: float = 2.0            # how hard the barrier pushes back
    max_rows: int = 8             # keep only the most threatening pairs


def capsule_spheres(link, start, end, radius, spacing=None):
    """Spheres of ``radius`` along a segment in ``link``'s frame, both ends included.

    Spaced at ``radius`` by default, which leaves the surface between two
    neighbours at most ~13% of a radius inside the capsule it stands for.
    """
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    spacing = float(spacing or radius)
    n = max(1, int(np.ceil(np.linalg.norm(end - start) / spacing)))
    return [(link, tuple(start + (end - start) * t), float(radius))
            for t in np.linspace(0.0, 1.0, n + 1)]


# ----------------------------------------------------------------------
# The default model: UR10e + sensor plate on the navi-wall column
# ----------------------------------------------------------------------
# Sized from the collision meshes in the robot's URDF (their bounds in each
# link's frame), with the tool0 prefix the navi-wall description uses. The
# plate's own mesh is oriented inconsistently with its sensors in the URDF, so
# the plate is covered from its functional geometry instead: a face about
# 0.40 x 0.43 m carrying the six sensors at (+/-0.155, +/-0.17), and the GPR
# standing 0.15 m proud of it. CHECK THE PLATE against the real one in RViz
# (self_collision_markers:=true) before trusting the margins it implies.
_PLATE_SPHERES = [("arm_plate_link", (x, y, 0.05), 0.09)
                  for x in (-0.13, 0.0, 0.13) for y in (-0.14, 0.0, 0.14)]
_PLATE_SPHERES += [("arm_plate_link", (-0.08, 0.0, 0.13), 0.06)]   # the GPR body

DEFAULT_SPHERES = (
    [("arm_shoulder_link", (0.0, 0.0, 0.0), 0.10)]
    + capsule_spheres("arm_upper_arm_link", (0.0, 0.0, 0.176), (-0.6127, 0.0, 0.176), 0.09)
    + capsule_spheres("arm_forearm_link", (0.0, 0.0, 0.045), (-0.5716, 0.0, 0.045), 0.065)
    + [("arm_wrist_1_link", (0.0, 0.0, -0.01), 0.07),
       ("arm_wrist_2_link", (0.0, 0.0, 0.0), 0.07),
       ("arm_wrist_3_link", (0.0, 0.0, -0.025), 0.055)]
    + capsule_spheres("arm_ee_cylinder_link", (0.0, 0.0, -0.075), (0.0, 0.0, 0.075), 0.05)
    + _PLATE_SPHERES
)

# Link pairs that can reach each other. Neighbours along the chain are left
# out (they touch by construction), and so is anything the arm's geometry
# cannot bring together — the SRDF's "Never" list says the same.
_DISTAL = ["arm_wrist_2_link", "arm_wrist_3_link", "arm_ee_cylinder_link", "arm_plate_link"]
DEFAULT_PAIRS = [
    (["arm_shoulder_link", "arm_upper_arm_link"], _DISTAL),
    (["arm_shoulder_link"], ["arm_wrist_1_link"]),
    (["arm_forearm_link"], ["arm_ee_cylinder_link", "arm_plate_link"]),
]

# The column's top 0.73 m (the part that rides with the arm), widened to the
# UR's own base housing and continued 0.10 m up through it. Radius is the
# larger of the base housing (0.095) and the column's half-diagonal (0.078).
# The turret body, from its mesh bounds in turret_link: x [-0.456, 0.407],
# y +/-0.306, top at 0.598 above turret_link.
DEFAULT_OBSTACLES = [
    {"name": "mast", "type": "cylinder", "frame": ROOT,
     "center": [0.0, 0.0], "radius": 0.095, "z_min": -0.73, "z_max": 0.10,
     # The upper arm's shoulder end sits against the base housing by
     # construction; only what hangs off the elbow can swing into the mast.
     "ignore": ["arm_shoulder_link", "arm_upper_arm_link"]},
    {"name": "body", "type": "box", "frame": BODY,
     "center": [-0.0245, 0.0, 0.1], "half_extents": [0.4315, 0.306, 0.498],
     "ignore": ["arm_shoulder_link"]},
]


# ----------------------------------------------------------------------
# Distances — vectorised over spheres, since the loop runs them every cycle
# ----------------------------------------------------------------------
def boxes_distance(centers, radii, box_center, half_extents):
    """Signed surface distances from ``k`` spheres to an axis-aligned box, and
    the unit normals pointing from the box toward each sphere. Negative inside,
    where the way out is by the nearest face."""
    q = np.atleast_2d(np.asarray(centers, dtype=float)) - np.asarray(box_center, dtype=float)
    half = np.asarray(half_extents, dtype=float)
    diff = q - np.clip(q, -half, half)
    outside = np.linalg.norm(diff, axis=1)
    depth = half - np.abs(q)
    axis = np.argmin(depth, axis=1)
    rows = np.arange(len(q))
    inside_normal = np.zeros_like(q)
    inside_normal[rows, axis] = np.where(q[rows, axis] >= 0.0, 1.0, -1.0)
    out = outside > 1e-9
    distance = np.where(out, outside, -depth[rows, axis]) - np.asarray(radii, dtype=float)
    normal = np.where(out[:, None], diff / np.maximum(outside, 1e-12)[:, None], inside_normal)
    return distance, normal


def cylinders_distance(centers, radii, axis_xy, cyl_radius, z_min, z_max):
    """Signed surface distances from ``k`` spheres to a vertical cylinder, and
    the unit normals pointing from the cylinder toward each sphere. Negative
    inside, where the way out is through the side or the nearer cap."""
    c = np.atleast_2d(np.asarray(centers, dtype=float))
    rel = c[:, :2] - np.asarray(axis_xy, dtype=float)
    r_xy = np.linalg.norm(rel, axis=1)
    radial = np.where((r_xy > 1e-9)[:, None], rel / np.maximum(r_xy, 1e-12)[:, None],
                      np.array([1.0, 0.0]))
    dr = r_xy - cyl_radius
    dz_below, dz_above = z_min - c[:, 2], c[:, 2] - z_max
    up = dz_above > dz_below
    dz = np.maximum(dz_below, dz_above)
    out_r, out_z = np.maximum(dr, 0.0), np.maximum(dz, 0.0)
    outside = np.hypot(out_r, out_z)
    out = outside > 0.0
    signed_z = np.where(up, 1.0, -1.0)
    out_normal = np.column_stack((radial * out_r[:, None], signed_z * out_z))
    out_normal /= np.maximum(outside, 1e-12)[:, None]
    by_side = dr >= dz               # inside: the shallower way out
    in_normal = np.where(by_side[:, None],
                         np.column_stack((radial, np.zeros(len(c)))),
                         np.column_stack((np.zeros((len(c), 2)), signed_z)))
    distance = np.where(out, outside, np.where(by_side, dr, dz)) - np.asarray(radii, dtype=float)
    return distance, np.where(out[:, None], out_normal, in_normal)


def sphere_box(center, radius, box_center, half_extents):
    """One sphere against a box: ``(distance, normal)``. See :func:`boxes_distance`."""
    d, n = boxes_distance([center], [radius], box_center, half_extents)
    return float(d[0]), n[0]


def sphere_cylinder(center, radius, axis_xy, cyl_radius, z_min, z_max):
    """One sphere against a cylinder: ``(distance, normal)``. See :func:`cylinders_distance`."""
    d, n = cylinders_distance([center], [radius], axis_xy, cyl_radius, z_min, z_max)
    return float(d[0]), n[0]


# ----------------------------------------------------------------------
# The model
# ----------------------------------------------------------------------
class SelfCollisionModel:
    """Sphere model of the arm, and the barrier rows it implies each cycle.

    ``joint_names`` fixes the column order of the rows — pass the QP's own
    (the main chain's) order. Spheres on links that are not below
    ``root_link`` in the URDF are dropped and listed in ``missing_links``
    rather than failing the whole model, so a description with a different
    tool still gets the arm covered.
    """

    def __init__(self, urdf_xml, root_link, joint_names, spheres=None, pairs=None,
                 obstacles=None, config=None):
        self.config = config or SelfCollisionConfig()
        self.joint_names = list(joint_names)
        spheres = DEFAULT_SPHERES if spheres is None else spheres
        pairs = DEFAULT_PAIRS if pairs is None else pairs
        obstacles = DEFAULT_OBSTACLES if obstacles is None else obstacles

        # The joint TREE under root_link that reaches every modelled link,
        # walked once per cycle: each joint after its parent, so a link's frame
        # is its last joint's. Walking one chain per link instead (and FK and
        # Jacobian separately) cost ~4 ms a cycle on the UR model.
        self.missing_links = []
        self._joints = []        # (joint, index of the parent entry or -1, QP column or -1)
        self._link_entry = {}    # link -> index into _joints of the joint that places it
        index_of = {}
        for link in sorted({s[0] for s in spheres}):
            try:
                chain = SerialChain.from_urdf(urdf_xml, root_link, link)
            except ValueError:
                self.missing_links.append(link)
                continue
            unknown = [n for n in chain.joint_names if n not in self.joint_names]
            if unknown:
                raise ValueError(f"link '{link}' moves with joints {unknown} that are not "
                                 f"among the QP's joints {self.joint_names}")
            parent = -1
            for joint in chain.joints:
                if joint.name not in index_of:
                    column = self.joint_names.index(joint.name) if joint.actuated else -1
                    index_of[joint.name] = len(self._joints)
                    self._joints.append((joint, parent, column))
                parent = index_of[joint.name]
            self._link_entry[link] = parent

        self.spheres = [(link, np.asarray(c, dtype=float), float(r))
                        for link, c, r in spheres if link in self._link_entry]
        self._offsets = np.array([c for _, c, _ in self.spheres]).reshape(-1, 3)
        self._radii = np.array([r for _, _, r in self.spheres])
        self._sphere_entry = np.array([self._link_entry[link] for link, _, _ in self.spheres],
                                      dtype=int)
        # Which QP columns move each sphere: the actuated joints above its link.
        n = len(self.joint_names)
        moved_by = np.zeros((len(self._joints), n), dtype=bool)
        for k, (_, parent, column) in enumerate(self._joints):
            if parent >= 0:
                moved_by[k] = moved_by[parent]
            if column >= 0:
                moved_by[k, column] = True
        self._mask = moved_by[self._sphere_entry] if self.spheres else np.zeros((0, n), bool)
        self._prismatic = np.zeros(n, dtype=bool)
        for joint, _, column in self._joints:
            if column >= 0 and joint.type == "prismatic":
                self._prismatic[column] = True
        by_link = {}
        for i, (link, _, _) in enumerate(self.spheres):
            by_link.setdefault(link, []).append(i)

        self.pairs = []          # (i, j) sphere indices
        for group_a, group_b in pairs:
            for link_a in group_a:
                for link_b in group_b:
                    if link_a == link_b:
                        continue
                    for i in by_link.get(link_a, []):
                        for j in by_link.get(link_b, []):
                            self.pairs.append((i, j))
        self._pair_a = np.array([i for i, _ in self.pairs], dtype=int)
        self._pair_b = np.array([j for _, j in self.pairs], dtype=int)

        self.obstacles = []      # (obstacle dict, sphere indices it applies to)
        for obstacle in obstacles:
            if obstacle.get("frame", ROOT) not in (ROOT, BODY):
                raise ValueError(f"obstacle frame must be '{ROOT}' or '{BODY}', "
                                 f"not {obstacle.get('frame')!r}")
            if obstacle.get("type") not in ("box", "cylinder"):
                raise ValueError(f"unknown obstacle type {obstacle.get('type')!r}")
            ignore = set(obstacle.get("ignore", []))
            applies = [i for i, (link, _, _) in enumerate(self.spheres) if link not in ignore]
            self.obstacles.append((obstacle, applies))

    @classmethod
    def from_json(cls, text, urdf_xml, root_link, joint_names, config=None):
        """Build from a JSON override: any of ``spheres`` (``[[link, [x,y,z], r], ...]``),
        ``capsules`` (``[[link, [x,y,z], [x,y,z], r], ...]``, added to the spheres),
        ``pairs`` and ``obstacles``. Keys left out keep their defaults."""
        spec = json.loads(text) if text else {}
        spheres = None
        if "spheres" in spec or "capsules" in spec:
            spheres = [(s[0], tuple(s[1]), float(s[2])) for s in spec.get("spheres", [])]
            for link, a, b, r in spec.get("capsules", []):
                spheres += capsule_spheres(link, a, b, r)
        return cls(urdf_xml, root_link, joint_names, spheres=spheres,
                   pairs=spec.get("pairs"), obstacles=spec.get("obstacles"), config=config)

    # ------------------------------------------------------------------
    def sphere_states(self, q):
        """``(centres, Jacobians)``: ``S x 3`` sphere centres and their ``S x 3 x N``
        point Jacobians, in the root frame. ``q`` is in ``joint_names`` order."""
        q = np.asarray(q, dtype=float)
        n = len(self.joint_names)
        frames = []
        axes = np.zeros((n, 3))
        origins = np.zeros((n, 3))
        for joint, parent, column in self._joints:
            R, p = frames[parent] if parent >= 0 else (np.eye(3), np.zeros(3))
            p = p + R @ joint.origin_xyz
            R = R @ joint.origin_rot
            if column >= 0:
                axes[column] = R @ joint.axis
                origins[column] = p
                if joint.type == "prismatic":
                    p = p + axes[column] * float(q[column])
                else:
                    R = R @ axis_angle_to_matrix(joint.axis, float(q[column]))
            frames.append((R, p))
        rotations = np.array([frames[k][0] for k in self._sphere_entry]).reshape(-1, 3, 3)
        positions = np.array([frames[k][1] for k in self._sphere_entry]).reshape(-1, 3)
        centres = positions + np.einsum("sij,sj->si", rotations, self._offsets)
        # Revolute column i moves a point P at axis_i x (P - origin_i);
        # prismatic, at axis_i. Zero for joints not above the sphere's link.
        lever = centres[:, None, :] - origins[None, :, :]
        J = np.cross(np.broadcast_to(axes, lever.shape), lever)
        J[:, self._prismatic, :] = axes[self._prismatic]
        J = J * self._mask[:, :, None]
        return centres, J.transpose(0, 2, 1)

    def rows(self, q, T_root_body=None):
        """Barrier rows ``A @ qdot >= lower`` for this configuration.

        Returns ``(A, lower, closest, label)``: ``A`` is ``k x len(joint_names)``
        and empty when every pair is outside ``influence``; ``closest`` is the
        smallest signed clearance seen (inf with nothing modelled) and ``label``
        names the pair it belongs to, for the log. ``T_root_body`` is the pose of
        the BODY obstacle frame in the root frame; without it those obstacles
        are skipped.
        """
        cfg = self.config
        n = len(self.joint_names)
        centres, jacobians = self.sphere_states(q)
        # Every candidate as (distance, normal, sphere a, sphere b or -1, label
        # of the other side). Rows are only built for the ones kept.
        distances, normals, first, second, others = [], [], [], [], []

        if self.pairs:
            gap = centres[self._pair_a] - centres[self._pair_b]
            span = np.linalg.norm(gap, axis=1)
            keep = span > 1e-9
            distances.append((span - self._radii[self._pair_a] - self._radii[self._pair_b])[keep])
            normals.append((gap / np.maximum(span, 1e-12)[:, None])[keep])
            first.append(self._pair_a[keep])
            second.append(self._pair_b[keep])
            others.append([self.spheres[j][0] for j in self._pair_b[keep]])

        for obstacle, applies in self.obstacles:
            if not applies:
                continue
            if obstacle.get("frame", ROOT) == BODY:
                if T_root_body is None:
                    continue
                R, p = T_root_body[:3, :3], T_root_body[:3, 3]
            else:
                R, p = np.eye(3), np.zeros(3)
            idx = np.asarray(applies, dtype=int)
            local = (centres[idx] - p) @ R           # R^T (c - p), row-wise
            if obstacle["type"] == "box":
                d, normal = boxes_distance(local, self._radii[idx], obstacle["center"],
                                           obstacle["half_extents"])
            else:
                d, normal = cylinders_distance(local, self._radii[idx], obstacle["center"],
                                               obstacle["radius"], obstacle["z_min"],
                                               obstacle["z_max"])
            distances.append(d)
            normals.append(normal @ R.T)             # back to the root frame
            first.append(idx)
            second.append(np.full(len(idx), -1))
            others.append([obstacle.get("name", obstacle["type"])] * len(idx))

        empty = np.zeros((0, n)), np.zeros(0)
        if not distances:
            return (*empty, float("inf"), "")
        distances = np.concatenate(distances)
        if not len(distances):
            return (*empty, float("inf"), "")
        normals = np.concatenate(normals)
        first = np.concatenate(first)
        second = np.concatenate(second)
        others = [name for group in others for name in group]

        order = np.argsort(distances, kind="stable")
        best = order[0]
        closest = float(distances[best])
        label = f"{self.spheres[first[best]][0]}~{others[best]}"
        near = [k for k in order[:cfg.max_rows] if distances[k] < cfg.influence]
        if not near:
            return (*empty, closest, label)
        A = np.zeros((len(near), n))
        for row, k in enumerate(near):
            J = jacobians[first[k]]
            if second[k] >= 0:
                J = J - jacobians[second[k]]
            A[row] = normals[k] @ J
        lower = -cfg.alpha * (distances[near] - cfg.safety_margin)
        return A, lower, closest, label
