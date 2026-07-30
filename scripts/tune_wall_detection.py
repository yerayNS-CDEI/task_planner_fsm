#!/usr/bin/env python3
"""Offline tuner for map wall/column detection.

Loads a saved ROS map (``.yaml`` + image, the map_server format produced by
``nav2_map_server map_saver`` or rtabmap), runs :func:`detect_map_features`, and
renders an overlay PNG so detection parameters can be tuned WITHOUT launching
the FSM. The parameter names mirror the ``geometry_reconstruction_wall_*`` ROS
parameters one-to-one.

Usage:
    python3 scripts/tune_wall_detection.py --map my_map.yaml
    python3 scripts/tune_wall_detection.py --map my_map.yaml --out overlay.png \
        --column-max-extent-m 1.2 --approx-epsilon-m 0.15 --wall-min-length-m 0.8

The overlay marks: walls = red lines with yellow endpoints + ``W#`` labels,
columns = blue circles + ``C#`` labels.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

try:
    import cv2
except Exception as exc:  # pragma: no cover
    sys.exit(f"OpenCV (cv2) is required: {exc}")

# Allow running straight from the source tree without installing the package.
_OUTER_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _OUTER_PKG_DIR not in sys.path:
    sys.path.insert(0, _OUTER_PKG_DIR)

from task_planner_fsm.utils.map_wall_detection import detect_map_features  # noqa: E402


def load_occupancy_grid(yaml_path):
    """Load a map into (grid, origin_xy, resolution, display_image).

    Accepts either a map_server ``.yaml`` (+image) or a ``.npz`` dumped by the
    GeometryReconstruction state (the EXACT grid the FSM ran detection on -- use
    this to reproduce the live FSM result). ``grid`` is returned in OccupancyGrid
    orientation (row 0 = min-y, bottom-up), values in {-1 unknown, 0 free, ...}.
    """
    if yaml_path.endswith((".npz", ".npy")):
        data = np.load(yaml_path, allow_pickle=True)
        grid = np.asarray(data["grid"], dtype=np.int16)
        origin = (float(data["origin"][0]), float(data["origin"][1]))
        resolution = float(data["resolution"])
        # Build a top-down display image (occupied black, free white, unknown gray).
        disp = np.full(grid.shape, 205, dtype=np.uint8)
        disp[grid == 0] = 254
        disp[grid >= 65] = 0
        img = np.flipud(disp).copy()
        return grid, origin, resolution, img

    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    map_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(map_dir, image_path)

    resolution = float(meta["resolution"])
    origin = (float(meta["origin"][0]), float(meta["origin"][1]))
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)  # top-down (row 0 = top)
    if img is None:
        sys.exit(f"Could not read map image: {image_path}")

    # map_server occupancy probability convention.
    prob = (img.astype(np.float64) / 255.0) if negate else ((255.0 - img) / 255.0)
    occ = np.full(img.shape, -1, dtype=np.int16)
    occ[prob > occupied_thresh] = 100
    occ[prob < free_thresh] = 0

    # Image row 0 is the top (max y); OccupancyGrid row 0 is the bottom (min y).
    grid = np.flipud(occ).copy()
    return grid, origin, resolution, img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True, help="Path to the map .yaml file.")
    ap.add_argument("--out", default="wall_detection_overlay.png",
                    help="Output overlay PNG path.")
    # Detection params (defaults match detect_map_features / the ROS params).
    ap.add_argument("--occupied-thresh", type=int, default=50)
    ap.add_argument("--open-radius-cells", type=int, default=0)
    ap.add_argument("--close-radius-cells", type=int, default=2)
    ap.add_argument("--noise-min-area-cells", type=int, default=4)
    ap.add_argument("--column-max-extent-m", type=float, default=1.0)
    ap.add_argument("--column-max-aspect", type=float, default=2.5)
    ap.add_argument("--column-min-solidity", type=float, default=0.4)
    ap.add_argument("--obstacle-max-extent-m", type=float, default=2.0)
    ap.add_argument("--obstacle-max-unknown-frac", type=float, default=0.15)
    ap.add_argument("--wall-max-thickness-m", type=float, default=0.4)
    ap.add_argument("--wall-min-aspect", type=float, default=2.0)
    ap.add_argument("--wall-rect-fill-min", type=float, default=0.55)
    ap.add_argument("--approx-epsilon-m", type=float, default=0.4)
    ap.add_argument("--wall-min-length-m", type=float, default=1.0)
    ap.add_argument("--merge-angle-deg", type=float, default=14.0)
    ap.add_argument("--merge-dist-m", type=float, default=0.5)
    ap.add_argument("--merge-gap-m", type=float, default=0.5)
    ap.add_argument("--junction-split-tol-m", type=float, default=0.35)
    ap.add_argument("--junction-split-margin-m", type=float, default=1.0)
    ap.add_argument("--wall-fit-band-m", type=float, default=0.3)
    ap.add_argument("--embedded-col-min-protrusion-m", type=float, default=0.30)
    ap.add_argument("--embedded-col-min-diameter-m", type=float, default=0.4)
    ap.add_argument("--embedded-col-max-diameter-m", type=float, default=1.5)
    ap.add_argument("--inner-face-search-m", type=float, default=1.0)
    ap.add_argument("--corner-mend-tol-m", type=float, default=0.6)
    ap.add_argument("--corner-mend-max-extend-m", type=float, default=0.8)
    ap.add_argument("--corner-mend-min-angle-deg", type=float, default=20.0)
    args = ap.parse_args()

    grid, origin, resolution, img = load_occupancy_grid(args.map)
    height = grid.shape[0]
    print(f"Loaded map: {grid.shape[1]}x{grid.shape[0]} cells, res={resolution:.3f} m/cell, "
          f"origin={origin}")

    features = detect_map_features(
        grid, origin, resolution,
        occupied_thresh=args.occupied_thresh,
        open_radius_cells=args.open_radius_cells,
        close_radius_cells=args.close_radius_cells,
        noise_min_area_cells=args.noise_min_area_cells,
        column_max_extent_m=args.column_max_extent_m,
        column_max_aspect=args.column_max_aspect,
        column_min_solidity=args.column_min_solidity,
        obstacle_max_extent_m=args.obstacle_max_extent_m,
        obstacle_max_unknown_frac=args.obstacle_max_unknown_frac,
        wall_max_thickness_m=args.wall_max_thickness_m,
        wall_min_aspect=args.wall_min_aspect,
        wall_rect_fill_min=args.wall_rect_fill_min,
        approx_epsilon_m=args.approx_epsilon_m,
        min_wall_length_m=args.wall_min_length_m,
        merge_angle_deg=args.merge_angle_deg,
        merge_dist_m=args.merge_dist_m,
        merge_gap_m=args.merge_gap_m,
        junction_split_tol_m=args.junction_split_tol_m,
        junction_split_margin_m=args.junction_split_margin_m,
        wall_fit_band_m=args.wall_fit_band_m,
        embedded_col_min_protrusion_m=args.embedded_col_min_protrusion_m,
        embedded_col_min_diameter_m=args.embedded_col_min_diameter_m,
        embedded_col_max_diameter_m=args.embedded_col_max_diameter_m,
        inner_face_search_m=args.inner_face_search_m,
        corner_mend_tol_m=args.corner_mend_tol_m,
        corner_mend_max_extend_m=args.corner_mend_max_extend_m,
        corner_mend_min_angle_deg=args.corner_mend_min_angle_deg,
    )
    walls = features["walls"]
    columns = features["columns"]

    print(f"\nDetected {len(walls)} wall(s):")
    for i, (p0, p1) in enumerate(walls, 1):
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        print(f"  W{i}: ({p0[0]:.2f}, {p0[1]:.2f}) -> ({p1[0]:.2f}, {p1[1]:.2f})  |  {length:.2f} m")
    print(f"\nDetected {len(columns)} column(s):")
    for i, col in enumerate(columns, 1):
        cx, cy = col["center"]
        tag = " [embedded]" if col.get("embedded") else (" [obstacle]" if col.get("obstacle") else "")
        print(f"  O{i}: ({cx:.2f}, {cy:.2f})  |  r={col['radius_m']:.2f} m{tag}")

    def world_to_img(x, y):
        col = (x - origin[0]) / resolution - 0.5
        r = (y - origin[1]) / resolution - 0.5
        return int(round(col)), int(round((height - 1) - r))

    canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)  # top-down, BGR
    for i, (p0, p1) in enumerate(walls, 1):
        a = world_to_img(p0[0], p0[1])
        b = world_to_img(p1[0], p1[1])
        cv2.line(canvas, a, b, (0, 0, 255), 2)            # red wall
        cv2.circle(canvas, a, 3, (0, 200, 255), -1)       # yellow endpoints
        cv2.circle(canvas, b, 3, (0, 200, 255), -1)
        mid = ((a[0] + b[0]) // 2, (a[1] + b[1]) // 2)
        cv2.putText(canvas, f"W{i}", mid, cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 130, 0), 2, cv2.LINE_AA)        # green wall label
    for i, col in enumerate(columns, 1):
        c = world_to_img(*col["center"])
        rad_px = max(2, int(round(col["radius_m"] / resolution)))
        # free-standing columns blue, embedded (pilaster) orange, obstacles green
        if col.get("embedded"):
            circle_color = (0, 140, 255)
        elif col.get("obstacle"):
            circle_color = (0, 180, 0)
        else:
            circle_color = (255, 150, 0)
        cv2.circle(canvas, c, rad_px, circle_color, 2)
        cv2.putText(canvas, f"O{i}", (c[0] + 4, c[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (130, 0, 130), 2, cv2.LINE_AA)      # magenta object label

    cv2.imwrite(args.out, canvas)
    print(f"\nOverlay written to: {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
