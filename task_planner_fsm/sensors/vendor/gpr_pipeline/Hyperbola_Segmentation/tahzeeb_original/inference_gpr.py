"""
inference_gpr.py

Run inference with trained YOLO or Mask R-CNN models on GPR B-scan images.
Supports single images, folders, and test sets with COCO evaluation.

Outputs:
    - Detection visualizations (bounding boxes + masks + scores)
    - COCO mAP metrics (if ground truth annotations provided)
    - Results CSV with per-image detections
    - Summary statistics

Usage:
    # YOLO — single image
    python inference_gpr.py \
        --model-type yolo \
        --weights results/yolo_runs/E1_R100_S0_seed42/weights/best.pt \
        --source path/to/image.jpg \
        --output results/inference

    # YOLO — folder of images
    python inference_gpr.py \
        --model-type yolo \
        --weights results/yolo_runs/E1_R100_S0_seed42/weights/best.pt \
        --source path/to/test_images/ \
        --output results/inference

    # Mask R-CNN — folder with visualization
    python inference_gpr.py \
        --model-type maskrcnn \
        --weights results/maskrcnn_runs/maskrcnn_seed42/best.pt \
        --source path/to/test_images/ \
        --output results/inference \
        --backbone resnet50

    # Mask R-CNN — test set with COCO evaluation
    python inference_gpr.py \
        --model-type maskrcnn \
        --weights results/maskrcnn_runs/maskrcnn_seed42/best.pt \
        --source path/to/test_images/ \
        --gt-json  path/to/test/annotations.json \
        --output results/inference

Author: Tahzeeb Hussain — UPC BarcelonaTech / DISCOVER Project
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from PIL import Image, ImageDraw
from torchvision import transforms as T

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ─────────────────────────────────────────────────────────────────────────────
# COLOR SCHEME
# ─────────────────────────────────────────────────────────────────────────────

PRED_COLOR = (255, 50, 50)       # Red for predictions
GT_COLOR = (50, 255, 50)         # Green for ground truth
MASK_COLOR = (255, 50, 50, 80)   # Semi-transparent red for masks
TEXT_BG = (0, 0, 0, 180)         # Dark background for text


def _mask_to_polygon(mask_bool: np.ndarray) -> list:
    """Largest external contour of a binary mask -> [[x, y], ...] in tile pixels.

    Returns [] if OpenCV is unavailable or no contour is found. Coordinates share
    the tile frame of x1/y1 and the ridge, so the polygon overlays directly on the
    tile and stitches with the same src_x0 offset used for the ridge.
    """
    try:
        import cv2
    except Exception:
        return []
    cnts, _ = cv2.findContours(mask_bool.astype(np.uint8),
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return []
    c = max(cnts, key=cv2.contourArea)
    return c.reshape(-1, 2).tolist()


def _polygon_area_px(poly: np.ndarray) -> int:
    """Shoelace area of an (N,2) polygon, in pixels."""
    if poly is None or len(poly) < 3:
        return 0
    x, y = poly[:, 0], poly[:, 1]
    return int(abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))) / 2.0)


# ─────────────────────────────────────────────────────────────────────────────
# YOLO INFERENCE
# ─────────────────────────────────────────────────────────────────────────────

def run_yolo_inference(
    weights: Path, source: Path, output: Path,
    conf_thresh: float = 0.50, iou_thresh: float = 0.50,
    imgsz: int = 640, device: Optional[str] = None,
    save_masks: bool = False,
) -> list[dict]:
    """Run YOLO inference and save visualizations."""
    from ultralytics import YOLO

    if device is None:
        device = "0" if torch.cuda.is_available() else "cpu"

    model = YOLO(str(weights))
    output.mkdir(parents=True, exist_ok=True)

    # Get image paths
    if source.is_file():
        img_paths = [source]
    else:
        img_paths = sorted(
            list(source.glob("*.jpg")) + list(source.glob("*.png")) +
            list(source.glob("*.jpeg")))

    print(f"YOLO Inference: {len(img_paths)} images")
    print(f"  Weights: {weights}")
    print(f"  Conf: {conf_thresh}  IoU: {iou_thresh}")

    all_detections = []
    total_time = 0
    mask_records: list[dict] = []

    for img_path in img_paths:
        t0 = time.time()
        results = model.predict(
            source=str(img_path), conf=conf_thresh, iou=iou_thresh,
            imgsz=imgsz, device=device, verbose=False,
            save=False, show=False)
        inf_time = time.time() - t0
        total_time += inf_time

        result = results[0]

        # Extract detections
        detections = []
        if result.boxes is not None:
            for i in range(len(result.boxes)):
                box = result.boxes[i]
                det = {
                    "image": img_path.name,
                    "class": "Hyperbola",
                    "confidence": round(float(box.conf[0]), 4),
                    "x1": round(float(box.xyxy[0][0]), 1),
                    "y1": round(float(box.xyxy[0][1]), 1),
                    "x2": round(float(box.xyxy[0][2]), 1),
                    "y2": round(float(box.xyxy[0][3]), 1),
                    "inference_ms": round(inf_time * 1000, 1),
                }
                detections.append(det)
                all_detections.append(det)

                # Mask ridge (upper arc) for hyperbola fitting
                if (save_masks and result.masks is not None
                        and i < len(result.masks.xy)):
                    poly = np.asarray(result.masks.xy[i])
                    ridge = []
                    if poly.size:
                        xcol = poly[:, 0].astype(int)
                        for xc in range(int(xcol.min()), int(xcol.max()) + 1):
                            yy = poly[xcol == xc, 1]
                            if yy.size:
                                ridge.append([int(xc), int(yy.min())])
                    mask_records.append({
                        "image": img_path.name,
                        "x1": det["x1"], "y1": det["y1"],
                        "x2": det["x2"], "y2": det["y2"], "ridge": ridge,
                        "polygon": poly.astype(int).tolist() if poly.size else [],
                        "area_px": _polygon_area_px(poly)})

        # Save visualization
        vis_img = Image.open(img_path).convert("RGB")
        draw = ImageDraw.Draw(vis_img)

        for det in detections:
            x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
            conf = det["confidence"]
            draw.rectangle([x1, y1, x2, y2], outline="red", width=3)
            draw.text((x1, y1 - 15), f"Hyp {conf:.2f}", fill="red")

        # Draw masks if available
        if result.masks is not None:
            for mask in result.masks.data:
                mask_np = mask.cpu().numpy()
                mask_resized = np.array(
                    Image.fromarray(mask_np).resize(vis_img.size))
                vis_np = np.array(vis_img)
                vis_np[mask_resized > 0.5] = (
                    vis_np[mask_resized > 0.5] * 0.6 +
                    np.array([255, 50, 50]) * 0.4).astype(np.uint8)
                vis_img = Image.fromarray(vis_np)

        n_det = len(detections)
        label = f"{n_det} detection{'s' if n_det != 1 else ''}"
        vis_img.save(output / f"det_{img_path.name}", quality=90)

        if len(img_paths) <= 20 or img_paths.index(img_path) % 50 == 0:
            print(f"  {img_path.name}: {n_det} detections "
                  f"({inf_time*1000:.0f}ms)")

    if save_masks:
        with open(output / "masks.json", "w") as f:
            json.dump(mask_records, f)
        print(f"  Masks JSON: {output / 'masks.json'} ({len(mask_records)} masks)")

    fps = len(img_paths) / total_time if total_time > 0 else 0
    print(f"\n  Total: {len(all_detections)} detections across "
          f"{len(img_paths)} images")
    print(f"  Speed: {fps:.1f} FPS ({total_time/len(img_paths)*1000:.0f}ms/img)")
    print(f"  Saved to: {output}")

    return all_detections


# ─────────────────────────────────────────────────────────────────────────────
# MASK R-CNN INFERENCE
# ─────────────────────────────────────────────────────────────────────────────

def run_maskrcnn_inference(
    weights: Path, source: Path, output: Path,
    backbone: str = "resnet50",
    conf_thresh: float = 0.5, device_str: Optional[str] = None,
    save_masks: bool = False,
) -> list[dict]:
    """Run Mask R-CNN inference and save visualizations."""
    import torchvision
    from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
    from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

    device = torch.device(
        device_str if device_str else
        ("cuda" if torch.cuda.is_available() else "cpu"))

    # Load model
    if backbone == "resnet101":
        from torchvision.models.detection.backbone_utils import resnet_fpn_backbone
        backbone_net = resnet_fpn_backbone("resnet101", weights=None,
                                            trainable_layers=3)
        model = torchvision.models.detection.MaskRCNN(backbone_net, num_classes=2)
    else:
        from torchvision.models.detection import maskrcnn_resnet50_fpn_v2
        model = maskrcnn_resnet50_fpn_v2(weights=None, num_classes=2)
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, 2)
        in_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
        model.roi_heads.mask_predictor = MaskRCNNPredictor(in_mask, 256, 2)

    model.load_state_dict(torch.load(str(weights), map_location=device,
                                      weights_only=True))
    model.to(device)
    model.eval()

    output.mkdir(parents=True, exist_ok=True)

    if source.is_file():
        img_paths = [source]
    else:
        img_paths = sorted(
            list(source.glob("*.jpg")) + list(source.glob("*.png")) +
            list(source.glob("*.jpeg")))

    print(f"Mask R-CNN Inference: {len(img_paths)} images")
    print(f"  Weights: {weights}")
    print(f"  Backbone: {backbone}")
    print(f"  Conf threshold: {conf_thresh}")

    all_detections = []
    total_time = 0
    transform = T.ToTensor()
    mask_records: list[dict] = []

    with torch.no_grad():
        for img_path in img_paths:
            img_pil = Image.open(img_path).convert("RGB")
            img_tensor = transform(img_pil).to(device)

            t0 = time.time()
            outputs = model([img_tensor])[0]
            inf_time = time.time() - t0
            total_time += inf_time

            # Create visualization
            vis_np = np.array(img_pil).copy()
            detections = []

            for i in range(len(outputs["boxes"])):
                score = outputs["scores"][i].item()
                if score < conf_thresh:
                    continue

                x1, y1, x2, y2 = outputs["boxes"][i].cpu().numpy()

                det = {
                    "image": img_path.name,
                    "class": "Hyperbola",
                    "confidence": round(score, 4),
                    "x1": round(float(x1), 1),
                    "y1": round(float(y1), 1),
                    "x2": round(float(x2), 1),
                    "y2": round(float(y2), 1),
                    "inference_ms": round(inf_time * 1000, 1),
                }
                detections.append(det)
                all_detections.append(det)

                # Draw mask
                if "masks" in outputs:
                    mask = outputs["masks"][i, 0].cpu().numpy() > 0.5
                    vis_np[mask] = (vis_np[mask] * 0.5 +
                                    np.array(PRED_COLOR) * 0.5).astype(np.uint8)
                    # Mask ridge (topmost y per column = the hyperbola arc)
                    if save_masks:
                        ys, xs = np.where(mask)
                        ridge = []
                        if xs.size:
                            for xc in range(int(xs.min()), int(xs.max()) + 1):
                                col = ys[xs == xc]
                                if col.size:
                                    ridge.append([int(xc), int(col.min())])
                        mask_records.append({
                            "image": img_path.name,
                            "x1": det["x1"], "y1": det["y1"],
                            "x2": det["x2"], "y2": det["y2"], "ridge": ridge,
                            "polygon": _mask_to_polygon(mask),
                            "area_px": int(mask.sum())})

            # Draw boxes and scores on top of masks
            vis_img = Image.fromarray(vis_np)
            draw = ImageDraw.Draw(vis_img)
            for det in detections:
                x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
                draw.rectangle([x1, y1, x2, y2], outline="red", width=3)
                draw.text((x1, y1 - 15),
                          f"Hyperbola {det['confidence']:.2f}", fill="red")

            # Status text
            n_det = len(detections)
            draw.text((10, 10),
                      f"{n_det} detection{'s' if n_det != 1 else ''} | "
                      f"{inf_time*1000:.0f}ms", fill="yellow")

            vis_img.save(output / f"det_{img_path.name}", quality=90)

            if len(img_paths) <= 20 or img_paths.index(img_path) % 50 == 0:
                print(f"  {img_path.name}: {n_det} detections "
                      f"({inf_time*1000:.0f}ms)")

    if save_masks:
        with open(output / "masks.json", "w") as f:
            json.dump(mask_records, f)
        print(f"  Masks JSON: {output / 'masks.json'} ({len(mask_records)} masks)")

    fps = len(img_paths) / total_time if total_time > 0 else 0
    print(f"\n  Total: {len(all_detections)} detections across "
          f"{len(img_paths)} images")
    print(f"  Speed: {fps:.1f} FPS ({total_time/len(img_paths)*1000:.0f}ms/img)")
    print(f"  Saved to: {output}")

    return all_detections


# ─────────────────────────────────────────────────────────────────────────────
# COCO EVALUATION (with GT)
# ─────────────────────────────────────────────────────────────────────────────

def evaluate_with_gt(detections: list[dict], gt_json: Path,
                     output: Path) -> dict:
    """Evaluate detections against COCO ground truth."""
    try:
        from pycocotools.coco import COCO
        from pycocotools.cocoeval import COCOeval
    except ImportError:
        print("  pycocotools not available for evaluation")
        return {}

    coco_gt = COCO(str(gt_json))

    # Build filename → image_id mapping
    fname_to_id = {img["file_name"]: img["id"]
                    for img in coco_gt.dataset["images"]}

    # Convert detections to COCO format
    coco_dets = []
    for det in detections:
        img_id = fname_to_id.get(det["image"])
        if img_id is None:
            continue
        coco_dets.append({
            "image_id": img_id,
            "category_id": 1,
            "bbox": [det["x1"], det["y1"],
                     det["x2"] - det["x1"], det["y2"] - det["y1"]],
            "score": det["confidence"],
        })

    if not coco_dets:
        print("  No matching detections for evaluation")
        return {}

    coco_dt = coco_gt.loadRes(coco_dets)
    coco_eval = COCOeval(coco_gt, coco_dt, "bbox")
    coco_eval.evaluate()
    coco_eval.accumulate()

    print(f"\n{'='*60}")
    print("COCO EVALUATION RESULTS")
    print(f"{'='*60}")
    coco_eval.summarize()

    metrics = {
        "mAP@50:95": round(coco_eval.stats[0], 4),
        "mAP@50": round(coco_eval.stats[1], 4),
        "mAP@75": round(coco_eval.stats[2], 4),
        "AR@1": round(coco_eval.stats[6], 4),
        "AR@10": round(coco_eval.stats[7], 4),
        "AR@100": round(coco_eval.stats[8], 4),
    }

    # Save metrics
    metrics_path = output / "evaluation_metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"\n  Metrics saved: {metrics_path}")

    return metrics


# ─────────────────────────────────────────────────────────────────────────────
# SUMMARY FIGURE
# ─────────────────────────────────────────────────────────────────────────────

def create_summary_grid(output: Path, max_images: int = 16) -> None:
    """Create a grid of detection results for paper figures."""
    det_images = sorted(output.glob("det_*.jpg"))[:max_images]
    if not det_images:
        return

    n = len(det_images)
    cols = min(4, n)
    rows = (n + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
    if rows == 1 and cols == 1:
        axes = [[axes]]
    elif rows == 1:
        axes = [axes]
    elif cols == 1:
        axes = [[ax] for ax in axes]

    for i, img_path in enumerate(det_images):
        r, c = i // cols, i % cols
        img = plt.imread(str(img_path))
        axes[r][c].imshow(img)
        axes[r][c].set_title(img_path.stem.replace("det_", ""), fontsize=8)
        axes[r][c].axis("off")

    # Hide empty subplots
    for i in range(n, rows * cols):
        r, c = i // cols, i % cols
        axes[r][c].axis("off")

    plt.suptitle("GPR Hyperbola Detection Results", fontsize=14,
                 fontweight="bold")
    plt.tight_layout()
    grid_path = output / "detection_grid.png"
    plt.savefig(grid_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Grid saved: {grid_path}")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(
        description="GPR Hyperbola Detection Inference")

    p.add_argument("--model-type", required=True,
                   choices=["yolo", "maskrcnn"],
                   help="Model type: yolo or maskrcnn")
    p.add_argument("--weights", type=Path, required=True,
                   help="Path to trained weights (.pt)")
    p.add_argument("--source", type=Path, required=True,
                   help="Image file or directory of images")
    p.add_argument("--output", type=Path, default=Path("results/inference"),
                   help="Output directory for results")
    p.add_argument("--gt-json", type=Path, default=None,
                   help="COCO ground truth JSON for evaluation (optional)")
    p.add_argument("--conf", type=float, default=0.25,
                   help="Confidence threshold (default: 0.25)")
    p.add_argument("--backbone", type=str, default="resnet50",
                   choices=["resnet50", "resnet101"],
                   help="Mask R-CNN backbone (default: resnet50)")
    p.add_argument("--device", default=None)
    p.add_argument("--imgsz", type=int, default=640,
                   help="YOLO input size (default: 640)")
    p.add_argument("--save-masks", action="store_true",
                   help="Export per-detection mask to masks.json: ridge points (for "
                        "hyperbola fitting) PLUS full-mask polygon + area_px (for "
                        "mask-overlap dedup and faithful visualization)")

    a = p.parse_args()

    if not a.weights.exists():
        print(f"ERROR: Weights not found: {a.weights}", file=sys.stderr)
        return 1
    if not a.source.exists():
        print(f"ERROR: Source not found: {a.source}", file=sys.stderr)
        return 1

    a.output.mkdir(parents=True, exist_ok=True)

    # Run inference
    if a.model_type == "yolo":
        detections = run_yolo_inference(a.weights, a.source, a.output, conf_thresh=a.conf, imgsz=a.imgsz,
                                        device=a.device, save_masks=a.save_masks)
    else:
        detections = run_maskrcnn_inference(
            a.weights, a.source, a.output,
            backbone=a.backbone, conf_thresh=a.conf,
            device_str=a.device, save_masks=a.save_masks)

    # Save detections CSV
    if detections:
        det_csv = a.output / "detections.csv"
        with open(det_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=detections[0].keys())
            w.writeheader()
            w.writerows(detections)
        print(f"  Detections CSV: {det_csv}")

    # Evaluate against ground truth if provided
    if a.gt_json and a.gt_json.exists():
        evaluate_with_gt(detections, a.gt_json, a.output)

    # Create summary grid
    create_summary_grid(a.output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
