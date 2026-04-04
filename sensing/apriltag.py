#!/usr/bin/env python3
"""Detect AprilTags from a camera and estimate distance.

This script uses a simple pinhole-camera approximation:

    distance = (tag_size * focal_length) / tag_width_in_pixels

If you do not know your camera's focal length, the script can estimate it
from an assumed horizontal field of view. For better accuracy, pass a real
focal length or calibrate your camera.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    print(
        "Missing dependency: pupil-apriltags\n"
        "Install it with: pip install pupil-apriltags opencv-python numpy",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc


@dataclass
class DistanceEstimate:
    tag_id: int
    distance_m: float
    x_m: float
    y_m: float
    xy_distance_m: float
    tag_width_px: float


def estimate_focal_length(image_width_px: int, fov_deg: float) -> float:
    """Estimate focal length in pixels from horizontal field of view."""
    fov_rad = math.radians(fov_deg)
    return image_width_px / (2.0 * math.tan(fov_rad / 2.0))


def edge_length(p1: np.ndarray, p2: np.ndarray) -> float:
    return float(np.linalg.norm(p1 - p2))


def tag_pixel_width(corners: np.ndarray) -> float:
    """Estimate the tag's apparent width in pixels from its corners."""
    top = edge_length(corners[0], corners[1])
    right = edge_length(corners[1], corners[2])
    bottom = edge_length(corners[2], corners[3])
    left = edge_length(corners[3], corners[0])
    return float((top + right + bottom + left) / 4.0)


def estimate_distance_m(tag_size_m: float, focal_length_px: float, tag_width_px: float) -> float:
    return (tag_size_m * focal_length_px) / tag_width_px


def estimate_camera_position_m(
    center_px: np.ndarray,
    image_shape: tuple[int, int, int],
    focal_length_px: float,
    depth_m: float,
) -> tuple[float, float, float]:
    """Estimate the tag center position in the camera coordinate system.

    Returns (x_m, y_m, xy_distance_m), where x is positive to the right,
    y is positive downward, and xy_distance_m is the lateral distance from
    the camera's center line in the image plane.
    """
    image_height_px, image_width_px = image_shape[:2]
    cx = image_width_px / 2.0
    cy = image_height_px / 2.0

    x_m = ((float(center_px[0]) - cx) * depth_m) / focal_length_px
    y_m = ((float(center_px[1]) - cy) * depth_m) / focal_length_px
    xy_distance_m = math.hypot(x_m, y_m)
    return x_m, y_m, xy_distance_m


def draw_detection(frame: np.ndarray, corners: np.ndarray, text: str) -> None:
    pts = corners.astype(int)
    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

    x, y = pts[0]
    y = max(y - 10, 20)
    cv2.putText(
        frame,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Show the estimated distance to an AprilTag from a live camera feed."
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera index to open (default: 0)")
    parser.add_argument("--tag-size-m", type=float, default=0.16, help="Actual tag side length in meters (default: 0.16)")
    parser.add_argument(
        "--focal-length-px",
        type=float,
        default=None,
        help="Camera focal length in pixels. If omitted, it is estimated from --fov-deg and the frame width.",
    )
    parser.add_argument(
        "--fov-deg",
        type=float,
        default=60.0,
        help="Assumed horizontal field of view used when focal length is not provided (default: 60)",
    )
    parser.add_argument(
        "--family",
        type=str,
        default="tag36h11",
        help="AprilTag family to detect (default: tag36h11)",
    )
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.0,
        help="Detection decision margin threshold; larger values reduce false positives (default: 0)",
    )
    parser.add_argument("--width", type=int, default=1280, help="Requested camera width (default: 1280)")
    parser.add_argument("--height", type=int, default=720, help="Requested camera height (default: 720)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    detector = Detector(
        families=args.family,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        nthreads=1,
    )

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Could not open camera {args.camera}", file=sys.stderr)
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    focal_length_px = args.focal_length_px
    print("Press q to quit.")
    if focal_length_px is None:
        print(
            f"Using approximate focal length from {args.fov_deg:.1f}° horizontal FOV until a real focal length is provided."
        )

    last_printed = {}

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read from camera.", file=sys.stderr)
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )

        if focal_length_px is None:
            focal = estimate_focal_length(frame.shape[1], args.fov_deg)
        else:
            focal = focal_length_px

        for det in detections:
            if det.decision_margin < args.confidence_threshold:
                continue

            width_px = tag_pixel_width(det.corners)
            if width_px <= 0:
                continue

            distance_m = estimate_distance_m(args.tag_size_m, focal, width_px)
            x_m, y_m, xy_distance_m = estimate_camera_position_m(det.center, frame.shape, focal, distance_m)
            label = f"ID {det.tag_id}: z {distance_m:.2f} m, x {x_m:.2f} m, y {y_m:.2f} m"
            draw_detection(frame, det.corners, label)

            estimate = DistanceEstimate(det.tag_id, distance_m, x_m, y_m, xy_distance_m, width_px)
            previous = last_printed.get(estimate.tag_id)
            if previous is None or abs(previous - estimate.distance_m) > 0.05:
                print(
                    f"Tag {estimate.tag_id}: z={estimate.distance_m:.2f} m, "
                    f"x={estimate.x_m:.2f} m, y={estimate.y_m:.2f} m, "
                    f"xy={estimate.xy_distance_m:.2f} m "
                    f"(apparent width {estimate.tag_width_px:.1f} px)"
                )
                last_printed[estimate.tag_id] = estimate.distance_m

        cv2.imshow("AprilTag Distance", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())