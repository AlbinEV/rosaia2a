"""RGB-D geometry primitives independent from ROS and RealSense."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import as_corners, marker_center


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Minimal pinhole camera model used after depth-to-colour alignment."""

    fx: float
    fy: float
    ppx: float
    ppy: float


def robust_marker_depth(
    depth_m: np.ndarray,
    corners: np.ndarray,
    *,
    inner_fraction: float = 0.60,
    minimum_samples: int = 10,
    minimum_depth_m: float = 0.10,
    maximum_depth_m: float = 5.0,
) -> tuple[float, int]:
    """
    Return a robust median depth from the inner part of a marker.

    The inner polygon avoids mixed foreground/background pixels at marker
    edges. A median-absolute-deviation gate then rejects isolated depth noise.
    """
    image = np.asarray(depth_m, dtype=np.float32)
    if image.ndim != 2:
        raise ValueError('depth image must be two-dimensional')
    if not 0.0 < inner_fraction <= 1.0:
        raise ValueError('inner_fraction must be in (0, 1]')
    if minimum_samples < 1:
        raise ValueError('minimum_samples must be positive')

    polygon = as_corners(corners).astype(np.float64)
    center = marker_center(polygon).astype(np.float64)
    polygon = center + inner_fraction * (polygon - center)
    polygon = np.rint(polygon).astype(np.int32)

    x0 = max(0, int(polygon[:, 0].min()))
    y0 = max(0, int(polygon[:, 1].min()))
    x1 = min(image.shape[1] - 1, int(polygon[:, 0].max()))
    y1 = min(image.shape[0] - 1, int(polygon[:, 1].max()))
    if x1 < x0 or y1 < y0:
        raise ValueError('marker polygon lies outside the depth image')

    mask = np.zeros((y1 - y0 + 1, x1 - x0 + 1), dtype=np.uint8)
    local_polygon = polygon - np.array([x0, y0], dtype=np.int32)
    cv2.fillConvexPoly(mask, local_polygon, 1)
    values = image[y0:y1 + 1, x0:x1 + 1][mask.astype(bool)]
    valid = values[
        np.isfinite(values)
        & (values >= minimum_depth_m)
        & (values <= maximum_depth_m)
    ]
    if valid.size < minimum_samples:
        raise ValueError('not enough valid marker depth samples')

    median = float(np.median(valid))
    mad = float(np.median(np.abs(valid - median)))
    if mad > 0.0:
        valid = valid[np.abs(valid - median) <= 3.5 * 1.4826 * mad]
    if valid.size < minimum_samples:
        raise ValueError('not enough depth samples after outlier rejection')
    return float(np.median(valid)), int(valid.size)


def deproject_pixel(
    pixel: np.ndarray, depth_m: float, intrinsics: PinholeIntrinsics
) -> np.ndarray:
    """Deproject one undistorted/aligned colour pixel into camera metres."""
    point = np.asarray(pixel, dtype=np.float64).reshape(2)
    if not np.isfinite(point).all() or not np.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError('pixel and depth must be finite and depth positive')
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise ValueError('focal lengths must be positive')
    return np.array([
        (point[0] - intrinsics.ppx) * depth_m / intrinsics.fx,
        (point[1] - intrinsics.ppy) * depth_m / intrinsics.fy,
        depth_m,
    ])


def project_to_base_frame(
    camera_points: np.ndarray,
    base_origin: np.ndarray,
    base_rotation: np.ndarray,
) -> np.ndarray:
    """Transform camera-frame points into a marker-aligned base frame."""
    points = np.asarray(camera_points, dtype=np.float64).reshape(-1, 3)
    origin = np.asarray(base_origin, dtype=np.float64).reshape(3)
    rotation = np.asarray(base_rotation, dtype=np.float64).reshape(3, 3)
    if not (
        np.isfinite(points).all()
        and np.isfinite(origin).all()
        and np.isfinite(rotation).all()
    ):
        raise ValueError('base transform inputs must be finite')
    return (points - origin) @ rotation


def robot_rotation_from_bases(
    left_base: np.ndarray,
    right_base: np.ndarray,
    *,
    minimum_baseline_m: float = 0.10,
) -> np.ndarray:
    """
    Build stable robot axes from both base centres and camera vertical.

    x runs from the left base to the right base. y is the optical-image down
    direction orthogonalized against x, hence it points along the arms in the
    present mounting. z completes a right-handed frame. Two distant base
    centres avoid the planar-pose ambiguity of PnP on one small square.
    """
    left = np.asarray(left_base, dtype=np.float64).reshape(3)
    right = np.asarray(right_base, dtype=np.float64).reshape(3)
    baseline = right - left
    length = float(np.linalg.norm(baseline))
    if not np.isfinite(length) or length < minimum_baseline_m:
        raise ValueError('base-marker baseline is too small')
    x_axis = baseline / length
    optical_down = np.array([0.0, 1.0, 0.0])
    y_axis = optical_down - np.dot(optical_down, x_axis) * x_axis
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm < 1e-6:
        raise ValueError('base baseline is degenerate with camera vertical')
    y_axis /= y_norm
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def frozen_reference_frame(
    samples: dict[int, list[np.ndarray]],
    reference_ids: list[int],
    *,
    minimum_baseline_m: float = 0.10,
) -> tuple[dict[int, np.ndarray], np.ndarray]:
    """Freeze robust base origins and their shared rotation from samples."""
    if len(reference_ids) < 2:
        raise ValueError('at least two reference markers are required')
    origins = {}
    for marker_id in reference_ids:
        values = np.asarray(samples.get(marker_id, []), dtype=np.float64)
        if values.ndim != 2 or values.shape[0] < 1 or values.shape[1] != 3:
            raise ValueError(f'missing reference samples for marker {marker_id}')
        if not np.isfinite(values).all():
            raise ValueError('reference samples must be finite')
        origins[marker_id] = np.median(values, axis=0)
    rotation = robot_rotation_from_bases(
        origins[reference_ids[0]],
        origins[reference_ids[1]],
        minimum_baseline_m=minimum_baseline_m,
    )
    return origins, rotation


def blend_rotation(
    previous: np.ndarray | None, current: np.ndarray, alpha: float
) -> np.ndarray:
    """Low-pass a rotation and project the result back onto SO(3)."""
    rotation = np.asarray(current, dtype=np.float64).reshape(3, 3)
    if previous is None:
        return rotation.copy()
    if not 0.0 < alpha <= 1.0:
        raise ValueError('alpha must be in (0, 1]')
    blended = (1.0 - alpha) * np.asarray(previous) + alpha * rotation
    left, _, right = np.linalg.svd(blended)
    result = left @ right
    if np.linalg.det(result) < 0.0:
        left[:, -1] *= -1.0
        result = left @ right
    return result
