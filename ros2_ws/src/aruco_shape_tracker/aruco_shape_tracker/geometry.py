"""Pure geometry used by the ArUco tracker and its unit tests."""

from __future__ import annotations

import cv2
import numpy as np


def as_corners(value: np.ndarray) -> np.ndarray:
    """Return one marker's OpenCV corners as a finite (4, 2) array."""
    corners = np.asarray(value, dtype=np.float32).reshape(4, 2)
    if not np.isfinite(corners).all():
        raise ValueError('marker corners must be finite')
    return corners


def quadrilateral_area(corners: np.ndarray) -> float:
    """Area in pixels squared, independent of winding direction."""
    return float(abs(cv2.contourArea(as_corners(corners))))


def marker_center(corners: np.ndarray) -> np.ndarray:
    """Projectively invariant center from the intersection of the diagonals."""
    points = np.column_stack((as_corners(corners), np.ones(4, dtype=np.float32)))
    first_diagonal = np.cross(points[0], points[2])
    second_diagonal = np.cross(points[1], points[3])
    intersection = np.cross(first_diagonal, second_diagonal)
    if not np.isfinite(intersection).all() or abs(float(intersection[2])) < 1e-9:
        raise ValueError('marker diagonals do not have a finite intersection')
    return (intersection[:2] / intersection[2]).astype(np.float32)
