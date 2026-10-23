from aruco_shape_tracker.geometry import (
    marker_center,
    quadrilateral_area,
)

import numpy as np
import pytest


def marker_at(center, half_size=0.04):
    x, y = center
    return np.array([
        [x - half_size, y + half_size],
        [x + half_size, y + half_size],
        [x + half_size, y - half_size],
        [x - half_size, y - half_size],
    ], dtype=np.float32)


def test_marker_helpers():
    corners = marker_at((3.0, -2.0), half_size=0.5)
    np.testing.assert_allclose(marker_center(corners), [3.0, -2.0])
    assert quadrilateral_area(corners) == pytest.approx(1.0)
