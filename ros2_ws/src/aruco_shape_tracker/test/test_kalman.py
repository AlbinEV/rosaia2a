from aruco_shape_tracker.kalman import PositionKalman3D

import numpy as np


def make_filter():
    return PositionKalman3D(0.001, 0.002, 0.5, 0.5)


def test_filter_initializes_and_smooths_measurements():
    tracker = make_filter()
    np.testing.assert_allclose(tracker.update([1, 2, 3], 1.0), [1, 2, 3])
    result = tracker.update([1.01, 2, 3], 1.03)
    assert 1.0 < result[0] < 1.01


def test_filter_resets_after_gap():
    tracker = make_filter()
    tracker.update([0, 0, 0], 1.0)
    result = tracker.update([2, 3, 4], 2.0)
    np.testing.assert_allclose(result, [2, 3, 4])
