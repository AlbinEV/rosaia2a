from aruco_shape_tracker.rgbd import (
    blend_rotation,
    deproject_pixel,
    frozen_reference_frame,
    PinholeIntrinsics,
    project_to_base_frame,
    robot_rotation_from_bases,
    robust_marker_depth,
)

import cv2
import numpy as np
import pytest


def test_robust_depth_rejects_edges_and_outlier():
    depth = np.ones((100, 100), dtype=np.float32)
    depth[39:62, 39:62] = 0.8
    depth[50, 50] = 4.0
    corners = np.array([[40, 40], [60, 40], [60, 60], [40, 60]])
    value, count = robust_marker_depth(depth, corners, inner_fraction=0.6)
    assert value == pytest.approx(0.8)
    assert count > 100


def test_deprojection_and_base_transform():
    intrinsics = PinholeIntrinsics(500.0, 500.0, 320.0, 240.0)
    point = deproject_pixel([370.0, 265.0], 1.0, intrinsics)
    np.testing.assert_allclose(point, [0.1, 0.05, 1.0])
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    result = project_to_base_frame([point], [0.1, 0.0, 1.0], rotation)
    np.testing.assert_allclose(result, [[0.05, 0.0, 0.0]], atol=1e-12)


def test_rotation_blend_stays_on_so3():
    first = np.eye(3)
    second, _ = cv2.Rodrigues(np.array([0.2, 0.1, -0.3]))
    result = blend_rotation(first, second, 0.2)
    np.testing.assert_allclose(result.T @ result, np.eye(3), atol=1e-12)
    assert np.linalg.det(result) == pytest.approx(1.0)


def test_robot_frame_uses_base_baseline_and_camera_down():
    rotation = robot_rotation_from_bases(
        [-0.2, -0.3, 0.85], [0.2, -0.3, 0.87]
    )
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)
    assert rotation[:, 0] @ np.array([0.4, 0.0, 0.02]) > 0.0
    assert rotation[:, 1] @ np.array([0.0, 1.0, 0.0]) > 0.99


def test_frozen_reference_frame_uses_robust_origins():
    samples = {
        0: [
            np.array([-0.2, -0.3, 0.85]),
            np.array([-0.201, -0.3, 0.85]),
            np.array([2.0, 2.0, 2.0]),
        ],
        1: [
            np.array([0.2, -0.3, 0.87]),
            np.array([0.201, -0.3, 0.87]),
            np.array([3.0, 3.0, 3.0]),
        ],
    }
    origins, rotation = frozen_reference_frame(samples, [0, 1])
    np.testing.assert_allclose(origins[0], [-0.2, -0.3, 0.85], atol=0.002)
    np.testing.assert_allclose(origins[1], [0.201, -0.3, 0.87], atol=0.002)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
