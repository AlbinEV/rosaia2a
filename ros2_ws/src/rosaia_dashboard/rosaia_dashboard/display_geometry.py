"""Home-relative display geometry, independent of measured robot state."""

import numpy as np


def home_display_reference(points):
    """Align only the home base-to-first-marker direction with display down."""
    home = np.array([[p['x'], p['y'], p['z']] for p in points], dtype=float)
    if home.shape != (3, 3) or not np.isfinite(home).all():
        raise ValueError('three finite home marker positions required')
    length = np.linalg.norm(home[0])
    if length < 1e-3:
        raise ValueError('first home marker is too close to its base')
    down = home[0] / length
    right = np.array([1., 0., 0.]) - down[0] * down
    if np.linalg.norm(right) < 1e-6:
        right = np.array([0., 0., 1.]) - down[2] * down
    right /= np.linalg.norm(right)
    normal = np.cross(right, down)
    matrix = np.column_stack((right, down, normal))
    stations = np.cumsum(np.linalg.norm(
        np.diff(np.vstack((np.zeros((1, 3)), home)), axis=0), axis=1))
    return {'home_points': home.tolist(), 'rotation': matrix.tolist(),
            'stations_m': stations.tolist()}


def rectify_home_shape(points, reference):
    """Rotate the measured shape rigidly, preserving its bends and lengths."""
    values = np.array([[p['x'], p['y'], p['z']] for p in points], dtype=float)
    result = values @ np.asarray(reference['rotation'])
    return [{'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])}
            for p in result]


def rectified_tip_xy(raw_xy, reference):
    """Project a learned planar tip into the same home-relative display."""
    home = np.asarray(reference['home_points'])
    point = np.array([raw_xy[0], raw_xy[1], home[-1, 2]])
    transformed = point @ np.asarray(reference['rotation'])
    return transformed[:2]
