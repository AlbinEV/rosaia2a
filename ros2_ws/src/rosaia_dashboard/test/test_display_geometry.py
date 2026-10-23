"""Check that home rectification is explicit and preserves source geometry."""

import numpy as np
import pytest

from rosaia_dashboard.display_geometry import (
    home_display_reference,
    rectified_tip_xy,
    rectify_home_shape,
)


def home_points():
    """Return a tilted, bent three-marker home in metres."""
    return [{'x': .02, 'y': .2, 'z': .04},
            {'x': .05, 'y': .4, 'z': .06},
            {'x': .08, 'y': .6, 'z': .08}]


def test_home_is_drawn_vertical_without_changing_measurements():
    """Only the first marker is aligned; subsequent bending is retained."""
    points = home_points()
    reference = home_display_reference(points)
    corrected = rectify_home_shape(points, reference)
    assert corrected[0]['x'] == pytest.approx(0, abs=1e-12)
    assert corrected[0]['z'] == pytest.approx(0, abs=1e-12)
    assert abs(corrected[-1]['x']) > 1e-3
    assert all(p['y'] > 0 for p in corrected)
    assert corrected[0]['y'] < corrected[1]['y'] < corrected[2]['y']
    assert points == home_points()
    rotation = np.asarray(reference['rotation'])
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    original = np.array([[p['x'], p['y'], p['z']] for p in points])
    display = np.array([[p['x'], p['y'], p['z']] for p in corrected])
    np.testing.assert_allclose(np.linalg.norm(np.diff(original, axis=0), axis=1),
                               np.linalg.norm(np.diff(display, axis=0), axis=1))


def test_live_and_model_tip_use_the_same_display_reference():
    """Planar learned tips and measured tips share the display transform."""
    points = home_points()
    reference = home_display_reference(points)
    points[-1] = dict(points[-1], x=.1)
    displayed = rectify_home_shape(points, reference)[-1]
    tip = rectified_tip_xy([points[-1]['x'], points[-1]['y']], reference)
    np.testing.assert_allclose(tip, [displayed['x'], displayed['y']])


def test_degenerate_home_is_rejected():
    """A zero-length home axis cannot define a view orientation."""
    with pytest.raises(ValueError):
        home_display_reference([{'x': 0, 'y': 0, 'z': 0}] * 3)
