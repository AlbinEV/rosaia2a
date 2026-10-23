"""Tests for non-actuating identification reference generation."""

import numpy as np
import pytest

from rosaia_learning.excitation import (
    append_return_home,
    fourier_profile,
    step_profile,
    step_velocity_cycle,
    trapezoidal_velocity_profile,
)


def test_step_profile_levels_and_length():
    """Each step level receives the configured number of samples."""
    times, values = step_profile([0.0, 2.0, -2.0], 0.5, 10.0)
    assert len(times) == 15
    np.testing.assert_array_equal(values[:5], np.zeros(5))
    np.testing.assert_array_equal(values[5:10], np.full(5, 2.0))


def test_step_velocity_cycle_is_constant_and_returns_home():
    """The powered step cycle has equal constant outward and return areas."""
    times, values = step_velocity_cycle(2.0, 0.5, 10.0, 4.0)
    np.testing.assert_array_equal(values[:20], np.full(20, 4.0))
    np.testing.assert_array_equal(values[20:25], np.zeros(5))
    np.testing.assert_array_equal(values[25:45], np.full(20, -4.0))
    assert values[-1] == 0.0
    assert times[-1] == pytest.approx(4.5)


def test_fourier_profile_respects_peak_limit():
    """Multisine normalization reaches but does not exceed its limit."""
    _, values = fourier_profile([0.1, 0.23], 20.0, 50.0, 8.0, 7)
    assert np.max(np.abs(values)) <= 8.0 + 1e-12
    assert np.isclose(np.max(np.abs(values)), 8.0)


def test_trapezoid_has_requested_timing_peak_and_zero_endpoints():
    """The 5 s profile rises for 3 s and falls for the remaining 2 s."""
    times, values = trapezoidal_velocity_profile(5.0, 3.0, 2.0, 50.0, 8.0)
    assert len(times) == 251
    assert times[-1] == 5.0
    assert values[0] == 0.0
    assert values[-1] == 0.0
    assert values[150] == 8.0
    assert np.all(np.diff(values[:151]) >= 0.0)
    assert np.all(np.diff(values[150:]) <= 0.0)
    assert np.trapezoid(values, times) == pytest.approx(20.0)


def test_return_home_cycle_has_zero_net_displacement():
    """The mirrored return leg cancels the contraction displacement."""
    times, values = trapezoidal_velocity_profile(5.0, 3.0, 2.0, 50.0, 8.0)
    cycle_times, cycle_values = append_return_home(times, values, 1.0, 50.0)
    assert cycle_times[-1] == pytest.approx(11.0)
    assert np.trapezoid(cycle_values, cycle_times) == pytest.approx(0.0)
