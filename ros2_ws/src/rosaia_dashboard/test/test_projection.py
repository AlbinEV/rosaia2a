"""Tests for reachable-shape projection."""

import numpy as np
import pytest

from rosaia_dashboard.projection import (
    project_curvature_goal,
    project_planar_goal,
    signed_planar_curvature,
)
from rosaia_learning.model import NumpyMlpModel


def linear_model():
    """Return a deterministic one-dimensional six-state manifold."""
    slopes = np.array([1.0, 0.5, 2.0, -0.25, 3.0, 0.1])
    return NumpyMlpModel(
        q_mean=0.0,
        q_scale=1.0,
        x_mean=np.zeros(6),
        x_scale=np.ones(6),
        weights=(slopes.reshape(1, 6),),
        biases=(np.zeros(6),),
        q_min=0.0,
        q_max=10.0,
    )


def test_projection_recovers_reachable_coordinate():
    """A goal already on the manifold projects back to its q value."""
    model = linear_model()
    requested, _ = model.predict_and_jacobian(3.2)
    result = project_planar_goal(model, requested)
    assert result.q == pytest.approx(3.2, abs=1e-6)
    np.testing.assert_allclose(result.state, requested, atol=1e-6)
    assert result.weighted_error < 1e-6


def test_projection_clamps_unreachable_goal_to_range():
    """An out-of-range shape maps to the nearest endpoint."""
    model = linear_model()
    requested, _ = model.predict_and_jacobian(15.0)
    result = project_planar_goal(model, requested)
    assert result.q == pytest.approx(10.0, abs=1e-6)


def test_projection_rejects_invalid_weights():
    """At least one finite non-negative shape weight is required."""
    with pytest.raises(ValueError, match='non-negative'):
        project_planar_goal(linear_model(), np.zeros(6), component_weights=[0] * 6)


def test_current_q_resolves_flat_manifold_ambiguity():
    """A current-encoder prior selects the local point on a flat manifold."""
    model = linear_model()
    model.weights = (np.zeros((1, 6)),)
    requested, _ = model.predict_and_jacobian(0.0)
    result = project_planar_goal(
        model,
        requested,
        current_q=7.25,
        q_regularization=1e-8,
    )
    assert result.q == pytest.approx(7.25, abs=1e-6)
    assert result.weighted_error < 1e-12


def test_regularization_requires_current_q():
    """An active encoder prior cannot silently run without encoder state."""
    with pytest.raises(ValueError, match='current q is required'):
        project_planar_goal(
            linear_model(),
            np.zeros(6),
            q_regularization=1e-8,
        )


def test_shape_difference_attenuates_encoder_prior():
    """A meaningfully different shape dominates the continuity prior."""
    model = linear_model()
    requested, _ = model.predict_and_jacobian(8.0)
    result = project_planar_goal(
        model,
        requested,
        current_q=0.0,
        q_regularization=1.0,
        q_regularization_transition_m=0.1,
    )
    assert result.q == pytest.approx(8.0, abs=1e-4)
    assert result.q_regularization < 1e-8


def test_signed_curvature_is_zero_for_straight_markers():
    """Collinear marker centres represent a straight backbone."""
    state = np.array([0.0, 1.0, 0.0, 2.0, 0.0, 3.0])
    assert signed_planar_curvature(state) == pytest.approx(0.0, abs=1e-12)


def test_curvature_projection_recovers_model_coordinate():
    """A reachable curvature maps back onto its learned manifold."""
    model = linear_model()
    target_state, _ = model.predict_and_jacobian(4.0)
    curvature = signed_planar_curvature(target_state)
    result = project_curvature_goal(
        model, curvature, current_q=4.0, curvature_tolerance=0.0
    )
    assert result.q == pytest.approx(4.0, abs=0.02)
