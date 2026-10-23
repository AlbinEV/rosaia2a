"""Tests for portable MLP inference and analytic differentiation."""

import numpy as np

from rosaia_learning.model import NumpyMlpModel


def example_model() -> NumpyMlpModel:
    """Construct a deterministic nonlinear test network."""
    return NumpyMlpModel(
        q_mean=2.0,
        q_scale=3.0,
        x_mean=np.arange(6, dtype=np.float64),
        x_scale=np.linspace(1.0, 2.0, 6),
        weights=(
            np.array([[0.5, -0.25]]),
            np.arange(12, dtype=np.float64).reshape(2, 6) / 10.0,
        ),
        biases=(np.array([0.1, -0.2]), np.zeros(6)),
        q_min=0.0,
        q_max=10.0,
    )


def test_analytic_jacobian_matches_central_difference():
    """The exported Jacobian agrees with a numerical derivative."""
    model = example_model()
    q_value = 4.0
    epsilon = 1e-6
    _, analytic = model.predict_and_jacobian(q_value)
    upper, _ = model.predict_and_jacobian(q_value + epsilon)
    lower, _ = model.predict_and_jacobian(q_value - epsilon)
    numerical = (upper - lower) / (2.0 * epsilon)
    np.testing.assert_allclose(analytic, numerical, rtol=1e-6, atol=1e-8)


def test_save_load_round_trip(tmp_path):
    """NPZ serialization preserves inference exactly."""
    model = example_model()
    model.save(tmp_path, metadata={'purpose': 'test'})
    loaded = NumpyMlpModel.load(tmp_path)
    expected = model.predict_and_jacobian(3.5)
    actual = loaded.predict_and_jacobian(3.5)
    np.testing.assert_allclose(expected[0], actual[0])
    np.testing.assert_allclose(expected[1], actual[1])
