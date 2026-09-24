"""One-dimensional projection of requested shapes onto a learned manifold."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ProjectionResult:
    """Closest feasible planar shape and its motor coordinate."""

    q: float
    state: np.ndarray
    weighted_error: float
    q_regularization: float = 0.0


@dataclass(frozen=True)
class CurvatureProjectionResult:
    """Nearest model state for one requested signed planar curvature."""

    q: float
    state: np.ndarray
    requested_curvature: float
    achieved_curvature: float


def signed_planar_curvature(state) -> float:
    """Fit a signed circle through the base and three ordered markers."""
    values = np.asarray(state, dtype=np.float64).reshape(-1)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('state must contain six finite planar coordinates')
    points = np.vstack((np.zeros((1, 2)), values.reshape(3, 2)))
    matrix = np.column_stack((2.0 * points, np.ones(4)))
    rhs = np.sum(points * points, axis=1)
    solution, _, rank, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
    if rank < 3:
        return 0.0
    center = solution[:2]
    radius_squared = float(solution[2] + center @ center)
    if radius_squared <= 1e-12:
        return 0.0
    segments = np.diff(points, axis=0)
    turns = [
        (
            segments[index, 0] * segments[index + 1, 1]
            - segments[index, 1] * segments[index + 1, 0]
        )
        for index in range(len(segments) - 1)
    ]
    orientation = float(np.sum(turns))
    if abs(orientation) <= 1e-12:
        return 0.0
    return float(np.sign(orientation) / np.sqrt(radius_squared))


def project_curvature_goal(
    model,
    requested_curvature: float,
    *,
    current_q: float | None = None,
    coarse_samples: int = 1001,
    curvature_tolerance: float = 0.02,
) -> CurvatureProjectionResult:
    """Select the closest reachable curvature, preferring q continuity."""
    curvature = float(requested_curvature)
    if not np.isfinite(curvature):
        raise ValueError('requested curvature must be finite')
    if current_q is not None and not np.isfinite(current_q):
        raise ValueError('current q must be finite')
    if coarse_samples < 3 or curvature_tolerance < 0.0:
        raise ValueError('invalid curvature projection resolution')
    grid = np.linspace(float(model.q_min), float(model.q_max), coarse_samples)
    states = np.vstack([
        model.predict_and_jacobian(value)[0] for value in grid
    ])
    curvatures = np.array([
        signed_planar_curvature(state) for state in states
    ])
    errors = np.abs(curvatures - curvature)
    candidates = np.flatnonzero(
        errors <= float(np.min(errors)) + curvature_tolerance
    )
    if current_q is None:
        best = int(candidates[np.argmin(errors[candidates])])
    else:
        best = int(candidates[np.argmin(np.abs(grid[candidates] - current_q))])
    return CurvatureProjectionResult(
        q=float(grid[best]),
        state=states[best],
        requested_curvature=curvature,
        achieved_curvature=float(curvatures[best]),
    )


def project_planar_goal(
    model,
    requested_state,
    *,
    component_weights=None,
    current_q: float | None = None,
    q_regularization: float = 0.0,
    q_regularization_transition_m: float | None = None,
    coarse_samples: int = 501,
    refinement_iterations: int = 40,
) -> ProjectionResult:
    """Minimize shape error with an optional current-encoder prior."""
    requested = np.asarray(requested_state, dtype=np.float64).reshape(-1)
    if requested.shape != (6,) or not np.isfinite(requested).all():
        raise ValueError('requested state must contain six finite values')
    weights = (
        np.ones(6, dtype=np.float64)
        if component_weights is None
        else np.asarray(component_weights, dtype=np.float64).reshape(-1)
    )
    if weights.shape != (6,) or not np.isfinite(weights).all():
        raise ValueError('component weights must contain six finite values')
    if np.any(weights < 0.0) or not np.any(weights > 0.0):
        raise ValueError('component weights must be non-negative and nonzero')
    if not np.isfinite(q_regularization) or q_regularization < 0.0:
        raise ValueError('q regularization must be finite and non-negative')
    if current_q is not None and not np.isfinite(current_q):
        raise ValueError('current q must be finite')
    if q_regularization > 0.0 and current_q is None:
        raise ValueError('current q is required when regularization is active')
    if (
        q_regularization_transition_m is not None
        and (
            not np.isfinite(q_regularization_transition_m)
            or q_regularization_transition_m <= 0.0
        )
    ):
        raise ValueError('q regularization transition must be positive')
    if coarse_samples < 3 or refinement_iterations < 0:
        raise ValueError('invalid projection resolution')

    lower = float(model.q_min)
    upper = float(model.q_max)
    if not np.isfinite([lower, upper]).all() or lower > upper:
        raise ValueError('model has an invalid q range')

    effective_regularization = q_regularization
    if q_regularization > 0.0 and q_regularization_transition_m is not None:
        current_state, _ = model.predict_and_jacobian(float(current_q))
        current_error = (np.asarray(current_state) - requested) * weights
        shape_distance = float(np.sqrt(current_error @ current_error))
        ratio = shape_distance / q_regularization_transition_m
        effective_regularization /= 1.0 + ratio ** 4

    def evaluate(q_value: float) -> tuple[float, np.ndarray]:
        state, _ = model.predict_and_jacobian(float(q_value))
        error = (np.asarray(state) - requested) * weights
        cost = float(error @ error)
        if effective_regularization > 0.0:
            cost += effective_regularization * (q_value - current_q) ** 2
        return cost, np.asarray(state, dtype=np.float64)

    grid = np.linspace(lower, upper, coarse_samples)
    costs = np.array([evaluate(value)[0] for value in grid])
    best = int(np.argmin(costs))
    left = float(grid[max(0, best - 1)])
    right = float(grid[min(len(grid) - 1, best + 1)])

    ratio = (np.sqrt(5.0) - 1.0) / 2.0
    c = right - ratio * (right - left)
    d = left + ratio * (right - left)
    cost_c, _ = evaluate(c)
    cost_d, _ = evaluate(d)
    for _ in range(refinement_iterations):
        if cost_c <= cost_d:
            right, d, cost_d = d, c, cost_c
            c = right - ratio * (right - left)
            cost_c, _ = evaluate(c)
        else:
            left, c, cost_c = c, d, cost_d
            d = left + ratio * (right - left)
            cost_d, _ = evaluate(d)
    q_value = (left + right) / 2.0
    _, state = evaluate(q_value)
    shape_error = (state - requested) * weights
    return ProjectionResult(
        q_value,
        state,
        float(np.sqrt(shape_error @ shape_error)),
        effective_regularization,
    )
