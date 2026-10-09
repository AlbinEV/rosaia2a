"""ROS-independent position-loop primitives for controlled arm motion."""

from __future__ import annotations

import math


def position_velocity_reference(
    error: float,
    previous_reference: float,
    dt_s: float,
    proportional_gain: float,
    minimum_velocity: float,
    maximum_velocity: float,
    maximum_acceleration: float,
    tolerance: float,
) -> float:
    """Return a bounded, acceleration-limited encoder velocity reference."""
    values = (
        error, previous_reference, dt_s, proportional_gain,
        minimum_velocity, maximum_velocity, maximum_acceleration, tolerance,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('position controller inputs must be finite')
    if (
        dt_s <= 0.0
        or proportional_gain <= 0.0
        or minimum_velocity < 0.0
        or maximum_velocity <= 0.0
        or minimum_velocity > maximum_velocity
        or maximum_acceleration <= 0.0
        or tolerance < 0.0
    ):
        raise ValueError('position controller bounds are inconsistent')
    if abs(error) <= tolerance:
        desired = 0.0
    else:
        direction = 1.0 if error > 0.0 else -1.0
        magnitude = min(
            maximum_velocity,
            max(minimum_velocity, proportional_gain * abs(error)),
        )
        desired = direction * magnitude
    maximum_delta = maximum_acceleration * dt_s
    delta = max(
        -maximum_delta,
        min(maximum_delta, desired - previous_reference),
    )
    reference = previous_reference + delta
    if desired == 0.0 and abs(reference) <= maximum_delta:
        return 0.0
    return reference
