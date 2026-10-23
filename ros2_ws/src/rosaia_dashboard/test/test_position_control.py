"""Tests for controlled encoder-position motion primitives."""

import pytest

from rosaia_dashboard.position_control import position_velocity_reference


def reference(error, previous=0.0, dt_s=0.1):
    """Evaluate the standard test controller."""
    return position_velocity_reference(
        error=error,
        previous_reference=previous,
        dt_s=dt_s,
        proportional_gain=0.5,
        minimum_velocity=2.0,
        maximum_velocity=20.0,
        maximum_acceleration=10.0,
        tolerance=2.0,
    )


def test_reference_accelerates_toward_target() -> None:
    """A distant target ramps positively without an impulsive step."""
    assert reference(100.0) == pytest.approx(1.0)
    assert reference(100.0, previous=1.0) == pytest.approx(2.0)


def test_reference_is_speed_limited() -> None:
    """The proportional request cannot exceed maximum velocity."""
    assert reference(100.0, previous=19.5) == pytest.approx(20.0)


def test_reference_brakes_inside_tolerance() -> None:
    """Entering the target band ramps the velocity reference to zero."""
    assert reference(1.0, previous=5.0) == pytest.approx(4.0)
    assert reference(1.0, previous=0.5) == pytest.approx(0.0)


def test_negative_error_generates_controlled_reverse_reference() -> None:
    """An overshoot can be corrected in the opposite direction."""
    assert reference(-20.0) == pytest.approx(-1.0)


@pytest.mark.parametrize('dt_s', [0.0, -0.1])
def test_invalid_period_is_rejected(dt_s) -> None:
    """Non-positive controller periods are invalid."""
    with pytest.raises(ValueError, match='inconsistent'):
        reference(10.0, dt_s=dt_s)
