"""ROS-independent encoder-velocity control primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FrictionSchedule:
    """Piecewise-linear breakaway/running PWM versus operational position."""

    positions: tuple[float, ...]
    breakaway_pwm: tuple[float, ...]
    running_pwm: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.positions or not (
            len(self.positions)
            == len(self.breakaway_pwm)
            == len(self.running_pwm)
        ):
            raise ValueError('friction schedule arrays must have equal length')
        if any(
            right <= left
            for left, right in zip(self.positions, self.positions[1:])
        ):
            raise ValueError('friction schedule positions must increase')
        if any(
            not 0.0 <= running <= breakaway <= 255.0
            for breakaway, running in zip(
                self.breakaway_pwm, self.running_pwm
            )
        ):
            raise ValueError('friction schedule PWM values are inconsistent')

    def sample(self, position: float) -> tuple[float, float]:
        """Return clamped linear interpolation (breakaway, running)."""
        if not math.isfinite(position):
            raise ValueError('position must be finite')
        if position <= self.positions[0]:
            return self.breakaway_pwm[0], self.running_pwm[0]
        if position >= self.positions[-1]:
            return self.breakaway_pwm[-1], self.running_pwm[-1]
        for index, right in enumerate(self.positions[1:], start=1):
            if position <= right:
                left = self.positions[index - 1]
                fraction = (position - left) / (right - left)
                breakaway = self.breakaway_pwm[index - 1] + fraction * (
                    self.breakaway_pwm[index]
                    - self.breakaway_pwm[index - 1]
                )
                running = self.running_pwm[index - 1] + fraction * (
                    self.running_pwm[index] - self.running_pwm[index - 1]
                )
                return breakaway, running
        raise AssertionError('unreachable interpolation branch')


@dataclass
class VelocityController:
    """PI plus breakaway feedforward for one signed motor channel."""

    command_to_q_sign: int
    breakaway_pwm: float
    running_pwm: float
    feedforward_pwm_per_count_s: float
    proportional_gain: float
    integral_gain: float
    maximum_pwm: int
    movement_velocity_threshold: float = 2.0
    movement_hold_s: float = 0.25
    maximum_pwm_rate: float = 400.0
    maximum_pwm_fall_rate: float = 2000.0
    reference_deadband: float = 0.05
    integral_limit: float = 200.0
    positive_friction: FrictionSchedule | None = None
    negative_friction: FrictionSchedule | None = None
    integral: float = 0.0
    reference_direction: int = 0
    movement_hold_remaining: float = 0.0
    last_command: float = 0.0

    def __post_init__(self) -> None:
        if self.command_to_q_sign not in (-1, 1):
            raise ValueError('command_to_q_sign must be -1 or 1')
        if not (
            0.0 <= self.running_pwm <= self.breakaway_pwm
            <= self.maximum_pwm <= 255
        ):
            raise ValueError('PWM bounds are inconsistent')
        if min(
            self.feedforward_pwm_per_count_s,
            self.proportional_gain,
            self.integral_gain,
            self.movement_velocity_threshold,
            self.movement_hold_s,
            self.reference_deadband,
            self.integral_limit,
        ) < 0.0:
            raise ValueError('controller gains and limits must be non-negative')
        if self.maximum_pwm_rate <= 0.0 or self.maximum_pwm_fall_rate <= 0.0:
            raise ValueError('PWM slew rates must be positive')

    def reset(self) -> None:
        """Clear dynamic state after a stop, stale input or direction change."""
        self.integral = 0.0
        self.reference_direction = 0
        self.movement_hold_remaining = 0.0
        self.last_command = 0.0

    def update(
        self,
        reference: float,
        measured: float,
        dt_s: float,
        position: float | None = None,
        output_scale: float = 1.0,
    ) -> int:
        """Return a signed integer PWM command for an operational velocity."""
        if not all(
            math.isfinite(value)
            for value in (reference, measured, dt_s, output_scale)
        ):
            raise ValueError('velocity controller inputs must be finite')
        if dt_s <= 0.0:
            raise ValueError('dt_s must be positive')
        if not 0.5 <= output_scale <= 1.5:
            raise ValueError('output_scale must be in [0.5, 1.5]')
        if abs(reference) <= self.reference_deadband:
            self.reset()
            return 0

        direction = 1 if reference > 0.0 else -1
        if self.reference_direction not in (0, direction):
            self.integral = 0.0
            self.movement_hold_remaining = 0.0
            self.last_command = 0.0
        self.reference_direction = direction
        if abs(measured) >= self.movement_velocity_threshold:
            self.movement_hold_remaining = self.movement_hold_s
        else:
            self.movement_hold_remaining = max(
                0.0, self.movement_hold_remaining - dt_s
            )
        breakaway_pwm = self.breakaway_pwm
        running_pwm = self.running_pwm
        schedule = (
            self.positive_friction if direction > 0 else self.negative_friction
        )
        if schedule is not None:
            if position is None:
                raise ValueError('position is required by friction schedule')
            breakaway_pwm, running_pwm = schedule.sample(position)
        base_pwm = (
            running_pwm
            if self.movement_hold_remaining > 0.0
            else breakaway_pwm
        )
        measured_along_reference = direction * measured
        error = abs(reference) - measured_along_reference
        candidate_integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * dt_s),
        )
        candidate_magnitude = (
            base_pwm
            + self.feedforward_pwm_per_count_s * abs(reference)
            + self.proportional_gain * error
            + self.integral_gain * candidate_integral
        )
        drives_high_saturation = (
            candidate_magnitude * output_scale > self.maximum_pwm
            and error > 0.0
        )
        drives_low_saturation = candidate_magnitude < 0.0 and error < 0.0
        if not (drives_high_saturation or drives_low_saturation):
            self.integral = candidate_integral
        magnitude = (
            base_pwm
            + self.feedforward_pwm_per_count_s * abs(reference)
            + self.proportional_gain * error
            + self.integral_gain * self.integral
        )
        magnitude *= output_scale
        magnitude = max(0.0, min(float(self.maximum_pwm), magnitude))
        command_direction = self.command_to_q_sign * direction
        target_command = command_direction * magnitude
        same_direction = target_command * self.last_command >= 0.0
        reducing_magnitude = abs(target_command) < abs(self.last_command)
        slew_rate = (
            self.maximum_pwm_fall_rate
            if same_direction and reducing_magnitude
            else self.maximum_pwm_rate
        )
        maximum_delta = slew_rate * dt_s
        delta = max(
            -maximum_delta,
            min(maximum_delta, target_command - self.last_command),
        )
        self.last_command += delta
        return int(round(self.last_command))


def reference_blocked(
    reference: float,
    position: float,
    lower: float,
    upper: float,
    guard: float,
) -> bool:
    """Return whether a reference points farther into a guarded position limit."""
    if guard < 0.0 or lower + guard >= upper - guard:
        raise ValueError('position guard is inconsistent with bounds')
    return (
        reference > 0.0 and position >= upper - guard
    ) or (
        reference < 0.0 and position <= lower + guard
    )
