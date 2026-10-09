"""Close the encoder-velocity loop and publish bounded motor PWM commands."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int16MultiArray, String

from .velocity import FrictionSchedule, reference_blocked, VelocityController


# Position-limit bit 64 is handled directionally below, so a command away
# from a boundary remains possible and can return the arm to home.
REJECTED_FAULT_MASK = 2 | 8 | 32


class EncoderVelocityController(Node):
    """Track independent encoder velocity references for both primary motors."""

    def __init__(self) -> None:
        super().__init__('encoder_velocity_controller')
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('reference_topics', [
            '/arm_1/encoder_velocity_reference',
            '/arm_2/encoder_velocity_reference',
        ])
        self.declare_parameter('encoder_signs', [1, -1])
        self.declare_parameter('command_to_q_signs', [-1, -1])
        self.declare_parameter('minimum_positions', [0.0, 0.0])
        self.declare_parameter('maximum_positions', [300.0, 300.0])
        self.declare_parameter('position_guard_counts', [10.0, 10.0])
        self.declare_parameter('breakaway_pwm', [150.0, 122.0])
        self.declare_parameter('running_pwm', [140.0, 105.0])
        self.declare_parameter('feedforward_pwm_per_count_s', [0.6, 0.6])
        self.declare_parameter('proportional_gain', [0.4, 0.4])
        self.declare_parameter('integral_gain', [0.15, 0.15])
        self.declare_parameter('maximum_pwm', [200, 200])
        self.declare_parameter('movement_velocity_threshold', [2.0, 2.0])
        self.declare_parameter('movement_hold_s', [0.25, 0.25])
        self.declare_parameter('maximum_pwm_rate', [400.0, 400.0])
        self.declare_parameter('maximum_pwm_fall_rate', [2000.0, 2000.0])
        for axis in range(2):
            for direction in ('contraction', 'release'):
                prefix = f'axis_{axis}_{direction}'
                self.declare_parameter(f'{prefix}_positions', [0.0])
                self.declare_parameter(f'{prefix}_breakaway_pwm', [0.0])
                self.declare_parameter(f'{prefix}_running_pwm', [0.0])
        self.declare_parameter('velocity_filter_alpha', 0.35)
        self.declare_parameter('maximum_reference_age_s', 0.15)
        self.declare_parameter('maximum_telemetry_age_s', 0.10)
        self.declare_parameter('initial_pwm_scale', 1.0)

        self._rate = float(self.get_parameter('control_rate_hz').value)
        topics = self._strings('reference_topics')
        self._encoder_signs = self._integers('encoder_signs')
        command_signs = self._integers('command_to_q_signs')
        self._minimum_positions = self._floats('minimum_positions')
        self._maximum_positions = self._floats('maximum_positions')
        self._position_guards = self._floats('position_guard_counts')
        breakaway = self._floats('breakaway_pwm')
        running = self._floats('running_pwm')
        feedforward = self._floats('feedforward_pwm_per_count_s')
        proportional = self._floats('proportional_gain')
        integral = self._floats('integral_gain')
        maximum_pwm = self._integers('maximum_pwm')
        movement_threshold = self._floats('movement_velocity_threshold')
        movement_hold = self._floats('movement_hold_s')
        maximum_pwm_rate = self._floats('maximum_pwm_rate')
        maximum_pwm_fall_rate = self._floats('maximum_pwm_fall_rate')
        groups = (
            topics, self._encoder_signs, command_signs,
            self._minimum_positions, self._maximum_positions,
            self._position_guards,
            breakaway, running, feedforward, proportional, integral,
            maximum_pwm, movement_threshold, movement_hold, maximum_pwm_rate,
            maximum_pwm_fall_rate,
        )
        if self._rate <= 0.0 or any(len(values) != 2 for values in groups):
            raise ValueError('controller rate must be positive and arrays length 2')
        if any(sign not in (-1, 1) for sign in self._encoder_signs):
            raise ValueError('encoder_signs entries must be -1 or 1')
        if any(
            lower >= upper
            for lower, upper in zip(
                self._minimum_positions, self._maximum_positions
            )
        ):
            raise ValueError('each minimum position must be below its maximum')
        if any(
            guard < 0.0 or lower + guard >= upper - guard
            for lower, upper, guard in zip(
                self._minimum_positions,
                self._maximum_positions,
                self._position_guards,
            )
        ):
            raise ValueError('position guards are inconsistent with bounds')

        self._controllers = [
            VelocityController(
                command_to_q_sign=command_signs[index],
                breakaway_pwm=breakaway[index],
                running_pwm=running[index],
                feedforward_pwm_per_count_s=feedforward[index],
                proportional_gain=proportional[index],
                integral_gain=integral[index],
                maximum_pwm=maximum_pwm[index],
                movement_velocity_threshold=movement_threshold[index],
                movement_hold_s=movement_hold[index],
                maximum_pwm_rate=maximum_pwm_rate[index],
                maximum_pwm_fall_rate=maximum_pwm_fall_rate[index],
                positive_friction=self._friction_schedule(
                    index, 'contraction'
                ),
                negative_friction=self._friction_schedule(index, 'release'),
            )
            for index in range(2)
        ]
        self._alpha = float(
            self.get_parameter('velocity_filter_alpha').value
        )
        self._maximum_reference_age = float(
            self.get_parameter('maximum_reference_age_s').value
        )
        self._maximum_telemetry_age = float(
            self.get_parameter('maximum_telemetry_age_s').value
        )
        if not 0.0 < self._alpha <= 1.0:
            raise ValueError('velocity_filter_alpha must be in (0, 1]')
        if min(
            self._maximum_reference_age, self._maximum_telemetry_age
        ) <= 0.0:
            raise ValueError('freshness limits must be positive')

        self._references = [0.0, 0.0]
        self._reference_times = [None, None]
        self._positions = None
        self._velocities = [0.0, 0.0]
        self._last_raw_positions = None
        self._last_telemetry_time = None
        self._firmware_flags = 0
        self._last_control_time = time.monotonic()
        self._last_reason = 'waiting_for_telemetry'
        self._pwm_scale = float(
            self.get_parameter('initial_pwm_scale').value
        )
        if not 0.8 <= self._pwm_scale <= 1.2:
            raise ValueError('initial_pwm_scale must be in [0.8, 1.2]')
        self._last_commands = [0, 0]

        self._command_publisher = self.create_publisher(
            Int16MultiArray, '/motor_command', 20
        )
        self._status_publisher = self.create_publisher(
            String, '/encoder_velocity_controller/status', 10
        )
        self._subscriptions = [
            self.create_subscription(String, '/motor_state', self._on_state, 50),
            self.create_subscription(
                Float64, '/motor_pwm_scale', self._on_pwm_scale, 10
            ),
        ]
        for axis, topic in enumerate(topics):
            self._subscriptions.append(self.create_subscription(
                Float64,
                topic,
                lambda message, selected=axis: self._on_reference(
                    selected, message
                ),
                20,
            ))
        self._cycle = 0
        self._timer = self.create_timer(1.0 / self._rate, self._control_cycle)
        self.get_logger().info(
            'encoder velocity controller active; waiting for references'
        )

    def _strings(self, name: str) -> list[str]:
        return [str(value) for value in self.get_parameter(name).value]

    def _integers(self, name: str) -> list[int]:
        return [int(value) for value in self.get_parameter(name).value]

    def _floats(self, name: str) -> list[float]:
        return [float(value) for value in self.get_parameter(name).value]

    def _friction_schedule(
        self, axis: int, direction: str
    ) -> FrictionSchedule | None:
        prefix = f'axis_{axis}_{direction}'
        positions = tuple(self._floats(f'{prefix}_positions'))
        breakaway = tuple(self._floats(f'{prefix}_breakaway_pwm'))
        running = tuple(self._floats(f'{prefix}_running_pwm'))
        # A single all-zero entry explicitly selects scalar fallback values.
        if positions == (0.0,) and breakaway == (0.0,) and running == (0.0,):
            return None
        return FrictionSchedule(positions, breakaway, running)

    def _on_reference(self, axis: int, message: Float64) -> None:
        value = float(message.data)
        if not (-1000.0 <= value <= 1000.0):
            self.get_logger().error(f'axis {axis}: invalid velocity reference')
            return
        self._references[axis] = value
        self._reference_times[axis] = time.monotonic()

    def _on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            raw = [int(value) for value in payload['encoder_counts']]
            flags = int(payload['flags'])
            if len(raw) != 2:
                raise ValueError('encoder_counts length is not two')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'ignored invalid motor_state: {error}')
            return
        now = time.monotonic()
        positions = [
            sign * value for sign, value in zip(self._encoder_signs, raw)
        ]
        if self._last_raw_positions is not None and now > self._last_telemetry_time:
            dt_s = now - self._last_telemetry_time
            measured = [
                sign * (value - previous) / dt_s
                for sign, value, previous in zip(
                    self._encoder_signs, raw, self._last_raw_positions
                )
            ]
            self._velocities = [
                (1.0 - self._alpha) * old + self._alpha * new
                for old, new in zip(self._velocities, measured)
            ]
        self._positions = positions
        self._last_raw_positions = raw
        self._last_telemetry_time = now
        self._firmware_flags = flags

    def _on_pwm_scale(self, message: Float64) -> None:
        """Apply a bounded global PWM multiplier only while output is zero."""
        scale = float(message.data)
        if not 0.8 <= scale <= 1.2:
            self.get_logger().error('rejected PWM scale outside [0.8, 1.2]')
            return
        if any(self._last_commands):
            self.get_logger().warning('rejected PWM scale while motors are active')
            return
        self._pwm_scale = scale
        self.get_logger().info(f'PWM output scale set to {scale:.2f}')

    def _zero_commands(self, reason: str) -> list[int]:
        for controller in self._controllers:
            controller.reset()
        self._last_reason = reason
        return [0, 0]

    def _control_cycle(self) -> None:
        now = time.monotonic()
        dt_s = max(1e-6, now - self._last_control_time)
        self._last_control_time = now
        if self._positions is None or self._last_telemetry_time is None:
            commands = self._zero_commands('waiting_for_telemetry')
        elif now - self._last_telemetry_time > self._maximum_telemetry_age:
            commands = self._zero_commands('stale_telemetry')
        elif self._firmware_flags & REJECTED_FAULT_MASK:
            commands = self._zero_commands('firmware_fault')
        else:
            commands = []
            self._last_reason = 'tracking'
            for axis in range(2):
                reference = self._references[axis]
                position = self._positions[axis]
                reference_stale = (
                    self._reference_times[axis] is None
                    or now - self._reference_times[axis]
                    > self._maximum_reference_age
                )
                blocked = reference_blocked(
                    reference,
                    position,
                    self._minimum_positions[axis],
                    self._maximum_positions[axis],
                    self._position_guards[axis],
                )
                if reference_stale:
                    self._controllers[axis].reset()
                    commands.append(0)
                    self._last_reason = f'axis_{axis}_stale_reference'
                elif blocked:
                    self._controllers[axis].reset()
                    commands.append(0)
                    self._last_reason = f'axis_{axis}_position_limit'
                else:
                    commands.append(self._controllers[axis].update(
                        reference,
                        self._velocities[axis],
                        dt_s,
                        position=position,
                        output_scale=self._pwm_scale,
                    ))
        self._last_commands = commands
        message = Int16MultiArray()
        message.data = commands
        self._command_publisher.publish(message)
        self._cycle += 1
        if self._cycle % max(1, round(self._rate)) == 0:
            status = String()
            status.data = json.dumps({
                'reason': self._last_reason,
                'position': self._positions,
                'velocity': self._velocities,
                'reference': self._references,
                'command': commands,
                'pwm_scale': self._pwm_scale,
            }, separators=(',', ':'))
            self._status_publisher.publish(status)

    def destroy_node(self):
        if rclpy.ok():
            message = Int16MultiArray()
            message.data = [0, 0]
            for _ in range(3):
                self._command_publisher.publish(message)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the two-axis encoder velocity controller."""
    rclpy.init(args=args)
    node = EncoderVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError, ValueError):
            pass
        if rclpy.ok():
            rclpy.shutdown()
