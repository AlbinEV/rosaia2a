"""ROS 2 node bridging motor commands and Bluno serial telemetry."""

import json
import time

import rclpy
from rclpy.node import Node
import serial
from serial import SerialException
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int64MultiArray, String
from std_srvs.srv import Trigger

from .protocol import (
    adc_to_current_a,
    CURRENT_UNAVAILABLE_MA,
    encode_command,
    encode_step_axis,
    encode_stop,
    encode_zero_encoders,
    parse_state,
    ProtocolError,
)


class BlunoMotorBridge(Node):
    """Fail-safe, reconnecting serial bridge for two logical primary motors."""

    def __init__(self) -> None:
        super().__init__('bluno_motor_bridge')

        self.declare_parameter('serial_port', '')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_rate_hz', 100.0)
        self.declare_parameter('command_timeout_ms', 300)
        self.declare_parameter('reconnect_period_s', 1.0)
        self.declare_parameter('max_abs_command', 0)
        self.declare_parameter('max_relative_step_counts', 10)
        self.declare_parameter('relative_step_timeout_s', 3.5)
        self.declare_parameter('active_axes', [True, False])
        self.declare_parameter('motor_sign', [1, 1])
        self.declare_parameter('axis_names', ['arm_1_primary', 'arm_2_primary'])
        self.declare_parameter('shield_channels', ['M2', 'M1'])
        self.declare_parameter('axis_0_encoder_cpr', -1)
        self.declare_parameter('axis_1_encoder_cpr', -1)
        self.declare_parameter('axis_0_gear_ratio', -1.0)
        self.declare_parameter('axis_1_gear_ratio', -1.0)
        self.declare_parameter('axis_0_current_sensitivity_v_per_a', -1.0)
        self.declare_parameter('axis_1_current_sensitivity_v_per_a', -1.0)
        self.declare_parameter('current_zero_adc', [512.0, 512.0])
        self.declare_parameter('current_polarity', [1, 1])
        self.declare_parameter('adc_reference_voltage_v', 5.0)
        self.declare_parameter('auto_current_zero', True)
        self.declare_parameter('auto_current_zero_samples', 25)
        self.declare_parameter('current_deadband_adc_counts', 1.0)

        self._port = str(self.get_parameter('serial_port').value)
        self._baud = int(self.get_parameter('baud_rate').value)
        self._rate_hz = float(self.get_parameter('command_rate_hz').value)
        self._timeout_s = float(self.get_parameter('command_timeout_ms').value) / 1000.0
        self._reconnect_s = float(self.get_parameter('reconnect_period_s').value)
        self._limit = int(self.get_parameter('max_abs_command').value)
        self._step_limit = int(
            self.get_parameter('max_relative_step_counts').value
        )
        self._step_timeout_s = float(
            self.get_parameter('relative_step_timeout_s').value
        )
        self._active_axes = tuple(bool(v) for v in self.get_parameter('active_axes').value)
        self._motor_sign = tuple(int(v) for v in self.get_parameter('motor_sign').value)
        self._axis_names = tuple(str(v) for v in self.get_parameter('axis_names').value)
        self._shield_channels = tuple(
            str(v) for v in self.get_parameter('shield_channels').value
        )
        self._current_sensitivity = (
            float(self.get_parameter(
                'axis_0_current_sensitivity_v_per_a'
            ).value),
            float(self.get_parameter(
                'axis_1_current_sensitivity_v_per_a'
            ).value),
        )
        self._current_zero_adc = tuple(
            float(v) for v in self.get_parameter('current_zero_adc').value
        )
        self._current_polarity = tuple(
            int(v) for v in self.get_parameter('current_polarity').value
        )
        self._adc_reference_v = float(
            self.get_parameter('adc_reference_voltage_v').value
        )
        self._auto_current_zero = bool(
            self.get_parameter('auto_current_zero').value
        )
        self._auto_current_zero_samples = int(
            self.get_parameter('auto_current_zero_samples').value
        )
        self._current_deadband_adc_counts = float(
            self.get_parameter('current_deadband_adc_counts').value
        )
        self._validate_parameters()

        self._serial = None
        self._firmware_ready = False
        self._rx_buffer = bytearray()
        self._stop_sent = False
        self._sequence = 0
        self._commands = (0, 0)
        self._last_written_commands = (0, 0)
        self._last_applied_commands = (0, 0)
        self._last_command_time = None
        self._last_connect_attempt = -float('inf')
        self._last_state_time = None
        self._last_status_publish = -float('inf')
        self._invalid_packet_count = 0
        self._last_firmware_error = None
        self._effective_current_zero = list(self._current_zero_adc)
        self._current_zero_sums = [0, 0]
        self._current_zero_count = 0
        self._step_active = False
        self._step_seen_motion = False
        self._step_deadline = None

        self._command_sub = self.create_subscription(
            Int16MultiArray, 'motor_command', self._on_command, 10
        )
        self._step_sub = self.create_subscription(
            Int16MultiArray, 'motor_step_command', self._on_step_command, 10
        )
        self._zero_service = self.create_service(
            Trigger, 'zero_encoders', self._on_zero_encoders
        )
        self._encoder_pub = self.create_publisher(
            Int64MultiArray, 'encoder_counts', 10
        )
        self._current_pub = self.create_publisher(
            Float32MultiArray, 'motor_current', 10
        )
        self._state_pub = self.create_publisher(String, 'motor_state', 10)
        self._status_pub = self.create_publisher(String, 'bluno/status', 10)
        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)

        if not self._port:
            self.get_logger().warning(
                'serial_port is empty: bridge is locked in disconnected STOP state'
            )
        if self._limit == 0:
            self.get_logger().warning(
                'max_abs_command is 0: all non-zero motor commands are rejected'
            )

    def _validate_parameters(self) -> None:
        if self._baud <= 0:
            raise ValueError('baud_rate must be positive')
        if self._rate_hz <= 0.0:
            raise ValueError('command_rate_hz must be positive')
        if self._timeout_s <= 0.0 or self._reconnect_s <= 0.0:
            raise ValueError('timeouts must be positive')
        if not 0 <= self._limit <= 255:
            raise ValueError('max_abs_command must be in [0, 255]')
        if not 1 <= self._step_limit <= 50:
            raise ValueError('max_relative_step_counts must be in [1, 50]')
        if self._step_timeout_s <= 3.0:
            raise ValueError('relative_step_timeout_s must exceed firmware timeout')
        for name, values in (
            ('active_axes', self._active_axes),
            ('motor_sign', self._motor_sign),
            ('axis_names', self._axis_names),
            ('shield_channels', self._shield_channels),
        ):
            if len(values) != 2:
                raise ValueError(f'{name} must contain exactly two entries')
        if any(sign not in (-1, 1) for sign in self._motor_sign):
            raise ValueError('motor_sign entries must be -1 or 1')
        if self._shield_channels != ('M2', 'M1'):
            raise ValueError('current board contract requires axis mapping [M2, M1]')
        if len(self._current_zero_adc) != 2 or any(
            not 0.0 <= value <= 1023.0 for value in self._current_zero_adc
        ):
            raise ValueError('current_zero_adc must contain two valid ADC values')
        if len(self._current_polarity) != 2 or any(
            value not in (-1, 1) for value in self._current_polarity
        ):
            raise ValueError('current_polarity must contain two signs')
        if self._adc_reference_v <= 0.0:
            raise ValueError('adc_reference_voltage_v must be positive')
        if self._auto_current_zero_samples <= 0:
            raise ValueError('auto_current_zero_samples must be positive')
        if self._current_deadband_adc_counts < 0.0:
            raise ValueError('current ADC deadband must be non-negative')
        if any(value == 0.0 for value in self._current_sensitivity):
            raise ValueError('current sensitivity must be positive or negative if unknown')

    def _on_command(self, message: Int16MultiArray) -> None:
        if len(message.data) != 2:
            self.get_logger().error('motor_command must contain exactly two integers')
            return

        requested = tuple(int(value) for value in message.data)
        if requested == (0, 0) and self._step_active:
            try:
                if self._serial is not None:
                    self._serial.write(encode_stop())
                    self._serial.flush()
            except (OSError, SerialException) as exc:
                self._disconnect(str(exc))
                return
            self._step_active = False
            self._step_seen_motion = False
            self._step_deadline = None
            self._last_written_commands = (0, 0)
            self._stop_sent = True
        if any(abs(value) > self._limit for value in requested):
            self.get_logger().error(
                f'rejected command {requested}: configured limit is {self._limit}'
            )
            return

        self._commands = tuple(
            value * sign if active else 0
            for value, sign, active in zip(
                requested, self._motor_sign, self._active_axes
            )
        )
        self._last_command_time = time.monotonic()

    def _on_step_command(self, message: Int16MultiArray) -> None:
        if len(message.data) != 3:
            self.get_logger().error(
                'motor_step_command must contain [axis, signed_counts, pwm]'
            )
            return
        axis, signed_counts, pwm = (int(value) for value in message.data)
        if axis not in (0, 1) or not self._active_axes[axis]:
            self.get_logger().error(f'rejected step: inactive/invalid axis {axis}')
            return
        if signed_counts == 0 or abs(signed_counts) > self._step_limit:
            self.get_logger().error(
                f'rejected step {signed_counts}: configured limit is '
                f'{self._step_limit} counts'
            )
            return
        if pwm <= 0 or pwm > self._limit:
            self.get_logger().error(
                f'rejected step PWM {pwm}: configured limit is {self._limit}'
            )
            return
        if self._serial is None or self._step_active:
            self.get_logger().error(
                'rejected step: serial disconnected or another step is active'
            )
            return
        try:
            self._serial.write(encode_step_axis(
                self._sequence, axis, signed_counts, pwm
            ))
        except (OSError, SerialException) as exc:
            self._disconnect(str(exc))
            return
        self._sequence = (self._sequence + 1) % (2**31)
        applied = [0, 0]
        applied[axis] = pwm if signed_counts > 0 else -pwm
        self._last_written_commands = tuple(applied)
        self._step_active = True
        self._step_seen_motion = False
        self._step_deadline = time.monotonic() + self._step_timeout_s
        self._commands = (0, 0)
        self._last_command_time = None
        self._stop_sent = False
        self.get_logger().info(
            f'sent bounded step axis={axis} counts={signed_counts} pwm={pwm}'
        )

    def _on_zero_encoders(self, request, response):
        """Zero both encoders only while firmware outputs are stopped."""
        del request
        if self._serial is None or not self._firmware_ready:
            response.success = False
            response.message = 'serial link or firmware is not ready'
            return response
        if (
            self._step_active
            or any(self._commands)
            or any(self._last_written_commands)
            or any(self._last_applied_commands)
        ):
            response.success = False
            response.message = 'motor output or bounded step is active'
            return response
        try:
            self._serial.write(encode_stop())
            self._serial.write(encode_zero_encoders())
            self._serial.flush()
        except (OSError, SerialException) as exc:
            self._disconnect(str(exc))
            response.success = False
            response.message = f'serial write failed: {exc}'
            return response
        self._commands = (0, 0)
        self._last_written_commands = (0, 0)
        self._last_applied_commands = (0, 0)
        self._last_command_time = None
        self._stop_sent = True
        response.success = True
        response.message = 'ZERO_ENC sent; verify encoder telemetry is [0, 0]'
        self.get_logger().info('encoder zero requested at mechanical home')
        return response

    def _connect_if_due(self, now: float) -> None:
        if self._serial is not None or not self._port:
            return
        if now - self._last_connect_attempt < self._reconnect_s:
            return
        self._last_connect_attempt = now
        try:
            self._serial = serial.Serial(
                self._port,
                self._baud,
                timeout=0,
                write_timeout=min(self._timeout_s, 0.1),
            )
            self._serial.reset_input_buffer()
            self._last_written_commands = (0, 0)
            self._last_applied_commands = (0, 0)
            self._rx_buffer.clear()
            self._firmware_ready = False
            self._effective_current_zero = list(self._current_zero_adc)
            self._current_zero_sums = [0, 0]
            self._current_zero_count = 0
            self._stop_sent = False
            self.get_logger().info(
                f'connected to {self._port} at {self._baud} baud; '
                'waiting for firmware boot'
            )
        except (OSError, SerialException) as exc:
            self._serial = None
            self.get_logger().warning(f'cannot open {self._port}: {exc}')

    def _disconnect(self, reason: str) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, SerialException):
                pass
        self._serial = None
        self._firmware_ready = False
        self._commands = (0, 0)
        self._last_written_commands = (0, 0)
        self._last_applied_commands = (0, 0)
        self._last_command_time = None
        self._rx_buffer.clear()
        self._stop_sent = False
        self._step_active = False
        self._step_seen_motion = False
        self._step_deadline = None
        self.get_logger().error(f'serial link closed: {reason}')

    def _tick(self) -> None:
        now = time.monotonic()
        self._connect_if_due(now)

        if self._serial is not None:
            try:
                self._read_available_state(now)
                if not self._firmware_ready:
                    pass
                elif self._step_active:
                    if now >= self._step_deadline:
                        self._serial.write(encode_stop())
                        self._last_written_commands = (0, 0)
                        self._step_active = False
                        self._stop_sent = True
                        self.get_logger().error(
                            'host step deadline reached; STOP sent'
                        )
                else:
                    command_fresh = (
                        self._last_command_time is not None
                        and now - self._last_command_time <= self._timeout_s
                    )
                    if command_fresh:
                        self._serial.write(encode_command(
                            self._sequence, self._commands
                        ))
                        self._last_written_commands = self._commands
                        self._sequence = (self._sequence + 1) % (2**31)
                        self._stop_sent = False
                    elif not self._stop_sent:
                        self._serial.write(encode_stop())
                        self._last_written_commands = (0, 0)
                        self._stop_sent = True
                self._read_available_state(now)
            except (OSError, SerialException) as exc:
                self._disconnect(str(exc))

        if now - self._last_status_publish >= 1.0:
            self._publish_status(now)
            self._last_status_publish = now

    def _read_available_state(self, now: float) -> None:
        available = self._serial.in_waiting
        if available > 0:
            self._rx_buffer.extend(self._serial.read(available))
        if len(self._rx_buffer) > 4096:
            self._invalid_packet_count += 1
            self._rx_buffer.clear()
            self.get_logger().error('serial receive buffer overflow; data discarded')
            return

        for _ in range(20):
            newline = self._rx_buffer.find(b'\n')
            if newline < 0:
                break
            line = bytes(self._rx_buffer[:newline]).rstrip(b'\r')
            del self._rx_buffer[:newline + 1]
            if not line:
                continue
            if line.startswith(b'BOOT,'):
                self._firmware_ready = True
                self.get_logger().info('firmware boot handshake received')
                continue
            if line.startswith((b'OK,', b'PONG,')):
                continue
            if line.startswith(b'ERR,'):
                self._last_firmware_error = line.decode('ascii', errors='replace')
                self.get_logger().error(
                    f'firmware rejected a command: {self._last_firmware_error}'
                )
                continue
            try:
                packet = parse_state(line)
            except ProtocolError as exc:
                self._invalid_packet_count += 1
                self.get_logger().warning(f'ignored malformed telemetry: {exc}')
                continue

            self._last_state_time = now
            self._firmware_ready = True
            if packet.applied_commands is not None:
                self._last_applied_commands = packet.applied_commands
            if (
                self._auto_current_zero
                and self._current_zero_count < self._auto_current_zero_samples
                and packet.applied_commands == (0, 0)
                and self._commands == (0, 0)
            ):
                for axis in range(2):
                    self._current_zero_sums[axis] += packet.raw_adc[axis]
                self._current_zero_count += 1
                if self._current_zero_count == self._auto_current_zero_samples:
                    self._effective_current_zero = [
                        total / self._auto_current_zero_samples
                        for total in self._current_zero_sums
                    ]
                    self.get_logger().info(
                        'host current zero calibrated: '
                        f'{self._effective_current_zero}'
                    )
            if self._step_active and packet.applied_commands is not None:
                if any(packet.applied_commands):
                    self._step_seen_motion = True
                elif self._step_seen_motion:
                    self._step_active = False
                    self._step_deadline = None
                    self._last_written_commands = (0, 0)
                    self._stop_sent = True
                    self.get_logger().info('bounded step completed in firmware')
            encoder_message = Int64MultiArray()
            encoder_message.data = list(packet.encoder_counts)
            self._encoder_pub.publish(encoder_message)

            current_message = Float32MultiArray()
            host_current = []
            for axis, firmware_ma in enumerate(packet.current_ma):
                if firmware_ma != CURRENT_UNAVAILABLE_MA:
                    host_current.append(firmware_ma / 1000.0)
                elif (
                    self._current_sensitivity[axis] > 0.0
                    and (
                        not self._auto_current_zero
                        or self._current_zero_count
                        >= self._auto_current_zero_samples
                    )
                ):
                    host_current.append(adc_to_current_a(
                        packet.raw_adc[axis],
                        self._effective_current_zero[axis],
                        self._adc_reference_v,
                        self._current_sensitivity[axis],
                        self._current_polarity[axis],
                        self._current_deadband_adc_counts,
                    ))
                else:
                    host_current.append(float('nan'))
            current_message.data = host_current
            self._current_pub.publish(current_message)

            state_message = String()
            payload = packet.as_dict()
            if packet.applied_commands is None:
                payload['applied_commands'] = self._last_written_commands
                payload['applied_command_source'] = 'host_write_fallback'
            else:
                payload['applied_command_source'] = 'firmware_output_state'
            payload['requested_commands'] = self._commands
            payload['current_a_host'] = host_current
            payload['axis_names'] = self._axis_names
            payload['shield_channels'] = self._shield_channels
            state_message.data = json.dumps(payload, separators=(',', ':'))
            self._state_pub.publish(state_message)

    def _publish_status(self, now: float) -> None:
        state_age = (
            None if self._last_state_time is None else now - self._last_state_time
        )
        message = String()
        message.data = json.dumps(
            {
                'connected': self._serial is not None,
                'firmware_ready': self._firmware_ready,
                'current_zero_adc': self._effective_current_zero,
                'current_zero_ready': (
                    not self._auto_current_zero
                    or self._current_zero_count
                    >= self._auto_current_zero_samples
                ),
                'serial_port': self._port,
                'motion_unlocked': self._limit > 0,
                'active_axes': self._active_axes,
                'last_state_age_s': state_age,
                'invalid_packet_count': self._invalid_packet_count,
                'last_firmware_error': self._last_firmware_error,
                'bounded_step_active': self._step_active,
            },
            separators=(',', ':'),
        )
        self._status_pub.publish(message)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.write(encode_stop())
                self._last_written_commands = (0, 0)
                self._serial.flush()
            except (OSError, SerialException):
                pass
            try:
                self._serial.close()
            except (OSError, SerialException):
                pass
        return super().destroy_node()


def main(args=None) -> None:
    """Run the ROS 2 bridge node."""
    rclpy.init(args=args)
    node = BlunoMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # A launch process can forward SIGINT while shutdown is in progress.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
