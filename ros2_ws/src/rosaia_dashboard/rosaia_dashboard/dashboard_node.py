"""Serve a live web dashboard backed by ROS 2 sensor and goal topics."""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosaia_learning.model import NumpyMlpModel
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int16MultiArray, String

from .projection import (
    project_curvature_goal,
    project_planar_goal,
    signed_planar_curvature,
)


def _cloud_payload(message: PointCloud) -> dict:
    """Convert a complete three-marker cloud into JSON-compatible values."""
    if len(message.points) != 3:
        raise ValueError('shape cloud must contain exactly three points')
    points = [
        {'x': float(point.x), 'y': float(point.y), 'z': float(point.z)}
        for point in message.points
    ]
    if not all(math.isfinite(value) for point in points for value in point.values()):
        raise ValueError('shape cloud contains non-finite coordinates')
    return {'frame_id': message.header.frame_id, 'points': points}


def _model_summary(directory: str, model) -> dict:
    """Read compact validation and approval information for the UI."""
    summary = {
        'available': model is not None,
        'approved': False,
        'converged': None,
        'direction': None,
        'validation_rmse_mm': None,
        'validation_p95_mm': None,
        'q_min': None if model is None else model.q_min,
        'q_max': None if model is None else model.q_max,
        'curvature_min': None,
        'curvature_max': None,
    }
    if model is None:
        return summary
    try:
        metadata = json.loads((Path(directory) / 'metadata.json').read_text())
    except (OSError, json.JSONDecodeError):
        return summary
    training = metadata.get('training', {})
    approval = metadata.get('approval', {})
    summary.update({
        'approved': approval.get('status') == 'approved',
        'converged': training.get('converged'),
        'direction': training.get('direction'),
        'validation_rmse_mm': 1000.0 * float(
            training['validation_rmse_total']
        ) if training.get('validation_rmse_total') is not None else None,
        'validation_p95_mm': 1000.0 * float(
            training['validation_p95_total']
        ) if training.get('validation_p95_total') is not None else None,
    })
    grid = np.linspace(model.q_min, model.q_max, 501)
    curvatures = [
        signed_planar_curvature(model.predict_and_jacobian(value)[0])
        for value in grid
    ]
    summary['curvature_min'] = float(min(curvatures))
    summary['curvature_max'] = float(max(curvatures))
    return summary


class DashboardNode(Node):
    """Aggregate live sensors and expose feasible visual goals over HTTP."""

    def __init__(self) -> None:
        super().__init__('rosaia_dashboard')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        self.declare_parameter(
            'state_topics', ['/arm_1/aruco_state', '/arm_2/aruco_state']
        )
        self.declare_parameter(
            'goal_topics', ['/arm_1/shape_goal', '/arm_2/shape_goal']
        )
        self.declare_parameter('model_directories', ['', ''])
        self.declare_parameter('arm_1_model_directory', '')
        self.declare_parameter('arm_2_model_directory', '')
        self.declare_parameter('axis_indices', [0, 1])
        self.declare_parameter('encoder_signs', [1, -1])
        self.declare_parameter('encoder_zero_raws', [0, 0])
        self.declare_parameter('projection_q_regularization', [1e-8, 1e-8])
        self.declare_parameter(
            'projection_q_regularization_transition_m', [0.003, 0.003]
        )
        self.declare_parameter('enable_goal_publish', False)
        self.declare_parameter('enable_motor_execute', False)
        self.declare_parameter('maximum_move_counts', 220)
        self.declare_parameter('motor_step_counts', 5)
        self.declare_parameter('target_tolerance_counts', 5)
        self.declare_parameter('motor_step_timeout_s', 4.0)
        self.declare_parameter('motor_step_retries', 3)
        self.declare_parameter('command_to_q_signs', [-1, -1])
        self.declare_parameter('contraction_pwm', [110, 180])
        self.declare_parameter('release_pwm', [105, 120])
        self.declare_parameter('high_q_thresholds', [190, 10000])
        self.declare_parameter('high_q_contraction_pwm', [130, 180])
        self.declare_parameter('maximum_abs_current_a', [2.8, 2.8])
        self.declare_parameter('motion_mode', 'continuous')
        self.declare_parameter('continuous_control_rate_hz', 30.0)
        self.declare_parameter('continuous_motion_timeout_s', 30.0)
        self.declare_parameter('continuous_no_motion_timeout_s', 3.0)
        self.declare_parameter('continuous_ramp_s', 0.5)

        names = [str(value) for value in self.get_parameter('arm_names').value]
        state_topics = [
            str(value) for value in self.get_parameter('state_topics').value
        ]
        goal_topics = [
            str(value) for value in self.get_parameter('goal_topics').value
        ]
        directories = [
            str(value)
            for value in self.get_parameter('model_directories').value
        ]
        directory_overrides = [
            str(self.get_parameter('arm_1_model_directory').value),
            str(self.get_parameter('arm_2_model_directory').value),
        ]
        directories = [
            override or directory
            for override, directory in zip(directory_overrides, directories)
        ]
        axes = [int(value) for value in self.get_parameter('axis_indices').value]
        signs = [
            int(value) for value in self.get_parameter('encoder_signs').value
        ]
        zeros = [
            int(value) for value in self.get_parameter('encoder_zero_raws').value
        ]
        regularization = [
            float(value)
            for value in self.get_parameter(
                'projection_q_regularization'
            ).value
        ]
        transitions = [
            float(value)
            for value in self.get_parameter(
                'projection_q_regularization_transition_m'
            ).value
        ]
        lengths = {
            len(names), len(state_topics), len(goal_topics), len(directories),
            len(axes), len(signs), len(zeros), len(regularization),
            len(transitions),
        }
        if not names or len(lengths) != 1:
            raise ValueError('per-arm dashboard parameter lengths must match')
        if any(axis not in (0, 1) for axis in axes):
            raise ValueError('axis indices must be 0 or 1')
        if any(sign not in (-1, 1) for sign in signs):
            raise ValueError('encoder signs must be -1 or 1')
        if any(not math.isfinite(value) or value < 0.0 for value in regularization):
            raise ValueError('projection q regularization must be non-negative')
        if any(not math.isfinite(value) or value <= 0.0 for value in transitions):
            raise ValueError('projection transition must be positive')

        command_signs = [
            int(value)
            for value in self.get_parameter('command_to_q_signs').value
        ]
        contraction_pwm = [
            int(value) for value in self.get_parameter('contraction_pwm').value
        ]
        release_pwm = [
            int(value) for value in self.get_parameter('release_pwm').value
        ]
        high_thresholds = [
            int(value) for value in self.get_parameter('high_q_thresholds').value
        ]
        high_pwm = [
            int(value)
            for value in self.get_parameter('high_q_contraction_pwm').value
        ]
        maximum_current = [
            float(value)
            for value in self.get_parameter('maximum_abs_current_a').value
        ]
        motor_groups = (
            command_signs, contraction_pwm, release_pwm,
            high_thresholds, high_pwm, maximum_current,
        )
        if any(len(values) != len(names) for values in motor_groups):
            raise ValueError('per-arm motor parameter lengths must match')
        if any(value not in (-1, 1) for value in command_signs):
            raise ValueError('command-to-q signs must be -1 or 1')
        if any(
            not 1 <= value <= 255
            for values in (contraction_pwm, release_pwm, high_pwm)
            for value in values
        ):
            raise ValueError('motor PWM values must be in 1..255')
        if any(not math.isfinite(value) or value <= 0.0 for value in maximum_current):
            raise ValueError('maximum currents must be positive')

        self._lock = threading.Lock()
        self._goal_enabled = bool(
            self.get_parameter('enable_goal_publish').value
        )
        self._motor_execute_enabled = bool(
            self.get_parameter('enable_motor_execute').value
        )
        self._maximum_move = int(
            self.get_parameter('maximum_move_counts').value
        )
        self._motor_step_counts = int(
            self.get_parameter('motor_step_counts').value
        )
        self._target_tolerance = int(
            self.get_parameter('target_tolerance_counts').value
        )
        self._motor_step_timeout = float(
            self.get_parameter('motor_step_timeout_s').value
        )
        self._motor_step_retries = int(
            self.get_parameter('motor_step_retries').value
        )
        self._motion_mode = str(
            self.get_parameter('motion_mode').value
        )
        self._continuous_rate = float(
            self.get_parameter('continuous_control_rate_hz').value
        )
        self._continuous_timeout = float(
            self.get_parameter('continuous_motion_timeout_s').value
        )
        self._continuous_no_motion_timeout = float(
            self.get_parameter('continuous_no_motion_timeout_s').value
        )
        self._continuous_ramp = float(
            self.get_parameter('continuous_ramp_s').value
        )
        if (
            self._maximum_move <= 0
            or not 1 <= self._motor_step_counts <= 10
            or self._target_tolerance < 0
            or self._motor_step_timeout <= 0.0
            or self._motor_step_retries < 0
            or self._motion_mode not in ('bounded_steps', 'continuous')
            or min(
                self._continuous_rate,
                self._continuous_timeout,
                self._continuous_no_motion_timeout,
                self._continuous_ramp,
            ) <= 0.0
        ):
            raise ValueError('invalid bounded motor execution parameters')
        self._state = {
            'goal_publish_enabled': self._goal_enabled,
            'motor_execute_enabled': self._motor_execute_enabled,
            'motion_mode': self._motion_mode,
            'vision': None,
            'motor': None,
            'motion': {'state': 'idle'},
            'arms': {},
        }
        self._models = {}
        self._model_summaries = {}
        self._arm_encoder_config = {}
        self._goal_publishers = {}
        self._owned_subscriptions = []
        for index, (
            name, state_topic, goal_topic, directory, axis, sign, zero,
            penalty, transition,
        ) in enumerate(zip(
            names, state_topics, goal_topics, directories,
            axes, signs, zeros, regularization, transitions,
        )):
            model = NumpyMlpModel.load(directory) if directory else None
            self._models[name] = model
            self._arm_encoder_config[name] = {
                'axis': axis,
                'sign': sign,
                'zero': zero,
                'regularization': penalty,
                'transition_m': transition,
                'command_to_q_sign': command_signs[index],
                'contraction_pwm': contraction_pwm[index],
                'release_pwm': release_pwm[index],
                'high_q_threshold': high_thresholds[index],
                'high_q_contraction_pwm': high_pwm[index],
                'maximum_abs_current_a': maximum_current[index],
            }
            self._model_summaries[name] = _model_summary(directory, model)
            self._goal_publishers[name] = self.create_publisher(
                PointCloud, goal_topic, 10
            )
            self._state['arms'][name] = {
                'current': None,
                'requested': None,
                'feasible': None,
                'projection': None,
                'model': self._model_summaries[name],
            }
            self._owned_subscriptions.append(self.create_subscription(
                PointCloud,
                state_topic,
                lambda message, arm=name: self._on_shape(arm, message),
                qos_profile_sensor_data,
            ))
        self._owned_subscriptions.extend([
            self.create_subscription(String, '/motor_state', self._on_motor, 20),
            self.create_subscription(String, '/aruco/status', self._on_vision, 10),
        ])
        self._step_publisher = self.create_publisher(
            Int16MultiArray, '/motor_step_command', 10
        )
        self._zero_publisher = self.create_publisher(
            Int16MultiArray, '/motor_command', 10
        )
        self._motor_received_at = None
        self._motion_cancel = threading.Event()
        self._motion_thread = None

        web_root = Path(get_package_share_directory('rosaia_dashboard')) / 'web'
        self._index = (web_root / 'index.html').read_bytes()
        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        if not 0 < port < 65536:
            raise ValueError('dashboard port must be in 1..65535')
        self._server = ThreadingHTTPServer(
            (host, port), self._handler_class()
        )
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name='rosaia_dashboard_http',
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(
            f'dashboard ready at http://{host}:{port}; '
            f'goal publishing enabled={self._goal_enabled}'
        )

    def _on_shape(self, arm_name: str, message: PointCloud) -> None:
        try:
            payload = _cloud_payload(message)
        except ValueError as error:
            self.get_logger().warning(f'{arm_name}: {error}')
            return
        with self._lock:
            self._state['arms'][arm_name]['current'] = payload

    def _on_motor(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state['motor'] = payload
            self._motor_received_at = time.monotonic()

    def _on_vision(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state['vision'] = payload

    def snapshot(self) -> dict:
        """Return an isolated JSON round-trip copy of current dashboard state."""
        with self._lock:
            return json.loads(json.dumps(self._state))

    def set_goal(self, payload: dict) -> tuple[int, dict]:
        """Project a requested planar shape and optionally publish it."""
        arm_name = str(payload.get('arm', ''))
        if arm_name not in self._models:
            return HTTPStatus.BAD_REQUEST, {'error': 'unknown arm'}
        points = payload.get('points')
        if not isinstance(points, list) or len(points) != 3:
            return HTTPStatus.BAD_REQUEST, {'error': 'exactly three points required'}
        try:
            values = np.array([
                [float(point['x']), float(point['y'])] for point in points
            ], dtype=np.float64).reshape(6)
        except (KeyError, TypeError, ValueError):
            return HTTPStatus.BAD_REQUEST, {'error': 'invalid point coordinates'}
        if not np.isfinite(values).all():
            return HTTPStatus.BAD_REQUEST, {'error': 'coordinates must be finite'}
        model = self._models[arm_name]
        if model is None:
            return HTTPStatus.CONFLICT, {'error': 'no validated model configured'}

        snapshot = self.snapshot()
        current = snapshot['arms'][arm_name]['current']
        motor = snapshot['motor']
        encoder_config = self._arm_encoder_config[arm_name]
        current_q = None
        if motor is not None:
            counts = motor.get('encoder_counts')
            axis = encoder_config['axis']
            if isinstance(counts, list) and len(counts) > axis:
                current_q = encoder_config['sign'] * (
                    float(counts[axis]) - encoder_config['zero']
                )
        if encoder_config['regularization'] > 0.0 and current_q is None:
            return HTTPStatus.CONFLICT, {
                'error': 'current encoder is unavailable for projection'
            }
        result = project_planar_goal(
            model,
            values,
            current_q=current_q,
            q_regularization=encoder_config['regularization'],
            q_regularization_transition_m=encoder_config['transition_m'],
        )
        z_values = (
            [point['z'] for point in current['points']]
            if current is not None else [0.0, 0.0, 0.0]
        )
        requested = [
            {'x': float(values[2 * index]), 'y': float(values[2 * index + 1]),
             'z': float(z_values[index])}
            for index in range(3)
        ]
        feasible = [
            {'x': float(result.state[2 * index]),
             'y': float(result.state[2 * index + 1]),
             'z': float(z_values[index])}
            for index in range(3)
        ]
        projection = {
            'q': result.q,
            'q_current': current_q,
            'q_regularization': result.q_regularization,
            'q_regularization_transition_m': encoder_config['transition_m'],
            'weighted_error_m': result.weighted_error,
        }
        with self._lock:
            arm_state = self._state['arms'][arm_name]
            arm_state['requested'] = {'points': requested}
            arm_state['feasible'] = {'points': feasible}
            arm_state['projection'] = projection

        publish = bool(payload.get('publish', False))
        if publish:
            if not self._goal_enabled:
                return HTTPStatus.FORBIDDEN, {
                    'error': 'goal publishing is disabled',
                    'projection': projection,
                    'feasible': feasible,
                }
            if not self._model_summaries[arm_name]['approved']:
                return HTTPStatus.FORBIDDEN, {
                    'error': 'model is not approved for online control',
                    'projection': projection,
                    'feasible': feasible,
                }
            message = PointCloud()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = (
                current['frame_id'] if current is not None
                else f'{arm_name}_base'
            )
            from geometry_msgs.msg import Point32
            message.points = [Point32(**point) for point in feasible]
            self._goal_publishers[arm_name].publish(message)
        return HTTPStatus.OK, {
            'projection': projection,
            'requested': requested,
            'feasible': feasible,
            'published': publish,
        }

    def set_curvature_goal(self, payload: dict) -> tuple[int, dict]:
        """Infer a reachable shape and encoder target from curvature."""
        arm_name = str(payload.get('arm', ''))
        if arm_name not in self._models:
            return HTTPStatus.BAD_REQUEST, {'error': 'unknown arm'}
        try:
            requested_curvature = float(payload['curvature'])
        except (KeyError, TypeError, ValueError):
            return HTTPStatus.BAD_REQUEST, {'error': 'invalid curvature'}
        if not math.isfinite(requested_curvature):
            return HTTPStatus.BAD_REQUEST, {'error': 'invalid curvature'}
        model = self._models[arm_name]
        if model is None:
            return HTTPStatus.CONFLICT, {'error': 'no model configured'}
        snapshot = self.snapshot()
        motor = snapshot.get('motor')
        current = snapshot['arms'][arm_name]['current']
        config = self._arm_encoder_config[arm_name]
        if motor is None:
            return HTTPStatus.CONFLICT, {'error': 'motor telemetry unavailable'}
        counts = motor.get('encoder_counts', [])
        if len(counts) <= config['axis']:
            return HTTPStatus.CONFLICT, {'error': 'encoder is unavailable'}
        current_q = config['sign'] * (
            float(counts[config['axis']]) - config['zero']
        )
        result = project_curvature_goal(
            model, requested_curvature, current_q=current_q
        )
        curvature_min = self._model_summaries[arm_name]['curvature_min']
        curvature_max = self._model_summaries[arm_name]['curvature_max']
        z_values = (
            [point['z'] for point in current['points']]
            if current is not None else [0.0, 0.0, 0.0]
        )
        feasible = [
            {
                'x': float(result.state[2 * index]),
                'y': float(result.state[2 * index + 1]),
                'z': float(z_values[index]),
            }
            for index in range(3)
        ]
        projection = {
            'q': result.q,
            'q_current': current_q,
            'curvature_target': result.requested_curvature,
            'curvature_achieved': result.achieved_curvature,
            'curvature_error': (
                result.achieved_curvature - result.requested_curvature
            ),
            'curvature_min': curvature_min,
            'curvature_max': curvature_max,
            'curvature_clamped': not (
                curvature_min <= requested_curvature <= curvature_max
            ),
            'weighted_error_m': 0.0,
        }
        with self._lock:
            arm_state = self._state['arms'][arm_name]
            arm_state['requested'] = None
            arm_state['feasible'] = {'points': feasible}
            arm_state['projection'] = projection
        return HTTPStatus.OK, {
            'projection': projection,
            'feasible': feasible,
        }

    def execute_projection(self, payload: dict) -> tuple[int, dict]:
        """Start a bounded encoder move to the last inferred q target."""
        arm_name = str(payload.get('arm', ''))
        if arm_name not in self._models:
            return HTTPStatus.BAD_REQUEST, {'error': 'unknown arm'}
        if not self._motor_execute_enabled:
            return HTTPStatus.FORBIDDEN, {'error': 'motor execution is disabled'}
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                return HTTPStatus.CONFLICT, {'error': 'another motion is active'}
            arm_state = self._state['arms'][arm_name]
            projection = arm_state.get('projection')
            motor = self._state.get('motor')
            vision = self._state.get('vision') or {}
            config = self._arm_encoder_config[arm_name]
            if projection is None:
                return HTTPStatus.CONFLICT, {'error': 'calculate inference first'}
            if motor is None or self._motor_received_at is None:
                return HTTPStatus.CONFLICT, {'error': 'motor telemetry unavailable'}
            if time.monotonic() - self._motor_received_at > 0.2:
                return HTTPStatus.CONFLICT, {'error': 'motor telemetry is stale'}
            if int(motor.get('flags', 0)) & ~16:
                return HTTPStatus.CONFLICT, {'error': 'firmware fault is active'}
            if any(int(value) for value in motor.get('applied_commands', [])):
                return HTTPStatus.CONFLICT, {'error': 'motors are not stopped'}
            currents = motor.get('current_a_host', [])
            if (
                len(currents) > config['axis']
                and abs(float(currents[config['axis']]))
                > config['maximum_abs_current_a']
            ):
                return HTTPStatus.CONFLICT, {'error': 'motor current is too high'}
            if arm_name not in vision.get('complete_arms', []):
                return HTTPStatus.CONFLICT, {'error': 'arm vision is incomplete'}
            counts = motor.get('encoder_counts', [])
            if len(counts) <= config['axis']:
                return HTTPStatus.CONFLICT, {'error': 'encoder is unavailable'}
            current_q = config['sign'] * (
                int(counts[config['axis']]) - config['zero']
            )
            target_q = int(round(float(projection['q'])))
            model = self._models[arm_name]
            if not model.q_min <= target_q <= model.q_max:
                return HTTPStatus.BAD_REQUEST, {'error': 'target is outside model range'}
            if abs(target_q - current_q) > self._maximum_move:
                return HTTPStatus.BAD_REQUEST, {
                    'error': (
                        f'target exceeds maximum single move of '
                        f'{self._maximum_move} counts'
                    )
                }
            self._motion_cancel.clear()
            self._state['motion'] = {
                'state': 'starting', 'arm': arm_name,
                'current_q': current_q, 'target_q': target_q,
            }
            motion_target = (
                self._run_continuous_motion
                if self._motion_mode == 'continuous'
                else self._run_bounded_motion
            )
            self._motion_thread = threading.Thread(
                target=motion_target,
                args=(arm_name, target_q),
                name=f'dashboard_motion_{arm_name}',
                daemon=True,
            )
            self._motion_thread.start()
        return HTTPStatus.ACCEPTED, {
            'state': 'starting', 'arm': arm_name,
            'current_q': current_q, 'target_q': target_q,
            'mode': self._motion_mode,
        }

    def cancel_motion(self) -> tuple[int, dict]:
        """Cancel the sequence after the current firmware-bounded step."""
        self._motion_cancel.set()
        self._zero_publisher.publish(Int16MultiArray(data=[0, 0]))
        return HTTPStatus.OK, {
            'state': 'cancelling',
            'message': 'current bounded step may finish before cancellation',
        }

    def _motion_error(self, message: str) -> None:
        with self._lock:
            self._state['motion']['state'] = 'error'
            self._state['motion']['error'] = message
        self._zero_publisher.publish(Int16MultiArray(data=[0, 0]))

    def _run_bounded_motion(self, arm_name: str, target_q: int) -> None:
        """Reach one inferred target through firmware-limited encoder steps."""
        config = self._arm_encoder_config[arm_name]
        consecutive_timeouts = 0
        while not self._motion_cancel.is_set():
            with self._lock:
                motor = json.loads(json.dumps(self._state.get('motor')))
                received_at = self._motor_received_at
                vision = json.loads(json.dumps(self._state.get('vision') or {}))
            if motor is None or received_at is None:
                self._motion_error('motor telemetry unavailable')
                return
            if time.monotonic() - received_at > 0.2:
                self._motion_error('motor telemetry became stale')
                return
            flags = int(motor.get('flags', 0))
            if flags & ~16:
                self._motion_error(f'firmware fault flags={flags}')
                return
            currents = motor.get('current_a_host', [])
            if (
                len(currents) > config['axis']
                and abs(float(currents[config['axis']]))
                > config['maximum_abs_current_a']
            ):
                self._motion_error('motor current limit exceeded')
                return
            if arm_name not in vision.get('complete_arms', []):
                self._motion_error('arm vision became incomplete')
                return
            raw = int(motor['encoder_counts'][config['axis']])
            current_q = config['sign'] * (raw - config['zero'])
            error = target_q - current_q
            with self._lock:
                self._state['motion'].update({
                    'state': 'moving', 'current_q': current_q,
                    'target_q': target_q,
                })
            if abs(error) <= self._target_tolerance:
                with self._lock:
                    self._state['motion']['state'] = 'complete'
                self._zero_publisher.publish(Int16MultiArray(data=[0, 0]))
                return
            direction = 1 if error > 0 else -1
            magnitude = min(self._motor_step_counts, abs(error))
            pwm = config['release_pwm']
            if direction > 0:
                pwm = config['contraction_pwm']
                if current_q >= config['high_q_threshold']:
                    pwm = config['high_q_contraction_pwm']
            signed_counts = config['command_to_q_sign'] * direction * magnitude
            step = Int16MultiArray(data=[config['axis'], signed_counts, pwm])
            start_q = current_q
            self._step_publisher.publish(step)
            deadline = time.monotonic() + self._motor_step_timeout
            moved = False
            last_q = current_q
            stable_since = time.monotonic()
            while time.monotonic() < deadline:
                if self._motion_cancel.wait(0.02):
                    self._zero_publisher.publish(Int16MultiArray(data=[0, 0]))
                    with self._lock:
                        self._state['motion']['state'] = 'cancelled'
                    return
                with self._lock:
                    live = self._state.get('motor')
                    live_received = self._motor_received_at
                if live is None or time.monotonic() - live_received > 0.2:
                    self._motion_error('motor telemetry became stale')
                    return
                live_flags = int(live.get('flags', 0))
                if live_flags & ~16:
                    self._motion_error(f'firmware fault flags={live_flags}')
                    return
                live_currents = live.get('current_a_host', [])
                if (
                    len(live_currents) > config['axis']
                    and abs(float(live_currents[config['axis']]))
                    > config['maximum_abs_current_a']
                ):
                    self._motion_error('motor current limit exceeded')
                    return
                live_raw = int(live['encoder_counts'][config['axis']])
                live_q = config['sign'] * (live_raw - config['zero'])
                if live_q != start_q:
                    moved = True
                if live_q != last_q:
                    last_q = live_q
                    stable_since = time.monotonic()
                applied = live.get('applied_commands', [0, 0])
                if (
                    moved and not any(int(value) for value in applied)
                    and time.monotonic() - stable_since >= 0.15
                ):
                    consecutive_timeouts = 0
                    break
            else:
                consecutive_timeouts += 1
                if consecutive_timeouts > self._motor_step_retries:
                    self._motion_error(
                        f'bounded step timeout at q={start_q} after '
                        f'{consecutive_timeouts} attempts'
                    )
                    return
                with self._lock:
                    self._state['motion'].update({
                        'state': 'retrying',
                        'retry': consecutive_timeouts,
                    })
        with self._lock:
            self._state['motion']['state'] = 'cancelled'

    def _publish_zero_command(self) -> None:
        """Publish several zeros so the bridge and watchdog both stop."""
        message = Int16MultiArray(data=[0, 0])
        for _ in range(3):
            self._zero_publisher.publish(message)
            time.sleep(0.01)

    def _run_continuous_motion(self, arm_name: str, target_q: int) -> None:
        """Continuously drive one axis toward an inferred encoder target."""
        config = self._arm_encoder_config[arm_name]
        started = time.monotonic()
        last_motion = started
        last_q = None
        initial_direction = None
        period = 1.0 / self._continuous_rate
        try:
            while not self._motion_cancel.is_set():
                cycle_started = time.monotonic()
                with self._lock:
                    motor = json.loads(json.dumps(self._state.get('motor')))
                    received_at = self._motor_received_at
                    vision = json.loads(
                        json.dumps(self._state.get('vision') or {})
                    )
                if motor is None or received_at is None:
                    self._motion_error('motor telemetry unavailable')
                    return
                now = time.monotonic()
                if now - received_at > 0.2:
                    self._motion_error('motor telemetry became stale')
                    return
                flags = int(motor.get('flags', 0))
                if flags & ~16:
                    self._motion_error(f'firmware fault flags={flags}')
                    return
                if arm_name not in vision.get('complete_arms', []):
                    self._motion_error('arm vision became incomplete')
                    return
                currents = motor.get('current_a_host', [])
                if (
                    len(currents) > config['axis']
                    and abs(float(currents[config['axis']]))
                    > config['maximum_abs_current_a']
                ):
                    self._motion_error('motor current limit exceeded')
                    return
                raw = int(motor['encoder_counts'][config['axis']])
                current_q = config['sign'] * (raw - config['zero'])
                error = target_q - current_q
                if last_q is None or current_q != last_q:
                    last_q = current_q
                    last_motion = now
                if abs(error) <= self._target_tolerance:
                    with self._lock:
                        self._state['motion'].update({
                            'state': 'complete', 'current_q': current_q,
                            'target_q': target_q,
                        })
                    return
                direction = 1 if error > 0 else -1
                if initial_direction is None:
                    initial_direction = direction
                elif direction != initial_direction:
                    with self._lock:
                        self._state['motion'].update({
                            'state': 'complete', 'current_q': current_q,
                            'target_q': target_q, 'overshot': True,
                        })
                    return
                if now - started > self._continuous_timeout:
                    self._motion_error('continuous motion timeout')
                    return
                if now - last_motion > self._continuous_no_motion_timeout:
                    self._motion_error('encoder did not move')
                    return
                pwm = config['release_pwm']
                if direction > 0:
                    pwm = config['contraction_pwm']
                    if current_q >= config['high_q_threshold']:
                        pwm = config['high_q_contraction_pwm']
                ramp = min(
                    1.0,
                    0.65 + 0.35 * (now - started) / self._continuous_ramp,
                )
                applied_pwm = max(1, round(pwm * ramp))
                command = (
                    config['command_to_q_sign'] * direction * applied_pwm
                )
                commands = [0, 0]
                commands[config['axis']] = command
                self._zero_publisher.publish(Int16MultiArray(data=commands))
                with self._lock:
                    self._state['motion'].update({
                        'state': 'moving', 'mode': 'continuous',
                        'current_q': current_q, 'target_q': target_q,
                        'command': command,
                    })
                remaining = period - (time.monotonic() - cycle_started)
                if remaining > 0.0:
                    self._motion_cancel.wait(remaining)
            with self._lock:
                self._state['motion']['state'] = 'cancelled'
        finally:
            self._publish_zero_command()

    def _handler_class(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _json(self, status, payload):
                encoded = json.dumps(payload, separators=(',', ':')).encode()
                self.send_response(int(status))
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def do_GET(self):
                if self.path == '/':
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(node._index)))
                    self.end_headers()
                    self.wfile.write(node._index)
                elif self.path == '/api/state':
                    self._json(HTTPStatus.OK, node.snapshot())
                else:
                    self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})

            def do_POST(self):
                if self.path not in (
                    '/api/goal', '/api/curvature',
                    '/api/execute', '/api/cancel',
                ):
                    self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 65536:
                        raise ValueError('invalid body length')
                    payload = json.loads(self.rfile.read(length))
                except (ValueError, json.JSONDecodeError):
                    self._json(HTTPStatus.BAD_REQUEST, {'error': 'invalid JSON'})
                    return
                if self.path == '/api/goal':
                    status, response = node.set_goal(payload)
                elif self.path == '/api/curvature':
                    status, response = node.set_curvature_goal(payload)
                elif self.path == '/api/execute':
                    status, response = node.execute_projection(payload)
                else:
                    status, response = node.cancel_motion()
                self._json(status, response)

            def log_message(self, *_args):
                return

        return Handler

    def destroy_node(self):
        self._motion_cancel.set()
        self._zero_publisher.publish(Int16MultiArray(data=[0, 0]))
        if self._motion_thread is not None:
            self._motion_thread.join(timeout=4.0)
        if hasattr(self, '_server'):
            self._server.shutdown()
            self._server.server_close()
            self._server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the ROS-backed HTTP dashboard."""
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
