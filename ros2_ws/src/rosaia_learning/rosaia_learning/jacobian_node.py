"""Publish per-arm MLP Jacobians from the latest encoder positions."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Int64MultiArray, String

from .model import NumpyMlpModel


@dataclass
class ArmModel:
    """Runtime mapping between one encoder axis and one learned model."""

    name: str
    axis: int
    encoder_sign: int
    model: NumpyMlpModel
    publisher: object


class JacobianPublisher(Node):
    """Evaluate scalar-input MLP Jacobians at current encoder positions."""

    def __init__(self) -> None:
        """Load configured models and create the ROS inference interface."""
        super().__init__('jacobian_publisher')
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        self.declare_parameter('axis_indices', [0, 1])
        self.declare_parameter('encoder_signs', [1, -1])
        self.declare_parameter('model_directories', ['', ''])
        self.declare_parameter(
            'output_topics', ['/arm_1/jacobian', '/arm_2/jacobian']
        )
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('maximum_encoder_age_s', 0.15)

        names = list(self.get_parameter('arm_names').value)
        axes = [int(value) for value in self.get_parameter('axis_indices').value]
        signs = [
            int(value) for value in self.get_parameter('encoder_signs').value
        ]
        directories = list(self.get_parameter('model_directories').value)
        topics = list(self.get_parameter('output_topics').value)
        lengths = {len(names), len(axes), len(signs), len(directories), len(topics)}
        if len(lengths) != 1 or not names:
            raise ValueError('all per-arm parameter arrays must have equal length')
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._maximum_age = float(
            self.get_parameter('maximum_encoder_age_s').value
        )
        if rate <= 0.0 or self._maximum_age <= 0.0:
            raise ValueError('publish rate and encoder age must be positive')

        self._arms = []
        for name, axis, sign, directory, topic in zip(
            names, axes, signs, directories, topics
        ):
            if axis not in (0, 1) or sign not in (-1, 1):
                raise ValueError('invalid axis index or encoder sign')
            if not directory:
                self.get_logger().warning(f'{name}: no model configured')
                continue
            model = NumpyMlpModel.load(Path(directory))
            publisher = self.create_publisher(Float64MultiArray, str(topic), 10)
            self._arms.append(
                ArmModel(str(name), axis, sign, model, publisher)
            )
            self.get_logger().info(
                f'{name}: loaded MLP over q=[{model.q_min}, {model.q_max}]'
            )
        self._encoder_counts = None
        self._encoder_received_at = None
        self._encoder_subscription = self.create_subscription(
            Int64MultiArray, '/encoder_counts', self._on_encoder, 20
        )
        self._status_publisher = self.create_publisher(
            String, '/jacobian_model/status', 10
        )
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._cycle = 0

    def _on_encoder(self, message: Int64MultiArray) -> None:
        if len(message.data) != 2:
            self.get_logger().warning('encoder_counts must contain two values')
            return
        self._encoder_counts = (int(message.data[0]), int(message.data[1]))
        self._encoder_received_at = time.monotonic()

    def _publish(self) -> None:
        now = time.monotonic()
        status = {}
        for arm in self._arms:
            if self._encoder_counts is None:
                status[arm.name] = {'state': 'waiting_for_encoder'}
                continue
            age = now - self._encoder_received_at
            if age > self._maximum_age:
                status[arm.name] = {'state': 'stale_encoder', 'age_s': age}
                continue
            q_value = arm.encoder_sign * self._encoder_counts[arm.axis]
            if not arm.model.q_min <= q_value <= arm.model.q_max:
                status[arm.name] = {
                    'state': 'outside_training_range', 'q': q_value
                }
                continue
            _, jacobian = arm.model.predict_and_jacobian(q_value)
            message = Float64MultiArray()
            message.data = jacobian.tolist()
            arm.publisher.publish(message)
            status[arm.name] = {'state': 'active', 'q': q_value}
        self._cycle += 1
        if self._cycle % 30 == 0:
            message = String()
            message.data = json.dumps(status, separators=(',', ':'))
            self._status_publisher.publish(message)


def main(args=None) -> None:
    """Run the learned Jacobian publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = JacobianPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, RuntimeError, ValueError):
                pass
        if rclpy.ok():
            rclpy.shutdown()
