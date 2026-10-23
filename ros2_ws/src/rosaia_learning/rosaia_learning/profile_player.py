"""Publish a bounded encoder-velocity calibration cycle."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .excitation import (
    append_return_home,
    step_velocity_cycle,
    trapezoidal_velocity_profile,
)


class ProfilePlayer(Node):
    """Publish two fresh per-axis references while one profile is active."""

    def __init__(self) -> None:
        super().__init__('velocity_profile_player')
        self.reference_publishers = [
            self.create_publisher(
                Float64, f'/arm_{axis + 1}/encoder_velocity_reference', 20
            )
            for axis in range(2)
        ]

    def publish_pair(self, axis: int, value: float) -> None:
        """Publish one active reference and an explicit zero on the other axis."""
        for selected, publisher in enumerate(self.reference_publishers):
            message = Float64()
            message.data = float(value) if selected == axis else 0.0
            publisher.publish(message)


def main(args=None) -> None:
    """Play one return-home velocity cycle in wall-clock time."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--type', choices=('step', 'trapezoid'), default='trapezoid'
    )
    parser.add_argument('--axis', type=int, choices=(0, 1), required=True)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--rise-time', type=float, default=3.0)
    parser.add_argument('--fall-time', type=float, default=3.0)
    parser.add_argument('--maximum-velocity', type=float, default=30.0)
    parser.add_argument('--home-dwell', type=float, default=1.0)
    parser.add_argument('--rate', type=float, default=50.0)
    options, ros_arguments = parser.parse_known_args(args)
    if options.type == 'step':
        times, values = step_velocity_cycle(
            options.duration,
            options.home_dwell,
            options.rate,
            options.maximum_velocity,
        )
    else:
        times, values = trapezoidal_velocity_profile(
            options.duration,
            options.rise_time,
            options.fall_time,
            options.rate,
            options.maximum_velocity,
        )
        times, values = append_return_home(
            times, values, options.home_dwell, options.rate
        )

    rclpy.init(args=ros_arguments)
    node = ProfilePlayer()
    try:
        deadline = time.monotonic() + 5.0
        while (
            node.reference_publishers[options.axis].get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.reference_publishers[options.axis].get_subscription_count() == 0:
            raise RuntimeError('velocity controller is not subscribed')
        started = time.monotonic()
        for scheduled, value in zip(times, values):
            while time.monotonic() - started < scheduled:
                rclpy.spin_once(node, timeout_sec=0.002)
            node.publish_pair(options.axis, float(value))
        for _ in range(20):
            node.publish_pair(options.axis, 0.0)
            rclpy.spin_once(node, timeout_sec=0.01)
        node.get_logger().info(
            f'completed {options.type} profile in {times[-1]:.1f} s; '
            'zero reference published'
        )
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.publish_pair(options.axis, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
