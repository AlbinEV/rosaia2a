#!/usr/bin/env python3
"""Sample the live sensor pipeline with the bridge locked to zero command."""

import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]


def main():
    """Run an isolated diagnostic session and stop its child processes."""
    rclpy.init()
    node = rclpy.create_node('rosaia_live_data_audit')
    from urllib.request import urlopen
    folder = ROOT / 'data' / 'reports' / time.strftime('live_%Y%m%d_%H%M%S')
    folder.mkdir(parents=True)
    commands = [
        ['ros2', 'run', 'bluno_motor_bridge', 'bluno_motor_bridge_node',
         '--ros-args', '--params-file', str(ROOT / 'ros2_ws/src/bluno_motor_bridge/config/controller.yaml'),
         '-p', 'max_abs_command:=0'],
        [sys.executable, '-c', 'from aruco_shape_tracker.tracker_node import main; main()',
         '--ros-args', '--params-file', str(ROOT / 'ros2_ws/src/aruco_shape_tracker/config/two_arms.yaml')],
        ['ros2', 'launch', 'rosaia_dashboard', 'dashboard.launch.py'],
    ]
    counts, latest, first, last = {}, {}, {}, {}
    subscriptions = []

    def receive(topic, message):
        now = time.monotonic()
        counts[topic] = counts.get(topic, 0) + 1
        first.setdefault(topic, now)
        last[topic] = now
        if isinstance(message, String):
            latest[topic] = json.loads(message.data)
        else:
            latest[topic] = {'points': [[p.x, p.y, p.z] for p in message.points],
                             'frame': message.header.frame_id}

    for topic, kind in [('/motor_state', String), ('/bluno/status', String),
                        ('/aruco/status', String), ('/arm_1/aruco_state', PointCloud),
                        ('/arm_2/aruco_state', PointCloud)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda message, name=topic: receive(name, message), qos_profile_sensor_data))
    processes = []
    try:
        for index, command in enumerate(commands):
            with (folder / f'process_{index}.log').open('w') as log:
                processes.append(subprocess.Popen(command, cwd=ROOT, stdout=log,
                                                   stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        report = {'counts': counts, 'latest': latest,
                  'rates_hz': {topic: (count - 1) / (last[topic] - first[topic])
                               for topic, count in counts.items() if count > 1},
                  'process_returncodes': [p.poll() for p in processes],
                  'tracker_implementation': 'python_reference',
                  'bridge_max_abs_command': 0}
        try:
            with urlopen('http://127.0.0.1:8765/api/state', timeout=3) as response:
                report['dashboard'] = json.load(response)
        except Exception as error:
            report['dashboard_error'] = str(error)
        (folder / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        print(f'Report: {folder / "report.json"}')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                try:
                    import os
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
