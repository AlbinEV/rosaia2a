#!/usr/bin/env python3
"""Supervised, bounded approach of markers 4/7 in the common camera frame."""

from collections import deque
import csv
import json
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int16MultiArray, String

from rosaia_workflow import ROOT, nodes, preflight, recorder


def main():
    preflight()
    if '/encoder_velocity_controller' in nodes():
        raise RuntimeError('stop velocity controller before bounded motion')
    label = time.strftime('tip_gap_%Y%m%d_%H%M%S')
    folder = ROOT / 'data/workflows' / label
    folder.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('approach_tip_gap')
    motor = [None, 0.0]
    pairs = {}
    measured = deque(maxlen=30)
    steps = []
    result = {'target_m': .10, 'tolerance_m': .01, 'target_reached': False,
              'frame': 'shared camera optical frame', 'encoder_zero_changed': False}
    raw_file = (folder / 'raw_geometry.csv').open('w')
    writer = csv.writer(raw_file)
    writer.writerow(['camera_time_ns', 'gap_m', 'marker4_x', 'marker4_y', 'marker4_z',
                     'marker7_x', 'marker7_y', 'marker7_z', 'encoder0', 'encoder1'])
    motor_file = (folder / 'motor.jsonl').open('w')

    def on_motor(msg):
        value = json.loads(msg.data)
        motor[:] = [value, time.monotonic()]
        motor_file.write(json.dumps({'host_monotonic': motor[1], **value})+'\n')

    def on_cloud(msg):
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        arm = 0 if any(c.name == 'marker_id' and 4 in c.values for c in msg.channels) else 1
        if len(msg.points) != 3:
            return
        key = (msg.header.frame_id, stamp)
        entry = pairs.setdefault(key, {})
        entry[arm] = np.array([[p.x, p.y, p.z] for p in msg.points])
        if len(entry) == 2:
            points = np.stack((entry[0], entry[1]))
            distance = float(np.linalg.norm(points[1, -1]-points[0, -1]))
            measured.append((time.monotonic(), distance, points))
            counts = motor[0]['encoder_counts'] if motor[0] else [None, None]
            writer.writerow([stamp, distance, *points[0, -1], *points[1, -1], *counts])
            del pairs[key]
        while len(pairs) > 20:
            del pairs[next(iter(pairs))]

    subscriptions = [node.create_subscription(String, '/motor_state', on_motor, 30)]
    subscriptions += [node.create_subscription(
        PointCloud, f'/arm_{i}/aruco_points_3d_raw', on_cloud, qos_profile_sensor_data)
        for i in (1, 2)]
    command = node.create_publisher(Int16MultiArray, '/motor_step_command', 10)
    stop = node.create_publisher(Int16MultiArray, '/motor_command', 10)

    def halt():
        for _ in range(3):
            stop.publish(Int16MultiArray(data=[0, 0]))
            rclpy.spin_once(node, timeout_sec=.02)

    def geometry():
        recent = [p for t, d, p in measured if time.monotonic()-t < .5]
        if len(recent) < 5:
            raise RuntimeError('insufficient fresh synchronized marker pairs')
        points = np.median(np.stack(recent), axis=0)
        return float(np.linalg.norm(points[1, -1]-points[0, -1])), points

    def check():
        if motor[0] is None or time.monotonic()-motor[1] > .2:
            raise RuntimeError('motor telemetry is stale')
        if not measured or time.monotonic()-measured[-1][0] > .2:
            raise RuntimeError('camera geometry is stale')
        s = motor[0]
        if int(s['flags']) & ~16 or any(abs(v) > 2.8 for v in s['current_a_host']):
            raise RuntimeError('motor fault/current guard')
        q = [s['encoder_counts'][0], -s['encoder_counts'][1]]
        if any(v < -16 or v > 140 for v in q):
            raise RuntimeError('staged exploration position guard')
        if measured[-1][1] < .08:
            raise RuntimeError('minimum 8 cm separation guard')

    def hold(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            check()

    def step(axis):
        before = list(motor[0]['encoder_counts'])
        before_gap, before_points = geometry()
        pwm = (150, 200)[axis]
        start = time.monotonic()
        command.publish(Int16MultiArray(data=[axis, -5, pwm]))
        moved = False
        seen_output = False
        while time.monotonic()-start < 1.2:
            rclpy.spin_once(node, timeout_sec=.01)
            check()
            state = motor[0]
            seen_output |= any(state['applied_commands'])
            delta = (state['encoder_counts'][axis]-before[axis])*(1 if axis == 0 else -1)
            if delta < -2 or delta > 17 or abs(state['encoder_counts'][1-axis]-before[1-axis]) > 2:
                raise RuntimeError('bounded step displacement/direction guard')
            moved |= delta >= 2
            if moved and not any(state['applied_commands']):
                break
            if measured[-1][1] <= .105:
                break
        halt()
        hold(.7)
        gap, points = geometry()
        entry = {'axis': axis, 'pwm': pwm, 'before': before,
                 'after': list(motor[0]['encoder_counts']),
                 'gap_before_m': before_gap, 'gap_after_m': gap,
                 'tip_changes_m': (points[:, -1]-before_points[:, -1]).tolist(),
                 'seen_active_output': bool(seen_output)}
        steps.append(entry)
        print(json.dumps(entry), flush=True)
        if not moved:
            raise RuntimeError('no completed encoder step; no automatic PWM increase')
        if gap > before_gap + .008:
            raise RuntimeError('step increased tip separation')
        return gap, points

    try:
        end = time.monotonic()+6
        while motor[0] is None or len(measured) < 8 or command.get_subscription_count() == 0:
            if time.monotonic() > end:
                raise RuntimeError('bridge/camera not ready')
            rclpy.spin_once(node, timeout_sec=.02)
        check()
        if any(motor[0]['applied_commands']):
            raise RuntimeError('motors are already active')
        gap, initial_points = geometry()
        segments = np.linalg.norm(np.diff(initial_points, axis=1), axis=2)
        result['initial_gap_m'] = gap
        result['initial_adjacent_marker_distances_m'] = segments.tolist()
        print('Initial gap / adjacent marker distances:', gap, segments.tolist(), flush=True)
        if not np.all((segments > .08) & (segments < .24)):
            raise RuntimeError('adjacent-marker scale sanity check failed')
        # No assumed mapping: each axis must measurably affect some tip within
        # 40 operational counts. This prevents winding an unresponsive tendon.
        axis_starts = [motor[0]['encoder_counts'][0], -motor[0]['encoder_counts'][1]]
        evidence = [[], []]
        with recorder(label, folder / 'logs', False):
            # Recorder startup waits without spinning this node. Drain queued
            # sensor callbacks before evaluating their freshness or moving.
            end = time.monotonic()+.6
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.005)
            hold(1)
            for cycle in range(24):
                for axis in (0, 1):
                    gap, points = geometry()
                    if gap <= .11:
                        result['target_reached'] = gap >= .09
                        break
                    q = (1 if axis == 0 else -1)*motor[0]['encoder_counts'][axis]
                    if q >= 120:
                        raise RuntimeError('120-count exploration budget reached')
                    gap, points = step(axis)
                    evidence[axis].append(np.asarray(steps[-1]['tip_changes_m']))
                    cumulative = np.sum(evidence[axis], axis=0)
                    q = (1 if axis == 0 else -1)*motor[0]['encoder_counts'][axis]
                    if q-axis_starts[axis] >= 40 and np.max(np.linalg.norm(cumulative, axis=1)) < .005:
                        raise RuntimeError(f'axis {axis}: 40 counts without clear tip response; inspect tendon')
                if gap <= .11:
                    result['target_reached'] = gap >= .09
                    break
            else:
                raise RuntimeError('maximum number of bounded steps reached')
            halt()
            hold(1)
    except BaseException as exc:
        result['error'] = str(exc)
        print('Stopped:', exc, flush=True)
    finally:
        halt()
        result['steps'] = steps
        result['final_encoder_counts'] = motor[0]['encoder_counts'] if motor[0] else None
        result['final_applied_commands'] = motor[0]['applied_commands'] if motor[0] else None
        if measured:
            result['last_raw_gap_m'] = measured[-1][1]
        (folder / 'result.json').write_text(json.dumps(result, indent=2))
        raw_file.close()
        motor_file.close()
        node.destroy_node()
        rclpy.shutdown()
        print('Report:', folder / 'result.json', flush=True)


if __name__ == '__main__':
    main()
