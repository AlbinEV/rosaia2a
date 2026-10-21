#!/usr/bin/env python3
"""Acquire two short, bounded cycles without changing the encoder/home reference."""

import json
import argparse
from pathlib import Path
import time

import rclpy
from std_msgs.msg import Int16MultiArray, String

from rosaia_workflow import ROOT, api, nodes, preflight, recorder, run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--return-first-axis0', type=int)
    options = parser.parse_args()
    preflight()
    if '/encoder_velocity_controller' in nodes():
        raise RuntimeError('stop velocity controller before calibration')
    label = time.strftime('small_%Y%m%d_%H%M%S')
    folder = ROOT / 'data/workflows' / label
    folder.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('small_motion_calibration')
    latest = [None, 0.0]
    steps = []

    def receive(msg):
        latest[:] = [json.loads(msg.data), time.monotonic()]

    sub = node.create_subscription(String, '/motor_state', receive, 20)
    step = node.create_publisher(Int16MultiArray, '/motor_step_command', 10)
    stop = node.create_publisher(Int16MultiArray, '/motor_command', 10)

    def halt():
        for _ in range(3):
            stop.publish(Int16MultiArray(data=[0, 0]))
            rclpy.spin_once(node, timeout_sec=0.02)

    def tick():
        rclpy.spin_once(node, timeout_sec=0.01)
        if latest[0] is None or time.monotonic() - latest[1] > .2:
            raise RuntimeError('stale motor telemetry')
        s = latest[0]
        if int(s['flags']) & ~16 or any(abs(c) > 2.8 for c in s['current_a_host']):
            raise RuntimeError('motor fault/current guard')
        if any(abs(s['encoder_counts'][i] - origin[i]) > 48 for i in (0, 1)):
            raise RuntimeError('48-count session travel guard')

    def hold(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            tick()

    def move(axis, direction, pwm):
        vision = api('/api/state')['vision']
        if set(vision.get('complete_arms', [])) != {'arm_1', 'arm_2'}:
            raise RuntimeError('markers missing')
        before = list(latest[0]['encoder_counts'])
        step.publish(Int16MultiArray(data=[axis, -direction * 5, pwm]))
        end = time.monotonic() + 1.2
        moved = False
        while time.monotonic() < end:
            tick()
            counts = latest[0]['encoder_counts']
            if abs(counts[1-axis] - before[1-axis]) > 2:
                raise RuntimeError('other motor moved')
            displacement = (counts[axis] - before[axis]) * (1 if axis == 0 else -1)
            if displacement * direction < -2 or abs(displacement) > 16:
                raise RuntimeError('step direction/travel guard')
            moved |= abs(displacement) >= 2
            if moved and not any(latest[0]['applied_commands']):
                break
        halt()
        hold(.7)
        after = list(latest[0]['encoder_counts'])
        steps.append(dict(axis=axis, direction=direction, pwm=pwm,
                          before=before, after=after))
        print(json.dumps(steps[-1]), flush=True)
        return abs(after[axis] - before[axis]) >= 2

    error = None
    try:
        end = time.monotonic() + 5
        while latest[0] is None or step.get_subscription_count() == 0:
            if time.monotonic() > end:
                raise RuntimeError('bridge unavailable')
            rclpy.spin_once(node, timeout_sec=.02)
        origin = list(latest[0]['encoder_counts'])
        if any(latest[0]['applied_commands']):
            raise RuntimeError('motors already moving')
        if options.return_first_axis0 is not None:
            for _ in range(6):
                if latest[0]['encoder_counts'][0] <= options.return_first_axis0 + 4:
                    break
                if not move(0, -1, 110):
                    raise RuntimeError('no return motion at PWM 110')
            else:
                raise RuntimeError('initial return step budget exceeded')
            origin = list(latest[0]['encoder_counts'])
        sessions = {}
        for role in ('train', 'validation'):
            name = f'{label}_{role}'
            with recorder(name, folder / 'logs', False):
                hold(1)
                for axis, forward, reverse in ((0, 110, 110), (1, 180, 120)):
                    sign = 1 if axis == 0 else -1
                    start = sign * latest[0]['encoder_counts'][axis]
                    for _ in range(5):
                        if sign * latest[0]['encoder_counts'][axis] >= start + 12:
                            break
                        if not move(axis, 1, forward):
                            if sign * latest[0]['encoder_counts'][axis] - start >= 8:
                                break
                            raise RuntimeError(f'axis {axis}: no outbound motion')
                    for _ in range(6):
                        if sign * latest[0]['encoder_counts'][axis] <= start + 4:
                            break
                        if not move(axis, -1, reverse):
                            # One modest retry after a stopped, bounded attempt.
                            if not move(axis, -1, reverse + 15):
                                raise RuntimeError(f'axis {axis}: no return motion')
                    else:
                        raise RuntimeError('return exceeded step budget')
                hold(1)
            sessions[role] = str(ROOT / 'data/sessions' / name)
        # Preserve the existing absolute encoder zero; never zero at a displaced pose.
        models = {}
        for arm in ('arm_1', 'arm_2'):
            target = ROOT / 'data/models' / f'{label}_{arm}'
            run([str(ROOT / 'scripts/rosaia.sh'), 'train', '--arm', arm,
                 '--train-session', sessions['train'], '--validation-session',
                 sessions['validation'], '--encoder-zero-raw', '0',
                 '--hidden', '8,8', '--max-iterations', '4000',
                 '--q-bin-width', '2', '--maximum-samples-per-session-bin', '30',
                 '--output', str(target)])
            models[arm] = str(target)
        (folder / 'models_pending_review.json').write_text(json.dumps(models, indent=2))
        print('Models trained, pending validation review:', models, flush=True)
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        halt()
        (folder / 'motion_log.json').write_text(json.dumps(
            {'steps': steps, 'error': error, 'encoder_zero_changed': False}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
