#!/usr/bin/env python3
"""Find a small inward bend from an author-confirmed home; no encoder reset."""

import json
import argparse
import time

import numpy as np
import rclpy
from std_msgs.msg import Int16MultiArray, String

from rosaia_workflow import ROOT, api, nodes, preflight, recorder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--return-only', action='store_true')
    parser.add_argument('--axis', type=int, choices=(0, 1))
    parser.add_argument('--explore', action='store_true',
                        help='observe both camera arms without assuming motor association')
    parser.add_argument('--pulse-only', action='store_true',
                        help='one outbound pulse per axis, followed by coarse return')
    parser.add_argument('--pwm0', type=int, default=110)
    parser.add_argument('--pwm1', type=int, default=180)
    parser.add_argument('--return-pwm0', type=int, default=110)
    parser.add_argument('--step-counts', type=int, default=5)
    parser.add_argument('--step-seconds', type=float, default=1.2)
    parser.add_argument('--timed-seconds', type=float, default=0,
                        help='outbound PWM duration, stopped early at 40 counts')
    options = parser.parse_args()
    if not 1 <= options.pwm0 <= 200 or not 1 <= options.pwm1 <= 200:
        parser.error('diagnostic PWM must be between 1 and 200')
    if not 1 <= options.return_pwm0 <= 120:
        parser.error('return PWM must be between 1 and 120')
    if not 1 <= options.step_counts <= 10 or not .2 <= options.step_seconds <= 2:
        parser.error('diagnostic step limit: 1..10 counts, .2..2 seconds')
    if not 0 <= options.timed_seconds <= .5:
        parser.error('timed impulse must be between 0 and .5 seconds')
    initial = preflight()
    if not initial['home_calibrated'] or (not options.return_only and
            any(abs(q) > 4 for q in initial['motor']['encoder_counts'])):
        raise RuntimeError('requires confirmed, zeroed OPEN home')
    if '/encoder_velocity_controller' in nodes():
        raise RuntimeError('stop velocity controller first')
    label = time.strftime('open_close_%Y%m%d_%H%M%S')
    folder = ROOT / 'data/workflows' / label
    folder.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('find_open_close')
    latest = [None, 0.0]
    trace = []
    def receive(msg):
        latest[:] = [json.loads(msg.data), time.monotonic()]
    sub = node.create_subscription(String, '/motor_state', receive, 20)
    step = node.create_publisher(Int16MultiArray, '/motor_step_command', 10)
    stop = node.create_publisher(Int16MultiArray, '/motor_command', 10)

    def halt():
        for _ in range(3):
            stop.publish(Int16MultiArray(data=[0, 0]))
            rclpy.spin_once(node, timeout_sec=.02)

    def tick():
        rclpy.spin_once(node, timeout_sec=.01)
        if latest[0] is None or time.monotonic() - latest[1] > .2:
            raise RuntimeError('stale motor telemetry')
        s = latest[0]
        if int(s['flags']) & ~16 or any(abs(c) > 2.8 for c in s['current_a_host']):
            raise RuntimeError('motor fault/current guard')
        q = [s['encoder_counts'][0], -s['encoder_counts'][1]]
        upper = [56, 56]
        if options.return_only:
            upper = [max(56, initial['motor']['encoder_counts'][0] + 4),
                     max(56, -initial['motor']['encoder_counts'][1] + 4)]
        if any(v < -16 or v > limit for v, limit in zip(q, upper)):
            raise RuntimeError('small excursion guard')

    def hold(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            tick()

    def measure():
        samples = []
        frames = set()
        end = time.monotonic() + 2.0
        while len(samples) < 4 and time.monotonic() < end:
            hold(.06)
            s = api('/api/state')
            v = s['vision']
            if set(v.get('complete_arms', [])) != {'arm_1', 'arm_2'}:
                raise RuntimeError('incomplete vision')
            points = [[p['x'], p['y'], p['z']]
                      for arm in ('arm_1', 'arm_2')
                      for p in s['arms'][arm]['current']['points']]
            # Status is published at 1 Hz, point clouds at camera rate.
            key = tuple(value for point in points for value in point)
            if key in frames:
                continue
            frames.add(key)
            samples.append(points)
        if len(samples) < 4:
            raise RuntimeError('vision is not updating')
        return np.median(np.asarray(samples), axis=0).reshape(2, 3, 3)

    def move(axis, direction, pwm):
        measure()  # Require fresh complete vision before each bounded pulse.
        before = list(latest[0]['encoder_counts'])
        if direction > 0 and options.timed_seconds:
            started = time.monotonic()
            deadline = started + options.timed_seconds
            next_send = started
            travel_guard = False
            while time.monotonic() < deadline:
                tick()
                c = latest[0]['encoder_counts']
                delta = (c[axis] - before[axis]) * (1 if axis == 0 else -1)
                if abs(c[1-axis] - before[1-axis]) > 2 or delta < -2:
                    raise RuntimeError('timed pulse direction/other encoder guard')
                if abs(delta) >= 40:
                    travel_guard = True
                    break
                if time.monotonic() >= next_send:
                    command = [0, 0]
                    command[axis] = -pwm
                    stop.publish(Int16MultiArray(data=command))
                    next_send = time.monotonic() + .04
            elapsed = time.monotonic() - started
            halt()
            hold(.5)
            after = list(latest[0]['encoder_counts'])
            entry = dict(axis=axis, direction=direction, pwm=pwm,
                         before=before, after=after, timed_duration_s=elapsed,
                         requested_duration_s=options.timed_seconds,
                         travel_guard_triggered=travel_guard)
            trace.append(entry)
            print(json.dumps(entry), flush=True)
            return abs(after[axis]-before[axis]) >= 2
        count_limit = options.step_counts if direction > 0 else 5
        step.publish(Int16MultiArray(data=[axis, -direction*count_limit, pwm]))
        end = time.monotonic() + options.step_seconds
        moved = False
        while time.monotonic() < end:
            tick()
            c = latest[0]['encoder_counts']
            delta = (c[axis] - before[axis]) * (1 if axis == 0 else -1)
            if abs(c[1-axis] - before[1-axis]) > 2 or abs(delta) > count_limit + 12:
                raise RuntimeError('pulse excursion/other encoder guard')
            if delta*direction < -2:
                raise RuntimeError('unexpected motor direction')
            moved |= abs(delta) >= 2
            if moved and not any(latest[0]['applied_commands']):
                break
        halt()
        hold(.5)
        after = list(latest[0]['encoder_counts'])
        entry = dict(axis=axis, direction=direction, pwm=pwm, before=before, after=after,
                     count_limit=count_limit, maximum_duration_s=options.step_seconds)
        trace.append(entry)
        print(json.dumps(entry), flush=True)
        return abs(after[axis]-before[axis]) >= 2

    result = {'label': label, 'open': {'arm_1': 0, 'arm_2': 0},
              'close_candidates': {}, 'verified': False}
    try:
        end = time.monotonic() + 5
        while latest[0] is None or step.get_subscription_count() == 0:
            if time.monotonic() > end:
                raise RuntimeError('bridge unavailable')
            rclpy.spin_once(node, timeout_sec=.02)
        with recorder(label, folder / 'logs', False):
            home = measure()
            result['home_points'] = home.tolist()
            for axis, forward, reverse in ((0, options.pwm0, options.return_pwm0), (1, options.pwm1, 120)):
                if options.axis is not None and axis != options.axis:
                    continue
                sign = 1 if axis == 0 else -1
                inward_sign = 1 if axis == 0 else -1
                found = False
                observed_camera_arm = None
                inward = 0.0
                for _ in range(0 if options.return_only else (1 if options.pulse_only else 8)):
                    if sign*latest[0]['encoder_counts'][axis] >= 40:
                        break
                    moved = move(axis, 1, forward)
                    if not moved and axis == 0 and forward == 110 and not options.pulse_only:
                        forward = 130
                        moved = move(axis, 1, forward)
                    if not moved:
                        print(f'axis {axis}: stalled; ending outbound search', flush=True)
                        break
                    observed = measure()
                    visual_axis = axis
                    if options.explore:
                        tip_changes = observed[:, -1] - home[:, -1]
                        visual_axis = int(np.argmax(np.linalg.norm(tip_changes, axis=1)))
                        observed_camera_arm = f'arm_{visual_axis+1}'
                        inward_sign = 1 if visual_axis == 0 else -1
                        print('Both camera tip changes [mm]:',
                              (1000*tip_changes).round(2).tolist(), flush=True)
                    inward = inward_sign * (observed[visual_axis, -1, 0] - home[visual_axis, -1, 0])
                    print(f'axis {axis}: inward tip change {inward*1000:.1f} mm', flush=True)
                    trace[-1]['observed_points'] = observed.tolist()
                    trace[-1]['inward_tip_m'] = float(inward)
                    if inward < -.008:
                        raise RuntimeError(f'axis {axis}: tip moved outward, not closing')
                    if inward >= .008:
                        found = True
                        break
                candidate_key = f'axis_{axis}' if options.explore else f'arm_{axis+1}'
                result['close_candidates'][candidate_key] = {
                    'q': sign*latest[0]['encoder_counts'][axis], 'inward_verified': found,
                    'forward_pwm': forward, 'reverse_pwm': reverse,
                    'observed_camera_arm': observed_camera_arm,
                    'inward_tip_m': float(inward)}
                # Coarse return: stop on first crossing, never chase a precise zero.
                for _ in range(10):
                    if sign*latest[0]['encoder_counts'][axis] <= 4:
                        break
                    if not move(axis, -1, reverse):
                        raise RuntimeError(f'axis {axis}: return stalled')
                else:
                    raise RuntimeError('return step budget exceeded')
                returned = measure()
                visual_axis = int(observed_camera_arm[-1])-1 if observed_camera_arm else axis
                result['close_candidates'][candidate_key]['return_tip_error_m'] = float(
                    np.linalg.norm(returned[visual_axis, -1] - home[visual_axis, -1]))
            result['verified'] = len(result['close_candidates']) == 2 and all(c['inward_verified'] and c['return_tip_error_m'] < .01
                                     for c in result['close_candidates'].values())
            print(json.dumps(result, indent=2), flush=True)
    except BaseException as exc:
        result['error'] = str(exc)
        raise
    finally:
        halt()
        result['steps'] = trace
        result['final_encoder_counts'] = latest[0]['encoder_counts'] if latest[0] else None
        (folder / 'result.json').write_text(json.dumps(result, indent=2))
        print('Report:', folder / 'result.json', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
