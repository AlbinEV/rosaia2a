#!/usr/bin/env python3
"""Check one five-count contraction and return per axis, with live limits."""

import json
import argparse
from pathlib import Path
import time
import subprocess

import rclpy
from std_msgs.msg import Int16MultiArray, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', type=int, choices=(0, 1))
    parser.add_argument('--step-timeout', type=float, default=1.0)
    parser.add_argument('--pwm-increase', type=int, default=0)
    parser.add_argument('--return-pwm', type=int)
    parser.add_argument('--outbound-only', action='store_true')
    options = parser.parse_args()
    if not 0.2 <= options.step_timeout <= 2.5:
        parser.error('--step-timeout must be between 0.2 and 2.5 seconds')
    if not 0 <= options.pwm_increase <= 20:
        parser.error('--pwm-increase must be between 0 and 20')
    if options.return_pwm is not None and not 1 <= options.return_pwm <= 180:
        parser.error('--return-pwm must be between 1 and 180')
    rclpy.init()
    node = rclpy.create_node('motor_mapping_check')
    state = [None, 0.0]
    trace = []
    result = []

    def receive(message):
        state[0] = json.loads(message.data)
        state[1] = time.monotonic()
        trace.append({'host_time': state[1], **state[0]})

    subscription = node.create_subscription(String, '/motor_state', receive, 20)
    step = node.create_publisher(Int16MultiArray, '/motor_step_command', 10)
    stop = node.create_publisher(Int16MultiArray, '/motor_command', 10)

    def tick():
        rclpy.spin_once(node, timeout_sec=0.01)
        if state[0] is None or time.monotonic() - state[1] > 0.2:
            raise RuntimeError('telemetry unavailable or stale')
        if any(abs(x) > 2.8 for x in state[0]['current_a_host']):
            raise RuntimeError('host current estimate exceeded 2.8 A')
        # A previous bounded-step timeout remains latched while stopped.
        # STEP_AXIS clears it; the host deadline remains below firmware timeout.
        allowed_flags = 17 if not any(state[0]['applied_commands']) else 16
        if int(state[0]['flags']) & ~allowed_flags:
            raise RuntimeError(f'firmware flags={state[0]["flags"]}')

    def move(axis, delta, pwm):
        initial = list(state[0]['encoder_counts'])
        step.publish(Int16MultiArray(data=[axis, delta, pwm]))
        deadline = time.monotonic() + options.step_timeout
        while time.monotonic() < deadline:
            tick()
            counts = state[0]['encoder_counts']
            if abs(counts[1-axis] - initial[1-axis]) > 2:
                raise RuntimeError('other encoder moved')
            displacement = counts[axis] - initial[axis]
            if abs(displacement) > 12:
                raise RuntimeError('step exceeded diagnostic range')
            if abs(displacement) >= 3 and not any(state[0]['applied_commands']):
                target_time = time.monotonic() + 0.2
                while time.monotonic() < target_time:
                    tick()
                final = list(state[0]['encoder_counts'])
                entry = {'axis': axis, 'signed_step': delta, 'pwm': pwm,
                         'initial_counts': initial, 'final_counts': final}
                result.append(entry)
                print(json.dumps(entry), flush=True)
                return final[axis] - initial[axis]
        raise RuntimeError(f'axis {axis}: no completed small step within {options.step_timeout:g} seconds')

    root = Path(__file__).resolve().parents[1]
    folder = root / 'data/control_tests'
    folder.mkdir(parents=True, exist_ok=True)
    error = None
    try:
        deadline = time.monotonic() + 5
        while state[0] is None or step.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('bridge not ready')
            rclpy.spin_once(node, timeout_sec=0.02)
        active_nodes = subprocess.run(
            ['ros2', 'node', 'list'], capture_output=True, text=True,
            check=True, timeout=5,
        ).stdout.splitlines()
        if '/encoder_velocity_controller' in active_nodes:
            raise RuntimeError('stop encoder_velocity_controller first')
        tick()
        if any(state[0]['applied_commands']):
            raise RuntimeError('motors are already active')
        for axis, outbound_pwm, return_pwm, expected_sign in (
                (0, 110, 105, 1), (1, 180, 120, -1)):
            if options.axis is not None and options.axis != axis:
                continue
            displacement = move(axis, -5, outbound_pwm + options.pwm_increase)
            if displacement * expected_sign <= 0:
                raise RuntimeError(f'axis {axis}: encoder sign differs from configuration')
            if options.outbound_only:
                continue
            returned = move(axis, min(abs(displacement), 10),
                            options.return_pwm if options.return_pwm is not None
                            else return_pwm + options.pwm_increase)
            if returned * expected_sign >= 0:
                raise RuntimeError(f'axis {axis}: return sign differs from configuration')
    except (RuntimeError, KeyboardInterrupt) as exc:
        error = str(exc) or 'interrupted'
        print(f'Test stopped: {error}', flush=True)
    finally:
        for _ in range(3):
            stop.publish(Int16MultiArray(data=[0, 0]))
            rclpy.spin_once(node, timeout_sec=0.03)
        path = folder / f'mapping_{time.strftime("%Y%m%d_%H%M%S")}.json'
        path.write_text(json.dumps({'steps': result, 'error': error, 'trace': trace}, indent=2))
        print(f'Log: {path}', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    if error:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
