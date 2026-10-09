#!/usr/bin/env python3
"""Move both encoder axes synchronously and return them to home."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

from run_controlled_position_test import API, ROOT, request


TERMINAL_STATES = {'complete', 'error', 'cancelled'}


def execute(writer, targets: dict[str, int], phase: str, timeout: float) -> None:
    """Execute and log one coordinated two-axis target."""
    vision_deadline = time.monotonic() + 5.0
    while True:
        try:
            response = request('/api/positions', {'targets': targets})
            break
        except RuntimeError as error:
            if (
                'vision is incomplete' not in str(error)
                or time.monotonic() >= vision_deadline
            ):
                raise
            time.sleep(0.1)
    print(
        f'{phase}: q={response["current_q"]} -> {targets} '
        f'({response["mode"]})', flush=True,
    )
    started = time.monotonic()
    deadline = started + timeout
    last_print = -1.0
    while time.monotonic() < deadline:
        state = request('/api/state')
        motion = state.get('motion') or {}
        motor = state.get('motor') or {}
        current_q = motion.get('current_q') or {}
        motion_error = motion.get('error')
        errors = motion_error if isinstance(motion_error, dict) else {}
        references = motion.get('velocity_reference') or {}
        reversals = motion.get('reversals') or {}
        counts = motor.get('encoder_counts') or [None, None]
        commands = motor.get('applied_commands') or [None, None]
        currents = motor.get('current_a_host') or [None, None]
        elapsed = time.monotonic() - started
        row = {
            'phase': phase, 'elapsed_s': elapsed,
            'state': motion.get('state'),
            'arm_1_target_q': targets['arm_1'],
            'arm_1_current_q': current_q.get('arm_1'),
            'arm_1_error_q': errors.get('arm_1'),
            'arm_1_velocity_reference': references.get('arm_1'),
            'arm_1_reversals': reversals.get('arm_1'),
            'arm_1_encoder_raw': counts[0],
            'arm_1_applied_pwm': commands[0],
            'arm_1_current_a': currents[0],
            'arm_2_target_q': targets['arm_2'],
            'arm_2_current_q': current_q.get('arm_2'),
            'arm_2_error_q': errors.get('arm_2'),
            'arm_2_velocity_reference': references.get('arm_2'),
            'arm_2_reversals': reversals.get('arm_2'),
            'arm_2_encoder_raw': counts[1],
            'arm_2_applied_pwm': commands[1],
            'arm_2_current_a': currents[1],
            'flags': motor.get('flags'),
        }
        writer.writerow(row)
        if elapsed - last_print >= 0.25:
            print(
                f'  t={elapsed:5.2f}s {row["state"]:>14} '
                f'q=({row["arm_1_current_q"]},{row["arm_2_current_q"]}) '
                f'e=({row["arm_1_error_q"]},{row["arm_2_error_q"]}) '
                f'pwm=({commands[0]},{commands[1]})', flush=True,
            )
            last_print = elapsed
        if row['state'] in TERMINAL_STATES:
            if row['state'] != 'complete':
                raise RuntimeError(
                    f'{phase} ended as {row["state"]}: '
                    f'{motion_error or ""}'
                )
            time.sleep(0.1)
            return
        time.sleep(0.05)
    request('/api/cancel', {})
    raise RuntimeError(f'{phase} timed out after {timeout:.1f} s')


def arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-1-target', type=int, required=True)
    parser.add_argument('--arm-2-target', type=int, required=True)
    parser.add_argument('--home', type=int, default=0)
    parser.add_argument('--no-return-home', action='store_true')
    parser.add_argument('--timeout', type=float, default=40.0)
    parser.add_argument('--label')
    return parser.parse_args()


def main() -> None:
    """Run a synchronized outbound and return-home test."""
    options = arguments()
    label = options.label or datetime.now().strftime('%Y%m%d_%H%M%S')
    output = ROOT / 'data' / 'control_tests' / f'{label}_both.csv'
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        'phase', 'elapsed_s', 'state',
        'arm_1_target_q', 'arm_1_current_q', 'arm_1_error_q',
        'arm_1_velocity_reference', 'arm_1_reversals',
        'arm_1_encoder_raw', 'arm_1_applied_pwm', 'arm_1_current_a',
        'arm_2_target_q', 'arm_2_current_q', 'arm_2_error_q',
        'arm_2_velocity_reference', 'arm_2_reversals',
        'arm_2_encoder_raw', 'arm_2_applied_pwm', 'arm_2_current_a', 'flags',
    ]
    try:
        with output.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            execute(writer, {
                'arm_1': options.arm_1_target,
                'arm_2': options.arm_2_target,
            }, 'outbound', options.timeout)
            if not options.no_return_home:
                execute(writer, {
                    'arm_1': options.home, 'arm_2': options.home,
                }, 'return_home', options.timeout)
    except KeyboardInterrupt:
        request('/api/cancel', {})
        raise SystemExit(130)
    except RuntimeError as error:
        try:
            request('/api/cancel', {})
        except RuntimeError:
            pass
        print(f'test failed: {error}', file=sys.stderr)
        raise SystemExit(2) from error
    print(f'test complete; log: {output}')


if __name__ == '__main__':
    main()
