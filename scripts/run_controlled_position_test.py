#!/usr/bin/env python3
"""Run and log one controlled encoder-position move with optional return home."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
API = 'http://127.0.0.1:8765'
TERMINAL_STATES = {'complete', 'error', 'cancelled'}


def request(path: str, payload: dict | None = None) -> dict:
    """Call the local dashboard API."""
    body = None if payload is None else json.dumps(payload).encode()
    call = Request(
        API + path,
        data=body,
        headers={'Content-Type': 'application/json'} if body else {},
        method='POST' if body else 'GET',
    )
    try:
        with urlopen(call, timeout=3.0) as response:
            return json.load(response)
    except HTTPError as error:
        detail = error.read().decode(errors='replace')
        raise RuntimeError(f'{path}: HTTP {error.code}: {detail}') from error
    except URLError as error:
        raise RuntimeError(f'dashboard unavailable: {error.reason}') from error


def sample_row(phase: str, started: float, state: dict, arm: str) -> dict:
    """Flatten live motion and motor state into one CSV row."""
    motion = state.get('motion') or {}
    motor = state.get('motor') or {}
    axis = 0 if arm == 'arm_1' else 1
    values = motor.get('encoder_counts') or [None, None]
    commands = motor.get('applied_commands') or [None, None]
    currents = motor.get('current_a_host') or [None, None]
    return {
        'phase': phase,
        'elapsed_s': time.monotonic() - started,
        'state': motion.get('state'),
        'target_q': motion.get('target_q'),
        'current_q': motion.get('current_q'),
        'error_q': motion.get('error'),
        'velocity_reference': motion.get('velocity_reference'),
        'reversals': motion.get('reversals'),
        'encoder_raw': values[axis],
        'applied_pwm': commands[axis],
        'current_a': currents[axis],
        'flags': motor.get('flags'),
    }


def execute_and_log(
    writer: csv.DictWriter,
    arm: str,
    target: int,
    phase: str,
    timeout: float,
) -> dict:
    """Execute one target and log until the controller reaches a terminal state."""
    response = request('/api/position', {'arm': arm, 'target_q': target})
    print(
        f'{phase}: q={response["current_q"]} -> {target} '
        f'({response["mode"]})',
        flush=True,
    )
    started = time.monotonic()
    deadline = started + timeout
    last_print = -1.0
    while time.monotonic() < deadline:
        state = request('/api/state')
        row = sample_row(phase, started, state, arm)
        writer.writerow(row)
        elapsed = float(row['elapsed_s'])
        if elapsed - last_print >= 0.25:
            print(
                f'  t={elapsed:5.2f}s state={row["state"]:>14} '
                f'q={row["current_q"]} e={row["error_q"]} '
                f'qdot_ref={row["velocity_reference"]} '
                f'pwm={row["applied_pwm"]}',
                flush=True,
            )
            last_print = elapsed
        if row['state'] in TERMINAL_STATES:
            if row['state'] != 'complete':
                raise RuntimeError(
                    f'{phase} ended as {row["state"]}: '
                    f'{state.get("motion", {}).get("error", "")}'
                )
            # The motion thread publishes its final zero references after
            # exposing the complete state. Let it exit before the next target.
            time.sleep(0.1)
            return state
        time.sleep(0.05)
    request('/api/cancel', {})
    raise RuntimeError(f'{phase} timed out after {timeout:.1f} s')


def arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=('arm_1', 'arm_2'), required=True)
    parser.add_argument('--target', type=int, required=True)
    parser.add_argument('--home', type=int, default=0)
    parser.add_argument('--no-return-home', action='store_true')
    parser.add_argument('--timeout', type=float, default=40.0)
    parser.add_argument('--label')
    return parser.parse_args()


def main() -> None:
    """Run the outbound move and return-home sequence."""
    options = arguments()
    label = options.label or datetime.now().strftime('%Y%m%d_%H%M%S')
    output = ROOT / 'data' / 'control_tests' / f'{label}_{options.arm}.csv'
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = list(sample_row('', 0.0, {}, options.arm))
    try:
        with output.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            execute_and_log(
                writer, options.arm, options.target, 'outbound', options.timeout
            )
            if not options.no_return_home and options.target != options.home:
                execute_and_log(
                    writer, options.arm, options.home, 'return_home',
                    options.timeout,
                )
    except KeyboardInterrupt:
        request('/api/cancel', {})
        print('\ncancel requested', file=sys.stderr)
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
