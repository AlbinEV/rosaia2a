#!/usr/bin/env python3
"""Replay saved close poses using the same bounded manual-jog safety checks."""

import argparse
import json
import time

from rosaia_workflow import ROOT, api
from urllib.request import Request, urlopen


def control(path, payload=None):
    request = Request('http://127.0.0.1:8767'+path,
                      data=None if payload is None else json.dumps(payload).encode(),
                      headers={'Content-Type': 'application/json'})
    with urlopen(request, timeout=3) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycles', type=int, choices=(1, 2), default=2)
    options = parser.parse_args()
    nominal_path = ROOT / 'data/control_tests/jog_nominals.json'
    nominal = json.loads(nominal_path.read_text())
    close = [nominal[str(axis)]['close']['q'] for axis in (0, 1)]
    if any(type(q) is not int or q <= 0 for q in close):
        raise RuntimeError('both saved close positions are required')
    label = time.strftime('saved_closures_%Y%m%d_%H%M%S')
    folder = ROOT / 'data/workflows' / label
    folder.mkdir(parents=True)
    result = {'label': label, 'close_targets': close, 'home_targets': [0, 0],
              'cycles_requested': options.cycles, 'completed': False, 'stages': []}

    def record():
        (folder / 'result.json').write_text(json.dumps(result, indent=2))

    def checked():
        state = control('/simple/status')
        if not state['ready'] or not state['armed']:
            raise RuntimeError('camera/telemetry/HOME unavailable')
        return state

    def stage(name, targets):
        initial = checked()
        if initial['moving']:
            raise RuntimeError('another motion is active')
        directions = [1 if targets[i] > initial['jog_positions'][i] else -1 for i in (0, 1)]
        event = {'name': name, 'targets': targets, 'initial': initial['jog_positions'],
                 'start_ns': time.time_ns(), 'steps': []}
        result['stages'].append(event)
        print(f'{name}: {initial["jog_positions"]} -> {targets}', flush=True)
        deadline = time.monotonic()+90
        for _ in range(60):
            changed = False
            for axis in (0, 1):
                state = checked()
                q = state['jog_positions'][axis]
                if directions[axis]*(targets[axis]-q) <= 4:
                    continue
                if time.monotonic() > deadline:
                    raise RuntimeError('stage timeout')
                amount = min(5, abs(targets[axis]-q))
                control('/simple/jog', {'axis': axis, 'direction': directions[axis],
                                        'step_counts': amount})
                end = time.monotonic()+3
                while time.monotonic() < end:
                    time.sleep(.06)
                    state = checked()
                    if not state['moving']:
                        break
                else:
                    raise RuntimeError('jog timeout')
                if state['error']:
                    raise RuntimeError(state['error'])
                if abs(state['jog_positions'][axis]-q) < 1:
                    raise RuntimeError('no encoder progress')
                event['steps'].append({'axis': axis, 'before_q': q,
                                       'after': state['jog_positions'], 'time_ns': time.time_ns()})
                print(f'  axis {axis}: {state["jog_positions"]}', flush=True)
                record()
                changed = True
            if not changed:
                break
        else:
            raise RuntimeError('stage step budget exceeded')
        time.sleep(.8)
        event['final'] = checked()['jog_positions']
        event['observed_shapes'] = {arm: value['current'] for arm, value in api('/api/state')['arms'].items()}
        event['end_ns'] = time.time_ns()
        record()
        print(f'{name} complete: {event["final"]}', flush=True)

    try:
        for cycle in range(options.cycles):
            stage(f'HOME_{cycle+1}', [0, 0])
            stage(f'CLOSE_{cycle+1}', close)
        result['completed'] = True
    except BaseException as exc:
        result['error'] = str(exc)
        print('Stopped:', exc, flush=True)
    finally:
        control('/simple/stop', {})
        result['final_status'] = control('/simple/status')
        record()
        print('Report:', folder / 'result.json', flush=True)


if __name__ == '__main__':
    main()
