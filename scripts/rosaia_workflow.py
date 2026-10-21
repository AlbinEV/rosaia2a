#!/usr/bin/env python3
"""One-command bring-up, calibration, training, and concept demonstration."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
import webbrowser


ROOT = Path(__file__).resolve().parents[1]
ROSAIA = ROOT / 'scripts' / 'rosaia.sh'
DATA = ROOT / 'data'
DASHBOARD = 'http://127.0.0.1:8765'
RUNTIME = Path(f'/tmp/rosaia_workflow_{os.getuid()}.json')


def run(command: list[str], *, dry_run: bool = False) -> None:
    """Print and execute one workflow command."""
    print('+', ' '.join(command), flush=True)
    if not dry_run:
        subprocess.run(command, cwd=ROOT, check=True)


def api(path: str, payload: dict | None = None) -> dict:
    """Read or update the local dashboard API."""
    body = None if payload is None else json.dumps(payload).encode()
    request = Request(
        DASHBOARD + path,
        data=body,
        headers={'Content-Type': 'application/json'} if body else {},
        method='POST' if body else 'GET',
    )
    try:
        with urlopen(request, timeout=3.0) as response:
            return json.load(response)
    except HTTPError as error:
        detail = error.read().decode(errors='replace')
        raise RuntimeError(f'{path}: HTTP {error.code}: {detail}') from error
    except URLError as error:
        raise RuntimeError(f'dashboard unavailable: {error.reason}') from error


def nodes() -> set[str]:
    result = subprocess.run(
        ['ros2', 'node', 'list'], text=True, capture_output=True, check=True
    )
    return set(result.stdout.splitlines())


def preflight(*, require_complete_vision: bool = True) -> dict:
    """Reject calibration unless telemetry and both visual states are valid."""
    state = api('/api/state')
    motor = state.get('motor') or {}
    vision = state.get('vision') or {}
    counts = motor.get('encoder_counts')
    if not isinstance(counts, list) or len(counts) != 2:
        raise RuntimeError('motor telemetry unavailable')
    if any(int(value) for value in motor.get('applied_commands', [])):
        raise RuntimeError('motor outputs are not stopped')
    flags = int(motor.get('flags', 0))
    if flags & ~16:
        raise RuntimeError(f'unexpected firmware flags={flags}')
    if require_complete_vision and set(vision.get('complete_arms', [])) != {'arm_1', 'arm_2'}:
        raise RuntimeError('both complete visual arm states are required')
    if require_complete_vision and vision.get('implementation') != 'cpp':
        raise RuntimeError('the C++ ArUco tracker is not active')
    print(
        f'preflight OK: encoder={counts}, flags={flags}, '
        f"vision={vision.get('processed_fps', 0):.1f} Hz, "
        f"home={'OK' if state.get('home_calibrated') else 'required'}",
        flush=True,
    )
    return state


def call_trigger(service: str, *, dry_run: bool = False) -> None:
    run(
        ['ros2', 'service', 'call', service, 'std_srvs/srv/Trigger', '{}'],
        dry_run=dry_run,
    )


def wait_for(predicate, description: str, timeout: float = 12.0) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = api('/api/state')
        if predicate(state):
            return state
        time.sleep(0.2)
    raise RuntimeError(f'timeout waiting for {description}')


@contextmanager
def recorder(name: str, log_directory: Path, dry_run: bool):
    """Run one recorder and always request a clean ROS shutdown."""
    command = [str(ROSAIA), 'recorder', name]
    print('+', ' '.join(command), flush=True)
    if dry_run:
        yield
        return
    log_directory.mkdir(parents=True, exist_ok=True)
    with (log_directory / f'{name}.log').open('w') as output:
        process = subprocess.Popen(
            command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            time.sleep(1.0)
            if process.poll() is not None:
                raise RuntimeError(f'recorder {name} exited during startup')
            yield
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3.0)


def sweep_command(axis: int, points: str, dwell: float) -> list[str]:
    common = [
        str(ROSAIA), 'sweep', '--axis', str(axis),
        '--command-to-q-sign', '-1', '--points', points,
        '--step-counts', '5', '--dwell', str(dwell),
        '--tolerance', '5', '--step-timeout', '3', '--maximum-q', '220',
    ]
    if axis == 0:
        return common + [
            '--encoder-sign', '1', '--pwm', '110',
            '--pwm-schedule', '0:110,190:130', '--reverse-pwm', '105',
        ]
    return common + [
        '--encoder-sign', '-1', '--pwm', '180', '--reverse-pwm', '120',
    ]


def acquire_session(
    name: str, axis: int, points: str, dwell: float,
    logs: Path, dry_run: bool,
) -> Path:
    with recorder(name, logs, dry_run):
        run(sweep_command(axis, points, dwell), dry_run=dry_run)
    return DATA / 'sessions' / name


def audit(session: Path, arm: str, maximum_q: int, report: Path, dry: bool):
    run([
        str(ROSAIA), 'audit', '--session', str(session), '--arm', arm,
        '--minimum-rows', '100', '--minimum-q-span', str(int(0.75 * maximum_q)),
        '--output', str(report),
    ], dry_run=dry)


def train(
    arm: str, training: Path, validation: Path, model: Path,
    maximum_q: int, dry: bool,
):
    run([
        str(ROSAIA), 'train', '--arm', arm,
        '--train-session', str(training),
        '--validation-session', str(validation),
        '--hidden', '32,32', '--max-iterations', '4000',
        '--q-bin-width', '5', '--maximum-samples-per-session-bin', '50',
        '--encoder-zero-raw', '0', '--minimum-q', '0',
        '--maximum-q', str(maximum_q), '--output', str(model),
    ], dry_run=dry)


def calibration(options: argparse.Namespace) -> None:
    """Acquire two held-out sweeps per arm and train fresh demo models."""
    if getattr(options, 'quick', False):
        options.train_points = '0,10,20,30,40'
        options.validation_points = '0,5,15,25,35,40'
        options.dwell = max(options.dwell, 1.0)
    preflight()
    if not options.yes:
        answer = input(
            'Bracci in HOME meccanica, camera fissata, marker visibili e '
            'alimentazione motori pronta. Scrivi HOME per continuare: '
        )
        if answer.strip().upper() != 'HOME':
            raise RuntimeError('calibration cancelled')

    call_trigger('/zero_encoders', dry_run=options.dry_run)
    call_trigger('/aruco/reset_reference', dry_run=options.dry_run)
    if not options.dry_run:
        wait_for(
            lambda state: state['motor']['encoder_counts'] == [0, 0],
            'encoder zero',
        )
        wait_for(
            lambda state: state['vision'].get('reference_frame_locked')
            and set(state['vision'].get('complete_arms', []))
            == {'arm_1', 'arm_2'},
            'reference-frame lock',
        )
        preflight()

    label = options.label or datetime.now().strftime('%Y%m%d_%H%M%S')
    workflow_root = DATA / 'workflows' / label
    logs = workflow_root / 'logs'
    reports = workflow_root / 'reports'
    if not options.dry_run:
        reports.mkdir(parents=True, exist_ok=True)
    maximum_q = max(int(value) for value in options.train_points.split(','))
    if max(int(value) for value in options.validation_points.split(',')) > maximum_q:
        raise ValueError('validation points must remain inside the train range')

    sessions = {}
    for arm, axis in (('arm_1', 0), ('arm_2', 1)):
        for role, points in (
            ('train', options.train_points),
            ('validation', options.validation_points),
        ):
            name = f'{label}_{arm}_{role}'
            print(f'\n=== {arm} · {role} ===', flush=True)
            sessions[(arm, role)] = acquire_session(
                name, axis, points, options.dwell, logs, options.dry_run
            )
            audit(
                sessions[(arm, role)], arm, maximum_q,
                reports / f'{name}_audit.json', options.dry_run,
            )

    models = {
        'arm_1': (DATA / 'models' / f'{label}_arm1_mlp').resolve(),
        'arm_2': (DATA / 'models' / f'{label}_arm2_mlp').resolve(),
    }
    for arm in ('arm_1', 'arm_2'):
        train(
            arm, sessions[(arm, 'train')], sessions[(arm, 'validation')],
            models[arm], maximum_q, options.dry_run,
        )

    manifest = {
        'label': label,
        'created_local': datetime.now().astimezone().isoformat(),
        'motor_supply_v': 5.6,
        'train_points': options.train_points,
        'validation_points': options.validation_points,
        'sessions': {f'{arm}_{role}': str(path.resolve())
                     for (arm, role), path in sessions.items()},
        'models': {arm: str(path) for arm, path in models.items()},
    }
    if not options.dry_run:
        workflow_root.mkdir(parents=True, exist_ok=True)
        (workflow_root / 'manifest.json').write_text(
            json.dumps(manifest, indent=2) + '\n'
        )
        (DATA / 'workflows' / 'latest.json').write_text(
            json.dumps(manifest, indent=2) + '\n'
        )
        response = api('/api/models', {'models': manifest['models']})
        print('dashboard models reloaded:', json.dumps(response['models'], indent=2))
        seed_demo_projection()
        if not options.no_open:
            webbrowser.open(DASHBOARD)
    print(f'\nCalibration workflow complete: {workflow_root}', flush=True)


def seed_demo_projection() -> None:
    """Populate the dashboard with tested, moderate demo curvatures."""
    state = api('/api/state')
    requested = {'arm_1': 0.980, 'arm_2': 0.921}
    for arm, curvature in requested.items():
        model = state['arms'][arm]['model']
        bounds = sorted((abs(model['curvature_min']), abs(model['curvature_max'])))
        curvature = min(max(curvature, bounds[0]), bounds[1])
        api('/api/curvature', {'arm': arm, 'curvature': curvature})


def demo(options: argparse.Namespace) -> None:
    required = {
        '/bluno_motor_bridge', '/encoder_velocity_controller',
        '/aruco_shape_tracker', '/rosaia_dashboard',
    }
    active = required & nodes()
    if active != required:
        if active:
            missing = ', '.join(sorted(required - active))
            raise RuntimeError(
                f'partial stack detected (missing {missing}); run '
                '"scripts/rosaia.sh workflow stop" and retry the demo'
            )
        start(argparse.Namespace(enable_motion=True))
    preflight()
    if not api('/api/state').get('motor_execute_enabled'):
        raise RuntimeError(
            'dashboard motion is disabled; restart with workflow demo'
        )
    latest = DATA / 'workflows' / 'latest.json'
    if latest.exists():
        manifest = json.loads(latest.read_text())
        api('/api/models', {'models': manifest['models']})
        print(f"loaded calibration {manifest['label']}")
    seed_demo_projection()
    if not options.no_open:
        webbrowser.open(DASHBOARD)
    print(f'dashboard ready: {DASHBOARD}')
    print('Inference is populated; motor execution remains an explicit UI action.')


def start(options: argparse.Namespace) -> None:
    """Start missing ROS components and retain their process groups."""
    existing = nodes()
    required = {
        '/bluno_motor_bridge', '/encoder_velocity_controller',
        '/aruco_shape_tracker', '/rosaia_dashboard',
    }
    without_velocity = getattr(options, 'without_velocity', False)
    if without_velocity:
        required.remove('/encoder_velocity_controller')
        if '/encoder_velocity_controller' in existing:
            raise RuntimeError('stop the existing stack before starting the bounded-step demo')
    if existing & required:
        if required <= existing:
            state = preflight(require_complete_vision=False)
            if bool(state.get('motor_execute_enabled')) != options.enable_motion:
                raise RuntimeError('stack already running with a different motion setting; stop it first')
            print(f'stack already running: {DASHBOARD}')
            return
        raise RuntimeError('partial stack already running; stop it before restarting')
    log_root = DATA / 'logs' / datetime.now().strftime('%Y%m%d_%H%M%S')
    log_root.mkdir(parents=True, exist_ok=True)
    registry = []
    specifications = [
        ('/bluno_motor_bridge', [str(ROSAIA), 'bridge'], 'bridge.log'),
        (
            '/encoder_velocity_controller',
            [str(ROSAIA), 'velocity-controller'],
            'velocity_controller.log',
        ),
        ('/aruco_shape_tracker', [str(ROSAIA), 'vision'], 'vision.log'),
    ]
    if without_velocity:
        specifications = [item for item in specifications if item[0] != '/encoder_velocity_controller']
    latest = DATA / 'workflows' / 'latest.json'
    models = None
    if latest.exists():
        models = json.loads(latest.read_text())['models']
    else:
        previous = {
            'arm_1': str((DATA / 'models' / 'arm1_shape_5v6_v1').resolve()),
            'arm_2': str((DATA / 'models' / 'arm2_shape_5v6_v1').resolve()),
        }
        if all(Path(directory).is_dir() for directory in previous.values()):
            models = previous
    dashboard = [str(ROSAIA), 'dashboard']
    if models:
        dashboard += [
            f"arm_1_model:={models['arm_1']}",
            f"arm_2_model:={models['arm_2']}",
        ]
    dashboard += [
        f"enable_motor_execute:={'true' if options.enable_motion else 'false'}"
    ]
    specifications.append(('/rosaia_dashboard', dashboard, 'dashboard.log'))
    processes = []
    try:
        for node, command, filename in specifications:
            with (log_root / filename).open('w') as output:
                process = subprocess.Popen(
                    command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            processes.append(process)
            registry.append({'node': node, 'pid': process.pid, 'command': command})
            RUNTIME.write_text(json.dumps({'processes': registry}, indent=2) + '\n')
        deadline = time.monotonic() + 15.0
        last_error = 'nodes not available'
        while time.monotonic() < deadline:
            for process, (node, _, filename) in zip(processes, specifications):
                if process.poll() is not None:
                    raise RuntimeError(f'{node} exited; inspect {log_root / filename}')
            if required <= nodes():
                try:
                    preflight(require_complete_vision=False)
                except RuntimeError as error:
                    last_error = str(error)
                else:
                    print(f'stack ready; logs: {log_root}')
                    return
            time.sleep(0.5)
        raise RuntimeError(
            f'stack startup timeout ({last_error}); inspect {log_root}'
        )
    except BaseException:
        for process in reversed(processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=5.0)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.0)
        RUNTIME.unlink(missing_ok=True)
        raise


def stop(_options: argparse.Namespace) -> None:
    run([str(ROSAIA), 'stop'])
    if not RUNTIME.exists():
        print('no workflow-owned process registry found')
        return
    registry = json.loads(RUNTIME.read_text())
    for item in reversed(registry.get('processes', [])):
        try:
            os.killpg(int(item['pid']), signal.SIGINT)
        except ProcessLookupError:
            pass
    RUNTIME.unlink(missing_ok=True)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    start_parser = commands.add_parser('start', help='start the complete stack')
    start_parser.add_argument('--enable-motion', action='store_true')
    start_parser.add_argument('--without-velocity', action='store_true',
                              help='camera/bridge/dashboard only for the bounded-step final demo')
    start_parser.set_defaults(function=start)
    calibration_parser = commands.add_parser(
        'calibrate', help='zero, acquire, audit, train, and load both MLPs'
    )
    calibration_parser.add_argument('--label')
    calibration_parser.add_argument(
        '--quick', action='store_true',
        help='pilot calibration over 0..40 counts; separate train/validation sweeps',
    )
    calibration_parser.add_argument(
        '--train-points', default='0,20,40,60,80,100,120,140,160,180,200'
    )
    calibration_parser.add_argument(
        '--validation-points', default='0,25,50,75,100,125,150,175,200'
    )
    calibration_parser.add_argument('--dwell', type=float, default=0.6)
    calibration_parser.add_argument('--yes', action='store_true')
    calibration_parser.add_argument('--dry-run', action='store_true')
    calibration_parser.add_argument('--no-open', action='store_true')
    calibration_parser.set_defaults(function=calibration)
    demo_parser = commands.add_parser('demo', help='load models and open UI')
    demo_parser.add_argument('--no-open', action='store_true')
    demo_parser.set_defaults(function=demo)
    stop_parser = commands.add_parser('stop', help='stop workflow-owned stack')
    stop_parser.set_defaults(function=stop)
    return parser.parse_args()


def main() -> None:
    options = arguments()
    try:
        options.function(options)
    except (RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f'workflow failed: {error}', file=sys.stderr)
        raise SystemExit(2) from error


if __name__ == '__main__':
    main()
