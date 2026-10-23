"""Audit recorded sessions before they are admitted to MLP training."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


EXPECTED_FIRMWARE_FLAG_MASK = 16
STATE_3D_COLUMNS = [
    'x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'x3', 'y3', 'z3',
]


def _rate_hz(times_ns: np.ndarray) -> float | None:
    """Return the mean sample rate over a timestamp span."""
    if len(times_ns) < 2:
        return None
    duration_s = (float(np.max(times_ns)) - float(np.min(times_ns))) / 1e9
    if duration_s <= 0.0:
        return None
    return float((len(times_ns) - 1) / duration_s)


def _number(value: float | None) -> float | None:
    """Convert NumPy values to finite JSON numbers."""
    if value is None or not np.isfinite(value):
        return None
    return float(value)


def audit_session(
    session_value: str | Path,
    minimum_rows: int = 100,
    minimum_q_span: float = 30.0,
    selected_arms: set[str] | None = None,
) -> dict:
    """Return quality metrics and training-readiness checks for one session."""
    session = Path(session_value).resolve()
    metadata_path = session / 'metadata.json'
    samples_path = session / 'samples.csv'
    telemetry_path = session / 'telemetry.csv'
    missing_files = [
        path.name for path in (metadata_path, samples_path, telemetry_path)
        if not path.is_file()
    ]
    if missing_files:
        return {
            'session': str(session),
            'ready_for_training': False,
            'errors': [f'missing file: {name}' for name in missing_files],
            'arms': {},
        }

    try:
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        samples = pd.read_csv(samples_path)
        telemetry = pd.read_csv(telemetry_path)
    except (json.JSONDecodeError, OSError, pd.errors.ParserError) as error:
        return {
            'session': str(session),
            'ready_for_training': False,
            'errors': [str(error)],
            'arms': {},
        }

    errors = []
    schema_version = int(metadata.get('schema_version', 0))
    if schema_version < 5:
        errors.append(f'schema_version {schema_version}; version 5 required')
    required_sample_columns = {
        'camera_time_ns', 'arm', 'q_operational', 'telemetry_age_ms',
        'command', 'current_a', 'firmware_flags', *STATE_3D_COLUMNS,
    }
    missing_columns = sorted(required_sample_columns - set(samples.columns))
    if missing_columns:
        errors.append(f'samples.csv missing columns: {missing_columns}')
    if 'receive_time_ns' not in telemetry.columns:
        errors.append('telemetry.csv missing column: receive_time_ns')
    if errors:
        return {
            'session': str(session),
            'schema_version': schema_version,
            'ready_for_training': False,
            'errors': errors,
            'arms': {},
        }

    telemetry_times = telemetry['receive_time_ns'].to_numpy(dtype=np.int64)
    arms = {}
    configured_arms = [
        str(value.get('name')) for value in metadata.get('arms', [])
        if value.get('name')
    ]
    arm_names = configured_arms or sorted(samples['arm'].dropna().unique())
    if selected_arms is not None:
        arm_names = [name for name in arm_names if name in selected_arms]
        absent = sorted(selected_arms - set(arm_names))
        errors.extend(f'arm not configured: {name}' for name in absent)
    for arm_name in arm_names:
        frame = samples.loc[samples['arm'] == arm_name].copy()
        arm_errors = []
        if frame.empty:
            arm_errors.append('no visual samples')
            arms[arm_name] = {
                'ready_for_training': False,
                'errors': arm_errors,
                'rows': 0,
            }
            continue

        numeric_columns = [
            'camera_time_ns', 'q_operational', 'telemetry_age_ms', 'command',
            'current_a', 'firmware_flags', *STATE_3D_COLUMNS,
        ]
        numeric = frame[numeric_columns].apply(pd.to_numeric, errors='coerce')
        finite_rows = np.isfinite(numeric.to_numpy(dtype=np.float64)).all(axis=1)
        finite_fraction = float(np.mean(finite_rows))
        valid = numeric.loc[finite_rows].copy()
        if len(frame) < minimum_rows:
            arm_errors.append(f'only {len(frame)} rows; need {minimum_rows}')
        if finite_fraction < 1.0:
            arm_errors.append(
                f'non-finite rows present ({finite_fraction:.3f} finite)'
            )
        if valid.empty:
            arm_errors.append('no finite rows')
            arms[arm_name] = {
                'ready_for_training': False,
                'errors': arm_errors,
                'rows': int(len(frame)),
                'finite_fraction': finite_fraction,
            }
            continue

        camera_times = valid['camera_time_ns'].to_numpy(dtype=np.int64)
        q_values = valid['q_operational'].to_numpy(dtype=np.float64)
        q_span = float(np.max(q_values) - np.min(q_values))
        if q_span < minimum_q_span:
            arm_errors.append(
                f'q span {q_span:.3f}; need at least {minimum_q_span:.3f}'
            )
        time_order = np.argsort(camera_times, kind='stable')
        delta_q = np.diff(q_values[time_order])
        positive_steps = int(np.count_nonzero(delta_q > 0.25))
        negative_steps = int(np.count_nonzero(delta_q < -0.25))
        if positive_steps == 0 or negative_steps == 0:
            arm_errors.append('both q directions were not observed')
        flags = valid['firmware_flags'].to_numpy(dtype=np.int64)
        unexpected_flags = sorted({
            int(value & ~EXPECTED_FIRMWARE_FLAG_MASK) for value in flags
            if value & ~EXPECTED_FIRMWARE_FLAG_MASK
        })
        if unexpected_flags:
            arm_errors.append(f'unexpected firmware flags: {unexpected_flags}')

        current = valid['current_a'].to_numpy(dtype=np.float64)
        age = valid['telemetry_age_ms'].to_numpy(dtype=np.float64)
        arms[arm_name] = {
            'ready_for_training': not arm_errors,
            'errors': arm_errors,
            'rows': int(len(frame)),
            'finite_fraction': finite_fraction,
            'visual_rate_hz': _number(_rate_hz(camera_times)),
            'q_min': float(np.min(q_values)),
            'q_max': float(np.max(q_values)),
            'q_span': q_span,
            'positive_q_steps': positive_steps,
            'negative_q_steps': negative_steps,
            'telemetry_age_ms_p95': float(np.percentile(age, 95)),
            'telemetry_age_ms_max': float(np.max(age)),
            'current_a_min': float(np.min(current)),
            'current_a_max': float(np.max(current)),
            'unexpected_firmware_flags': unexpected_flags,
        }

    return {
        'session': str(session),
        'schema_version': schema_version,
        'ready_for_training': bool(arms) and not errors and all(
            value['ready_for_training'] for value in arms.values()
        ),
        'errors': errors,
        'telemetry_rows': int(len(telemetry)),
        'telemetry_rate_hz': _number(_rate_hz(telemetry_times)),
        'arms': arms,
    }


def main(args=None) -> None:
    """Audit sessions and optionally persist a machine-readable report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', action='append', required=True)
    parser.add_argument('--minimum-rows', type=int, default=100)
    parser.add_argument('--minimum-q-span', type=float, default=30.0)
    parser.add_argument(
        '--arm', action='append', choices=('arm_1', 'arm_2'),
        help='audit only this arm; repeat for more than one',
    )
    parser.add_argument('--output')
    options = parser.parse_args(args)
    if options.minimum_rows <= 0 or options.minimum_q_span < 0.0:
        parser.error('minimum rows must be positive and q span non-negative')
    reports = [
        audit_session(
            path,
            options.minimum_rows,
            options.minimum_q_span,
            set(options.arm) if options.arm else None,
        )
        for path in options.session
    ]
    payload = {
        'all_ready_for_training': all(
            report['ready_for_training'] for report in reports
        ),
        'sessions': reports,
    }
    rendered = json.dumps(payload, indent=2) + '\n'
    if options.output:
        output = Path(options.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding='utf-8')
    print(rendered, end='')


if __name__ == '__main__':
    main()
