"""Load and validate recorder sessions for static kinematic learning."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd


STATE_COLUMNS = ['x1', 'y1', 'x2', 'y2', 'x3', 'y3']
REQUIRED_COLUMNS = [
    'camera_time_ns', 'arm', 'axis', 'q_operational', 'command', 'adc_raw',
    'current_ma', 'firmware_flags', *STATE_COLUMNS,
]
REJECTED_FAULT_MASK = 2 | 8 | 32 | 64


def load_sessions(
    session_paths: list[str | Path],
    arm_name: str,
    camera_to_motor_offset_ms: float = 0.0,
    encoder_zero_raw: int | None = None,
) -> pd.DataFrame:
    """Load rows and interpolate motor signals at corrected camera times."""
    frames = []
    for session_value in session_paths:
        session = Path(session_value)
        source = session / 'samples.csv'
        if not source.is_file():
            raise FileNotFoundError(f'missing session samples: {source}')
        frame = pd.read_csv(source)
        missing = sorted(set(REQUIRED_COLUMNS) - set(frame.columns))
        if missing:
            raise ValueError(f'{source}: missing columns {missing}')
        selected = list(REQUIRED_COLUMNS)
        if 'current_a' in frame.columns:
            selected.append('current_a')
        frame = frame.loc[frame['arm'] == arm_name, selected].copy()
        if 'current_a' not in frame.columns:
            frame['current_a'] = np.nan
        if frame.empty:
            continue
        frame = _align_motor_stream(
            session,
            frame,
            arm_name,
            camera_to_motor_offset_ms,
            encoder_zero_raw,
        )
        frame['session'] = session.name
        frames.append(frame)
    if not frames:
        raise ValueError('at least one session path is required')
    data = pd.concat(frames, ignore_index=True)
    numeric = ['q_operational', 'firmware_flags', *STATE_COLUMNS]
    finite = np.isfinite(data[numeric].to_numpy(dtype=np.float64)).all(axis=1)
    faults = data['firmware_flags'].astype(np.int64).to_numpy()
    accepted = finite & ((faults & REJECTED_FAULT_MASK) == 0)
    data = data.loc[accepted].sort_values(
        ['session', 'camera_time_ns'], kind='stable'
    )
    if len(data) < 20:
        raise ValueError(
            f'only {len(data)} accepted rows for {arm_name}; need at least 20'
        )
    return data.reset_index(drop=True)


def _align_motor_stream(
    session: Path,
    frame: pd.DataFrame,
    arm_name: str,
    camera_to_motor_offset_ms: float,
    encoder_zero_raw_override: int | None,
) -> pd.DataFrame:
    """Interpolate encoder/current data onto corrected camera timestamps."""
    metadata_path = session / 'metadata.json'
    telemetry_path = session / 'telemetry.csv'
    if not metadata_path.is_file() or not telemetry_path.is_file():
        raise FileNotFoundError(
            f'{session}: metadata.json and telemetry.csv are required'
        )
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    arm_configs = {
        value['name']: value for value in metadata.get('arms', [])
    }
    if arm_name not in arm_configs:
        raise ValueError(f'{session}: missing metadata for {arm_name}')
    axis = int(arm_configs[arm_name]['axis'])
    sign = int(arm_configs[arm_name]['encoder_sign'])
    zero_value = arm_configs[arm_name].get('encoder_zero_raw', 0)
    if zero_value is None:
        raise ValueError(f'{session}: encoder zero was not captured')
    encoder_zero = (
        int(zero_value)
        if encoder_zero_raw_override is None
        else int(encoder_zero_raw_override)
    )
    if axis not in (0, 1) or sign not in (-1, 1):
        raise ValueError(f'{session}: invalid axis/sign for {arm_name}')
    if set(frame['axis'].astype(int).unique()) != {axis}:
        raise ValueError(f'{session}: sample axis disagrees with metadata')

    telemetry = pd.read_csv(telemetry_path)
    columns = [
        'receive_time_ns', f'encoder_{axis}_raw', f'command_{axis}',
        f'adc_{axis}_raw', f'current_{axis}_ma',
    ]
    host_current_column = f'current_{axis}_a'
    if host_current_column in telemetry.columns:
        columns.append(host_current_column)
    missing = sorted(set(columns) - set(telemetry.columns))
    if missing:
        raise ValueError(f'{telemetry_path}: missing columns {missing}')
    telemetry = telemetry[columns].dropna().sort_values(
        'receive_time_ns', kind='stable'
    )
    telemetry = telemetry.drop_duplicates('receive_time_ns', keep='last')
    if len(telemetry) < 2:
        raise ValueError(f'{telemetry_path}: need at least two telemetry rows')

    motor_times = telemetry['receive_time_ns'].to_numpy(dtype=np.int64)
    camera_times = frame['camera_time_ns'].to_numpy(dtype=np.int64)
    query_times = camera_times + round(camera_to_motor_offset_ms * 1_000_000.0)
    in_range = (query_times >= motor_times[0]) & (query_times <= motor_times[-1])
    frame = frame.loc[in_range].copy()
    query_times = query_times[in_range]
    if frame.empty:
        raise ValueError(f'{session}: no camera timestamps overlap telemetry')

    def interpolate(column: str) -> np.ndarray:
        return np.interp(
            query_times,
            motor_times,
            telemetry[column].to_numpy(dtype=np.float64),
        )

    frame['q_operational'] = sign * (
        interpolate(f'encoder_{axis}_raw') - encoder_zero
    )
    frame['command'] = interpolate(f'command_{axis}')
    frame['adc_raw'] = interpolate(f'adc_{axis}_raw')
    frame['current_ma'] = interpolate(f'current_{axis}_ma')
    if host_current_column in telemetry.columns:
        frame['current_a'] = interpolate(host_current_column)
    return frame


def arrays(data: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Extract scalar position input and ordered six-state output arrays."""
    q_values = data['q_operational'].to_numpy(dtype=np.float64).reshape(-1, 1)
    states = data[STATE_COLUMNS].to_numpy(dtype=np.float64)
    return q_values, states


def add_motion_direction(
    data: pd.DataFrame, command_to_q_sign: int = -1
) -> pd.DataFrame:
    """Label motion branches, retaining the last direction during dwell."""
    if command_to_q_sign not in (-1, 1):
        raise ValueError('command_to_q_sign must be -1 or 1')
    labelled = data.copy()
    labelled['motion_direction'] = 0
    for _, indices in labelled.groupby('session', sort=False).groups.items():
        command = labelled.loc[indices, 'command'].to_numpy(dtype=np.float64)
        direction = np.sign(command * command_to_q_sign).astype(np.int8)
        last = 0
        for index, value in enumerate(direction):
            if value:
                last = int(value)
            direction[index] = last
        labelled.loc[indices, 'motion_direction'] = direction
    return labelled


def select_motion_direction(
    data: pd.DataFrame, direction: str, command_to_q_sign: int = -1
) -> pd.DataFrame:
    """Select increasing or decreasing-q branches from complete sessions."""
    if direction == 'all':
        return data.copy()
    signs = {'increasing': 1, 'decreasing': -1}
    if direction not in signs:
        raise ValueError(f'unsupported motion direction: {direction}')
    labelled = add_motion_direction(data, command_to_q_sign)
    selected = labelled.loc[
        labelled['motion_direction'] == signs[direction]
    ].copy()
    if len(selected) < 20:
        raise ValueError(
            f'only {len(selected)} rows remain for direction {direction}'
        )
    return selected.reset_index(drop=True)


def balance_q_bins(
    data: pd.DataFrame, bin_width: float, maximum_per_session_bin: int
) -> pd.DataFrame:
    """Cap dense dwell regions while retaining every session and q bin."""
    if bin_width <= 0.0 or maximum_per_session_bin <= 0:
        raise ValueError('bin width and maximum samples must be positive')
    balanced = data.copy()
    balanced['_q_bin'] = np.floor(
        balanced['q_operational'].to_numpy(dtype=np.float64) / bin_width
    ).astype(np.int64)
    kept = []
    for _, group in balanced.groupby(['session', '_q_bin'], sort=False):
        if len(group) <= maximum_per_session_bin:
            kept.append(group)
            continue
        positions = np.linspace(
            0, len(group) - 1, maximum_per_session_bin, dtype=np.int64
        )
        kept.append(group.iloc[positions])
    result = pd.concat(kept, ignore_index=True)
    return result.drop(columns=['_q_bin'])
