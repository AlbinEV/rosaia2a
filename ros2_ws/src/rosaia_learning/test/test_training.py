"""Integration test for session loading, MLP fitting, and validation."""

import json

import numpy as np
import pandas as pd
import pytest

from rosaia_learning.dataset import (
    add_motion_direction,
    balance_q_bins,
    STATE_COLUMNS,
)
from rosaia_learning.train import train_model


def write_session(path, q_values):
    """Write one synthetic recorder-compatible session."""
    path.mkdir()
    q_values = np.asarray(q_values, dtype=np.float64)
    encoder_zero = 17
    times = np.arange(len(q_values), dtype=np.int64) * 20_000_000
    rows = {
        'camera_time_ns': times,
        'arm': ['arm_1'] * len(q_values),
        'axis': np.zeros(len(q_values), dtype=np.int64),
        'q_operational': q_values,
        'command': np.zeros(len(q_values)),
        'adc_raw': np.full(len(q_values), 512),
        'current_ma': np.full(len(q_values), -(2**31)),
        'current_a': np.full(len(q_values), 0.25),
        'firmware_flags': np.full(len(q_values), 16),
    }
    states = np.column_stack([
        0.1 * q_values,
        np.sin(q_values / 20.0),
        0.2 * q_values + 1.0,
        np.cos(q_values / 30.0),
        -0.05 * q_values,
        0.002 * q_values * q_values,
    ])
    for index, column in enumerate(STATE_COLUMNS):
        rows[column] = states[:, index]
    pd.DataFrame(rows).to_csv(path / 'samples.csv', index=False)
    telemetry = {
        'receive_time_ns': times,
        'encoder_0_raw': encoder_zero - q_values,
        'encoder_1_raw': np.zeros(len(q_values)),
        'command_0': np.zeros(len(q_values)),
        'command_1': np.zeros(len(q_values)),
        'adc_0_raw': np.full(len(q_values), 512),
        'adc_1_raw': np.full(len(q_values), 512),
        'current_0_ma': np.full(len(q_values), -(2**31)),
        'current_1_ma': np.full(len(q_values), -(2**31)),
        'current_0_a': np.full(len(q_values), 0.25),
        'current_1_a': np.zeros(len(q_values)),
        'firmware_flags': np.full(len(q_values), 16),
    }
    pd.DataFrame(telemetry).to_csv(path / 'telemetry.csv', index=False)
    metadata = {
        'schema_version': 5,
        'arms': [
            {
                'name': 'arm_1',
                'state_topic': '/arm_1/aruco_state',
                'axis': 0,
                'encoder_sign': -1,
                'encoder_zero_raw': encoder_zero,
            }
        ],
    }
    (path / 'metadata.json').write_text(
        json.dumps(metadata), encoding='utf-8'
    )


def test_train_model_uses_held_out_session(tmp_path):
    """Training returns finite held-out metrics and an analytic model."""
    training = tmp_path / 'training'
    validation = tmp_path / 'validation'
    write_session(training, np.linspace(0.0, 100.0, 120))
    write_session(validation, np.linspace(2.0, 98.0, 60))
    model, metrics = train_model(
        [str(training)],
        [str(validation)],
        'arm_1',
        (12, 12),
        3,
        2000,
    )
    state, jacobian = model.predict_and_jacobian(50.0)
    assert np.isfinite(state).all()
    assert np.isfinite(jacobian).all()
    assert metrics['train_rows'] == 120
    assert metrics['validation_rows'] == 60
    assert metrics['validation_rmse_total'] < 0.2
    assert metrics['validation_extrapolation_fraction'] == 0.0
    assert len(metrics['validation_p95_components']) == 6


def test_train_model_rejects_session_leakage(tmp_path):
    """A session cannot appear in both training and validation sets."""
    session = tmp_path / 'same_session'
    write_session(session, np.linspace(0.0, 100.0, 120))
    with pytest.raises(ValueError, match='overlap'):
        train_model(
            [str(session)], [str(session)], 'arm_1', (8,), 3, 10
        )


def test_direction_labels_persist_through_dwell():
    """Zero-command dwell inherits the most recent physical q direction."""
    frame = pd.DataFrame({
        'session': ['one'] * 6,
        'command': [0, -80, 0, 0, 70, 0],
    })
    labelled = add_motion_direction(frame, command_to_q_sign=-1)
    assert labelled['motion_direction'].tolist() == [0, 1, 1, 1, -1, -1]


def test_q_bin_balancing_caps_each_session_independently():
    """Dense dwell bins cannot dominate and both sessions are retained."""
    frame = pd.DataFrame({
        'session': ['one'] * 20 + ['two'] * 20,
        'q_operational': [1.0] * 40,
        'command': [0] * 40,
    })
    balanced = balance_q_bins(frame, 5.0, 6)
    assert len(balanced) == 12
    assert balanced.groupby('session').size().to_dict() == {
        'one': 6, 'two': 6,
    }


def test_training_can_override_session_encoder_zero(tmp_path):
    """A shared physical zero aligns sessions started at different counts."""
    training = tmp_path / 'training'
    validation = tmp_path / 'validation'
    write_session(training, np.linspace(0.0, 100.0, 120))
    write_session(validation, np.linspace(2.0, 98.0, 60))
    _, metrics = train_model(
        [str(training)], [str(validation)], 'arm_1', (8,), 3, 20,
        encoder_zero_raw=0,
    )
    assert metrics['encoder_zero_raw'] == 0
    assert metrics['train_q_range'][0] == pytest.approx(-17.0)
