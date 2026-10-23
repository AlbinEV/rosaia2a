"""Tests for recorder-session quality auditing."""

import json

import numpy as np
import pandas as pd

from rosaia_learning.audit import audit_session


def _write_session(path, q_values, schema_version=5):
    """Create a minimal schema-v5 session for audit tests."""
    path.mkdir()
    q_values = np.asarray(q_values, dtype=np.float64)
    count = len(q_values)
    times = np.arange(count, dtype=np.int64) * 20_000_000
    samples = {
        'camera_time_ns': times,
        'arm': ['arm_1'] * count,
        'q_operational': q_values,
        'telemetry_age_ms': np.full(count, 10.0),
        'command': np.zeros(count),
        'current_a': np.zeros(count),
        'firmware_flags': np.full(count, 16),
    }
    for index, name in enumerate(
        ('x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'x3', 'y3', 'z3')
    ):
        samples[name] = q_values * (index + 1) / 1000.0
    pd.DataFrame(samples).to_csv(path / 'samples.csv', index=False)
    pd.DataFrame({'receive_time_ns': times}).to_csv(
        path / 'telemetry.csv', index=False
    )
    metadata = {
        'schema_version': schema_version,
        'arms': [{'name': 'arm_1'}],
    }
    (path / 'metadata.json').write_text(
        json.dumps(metadata), encoding='utf-8'
    )


def test_bidirectional_session_passes(tmp_path):
    """A finite, sufficiently wide bidirectional trajectory is accepted."""
    path = tmp_path / 'valid'
    trajectory = np.concatenate((np.linspace(0, 50, 60), np.linspace(50, 0, 60)))
    _write_session(path, trajectory)
    report = audit_session(path, minimum_rows=100, minimum_q_span=30.0)
    assert report['ready_for_training']
    assert report['arms']['arm_1']['positive_q_steps'] > 0
    assert report['arms']['arm_1']['negative_q_steps'] > 0


def test_static_session_is_reported_but_rejected(tmp_path):
    """A static baseline remains inspectable but is not a training sweep."""
    path = tmp_path / 'static'
    _write_session(path, np.zeros(120))
    report = audit_session(path, minimum_rows=100, minimum_q_span=30.0)
    arm = report['arms']['arm_1']
    assert not report['ready_for_training']
    assert arm['q_span'] == 0.0
    assert 'both q directions were not observed' in arm['errors']


def test_missing_selected_arm_is_rejected(tmp_path):
    """Selecting an arm absent from metadata produces an explicit error."""
    path = tmp_path / 'arm_1_only'
    trajectory = np.concatenate((np.linspace(0, 50, 60), np.linspace(50, 0, 60)))
    _write_session(path, trajectory)
    report = audit_session(path, selected_arms={'arm_2'})
    assert not report['ready_for_training']
    assert report['errors'] == ['arm not configured: arm_2']
