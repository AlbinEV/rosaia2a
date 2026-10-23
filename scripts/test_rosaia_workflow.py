"""Regression tests for stack ownership, startup cleanup and vision gates."""

import argparse
import importlib.util
from pathlib import Path
from unittest.mock import Mock, patch

import pytest


spec = importlib.util.spec_from_file_location(
    'workflow', Path(__file__).with_name('rosaia_workflow.py')
)
workflow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workflow)


def test_diagnostic_start_allows_missing_markers():
    state = {'motor': {'encoder_counts': [0, 0], 'applied_commands': [0, 0]},
             'vision': {}}
    with patch.object(workflow, 'api', return_value=state):
        assert workflow.preflight(require_complete_vision=False) == state
        with pytest.raises(RuntimeError, match='complete visual'):
            workflow.preflight()


def test_partial_stack_is_not_adopted():
    with patch.object(workflow, 'nodes', return_value={'/bluno_motor_bridge'}):
        with pytest.raises(RuntimeError, match='partial stack'):
            workflow.start(argparse.Namespace(enable_motion=False))


def test_restart_keeps_process_registry(tmp_path):
    registry = tmp_path / 'runtime.json'
    registry.write_text('{"processes": [{"pid": 123}]}')
    required = {'/bluno_motor_bridge', '/encoder_velocity_controller',
                '/aruco_shape_tracker', '/rosaia_dashboard'}
    with patch.object(workflow, 'nodes', return_value=required), \
            patch.object(workflow, 'preflight', return_value={'motor_execute_enabled': False}), \
            patch.object(workflow, 'RUNTIME', registry):
        workflow.start(argparse.Namespace(enable_motion=False))
    assert '123' in registry.read_text()


def test_launch_failure_cleans_up_started_processes(tmp_path):
    process = Mock(pid=123)
    process.poll.return_value = None
    with patch.object(workflow, 'nodes', return_value=set()), \
            patch.object(workflow, 'DATA', tmp_path), \
            patch.object(workflow, 'RUNTIME', tmp_path / 'runtime.json'), \
            patch.object(workflow.subprocess, 'Popen', side_effect=[process, OSError('launch failed')]), \
            patch.object(workflow.os, 'killpg') as kill:
        with pytest.raises(OSError, match='launch failed'):
            workflow.start(argparse.Namespace(enable_motion=False))
        kill.assert_called_once_with(123, workflow.signal.SIGINT)
        process.wait.assert_called_once()
    assert not (tmp_path / 'runtime.json').exists()
