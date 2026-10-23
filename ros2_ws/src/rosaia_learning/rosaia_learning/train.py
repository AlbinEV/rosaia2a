"""Command-line MLP training with trajectory-level validation sessions."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import warnings

import numpy as np
from sklearn.exceptions import ConvergenceWarning
from sklearn.neural_network import MLPRegressor

from .dataset import (
    arrays,
    balance_q_bins,
    load_sessions,
    select_motion_direction,
)
from .model import NumpyMlpModel


def _positive_widths(value: str) -> tuple[int, ...]:
    widths = tuple(int(item) for item in value.split(',') if item.strip())
    if not widths or any(item <= 0 for item in widths):
        raise argparse.ArgumentTypeError('hidden widths must be positive')
    return widths


def _safe_scale(values: np.ndarray) -> np.ndarray:
    scale = np.std(values, axis=0)
    return np.where(scale < 1e-12, 1.0, scale)


def train_model(
    train_sessions: list[str],
    validation_sessions: list[str],
    arm_name: str,
    hidden_widths: tuple[int, ...],
    random_seed: int,
    maximum_iterations: int,
    camera_to_motor_offset_ms: float = 0.0,
    direction: str = 'all',
    command_to_q_sign: int = -1,
    q_bin_width: float = 0.0,
    maximum_samples_per_session_bin: int = 0,
    encoder_zero_raw: int | None = None,
    minimum_q: float | None = None,
    maximum_q: float | None = None,
) -> tuple[NumpyMlpModel, dict]:
    """Fit an MLP and evaluate it only on held-out complete sessions."""
    train_paths = {str(Path(value).resolve()) for value in train_sessions}
    validation_paths = {
        str(Path(value).resolve()) for value in validation_sessions
    }
    overlap = sorted(train_paths & validation_paths)
    if overlap:
        raise ValueError(
            f'train and validation sessions overlap: {overlap}'
        )
    train_data = load_sessions(
        train_sessions,
        arm_name,
        camera_to_motor_offset_ms,
        encoder_zero_raw,
    )
    validation_data = load_sessions(
        validation_sessions,
        arm_name,
        camera_to_motor_offset_ms,
        encoder_zero_raw,
    )
    train_data = select_motion_direction(
        train_data, direction, command_to_q_sign
    )
    validation_data = select_motion_direction(
        validation_data, direction, command_to_q_sign
    )
    if (
        minimum_q is not None
        and maximum_q is not None
        and minimum_q > maximum_q
    ):
        raise ValueError('minimum q cannot exceed maximum q')
    train_rows_before_range_filter = len(train_data)
    validation_rows_before_range_filter = len(validation_data)
    if minimum_q is not None:
        train_data = train_data.loc[
            train_data['q_operational'] >= minimum_q
        ].copy()
        validation_data = validation_data.loc[
            validation_data['q_operational'] >= minimum_q
        ].copy()
    if maximum_q is not None:
        train_data = train_data.loc[
            train_data['q_operational'] <= maximum_q
        ].copy()
        validation_data = validation_data.loc[
            validation_data['q_operational'] <= maximum_q
        ].copy()
    if len(train_data) < 20 or len(validation_data) < 20:
        raise ValueError('q-range filter leaves fewer than 20 rows')
    unbalanced_train_rows = len(train_data)
    if q_bin_width > 0.0 or maximum_samples_per_session_bin > 0:
        if q_bin_width <= 0.0 or maximum_samples_per_session_bin <= 0:
            raise ValueError(
                'q bin width and maximum samples must both be enabled'
            )
        train_data = balance_q_bins(
            train_data, q_bin_width, maximum_samples_per_session_bin
        )
    q_train, x_train = arrays(train_data)
    q_validation, x_validation = arrays(validation_data)

    q_mean = float(np.mean(q_train))
    q_scale = float(_safe_scale(q_train)[0])
    x_mean = np.mean(x_train, axis=0)
    x_scale = _safe_scale(x_train)
    q_scaled = (q_train - q_mean) / q_scale
    x_scaled = (x_train - x_mean) / x_scale

    estimator = MLPRegressor(
        hidden_layer_sizes=hidden_widths,
        activation='tanh',
        solver='adam',
        alpha=1e-5,
        batch_size=min(256, len(q_train)),
        learning_rate_init=1e-3,
        max_iter=maximum_iterations,
        shuffle=True,
        random_state=random_seed,
        early_stopping=False,
        tol=1e-7,
        n_iter_no_change=100,
    )
    with warnings.catch_warnings(record=True) as fit_warnings:
        warnings.simplefilter('always', ConvergenceWarning)
        estimator.fit(q_scaled, x_scaled)
    converged = not any(
        issubclass(item.category, ConvergenceWarning)
        for item in fit_warnings
    )
    model = NumpyMlpModel(
        q_mean=q_mean,
        q_scale=q_scale,
        x_mean=x_mean,
        x_scale=x_scale,
        weights=tuple(estimator.coefs_),
        biases=tuple(estimator.intercepts_),
        q_min=float(np.min(q_train)),
        q_max=float(np.max(q_train)),
    )
    predictions = np.vstack([
        model.predict_and_jacobian(value)[0]
        for value in q_validation[:, 0]
    ])
    errors = predictions - x_validation
    absolute_errors = np.abs(errors)
    rmse_per_component = np.sqrt(np.mean(errors * errors, axis=0))
    metrics = {
        'arm': arm_name,
        'created_at': datetime.now().astimezone().isoformat(),
        'train_sessions': [str(Path(value)) for value in train_sessions],
        'validation_sessions': [
            str(Path(value)) for value in validation_sessions
        ],
        'train_rows': int(len(train_data)),
        'unbalanced_train_rows': int(unbalanced_train_rows),
        'validation_rows': int(len(validation_data)),
        'direction': direction,
        'command_to_q_sign': command_to_q_sign,
        'q_bin_width': q_bin_width,
        'maximum_samples_per_session_bin': (
            maximum_samples_per_session_bin
        ),
        'encoder_zero_raw': encoder_zero_raw,
        'minimum_q': minimum_q,
        'maximum_q': maximum_q,
        'train_rows_before_range_filter': train_rows_before_range_filter,
        'validation_rows_before_range_filter': (
            validation_rows_before_range_filter
        ),
        'hidden_widths': list(hidden_widths),
        'random_seed': random_seed,
        'camera_to_motor_offset_ms': camera_to_motor_offset_ms,
        'iterations': int(estimator.n_iter_),
        'converged': converged,
        'loss': float(estimator.loss_),
        'validation_rmse_components': rmse_per_component.tolist(),
        'validation_rmse_total': float(np.sqrt(np.mean(errors * errors))),
        'validation_p95_components': np.percentile(
            absolute_errors, 95, axis=0
        ).tolist(),
        'validation_p95_total': float(np.percentile(absolute_errors, 95)),
        'validation_max_components': np.max(
            absolute_errors, axis=0
        ).tolist(),
        'validation_max_total': float(np.max(absolute_errors)),
        'train_q_range': [float(np.min(q_train)), float(np.max(q_train))],
        'validation_q_range': [
            float(np.min(q_validation)), float(np.max(q_validation))
        ],
        'validation_extrapolation_fraction': float(np.mean(
            (q_validation[:, 0] < np.min(q_train))
            | (q_validation[:, 0] > np.max(q_train))
        )),
    }
    return model, metrics


def main(args=None) -> None:
    """Train and export one per-arm scalar-input MLP."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train-session', action='append', required=True)
    parser.add_argument('--validation-session', action='append', required=True)
    parser.add_argument('--arm', choices=('arm_1', 'arm_2'), required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--hidden', type=_positive_widths, default=(32, 32))
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--max-iterations', type=int, default=2000)
    parser.add_argument('--camera-to-motor-offset-ms', type=float, default=0.0)
    parser.add_argument(
        '--direction', choices=('all', 'increasing', 'decreasing'),
        default='all',
    )
    parser.add_argument(
        '--command-to-q-sign', type=int, choices=(-1, 1), default=-1
    )
    parser.add_argument('--q-bin-width', type=float, default=0.0)
    parser.add_argument(
        '--maximum-samples-per-session-bin', type=int, default=0
    )
    parser.add_argument(
        '--encoder-zero-raw', type=int,
        help='common physical-home raw count; overrides per-session zero',
    )
    parser.add_argument('--minimum-q', type=float)
    parser.add_argument('--maximum-q', type=float)
    options = parser.parse_args(args)
    model, metrics = train_model(
        options.train_session,
        options.validation_session,
        options.arm,
        options.hidden,
        options.seed,
        options.max_iterations,
        options.camera_to_motor_offset_ms,
        options.direction,
        options.command_to_q_sign,
        options.q_bin_width,
        options.maximum_samples_per_session_bin,
        options.encoder_zero_raw,
        options.minimum_q,
        options.maximum_q,
    )
    model.save(options.output, metadata={'training': metrics})
    print(json.dumps(metrics, indent=2))
