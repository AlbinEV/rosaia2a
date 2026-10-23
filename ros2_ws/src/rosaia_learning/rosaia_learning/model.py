"""Portable NumPy inference and analytic Jacobian for a tanh MLP."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


@dataclass
class NumpyMlpModel:
    """Scalar-input, six-output MLP with an analytic input derivative."""

    q_mean: float
    q_scale: float
    x_mean: np.ndarray
    x_scale: np.ndarray
    weights: tuple[np.ndarray, ...]
    biases: tuple[np.ndarray, ...]
    q_min: float
    q_max: float

    def __post_init__(self) -> None:
        """Validate layer and normalization dimensions."""
        self.x_mean = np.asarray(self.x_mean, dtype=np.float64).reshape(-1)
        self.x_scale = np.asarray(self.x_scale, dtype=np.float64).reshape(-1)
        self.weights = tuple(
            np.asarray(value, dtype=np.float64) for value in self.weights
        )
        self.biases = tuple(
            np.asarray(value, dtype=np.float64).reshape(-1)
            for value in self.biases
        )
        if self.q_scale <= 0.0 or self.x_mean.shape != (6,):
            raise ValueError('invalid input or output normalization')
        if self.x_scale.shape != (6,) or np.any(self.x_scale <= 0.0):
            raise ValueError('output scales must contain six positive values')
        if not self.weights or len(self.weights) != len(self.biases):
            raise ValueError('MLP weights and biases are inconsistent')
        width = 1
        for weight, bias in zip(self.weights, self.biases):
            if weight.ndim != 2 or weight.shape[0] != width:
                raise ValueError('MLP layer input width is inconsistent')
            if weight.shape[1] != bias.shape[0]:
                raise ValueError('MLP layer output width is inconsistent')
            width = weight.shape[1]
        if width != 6:
            raise ValueError('final MLP layer must have six outputs')
        if self.q_min > self.q_max:
            raise ValueError('q_min must not exceed q_max')

    def predict_and_jacobian(
        self, q_value: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return the visual state and its six-by-one Jacobian at ``q``."""
        activation = np.array(
            [[(float(q_value) - self.q_mean) / self.q_scale]],
            dtype=np.float64,
        )
        derivative = np.array([[1.0 / self.q_scale]], dtype=np.float64)
        for index, (weight, bias) in enumerate(
            zip(self.weights, self.biases)
        ):
            pre_activation = activation @ weight + bias
            pre_derivative = derivative @ weight
            if index < len(self.weights) - 1:
                activation = np.tanh(pre_activation)
                derivative = (
                    1.0 - activation * activation
                ) * pre_derivative
            else:
                activation = pre_activation
                derivative = pre_derivative
        state = activation.reshape(6) * self.x_scale + self.x_mean
        jacobian = derivative.reshape(6) * self.x_scale
        return state, jacobian

    def save(self, directory: str | Path, metadata: dict | None = None) -> None:
        """Save numeric weights and human-readable metadata without pickle."""
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        arrays = {
            'q_mean': np.array(self.q_mean),
            'q_scale': np.array(self.q_scale),
            'x_mean': self.x_mean,
            'x_scale': self.x_scale,
            'q_min': np.array(self.q_min),
            'q_max': np.array(self.q_max),
        }
        for index, (weight, bias) in enumerate(
            zip(self.weights, self.biases)
        ):
            arrays[f'weight_{index}'] = weight
            arrays[f'bias_{index}'] = bias
        np.savez_compressed(path / 'model.npz', **arrays)
        payload = {
            'schema_version': 1,
            'model_type': 'scalar_input_tanh_mlp',
            'input': 'q_operational_counts',
            'output': ['x1', 'y1', 'x2', 'y2', 'x3', 'y3'],
            'layer_widths': [
                int(weight.shape[1]) for weight in self.weights
            ],
        }
        if metadata:
            payload.update(metadata)
        (path / 'metadata.json').write_text(
            json.dumps(payload, indent=2) + '\n', encoding='utf-8'
        )

    @classmethod
    def load(cls, directory: str | Path) -> 'NumpyMlpModel':
        """Load a model saved by :meth:`save`."""
        path = Path(directory)
        with np.load(path / 'model.npz', allow_pickle=False) as archive:
            indices = sorted(
                int(name.split('_')[1])
                for name in archive.files
                if name.startswith('weight_')
            )
            return cls(
                q_mean=float(archive['q_mean']),
                q_scale=float(archive['q_scale']),
                x_mean=archive['x_mean'],
                x_scale=archive['x_scale'],
                weights=tuple(archive[f'weight_{i}'] for i in indices),
                biases=tuple(archive[f'bias_{i}'] for i in indices),
                q_min=float(archive['q_min']),
                q_max=float(archive['q_max']),
            )
