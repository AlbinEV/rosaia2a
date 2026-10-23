"""Small constant-velocity Kalman filter used for marker stabilization."""

from __future__ import annotations

import numpy as np


class PositionKalman3D:
    """Filter a 3-D position while retaining velocity as hidden state."""

    def __init__(
        self,
        lateral_std_m: float,
        depth_std_m: float,
        acceleration_std_m_s2: float,
        reset_gap_s: float,
    ) -> None:
        if min(
            lateral_std_m,
            depth_std_m,
            acceleration_std_m_s2,
            reset_gap_s,
        ) <= 0.0:
            raise ValueError('Kalman parameters must be positive')
        self._measurement_covariance = np.diag([
            lateral_std_m ** 2,
            lateral_std_m ** 2,
            depth_std_m ** 2,
        ])
        self._acceleration_variance = acceleration_std_m_s2 ** 2
        self._reset_gap_s = reset_gap_s
        self._state: np.ndarray | None = None
        self._covariance: np.ndarray | None = None
        self._time_s: float | None = None

    def update(self, measurement: np.ndarray, time_s: float) -> np.ndarray:
        """Incorporate a measured point and return the filtered position."""
        value = np.asarray(measurement, dtype=np.float64).reshape(3)
        if not np.isfinite(value).all() or not np.isfinite(time_s):
            raise ValueError('measurement and timestamp must be finite')
        if (
            self._state is None
            or self._time_s is None
            or time_s <= self._time_s
            or time_s - self._time_s > self._reset_gap_s
        ):
            self._state = np.r_[value, np.zeros(3)]
            self._covariance = np.diag([
                *np.diag(self._measurement_covariance), 0.1, 0.1, 0.1
            ])
            self._time_s = time_s
            return value.copy()

        dt = time_s - self._time_s
        transition = np.eye(6)
        transition[:3, 3:] = np.eye(3) * dt
        gain_vector = np.array([0.5 * dt ** 2, dt])
        process = np.kron(
            np.outer(gain_vector, gain_vector),
            np.eye(3) * self._acceleration_variance,
        )
        predicted_state = transition @ self._state
        predicted_covariance = (
            transition @ self._covariance @ transition.T + process
        )
        observation = np.zeros((3, 6))
        observation[:, :3] = np.eye(3)
        innovation_covariance = (
            observation @ predicted_covariance @ observation.T
            + self._measurement_covariance
        )
        kalman_gain = np.linalg.solve(
            innovation_covariance,
            observation @ predicted_covariance,
        ).T
        self._state = predicted_state + kalman_gain @ (
            value - observation @ predicted_state
        )
        identity = np.eye(6)
        correction = identity - kalman_gain @ observation
        self._covariance = (
            correction @ predicted_covariance @ correction.T
            + kalman_gain @ self._measurement_covariance @ kalman_gain.T
        )
        self._time_s = time_s
        return self._state[:3].copy()
