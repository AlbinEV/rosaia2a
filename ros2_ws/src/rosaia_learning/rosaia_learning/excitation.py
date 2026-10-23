"""Generate bounded velocity-reference profiles for identification."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def step_profile(
    levels: list[float], dwell_s: float, sample_rate_hz: float
) -> tuple[np.ndarray, np.ndarray]:
    """Generate consecutive constant levels with equal dwell time."""
    if not levels or dwell_s <= 0.0 or sample_rate_hz <= 0.0:
        raise ValueError('levels, dwell and sample rate must be positive')
    samples_per_level = max(1, round(dwell_s * sample_rate_hz))
    values = np.repeat(np.asarray(levels, dtype=np.float64), samples_per_level)
    times = np.arange(len(values), dtype=np.float64) / sample_rate_hz
    return times, values


def step_velocity_cycle(
    duration_s: float,
    dwell_s: float,
    sample_rate_hz: float,
    velocity: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate constant outward velocity, dwell, and constant return."""
    if (
        duration_s <= 0.0
        or dwell_s < 0.0
        or sample_rate_hz <= 0.0
        or velocity <= 0.0
    ):
        raise ValueError('invalid step-cycle parameters')
    move_count = max(1, round(duration_s * sample_rate_hz))
    dwell_count = round(dwell_s * sample_rate_hz)
    values = np.r_[
        np.full(move_count, velocity),
        np.zeros(dwell_count),
        np.full(move_count, -velocity),
        0.0,
    ]
    times = np.arange(len(values), dtype=np.float64) / sample_rate_hz
    return times, values


def fourier_profile(
    frequencies_hz: list[float],
    duration_s: float,
    sample_rate_hz: float,
    maximum_abs_velocity: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a phase-randomized, peak-normalized multisine profile."""
    if (
        not frequencies_hz
        or duration_s <= 0.0
        or sample_rate_hz <= 0.0
        or maximum_abs_velocity <= 0.0
        or any(value <= 0.0 for value in frequencies_hz)
    ):
        raise ValueError('invalid Fourier-profile parameters')
    count = max(2, round(duration_s * sample_rate_hz))
    times = np.arange(count, dtype=np.float64) / sample_rate_hz
    generator = np.random.default_rng(seed)
    phases = generator.uniform(0.0, 2.0 * np.pi, len(frequencies_hz))
    values = np.zeros_like(times)
    for frequency, phase in zip(frequencies_hz, phases):
        values += np.sin(2.0 * np.pi * frequency * times + phase)
    peak = float(np.max(np.abs(values)))
    if peak <= 1e-12:
        raise ValueError('degenerate Fourier profile')
    values *= maximum_abs_velocity / peak
    return times, values


def trapezoidal_velocity_profile(
    duration_s: float,
    rise_s: float,
    fall_s: float,
    sample_rate_hz: float,
    maximum_velocity: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a continuous accelerate/hold/decelerate velocity reference."""
    if (
        duration_s <= 0.0
        or rise_s <= 0.0
        or fall_s <= 0.0
        or rise_s + fall_s > duration_s
        or sample_rate_hz <= 0.0
        or maximum_velocity <= 0.0
    ):
        raise ValueError('invalid trapezoidal-profile parameters')
    count = max(2, round(duration_s * sample_rate_hz) + 1)
    times = np.linspace(0.0, duration_s, count)
    values = np.full(count, maximum_velocity, dtype=np.float64)
    rising = times < rise_s
    falling = times > duration_s - fall_s
    values[rising] = maximum_velocity * times[rising] / rise_s
    values[falling] = (
        maximum_velocity * (duration_s - times[falling]) / fall_s
    )
    values[0] = 0.0
    values[-1] = 0.0
    return times, values


def append_return_home(
    times: np.ndarray,
    values: np.ndarray,
    dwell_s: float,
    sample_rate_hz: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Append a dwell and sign-reversed copy with no duplicate endpoints."""
    if dwell_s < 0.0 or sample_rate_hz <= 0.0:
        raise ValueError('dwell must be non-negative and rate positive')
    dwell_count = round(dwell_s * sample_rate_hz)
    interval = 1.0 / sample_rate_hz
    dwell_values = np.zeros(dwell_count, dtype=np.float64)
    combined_values = np.r_[values, dwell_values, -values[1:]]
    combined_times = np.arange(len(combined_values), dtype=np.float64) * interval
    return combined_times, combined_values


def _numbers(value: str) -> list[float]:
    return [float(item) for item in value.split(',') if item.strip()]


def main(args=None) -> None:
    """Create a CSV reference profile without actuating hardware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--type', choices=('step', 'fourier', 'trapezoid'), required=True
    )
    parser.add_argument('--axis', type=int, choices=(0, 1), required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--levels', type=_numbers, default=[0, 5, 10, 0, -5, -10, 0])
    parser.add_argument('--dwell', type=float, default=2.0)
    parser.add_argument('--frequencies', type=_numbers, default=[0.05, 0.11, 0.19])
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--maximum-velocity', type=float, default=10.0)
    parser.add_argument('--rise-time', type=float, default=3.0)
    parser.add_argument('--fall-time', type=float, default=2.0)
    parser.add_argument('--return-home', action='store_true')
    parser.add_argument('--home-dwell', type=float, default=1.0)
    parser.add_argument('--seed', type=int, default=7)
    options = parser.parse_args(args)
    if options.type == 'step':
        times, values = step_profile(options.levels, options.dwell, options.rate)
    elif options.type == 'fourier':
        times, values = fourier_profile(
            options.frequencies,
            options.duration,
            options.rate,
            options.maximum_velocity,
            options.seed,
        )
    else:
        times, values = trapezoidal_velocity_profile(
            options.duration,
            options.rise_time,
            options.fall_time,
            options.rate,
            options.maximum_velocity,
        )
        if options.return_home:
            times, values = append_return_home(
                times, values, options.home_dwell, options.rate
            )
    output = Path(options.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'qdot_arm_1', 'qdot_arm_2'])
        for time_value, reference in zip(times, values):
            commands = [0.0, 0.0]
            commands[options.axis] = float(reference)
            writer.writerow([time_value, *commands])
    print(f'wrote {len(times)} non-actuating references to {output}')
