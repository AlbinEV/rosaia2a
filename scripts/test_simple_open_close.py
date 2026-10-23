"""Non-actuating checks for the coarse demo's toggle policy."""

import unittest
import json
from pathlib import Path
import tempfile

from simple_open_close import next_target, validate_target, validate_nominals, save_nominal_file, validate_jog, smooth_pwm, velocity_pwm, scaled_pwm, timer_wrapped, validate_wrap_reference


class ToggleTests(unittest.TestCase):
    def test_pwm_intensity_scales_tested_baselines(self):
        self.assertEqual(scaled_pwm(200, 100), 200)
        self.assertEqual(scaled_pwm(200, 120), 240)
        self.assertEqual(scaled_pwm(140, 50), 70)
        for percent in (49, 121, True, 100.5):
            with self.assertRaises(ValueError):
                scaled_pwm(200, percent)

    def test_timer_wrap_is_not_board_reset(self):
        before = {'mcu_time_us': 4294956404, 'sequence': 214718, 'encoder_counts': [79,-80]}
        after = {'mcu_time_us': 9628, 'sequence': 214719, 'encoder_counts': [79,-80]}
        self.assertTrue(timer_wrapped(before, after))
        self.assertFalse(timer_wrapped(before, {**after, 'sequence': 0}))
        self.assertFalse(timer_wrapped(before, {**after, 'encoder_counts': [0,0]}))

    def test_velocity_feedback_reduces_drive_when_too_fast(self):
        self.assertLess(velocity_pwm(80, 400, 84, 1, 150, .05),
                        velocity_pwm(80, 30, 84, 1, 150, .05))
        self.assertEqual(velocity_pwm(3, 0, 84, 1, 150, .5), 0)

    def test_velocity_feedback_breakaway_is_bounded(self):
        self.assertEqual(velocity_pwm(80, 0, 84, 1, 150, 1), 150)
        self.assertEqual(velocity_pwm(80, 900, 84, 1, 150, .05), 0)

    def test_continuous_pwm_ramp_and_bounds(self):
        self.assertEqual(smooth_pwm(80, 0, 150), 0)
        self.assertLess(smooth_pwm(80, .1, 150), smooth_pwm(80, .35, 150))
        self.assertEqual(smooth_pwm(3, 1, 200), 0)
        self.assertLess(smooth_pwm(8, 1, 150), smooth_pwm(80, 1, 150))
        for remaining in (5, 10, 80, 140):
            for elapsed in (0, .1, 1):
                self.assertTrue(0 <= smooth_pwm(remaining, elapsed, 200, .2) <= 200)

    def test_continuous_pwm_boost_never_exceeds_tested_peak(self):
        self.assertEqual(smooth_pwm(20, 1, 150, .5, boost=True), 150)

    def test_released_pose_closes(self):
        for q in (-8, -1, 0, 4):
            self.assertEqual(next_target(q), 20)

    def test_partial_or_complete_close_releases(self):
        for q in (5, 12, 16, 20, 30, 40):
            self.assertEqual(next_target(q), 0)

    def test_unknown_pose_is_rejected(self):
        for q in (-17, 41):
            with self.assertRaises(ValueError):
                next_target(q)

    def test_manual_targets(self):
        for value in (0, 12, 30, '20', '12.0'):
            self.assertEqual(validate_target(value), int(float(value)))

    def test_invalid_manual_targets(self):
        for value in (-1, 31, 12.5, True, None, '', 'abc', float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                validate_target(value)

    def test_nominal_order(self):
        self.assertEqual(validate_nominals(0, 24), (0, 24))
        self.assertEqual(next_target(2, 2, 24), 24)
        self.assertEqual(next_target(18, 2, 24), 2)
        for pair in ((20, 20), (20, 0), (0, 31)):
            with self.assertRaises(ValueError):
                validate_nominals(*pair)

    def test_atomic_persistence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'nominals.json'
            save_nominal_file(path, {'open_q': 0, 'close_q': 20})
            save_nominal_file(path, {'open_q': 0, 'close_q': 24})
            self.assertEqual(json.loads(path.read_text()), {'open_q': 0, 'close_q': 24})
            self.assertEqual(list(Path(directory).glob('.nominals_*')), [])

    def test_jog_is_one_bounded_step_on_either_axis(self):
        for axis in (0, 1):
            self.assertEqual(validate_jog(axis, 1, 0), -5)
            self.assertEqual(validate_jog(axis, -1, 20), 5)

    def test_jog_rejects_invalid_axes_and_limits(self):
        for args in ((True, 1, 0), (2, 1, 0), (0, True, 0),
                     (0, 0, 0)):
            with self.assertRaises(ValueError):
                validate_jog(*args)

    def test_jog_has_no_absolute_position_cap(self):
        self.assertEqual(validate_jog(1, 1, 120, 10), -10)
        self.assertEqual(validate_jog(0, 1, 100000, 1), -1)
        self.assertEqual(validate_jog(0, -1, -10, 3), 3)

    def test_jog_step_is_editable_within_firmware_bound(self):
        for step in (1, 3, 7, 10):
            self.assertEqual(validate_jog(0, 1, 0, step), -step)
        for step in (0, -1, 11, 1.5, True, '5', None):
            with self.assertRaises(ValueError):
                validate_jog(0, 1, 0, step)


if __name__ == '__main__':
    unittest.main()
