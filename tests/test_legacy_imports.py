"""Legacy import paths keep referring to the packaged implementations."""
import unittest

import kalman_target
import servo_diagnostics
import servo_timing
from robot_follow.control import target_filter, timing
from robot_follow.runtime import diagnostics


class LegacyImportTests(unittest.TestCase):
    def test_control_symbols_are_identical(self):
        self.assertIs(kalman_target.KalmanTarget3D, target_filter.KalmanTarget3D)
        self.assertIs(servo_timing.AdaptiveLeadGate, timing.AdaptiveLeadGate)
        self.assertIs(servo_timing._smoothstep, timing._smoothstep)

    def test_diagnostic_symbols_are_identical(self):
        self.assertIs(servo_diagnostics.DiagnosticRecorder, diagnostics.DiagnosticRecorder)
        self.assertIs(servo_diagnostics.ActualFeedbackSampler, diagnostics.ActualFeedbackSampler)
        self.assertIs(servo_diagnostics.DIAG_FIELDS, diagnostics.DIAG_FIELDS)


if __name__ == '__main__':
    unittest.main()
