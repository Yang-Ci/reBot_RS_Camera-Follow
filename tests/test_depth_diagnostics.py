"""Diagnostics distinguish camera, detector and depth rejection outcomes."""
import unittest

from robot_follow.perception.depth_diagnostics import DepthDiagnostics


class DepthDiagnosticsTests(unittest.TestCase):
    def test_counts_all_frames_and_resets_after_reporting(self):
        diagnostics = DepthDiagnostics(10., interval_s=5.)
        self.assertIsNone(diagnostics.record(None, 10.1, no_frame=True))
        self.assertIsNone(diagnostics.record(None, 10.2))
        self.assertIsNone(diagnostics.record({'depth_status': '深度跳变拒绝'}, 10.3))
        result = diagnostics.record({'depth_status': '有效'}, 15.)
        for text in ('帧/尝试=4', '相机无帧=1', '未检测到手=1', '深度跳变拒绝=1', '有效=1'):
            self.assertIn(text, result)
        self.assertIsNone(diagnostics.record({'depth_status': '有效'}, 15.1))
        self.assertIn('帧/尝试=2', diagnostics.record({'depth_status': '有效'}, 20.))


if __name__ == '__main__':
    unittest.main()
