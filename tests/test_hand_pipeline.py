"""Frame processing combines detector, bridge, tracker and depth gate."""
import unittest
from unittest import mock

import numpy as np

from robot_follow.perception.depth import DepthGate
from robot_follow.perception.pipeline import HandPipeline


class Tracker:
    def __init__(self):
        self.track = None
    def update(self, detection):
        if detection is None:
            return None
        u, v = detection
        self.track = {'u': u, 'v': v, 'lost': 0, 'x3d': 0, 'y3d': 0, 'z3d': 0}
        return self.track


class HandPipelineTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.detector = mock.Mock()
        self.bridge = mock.Mock()
        self.gate = DepthGate(desired_depth=.35, max_jump=.08, max_step=.05,
                              freeze_seconds=.3, depth_filter=lambda value: value)
        self.pipeline = HandPipeline(detector=self.detector, bridge=self.bridge,
                                     tracker_factory=Tracker, depth_gate=self.gate,
                                     median_depth=lambda *args: .4,
                                     project=lambda u, v, z: (u*z, v*z, z))

    def test_fresh_detection_accumulates_depth_and_keeps_landmarks(self):
        landmarks = object()
        self.detector.detect.return_value = (100., 100., landmarks)
        self.bridge.update.return_value = (100., 100., 'det')
        for i in range(5):
            frame = self.pipeline.process(self.image, None, .001, now=1 + i*.01)
        self.assertIs(frame.landmarks, landmarks)
        self.assertFalse(frame.track['bridged'])
        self.assertAlmostEqual(frame.track['z3d'], .4)

    def test_optical_flow_bridge_never_supplies_landmarks(self):
        self.detector.detect.return_value = (None, None, object())
        self.bridge.update.return_value = (100., 120., 'lk')
        frame = self.pipeline.process(self.image, None, .001, now=1.)
        self.assertIsNone(frame.landmarks)
        self.assertTrue(frame.track['bridged'])

    def test_infrared_gripper_region_is_excluded(self):
        self.pipeline.infrared = True
        self.detector.detect.return_value = (320., 300., None)
        self.bridge.update.return_value = (None, None, 'none')
        frame = self.pipeline.process(self.image, None, .001, now=1.)
        self.bridge.reset.assert_called_once()
        self.assertIsNone(frame.track)

    def test_reset_clears_tracking_and_depth_state(self):
        self.detector.detect.return_value = (100., 100., None)
        self.bridge.update.return_value = (100., 100., 'det')
        for i in range(5):
            self.pipeline.process(self.image, None, .001, now=1 + i*.01)
        old_tracker = self.pipeline.tracker
        new_filter = lambda value: value
        self.pipeline.reset(new_filter)
        self.assertIsNot(self.pipeline.tracker, old_tracker)
        self.assertEqual(self.gate.reference, 0.)
        self.assertIs(self.gate.depth_filter, new_filter)
        self.bridge.reset.assert_called_once()


if __name__ == '__main__':
    unittest.main()
