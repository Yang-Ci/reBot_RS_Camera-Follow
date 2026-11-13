"""The confirmed fixed gripper exclusion cannot supply a palm depth."""
import unittest
from unittest import mock

import numpy as np

from robot_follow.perception.gripper_mask import GripperMask
from robot_follow.perception.depth_sampling import get_median_depth
from robot_follow.perception.depth import DepthGate
from robot_follow.perception.pipeline import HandPipeline


class Tracker:
    def update(self, detection):
        if detection is None:
            return None
        return {'u': detection[0], 'v': detection[1], 'lost': 0,
                'x3d': 0., 'y3d': 0., 'z3d': 0.}


class GripperMaskTests(unittest.TestCase):
    def test_polygon_scales_and_rgb_input_is_not_mutated(self):
        for w, h in ((640, 480), (1280, 960)):
            mask = GripperMask(w, h)
            image = np.full((h, w, 3), 200, dtype=np.uint8)
            result = mask.apply_rgb(image)
            self.assertTrue(mask.contains(.55*w, .9*h))
            self.assertFalse(mask.contains(.5*w, .4*h))
            self.assertEqual(int(result[int(.9*h), int(.55*w), 0]), 0)
            self.assertEqual(int(image[int(.9*h), int(.55*w), 0]), 200)

    def test_masked_depth_pixels_and_masked_center_are_rejected(self):
        mask = GripperMask(640, 480)
        depth = np.full((480, 640), 500, dtype=np.uint16)
        depth[mask.allowed == 0] = 1100
        self.assertEqual(get_median_depth(depth, .001, 350, 440, allowed_mask=mask.allowed), 0.)
        self.assertAlmostEqual(get_median_depth(depth, .001, 320, 200, allowed_mask=mask.allowed), .5)
        boundary = np.argwhere((mask.allowed[:, :-1] != 0) & (mask.allowed[:, 1:] == 0))
        y, x = map(int, boundary[len(boundary)//2])
        self.assertAlmostEqual(get_median_depth(depth, .001, x, y, allowed_mask=mask.allowed), .5)
        with self.assertRaises(ValueError):
            get_median_depth(depth, .001, 320, 200, allowed_mask=np.zeros((1, 1)))

    def test_detector_and_optical_flow_cannot_select_masked_gripper(self):
        detector, bridge = mock.Mock(), mock.Mock()
        detector.detect.return_value = (350., 440., None)
        bridge.update.return_value = (350., 440., 'lk')
        gate = DepthGate(desired_depth=.35, max_jump=.08, max_step=.05,
                         freeze_seconds=.2, depth_filter=lambda value: value)
        pipeline = HandPipeline(detector=detector, bridge=bridge, tracker_factory=Tracker,
                                depth_gate=gate, median_depth=get_median_depth,
                                project=lambda u,v,z:(u,v,z), mask_rgb_gripper=True)
        image = np.full((480,640,3), 200, np.uint8)
        result = pipeline.process(image, np.full((480,640), 500, np.uint16), .001, now=1.)
        self.assertIsNone(result.track)
        self.assertTrue(bridge.reset.called)
        self.assertEqual(int(detector.detect.call_args.args[0][440,350,0]), 0)
        self.assertEqual(int(image[440,350,0]), 200)

    def test_default_pipeline_keeps_bottom_image_unmasked(self):
        detector, bridge = mock.Mock(), mock.Mock()
        detector.detect.return_value = (350., 440., None)
        bridge.update.return_value = (350., 440., 'det')
        gate = DepthGate(desired_depth=.35, max_jump=.08, max_step=.05,
                         freeze_seconds=.2, depth_filter=lambda value: value)
        pipeline = HandPipeline(detector=detector, bridge=bridge, tracker_factory=Tracker,
                                depth_gate=gate, median_depth=get_median_depth,
                                project=lambda u, v, z: (u, v, z))
        image = np.full((480, 640, 3), 200, np.uint8)
        depth = np.full((480, 640), 400, np.uint16)
        for i in range(5):
            result = pipeline.process(image, depth, .001, now=1. + i*.01)
        self.assertAlmostEqual(result.track['z3d'], .4)
        self.assertIsNone(pipeline.gripper_mask)
        self.assertEqual(int(detector.detect.call_args.args[0][440, 350, 0]), 200)

    def test_valid_palm_outside_gripper_keeps_its_depth(self):
        detector, bridge = mock.Mock(), mock.Mock()
        detector.detect.return_value = (320., 200., None)
        bridge.update.return_value = (320., 200., 'det')
        gate = DepthGate(desired_depth=.35, max_jump=.08, max_step=.05,
                         freeze_seconds=.2, depth_filter=lambda value: value)
        median = mock.Mock(wraps=get_median_depth)
        pipeline = HandPipeline(detector=detector, bridge=bridge, tracker_factory=Tracker,
                                depth_gate=gate, median_depth=median,
                                project=lambda u,v,z:(u,v,z), mask_rgb_gripper=True)
        image = np.full((480,640,3), 200, np.uint8)
        depth = np.full((480,640), 400, np.uint16)
        for i in range(5):
            result = pipeline.process(image, depth, .001, now=1. + i*.01)
        self.assertAlmostEqual(result.track['z3d'], .4)
        self.assertIs(median.call_args.kwargs['allowed_mask'], pipeline.gripper_mask.allowed)


if __name__ == '__main__':
    unittest.main()
