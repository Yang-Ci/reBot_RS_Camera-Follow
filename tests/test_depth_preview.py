"""Depth preview uses the current frame and never modifies control inputs."""
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from web_camera_preview import CameraPreviewWriter, compose_depth_preview


class DepthPreviewTests(unittest.TestCase):
    def setUp(self):
        self.color = np.full((480, 640, 3), 70, dtype=np.uint8)
        self.depth = np.full((480, 640), 500, dtype=np.uint16)
        self.depth[:, :320] = 0

    def test_inset_marks_invalid_pixels_black_and_preserves_inputs(self):
        color, depth = self.color.copy(), self.depth.copy()
        output = compose_depth_preview(self.color, self.depth, .001)
        np.testing.assert_array_equal(self.color, color)
        np.testing.assert_array_equal(self.depth, depth)
        np.testing.assert_array_equal(output[108, 18], [0, 0, 0])
        self.assertTrue(np.any(output[108, 208] != 0))
        np.testing.assert_array_equal(output[300, 500], color[300, 500])

    def test_absent_depth_is_black_not_previous_frame(self):
        output = compose_depth_preview(self.color, None, None)
        np.testing.assert_array_equal(output[108, 108], [0, 0, 0])
        np.testing.assert_array_equal(output[300, 500], self.color[300, 500])

    def test_writer_encodes_depth_in_background(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'camera.jpg'
            writer = CameraPreviewWriter(str(path), fps=10)
            try:
                writer.submit(self.color, self.depth, .001)
                end = time.monotonic() + 2.
                while not path.exists() and time.monotonic() < end:
                    time.sleep(.01)
                self.assertTrue(path.exists())
                preview = cv2.imread(str(path))
                self.assertEqual(preview.shape, self.color.shape)
                self.assertLess(int(preview[108, 18].max()), 10)
                self.assertGreater(int(preview[108, 208].max()), 20)
            finally:
                writer.close()


if __name__ == '__main__':
    unittest.main()
