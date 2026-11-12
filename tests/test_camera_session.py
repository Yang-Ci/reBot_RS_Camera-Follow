"""Camera replacement must preserve the stream used to calibrate control."""
import unittest
from unittest import mock

import numpy as np

from robot_follow.hardware.camera import CameraSession


class FakeCamera:
    def __init__(self, serial='one', width=640, fx=500., color=True):
        self.camera_serial = serial
        self.width, self.height = width, 480
        self.fx, self.fy, self.cx, self.cy = fx, 500., 320., 240.
        self.has_color = color
        self.image_stream = 'color' if color else 'infrared'
        self.camera_matrix = np.array([[fx, 0, 320], [0, 500, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros(5)
        self.stopped = 0
        self.frames = ('color', 'depth', .001)

    def get_frames(self):
        if isinstance(self.frames, Exception):
            raise self.frames
        return self.frames

    def stop(self):
        self.stopped += 1


class CameraSessionTests(unittest.TestCase):
    def test_matching_reconnect_and_idempotent_shutdown(self):
        first, second = FakeCamera(), FakeCamera()
        session = CameraSession(mock.Mock(side_effect=[first, second]), lambda camera: None)
        self.assertIs(session.open(), first)
        self.assertIs(session.reconnect(), second)
        self.assertEqual(first.stopped, 1)
        self.assertEqual(session.get_frames(), ('color', 'depth', .001))
        session.stop()
        session.stop()
        self.assertEqual(second.stopped, 1)

    def test_incompatible_reconnect_is_closed_and_remains_disconnected(self):
        for changed in (FakeCamera(serial='other'), FakeCamera(width=1280),
                        FakeCamera(fx=600), FakeCamera(color=False)):
            with self.subTest(changed=changed):
                first = FakeCamera()
                session = CameraSession(mock.Mock(side_effect=[first, changed]), lambda camera: None)
                session.open()
                with self.assertRaisesRegex(ValueError, '不一致'):
                    session.reconnect()
                self.assertEqual(first.stopped, 1)
                self.assertEqual(changed.stopped, 1)
                self.assertIsNone(session.camera)
                self.assertEqual(session.get_frames(), (None, None, None))

    def test_failed_open_can_be_retried(self):
        first, replacement = FakeCamera(), FakeCamera()
        session = CameraSession(mock.Mock(side_effect=[first, RuntimeError('offline'), replacement]),
                                lambda camera: None)
        session.open()
        with self.assertRaisesRegex(RuntimeError, 'offline'):
            session.reconnect()
        self.assertEqual(session.get_frames(), (None, None, None))
        self.assertIs(session.open(), replacement)
        session.stop()

    def test_validation_and_frame_failure(self):
        rejected = FakeCamera()
        session = CameraSession(lambda: rejected, mock.Mock(side_effect=ValueError('calibration')))
        with self.assertRaisesRegex(ValueError, 'calibration'):
            session.open()
        self.assertEqual(rejected.stopped, 1)
        camera = FakeCamera()
        session = CameraSession(lambda: camera, lambda camera: None)
        session.open()
        camera.frames = RuntimeError('device removed')
        self.assertEqual(session.get_frames(), (None, None, None))
        self.assertEqual(session.last_error, 'device removed')
        session.stop()


if __name__ == '__main__':
    unittest.main()
