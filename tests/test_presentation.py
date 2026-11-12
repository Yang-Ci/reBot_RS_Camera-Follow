"""Display and diagnostic formatting without a GUI or camera."""
import unittest
from unittest import mock

import numpy as np

from robot_follow.presentation.diagnostics import DebugFrame, format_debug
from robot_follow.presentation.display import DisplayFrame, VisionDisplay


class PresentationTests(unittest.TestCase):
    def frame(self, **overrides):
        values = dict(image=np.zeros((480, 640, 3), dtype=np.uint8), track=None,
                      landmarks=None, pinch_points=None, infrared=False,
                      desired_u=320, desired_v=240, gripper_enabled=True,
                      gripper_closed_deg=0., gripper_open_deg=100., gripper_deg=50.,
                      gripper_frozen=False, no_arm=False, follow_enabled=True,
                      following=True, status={'active': True, 'joint_speed': .2, 'error': ''},
                      fps=30.)
        values.update(overrides)
        return DisplayFrame(**values)

    def test_display_renders_follow_state_and_returns_key(self):
        palm, landmarks = mock.Mock(), mock.Mock()
        display = VisionDisplay(palm, landmarks)
        frame = self.frame(landmarks='hand', pinch_points=((1, 2), (3, 4)),
                           gripper_frozen=True)
        with mock.patch('robot_follow.presentation.display.cv2') as cv:
            cv.waitKey.return_value = ord('q')
            self.assertEqual(display.show(frame), ord('q'))
            texts = [call.args[1] for call in cv.putText.call_args_list]
            self.assertIn('FOLLOWING', texts)
            self.assertTrue(any('50%' in text and '[FROZEN]' in text for text in texts))
            cv.line.assert_called_once()
            cv.imshow.assert_called_once()
        palm.assert_called_once()
        landmarks.assert_called_once()
        self.assertTrue(np.all(frame.image == 0))

    def test_display_vision_only_and_hold(self):
        display = VisionDisplay(mock.Mock(), mock.Mock())
        with mock.patch('robot_follow.presentation.display.cv2') as cv:
            cv.waitKey.return_value = -1
            display.show(self.frame(no_arm=True))
            self.assertIn('VISION ONLY (--no-arm)', [call.args[1] for call in cv.putText.call_args_list])
            cv.reset_mock()
            display.show(self.frame(following=True, status={'active': False, 'joint_speed': 0., 'error': ''}))
            self.assertIn('HOLD (hand lost)', [call.args[1] for call in cv.putText.call_args_list])

    def test_debug_format_preserves_key_diagnostics(self):
        frame = DebugFrame(fps=30., track={'u': 100., 'v': 120., 'lost': 0, 'z3d': .35,
                                           'z3d_frozen': True, 'bridged': True},
                           following=True, visible=True, status={'joint_speed': .1, 'error': '', 'ffm': False},
                           pinch_px=100., gripper_deg=50., grab_ms=5., detect_ms=10.,
                           processing_ms=11., hardware_time=True, dropped_frames=2)
        text = format_debug(frame)
        for expected in ('❄冻结', '◦桥接', '·静止', 'pinch=100px→50.0°', 'hw=1', '丢帧=2'):
            self.assertIn(expected, text)


if __name__ == '__main__':
    unittest.main()
