"""Recovery discards partial streams and resets state only on success."""
import threading
import unittest
from unittest import mock

from robot_follow.runtime.observation import TargetObservation
from robot_follow.runtime.recovery import reconnect_camera


class CameraRecoveryTests(unittest.TestCase):
    def test_success_resets_pipeline_clock_and_target(self):
        session, pipeline, clock = mock.Mock(), mock.Mock(), mock.Mock()
        observation = TargetObservation(position_cam=[1., 2., 3.], visible=True)
        depth_filter = object()
        with mock.patch('robot_follow.runtime.recovery.time.sleep'):
            result = reconnect_camera(session, pipeline, clock, observation,
                                      threading.Lock(), lambda: depth_filter)
        self.assertTrue(result)
        session.stop.assert_called_once_with()
        session.open.assert_called_once_with()
        pipeline.reset.assert_called_once_with(depth_filter)
        clock.reset.assert_called_once_with()
        self.assertIsNone(observation.position_cam)
        self.assertFalse(observation.visible)

    def test_reset_failure_closes_new_stream(self):
        session, pipeline, clock = mock.Mock(), mock.Mock(), mock.Mock()
        pipeline.reset.side_effect = RuntimeError('tracking reset failed')
        observation = TargetObservation(position_cam=[1., 2., 3.], visible=False)
        with mock.patch('robot_follow.runtime.recovery.time.sleep'):
            result = reconnect_camera(session, pipeline, clock, observation,
                                      threading.Lock(), lambda: object())
        self.assertFalse(result)
        self.assertEqual(session.stop.call_count, 2)
        clock.reset.assert_not_called()
        self.assertEqual(observation.position_cam, [1., 2., 3.])


if __name__ == '__main__':
    unittest.main()
