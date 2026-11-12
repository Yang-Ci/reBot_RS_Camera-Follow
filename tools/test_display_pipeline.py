"""Exercise the non-headless display loop with fake hardware and GUI calls."""
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
source_path = ROOT / 'tools/offline_smoke_test.py'
source = source_path.read_text(encoding='utf-8').split('# 默认交付配置必须')[0]
namespace = {'__name__': 'display_pipeline_harness', '__file__': str(source_path)}
exec(compile(source, str(source_path), 'exec'), namespace)
module = namespace['main_module']


class DisplayPipelineTests(unittest.TestCase):
    def test_visual_mode_renders_and_quit_releases_camera(self):
        cameras = []
        class TrackedCamera(namespace['FakeCamera']):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                cameras.append(self)
        with mock.patch.object(sys, 'argv', ['palm_follow_grasp.py', '--no-arm']), \
             mock.patch.object(sys.modules['gemini2_camera'], 'Gemini2Camera', TrackedCamera), \
             mock.patch.object(module.cv2, 'namedWindow') as named, \
             mock.patch.object(module.cv2, 'resizeWindow'), \
             mock.patch.object(module.cv2, 'imshow') as show, \
             mock.patch.object(module.cv2, 'waitKey', return_value=ord('q')), \
             mock.patch.object(module.cv2, 'putText') as text, \
             mock.patch.object(module.cv2, 'destroyAllWindows') as destroy:
            module.main()
        named.assert_called_once()
        show.assert_called_once()
        destroy.assert_called_once()
        self.assertIn('VISION ONLY (--no-arm)', [call.args[1] for call in text.call_args_list])
        self.assertTrue(cameras[0].stopped)


if __name__ == '__main__':
    unittest.main()
