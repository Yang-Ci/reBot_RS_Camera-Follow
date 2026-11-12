"""Legacy hand-detection imports retain the model path and mutable settings."""
import os
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class HandImportTests(unittest.TestCase):
    def test_legacy_alias_model_path_and_frame_timestamps(self):
        code = r'''
import sys, types
from pathlib import Path
from unittest.mock import patch
import numpy as np
mp = types.ModuleType('mediapipe'); mp.__path__ = []
tasks = types.ModuleType('mediapipe.tasks'); tasks.__path__ = []
python = types.ModuleType('mediapipe.tasks.python'); python.__path__ = []
vision = types.ModuleType('mediapipe.tasks.python.vision')
tasks.python = python; python.vision = vision
sys.modules.update({'mediapipe': mp, 'mediapipe.tasks': tasks,
                    'mediapipe.tasks.python': python, 'mediapipe.tasks.python.vision': vision})
import palm_detection
from robot_follow.perception import hand
assert palm_detection is hand
assert Path(hand.MODEL_PATH) == Path.cwd() / 'config' / 'hand_landmarker.task'
palm_detection.PALM_FILTER = 'ema'
assert hand.PALM_FILTER == 'ema'

# Consecutive frames can share a millisecond; VIDEO mode requires strict order.
timestamps = []
detector = hand.PalmDetector.__new__(hand.PalmDetector)
detector._timestamp_ms = 0
detector.detector = types.SimpleNamespace(
    detect_for_video=lambda image, timestamp: (
        timestamps.append(timestamp) or types.SimpleNamespace(hand_landmarks=[])))
mp.ImageFormat = types.SimpleNamespace(SRGB=object())
mp.Image = lambda **kwargs: object()
frame = np.zeros((2, 2, 3), dtype=np.uint8)
with patch.object(hand.time, 'monotonic', side_effect=[100.0, 100.0, 99.0, 101.0]):
    for _ in range(4):
        assert detector.detect(frame) == (None, None, None)
assert timestamps == [100000, 100001, 100002, 101000], timestamps
'''
        env = os.environ.copy()
        env['PYTHONUTF8'] = '1'
        result = subprocess.run([sys.executable, '-c', code], cwd=ROOT, env=env,
                                capture_output=True, text=True, encoding='utf-8', errors='replace')
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
