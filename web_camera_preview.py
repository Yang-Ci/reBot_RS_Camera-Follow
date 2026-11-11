"""Best-effort, bounded preview of frames already acquired by the controller."""
import os
import queue
import threading
import time

import cv2


class CameraPreviewWriter:
    def __init__(self, path, fps=10):
        self.path = path
        self.period = 1.0 / fps
        self.last_submit = 0.0
        self.capture_started = None
        self.capture_count = 0
        self.capture_fps = None
        self.frames = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='camera-preview', daemon=True)
        self.thread.start()

    def submit(self, frame):
        now = time.monotonic()
        if self.capture_started is None:
            self.capture_started = now
        self.capture_count += 1
        elapsed = now - self.capture_started
        if elapsed >= 1.0:
            self.capture_fps = self.capture_count / elapsed
            self.capture_count = 0
            self.capture_started = now
        if now - self.last_submit < self.period or self.stop_event.is_set():
            return
        self.last_submit = now
        try:
            self.frames.put_nowait((frame.copy(), self.capture_fps))
        except queue.Full:
            pass

    def _run(self):
        temporary = self.path + '.tmp'
        while not self.stop_event.is_set():
            try:
                frame, capture_fps = self.frames.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok and not self.stop_event.is_set():
                    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
                    with os.fdopen(fd, 'wb') as output:
                        output.write(encoded.tobytes())
                    os.replace(temporary, self.path)
                    if capture_fps is not None:
                        with open(self.path + '.fps.tmp', 'w', encoding='ascii') as output:
                            output.write(f'{capture_fps:.1f}')
                        os.replace(self.path + '.fps.tmp', self.path + '.fps')
            except Exception as exc:
                print(f'[网页相机] 预览写入失败: {exc}', flush=True)
                break
        for path in (temporary, self.path, self.path + '.fps.tmp', self.path + '.fps'):
            try:
                os.unlink(path)
            except OSError:
                pass

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)


class StartupCameraPreview:
    """Own camera reads during HOME; relinquish it before the tracking loop."""
    def __init__(self, camera, writer):
        self.camera = camera
        self.writer = writer
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='startup-camera-preview', daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                color, _, _ = self.camera.get_frames()
                if color is not None:
                    self.writer.submit(color)
                else:
                    self.stop_event.wait(.01)
            except Exception as exc:
                print(f'[网页相机] 初始化期间读取失败: {exc}', flush=True)
                break

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        return not self.thread.is_alive()
