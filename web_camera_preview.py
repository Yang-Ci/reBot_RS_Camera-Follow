"""Best-effort, bounded preview of frames already acquired by the controller."""
import os
import queue
import threading
import time

import cv2
import numpy as np


def compose_depth_preview(color, depth, depth_scale):
    """Overlay a bounded, fixed-scale depth inset on a copy of the RGB frame.

    Black marks missing/out-of-range depth; red is near and blue is far.
    Depth data are for display only and never returned to the controller.
    """
    output = color.copy()
    height, width = output.shape[:2]
    if height < 80 or width < 120:
        return output
    inset_width = max(1, int(width * .45))
    unit = max(.5, width / 640.0)
    x = y = max(4, int(8 * unit))
    inset_height = max(1, min(int(inset_width * .75), height - y - 2))
    canvas = np.zeros((inset_height, inset_width, 3), dtype=np.uint8)
    valid_any = False
    try:
        scale = float(depth_scale)
        source = np.asarray(depth)
        if source.ndim == 2 and source.size and np.isfinite(scale) and scale > 0:
            small = cv2.resize(source, (inset_width, inset_height),
                               interpolation=cv2.INTER_NEAREST)
            meters = small.astype(np.float32) * scale
            valid = np.isfinite(meters) & (meters >= .15) & (meters <= 1.5)
            valid_any = bool(np.any(valid))
            normalized = np.clip((meters - .15) / 1.35, 0., 1.)
            normalized = np.where(valid, normalized, 0.)
            indices = np.uint8((1. - normalized) * 255.)
            canvas = cv2.applyColorMap(indices, cv2.COLORMAP_TURBO)
            canvas[~valid] = 0
    except (TypeError, ValueError, OverflowError, cv2.error):
        pass
    output[y:y + inset_height, x:x + inset_width] = canvas
    cv2.rectangle(output, (x, y), (x + inset_width - 1, y + inset_height - 1),
                  (230, 230, 230), 1)
    header_height = min(int(32 * unit), inset_height - 1)
    cv2.rectangle(output, (x + 1, y + 1),
                  (x + inset_width - 2, y + header_height), (15, 20, 15), -1)
    label = 'DEPTH 0.15-1.5m' if valid_any else 'DEPTH NO DATA'
    cv2.putText(output, label, (x + int(5 * unit), y + int(23 * unit)),
                cv2.FONT_HERSHEY_SIMPLEX, .62 * unit, (245, 245, 245),
                max(1, int(unit)), cv2.LINE_AA)
    if valid_any and inset_height > 55 * unit:
        cv2.rectangle(output, (x + 1, y + inset_height - int(25 * unit)),
                      (x + inset_width - 2, y + inset_height - 2), (15, 20, 15), -1)
        cv2.putText(output, 'RED NEAR / BLACK INVALID',
                    (x + int(5 * unit), y + inset_height - int(7 * unit)),
                    cv2.FONT_HERSHEY_SIMPLEX, .42 * unit, (235, 235, 235),
                    max(1, int(unit)), cv2.LINE_AA)
    return output


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

    def submit(self, frame, depth=None, depth_scale=None):
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
            self.frames.put_nowait((frame.copy(),
                                    None if depth is None else depth.copy(),
                                    depth_scale, self.capture_fps))
        except queue.Full:
            pass

    def _run(self):
        temporary = self.path + '.tmp'
        while not self.stop_event.is_set():
            try:
                frame, depth, depth_scale, capture_fps = self.frames.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                preview = compose_depth_preview(frame, depth, depth_scale)
                ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 75])
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
                color, depth, scale = self.camera.get_frames()
                if color is not None:
                    self.writer.submit(color, depth, scale)
                else:
                    self.stop_event.wait(.01)
            except Exception as exc:
                print(f'[网页相机] 初始化期间读取失败: {exc}', flush=True)
                break

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        return not self.thread.is_alive()
