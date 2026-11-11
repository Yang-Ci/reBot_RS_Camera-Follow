#!/usr/bin/env python3
"""Standalone Gemini camera preview. Never opens the robot or CAN."""
import json
import signal
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    if sys.argv[1] == '--list':
        from pyorbbecsdk import Context
        devices = Context().query_devices()
        result = []
        for i in range(devices.get_count()):
            name = devices.get_device_name_by_index(i)
            if 'gemini 336' in name.lower():
                result.append({'name': name, 'serial': devices.get_device_serial_number_by_index(i)})
        print('CAMERA_LIST=' + json.dumps(result), flush=True)
        return
    from gemini2_camera import Gemini2Camera
    from web_camera_preview import CameraPreviewWriter
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    camera = None
    writer = None
    try:
        camera = Gemini2Camera(fps=60, serial=sys.argv[2])
        if 'gemini 336' not in camera.device_name.lower():
            raise ValueError('所选设备不是 Gemini 336')
        writer = CameraPreviewWriter(sys.argv[3])
        while not stop.is_set():
            color, _, _ = camera.get_frames()
            if color is not None:
                writer.submit(color)
            else:
                stop.wait(.01)
    finally:
        if writer is not None:
            writer.close()
        if camera is not None:
            camera.stop()


if __name__ == '__main__':
    main()
