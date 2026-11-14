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
        result = []
        try:
            from pyorbbecsdk import Context
            devices = Context().query_devices()
            for i in range(devices.get_count()):
                name = devices.get_device_name_by_index(i)
                lower = name.lower()
                model = ('gemini305' if 'gemini 305' in lower else
                         'gemini336' if 'gemini 336' in lower else
                         'gemini2' if 'gemini 2' in lower else None)
                if model:
                    result.append({'name': name, 'serial': devices.get_device_serial_number_by_index(i), 'model': model})
        except (ImportError, RuntimeError):
            pass
        try:
            import pyrealsense2 as rs
            for device in rs.context().query_devices():
                name = device.get_info(rs.camera_info.name)
                if 'd435i' in name.lower():
                    result.append({'name': name, 'serial': device.get_info(rs.camera_info.serial_number), 'model': 'd435i'})
        except (ImportError, RuntimeError):
            pass
        print('CAMERA_LIST=' + json.dumps(result), flush=True)
        return
    from web_camera_preview import CameraPreviewWriter
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    camera = None
    writer = None
    try:
        model = sys.argv[4] if len(sys.argv) > 4 else 'gemini336'
        if model == 'd435i':
            from realsense_camera import RealSenseCamera
            camera = RealSenseCamera(fps=30, serial=sys.argv[2])
        elif model in ('gemini2', 'gemini305', 'gemini336'):
            from gemini2_camera import Gemini2Camera
            camera = Gemini2Camera(fps=30, serial=sys.argv[2])
            from robot_follow.perception.calibration import verify_orbbec_camera
            verify_orbbec_camera(camera, model, None, require_calibration=False)
            if model == 'gemini2' and ('gemini 2' not in camera.device_name.lower() or 'gemini 305' in camera.device_name.lower() or 'gemini 336' in camera.device_name.lower()):
                raise ValueError('所选设备不是 Gemini 2')
        else:
            raise ValueError('不支持的相机型号')
        writer = CameraPreviewWriter(sys.argv[3])
        while not stop.is_set():
            color, depth, scale = camera.get_frames()
            if color is not None:
                writer.submit(color, depth, scale)
            else:
                stop.wait(.01)
    finally:
        if writer is not None:
            writer.close()
        if camera is not None:
            camera.stop()


if __name__ == '__main__':
    main()
