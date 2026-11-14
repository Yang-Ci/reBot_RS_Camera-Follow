"""Verify camera and joint-offset provenance before hardware control."""

import json
import os
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def verify_d435i_calibration(camera, calibration):
    """Reject a provenance mismatch when a newer calibration records it."""
    if not calibration:
        return
    serial = calibration.get("camera_serial")
    if serial is not None:
        device = camera.pipeline.get_active_profile().get_device()
        actual = device.get_info(camera.rs.camera_info.serial_number)
        if str(actual) != str(serial):
            raise ValueError("当前 RealSense 序列号与手眼标定不一致，请重新标定")
    stream = calibration.get("image_stream")
    if stream is not None and stream != ("color" if camera.has_color else "infrared_1"):
        raise ValueError("当前相机图像流与手眼标定不一致，请重新标定")
    verify_calibration_joint_offset(calibration)


def verify_calibration_joint_offset(calibration):
    recorded_offset = calibration.get("joint_offset_rad")
    if recorded_offset is not None:
        offset_path = os.path.join(PROJECT_ROOT, "config", "joint_offset.json")
        if not os.path.exists(offset_path):
            raise ValueError("标定时的关节零位配置不存在，请重新检查标定")
        with open(offset_path) as f:
            current_offset = np.asarray(json.load(f)["joint_offset_rad"], dtype=float)
        recorded_offset = np.asarray(recorded_offset, dtype=float)
        if (current_offset.shape != (6,) or recorded_offset.shape != (6,) or
                not np.all(np.isfinite(current_offset)) or
                not np.all(np.isfinite(recorded_offset)) or
                not np.allclose(current_offset, recorded_offset, atol=1e-8, rtol=0)):
            raise ValueError("关节零位修正与手眼标定时不同，请重新标定")


def verify_orbbec_camera(camera, selected, calibration, *, require_calibration):
    """Prevent an installed 336 from silently using the old Gemini 2 calibration."""
    name = str(getattr(camera, 'device_name', ''))
    is_336 = 'gemini 336' in name.lower()
    is_305 = 'gemini 305' in name.lower()
    if selected == 'gemini305' and not is_305:
        raise ValueError(f'所选相机为 Gemini 305，但实际设备为 {name or "未知"}')
    if selected != 'gemini305' and is_305 and require_calibration:
        raise ValueError('实际设备为 Gemini 305；请使用 --camera gemini305')
    if selected == 'gemini336' and not is_336:
        raise ValueError(f'所选相机为 Gemini 336，但实际设备为 {name or "未知"}')
    if selected == 'gemini2' and is_336 and require_calibration:
        raise ValueError('实际设备为 Gemini 336；请使用 --camera gemini336 及其独立手眼标定')
    if selected != 'gemini336' or not require_calibration:
        return
    if not calibration or str(calibration.get('camera_serial', '')) != str(camera.camera_serial):
        raise ValueError('Gemini 336 序列号与标定不一致或未记录，请重新标定')
    if calibration.get('camera') != 'Orbbec Gemini 336' or calibration.get('image_stream') != 'color':
        raise ValueError('Gemini 336 标定的相机型号或画面类型不匹配')
    if calibration.get('image_size') != [camera.width, camera.height]:
        raise ValueError('Gemini 336 标定分辨率与当前画面不一致')
    for key, actual in (('camera_matrix', camera.camera_matrix), ('dist_coeffs', camera.dist_coeffs)):
        expected = np.asarray(calibration.get(key, []), dtype=float)
        actual = np.asarray(actual, dtype=float)
        if expected.shape != actual.shape or not np.all(np.isfinite(expected)) or not np.allclose(expected, actual, rtol=1e-5, atol=1e-5):
            raise ValueError(f'Gemini 336 {key} 与标定不一致')
    if calibration.get('joint_offset_rad') is None:
        raise ValueError('Gemini 336 标定未记录关节零位修正')
    verify_calibration_joint_offset(calibration)
