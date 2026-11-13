#!/usr/bin/env python3
"""Save a stationary ROS-held arm pose as the follow application's HOME.

Reads the existing controller only; never sends a movement, enable or home
command. The controller must already hold position with gravity mode stopped.
"""
import json
import os
import shutil
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
from auto_calibrate_camera import robot_fk
from ros_calibration_client import RosCalibrationArm


def validate(samples, q_min, q_max, fk, workspace):
    samples = np.asarray(samples, dtype=float)
    if samples.ndim != 2 or samples.shape[0] < 5 or samples.shape[1] != 6 or not np.all(np.isfinite(samples)):
        raise ValueError('关节反馈不足或无效')
    if np.max(np.ptp(samples, axis=0)) > np.deg2rad(0.3):
        raise ValueError('机械臂尚未稳定，不能保存 HOME')
    q = samples[-1].copy()
    if np.any(q < q_min) or np.any(q > q_max):
        raise ValueError('HOME 关节角超出跟随程序的软限位，请调整姿态')
    p = np.asarray(fk(q), dtype=float)
    low = np.asarray(workspace['min'], dtype=float)
    high = np.asarray(workspace['max'], dtype=float)
    radius = float(workspace['max_radius'])
    if p.shape != (3,) or not np.all(np.isfinite(p)) or np.any(p < low) or np.any(p > high) or np.linalg.norm(p[:2]) > radius:
        raise ValueError('HOME 法兰位置超出跟随程序的工作范围，请调整姿态')
    return q, p


def save(path, q, p):
    path = Path(path)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    backup = path.with_name(f'{path.stem}.backup_{stamp}{path.suffix}')
    if path.exists():
        # Exclusive creation prevents accidental overwrite of an earlier backup.
        with path.open('rb') as source, backup.open('xb') as dest:
            shutil.copyfileobj(source, dest)
            dest.flush()
            os.fsync(dest.fileno())
    else:
        backup = None
    data = {'home_joint_pos': q.tolist(), 'tcp_position': p.tolist(),
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
    fd, temporary = tempfile.mkstemp(prefix='.home-', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return backup


def main():
    with (ROOT / 'config' / 'follow.yaml').open() as f:
        workspace = yaml.safe_load(f)['workspace']
    arm = RosCalibrationArm()
    try:
        samples = []
        for _ in range(7):
            samples.append(arm.get_current_pos())
            time.sleep(0.25)
        q, p = validate(samples, arm.q_min, arm.q_max,
                        lambda value: robot_fk(arm, value)[:3, 3], workspace)
        print('HOME 关节角（度）:', np.rad2deg(q).round(2).tolist(), flush=True)
        print('法兰位置（米）:', p.round(4).tolist(), flush=True)
        backup = save(ROOT / 'config' / 'home_pose.json', q, p)
        print('已保存 HOME:', ROOT / 'config' / 'home_pose.json', flush=True)
        if backup:
            print('原 HOME 备份:', backup, flush=True)
    finally:
        arm.stop()

if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('保存 HOME 失败:', exc, file=sys.stderr)
        sys.exit(1)
