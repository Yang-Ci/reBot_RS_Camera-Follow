"""Validate and store personally taught HOME poses without connecting hardware."""
from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def validate_home_samples(samples, stamps, now, lower, upper, forward_kinematics, workspace):
    """Return a stationary, fresh joint pose within configured model limits.

    This validates a pose only; it cannot establish that the path to it or the
    attached hardware are collision-free.
    """
    samples = np.asarray(samples, dtype=float)
    stamps = np.asarray(stamps, dtype=float)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if (samples.ndim != 2 or samples.shape[0] < 5 or samples.shape[1] != 6
            or not np.isfinite(samples).all() or stamps.shape != (samples.shape[0],)
            or not np.isfinite(stamps).all() or not math.isfinite(now)):
        raise ValueError('关节反馈不足或无效')
    if (np.any(np.diff(stamps) <= 0) or stamps[-1] > now
            or now - stamps[-1] > .3 or stamps[-1] - stamps[0] < .4
            or np.max(np.diff(stamps)) > .3):
        raise ValueError('关节反馈过期或采样间隔无效')
    if np.max(np.ptp(samples, axis=0)) > np.deg2rad(.3):
        raise ValueError('机械臂尚未稳定，请保持静止后再保存')
    return validate_home_pose(samples[-1], lower, upper, forward_kinematics, workspace)


def validate_home_pose(pose, lower, upper, forward_kinematics, workspace):
    """Validate joint and workspace bounds for a proposed HOME, without motion."""
    pose = np.asarray(pose, dtype=float)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if (lower.shape != (6,) or upper.shape != (6,)
            or not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or np.any(lower >= upper)):
        raise ValueError('关节限位无效')
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError('HOME 关节角无效')
    pose = pose.copy()
    if np.any(pose < lower) or np.any(pose > upper):
        raise ValueError('HOME 关节角超出软限位')
    position = np.asarray(forward_kinematics(pose), dtype=float)
    low = np.asarray(workspace['min'], dtype=float)
    high = np.asarray(workspace['max'], dtype=float)
    radius = float(workspace['max_radius'])
    if (position.shape != (3,) or low.shape != (3,) or high.shape != (3,)
            or not np.isfinite(position).all() or not np.isfinite(low).all()
            or not np.isfinite(high).all() or not math.isfinite(radius)
            or radius <= 0 or np.any(low >= high)):
        raise ValueError('工作空间或 HOME 位置无效')
    if (np.any(position < low) or np.any(position > high)
            or np.linalg.norm(position[:2]) > radius):
        raise ValueError('HOME 位置超出工作空间')
    return pose, position


def profile_path(directory, name):
    """Map a user-visible name to a stable filename without allowing paths."""
    if not isinstance(name, str):
        raise ValueError('请输入 HOME 名称')
    name = name.strip()
    if not name or len(name) > 40 or any(ord(char) < 32 for char in name):
        raise ValueError('HOME 名称须为 1 至 40 个可见字符')
    return Path(directory) / (hashlib.sha256(name.encode('utf-8')).hexdigest() + '.json')


def save_profile(directory, name, pose, position):
    """Atomically save a validated candidate; keep the active HOME unchanged."""
    target = profile_path(directory, name)
    pose, position = np.asarray(pose, dtype=float), np.asarray(position, dtype=float)
    if (pose.shape != (6,) or position.shape != (3,) or not np.isfinite(pose).all()
            or not np.isfinite(position).all()):
        raise ValueError('HOME 数据无效')
    target.parent.mkdir(parents=True, exist_ok=True)
    data = {'name': name.strip(), 'home_joint_pos': pose.tolist(),
            'tcp_position': position.tolist(), 'timestamp': datetime.now(timezone.utc).isoformat()}
    fd, temp_name = tempfile.mkstemp(prefix='.home-', suffix='.tmp', dir=target.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, target)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    return target
