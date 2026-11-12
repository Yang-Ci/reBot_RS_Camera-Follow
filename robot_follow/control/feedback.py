"""HOME verification and feedback correction."""

import numpy as np


def verify_home_joint_position(actual, target, tolerance_deg=2.0):
    actual = np.asarray(actual, dtype=float)
    target = np.asarray(target, dtype=float)
    if actual.shape != (6,) or target.shape != (6,) or not np.all(np.isfinite(actual)):
        raise RuntimeError('HOME 关节反馈无效')
    error_deg = np.rad2deg(actual - target)
    tolerance = np.broadcast_to(np.asarray(tolerance_deg, dtype=float), (6,))
    if not np.all(np.isfinite(tolerance)) or np.any(tolerance <= 0):
        raise ValueError('HOME 到位容差无效')
    if np.any(np.abs(error_deg) > tolerance):
        raise RuntimeError(f'未到达 HOME，关节误差(度): {np.round(error_deg, 2).tolist()}')
    return error_deg


def near_limit_j2_floor(home_q2, soft_lower_q2):
    # Soft lower is model lower + 10°; preserve 4° model margin.
    return max(home_q2 - np.deg2rad(0.5), soft_lower_q2 - np.deg2rad(6.0))


def near_limit_feedback_error(snapshot, now, home_q2, soft_lower_q2):
    """Require recent J2 feedback while the special HOME recovery is active."""
    if snapshot is None or not np.isfinite(snapshot.get('time', np.nan)):
        return 'J2 实际反馈缺失或过期'
    try:
        read_ms = float(snapshot.get('read_ms', 0.0))
        if not np.isfinite(read_ms) or read_ms < 0 or now - snapshot['time'] + read_ms / 1000.0 > 1.5:
            return 'J2 实际反馈缺失或过期'
        actuals = np.asarray(snapshot['actual'], dtype=float)
        commands = np.asarray(snapshot['command'], dtype=float)
        if actuals.shape != (6,) or commands.shape != (6,) or not np.all(np.isfinite(actuals)) or not np.all(np.isfinite(commands)):
            return '关节实际反馈无效'
        actual, command = actuals[1], commands[1]
    except (KeyError, IndexError, TypeError, ValueError):
        return 'J2 实际反馈无效'
    if actual < near_limit_j2_floor(home_q2, soft_lower_q2):
        return 'J2 实际角度低于 HOME 安全下界'
    if min(actual, command) < soft_lower_q2 + np.deg2rad(2.0) and abs(actual-command) > np.deg2rad(1.75):
        return 'J2 实际角度与命令偏差过大'
    if np.any(np.abs(actuals-commands) > np.deg2rad(3.5)):
        return '实际关节角度与命令偏差过大'
    return ''


def feedback_corrected_joints(command, snapshot):
    """Estimate current position using the last paired actual/command sample."""
    return (np.asarray(command, dtype=float) + np.asarray(snapshot['actual'], dtype=float)
            - np.asarray(snapshot['command'], dtype=float))
