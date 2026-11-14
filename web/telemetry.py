"""Read diagnostic telemetry for the local dashboard."""

import csv
import json
import math
import time


def read_latest_state(project, controller):
    try:
        files = list((project / 'logs').glob('servo_*.csv'))
        if not files:
            return {'online': False, 'reason': '等待诊断日志'}
        path = max(files, key=lambda p: p.stat().st_mtime)
        modified = path.stat().st_mtime
    except OSError:
        return {'online': False, 'reason': '诊断日志暂不可用'}
    managed = controller.status()
    if managed['running'] and managed['started_at'] and modified < managed['started_at']:
        return {'online': False, 'reason': '等待本次真机程序的诊断数据'}
    if not managed['running'] and managed['ended_at'] and modified <= managed['ended_at']:
        return {'online': False, 'reason': '真机程序已退出'}
    age = max(0, time.time() - modified)
    if age > 3:
        return {'online': False, 'reason': '诊断数据已停止更新', 'age_s': age}
    try:
        with path.open('rb') as f:
            header = f.readline().decode('utf-8')
            f.seek(max(0, path.stat().st_size - 32768))
            tail = f.read().decode('utf-8')
        lines = tail.splitlines()
        # Ignore any incomplete trailing record while the recorder is writing.
        if not tail.endswith('\n'):
            lines = lines[:-1]
        names = next(csv.reader([header]))
        for line in reversed(lines):
            values = next(csv.reader([line]))
            if len(values) == len(names):
                row = dict(zip(names, values))
                break
        else:
            return {'online': False, 'reason': '等待完整诊断数据'}
        return state_from_row(project, row, age)
    except (OSError, ValueError, KeyError, csv.Error, UnicodeError):
        return {'online': False, 'reason': '诊断数据暂不可用'}



def gripper_open_fraction(row):
    """Normalize the commanded angle using the active gripper configuration."""
    try:
        command, closed, opened = (float(row[k]) for k in
            ('gripper_cmd_deg', 'gripper_closed_deg', 'gripper_open_deg'))
        if not all(math.isfinite(x) for x in (command, closed, opened)) or closed == opened:
            return None
        return min(1.0, max(0.0, (command - closed) / (opened - closed)))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def state_from_row(project, row, age):
    """Translate either a live sample or a historical CSV row for the UI."""
    try:
        actual = row.get('fb_valid') == '1'
        # A valid sample can age out while awaiting delivery or polling.
        # Legacy logs may omit age/rate; preserve their previous interpretation.
        if actual and row.get('fb_age_ms') not in (None, ''):
            try:
                feedback_age = float(row['fb_age_ms']) / 1000.0 + age
                rate = float(row.get('fb_rate_hz', 0))
                if (not math.isfinite(feedback_age) or feedback_age < 0 or
                        not math.isfinite(rate) or rate <= 0 or
                        feedback_age > max(1.0, 3.0 / rate)):
                    actual = False
            except (TypeError, ValueError, OverflowError):
                actual = False
        command_valid = row.get('q_cmd_valid')
        following = row.get('following') == '1'
        phase = row.get('phase', '')
        gripper = gripper_open_fraction(row)
        # Older logs did not include q_cmd_valid; active control was their only
        # command-validity indication. An explicit zero must never be overridden.
        usable_command = command_valid == '1' or (command_valid in (None, '') and following)
        if not actual and not usable_command:
            return {'online': True, 'source': 'unavailable', 'joints': None, 'age_s': age, 'phase': phase,
                    'gripper_open_fraction': gripper,
                    'servo_failed': row.get('servo_failed') == '1',
                    'visible': row.get('visible') == '1', 'following': following,
                    'hold': row.get('hold_latch') == '1', 'error': row.get('error', '')}
        source = 'actual' if actual else 'command'
        prefix = 'q_act_' if actual else 'q_cmd_'
        joints = [float(row[prefix + str(i)]) for i in range(1, 7)]
        if not all(math.isfinite(q) for q in joints):
            raise ValueError('nonfinite joints')
        try:
            offsets = json.loads((project / 'config/joint_offset.json').read_text())['joint_offset_rad']
            if len(offsets) != 6 or not all(math.isfinite(float(x)) for x in offsets):
                raise ValueError('invalid offsets')
            model_joints = [q + float(offset) for q, offset in zip(joints, offsets)]
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            model_joints = None
            offsets = None
        # The diagnostics thread reads actual and commanded angles together;
        # carry subsequent command motion forward for display only. This is an
        # estimate, not a new measurement or an input to the robot controller.
        estimated_model_joints = None
        if actual and offsets is not None and row.get('q_cmd_valid') == '1':
            try:
                command = [float(row['q_cmd_' + str(i)]) for i in range(1, 7)]
                paired = [float(row['q_fb_cmd_' + str(i)]) for i in range(1, 7)]
                if all(math.isfinite(q) for q in command + paired):
                    estimated_model_joints = [q + c - p + float(offset)
                                              for q, c, p, offset in zip(joints, command, paired, offsets)]
            except (KeyError, TypeError, ValueError):
                pass
        return {'online': True, 'source': source, 'joints': joints, 'phase': phase,
                'gripper_open_fraction': gripper,
                'servo_failed': row.get('servo_failed') == '1',
                'model_joints': model_joints, 'age_s': age,
                'estimated_model_joints': estimated_model_joints,
                'visible': row.get('visible') == '1', 'following': row.get('following') == '1',
                'hold': row.get('hold_latch') == '1', 'error': row.get('error', ''),
                'feedback_age_ms': row.get('fb_age_ms', ''),
                'feedback_rate_hz': row.get('fb_rate_hz', '')}
    except (OSError, ValueError, KeyError, TypeError, UnicodeError):
        return {'online': False, 'reason': '诊断数据暂不可用'}
