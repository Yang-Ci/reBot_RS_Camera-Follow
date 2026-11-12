"""Safety actions shared by robot control runtimes."""

import numpy as np


def _pause_workspace(robot, lock, clear_following, status, tcp_pos):
    with lock:
        clear_following()
        status['active'] = False
        status['joint_speed'] = 0.0
        status['error'] = '超出工作区，已保持并暂停'
    robot.hold()
    print(f'\n[安全] TCP 越过硬墙 {np.asarray(tcp_pos).round(2).tolist()}，'
          '保持并暂停跟随', flush=True)


def pause_outside_workspace(robot, lock, following, status, tcp_pos):
    """Compatibility entry point accepting the legacy one-element list."""
    _pause_workspace(robot, lock, lambda: following.__setitem__(0, False), status, tcp_pos)


def pause_runtime_workspace(robot, lock, control, status, tcp_pos):
    """Pause shared runtime state and hold the arm outside its workspace."""
    _pause_workspace(robot, lock, lambda: setattr(control, 'following', False), status, tcp_pos)


def apply_stationary_joint_hold(robot, enabled, locked, state, joint_velocity):
    """静止锁存时锚定当前指令位置，停止全部关节的后续速度指令。"""
    if not enabled or not locked:
        state[0] = False
        return joint_velocity
    if not state[0]:
        robot.hold()
        state[0] = True
    return np.zeros_like(joint_velocity)
