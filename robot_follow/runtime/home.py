"""Move an initialized arm to its verified HOME pose."""
from dataclasses import dataclass
from typing import Callable
import time

import numpy as np

from robot_follow.control.feedback import near_limit_j2_floor, verify_home_joint_position
from robot_follow.hardware.interfaces import RobotControl


@dataclass(frozen=True)
class HomeOptions:
    allow_limit_recovery: bool
    j2_command_offset_deg: float
    j34_tolerance_deg: float
    speed_deg_s: float
    follow_enabled: bool


@dataclass(frozen=True)
class HomeResult:
    near_limit: bool
    desired_rotation: np.ndarray | None


def initialize_home(robot: RobotControl, home: np.ndarray, options: HomeOptions,
                    validate_recovery: Callable, tcp_kinematics: Callable) -> HomeResult:
    """Verify feedback, move to HOME and establish the fixed orientation."""
    current = robot.get_current_pos()
    command = robot.get_command_pos()
    if np.max(np.abs(current - command)) > np.deg2rad(0.5):
        raise RuntimeError('HOME 预检时，机械臂实际位置与命令不一致')
    near_limit = validate_recovery(current, home, robot.q_min, robot.q_max,
                                   enabled=options.allow_limit_recovery)
    if near_limit:
        validate_recovery(command, home, robot.q_min, robot.q_max, enabled=True)
    compensated = near_limit and options.j2_command_offset_deg > 0
    command_home = home.copy()
    if compensated:
        command_home[1] += np.deg2rad(options.j2_command_offset_deg)
        validate_recovery(current, command_home, robot.q_min, robot.q_max, enabled=True)
        validate_recovery(command, command_home, robot.q_min, robot.q_max, enabled=True)
        print(f'[HOME] J2 指令补偿 +{options.j2_command_offset_deg:.2f}°；'
              '实际角度仍按示教 HOME 检查', flush=True)
    j34_tolerance = 3. if compensated else (2. if near_limit else options.j34_tolerance_deg)
    tolerance = np.array([2., .5 if near_limit else 2., j34_tolerance, j34_tolerance, 2., 2.])
    print(f'移动到初始位置: {home.tolist()}')
    if near_limit:
        print('[HOME] J2 在软限位外：仅从当前位置向限位内恢复，速度 4°/s', flush=True)
        _, _, jacobian = tcp_kinematics(robot, home)
        singular = np.linalg.svd(jacobian, compute_uv=False)
        print(f'[HOME] 最小雅可比奇异值 {singular[-1]:.5f}；'
              '近限位模式使用低关节速度', flush=True)
    robot.move_to_joints(command_home, speed_deg_s=4.0 if near_limit else options.speed_deg_s,
                         timeout=25.0 if near_limit else 15.0,
                         allow_limit_recovery=near_limit,
                         actual_tolerance_deg=tolerance,
                         require_exact_target=near_limit,
                         raise_on_timeout=True, report_progress=True,
                         actual_target_rad=home)
    time.sleep(0.5)
    actual_home = robot.get_current_pos()
    error = verify_home_joint_position(actual_home, home, tolerance)
    if near_limit and actual_home[1] < near_limit_j2_floor(home[1], robot.q_min[1]):
        raise RuntimeError('HOME 实际 J2 低于模型安全下界')
    print(f'[HOME] 实际关节误差（度）: {np.round(error, 2).tolist()}')
    if near_limit:
        robot.recovery_max_speed = np.deg2rad(2.0)
        robot.set_joint_speed_cap(4.0)

    desired_rotation = None
    if options.follow_enabled:
        fk_home = robot.forward_kinematics()
        desired_rotation = np.array(fk_home['rotation'])
        print(f"初始 TCP 位置: {np.array(fk_home['position']).round(3)} m")
    return HomeResult(near_limit=near_limit, desired_rotation=desired_rotation)
