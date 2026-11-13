#!/usr/bin/env python3
"""
reBot Arm B601-RS 机械臂适配层
替代原项目的 Panthera_lib.Panthera，对外暴露几乎相同的接口，
使得视觉伺服主程序（palm_follow_grasp.py）里的控制数学可以原样保留。

设计：
  ┌─ 运动下发：rebot_control（LAN-GER/rebot_control，MIT 位置控制 + 关节限速）
  │     原项目的 robot.Joint_Vel(关节速度) 在这里实现为：
  │     每个控制周期把关节速度积分成位置目标 q_target += dq*dt，
  │     再调 set_joint_angles()，由库内部 200Hz 平滑斜坡执行。
  │     效果等价于速度伺服，且天然带每关节速度上限保护。
  │
  ├─ 夹爪（J7）：通过 set_joint_angles 的第 7 个通道下发（单位：度），
  │     用 set_gripper_deg() 设置目标角。与 6 关节共用同一 CAN 主机，
  │     不存在第二个 motorbridge 连接争抢电机 7 的问题。
  │
  └─ 运动学/动力学：Pinocchio 直接加载 reBotArm_control_py 仓库里的 URDF
        （原项目 Panthera 也是这么做的，FK/雅可比/重力项全部对齐）

依赖：
    git clone https://github.com/LAN-GER/rebot_control.git
    pip install motorbridge pynput pyyaml pinocchio numpy
    URDF 来自 https://github.com/Seeed-Projects/reBotArm_control_py 的 urdf/ 目录

⚠️ 使用前先核对：
    1. 你机器上 rebot_control 的实际类名/方法名（不同版本可能有出入，
       本文件所有对机械臂 SDK 的调用都集中在 _ArmIO 类里，只需改这一个类）
    2. URDF 路径与关节名称（B601_RS_JOINT_NAMES）
    3. 单位：rebot_control 用【度】，Pinocchio 用【弧度】，本适配层对外一律用弧度

⚠️ 安全停机约定（2026-09 事故后修订）：
    stop() 必须走 rebot_control 的「缓慢回零 → 到位后失能」完整流程
    （return_to_zero=True, wait=True）。
    严禁在任何非零位/非重力平衡位姿下直接失能电机——
    屈肘姿态下失能 = 机械臂自由下落（已发生过真实事故）。
"""

import os
import sys
import time
import numpy as np

try:
    import pinocchio as pin
except ImportError:
    raise ImportError("未找到 Pinocchio；请使用已配置的 .venv-arm，或安装 pip 包 pin")

# ─────────────────────────────────────────────────────────────
# 需要你按实际环境修改的配置
# ─────────────────────────────────────────────────────────────
# rebot_control 仓库（git clone 使用，非 pip 包，需把仓库根目录加入 sys.path）
REBOT_CONTROL_PATH = os.path.expanduser(os.environ.get(
    "REBOT_CONTROL_PATH", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "vendor", "rebot_control")))

URDF_PATH = os.path.expanduser(os.environ.get(
    "REBOT_URDF_PATH", "~/reBotArm_control_py/urdf/RS/urdf/ReBot_Arm_RS.urdf"))

# B601 RS 的 6 个关节在 URDF 里的名称（按 1→6 顺序，已在 ReBot_Arm_RS.urdf 中核对）
B601_RS_JOINT_NAMES = ["joint1", "joint2", "joint3",
                       "joint4", "joint5", "joint6"]

# 软件关节限位（弧度）。rebot_control 明确声明【不含软件限位】，必须在这里加。
# 先用 URDF 里的 limit 再向内收 10° 作为保守值，跑顺后再放宽。
JOINT_LIMIT_MARGIN = np.deg2rad(10.0)


def clamp_recovery_target(current, proposed, lower, upper):
    """Do not snap joints outside soft limits; allow only inward recovery."""
    current = np.asarray(current, dtype=float)
    proposed = np.asarray(proposed, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if any(x.shape != (6,) or not np.all(np.isfinite(x)) for x in (current, proposed, lower, upper)):
        raise ValueError('关节目标或限位无效')
    if np.any(lower > upper):
        raise ValueError('关节限位无效')
    return np.clip(proposed, np.minimum(lower, current), np.maximum(upper, current))


def validate_home_target(target, lower, upper, *, enabled=False):
    """Check a candidate against model limits before enabling any motors."""
    target, lower, upper = (np.asarray(x, dtype=float) for x in (target, lower, upper))
    if any(x.shape != (6,) or not np.all(np.isfinite(x)) for x in (target, lower, upper)):
        raise ValueError('HOME 或关节反馈无效')
    outside = (target < lower) | (target > upper)
    if not np.any(outside):
        return False
    if not enabled:
        raise ValueError('HOME 超出软限位；需要显式启用向内恢复模式')
    if np.any(outside & (np.arange(6) != 1)) or target[1] >= lower[1]:
        raise ValueError('恢复模式仅支持 J2 低于下限的 HOME')
    # Preserve at least 4 degrees relative to the modelled hard limit at HOME.
    if target[1] < lower[1] - JOINT_LIMIT_MARGIN + np.deg2rad(4.0):
        raise ValueError('J2 HOME 离模型关节下限不足 4°')
    return True


def validate_home_recovery(current, target, lower, upper, *, enabled=False):
    """Validate an inward-only approach to the near-limit J2 HOME.

    This does not establish collision clearance or prove the model matches the
    physical arm. Other joints retain the existing ten-degree soft margin.
    """
    current = np.asarray(current, dtype=float)
    target, lower, upper = (np.asarray(x, dtype=float) for x in (target, lower, upper))
    if current.shape != (6,) or not np.all(np.isfinite(current)):
        raise ValueError('当前关节反馈无效')
    if not validate_home_target(target, lower, upper, enabled=enabled):
        return False
    if current[1] >= lower[1] or current[1] > target[1]:
        raise ValueError('近限位 HOME 仅允许从当前低限位外向内进入')
    return True


def load_soft_joint_limits(urdf_path=URDF_PATH, joint_names=B601_RS_JOINT_NAMES):
    """Load the same soft limits as RebotAdapter without opening CAN."""
    model = pin.buildModelFromUrdf(os.path.expanduser(urdf_path))
    lower = np.zeros(6)
    upper = np.zeros(6)
    for i, name in enumerate(joint_names):
        if not model.existJointName(name):
            raise ValueError(f'URDF 中缺少关节 {name}: {urdf_path}')
        qi = model.joints[model.getJointId(name)].idx_q
        lower[i] = model.lowerPositionLimit[qi] + JOINT_LIMIT_MARGIN
        upper[i] = model.upperPositionLimit[qi] - JOINT_LIMIT_MARGIN
    return lower, upper

# ── 关节零位修正（2026-09-30 手眼+零位联合标定实测）──────────────
# SDK 上报的关节角零位与 URDF 的关节零位存在固定小偏差
# （实测约 J2=0.75° J3=0.46° J4=-2.06° J5=-2.21°）。
# 不加修正时 FK 平移与实物系统性偏差 ~30%，手眼标定/视觉伺服全挂。
# 修正只作用于 FK/雅可比（_build_q_full），不影响下发给 SDK 的角度。
import json as _json
_JOINT_OFFSET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "config", "joint_offset.json")
if os.path.exists(_JOINT_OFFSET_FILE):
    with open(_JOINT_OFFSET_FILE) as _f:
        JOINT_OFFSET = np.array(_json.load(_f)["joint_offset_rad"],
                                dtype=np.float64)
    print(f"[RebotAdapter] 已加载关节零位修正 (度): "
          f"{np.rad2deg(JOINT_OFFSET).round(2).tolist()}")
else:
    JOINT_OFFSET = np.zeros(6)
    print("[RebotAdapter] ⚠️ 未找到 joint_offset.json，FK 将不带零位修正。"
          "请先核对适用于当前机械臂的零位数据")

# 每关节最大速度（弧度/秒）——双保险之一（控制层也有限幅）
# 左右跟随的速度上限 ≈ J1 限速 × 臂展，这是横向跟随的物理瓶颈。
# 最终版为 2.0；RS06 额定转速远高于此。实机已验证 2.0，不再继续上提；
# 若要提速需重新评估电机温度、整臂稳定性和工作空间安全。
# 关节速度上限 (rad/s)：用户实测 [1.5,1.5,1.5,2.0,2.0,2.0] 效果最佳。
# RS06/RS00 额定 ≈10.5 rad/s，硬件余量很大；瓶颈在视觉延迟而非电机。
# 用户实测定案：全 2.0 效果明显优于 [1.5,1.5,1.5,2.0,2.0,2.0]（J1-J3 不憋屈）。
MAX_JOINT_SPEED = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])

# ── 夹爪（J7）参数 ──────────────────────────────────────────
# J7 与 6 个关节共用同一条 CAN 总线、同一个 rebot_control 控制器，
# 夹爪目标角通过 set_joint_angles 的第 7 个通道下发（单位：度）。
# 捏合跟随需要夹爪有足够响应速度（RS00 参考 vlim≈3 rad/s ≈ 172°/s）。
GRIPPER_SPEED_DEG_S = 500.0                 # 夹爪最大速度（度/秒）
GRIPPER_SOFT_LIMIT_DEG = (-100.0, 400.0)    # 角度硬限幅（兜底保护；
                                            # 正常开合范围由 config/grasp.yaml 标定值决定，
                                            # 且应落在本范围内）

# 控制周期（位置目标积分步长）
CTRL_DT = 0.02   # 仅作 dt 钳制参照；实际积分用墙上时钟，主循环 100Hz 下行为不变


# ─────────────────────────────────────────────────────────────
# 底层 IO：所有与 rebot_control SDK 相关的调用只出现在这个类里
# ─────────────────────────────────────────────────────────────
def _stop_after_failed_start(stop):
    """Attempt safe shutdown while preserving both startup and stop failures."""
    try:
        stop()
    except Exception as exc:
        raise RuntimeError(f'初始化失败后回零失能也失败: {exc}') from exc


class _ArmIO:
    def __init__(self):
        # rebot_control 是源码仓库（无 pip 包），真实包名为 rebot，
        # 控制器类为 ReBotRSMITController（已在本地仓库核对：
        # set_joint_angles 收 7 个【度】(6关节+夹爪)、set_max_speeds 单位度/秒、
        # read_joint_angles 是同步 CAN 读、stop(return_to_zero, wait)）
        if REBOT_CONTROL_PATH not in sys.path:
            sys.path.insert(0, REBOT_CONTROL_PATH)
        from rebot import ReBotRSMITController
        self.arm = ReBotRSMITController()
        # enable_esc=False：SDK 的 Esc/SIGINT 拦截会在主线程内阻塞数秒
        # 执行回零（视觉窗口冻结"卡死"的根源），且与主程序的清理路径
        # 竞争（二次 Ctrl+C 半途失能电机、相机管道被二次关闭）。
        # 退出统一由 palm_follow_grasp 的 q/Esc/Ctrl+C 路径处理，
        # 最终同样调用本类的 stop()（回零→保持→失能），安全性不变。
        try:
            self.arm.start(enable_esc=False)        # start 内部自动 connect
            # 安全速度包络（度/秒）；第 7 项为夹爪
            self.arm.set_max_speeds(np.rad2deg(MAX_JOINT_SPEED).tolist()
                                    + [GRIPPER_SPEED_DEG_S])
        except BaseException:
            # start 可能已经连接或使能，初始化失败也走同一回零失能路径。
            _stop_after_failed_start(self.stop)
            raise
        # 夹爪（J7）目标角：初始化时读一次实际角度作为初始目标（防止使能瞬间
        # 夹爪猛动），之后由 set_gripper_deg() 更新。
        # 同步 CAN 读（read_joint_angles 是阻塞式 CAN 往返，官方建议 ≤30Hz）
        try:
            self.gripper_deg = float(self.arm.read_joint_angles()[6])
        except Exception:
            self.gripper_deg = 0.0

    def send_joint_positions_deg(self, q_deg_7):
        self.arm.set_joint_angles(q_deg_7)      # 7 个数：6 关节 + 夹爪保持

    def read_joint_positions_deg(self):
        return np.array(self.arm.read_joint_angles()[:6], dtype=np.float64)

    def stop(self):
        """
        安全停机：缓慢回零 → 确认到位 → 才失能电机。

        ★★★ 绝对禁止改回 return_to_zero=False ★★★
        return_to_zero=False 会在当前位姿（可能是屈肘抬起等需要力矩
        对抗重力的姿态）直接失能电机，导致机械臂自由下落。

        wait=True 同样不可省略：shutdown 是守护线程，不等它跑完主线程退出时，
        解释器会把「失能电机/关闭 CAN」杀在半路上。
        """
        self.arm.stop(return_to_zero=True, wait=True)


# ─────────────────────────────────────────────────────────────
# 对外适配器：接口对齐 Panthera
# ─────────────────────────────────────────────────────────────
class RebotAdapter:
    def __init__(self, urdf_path=URDF_PATH, joint_names=B601_RS_JOINT_NAMES):
        # Pinocchio 模型（运动学 + 重力项）
        self.model = pin.buildModelFromUrdf(os.path.expanduser(urdf_path))
        self.data = self.model.createData()
        self.joint_names = joint_names
        self.motor_count = 6

        # 关节限位（从 URDF 读取并内收 margin）
        self.q_min = np.zeros(6)
        self.q_max = np.zeros(6)
        for i, name in enumerate(joint_names):
            if not self.model.existJointName(name):
                raise ValueError(f"URDF 中缺少关节 {name}: {urdf_path}")
            jid = self.model.getJointId(name)
            qi = self.model.joints[jid].idx_q
            self.q_min[i] = self.model.lowerPositionLimit[qi] + JOINT_LIMIT_MARGIN
            self.q_max[i] = self.model.upperPositionLimit[qi] - JOINT_LIMIT_MARGIN

        # URDF 校验通过后才连接和使能机械臂。
        self.io = _ArmIO()

        # 速度伺服的状态：当前位置目标（初始 = 当前实际位置，防止使能瞬间跳动）
        try:
            q_now = self.get_current_pos()
            self._q_target = q_now.copy()
            self._last_cmd_time = time.time()
        except BaseException:
            _stop_after_failed_start(self.io.stop)
            raise

    # ── 状态读取 ─────────────────────────────────────────────
    def get_current_pos(self):
        """
        返回 6 关节实际角度（弧度）—— 对应 Panthera.get_current_pos()
        ⚠️ 这是 7 路串行同步 CAN 读（持有 io_lock，单次约数 ms~数十 ms），
        会挤占 200Hz MIT 下发线程。只允许低频使用（初始化、标定采样、
        到位等待），50Hz 伺服热循环里请用 get_command_pos()。
        """
        return np.deg2rad(self.io.read_joint_positions_deg())

    def get_paced_feedback(self, stop_event):
        """Read each joint separately so the MIT sender can run between reads.

        Each actual angle is paired with the corresponding command after that
        read. The sampler timestamps the *entire* sweep, including lock waits.
        No partial sweep is returned on failure or shutdown.
        """
        arm = self.io.arm
        actual, command = [], []
        if len(arm.motors) < 6:
            raise RuntimeError('关节反馈电机数量不足')
        for index in range(6):
            if stop_event.is_set():
                raise RuntimeError('关节反馈采样已停止')
            with arm._io_lock_guard(timeout=0.020):
                angle = arm.motors[index].robstride_get_param_f32(
                    0x7019, timeout_ms=50)
            actual.append(float(angle))
            command.append(np.deg2rad(arm.get_command_angles()[index]))
            # Yield the shared CAN lock between reads; interruption aborts the
            # entire snapshot instead of publishing partially updated angles.
            if index < 5 and stop_event.wait(0.003):
                raise RuntimeError('关节反馈采样已停止')
        return np.asarray(actual), np.asarray(command)

    def get_command_pos(self):
        """
        返回 6 关节"当前实际下发值"（弧度）：rebot_control 内部 200Hz 斜坡
        正在发送的角度，纯内存读取、无 CAN I/O，可在伺服热循环中每周期调用。
        伺服稳态下与实际位置差异很小；需要真值时低频调用 get_current_pos() 校准。
        """
        return np.deg2rad(np.array(self.io.arm.get_command_angles()[:6],
                                   dtype=np.float64))

    # ── 运动指令 ─────────────────────────────────────────────
    def joint_vel_cmd(self, dq_rad):
        """
        对应 Panthera.Joint_Vel(dq)：关节速度指令。
        实现方式：积分成位置目标后下发（50Hz）。
        dq_rad: 6 维关节速度（rad/s），None 或全零 = 保持
        """
        now = time.time()
        # dt 上限钳制到 3 个控制周期：hold()/move_to_joints() 之后或循环卡顿
        # 时若不钳制，积分项 dq*dt 会让目标位置跳变一大步（安全隐患）
        dt = min(max(now - self._last_cmd_time, 1e-3), 3 * CTRL_DT)
        self._last_cmd_time = now

        if dq_rad is not None:
            dq = np.clip(np.asarray(dq_rad, dtype=np.float64),
                         -MAX_JOINT_SPEED, MAX_JOINT_SPEED)
            joint_speed_cap = getattr(self, 'joint_speed_cap', None)
            if joint_speed_cap is not None:
                dq = np.clip(dq, -joint_speed_cap, joint_speed_cap)
            recovery_speed = getattr(self, 'recovery_max_speed', None)
            if recovery_speed is not None:
                outside = (self._q_target < self.q_min) | (self._q_target > self.q_max)
                dq = dq.copy()
                dq[outside] = np.clip(dq[outside], -recovery_speed, recovery_speed)
            proposed = self._q_target + dq * dt
            # If a joint starts outside the soft envelope, freeze outward
            # motion and allow only recovery; never snap it to the boundary.
            self._q_target = clamp_recovery_target(self._q_target, proposed,
                                                    self.q_min, self.q_max)

        q_deg = np.rad2deg(self._q_target).tolist()
        q_deg.append(self._gripper_keep_deg())     # 夹爪保持当前值
        self.io.send_joint_positions_deg(q_deg)

    def set_joint_speed_cap(self, speed_deg_s):
        """Limit both integration and SDK ramps for every arm joint."""
        speed_deg_s = float(speed_deg_s)
        if not np.isfinite(speed_deg_s) or not 0.0 < speed_deg_s <= 15.0:
            raise ValueError('关节安全速度限值无效')
        self.io.arm.set_max_speeds([speed_deg_s] * 6 + [GRIPPER_SPEED_DEG_S])
        self.joint_speed_cap = np.deg2rad(speed_deg_s)

    def hold(self):
        """保持当前位置（丢手/暂停时调用）。用内部下发值重锚定，无 CAN I/O。"""
        self._q_target = self.get_command_pos().copy()
        self._last_cmd_time = time.time()   # 重置积分时钟，防止恢复后 dt 过大
        q_deg = np.rad2deg(self._q_target).tolist()
        q_deg.append(self._gripper_keep_deg())
        self.io.send_joint_positions_deg(q_deg)

    def is_stopping(self):
        """SDK 信号或温度保护触发停机后，不再发送普通控制命令。"""
        return bool(getattr(self.io.arm, 'shutdown_started', False))

    def move_to_joints(self, q_rad, speed_deg_s=20.0, wait=True, timeout=15.0,
                       allow_limit_recovery=False, actual_tolerance_deg=2.0,
                       require_exact_target=False, raise_on_timeout=False,
                       report_progress=False, actual_target_rad=None):
        """关节目标；示教恢复模式允许超限关节保持原位或向限位内移动。"""
        if self.is_stopping():
            raise RuntimeError('控制器正在停机，拒绝关节运动')
        lower, upper = self.q_min, self.q_max
        if allow_limit_recovery:
            current = self.get_command_pos()
            lower = np.minimum(lower, current)
            upper = np.maximum(upper, current)
        if wait:
            tolerance = np.deg2rad(np.broadcast_to(np.asarray(actual_tolerance_deg, dtype=float), (6,)))
            if not np.all(np.isfinite(tolerance)) or np.any(tolerance <= 0):
                raise ValueError('到位容差无效')
            actual_target = np.asarray(q_rad if actual_target_rad is None else actual_target_rad, dtype=float)
            if actual_target.shape != (6,) or not np.all(np.isfinite(actual_target)):
                raise ValueError('实际到位目标无效')
        requested = np.asarray(q_rad, dtype=np.float64)
        target = np.clip(requested, lower, upper)
        if require_exact_target and not np.allclose(target, requested, rtol=0, atol=1e-10):
            raise ValueError('HOME 目标被限位裁剪，拒绝移动')
        self._q_target = target
        self.io.arm.set_max_speeds([speed_deg_s] * 6 + [GRIPPER_SPEED_DEG_S])
        q_deg = np.rad2deg(self._q_target).tolist()
        q_deg.append(self._gripper_keep_deg())
        self.io.send_joint_positions_deg(q_deg)
        if wait:
            t0 = time.time()
            next_report = t0
            arrived = False
            while time.time() - t0 < timeout:
                if self.is_stopping():
                    raise RuntimeError('控制器正在停机，中止等待关节到位')
                command = self.get_command_pos()
                report = report_progress and time.time() >= next_report
                actual = None
                if report or np.max(np.abs(command - self._q_target)) < np.deg2rad(1.0):
                    actual = self.get_current_pos()
                if report:
                    print(f'[HOME进度] {time.time()-t0:.1f}s '
                          f'指令误差(度)={np.round(np.rad2deg(command-self._q_target), 2).tolist()} '
                          f'实际误差(度)={np.round(np.rad2deg(actual-actual_target), 2).tolist()}', flush=True)
                    next_report = time.time() + 2.0
                if np.max(np.abs(command - self._q_target)) < np.deg2rad(1.0):
                    if np.all(np.abs(actual - actual_target) < tolerance):
                        arrived = True
                        break
                time.sleep(0.1)
            if not arrived and raise_on_timeout:
                command = self.get_command_pos()
                actual = self.get_current_pos()
                raise TimeoutError(
                    'HOME 等待超时；指令误差(度): '
                    f'{np.round(np.rad2deg(command-self._q_target), 2).tolist()}；'
                    '实际误差(度): '
                    f'{np.round(np.rad2deg(actual-actual_target), 2).tolist()}')
        # 非阻塞移动仍在进行，保留指定速度直到下一次动作设置新上限。
        if wait:
            self.io.arm.set_max_speeds(np.rad2deg(MAX_JOINT_SPEED).tolist()
                                       + [GRIPPER_SPEED_DEG_S])
        self._last_cmd_time = time.time()   # 重置积分时钟，防止之后 dt 过大

    def stop(self):
        self.io.stop()

    def _gripper_keep_deg(self):
        # 夹爪（J7）目标角：初始化时读取并缓存，之后由 set_gripper_deg() 更新。
        # 用缓存值随每个控制周期下发，避免每周期多发一次同步 CAN 读。
        return self.io.gripper_deg

    # ── 夹爪（J7）控制 ─────────────────────────────────────
    def set_gripper_deg(self, deg):
        """设定夹爪（J7）目标角度（度），随下一次 50Hz 指令下发生效。

        带硬限幅兜底。正常开合范围请先运行 gripper_j7_calibrate.py
        实测标定（结果写入 config/grasp.yaml），不要凭猜填写。
        """
        lo, hi = GRIPPER_SOFT_LIMIT_DEG
        self.io.gripper_deg = float(np.clip(deg, lo, hi))

    def get_gripper_deg(self):
        """当前夹爪目标角度（度）"""
        return float(self.io.gripper_deg)

    # ── 运动学（Pinocchio，单位弧度/米）────────────────────
    def _build_q_full(self, joint_positions):
        """SDK 关节角 → pinocchio q_full。★ 加 JOINT_OFFSET 零位修正 ★"""
        q_full = np.zeros(self.model.nq)
        for i, name in enumerate(self.joint_names):
            jid = self.model.getJointId(name)
            q_full[self.model.joints[jid].idx_q] = \
                joint_positions[i] + JOINT_OFFSET[i]
        return q_full

    def tcp_pose_and_jacobian(self, joint_positions, tool_offset):
        """Return TCP pose and world-aligned Jacobian for the requested joints."""
        from robot_follow.hardware.kinematics import tcp_pose_and_jacobian
        return tcp_pose_and_jacobian(self, joint_positions, tool_offset)

    def forward_kinematics(self):
        """对应 Panthera.forward_kinematics() → {'position','rotation'}"""
        q = self._build_q_full(self.get_current_pos())
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        jid = self.model.getJointId(self.joint_names[-1])
        oMi = self.data.oMi[jid]
        return {'position': oMi.translation.copy(),
                'rotation': oMi.rotation.copy()}

    def gravity_torque(self, q_rad):
        """对应 Panthera.get_Gravity(q)：广义重力项（标定拖拽模式用）"""
        q_full = self._build_q_full(q_rad)
        g_full = pin.computeGeneralizedGravity(self.model, self.data, q_full)
        cols = [self.model.joints[self.model.getJointId(n)].idx_v
                for n in self.joint_names]
        return g_full[cols]
