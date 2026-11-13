"""Visual-servo worker. Hardware and runtime dependencies are supplied explicitly."""

from __future__ import annotations

import time
from dataclasses import dataclass, fields
from queue import SimpleQueue
from threading import Event
from typing import TYPE_CHECKING, Any, Callable

import numpy as np

from robot_follow.control.feedback import near_limit_feedback_error, feedback_corrected_joints
from robot_follow.control.table_safety import feedback_error, limit_downward_velocity
from robot_follow.control.state import ServoState
from robot_follow.runtime.observation import TargetObservation
from robot_follow.runtime.control import RunControl
from robot_follow.runtime.telemetry import LiveTelemetryPublisher
from robot_follow.hardware.interfaces import RobotControl
from robot_follow.control.target_filter import KalmanTarget3D
from robot_follow.control.kinematics import (compute_damped_pseudoinverse, compute_manipulability,
    compute_orientation_error, camera_velocity_to_tcp)
from robot_follow.runtime.diagnostics import feedback_age_ms
from robot_follow.control.timing import (scaled_decay, compensate_palm_velocity, measurement_age,
    dynamic_lead_seconds, camera_point_to_world, world_point_to_camera,
    adaptive_feedforward_scale, scaled_alpha, horizontal_radial_soft_wall,
    camera_twist_from_tcp, AxisProcessNoiseGate, FastStartGate, SlowFollowGate,
    AdaptiveLeadGate, StaticVelocityHold)

if TYPE_CHECKING:
    from robot_follow.runtime.diagnostics import ActualFeedbackSampler, DiagnosticRecorder


@dataclass
class ServoContext:
    """Services and mutable state used by a single servo worker."""
    stop_ctrl: Event
    web_control: bool
    web_commands: SimpleQueue
    lock: Any
    control: RunControl
    follow_enabled: bool
    robot: RobotControl
    servo_state: ServoState
    observation: TargetObservation
    near_limit_home: bool
    feedback_sampler: ActualFeedbackSampler | None
    home_joint_pos: np.ndarray
    control_status: dict[str, Any]
    axis_q_gate: AxisProcessNoiseGate
    kf: KalmanTarget3D
    fast_start: FastStartGate
    slow_follow: SlowFollowGate
    desired_depth: float
    lead_gate: AdaptiveLeadGate
    t_tcp_camera: np.ndarray
    lambda_gain_xyz: np.ndarray
    desired_rotation: np.ndarray
    max_vel_cart: float
    static_vel_hold: StaticVelocityHold
    stationary_joint_hold: bool
    stationary_joint_hold_active: list[bool]
    diag_recorder: DiagnosticRecorder | None
    live_publisher: LiveTelemetryPublisher | None
    feedback_hz: float
    compute_tcp_fk_and_jacobian: Callable[..., Any]
    pause_outside_workspace: Callable[..., Any]
    apply_stationary_joint_hold: Callable[..., Any]
    table_safety_enabled: bool = False


@dataclass(frozen=True)
class ServoSettings:
    """Validated control values captured when the worker starts."""
    ctrl_rate: float
    target_stale_s: float
    kf_enable: bool
    kf_axis_adapt_enable: bool
    kf_static_q_scale: float
    kf_adapt_decay: float
    kf_resync_frames: int
    kf_axis_adapt_static_guard: bool
    kf_adapt_enable: bool
    kf_adapt_ref: float
    kf_adapt_max: float
    ff_kf_deadband_off: float
    ff_kf_deadband_on: float
    kf_ff_exit_frames: int
    ff_max_vel: float
    coast_max_vel: float
    coast_frames: int
    reach_soft_start: float
    coast_decay: float
    kf_dynamic_lead: bool
    kf_lead_ms: float
    kf_lead_max_ms: float
    kf_lead_adaptive: bool
    lost_hold_frames: int
    workspace_min: np.ndarray
    workspace_max: np.ndarray
    workspace_radius: float
    kf_slow_follow_enable: bool
    kf_slow_follow_release: float
    kf_slow_follow_speed: float
    position_deadband: float
    deadband_hysteresis: float
    servo_direction: float
    ff_taper_dist: float
    vel_feedforward: float
    ff_smooth: float
    ff_min_speed: float
    ff_adaptive_taper_enable: bool
    ff_taper_speed_ref: float
    ff_taper_max_dist: float
    reach_dock_dist: float
    reach_min_scale: float
    reach_soft_end: float
    reach_att_tau: float
    orientation_deadband: float
    k_orientation: float
    max_vel_angular: float
    reversal_fast_response: bool
    reversal_residual_scale: float
    smooth_alpha: float
    max_accel: float
    reversal_accel_boost: float
    static_vel_hold_enable: bool
    damping_adaptive: bool
    manipulability_threshold: float
    damping_base: float
    min_manipulability_speed_scale: float
    max_vel_joint: float

    @classmethod
    def from_configuration(cls, config):
        """Capture the validated follow settings used by one worker."""
        values = {item.name: getattr(config, item.name) for item in fields(cls)}
        # Arrays are copied so later configuration changes cannot move the walls.
        for key in ('workspace_min', 'workspace_max'):
            values[key] = np.array(values[key], copy=True)
        return cls(**values)


def run_servo_loop(context: ServoContext, settings: ServoSettings):
    """Keep a fatal control failure visible until the operator shuts down."""
    try:
        _run_servo_loop(context, settings)
    except Exception as exc:
        reason = f'伺服异常，已暂停: {exc}'
        with context.lock:
            context.control.following = False
            context.control.gesture_start_allowed = False
            context.control.servo_failed = True
            context.control_status.update(active=False, joint_speed=0.0, error=reason)
        try:
            context.robot.hold()
        except Exception as hold_error:
            reason += f'；保持失败，请使用硬件急停: {hold_error}'
            with context.lock:
                context.control_status['error'] = reason
        print(f'\n[安全] {reason}', flush=True)
        # Do not silently restart or initiate an unverified return path. Keep
        # reporting the fault until the operator requests shutdown.
        while not context.stop_ctrl.is_set():
            if context.live_publisher is not None:
                context.live_publisher.offer({
                    'host_time': time.monotonic(), 'following': 0, 'visible': 0,
                    'hold_latch': 0, 'fb_valid': 0, 'q_cmd_valid': 0,
                    'servo_failed': 1, 'error': reason})
            context.stop_ctrl.wait(.5)


def _run_servo_loop(context: ServoContext, settings: ServoSettings):
    next_deadline = time.monotonic()
    last_loop_time = None
    _zero3 = np.zeros(3)
    while not context.stop_ctrl.is_set():
        while not context.web_commands.empty():
            requested = context.web_commands.get_nowait()
            with context.lock:
                if requested == 'gesture':
                    if context.control.gesture_start_allowed and context.follow_enabled:
                        context.control.following = True
                        context.control.gesture_start_allowed = False
                else:
                    context.control.following = bool(requested) and context.follow_enabled
                    context.control.gesture_start_allowed = False
        if context.robot.is_stopping():
            with context.lock:
                context.control.following = False
                context.control.quit_requested.set()
            print('[安全] 控制器正在停机，退出跟随', flush=True)
            break
        next_deadline += settings.ctrl_rate
        t0 = time.monotonic()
        dt = (settings.ctrl_rate if last_loop_time is None else
              float(np.clip(t0 - last_loop_time, 1e-3, 0.06)))
        last_loop_time = t0
        q_cmd_log = None
        context.servo_state.static_hold_now = False
        compensated_speed_log = None

        with context.lock:
            following = context.control.following
            target_cam = (None if context.observation.position_cam is None
                          else np.array(context.observation.position_cam, dtype=np.float64))
            frozen_meas = context.observation.frozen
            visible = context.observation.visible
            obs_seq = context.observation.sequence
            obs_time = context.observation.observed_at
            arrival_time = context.observation.arrived_at
            pub_time = context.observation.published_at
            hw_time_used = context.observation.hardware_time

        if following and context.near_limit_home:
            snapshot = context.feedback_sampler.snapshot() if context.feedback_sampler is not None else None
            fault = near_limit_feedback_error(snapshot, t0, context.home_joint_pos[1], context.robot.q_min[1])
            if fault:
                with context.lock:
                    context.control.following = False
                    context.control_status['error'] = fault + '，跟随已暂停'
                print(f'\n[安全] {fault}，跟随已暂停', flush=True)
                following = False

        # 控制侧独立看门狗：即使视觉线程卡在检测/后处理，也不能让
        # visible 永久停留在 True。保留最后目标供原有有界 coast 使用。
        if (visible and obs_time > 0.0
                and t0 - obs_time > settings.target_stale_s):
            visible = False

        # Keep the original observation-loss/coasting behavior while still
        # requiring fresh actual-joint feedback for tabletop protection.
        table_feedback = None
        if following and context.table_safety_enabled:
            table_feedback = (context.feedback_sampler.snapshot()
                              if context.feedback_sampler is not None else None)
            reason = feedback_error(table_feedback, time.monotonic())
            if (not reason and target_cam is not None and
                    (target_cam.shape != (3,) or not np.all(np.isfinite(target_cam)))):
                reason = '手掌目标无效，已暂停'
            if reason:
                with context.lock:
                    context.control.following = False
                    context.control.gesture_start_allowed = False
                    context.control_status['error'] = reason
                    context.control_status['active'] = False
                print(f'\n[安全] {reason}', flush=True)
                following = False

        if not visible:
            context.servo_state.lost_counter += 1
        else:
            context.servo_state.lost_counter = 0

        # ── 时间戳卡尔曼：状态保存在最近观测时刻 ─────────────────
        # 新观测按真实观测间隔 update；重复控制周期不再反复修改后验状态。
        # 使用时再按 now-kf_time 外推到现在，避免异步观测的时间错位。
        new_kf_meas = bool(visible and target_cam is not None
                           and not frozen_meas and obs_seq != context.servo_state.kf_last_seq)
        lead_s = 0.0
        if settings.kf_enable:
            context.servo_state.kf_rejected_now = False
            if not visible or frozen_meas:
                context.servo_state.slow_active = False
            if settings.kf_axis_adapt_enable:
                if not visible or frozen_meas:
                    context.axis_q_gate.reset()
                else:
                    context.axis_q_gate.relax(dt)
                context.kf.q_scale = context.axis_q_gate.scales.copy()
                if not context.servo_state.ff_moving and settings.kf_static_q_scale < 1.0:
                    context.kf.q_scale = np.full(3, settings.kf_static_q_scale)
            else:
                context.kf.q_scale = max(1.0, context.kf.q_scale * scaled_decay(
                    settings.kf_adapt_decay, dt, settings.ctrl_rate))
            if new_kf_meas:
                meas_time = obs_time if obs_time > 0.0 else t0
                if context.kf.initialized:
                    # 异常/乱序时间戳不允许滤波器时间倒退。
                    meas_time = max(meas_time, context.servo_state.kf_time + 1e-3)
                if not context.kf.initialized:
                    context.kf.update(target_cam, 0.0)
                    context.servo_state.kf_time = meas_time
                    context.servo_state.kf_last_seq = obs_seq
                    context.servo_state.kf_reject_count = 0
                else:
                    dt_meas = float(np.clip(meas_time - context.servo_state.kf_time, 1e-3, 0.50))
                    predicted = context.kf.position() + context.kf.velocity() * dt_meas
                    innov = float(np.linalg.norm(target_cam - predicted))
                    gate = (0.06 + 2.0 * float(np.linalg.norm(context.kf.velocity()))
                            * dt_meas)
                    if innov > gate:
                        if settings.kf_axis_adapt_enable:
                            context.axis_q_gate.reset()
                            context.kf.q_scale = context.axis_q_gate.scales.copy()
                        context.servo_state.kf_reject_count += 1
                        context.servo_state.kf_rejected_now = True
                        if context.servo_state.kf_reject_count >= settings.kf_resync_frames:
                            context.kf.reset()
                            context.kf.update(target_cam, 0.0)
                            context.servo_state.kf_time = meas_time
                            context.servo_state.kf_reject_count = 0
                            context.servo_state.hold_latch = None
                            context.servo_state.hold_world = None
                            context.fast_start.reset()
                            context.slow_follow.reset()
                            context.servo_state.slow_active = False
                            with context.lock:
                                context.control_status["error"] = "KF重同步"
                        else:
                            context.kf.predict(dt_meas)
                            context.servo_state.kf_time = meas_time
                    else:
                        if settings.kf_axis_adapt_enable:
                            # v9 会先放大 Q、随后才复位，静止噪声可能因此被
                            # 放大并误解锁。静止观测直接用基准 Q；运动确认后
                            # 原分轴加速增强不变，false 可独立复现 v9 顺序。
                            q_valid = (context.servo_state.ff_moving
                                       or not settings.kf_axis_adapt_static_guard)
                            context.kf.q_scale = context.axis_q_gate.update(
                                target_cam - predicted, dt, valid=q_valid)
                        elif settings.kf_adapt_enable:
                            context.kf.q_scale = float(np.clip(
                                innov / settings.kf_adapt_ref, 1.0, settings.kf_adapt_max))
                        context.kf.update(target_cam, dt_meas)
                        context.servo_state.kf_time = meas_time
                        context.servo_state.kf_reject_count = 0
                    context.servo_state.kf_last_seq = obs_seq

            _kv_raw = context.kf.velocity() if context.kf.initialized else np.zeros(3)
            if frozen_meas:
                # Original behavior: preserve lateral tracking, suppress stale depth velocity.
                _kv_raw[2] = 0.0
            # 相机装在手腕上：KF 看到的是“手掌相对相机”的速度，
            # 机械臂自身的运动也会让静止手掌在画面里移动。静止判定
            # 必须使用扣除相机自运动后的手掌速度，否则停手后会形成
            # “锁存→机械臂动→KF 看到假速度→解锁”的极限环。
            # 运动段 FF/lead 保持原相机系速度（实机验证过的稳定手感）；
            # 补偿速度只用于静止判定，避免自运动反馈进前馈/lead。
            _kv = compensate_palm_velocity(
                _kv_raw, context.servo_state.prev_camera_twist, context.kf.position()) \
                if context.kf.initialized else _kv_raw.copy()
            if frozen_meas:
                _kv[2] = 0.0
            _kvn = float(np.linalg.norm(_kv))
            compensated_speed_log = _kvn
            _was_moving = context.servo_state.ff_moving
            if context.servo_state.ff_moving:
                if _kvn < settings.ff_kf_deadband_off:
                    context.servo_state.ff_moving = False
                    context.servo_state.ff_exit_count = 0
            elif _kvn > settings.ff_kf_deadband_on:
                context.servo_state.ff_exit_count += 1
                if context.servo_state.ff_exit_count >= settings.kf_ff_exit_frames:
                    context.servo_state.ff_moving = True
                    context.servo_state.ff_exit_count = 0
            else:
                context.servo_state.ff_exit_count = 0

            # 机械臂已在死区内连续多周期零速，但 KF 残余速度仍把
            # ff_moving 拉在 True 时，强制进入静止。否则残余目标漂移
            # 会每隔几周期推一次关节，表现为上下“点动”。
            if (context.servo_state.ff_moving and context.servo_state.in_deadband
                    and context.servo_state.last_joint_speed < 0.03):
                context.servo_state.settled_frames += 1
                if context.servo_state.settled_frames >= 8:
                    context.servo_state.ff_moving = False
                    context.servo_state.ff_exit_count = 0
                    context.servo_state.settled_frames = 0
            else:
                context.servo_state.settled_frames = 0

            # 静止确认后立即回到 v8 基准 Q，不让加速段倍率残留到停靠。
            if settings.kf_axis_adapt_enable and not context.servo_state.ff_moving:
                context.axis_q_gate.reset()
                context.kf.q_scale = (np.full(3, settings.kf_static_q_scale)
                              if settings.kf_static_q_scale < 1.0
                              else context.axis_q_gate.scales.copy())

            # 额外快速通道：连续新三维观测同向位移，解除静止锁存。
            # 原速度确认通道保持不变，静止进入条件也不变。
            # 必须同时存在补偿后的真实手掌速度，避免纯相机自运动
            # 通过原始像素位移误触发快速通道。
            if (not context.servo_state.ff_moving and context.servo_state.hold_latch is not None
                    and new_kf_meas and not context.servo_state.kf_rejected_now
                    and _kvn > settings.ff_kf_deadband_off
                    and context.fast_start.update(target_cam)):
                context.servo_state.ff_moving = True
                context.servo_state.hold_latch = None
                context.servo_state.ff_exit_count = 0
                context.fast_start.reset()
                with context.lock:
                    context.control_status["fast_start"] = True

            if context.servo_state.ff_moving != _was_moving:
                with context.lock:
                    context.control_status["ffm"] = context.servo_state.ff_moving

            if context.servo_state.ff_moving:
                context.servo_state.hold_latch = None
                context.servo_state.hold_world = None
                context.fast_start.reset()
                context.slow_follow.reset()
                context.servo_state.slow_active = False
            elif context.kf.initialized:
                if context.servo_state.hold_latch is None:
                    # hold_latch 是静止状态标记；世界系目标 hold_world
                    # 在控制线程拿到 FK 位姿后建立（见控制段）。
                    context.servo_state.hold_latch = context.kf.position().copy()
                    context.servo_state.hold_world = None
                    context.fast_start.reset(target_cam if target_cam is not None
                                     else context.kf.position())
                    context.slow_follow.reset()
                    context.servo_state.slow_active = False
                elif context.servo_state.kf_rejected_now:
                    context.slow_follow.reset()
                    context.servo_state.slow_active = False
            # 静止判定用补偿速度；正常运动段的前馈仍沿用原相机系
            # KF 速度，保持已经实机验证过的巡航/高速手感不变。
            # 静止时 ff_moving=False 会把前馈整体归零，不会把自运动
            # 注入停手瞬间。
            context.servo_state.target_vel_f = (np.clip(_kv_raw, -settings.ff_max_vel, settings.ff_max_vel)
                               if context.servo_state.ff_moving else np.zeros(3))

        # ── 丢失外推：全部按实际 dt 积分/衰减 ─────────────────────
        if visible:
            context.servo_state.coast_offset[:] = 0.0
            cv = context.servo_state.target_vel_f.copy()
            cvn = float(np.linalg.norm(cv))
            if cvn > settings.coast_max_vel:
                cv *= settings.coast_max_vel / cvn
            context.servo_state.coast_vel = cv
            context.servo_state.coast_err = float(np.linalg.norm(
                target_cam - np.array([0.0, 0.0, context.desired_depth]))) \
                if target_cam is not None else 1e9
        elif (target_cam is not None and 0 < context.servo_state.lost_counter <= settings.coast_frames
                and context.servo_state.coast_err < 0.15
                and not context.servo_state.near_wall and context.servo_state.reach_r_now <= settings.reach_soft_start):
            context.servo_state.coast_offset += context.servo_state.coast_vel * dt
            context.servo_state.coast_vel *= scaled_decay(settings.coast_decay, dt, settings.ctrl_rate)
        if target_cam is not None and context.servo_state.lost_counter > 0:
            target_cam = target_cam + context.servo_state.coast_offset

        # KF 位置先按观测年龄外推到现在；额外 lead 单独加权。
        # 年龄补偿始终保留；静止时只撤销额外 lead 和前馈。
        lead_raw_s = 0.0
        lead_error_now = 0.0
        kf_speed_now = (float(np.linalg.norm(_kv_raw))
                        if settings.kf_enable and context.kf.initialized else 0.0)
        context.servo_state.lead_weight_now = 0.0
        context.servo_state.lead_target_weight = 0.0
        if (not settings.kf_enable or not visible or frozen_meas
                or context.servo_state.kf_rejected_now):
            context.lead_gate.reset()
        if (settings.kf_enable and visible and context.kf.initialized
                and not context.servo_state.kf_rejected_now):
            if not frozen_meas:
                # 硬件成像时刻包含传输段；lead 的处理项只计算主机处理，
                # 传输延迟已经包含在 age_s 中，不能重复补偿。
                processing_s = max(0.0, pub_time - arrival_time)
                age_s = measurement_age(t0, context.servo_state.kf_time, max_age=0.25)
                # 年龄外推必须用原始相机系速度：静止手掌在机械臂运动时
                # 的视位置变化，正是“现在”应有的观测位置。
                now_target = context.kf.position() + _kv_raw * age_s
                lead_error_now = float(np.linalg.norm(
                    now_target - np.array([0.0, 0.0, context.desired_depth])))
                if context.servo_state.hold_latch is not None:
                    # 静止时只撤销前馈和额外 lead。这里先给实时滤波
                    # 目标作为退路；控制段会用世界系 hold_world 投影
                    # 覆盖它，避免位置环追相机系噪声。
                    context.lead_gate.reset()
                    lead_s = 0.0
                    target_cam = now_target
                else:
                    lead_raw_s = (dynamic_lead_seconds(
                        settings.kf_lead_ms, processing_s, settings.kf_lead_max_ms)
                        if settings.kf_dynamic_lead and settings.kf_lead_ms > 0.0
                        else settings.kf_lead_ms / 1000.0)
                    if settings.kf_lead_adaptive and lead_raw_s > 0.0:
                        context.servo_state.lead_weight_now, context.servo_state.lead_target_weight = context.lead_gate.update(
                            _kv_raw, now_target - np.array(
                                [0.0, 0.0, context.desired_depth]),
                            dt, context.servo_state.ff_moving, valid=True)
                    else:
                        context.lead_gate.reset()
                        # false 只用于复现旧控制逻辑；正常实机保持 true。
                        context.servo_state.lead_weight_now = (1.0 if lead_raw_s > 0.0 else 0.0)
                        context.servo_state.lead_target_weight = context.servo_state.lead_weight_now
                    lead_s = lead_raw_s * context.servo_state.lead_weight_now
                    target_cam = now_target + _kv_raw * lead_s

        if following and target_cam is not None and context.servo_state.lost_counter <= settings.lost_hold_frames:
            try:
                current_joints = context.robot.get_command_pos()
                q_cmd_log = current_joints.copy()
                kinematic_joints = (feedback_corrected_joints(current_joints, snapshot)
                                    if context.near_limit_home else current_joints)
                tcp_pos, tcp_rot, J_robot = context.compute_tcp_fk_and_jacobian(
                    context.robot, kinematic_joints)
                actual_tcp = tcp_pos
                if context.table_safety_enabled:
                    actual_tcp, _, _ = context.compute_tcp_fk_and_jacobian(
                        context.robot, table_feedback['actual'])
                    _, below_floor = limit_downward_velocity(0.0, tcp_pos[2], actual_tcp[2])
                    if below_floor:
                        with context.lock:
                            context.control.following = False
                            context.control.gesture_start_allowed = False
                            context.control_status['active'] = False
                            context.control_status['joint_speed'] = 0.0
                            context.control_status['error'] = '桌面安全边界触发，已暂停'
                        context.robot.hold()
                        print('\n[安全] 桌面安全边界触发，保持并暂停', flush=True)
                        continue
                with context.lock:
                    context.control_status["tcp"] = tcp_pos.round(3).tolist()

                tcp_radius = float(np.hypot(tcp_pos[0], tcp_pos[1]))
                if (np.any(tcp_pos < settings.workspace_min - 0.05)
                        or np.any(tcp_pos > settings.workspace_max + 0.05)
                        or tcp_radius > settings.workspace_radius + 0.05):
                    context.pause_outside_workspace(context.robot, context.lock, context.control, context.control_status, tcp_pos)
                    continue

                # ── 静止世界系目标 ─────────────────────────────
                # 静止时把掌心锁在世界系：位置环不再追相机系噪声或
                # 随机械臂自运动漂移的画面点。慢拖在 world_obs 上确认，
                # 只会以受限速度迁移 hold_world，跟手性不靠相机系噪声。
                if (context.t_tcp_camera is not None and context.servo_state.hold_latch is not None):
                    if context.servo_state.hold_world is None:
                        world_hold = camera_point_to_world(
                            target_cam, tcp_pos, tcp_rot, context.t_tcp_camera)
                        if np.linalg.norm(world_hold) > 1e-6:
                            context.servo_state.hold_world = world_hold
                            context.slow_follow.reset(context.servo_state.hold_world)
                            context.servo_state.slow_active = False
                    if context.servo_state.hold_world is not None:
                        if (new_kf_meas and not frozen_meas
                                and not context.servo_state.kf_rejected_now):
                            world_obs = camera_point_to_world(
                                target_cam, tcp_pos, tcp_rot,
                                context.t_tcp_camera)
                            if np.linalg.norm(world_obs) > 1e-6:
                                context.servo_state.slow_active = context.slow_follow.update(
                                    world_obs,
                                    enabled=(settings.kf_slow_follow_enable
                                             and not context.servo_state.in_deadband))
                                delta = world_obs - context.servo_state.hold_world
                                distance = float(np.linalg.norm(delta))
                                if (context.servo_state.slow_active
                                        and distance > settings.kf_slow_follow_release):
                                    step = min(
                                        distance - settings.kf_slow_follow_release,
                                        settings.kf_slow_follow_speed * dt)
                                    context.servo_state.hold_world = (context.servo_state.hold_world
                                                     + delta / distance * step)
                                # 不再使用“偏离 hold_tol 就 0.06m/s 拉回”
                                # 的旧兜底：它会放大量测噪声并造成静止
                                # 上下漂移。慢拖只由方向/帧确认的
                                # SlowFollowGate 迁移。
                        held_cam = world_point_to_camera(
                            context.servo_state.hold_world, tcp_pos, tcp_rot, context.t_tcp_camera)
                        if np.linalg.norm(held_cam) > 1e-6:
                            target_cam = held_cam

                target_desired = np.array([0.0, 0.0, context.desired_depth])
                pos_err_cam = target_cam - target_desired
                pos_err_norm = float(np.linalg.norm(pos_err_cam))
                if context.servo_state.in_deadband:
                    if pos_err_norm > settings.position_deadband * settings.deadband_hysteresis:
                        context.servo_state.in_deadband = False
                elif pos_err_norm < settings.position_deadband:
                    context.servo_state.in_deadband = True
                v_cam = np.zeros(6)
                if not context.servo_state.in_deadband:
                    v_cam[:3] = settings.servo_direction * context.lambda_gain_xyz * pos_err_cam

                context.servo_state.ff_taper_now = settings.ff_taper_dist
                context.servo_state.ff_approach_speed = 0.0
                context.servo_state.ff_scale_now = 0.0
                if settings.vel_feedforward > 0.0:
                    if not settings.kf_enable and context.servo_state.prev_target_cam is not None:
                        if np.linalg.norm(target_cam - context.servo_state.prev_target_cam) < 0.06:
                            tv = (target_cam - context.servo_state.prev_target_cam) / dt
                            tv = np.clip(tv, -settings.ff_max_vel, settings.ff_max_vel)
                            context.servo_state.target_vel_f = (settings.ff_smooth * tv
                                               + (1 - settings.ff_smooth) * context.servo_state.target_vel_f)
                    context.servo_state.prev_target_cam = target_cam.copy()
                    if context.servo_state.in_deadband or \
                            np.linalg.norm(context.servo_state.target_vel_f) < settings.ff_min_speed:
                        context.servo_state.target_vel_f = np.zeros(3)
                    else:
                        ff_scale, context.servo_state.ff_taper_now, context.servo_state.ff_approach_speed = \
                            adaptive_feedforward_scale(
                                pos_err_cam, context.servo_state.target_vel_f, settings.ff_taper_dist,
                                settings.ff_adaptive_taper_enable, settings.ff_taper_speed_ref,
                                settings.ff_taper_max_dist)
                        context.servo_state.ff_scale_now = ff_scale
                        v_cam[:3] += (settings.vel_feedforward * ff_scale
                                      * context.servo_state.target_vel_f)

                # 静止时保持当前机械臂位姿：不再追世界系掌心点做
                # “对中/补距离”运动。手掌停住后目标立即冻结，避免
                # 残余目标漂移或慢慢对中造成肉眼可见的小幅晃动。
                if context.servo_state.hold_latch is not None:
                    v_cam[:3] = 0.0

                ori_err = compute_orientation_error(tcp_rot, context.desired_rotation)
                v_cam[:3] = context.max_vel_cart * np.tanh(v_cam[:3] / context.max_vel_cart)

                reach_r = float(np.hypot(tcp_pos[0], tcp_pos[1]))
                context.servo_state.reach_r_now = reach_r
                reach_scale = 1.0
                att_target = 1.0
                if reach_r > settings.reach_soft_start:
                    dock = float(np.clip(
                        1.0 - pos_err_norm / settings.reach_dock_dist, 0.0, 1.0))
                    reach_scale = float(np.clip(
                        1.0 - (1.0 - settings.reach_min_scale)
                        * (reach_r - settings.reach_soft_start)
                        / max(settings.reach_soft_end - settings.reach_soft_start, 1e-6),
                        settings.reach_min_scale, 1.0))
                    att_target = 1.0 - (1.0 - reach_scale) * dock
                context.servo_state.reach_att += ((att_target - context.servo_state.reach_att)
                                 * min(1.0, dt / settings.reach_att_tau))
                v_cam[:3] *= context.servo_state.reach_att

                angular_world = (settings.k_orientation * ori_err
                                 if np.linalg.norm(ori_err) > settings.orientation_deadband
                                 else np.zeros(3))
                angular_world = np.clip(angular_world, -settings.max_vel_angular, settings.max_vel_angular)

                target_reversing = (np.dot(v_cam[:3], context.servo_state.prev_cmd_vel) < 0)
                if (settings.reversal_fast_response
                        and np.dot(v_cam[:3], context.servo_state.actual_velocity[:3]) < 0):
                    # 不再把旧速度直接清零（会先完全停住再反向，产生
                    # 换向顿挫）；保留少量旧方向惯性，让 EMA 平滑穿零。
                    context.servo_state.actual_velocity[:3] *= settings.reversal_residual_scale
                context.servo_state.actual_velocity += ((v_cam - context.servo_state.actual_velocity)
                                       * scaled_alpha(settings.smooth_alpha, dt, settings.ctrl_rate))

                dv_cmd = context.servo_state.actual_velocity[:3] - context.servo_state.prev_cmd_vel
                accel_cap = settings.max_accel
                if target_reversing or np.dot(
                        context.servo_state.actual_velocity[:3], context.servo_state.prev_cmd_vel) < 0:
                    accel_cap *= settings.reversal_accel_boost
                max_dv = accel_cap * dt
                dv_norm = float(np.linalg.norm(dv_cmd))
                if dv_norm > max_dv:
                    context.servo_state.actual_velocity[:3] = (context.servo_state.prev_cmd_vel
                                              + dv_cmd * (max_dv / dv_norm))
                # 死区内关节命令原本就会归零；这里进一步清除 EMA/加速度
                # 限制器中的低速残态，避免下一次死区边界波动带出旧速度。
                # 高速停靠不清零，仍由原有加速度限制器平顺衰减。
                context.servo_state.actual_velocity, context.servo_state.static_hold_now = context.static_vel_hold.update(
                    context.servo_state.actual_velocity,
                    stationary=(not context.servo_state.ff_moving
                                and context.servo_state.hold_latch is not None),
                    in_deadband=context.servo_state.in_deadband,
                    enabled=settings.static_vel_hold_enable)
                if context.servo_state.static_hold_now:
                    context.servo_state.prev_cmd_vel[:] = 0.0
                else:
                    context.servo_state.prev_cmd_vel = context.servo_state.actual_velocity[:3].copy()

                v_tcp = camera_velocity_to_tcp(context.servo_state.actual_velocity, context.t_tcp_camera)
                v_world = np.zeros(6)
                v_world[0:3] = tcp_rot @ v_tcp[0:3]
                v_world[3:6] = angular_world

                if context.table_safety_enabled:
                    v_world[2], _ = limit_downward_velocity(v_world[2], tcp_pos[2], actual_tcp[2])

                wall_touch = False
                for ax in range(3):
                    if tcp_pos[ax] <= settings.workspace_min[ax] and v_world[ax] < 0:
                        v_world[ax] = 0.0
                        wall_touch = True
                    elif tcp_pos[ax] >= settings.workspace_max[ax] and v_world[ax] > 0:
                        v_world[ax] = 0.0
                        wall_touch = True
                # 70% 臂展水平径向软墙：只削 X/Y 继续向外的分量，保留
                # 切向、向内及 Z 运动；避免三维球墙在高姿态过度压缩空间。
                v_world[:3], radial_touch = horizontal_radial_soft_wall(
                    tcp_pos, v_world[:3], settings.workspace_radius)
                wall_touch = wall_touch or radial_touch
                context.servo_state.near_wall = bool(
                    np.any(tcp_pos < settings.workspace_min + 0.05)
                    or np.any(tcp_pos > settings.workspace_max - 0.05)
                    or tcp_radius > settings.workspace_radius - 0.05)

                manip = compute_manipulability(J_robot)
                if settings.damping_adaptive and manip < settings.manipulability_threshold:
                    damping = settings.damping_base * (1.0 + (settings.manipulability_threshold - manip) * 100)
                    v_world *= max(settings.min_manipulability_speed_scale,
                                   manip / settings.manipulability_threshold)
                else:
                    damping = settings.damping_base

                joint_vel = compute_damped_pseudoinverse(J_robot, damping) @ v_world
                joint_vel = np.clip(joint_vel, -settings.max_vel_joint, settings.max_vel_joint)
                if (context.servo_state.in_deadband and
                        np.linalg.norm(ori_err) <= settings.orientation_deadband):
                    joint_vel[:] = 0.0
                joint_vel = context.apply_stationary_joint_hold(
                    context.robot, context.stationary_joint_hold,
                    context.servo_state.hold_latch is not None, context.stationary_joint_hold_active, joint_vel)
                if context.stationary_joint_hold_active[0]:
                    context.servo_state.actual_velocity[:] = 0.0
                    context.servo_state.prev_cmd_vel[:] = 0.0
                context.servo_state.last_joint_speed = float(np.linalg.norm(joint_vel))

                # 用适配层内部下发角（含 200Hz 斜坡）的差分估计本周
                # 实际相机 twist，下一周期从 KF 观测速度中扣除。
                # 这比直接用期望 joint_vel 更接近真实 deceleration：
                # 指令归零后斜坡仍在走，不能把相机当成已静止。
                if (context.t_tcp_camera is not None
                        and context.servo_state.prev_command_joints is not None):
                    qdot_cmd = ((current_joints - context.servo_state.prev_command_joints)
                                / max(float(dt), 1e-3))
                    # 控制周期抖动时防止差分被放大；正常斜坡本就在
                    # MAX_VEL_JOINT 内，钳位只影响异常采样。
                    qdot_cmd = np.clip(qdot_cmd,
                                       -settings.max_vel_joint, settings.max_vel_joint)
                    if np.all(np.isfinite(qdot_cmd)):
                        context.servo_state.prev_camera_twist = camera_twist_from_tcp(
                            J_robot @ qdot_cmd, tcp_rot, context.t_tcp_camera)
                    else:
                        context.servo_state.prev_camera_twist[:] = 0.0
                else:
                    context.servo_state.prev_camera_twist[:] = 0.0
                context.servo_state.prev_command_joints = current_joints.copy()

                context.robot.joint_vel_cmd(joint_vel)
                with context.lock:
                    context.control_status["joint_speed"] = float(np.linalg.norm(joint_vel))
                    _status = "贴墙滑动中" if wall_touch else ""
                    if context.servo_state.reach_att < 0.95:
                        _status += ("" if not _status else " ") + \
                            f"伸展衰减x{context.servo_state.reach_att:.2f}"
                    context.control_status["error"] = _status
                    context.control_status["active"] = True

            except Exception as exc:
                context.robot.hold()
                context.servo_state.prev_camera_twist[:] = 0.0
                context.servo_state.prev_command_joints = None
                context.servo_state.hold_world = None
                context.servo_state.last_joint_speed = 0.0
                context.servo_state.settled_frames = 0
                with context.lock:
                    if context.table_safety_enabled:
                        context.control.following = False
                        context.control.gesture_start_allowed = False
                    context.control_status["error"] = (
                        f'伺服异常，已暂停: {exc}' if context.table_safety_enabled else str(exc))
                    context.control_status["active"] = False
        else:
            context.stationary_joint_hold_active[0] = False
            context.servo_state.actual_velocity = np.zeros(6)
            context.servo_state.prev_cmd_vel[:] = 0.0
            context.servo_state.prev_target_cam = None
            context.servo_state.prev_camera_twist[:] = 0.0
            context.servo_state.prev_command_joints = None
            context.kf.reset()
            context.servo_state.kf_last_seq = -1
            context.servo_state.kf_time = 0.0
            context.servo_state.kf_reject_count = 0
            context.servo_state.ff_moving = False
            context.servo_state.ff_exit_count = 0
            context.servo_state.hold_latch = None
            context.servo_state.hold_world = None
            context.lead_gate.reset()
            context.fast_start.reset()
            context.servo_state.slow_active = False
            context.axis_q_gate.reset()
            context.static_vel_hold.reset()
            context.servo_state.static_hold_now = False
            context.servo_state.last_joint_speed = 0.0
            context.servo_state.settled_frames = 0
            context.servo_state.reach_att = 1.0
            context.servo_state.target_vel_f = np.zeros(3)
            context.servo_state.in_deadband = False
            context.servo_state.coast_offset[:] = 0.0
            context.servo_state.coast_vel[:] = 0.0
            context.servo_state.coast_err = 1e9
            context.robot.hold()
            with context.lock:
                context.control_status["joint_speed"] = 0.0
                context.control_status["active"] = False
                if following and context.servo_state.lost_counter > settings.lost_hold_frames:
                    context.control_status["error"] = "手掌丢失，保持位置"

        elapsed = time.monotonic() - t0
        with context.lock:
            context.control_status["ctrl_dt_ms"] = dt * 1000.0
            context.control_status["ctrl_work_ms"] = elapsed * 1000.0
            context.control_status["ctrl_overruns"] = (
                context.control_status.get("ctrl_overruns", 0)
                + (1 if elapsed > settings.ctrl_rate else 0))
            context.control_status["target_age_ms"] = (
                measurement_age(t0, obs_time) * 1000.0 if obs_time > 0 else 0.0)
            context.control_status["lead_ms"] = lead_s * 1000.0
            context.control_status["lead_raw_ms"] = lead_raw_s * 1000.0
            context.control_status["lead_weight"] = context.servo_state.lead_weight_now
            context.control_status["lead_target_weight"] = context.servo_state.lead_target_weight
            context.control_status["kf_speed"] = kf_speed_now
            context.control_status["lead_error"] = lead_error_now
            context.control_status["ffm"] = context.servo_state.ff_moving
            context.control_status["hold_latched"] = context.servo_state.hold_latch is not None
            context.control_status["target_seq"] = obs_seq
            context.control_status["target_frozen"] = frozen_meas
            context.control_status["slow_active"] = context.servo_state.slow_active
        publish_live = (context.live_publisher is not None
                        and context.live_publisher.due(t0))
        if context.diag_recorder is not None or publish_live:
            target_values = ([""] * 3 if target_cam is None
                             else np.asarray(target_cam).tolist())
            kv_log = _kv_raw if settings.kf_enable else _zero3
            with context.lock:
                status_snapshot = dict(context.control_status)
                processing_ms = context.observation.processing_ms
            feedback = (context.feedback_sampler.snapshot()
                        if context.feedback_sampler is not None else None)
            feedback_age = feedback_age_ms(
                feedback, time.monotonic(),
                feedback.get("rate_hz", context.feedback_hz) if feedback else context.feedback_hz)
            feedback_fresh = feedback_age is not None
            q_scale_log = np.asarray(context.kf.q_scale, dtype=np.float64)
            if q_scale_log.ndim == 0:
                q_scale_log = np.full(3, float(q_scale_log))
            row = {
                "host_time": f"{t0:.6f}", "dt_s": f"{dt:.6f}",
                "seq": obs_seq, "new_obs": int(new_kf_meas),
                "following": int(following), "visible": int(visible),
                "frozen": int(frozen_meas), "kf_valid": int(context.kf.initialized),
                "ff_moving": int(context.servo_state.ff_moving),
                "hold_latch": int(context.servo_state.hold_latch is not None),
                "slow_active": int(context.servo_state.slow_active),
                "static_hold": int(context.servo_state.static_hold_now),
                "lead_weight": context.servo_state.lead_weight_now,
                "lead_raw_s": lead_raw_s, "lead_s": lead_s,
                "obs_age_ms": status_snapshot.get("target_age_ms", 0.0),
                "processing_ms": processing_ms,
                "ctrl_work_ms": elapsed * 1000.0,
                "overrun": int(elapsed > settings.ctrl_rate),
                "kf_q_scale_x": q_scale_log[0],
                "kf_q_scale_y": q_scale_log[1],
                "kf_q_scale_z": q_scale_log[2],
                "ff_taper_now": context.servo_state.ff_taper_now,
                "ff_approach_speed": context.servo_state.ff_approach_speed,
                "ff_scale": context.servo_state.ff_scale_now,
                "fb_valid": int(feedback_fresh),
                "fb_time": (f"{feedback['time']:.6f}"
                            if feedback_fresh else ""),
                "fb_age_ms": feedback_age if feedback_fresh else "",
                "fb_read_ms": feedback["read_ms"] if feedback_fresh else "",
                "fb_rate_hz": feedback.get("rate_hz", context.feedback_hz) if feedback_fresh else "",
                "tracking_err_max": (float(np.max(np.abs(
                    feedback["actual"] - feedback["command"])))
                    if feedback_fresh else ""),
                "error_norm": (float(np.linalg.norm(
                    target_cam - np.array([0.0, 0.0, context.desired_depth])))
                    if target_cam is not None else ""),
                "kv_x": kv_log[0], "kv_y": kv_log[1], "kv_z": kv_log[2],
                "compensated_palm_speed": (compensated_speed_log
                                            if compensated_speed_log is not None else ""),
                "stationary_joint_hold": int(context.stationary_joint_hold_active[0]),
                "joint_speed": status_snapshot.get("joint_speed", 0.0),
                "q_cmd_valid": 1,
                "hw_time_used": int(hw_time_used),
                "error": status_snapshot.get("error", ""),
            }
            for axis in range(3):
                row[f"target_{'xyz'[axis]}"] = target_values[axis]
            if q_cmd_log is None:
                q_cmd_log = context.robot.get_command_pos()  # 仅读内存，待机时也记录当前指令
            for joint in range(6):
                row[f"q_cmd_{joint + 1}"] = q_cmd_log[joint]
                row[f"q_act_{joint + 1}"] = (
                    feedback["actual"][joint] if feedback_fresh else "")
                row[f"q_fb_cmd_{joint + 1}"] = (
                    feedback["command"][joint] if feedback_fresh else "")
            if context.diag_recorder is not None:
                context.diag_recorder.log(row)
            if publish_live:
                context.live_publisher.offer(row)
        delay = next_deadline - time.monotonic()
        if delay > 0:
            context.stop_ctrl.wait(delay)
        else:
            # 超时后从当前时刻重新排期，不为补周期而连续空转。
            next_deadline = time.monotonic()
