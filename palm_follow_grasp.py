#!/usr/bin/env python3
"""
手掌跟随 + 捏合抓取 一体主程序（B601 RS + Orbbec Gemini 2）
=================================================================
一个脚本同时实现两个功能（共用一台相机、一次 MediaPipe 检测、一条 CAN 总线）：

  【跟随】掌心 3D（相机系）→ 误差 e = P_cam - [0,0,d]
         → 目标速度前馈 + 分方向增益 → tanh 软饱和
         → 相机系速度 → 伴随变换到 TCP → 世界系 twist
         → 阻尼伪逆雅可比（可操作度奇异保护）→ 关节速度 → 适配层积分下发
         （控制数学与 palm_follow_fixed_orientation.py 一致，
           新增前馈/分方向增益/软饱和/反转清惯性四项高速跟随改进）

  【抓取】拇指尖(4)—食指尖(8) 像素距离 → One Euro 滤波 → 线性映射为
         夹爪 J7 目标角度（度）→ 随 100Hz 关节指令经 rebot_control
         第 7 通道下发（单一 CAN 主机，无总线冲突）。

运行模式：
    python palm_follow_grasp.py                  # 跟随 + 夹爪（完整模式）
    python palm_follow_grasp.py --no-gripper     # 仅手掌跟随（夹爪保持打开）
    python palm_follow_grasp.py --no-follow      # 仅捏合夹爪（手臂保持不动，
                                                 #   无需手眼标定，可先验证夹爪映射）
    python palm_follow_grasp.py --no-arm         # 纯视觉演练（不碰硬件）
    python palm_follow_grasp.py --headless       # 无窗口模式（SSH 场景，q 键退出）

操作：
    R        : 请求开始跟随（--no-follow 时无效）
    g / 空格 : 暂停跟随
    q        : 退出（停止并保持 → 缓慢回零 → 失能）
    Esc      : 请求退出并回零，不是硬件急停

⚠️ 首次运行前：
    1. 夹爪开合标定：tools/gripper_j7_calibrate.py（结果写入 config/grasp.yaml）
    2. 手眼标定：tools/handeye_joint_calib.py（生成 config/hand_eye_calibration.json
       和 config/joint_offset.json，仅跟随功能需要）
    3. 先用固定假目标低速测试跟随（--gain 1.5 --max-vel 0.10），再上真人手
"""

import copy
import os
import sys
import json
import time
import threading
import queue
import signal
from datetime import datetime

import numpy as np
import cv2
import yaml
from pynput import keyboard as pynput_kb

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_follow.configuration.gripper import GripperSettings, load_grasp_cfg
from robot_follow.configuration.follow import FollowConfiguration
from robot_follow.configuration.motion import resolve_motion_options
from robot_follow.configuration.validation import validate_follow_cfg
from robot_follow.control.feedback import (verify_home_joint_position,
    near_limit_j2_floor, near_limit_feedback_error as near_limit_feedback_error,
    feedback_corrected_joints as feedback_corrected_joints)
from robot_follow.perception.ir_filter import in_ir_gripper_region, ir_gripper_detection
from robot_follow.perception.depth import DepthGate
from robot_follow.perception.pipeline import HandPipeline
from robot_follow.hardware.camera import CameraSession
from robot_follow.perception.calibration import (verify_d435i_calibration,
    verify_calibration_joint_offset, verify_orbbec_camera)
from robot_follow.control.guards import (pause_outside_workspace, pause_runtime_workspace,
    apply_stationary_joint_hold)
from robot_follow.runtime.observation import TargetObservation
from robot_follow.runtime.control import RunControl
from robot_follow.runtime.home import HomeOptions, initialize_home
from robot_follow.runtime.telemetry import LiveTelemetryPublisher, PhaseTelemetry
from robot_follow.perception.start_gesture import OpenPalmStartGate
from robot_follow.runtime.resources import RuntimeResources
from robot_follow.runtime.vision import VisionContext, VisionOptions, run_vision
from robot_follow.control.components import ServoComponents
from robot_follow.control.gripper import GripperMapper
from robot_follow.control.servo import ServoContext, ServoSettings, run_servo_loop
from robot_follow.runtime.cli import parse_args
# Re-export earlier public helpers for existing scripts.
from robot_follow.control.kinematics import (
    compute_damped_pseudoinverse as compute_damped_pseudoinverse,
    compute_manipulability as compute_manipulability,
    rotation_matrix_to_axis_angle as rotation_matrix_to_axis_angle,
    compute_orientation_error as compute_orientation_error,
    skew, camera_velocity_to_tcp as camera_velocity_to_tcp)

from robot_follow.control.timing import CameraTimestampCalibrator
from robot_follow.runtime.diagnostics import DiagnosticRecorder, ActualFeedbackSampler
from palm_detection import (PalmDetector, PalmTracker, OneEuroFilter,
                            LKPalmBridge,
                            get_median_depth, pixel_to_3d,
                            draw_palm, draw_landmarks, pinch_distance_px)
# 注意：RebotAdapter 在 main() 中惰性导入（--no-arm 模式不依赖 pinocchio/rebot_control）


# ─── 初始位姿 ────────────────────────────────────────────────
# 已用 URDF + Pinocchio 网格搜索验证远离奇异。⚠️ 不要用全零位
# （全零位在 J2/J3 限位边界上且可操作度仅 0.0017，近奇异）。
HOME_JOINT_POS = [0.0, 1.2, 1.05, 0.0, 0.0, 0.0]

# 自定义 HOME：若存在 tools/teach_home.py 生成的 config/home_pose.json 则优先使用
_HOME_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "config", "home_pose.json")
if os.path.exists(_HOME_FILE):
    with open(_HOME_FILE) as _f:
        _home = json.load(_f)
    HOME_JOINT_POS = _home["home_joint_pos"]
    print(f"[HOME] 使用示教 HOME: {np.round(HOME_JOINT_POS, 3).tolist()}"
          f"  (TCP≈{np.round(_home['tcp_position'], 3).tolist()})")

# ─── 视觉伺服控制参数 ───────────────────────────────────────
# Legacy public constants remain available to existing integrations.
_legacy_follow_config = FollowConfiguration()
globals().update({key.upper(): value for key, value in vars(_legacy_follow_config).items()})
TOOL_OFFSET = _legacy_follow_config.tool_offset


def load_follow_cfg(path):
    """Load settings using the compatibility configuration instance."""
    return _legacy_follow_config.load_follow_cfg(path)



def load_camera_follow_cfg(camera, explicit_path, config_dir):
    """默认只为 Gemini 336 叠加独立参数；显式配置始终优先。"""
    cfg = load_follow_cfg(explicit_path or os.path.join(config_dir, 'follow.yaml'))
    if camera == 'gemini336' and explicit_path is None:
        profile_path = os.path.join(config_dir, 'follow_gemini336.yaml')
        with open(profile_path, encoding='utf-8') as f:
            profile = yaml.safe_load(f) or {}
        cfg['follow'].update(profile.get('follow', {}))
        print(f'[配置] Gemini 336 静止稳定参数: {profile_path}', flush=True)
    return cfg


def apply_follow_cfg(cfg):
    """Compatibility API: update legacy exported settings for existing scripts."""
    _legacy_follow_config.apply_follow_cfg(cfg)
    globals().update({key.upper(): value for key, value in vars(_legacy_follow_config).items()})



# ═══ TCP 运动学适配 ═══

def compute_tcp_fk_and_jacobian(robot, joint_positions):
    """Compatibility entry point for TCP kinematics with the configured offset."""
    if hasattr(robot, 'tcp_pose_and_jacobian'):
        return robot.tcp_pose_and_jacobian(joint_positions, TOOL_OFFSET)
    from robot_follow.hardware.kinematics import tcp_pose_and_jacobian
    return tcp_pose_and_jacobian(robot, joint_positions, TOOL_OFFSET)


# ═══ 主程序 ═══

def main():
    with RuntimeResources() as resources:
        return _run(resources)


def _run(resources):
    args = parse_args()
    resources.web_control = args.web_control
    if args.web_auto_follow:
        raise ValueError('--web-auto-follow 已停用：HOME 后请按 R 或张开手掌 3 秒')
    web_shutdown_requested = [False]
    def request_web_shutdown(_sig, _frame):
        web_shutdown_requested[0] = True
    if args.web_control:
        # During HOME initialization defer shutdown until the normal cleanup
        # path is established; interrupting a move before it exists is unsafe.
        resources.original_sigint_handler = signal.signal(
            signal.SIGINT, request_web_shutdown)
    if args.gemini2_urdf_pose and args.camera != 'gemini336':
        raise ValueError('--gemini2-urdf-pose 仅适用于 --camera gemini336')
    if args.gemini2_urdf_pose and args.gemini336_pose == 'calibration':
        raise ValueError('--gemini2-urdf-pose 不能与 calibration 同时使用')
    use_336_urdf = args.camera == 'gemini336' and args.gemini336_pose == 'urdf'
    if not np.isfinite(args.home_j34_tolerance) or not 2 <= args.home_j34_tolerance <= 4:
        raise ValueError('--home-j34-tolerance 必须在 2~4° 内')
    if not np.isfinite(args.home_speed) or not 0 < args.home_speed <= 15:
        raise ValueError('--home-speed 必须大于 0 且不超过 15°/s')
    if args.keep_gripper_position and not args.no_gripper:
        raise ValueError('--keep-gripper-position 需要同时指定 --no-gripper')
    home_joint_pos = np.asarray(HOME_JOINT_POS, dtype=float)
    if args.home_file:
        with open(args.home_file, encoding='utf-8') as f:
            home_joint_pos = np.asarray(json.load(f)['home_joint_pos'], dtype=float)
        print(f'[HOME] 采用指定文件: {args.home_file}', flush=True)
    if home_joint_pos.shape != (6,) or not np.all(np.isfinite(home_joint_pos)):
        raise ValueError('HOME 文件必须包含 6 个有限关节角')
    if not args.no_arm:
        from rebot_adapter import load_soft_joint_limits, validate_home_target
        lower, upper = load_soft_joint_limits()
        near_limit_preflight = validate_home_target(home_joint_pos, lower, upper,
                                                  enabled=args.allow_home_limit_recovery)
        if (not np.isfinite(args.home_j2_command_offset) or
                not 0 <= args.home_j2_command_offset <= 2 or
                (args.home_j2_command_offset and not near_limit_preflight)):
            raise ValueError('J2 指令补偿仅支持近限位 HOME，范围 0~2°')

    # 跟随参数：yaml 为默认值，命令行参数优先级更高
    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config')
    follow_cfg = load_camera_follow_cfg(args.camera, args.follow_cfg, config_dir)
    validate_follow_cfg(follow_cfg)
    runtime_config = copy.deepcopy(_legacy_follow_config)
    runtime_config.apply_follow_cfg(follow_cfg)
    fcfg = follow_cfg["follow"]

    motion = resolve_motion_options(args, fcfg)
    desired_depth = motion.desired_depth
    lambda_gain_xyz = np.array(motion.gains)
    max_vel_cart = motion.max_velocity
    camera_fps = motion.camera_fps
    color_exposure_ms = motion.color_exposure_ms

    if args.ir_emitter == "off" and not args.no_arm:
        raise ValueError("--ir-emitter off 仅用于 --no-arm 纯视觉测试")

    follow_enabled = not args.no_follow
    grasp_enabled = not args.no_gripper

    # 捏合 → 夹爪映射配置
    grasp_cfg = load_grasp_cfg(args.grasp_cfg)
    grasp_cfg["gripper"]["enabled"] = (grasp_cfg["gripper"]["enabled"]
                                       and grasp_enabled)
    grasp_on = grasp_cfg["gripper"]["enabled"] and not args.no_arm
    gripper_settings = GripperSettings.from_config(grasp_cfg)
    pinch_closed_px, pinch_open_px = gripper_settings.closed_px, gripper_settings.open_px
    grasp_smoothing, grasp_deadband = gripper_settings.smoothing, gripper_settings.deadband_deg
    pinch_filter_mode = gripper_settings.filter_mode
    gripper_closed_deg, gripper_open_deg = gripper_settings.closed_deg, gripper_settings.open_deg
    hold_while_moving = gripper_settings.hold_while_moving
    move_speed_thresh = gripper_settings.move_speed_threshold

    print("\n" + "=" * 60)
    print(f"手掌跟随 + 捏合抓取 (B601 RS + {args.camera})")
    print("=" * 60)
    if args.no_gripper and not args.no_arm:
        gripper_mode_text = ("HOLD (--no-gripper)" if args.keep_gripper_position
                             else "OPEN (--no-gripper)")
    elif grasp_cfg["gripper"]["enabled"] and args.no_arm:
        gripper_mode_text = "PREVIEW(仅预览)"
    else:
        gripper_mode_text = "ON" if grasp_on else "OFF"
    print(f"模式: follow={'ON' if follow_enabled else 'OFF'}  "
          f"gripper={gripper_mode_text}  "
          f"{'[纯视觉演练]' if args.no_arm else ''}"
          f"{'[无窗口]' if args.headless else ''}")

    # ── 手眼标定（仅跟随模式需要）──
    T_tcp_camera = None
    calibration_data = None
    if follow_enabled and not args.no_arm and use_336_urdf:
        from rebot_adapter import URDF_PATH
        from urdf_camera_pose import gemini336_nominal_tcp_camera
        T_tcp_camera = gemini336_nominal_tcp_camera(URDF_PATH, runtime_config.tool_offset)
        print('[相机位姿] 使用 Gemini 336 V1 支架 CAD 名义位姿（未实测验证）；请核对实际安装和运动方向', flush=True)
        print(T_tcp_camera, flush=True)
    elif follow_enabled and not args.no_arm:
        calib_name = {"d435i": "hand_eye_calibration_d435i.json",
                      "gemini336": "hand_eye_calibration_gemini336.json",
                      "gemini2": "hand_eye_calibration.json"}[args.camera]
        calib_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", calib_name)
        if not os.path.exists(calib_file):
            raise FileNotFoundError(
                f"未找到 {calib_file}，请先完成所选相机的手眼标定；"
                f"或先用 --no-follow 只验证夹爪功能")
        with open(calib_file, 'r') as f:
            calibration_data = json.load(f)
            T_tcp_camera = np.array(calibration_data["T_tcp_camera"])

    # ── 相机与检测 ──
    if args.camera == "d435i":
        from realsense_camera import RealSenseCamera as Camera
    else:
        from gemini2_camera import Gemini2Camera as Camera
    print(f"初始化 {args.camera} 相机（{camera_fps} fps）...")
    camera_kwargs = dict(fps=camera_fps, color_exposure_ms=color_exposure_ms)
    if args.web_control and args.camera == 'gemini336' and os.environ.get('REBOT_CAMERA_SERIAL'):
        camera_kwargs['serial'] = os.environ['REBOT_CAMERA_SERIAL']
    if args.camera == "d435i":
        camera_kwargs["ir_emitter"] = ("off" if args.ir_emitter == "off" or
                                       (args.ir_emitter == "auto" and args.no_arm)
                                       else "on")
    def validate_camera(camera):
        if args.camera == "d435i":
            verify_d435i_calibration(camera, calibration_data)
        else:
            verify_orbbec_camera(camera, args.camera, calibration_data,
                                 require_calibration=(follow_enabled and not args.no_arm
                                                      and not use_336_urdf))

    camera_session = CameraSession(lambda: Camera(**camera_kwargs), validate_camera)
    resources.camera = camera_session
    cam = camera_session.open()
    fx, fy, cx, cy = cam.fx, cam.fy, cam.cx, cam.cy

    def project_pixel(u, v, depth):
        if args.camera in ("d435i", "gemini336"):
            xy = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64),
                                     camera_session.camera.camera_matrix,
                                     camera_session.camera.dist_coeffs)[0, 0]
            return xy[0] * depth, xy[1] * depth, depth
        return pixel_to_3d(u, v, depth, fx, fy, cx, cy)
    # 捏合像素阈值按 640 宽标定（grasp.yaml 中 closed_px/open_px）；
    # 相机实际分辨率不同时（如 60fps 模式下 1280x720）按比例自动换算
    pinch_scale = cam.width / 640.0 if cam.width != 640 else 1.0
    if pinch_scale != 1.0:
        pinch_closed_px *= pinch_scale
        pinch_open_px *= pinch_scale
        print(f"[配置] 相机宽度 {cam.width}px，捏合阈值自动 x{pinch_scale:.2f}: "
              f"{pinch_closed_px:.0f}px / {pinch_open_px:.0f}px")
    desired_u, desired_v = int(round(cx)), int(round(cy))   # 期望掌心点 = 画面中心

    print("初始化手掌检测...")
    ir_image = (args.camera == "d435i" and not getattr(cam, "has_color", True))
    if ir_image:
        print("[检测器] 红外模式：9px 中值滤波，检测阈值 0.3；启用夹爪区域过滤")
        detector = PalmDetector(backend=args.detector_backend,
                                min_detection_confidence=0.3,
                                min_presence_confidence=0.3)
    else:
        detector = PalmDetector(backend=args.detector_backend)
    resources.detector = detector
    # 检测+跟踪混合：掉检帧光流桥接（hybrid 系列参数见 follow.yaml）
    lk_bridge = LKPalmBridge(
        enable=bool(fcfg.get("hybrid_track_enable", True)),
        max_frames=int(fcfg.get("hybrid_max_frames", 15)),
        box_half=int(fcfg.get("hybrid_box_half", 45)),
        max_step_px=float(fcfg.get("hybrid_max_step_px", 90.0)))

    preview_writer = None
    startup_preview = None
    if args.web_control and os.environ.get('REBOT_WEB_PREVIEW_PATH'):
        try:
            from web_camera_preview import CameraPreviewWriter, StartupCameraPreview
            preview_writer = CameraPreviewWriter(os.environ['REBOT_WEB_PREVIEW_PATH'])
            resources.preview_writer = preview_writer
            startup_preview = StartupCameraPreview(cam, preview_writer)
            resources.startup_preview = startup_preview
            print('[网页相机] 初始化期间预览已启动', flush=True)
        except Exception as exc:
            if preview_writer is not None:
                preview_writer.close()
                preview_writer = None
                resources.preview_writer = None
            print(f'[网页相机] 预览不可用: {exc}', flush=True)

    # ── 机械臂 ──
    robot = None
    live_publisher = None
    desired_rotation = None
    near_limit_home = False
    if not args.no_arm:
        from rebot_adapter import RebotAdapter, validate_home_recovery
        print("初始化机械臂...")
        robot = RebotAdapter()
        resources.robot = robot

        # Some versions of the arm SDK install their own SIGINT handler even
        # with the keyboard listener disabled. Restore the web lifecycle so
        # shutdown remains coordinated and return telemetry stays available.
        if args.web_control:
            signal.signal(signal.SIGINT, request_web_shutdown)

        if args.web_control:
            try:
                live_publisher = LiveTelemetryPublisher.from_environment()
                if live_publisher is not None:
                    live_publisher.start()
                    resources.live_publisher = live_publisher
                    resources.phase_telemetry = PhaseTelemetry(robot, live_publisher).start()
                    resources.phase_telemetry.set_phase('home')
            except (OSError, ValueError, RuntimeError) as exc:
                print(f'[网页] 实时状态不可用: {exc}', flush=True)

        try:
            result = initialize_home(
                robot, home_joint_pos,
                HomeOptions(allow_limit_recovery=args.allow_home_limit_recovery,
                            j2_command_offset_deg=args.home_j2_command_offset,
                            j34_tolerance_deg=args.home_j34_tolerance,
                            speed_deg_s=args.home_speed, follow_enabled=follow_enabled),
                validate_recovery=validate_home_recovery,
                tcp_kinematics=compute_tcp_fk_and_jacobian)
            near_limit_home = result.near_limit
            desired_rotation = result.desired_rotation
            if startup_preview is not None and not startup_preview.stop():
                raise RuntimeError('相机预览线程未能退出，无法安全转交相机')
            startup_preview = None
            resources.startup_preview = None
        except BaseException:
            print('[HOME] 初始化失败，保持后安全回零并失能', file=sys.stderr, flush=True)
            raise

    if startup_preview is not None:
        if not startup_preview.stop():
            raise RuntimeError('相机预览线程未能退出，无法安全转交相机')
        startup_preview = None
        resources.startup_preview = None

    # --no-gripper 是“不做捏合控制”，不是“夹爪保持当前角度”：
    # 夹爪打开可减少对腕部相机的视野遮挡。仍然只走 J7 适配层，
    # 与 6 关节共用单一 CAN 主机，不另开 motorbridge。
    if robot is not None and robot.is_stopping():
        print('[安全] 控制器已停机，程序退出', flush=True)
        return
    if args.no_gripper and robot is not None and not args.keep_gripper_position:
        robot.set_gripper_deg(gripper_open_deg)
        print(f"[夹爪] --no-gripper：保持打开 {gripper_open_deg:.1f}°"
              "（不做捏合控制）")
    elif args.no_gripper and robot is not None:
        print('[夹爪] 保持当前位置（不做捏合控制）', flush=True)

    # 夹爪平滑初值 = 当前实际角度，避免启动瞬间夹爪猛动
    if robot is not None:
        smoothed_gripper_deg = robot.get_gripper_deg()
    else:
        smoothed_gripper_deg = (gripper_closed_deg + gripper_open_deg) / 2.0
    pinch_oe = OneEuroFilter(
        gripper_settings.min_cutoff, gripper_settings.beta)
    gripper_mapper = GripperMapper(
        closed_px=pinch_closed_px, open_px=pinch_open_px,
        closed_deg=gripper_closed_deg, open_deg=gripper_open_deg,
        initial_deg=smoothed_gripper_deg, filter_mode=pinch_filter_mode,
        smoothing=grasp_smoothing, deadband=grasp_deadband,
        hold_while_moving=hold_while_moving, euro_filter=pinch_oe)

    # ── 共享状态 ────────────────────────────────────────────
    lock = threading.Lock()
    observation = TargetObservation()  # 访问由 lock 保护
    run_control = RunControl(following=False)
    if follow_enabled:
        print('[跟随] HOME 已就绪，等待按 R 或张开手掌 3 秒', flush=True)
    control_status = {"joint_speed": 0.0, "error": "", "active": False}

    if follow_enabled:
        print(f"跟随参数: 距离 {desired_depth}m  "
              f"gain(x/y/z)={lambda_gain_xyz.tolist()}  "
              f"max_vel={max_vel_cart}  前馈={runtime_config.vel_feedforward}")
        if runtime_config.kf_enable:
            print(f"卡尔曼: Q={runtime_config.kf_q} R={runtime_config.kf_r} Rxy={runtime_config.kf_r_xy} Rz={runtime_config.kf_r_z} "
                  f"lead={'动态' if runtime_config.kf_dynamic_lead else '固定'} "
                  f"base={runtime_config.kf_lead_ms}ms max={runtime_config.kf_lead_max_ms}ms  "
                  f"速度死区={runtime_config.ff_kf_deadband_off}/{runtime_config.ff_kf_deadband_on}m/s")
            print(f"自适应lead: {'ON' if runtime_config.kf_lead_adaptive else 'OFF'} "
                  f"(近端={runtime_config.kf_lead_near_dist:.3f}m 全量={runtime_config.kf_lead_full_dist:.3f}m "
                  f"攻/释={runtime_config.kf_lead_attack_tau:.3f}/{runtime_config.kf_lead_release_tau:.3f}s)")
            print(f"静止快速退出: {'ON' if runtime_config.kf_fast_start_enable else 'OFF'} "
                  f"({runtime_config.kf_fast_start_frames}帧 / {runtime_config.kf_fast_start_dist:.3f}m)  "
                  f"eye-in-hand 自运动补偿: ON")
            print(f"静止稳定: 基准Q保护={'ON' if runtime_config.kf_axis_adapt_static_guard else 'OFF'} "
                  f"静止Q倍率={runtime_config.kf_static_q_scale:.2f} "
                  f"残余速度保持={'ON' if runtime_config.static_vel_hold_enable else 'OFF'} "
                  f"(≤{runtime_config.static_vel_hold_speed:.3f}m/s, "
                  f"{runtime_config.static_vel_hold_delay_s:.3f}s)")
    if grasp_on or (grasp_cfg["gripper"]["enabled"] and args.no_arm):
        print(f"夹爪映射: 捏合 {pinch_closed_px:.0f}px → {gripper_closed_deg:.1f}° | "
              f"张开 {pinch_open_px:.0f}px → {gripper_open_deg:.1f}°"
              f"  滤波={pinch_filter_mode}"
              + ("  [手臂运动时冻结]" if hold_while_moving else ""))
    print("操作: R 开始跟随 | g/空格 暂停跟随 | q/Esc/Ctrl+C 退出并安全回零")
    print("=" * 60 + "\n")

    # ── 键盘监听 ────────────────────────────────────────────
    def pause_tracking():
        if not follow_enabled:
            print("\n[跟随] 当前为 --no-follow 模式，跟随不可用")
            return
        with lock:
            run_control.following = False
            run_control.gesture_start_allowed = False
        print("\n[跟随] 暂停（保持当前位置）")

    def on_press(key):
        try:
            if key.char in ('r', 'R'):
                if follow_enabled:
                    with lock:
                        run_control.following = not run_control.servo_failed
                        run_control.gesture_start_allowed = False
            elif key.char == 'g':
                pause_tracking()
            elif key.char == 'q':
                run_control.quit_requested.set()    # headless 模式下的退出通道
        except AttributeError:
            if key == pynput_kb.Key.space:
                pause_tracking()
            elif key == pynput_kb.Key.esc:
                run_control.quit_requested.set()    # Esc = 退出并安全回零（统一退出路径）

    web_commands = queue.SimpleQueue()
    previous_handlers = resources.previous_handlers
    web_ready_path = os.environ.get('REBOT_WEB_READY_PATH') if args.web_control else None
    resources.web_ready_path = web_ready_path
    if args.web_control:
        # Signal handlers only enqueue; the servo thread applies explicit,
        # idempotent state changes, independent of camera frame delivery.
        for sig, enabled in ((signal.SIGUSR1, True), (signal.SIGUSR2, False)):
            previous_handlers[sig] = signal.signal(sig, lambda _sig, _frame, value=enabled: web_commands.put(value))
        kb_listener = None  # browser handles keys; avoids double toggles from pynput
    else:
        kb_listener = pynput_kb.Listener(on_press=on_press)
        resources.keyboard = kb_listener
        kb_listener.start()

    # ── 伺服控制线程（100Hz；--no-arm 时不启动）───────────
    stop_ctrl = threading.Event()
    resources.stop_event = stop_ctrl
    servo_components = ServoComponents.from_configuration(runtime_config)
    depth_oe = OneEuroFilter(runtime_config.depth_oe_min_cutoff, runtime_config.depth_oe_beta)
    stationary_joint_hold_active = [False]
    depth_gate = DepthGate(
        desired_depth=desired_depth, max_jump=runtime_config.z3d_max_jump,
        max_step=runtime_config.z3d_max_rate, freeze_seconds=runtime_config.z_freeze_max,
        depth_filter=depth_oe)
    hand_pipeline = HandPipeline(
        detector=detector, bridge=lk_bridge, tracker_factory=PalmTracker,
        depth_gate=depth_gate, median_depth=get_median_depth,
        project=project_pixel, infrared=ir_image)
    ctrl_thread = None
    timestamp_calibrator = CameraTimestampCalibrator(
        max_transport_s=runtime_config.camera_hw_max_transport_s)
    diag_recorder = None
    if runtime_config.diag_enable and robot is not None:
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(
            log_dir, "servo_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            + f"_{os.getpid()}.csv")
        try:
            diag_recorder = DiagnosticRecorder(log_path, runtime_config.diag_queue_size).start()
            resources.diagnostic_recorder = diag_recorder
            print(f"[诊断] 高频日志: {log_path}")
        except OSError as exc:
            print(f"[诊断] 日志初始化失败，不影响控制: {exc}")
            diag_recorder = None
    feedback_sampler = None
    feedback_hz = 2.0 if near_limit_home else runtime_config.diag_actual_feedback_hz
    if args.camera == 'gemini336':
        feedback_hz = 5.0
    if robot is not None and (args.camera == 'gemini336' or near_limit_home or ((diag_recorder is not None or live_publisher is not None) and feedback_hz > 0.0)):
        try:
            feedback_sampler = ActualFeedbackSampler(
                robot, hz=feedback_hz, paced=(args.camera == 'gemini336')).start()
            resources.feedback_sampler = feedback_sampler
            print(f"[反馈] 实际关节采样 {feedback_hz:.1f}Hz"
                  + ('，用于桌面保护' if args.camera == 'gemini336'
                     else '，用于近限位保护' if near_limit_home else '（仅诊断）'))
        except Exception as exc:
            print(f"[诊断] 实际关节采样启动失败，已禁用: {exc}")
            feedback_sampler = None


    if robot is not None:
        servo_context = ServoContext(
            stop_ctrl=stop_ctrl,
            web_control=args.web_control,
            web_commands=web_commands,
            lock=lock,
            control=run_control,
            follow_enabled=follow_enabled,
            robot=robot,
            servo_state=servo_components.state,
            observation=observation,
            near_limit_home=near_limit_home,
            feedback_sampler=feedback_sampler,
            home_joint_pos=home_joint_pos,
            control_status=control_status,
            axis_q_gate=servo_components.axis_noise,
            kf=servo_components.target_filter,
            fast_start=servo_components.fast_start,
            slow_follow=servo_components.slow_follow,
            desired_depth=desired_depth,
            lead_gate=servo_components.lead,
            t_tcp_camera=T_tcp_camera,
            lambda_gain_xyz=lambda_gain_xyz,
            desired_rotation=desired_rotation,
            max_vel_cart=max_vel_cart,
            static_vel_hold=servo_components.static_hold,
            stationary_joint_hold=bool(fcfg['stationary_joint_hold']),
            stationary_joint_hold_active=stationary_joint_hold_active,
            diag_recorder=diag_recorder,
            live_publisher=live_publisher,
            feedback_hz=feedback_hz,
            compute_tcp_fk_and_jacobian=compute_tcp_fk_and_jacobian,
            pause_outside_workspace=pause_runtime_workspace,
            apply_stationary_joint_hold=apply_stationary_joint_hold,
            table_safety_enabled=(args.camera == 'gemini336'),
        )
        servo_settings = ServoSettings.from_configuration(runtime_config)
        if resources.phase_telemetry is not None:
            resources.phase_telemetry.set_phase(None)
        ctrl_thread = threading.Thread(
            target=run_servo_loop, args=(servo_context, servo_settings), daemon=True)
        resources.control_thread = ctrl_thread
        ctrl_thread.start()
        if args.web_control and web_ready_path:
            try:
                with open(web_ready_path, 'x', encoding='ascii') as ready_file:
                    ready_file.write(str(os.getpid()))
            except OSError as exc:
                print(f'[网页] 无法发布就绪状态：{exc}', flush=True)

    run_vision(VisionContext(
        options=VisionOptions(
            headless=args.headless, web_control=args.web_control, no_arm=args.no_arm,
            follow_enabled=follow_enabled, gripper_enabled=grasp_cfg['gripper']['enabled'],
            infrared=ir_image, desired_u=desired_u, desired_v=desired_v,
            move_speed_threshold=move_speed_thresh,
            camera_hw_timestamp=runtime_config.camera_hw_timestamp_enable,
            depth_filter_cutoff=runtime_config.depth_oe_min_cutoff,
            depth_filter_beta=runtime_config.depth_oe_beta),
        camera_session=camera_session,
        control_status=control_status,
        draw_landmarks=draw_landmarks,
        draw_palm=draw_palm,
        gripper_mapper=gripper_mapper,
        hand_pipeline=hand_pipeline,
        lock=lock,
        observation=observation,
        pinch_distance_px=pinch_distance_px,
        preview_writer=preview_writer,
        resources=resources,
        robot=robot,
        run_control=run_control,
        timestamp_calibrator=timestamp_calibrator,
        pause_tracking=pause_tracking,
        web_shutdown_requested=web_shutdown_requested,
        one_euro_filter=OneEuroFilter,
        startup_gate=OpenPalmStartGate() if follow_enabled and robot is not None else None,
        request_start=lambda: web_commands.put('gesture'),
    ))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"\n错误: {exc}", file=sys.stderr, flush=True)
        import traceback
        traceback.print_exc()
        raise SystemExit(1) from exc
