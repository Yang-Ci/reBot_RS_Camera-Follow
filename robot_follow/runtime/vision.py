"""Run the camera, hand tracking, gripper and optional local display."""

import time
from dataclasses import dataclass
from typing import Any, Callable

import cv2

from robot_follow.presentation.display import DisplayFrame, VisionDisplay
from robot_follow.presentation.diagnostics import DebugFrame, format_debug
from robot_follow.runtime.recovery import reconnect_camera
from robot_follow.runtime.resources import RuntimeResources
from robot_follow.runtime.control import RunControl
from robot_follow.runtime.observation import TargetObservation
from robot_follow.control.gripper import GripperMapper
from robot_follow.hardware.camera import CameraSession
from robot_follow.perception.pipeline import HandPipeline
from robot_follow.perception.depth_diagnostics import DepthDiagnostics
from robot_follow.perception.start_gesture import is_open_palm


@dataclass(frozen=True)
class VisionOptions:
    """Immutable settings needed by the visual runtime."""
    headless: bool
    web_control: bool
    no_arm: bool
    follow_enabled: bool
    gripper_enabled: bool
    infrared: bool
    desired_u: int
    desired_v: int
    move_speed_threshold: float
    camera_hw_timestamp: bool
    depth_filter_cutoff: float
    depth_filter_beta: float


@dataclass
class VisionContext:
    """Dependencies owned by the application for one visual runtime."""
    options: VisionOptions
    camera_session: CameraSession
    control_status: dict
    draw_landmarks: Callable
    draw_palm: Callable
    gripper_mapper: GripperMapper
    hand_pipeline: HandPipeline
    lock: Any
    observation: TargetObservation
    pinch_distance_px: Callable
    preview_writer: Any
    resources: RuntimeResources
    robot: Any
    run_control: RunControl
    timestamp_calibrator: Any
    pause_tracking: Callable
    web_shutdown_requested: list[bool]
    one_euro_filter: Callable
    startup_gate: Any = None
    request_start: Callable | None = None


def run_vision(context: VisionContext):
    vision_display = VisionDisplay(context.draw_palm, context.draw_landmarks)
    if not context.options.headless:
        cv2.namedWindow('Palm Follow + Grasp', cv2.WINDOW_NORMAL)
        context.resources.close_windows = cv2.destroyAllWindows
        cv2.resizeWindow('Palm Follow + Grasp', 640, 480)

    fps = 0.0
    t_prev_frame = time.time()
    ms_grab = 0.0   # get_frames 耗时 EMA (ms)
    ms_det = 0.0    # 手掌识别耗时 EMA (ms)
    camera_timestamp_warned = False
    last_debug_at = 0.0
    depth_diagnostics = DepthDiagnostics(time.monotonic())
    try:
        last_frame_time = time.time()
        while not context.run_control.quit_requested.is_set() and not context.web_shutdown_requested[0]:
            if context.robot is not None and context.robot.is_stopping():
                print('[安全] 控制器正在停机，退出视觉程序', flush=True)
                break
            _t0 = time.perf_counter()
            cam = context.camera_session.camera
            color_image, depth_raw, depth_scale = context.camera_session.get_frames()
            frame_arrival_time = time.monotonic()
            raw_timestamp_s = None
            if context.options.camera_hw_timestamp and color_image is not None:
                try:
                    raw_timestamp_s, _ = cam.get_last_frame_timing()
                except (AttributeError, TypeError, ValueError, RuntimeError) as exc:
                    if not camera_timestamp_warned:
                        print(f"[相机时间] 接口不兼容，回退主机时刻: {exc}")
                        camera_timestamp_warned = True
            frame_obs_time, frame_hw_time = context.timestamp_calibrator.update(
                raw_timestamp_s, frame_arrival_time)
            _t1 = time.perf_counter()
            if color_image is None:
                summary = depth_diagnostics.record(None, frame_arrival_time, no_frame=True)
                if summary:
                    print(summary, flush=True)
                if context.startup_gate is not None:
                    context.startup_gate.reset()
                # 取流失败也属于观测丢失：让伺服进入有界 coast→保持流程，
                # 不能把最后一次 visible=True 永久留在共享状态中。
                with context.lock:
                    context.observation.visible = False
                # 无帧分支（USB 抖动/超时）：必须继续泵 GUI 事件，
                # 否则 waitKey 长期不执行，窗口被系统判"未响应"（卡死）
                if not context.options.headless:
                    cv2.waitKey(1)
                time.sleep(0.002)   # 防空转拉满 CPU
                # 帧看门狗：连续 2 秒无有效帧 → 判定相机掉线，自动重连
                # （手腕线束随机械臂弯折，USB 瞬断后 SDK 不会自愈）
                if time.time() - last_frame_time > 2.0:
                    print("\n[相机] 超过 2 秒无有效帧，尝试断开重连...")
                    reconnect_camera(
                        context.camera_session, context.hand_pipeline, context.timestamp_calibrator,
                        context.observation, context.lock,
                        lambda: context.one_euro_filter(context.options.depth_filter_cutoff,
                                              context.options.depth_filter_beta))
                    last_frame_time = time.time()
                continue
            last_frame_time = time.time()
            if context.preview_writer is not None:
                try:
                    context.preview_writer.submit(color_image, depth_raw, depth_scale)
                except Exception as exc:
                    print(f'[网页相机] 预览帧不可用: {exc}', flush=True)
                    context.preview_writer.close()
                    context.preview_writer = None
                    context.resources.preview_writer = None

            # 帧率统计（EMA）
            t_now = time.time()
            if t_now > t_prev_frame:
                fps = 0.2 * (1.0 / (t_now - t_prev_frame)) + 0.8 * fps
            t_prev_frame = t_now

            hand_frame = context.hand_pipeline.process(color_image, depth_raw, depth_scale)
            track, landmarks = hand_frame.track, hand_frame.landmarks
            summary = depth_diagnostics.record(track, time.monotonic())
            if summary:
                print(summary, flush=True)
            ms_grab = 0.9 * ms_grab + 0.1 * (_t1 - _t0) * 1000
            ms_det = 0.9 * ms_det + 0.1 * (hand_frame.detection_finished - _t1) * 1000

            target_publish_time = time.monotonic()
            with context.lock:
                context.observation.publish(track, observed_at=frame_obs_time,
                                    arrived_at=frame_arrival_time,
                                    published_at=target_publish_time,
                                    hardware_time=frame_hw_time)

            if context.startup_gate is not None and context.request_start is not None:
                with context.lock:
                    eligible = (context.run_control.gesture_start_allowed
                                and not context.run_control.following
                                and context.observation.visible
                                and not context.observation.frozen)
                opened = is_open_palm(landmarks, color_image.shape[1], color_image.shape[0])
                if context.startup_gate.update(frame_obs_time, eligible=eligible, opened=opened):
                    print('[跟随] 张开手掌已保持 3 秒，请求启动', flush=True)
                    context.request_start()

            # ── 捏合 → 夹爪（与跟随共用同一次检测结果）──
            # 新鲜度门控（log19 修复 + log21 修正）：
            # 跟踪器外推帧的捏合距离是纯噪声（实测 24~201px 乱跳），
            # 必须冻结；但捏合是 2D 测量、与深度无关——深度失效帧
            # （z3d=0）的捏合值依然准确，不能一并冻结，否则夹爪在
            # 近距/侧倾等深度易丢的场景表现得"没有反应"。
            # 因此门控只要求关键点是本帧实测（lost==0），不看深度。
            pinch_fresh = (track is not None and track['lost'] == 0)
            pinch_pts = None
            with context.lock:
                gripper_allowed = (not context.run_control.servo_failed
                                   and (not context.options.follow_enabled
                                        or context.run_control.following))
            if (context.options.gripper_enabled and landmarks is not None
                    and pinch_fresh and gripper_allowed):
                pinch_px, p1, p2 = context.pinch_distance_px(landmarks, color_image.shape)
                pinch_pts = (p1, p2)
                context.gripper_mapper.update_target(pinch_px)
                with context.lock:
                    arm_moving = context.control_status["joint_speed"] > context.options.move_speed_threshold
                candidate = context.gripper_mapper.candidate(arm_moving)
                if candidate is not None:
                    if context.robot is not None:
                        context.robot.set_gripper_deg(candidate)
                    context.gripper_mapper.committed(candidate)
            # 手丢失时：夹爪保持最后一次目标角，不漂移

            # Display and logging consume snapshots, never hardware interfaces.
            if time.time() - last_debug_at > 0.5:
                last_debug_at = time.time()
                with context.lock:
                    debug_following = context.run_control.following
                    debug_visible = context.observation.visible
                    debug_status = dict(context.control_status)
                    processing_ms = context.observation.processing_ms
                    hardware_time = context.observation.hardware_time
                print(format_debug(DebugFrame(
                    fps=fps, track=track, following=debug_following,
                    visible=debug_visible, status=debug_status,
                    pinch_px=context.gripper_mapper.pinch_px, gripper_deg=context.gripper_mapper.last_sent,
                    grab_ms=ms_grab, detect_ms=ms_det, processing_ms=processing_ms,
                    hardware_time=hardware_time, dropped_frames=getattr(cam, 'dropped_frames', 0))))

            if not context.options.headless:
                with context.lock:
                    following = context.run_control.following
                    status = dict(context.control_status)
                key = vision_display.show(DisplayFrame(
                    image=color_image, track=track, landmarks=landmarks,
                    pinch_points=pinch_pts, infrared=context.options.infrared,
                    desired_u=context.options.desired_u, desired_v=context.options.desired_v,
                    gripper_enabled=context.options.gripper_enabled,
                    gripper_closed_deg=context.gripper_mapper.closed_deg, gripper_open_deg=context.gripper_mapper.open_deg,
                    gripper_deg=context.gripper_mapper.last_sent, gripper_frozen=context.gripper_mapper.frozen,
                    no_arm=context.options.no_arm, follow_enabled=context.options.follow_enabled,
                    following=following, status=status, fps=fps))
                if key == ord('q'):
                    context.run_control.quit_requested.set()
                if key in (ord('r'), ord('R')) and context.request_start is not None:
                    # Explicit start is allowed again after a safety pause.
                    with context.lock:
                        context.run_control.following = (context.options.follow_enabled
                                                         and not context.run_control.servo_failed)
                        context.run_control.gesture_start_allowed = False
                if key in (ord('g'), ord(' ')):
                    context.pause_tracking()
            else:
                time.sleep(0.005)   # headless：让出 CPU，等待键盘事件

    except KeyboardInterrupt:
        print("\n程序被中断（Ctrl+C），走统一退出路径...")
