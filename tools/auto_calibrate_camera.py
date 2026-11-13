#!/usr/bin/env python3
"""D435i/D430i, Gemini 2 or Gemini 336 eye-in-hand calibration.

No hardware is accessed when importing this module. `--check` only opens the
camera. `--run` enables the arm and automatically collects calibration poses.
"""
import argparse
import json
import os
import signal
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
from hand_eye_solver import calibrate_hand_eye

MARKER_SIZE = 0.100
# The connected D430i's tabletop view produced stable detections around
# 109 pixels. Smaller detections still face reprojection, stability, and
# calibration-residual checks below.
MIN_SIDE = 95.0
MAX_REPROJECTION_PX = 2.0
MIN_SAMPLES = 12
POSE_COUNT = 24
MAX_PERTURB = np.deg2rad(np.array([2., 2., 2., 3., 3., 4.]))
MAX_STEP = np.deg2rad(7.0)
PLAN_MAX_STEP = np.deg2rad(3.0)  # smaller visual displacement plus tracking margin
MOVE_SPEED = 4.0
# Existing project limits, inset for the camera bracket. These apply to the
# flange only and cannot detect collisions of other arm links.
BOX_MIN = np.array([0.21, -0.06, 0.24])
BOX_MAX = np.array([0.36, 0.24, 0.39])
OUTPUT = ROOT / 'config' / 'hand_eye_calibration_d435i.json'


@dataclass
class Observation:
    T: np.ndarray
    width: float
    error: float
    center: tuple


class MarkerUnusable(RuntimeError):
    """A visible marker is unsuitable for a calibration sample."""


class MarkerLost(RuntimeError):
    """Camera frames are arriving but the calibration marker left the view."""


class MarkerDetector:
    def __init__(self, K, distortion, min_side=MIN_SIDE):
        self.K = np.asarray(K, dtype=np.float64)
        self.distortion = np.asarray(distortion, dtype=np.float64)
        self.min_side = float(min_side)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        s = MARKER_SIZE / 2
        self.object_points = np.array([[-s, s, 0], [s, s, 0],
                                       [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

    def detect(self, image):
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None:
            raise ValueError('未检测到 ArUco 4X4_50 ID 0')
        matches = np.flatnonzero(np.asarray(ids).ravel() == 0)
        if len(matches) != 1:
            raise ValueError('画面中必须有且只有一个 ID 0 标记')
        pts = corners[int(matches[0])].reshape(4, 2).astype(np.float32)
        height, width = image.shape[:2]
        if np.any(pts[:, 0] < 5) or np.any(pts[:, 0] >= width-5) or \
           np.any(pts[:, 1] < 5) or np.any(pts[:, 1] >= height-5):
            raise ValueError('标记太靠近画面边缘')
        side = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
        if side < self.min_side:
            raise ValueError(f'标记过小：{side:.1f}px，至少 {self.min_side:.0f}px')
        ok, rv, tv = cv2.solvePnP(self.object_points, pts, self.K, self.distortion,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            raise ValueError('标记位姿求解失败')
        rotation = cv2.Rodrigues(rv)[0]
        position = np.asarray(tv).reshape(3)
        proj = cv2.projectPoints(self.object_points, rv, tv, self.K, self.distortion)[0].reshape(4, 2)
        err = float(np.sqrt(np.mean(np.sum((pts - proj)**2, axis=1))))
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)) \
                or not np.isfinite(err) or not 0.15 <= position[2] <= 0.8 \
                or err > MAX_REPROJECTION_PX:
            raise ValueError(f'标记位姿无效或重投影误差过大：{err:.2f}px')
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rotation, position
        return Observation(T, side, err, tuple(pts.mean(axis=0)))


def rotation_angle(R):
    return float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def mean_observation(observations):
    if len(observations) < 5:
        raise ValueError('连续有效标记帧不足')
    translations = np.array([obs.T[:3, 3] for obs in observations])
    mean = translations.mean(axis=0)
    if np.max(np.linalg.norm(translations - mean, axis=1)) > 0.008:
        raise ValueError('标记位置不稳定')
    # Orthogonal projection of the mean rotation back onto SO(3).
    U, _, Vt = np.linalg.svd(sum(o.T[:3, :3] for o in observations))
    D = np.diag([1, 1, np.linalg.det(U @ Vt)])
    rotation = U @ D @ Vt
    if max(rotation_angle(rotation.T @ o.T[:3, :3]) for o in observations) > np.deg2rad(4):
        raise ValueError('标记角度不稳定')
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation, mean
    return T


def within_bounds(q, q_min, q_max):
    return bool(np.all(np.isfinite(q)) and np.all(q >= q_min) and np.all(q <= q_max))


def segment_safe(q0, q1, q_min, q_max, fk):
    if not within_bounds(q0, q_min, q_max) or not within_bounds(q1, q_min, q_max):
        return False
    count = max(2, int(np.ceil(np.max(np.abs(q1-q0)) / np.deg2rad(0.5))) + 1)
    # The controller advances all joints at the same speed limit independently;
    # small moves finish first, so its path is not generally a straight line.
    delta = q1 - q0
    distance = np.max(np.abs(delta))
    for t in np.linspace(0, 1, count):
        # Direct SDK ramps axes independently; ROS uses a synchronized cubic
        # segment. Verify both paths so the same plan is valid for either.
        paths = (q0 + np.sign(delta) * np.minimum(np.abs(delta), t * distance),
                 q0 + t * delta)
        for q in paths:
            position = np.asarray(fk(q))
            if position.shape != (3,) or not np.all(np.isfinite(position)) or \
                    np.any(position < BOX_MIN) or np.any(position > BOX_MAX):
                return False
    return True


def make_plan(start, q_min, q_max, fk, count=POSE_COUNT):
    start, q_min, q_max = map(lambda q: np.asarray(q, dtype=float), (start, q_min, q_max))
    if not segment_safe(start, start, q_min, q_max, fk):
        raise ValueError('当前关节角或法兰位置不在软件安全范围内；请先调整起始姿态')
    rng = np.random.default_rng(1701)
    current = start.copy()
    poses = []
    for _ in range(4000):
        target = start + rng.uniform(-1.0, 1.0, 6) * MAX_PERTURB
        if np.max(np.abs(target-current)) > PLAN_MAX_STEP:
            continue
        if not segment_safe(current, target, q_min, q_max, fk):
            continue
        if not segment_safe(target, start, q_min, q_max, fk):
            continue
        poses.append(target)
        current = target
        if len(poses) == count:
            return poses
    raise ValueError(f'仅找到 {len(poses)} 个有效姿态，起点可能太靠近安全边界')


def evaluate_calibration(robot_poses, marker_poses):
    if len(robot_poses) < MIN_SAMPLES or len(robot_poses) != len(marker_poses):
        raise ValueError(f'有效样本少于 {MIN_SAMPLES} 组')
    base0 = robot_poses[0]
    vectors = np.array([cv2.Rodrigues(base0[:3, :3].T @ p[:3, :3])[0].ravel()
                        for p in robot_poses[1:]])
    singular = np.linalg.svd(vectors, compute_uv=False)
    if len(singular) < 3 or singular[0] < 0.05 or singular[2] < singular[0] * 0.05:
        raise ValueError('采样旋转方向不足，标定结果不可观测')
    T, method = calibrate_hand_eye([p[:3, :3] for p in robot_poses],
                                   [p[:3, 3] for p in robot_poses],
                                   [p[:3, :3] for p in marker_poses],
                                   [p[:3, 3] for p in marker_poses])
    if T.shape != (4, 4) or not np.all(np.isfinite(T)) or \
       not np.allclose(T[3], [0, 0, 0, 1]) or \
       not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-3) or \
       abs(np.linalg.det(T[:3, :3])-1) > 1e-3 or np.linalg.norm(T[:3, 3]) > 0.5:
        raise ValueError('求解出的手眼变换无效')
    fixed = [a @ T @ b for a, b in zip(robot_poses, marker_poses)]
    positions = np.array([p[:3, 3] for p in fixed])
    spread = float(np.linalg.norm(positions.std(axis=0)))
    angular = np.array([rotation_angle(fixed[0][:3, :3].T @ p[:3, :3]) for p in fixed])
    angle_med = float(np.rad2deg(np.median(angular)))
    if spread > 0.02 or angle_med > 3.0:
        raise ValueError(f'标定质量不合格：位置分散 {spread*100:.2f}cm，角度中位差 {angle_med:.2f}°')
    return T, {'method': method, 'num_samples': len(fixed),
               'residual_spread_cm': round(spread*100, 3),
               'marker_rotation_median_deg': round(angle_med, 3),
               'rotation_excitation_singular_values_rad': singular.tolist()}


def write_result(path, data):
    """Atomically create a new result without overwriting an existing calibration."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix='.handeye-', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.link(temp_name, path)  # atomic; fails if target already exists
    finally:
        os.unlink(temp_name)


def window_stopped(name):
    """Q/Esc works with window focus; closing the window also requests stop."""
    key = cv2.waitKey(1) & 0xff
    if key in (ord('q'), ord('Q'), 27):
        return True
    try:
        return cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def observe(camera, detector, timeout=5.0, show=True):
    end = time.monotonic() + timeout
    samples = []
    last_error = '相机无有效帧'
    frame_count = 0
    valid_count = 0
    while time.monotonic() < end:
        image, _, _ = camera.get_frames()
        if image is None:
            samples.clear()
            continue
        frame_count += 1
        try:
            observation = detector.detect(image)
            valid_count += 1
            samples.append(observation)
            last_error = ''
        except ValueError as exc:
            samples.clear()
            last_error = str(exc)
        if show:
            canvas = image.copy()
            label = (f'ID=0 w={observation.width:.0f}px z={observation.T[2,3]:.3f}m'
                     if not last_error else 'Marker invalid or too small')
            cv2.putText(canvas, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0) if not last_error else (0, 0, 255), 2)
            cv2.imshow('Camera Auto Calibration (q = stop)', canvas)
            if window_stopped('Camera Auto Calibration (q = stop)'):
                raise KeyboardInterrupt
        if len(samples) >= 5:
            try:
                T = mean_observation(samples[-5:])
                return T, samples[-1]
            except ValueError as exc:
                last_error = str(exc)
                samples = samples[-4:]
    if frame_count >= 20 and ((valid_count > 0 and not last_error) or last_error.startswith(
            ('标记过小', '标记位姿无效或重投影误差过大', '标记位置不稳定', '标记角度不稳定'))):
        if not last_error:
            last_error = '连续稳定帧不足'
        raise MarkerUnusable(f'标记检测失败：{last_error}')
    if frame_count >= 20 and last_error.startswith(('未检测到 ArUco', '标记太靠近画面边缘')):
        raise MarkerLost(f'标记检测失败：{last_error}')
    raise RuntimeError(f'标记检测失败：{last_error}')


def preview(camera, detector):
    """Keep a camera-only view open so the user can position the marker."""
    name = 'D435i Marker Preview (Q to quit)'
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, 960, 720)
    while True:
        image, _, _ = camera.get_frames()
        if image is None:
            if window_stopped(name):
                return
            continue
        canvas = image.copy()
        try:
            obs = detector.detect(image)
            text = f'ID 0 OK   width={obs.width:.0f}px   distance={obs.T[2,3]:.3f}m'
            color = (0, 255, 0)
            cv2.drawMarker(canvas, (int(obs.center[0]), int(obs.center[1])),
                           color, cv2.MARKER_CROSS, 24, 2)
        except ValueError as exc:
            message = str(exc)
            text = (f'MARKER TOO SMALL: MOVE CLOSER (min {MIN_SIDE:.0f}px)' if '过小' in message
                    else 'MARKER NOT READY: center ID 0 in the image')
            color = (0, 0, 255)
        cv2.putText(canvas, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cv2.putText(canvas, 'Q / Esc = quit   Camera only', (8, image.shape[0]-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow(name, canvas)
        if window_stopped(name):
            return


def observe_start(camera, detector, *, show, allow_small):
    try:
        T, info = observe(camera, detector, show=show)
        return T, info, True
    except MarkerUnusable as exc:
        if not allow_small or '标记过小' not in str(exc):
            raise
        # A smaller but detectable, stable marker only confirms visibility;
        # it is never included in the calibration data.
        tracking = MarkerDetector(camera.camera_matrix, camera.dist_coeffs, min_side=80.)
        T, info = observe(camera, tracking, show=show)
        print(f'[提示] 起点标记宽 {info.width:.1f}px，仅用于确认可见，不计入样本', flush=True)
        return T, info, False


def robot_fk(robot, q):
    import pinocchio as pin
    pin.forwardKinematics(robot.model, robot.data, robot._build_q_full(q))
    pin.updateFramePlacements(robot.model, robot.data)
    jid = robot.model.getJointId(robot.joint_names[-1])
    pose = np.eye(4)
    pose[:3, :3] = robot.data.oMi[jid].rotation
    pose[:3, 3] = robot.data.oMi[jid].translation
    return pose


def check_quit(robot, show):
    if getattr(robot, 'is_ros_client', False):
        if robot.process.poll() is not None:
            raise RuntimeError('ROS 标定客户端已退出')
        if show and window_stopped('Camera Auto Calibration (q = stop)'):
            raise KeyboardInterrupt
        return
    arm = robot.io.arm
    if arm.shutdown_started or arm.is_stopped:
        raise RuntimeError('机械臂已进入停止流程')
    if getattr(arm, 'last_error', None):
        raise RuntimeError(f'控制线程错误：{arm.last_error}')
    if show and window_stopped('Camera Auto Calibration (q = stop)'):
        raise KeyboardInterrupt


def move_and_wait(robot, target, show, validated_start=None):
    if getattr(robot, 'is_ros_client', False):
        return robot.move(target, validated_start, MOVE_SPEED, lambda: check_quit(robot, show))
    # Do not use the adapter's wait=True: it can silently time out.
    robot.move_to_joints(target, speed_deg_s=MOVE_SPEED, wait=False)
    end = time.monotonic() + 20
    while time.monotonic() < end:
        check_quit(robot, show)
        if np.max(np.abs(robot.get_command_pos()-target)) < np.deg2rad(0.5):
            actual = robot.get_current_pos()
            if np.max(np.abs(actual-target)) < np.deg2rad(1.0):
                time.sleep(0.5)
                return actual
        time.sleep(0.05)
    raise RuntimeError('机械臂未在时限内到达目标，停止采样')


def main():
    parser = argparse.ArgumentParser(description='当前姿态自动手眼标定')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true', help='只检查相机和标记，不连接机械臂')
    mode.add_argument('--preview', action='store_true', help='持续显示标记画面，不连接机械臂')
    mode.add_argument('--preflight', action='store_true', help='只读检查相机和采样计划，不执行运动（须配合 --ros）')
    mode.add_argument('--run', action='store_true', help='使能机械臂并自动采样')
    parser.add_argument('--headless', action='store_true', help='不显示画面；使用 Ctrl+C 停止')
    parser.add_argument('--ros', action='store_true', help='复用已启动的 ROS 机械臂控制器，不直接连接 CAN')
    parser.add_argument('--camera', choices=['d435i', 'gemini2', 'gemini336'], default='d435i', help='相机型号')
    parser.add_argument('--output', type=Path, default=None, help='标定结果路径（禁止覆盖已有文件）')
    parser.add_argument('--box-min-x', type=float, default=None,
                        help='本次标定的法兰 x 坐标下限（米）；须现场确认机械臂及附件的完整运动路径')
    parser.add_argument('--pose-span-scale', type=float, default=1.0,
                        help='放大围绕起始姿态的采样范围，范围 1.0–2.0；不改变单步或速度限制')
    args = parser.parse_args()
    if not np.isfinite(args.pose_span_scale) or not 1.0 <= args.pose_span_scale <= 2.0:
        parser.error('--pose-span-scale 必须在 1.0 到 2.0 之间')
    if args.pose_span_scale != 1.0:
        MAX_PERTURB[:] *= args.pose_span_scale
        print(f'[范围] 相对起点各关节最大偏移 {np.round(np.degrees(MAX_PERTURB), 1).tolist()}°；单步和速度限制不变', flush=True)
    if args.box_min_x is not None:
        if not np.isfinite(args.box_min_x) or not 0.0 <= args.box_min_x < BOX_MAX[0]:
            parser.error('--box-min-x 必须是非负有限数且小于 x 上限')
        BOX_MIN[0] = args.box_min_x
        print(f'[范围] 本次法兰 x 下限 {BOX_MIN[0]:.3f}m；软件不检查整臂和附件碰撞', flush=True)
    if args.output is None:
        args.output = {
            'd435i': OUTPUT,
            'gemini2': ROOT / 'config' / 'hand_eye_calibration.json',
            'gemini336': ROOT / 'config' / 'hand_eye_calibration_gemini336.json',
        }[args.camera]
    if args.preview and args.headless:
        parser.error('--preview 需要图形窗口，不能与 --headless 一起使用')
    if args.preflight and not args.ros:
        parser.error('--preflight 须配合 --ros')
    if args.run and args.output.exists():
        raise FileExistsError(f'结果已存在，不覆盖：{args.output}；可用 --output 指定新路径')
    camera = None
    robot = None
    old_handlers = {}
    try:
        if args.camera == 'd435i':
            from realsense_camera import RealSenseCamera
            camera = RealSenseCamera(fps=30, ir_emitter='on')
        else:
            from gemini2_camera import Gemini2Camera
            camera = Gemini2Camera(fps=30, color_exposure_ms=0)
            device_name = camera.device_name.lower()
            if args.camera == 'gemini336' and 'gemini 336' not in device_name:
                raise ValueError(f'所选相机为 Gemini 336，实际设备为 {camera.device_name}')
            if args.camera == 'gemini2' and ('gemini 2' not in device_name or 'gemini 336' in device_name):
                raise ValueError(f'所选相机为 Gemini 2，实际设备为 {camera.device_name}')
        detector = MarkerDetector(camera.camera_matrix, camera.dist_coeffs)
        if args.preview:
            print('[预览] 相机画面将持续显示；q / Esc 退出，不连接机械臂', flush=True)
            preview(camera, detector)
            return
        if not args.headless:
            cv2.namedWindow('Camera Auto Calibration (q = stop)', cv2.WINDOW_NORMAL)
        first, info, usable_start = observe_start(camera, detector, show=not args.headless,
                                                  allow_small=args.run or args.preflight)
        print(f'[相机] 标记 ID=0，宽 {info.width:.1f}px，z={first[2,3]:.3f}m，'
              f'重投影误差 {info.error:.2f}px', flush=True)
        if args.check:
            print('[检查通过] 未连接机械臂')
            return
        from rebot_adapter import RebotAdapter, JOINT_OFFSET
        if args.ros:
            from ros_calibration_client import RosCalibrationArm
            robot = RosCalibrationArm()
        else:
            robot = RebotAdapter()
        # The SDK installs signal handlers that can stop motors without ending
        # the image loop. Install ours after startup so cleanup is always reached.
        def interrupt(_signum, _frame):
            raise KeyboardInterrupt
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, interrupt)
        check_quit(robot, not args.headless)
        start = robot.get_current_pos()
        # Re-sample after connecting the arm; the camera frame captured before
        # enabling must not be paired with a later joint pose.
        first, info, usable_start = observe_start(camera, detector, show=not args.headless,
                                                  allow_small=True)
        if np.max(np.abs(robot.get_current_pos() - start)) > np.deg2rad(0.5):
            raise RuntimeError('起始采样期间机械臂发生移动')
        fk = lambda q: robot_fk(robot, q)[:3, 3].copy()
        plan = make_plan(start, robot.q_min, robot.q_max, fk)
        if args.preflight:
            print(f'[预检通过] {len(plan)} 个姿态；未发送运动指令', flush=True)
            return
        print(f'[预检] {len(plan)} 个姿态，速度 {MOVE_SPEED:.1f}°/s，开始自动采样', flush=True)
        robot_poses = [robot_fk(robot, start)] if usable_start else []
        marker_poses = [first] if usable_start else []
        for i, target in enumerate(plan, 1):
            check_quit(robot, not args.headless)
            actual_before = robot.get_current_pos()
            if np.max(np.abs(actual_before-target)) > MAX_STEP or \
                    not segment_safe(actual_before, target, robot.q_min, robot.q_max, fk):
                raise RuntimeError('实际起点与计划不一致或路径超出安全范围')
            actual = move_and_wait(robot, target, not args.headless, actual_before)
            try:
                marker, obs = observe(camera, detector, show=not args.headless)
            except MarkerUnusable as exc:
                print(f'[跳过 {i}/{len(plan)}] {exc}', flush=True)
                continue
            except MarkerLost as exc:
                # Stop moving immediately. Previously accepted samples are
                # useful only when the unchanged quality gates accept them.
                if len(robot_poses) < MIN_SAMPLES:
                    raise
                print(f'[提前结束] {exc}；对已有 {len(robot_poses)} 组样本进行质量检查', flush=True)
                break
            actual_after = robot.get_current_pos()
            if np.max(np.abs(actual_after-actual)) > np.deg2rad(0.5):
                raise RuntimeError('采样期间机械臂发生移动')
            robot_poses.append(robot_fk(robot, actual_after))
            marker_poses.append(marker)
            print(f'[采样 {i}/{len(plan)}] w={obs.width:.0f}px z={marker[2,3]:.3f}m', flush=True)
        # Preserve the measurements even when the quality gate rejects the
        # solution, so the failure can be diagnosed without moving the arm again.
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        samples_path = args.output.with_name(f'{args.output.stem}_samples_{stamp}.npz')
        samples_path.parent.mkdir(parents=True, exist_ok=True)
        with samples_path.open('xb') as samples_file:
            np.savez_compressed(samples_file, robot_poses=np.asarray(robot_poses),
                                marker_poses=np.asarray(marker_poses),
                                joint_offset_rad=np.asarray(JOINT_OFFSET),
                                camera_matrix=np.asarray(camera.camera_matrix),
                                dist_coeffs=np.asarray(camera.dist_coeffs),
                                camera_serial=str(getattr(camera, 'camera_serial', '')),
                                image_size=np.asarray([camera.width, camera.height]))
        print(f'[样本] 已保存 {samples_path}', flush=True)
        T, metrics = evaluate_calibration(robot_poses, marker_poses)
        if args.camera == 'd435i':
            device = camera.pipeline.get_active_profile().get_device()
            camera_serial = device.get_info(camera.rs.camera_info.serial_number)
            camera_name = 'RealSense D435i/D430i'
            image_stream = 'color' if camera.has_color else 'infrared_1'
        else:
            camera_serial = camera.camera_serial
            camera_name = 'Orbbec Gemini 336' if args.camera == 'gemini336' else 'Orbbec Gemini 2'
            image_stream = 'color'
        data = {'T_tcp_camera': T.tolist(), 'timestamp': datetime.now(timezone.utc).isoformat(),
                'camera': camera_name, 'camera_serial': camera_serial,
                'image_stream': image_stream, 'image_size': [camera.width, camera.height],
                'camera_matrix': np.asarray(camera.camera_matrix).tolist(),
                'dist_coeffs': np.asarray(camera.dist_coeffs).tolist(),
                'marker': 'ArUco DICT_4X4_50 ID 0', 'marker_size_m': MARKER_SIZE,
                'joint_offset_rad': np.asarray(JOINT_OFFSET).tolist(),
                'flange_box_min_m': BOX_MIN.tolist(), 'flange_box_max_m': BOX_MAX.tolist(),
                'pose_perturb_limit_deg': np.degrees(MAX_PERTURB).tolist(), **metrics}
        # Use a separate, no-overwrite file to avoid silently replacing any prior result.
        write_result(args.output, data)
        print(f'[完成] 已保存 {args.output}，位置分散 {metrics["residual_spread_cm"]:.3f}cm')
    finally:
        if robot is not None:
            if getattr(robot, 'is_ros_client', False):
                print('[退出] 停止标定客户端；机械臂由现有 ROS 控制器保持', flush=True)
            else:
                print('[退出] 程序将调用控制器回零并失能；确保回零路径畅通', flush=True)
                try:
                    robot.hold()
                except Exception as exc:
                    print(f'[退出] 保持失败：{exc}', file=sys.stderr)
            robot.stop()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        if camera is not None:
            camera.stop()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n标定已停止，请查看上方退出状态。', file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f'自动标定失败：{exc}', file=sys.stderr)
        sys.exit(1)
