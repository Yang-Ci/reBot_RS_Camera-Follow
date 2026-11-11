#!/usr/bin/env python3
"""D435i RGB and depth camera, with depth aligned to the color stream."""
import numpy as np
import cv2


class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30, align_to_color=True,
                 color_exposure_ms=0.0, ir_emitter="on"):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise ImportError("缺少 pyrealsense2：请安装 requirements-d435i.txt") from exc
        if not align_to_color:
            raise ValueError("本项目要求深度对齐到彩色图")
        if ir_emitter not in ("on", "off"):
            raise ValueError("ir_emitter 必须为 on 或 off")
        self.rs = rs
        self._emitter_sensor = None
        self._emitter_original = None
        self.pipeline = rs.pipeline()
        config = rs.config()
        devices = rs.context().query_devices()
        if not devices:
            raise RuntimeError("未发现 RealSense 相机，请检查 USB 连接和设备权限")
        device = devices[0]
        config.enable_device(device.get_info(rs.camera_info.serial_number))
        self.has_color = any(
            p.stream_type() == rs.stream.color
            for sensor in device.query_sensors() for p in sensor.get_stream_profiles())
        self.image_stream = rs.stream.color if self.has_color else rs.stream.infrared
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        if self.has_color:
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        else:
            config.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
            print("[RealSense] 没有 RGB 传感器，使用左红外画面；手部识别效果需实测")
        try:
            profile = self.pipeline.start(config)
        except RuntimeError as exc:
            raise RuntimeError(f"D435i 启动失败（{width}x{height}@{fps}）：{exc}。"
                               "请检查 USB 连接、权限和流配置；可尝试 --fps 30") from exc
        try:
            device = profile.get_device()
            if not self.has_color and ir_emitter == "off":
                sensor = device.first_depth_sensor()
                if not sensor.supports(rs.option.emitter_enabled):
                    raise RuntimeError("相机不支持控制红外投影，请使用 --ir-emitter on")
                self._emitter_sensor = sensor
                self._emitter_original = sensor.get_option(rs.option.emitter_enabled)
                sensor.set_option(rs.option.emitter_enabled, 0)
                print("[RealSense] 红外投影已关闭（仅视觉测试；深度质量可能下降）")
            intr = profile.get_stream(self.image_stream, 0 if self.has_color else 1).as_video_stream_profile().get_intrinsics()
            self.width, self.height = intr.width, intr.height
            self.fx, self.fy, self.cx, self.cy = intr.fx, intr.fy, intr.ppx, intr.ppy
            self.camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                                           [0, intr.fy, intr.ppy],
                                           [0, 0, 1]], dtype=np.float64)
            # D435i color streams normally use Brown-Conrady coefficients.
            if intr.model not in (rs.distortion.none, rs.distortion.brown_conrady,
                                  rs.distortion.modified_brown_conrady):
                raise RuntimeError(f"不支持的彩色畸变模型: {intr.model}")
            self.dist_coeffs = np.array(intr.coeffs if intr.model != rs.distortion.none
                                        else [0.] * 5, dtype=np.float64)
            self.depth_scale = device.first_depth_sensor().get_depth_scale()
            self.align = rs.align(self.image_stream)
            self.dropped_frames = 0
            self._last_frame_timestamp_s = None
            self._timestamp_source = ""
            if color_exposure_ms > 0 and not self.has_color:
                raise ValueError("红外回退模式不支持 --color-exposure，请使用 0")
            if color_exposure_ms > 0:
                sensor = next((s for s in device.query_sensors()
                               if s.supports(rs.option.enable_auto_exposure)
                               and s.supports(rs.option.exposure)
                               and s.get_info(rs.camera_info.name) == 'RGB Camera'), None)
                if sensor is None:
                    raise RuntimeError("无法设置 RGB 曝光，请使用 --color-exposure 0")
                sensor.set_option(rs.option.enable_auto_exposure, 0)
                sensor.set_option(rs.option.exposure, color_exposure_ms * 1000.0)
            print(f"[D435i] {self.width}x{self.height}@{fps}，深度单位={self.depth_scale:.6f}m，"
                  f"fx={self.fx:.2f} fy={self.fy:.2f}")
        except Exception:
            self._restore_emitter()
            self.pipeline.stop()
            raise

    def get_frames(self, latest_only=True):
        try:
            frames = self.pipeline.wait_for_frames(1000)
            if latest_only:
                while True:
                    newer = self.pipeline.poll_for_frames()
                    if not newer:
                        break
                    frames = newer
                    self.dropped_frames += 1
            frames = self.align.process(frames)
            color = (frames.get_color_frame() if self.has_color
                     else frames.get_infrared_frame(1))
            depth = frames.get_depth_frame()
            if not color or not depth:
                return None, None, None
            self._last_frame_timestamp_s = color.get_timestamp() * 0.001
            self._timestamp_source = str(color.get_frame_timestamp_domain())
            image = np.asanyarray(color.get_data()).copy()
            if not self.has_color:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            return (image, np.asanyarray(depth.get_data()).copy(), self.depth_scale)
        except RuntimeError:
            self._last_frame_timestamp_s = None
            return None, None, None

    def get_last_frame_timing(self):
        return self._last_frame_timestamp_s, self._timestamp_source

    def _restore_emitter(self):
        if self._emitter_sensor is not None and self._emitter_original is not None:
            try:
                self._emitter_sensor.set_option(self.rs.option.emitter_enabled,
                                                self._emitter_original)
            except RuntimeError as exc:
                print(f"[RealSense] 恢复投影设置失败: {exc}")
            finally:
                self._emitter_sensor = None
                self._emitter_original = None

    def stop(self):
        self._restore_emitter()
        self.pipeline.stop()
