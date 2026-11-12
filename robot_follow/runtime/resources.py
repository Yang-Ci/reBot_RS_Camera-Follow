"""Own runtime resources from acquisition through safe shutdown."""
from __future__ import annotations

import os
import signal
import sys
import time


class RuntimeResources:
    """Release acquired resources exactly once, including on startup failure."""

    def __init__(self):
        self.camera = None
        self.detector = None
        self.robot = None
        self.preview_writer = None
        self.startup_preview = None
        self.keyboard = None
        self.stop_event = None
        self.control_thread = None
        self.feedback_sampler = None
        self.diagnostic_recorder = None
        self.live_publisher = None
        self.previous_handlers = {}
        self.original_sigint_handler = None
        self.web_ready_path = None
        self.web_control = False
        self.close_windows = None
        self._closed = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        errors = self.close()
        if errors and exc is None:
            raise RuntimeError('运行资源清理失败: ' + '; '.join(errors))
        return False

    def close(self):
        if self._closed:
            return []
        self._closed = True
        errors = []

        def attempt(name, action):
            try:
                action()
            except BaseException as exc:
                message = f'{name}: {exc}'
                errors.append(message)
                print(f'[清理] {message}', file=sys.stderr, flush=True)

        if self.web_control:
            for sig in self.previous_handlers:
                attempt('屏蔽网页信号', lambda sig=sig: signal.signal(sig, signal.SIG_IGN))
            if self.web_ready_path:
                def remove_ready():
                    try:
                        os.unlink(self.web_ready_path)
                    except FileNotFoundError:
                        pass
                attempt('移除就绪标记', remove_ready)
        if self.stop_event is not None:
            attempt('停止伺服线程', self.stop_event.set)
        if self.preview_writer is not None:
            attempt('停止预览写入', self.preview_writer.stop_event.set)
        if self.control_thread is not None and self.control_thread.ident is not None:
            def join_control():
                self.control_thread.join(timeout=1.0)
                if self.control_thread.is_alive():
                    raise RuntimeError('等待超时，伺服线程仍在运行')
            attempt('等待伺服线程', join_control)
        if self.live_publisher is not None:
            attempt('停止实时状态', self.live_publisher.close)
        # CAN feedback reads must stop before the arm's return-to-zero sequence.
        if self.feedback_sampler is not None:
            attempt('停止关节反馈', self.feedback_sampler.close)
        if self.diagnostic_recorder is not None:
            def close_diagnostics():
                recorder = self.diagnostic_recorder
                recorder.close()
                print(f'[诊断] 写入 {recorder.written} 行，队列丢弃 {recorder.dropped} 行'
                      + (f'，错误={recorder.error}' if recorder.error else ''))
            attempt('关闭诊断日志', close_diagnostics)
        if self.keyboard is not None:
            attempt('停止键盘监听', self.keyboard.stop)
        if self.detector is not None:
            attempt('关闭手部检测器', self.detector.close)
        if self.robot is not None:
            def hold_robot():
                if not self.robot.is_stopping():
                    print('机械臂保持当前位置...')
                    self.robot.hold()
                    time.sleep(0.3)
            attempt('保持机械臂', hold_robot)
            print('正在安全回零（约3秒），窗口短暂无响应属正常现象，请勿重复按键...')
            attempt('机械臂回零失能', self.robot.stop)
        if self.startup_preview is not None:
            attempt('停止启动预览', self.startup_preview.stop)
        if self.preview_writer is not None:
            attempt('关闭预览', self.preview_writer.close)
        if self.close_windows is not None:
            attempt('关闭视觉窗口', self.close_windows)
        if self.camera is not None:
            attempt('关闭相机', self.camera.stop)
        if self.web_control:
            for sig, handler in self.previous_handlers.items():
                attempt('恢复网页信号', lambda sig=sig, handler=handler: signal.signal(sig, handler))
            if self.original_sigint_handler is not None:
                attempt('恢复中断信号',
                        lambda: signal.signal(signal.SIGINT, self.original_sigint_handler))
        print('程序退出')
        return errors
