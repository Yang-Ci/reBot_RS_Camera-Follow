#!/usr/bin/env python3
"""Run gravity-assisted HOME teaching under the web process manager.

This commands real motors. It must be used only after checking the physical
workspace, support, emergency stop, model, payload, and controller ownership.
"""
from __future__ import annotations

import os
import fcntl
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rebot_adapter import RebotAdapter
from robot_follow.runtime.gravity_session import GravitySession
from robot_follow.runtime.telemetry import LiveTelemetryPublisher, PhaseTelemetry


def write_ready(path, value):
    if not path:
        return
    temporary = Path(str(path) + '.tmp')
    temporary.write_text(value, encoding='ascii')
    os.replace(temporary, path)


def main():
    # An inherited controller lock keeps this child mutually exclusive even
    # if the web server exits unexpectedly. Direct invocation is unsupported.
    try:
        lock_fd = int(os.environ['REBOT_CONTROL_LOCK_FD'])
        os.fstat(lock_fd)
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (KeyError, ValueError, OSError) as exc:
        raise RuntimeError('请从网页启动重力示教，控制器互斥锁不可用') from exc
    ready = os.environ.get('REBOT_WEB_READY_PATH')
    stopping = [False]
    def request_stop(_signum, _frame):
        stopping[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    publisher = LiveTelemetryPublisher.from_environment()
    if publisher:
        publisher.start()
    robot = None
    session = None
    phase = None
    stopped = False
    try:
        robot = RebotAdapter()
        # RebotAdapter's SDK installs its own handlers during initialization.
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, request_stop)
        session = GravitySession(robot, publisher)
        try:
            session.start()
        except Exception as exc:
            # Report before the SDK's return-and-disable cleanup, not seconds later.
            print(f'错误: 重力示教启动检查失败：{exc}；即将回零并失能。', flush=True)
            raise
        write_ready(ready, str(os.getpid()))
        print('[重力示教] 已就绪；手推后静止锁定，退出将回零。', flush=True)
        while True:
            if robot.is_stopping():
                print('[重力示教] 控制器正在停止，等待停机。', flush=True)
                robot.stop()
                stopped = True
                break
            if stopping[0]:
                try:
                    # stop() requires a fresh measured pose before handoff.
                    if publisher:
                        phase = PhaseTelemetry(robot, publisher).start()
                    # Detach and synchronize SDK targets before activating return telemetry.
                    q, _, _ = session._read()
                    session.sender.detach(q)
                    session.attached = False
                    if phase:
                        phase.set_phase('return')
                    robot.stop()
                    stopped = True
                    break
                except Exception as exc:
                    if phase:
                        phase.close()
                        phase = None
                    session.error = str(exc)
                    session.sender.fault.set()
                    print(f'错误: 无法确认安全交接：{exc}；请支撑机械臂并检查硬件急停。', flush=True)
                    if publisher:
                        publisher.offer({'host_time': time.monotonic(), 'phase':'gravity',
                            'fb_valid':0, 'q_cmd_valid':0, 'following':0, 'visible':0,
                            'hold_latch':0, 'servo_failed':1, 'error':'示教无法安全退出：'+str(exc)})
                    stopping[0] = False
                    write_ready(ready, 'stop_failed=' + str(os.getpid()))
            elif not session.error:
                try:
                    session.step()
                except Exception as exc:
                    print(f'错误: 示教控制异常：{exc}；请支撑机械臂并检查硬件急停。', flush=True)
                    write_ready(ready, 'fault=' + str(os.getpid()))
            if session.error:
                session.publish_fault()
            time.sleep(.01 if not session.error else .1)
    finally:
        # Never deliberately exit a live teaching session with stale feedback.
        if session is not None and session.attached:
            session.sender.fault.set()
            print('错误: 示教会话仍在运行，请支撑机械臂并使用硬件急停。', flush=True)
            while session.attached and not robot.is_stopping():
                if stopping[0]:
                    try:
                        session.stop()
                        stopped = True
                        break
                    except Exception:
                        stopping[0] = False
                time.sleep(.1)
        if robot is not None and not stopped and (session is None or not session.attached):
            robot.stop()
        if phase:
            phase.close()
        if publisher:
            publisher.close()
        if ready:
            try:
                Path(ready).unlink()
            except FileNotFoundError:
                pass


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'错误: 重力示教启动失败：{exc}', file=sys.stderr, flush=True)
        sys.exit(1)
