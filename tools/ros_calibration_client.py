"""Client for the existing ROS arm controller; never opens CAN itself."""
import json
import os
from pathlib import Path
import select
import subprocess
import time

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
ROS_ENV = os.environ.get('REBOT_ROS_ENV')


class RosCalibrationArm:
    is_ros_client = True
    def __init__(self):
        if not ROS_ENV or not Path(ROS_ENV).is_file():
            raise RuntimeError('ROS 标定模式需要将 REBOT_ROS_ENV 设置为已安装 ROS 工作空间的 setup.bash；普通真机跟随不需要 ROS')
        import pinocchio as pin
        from rebot_adapter import URDF_PATH, B601_RS_JOINT_NAMES, JOINT_LIMIT_MARGIN
        self.model = pin.buildModelFromUrdf(URDF_PATH)
        self.data = self.model.createData()
        self.joint_names = B601_RS_JOINT_NAMES
        ids = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.joint_names]
        self.q_min = self.model.lowerPositionLimit[ids] + JOINT_LIMIT_MARGIN
        self.q_max = self.model.upperPositionLimit[ids] - JOINT_LIMIT_MARGIN
        self.buffer = b''
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = '/tmp/ros_calibration_logs'
        # Do not inherit the Python 3.10 venv site-packages into ROS Python 3.12.
        env.pop('PYTHONPATH', None)
        env.pop('VIRTUAL_ENV', None)
        self.process = subprocess.Popen(
            ['bash', '-c', 'source "$1"; exec python "$2"', 'ros-bridge',
             ROS_ENV, str(ROOT / 'tools' / 'ros_calibration_bridge.py')],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        try:
            deadline = time.monotonic() + 10
            while True:
                try:
                    self.get_current_pos()
                    break
                except RuntimeError:
                    if time.monotonic() > deadline or self.process.poll() is not None:
                        raise
                    time.sleep(0.2)
        except BaseException:
            self.stop()
            raise

    def _build_q_full(self, q):
        from rebot_adapter import JOINT_OFFSET
        result = np.zeros(self.model.nq)
        for i, name in enumerate(self.joint_names):
            result[self.model.joints[self.model.getJointId(name)].idx_q] = q[i] + JOINT_OFFSET[i]
        return result

    def request(self, obj, timeout=5., poll=None):
        if self.process.poll() is not None:
            raise RuntimeError('ROS 标定客户端已退出')
        self.process.stdin.write((json.dumps(obj, allow_nan=False)+'\n').encode())
        self.process.stdin.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if poll:
                poll()
            if b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                try:
                    response = json.loads(line)
                except ValueError as exc:
                    raise RuntimeError(f'ROS 响应无效：{line!r}') from exc
                if not response.get('ok'):
                    raise RuntimeError('ROS 控制器：' + response.get('error', 'unknown error'))
                return response
            ready, _, _ = select.select([self.process.stdout], [], [], 0.05)
            if ready:
                data = os.read(self.process.stdout.fileno(), 4096)
                if not data:
                    raise RuntimeError('ROS 标定客户端连接已断开')
                self.buffer += data
        raise RuntimeError('ROS 控制器响应超时')

    def get_current_pos(self):
        return np.asarray(self.request({'op': 'state'})['positions'], dtype=float)

    def move(self, target, current, speed_deg_s, poll):
        # One zero-end-velocity cubic Hermite segment has peak speed 1.5*d/T.
        # Include the maximum allowed state discrepancy at acceptance time.
        duration = max(0.1, 1.5 * (np.max(np.abs(target-current)) + np.deg2rad(0.5)) / np.deg2rad(speed_deg_s))
        result = self.request({'op': 'move', 'target': target.tolist(),
                               'expected': current.tolist(), 'duration': duration},
                              timeout=duration+10., poll=poll)
        actual = np.asarray(result['positions'])
        if np.max(np.abs(actual-target)) > np.deg2rad(1.):
            raise RuntimeError('机械臂未到达目标，停止采样')
        # The ROS action can return before a loaded arm has fully settled.
        # Require consecutive actual feedback readings to agree before pairing
        # an image with a joint pose.
        deadline = time.monotonic() + 8.
        recent = []
        while time.monotonic() < deadline:
            poll()
            time.sleep(0.25)
            actual = self.get_current_pos()
            recent.append(actual)
            recent = recent[-5:]
            if len(recent) == 5 and np.max(np.ptp(np.asarray(recent), axis=0)) < np.deg2rad(0.2):
                if np.max(np.abs(actual-target)) > np.deg2rad(1.):
                    raise RuntimeError('机械臂稳定后偏离目标，停止采样')
                return actual
        raise RuntimeError('机械臂未在时限内稳定，停止采样')

    def stop(self):
        # Closing the client cancels an active action; never shut down ROS,
        # since the hardware controller homes the arm on shutdown.
        try:
            if self.process.poll() is None:
                self.process.stdin.close()
                self.process.wait(timeout=6)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            if self.process.poll() is None:
                self.process.terminate()
            raise RuntimeError('ROS 标定客户端未能确认停止；请检查机械臂')
