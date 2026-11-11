#!/usr/bin/env python3
"""ROS client for calibration; CAN remains exclusively owned by ROS controller.

JSON lines on stdin/stdout. EOF, SIGINT or SIGTERM cancels an active goal.
This client does not start or shut down the hardware controller.
"""
import json
import math
import os
import queue
import signal
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from rebotarm_msgs.msg import ArmStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

NAMES = [f'joint{i}' for i in range(1, 7)]
requests = queue.Queue()


def reader():
    try:
        for line in sys.stdin:
            try:
                requests.put(json.loads(line))
            except (ValueError, TypeError):
                requests.put({'op': 'invalid'})
    finally:
        requests.put({'op': 'close'})


def reply(value):
    print(json.dumps(value, allow_nan=False), flush=True)


class Bridge(Node):
    def __init__(self):
        super().__init__('calibration_bridge')
        self.positions = None
        self.joint_time = 0.
        self.status = None
        self.status_time = 0.
        self.create_subscription(JointState, '/rebot_arm_rs/joint_states', self.on_joints, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_sub = self.create_subscription(ArmStatus, '/rebot_arm_rs/arm_status', self.on_status, latched)
        self.action = ActionClient(self, FollowJointTrajectory, '/rebot_arm_rs/follow_joint_trajectory')
        self.goal_handle = None
        self.pending = None
        self.result = None
        self.closing = False
        self.deadline = 0.
        self.cancel_sent = False

    def on_joints(self, msg):
        try:
            values = [float(msg.position[list(msg.name).index(n)]) for n in NAMES]
            if all(math.isfinite(v) for v in values):
                self.positions, self.joint_time = values, time.monotonic()
        except (IndexError, ValueError):
            pass

    def on_status(self, msg):
        self.status = {'enabled': msg.enabled, 'active': msg.control_loop_active,
                       'state': msg.state_machine, 'errors': list(msg.error_codes),
                       'joint_status': list(msg.per_joint_status_code)}
        self.status_time = time.monotonic()

    def ready(self):
        if self.positions is None or time.monotonic() - self.joint_time > 1.0:
            raise RuntimeError('joint state unavailable or stale')
        # Status is published only on changes, with transient-local durability.
        if self.status is None or self.status_sub.get_publisher_count() != 1:
            raise RuntimeError('arm status or controller unavailable')
        if (not self.status['enabled'] or not self.status['active'] or
                self.status['state'] != 'IDLE' or self.status['errors'] or
                len(self.status['joint_status']) != 6 or any(self.status['joint_status'])):
            raise RuntimeError(f'arm not ready: {self.status}')
        if not self.action.server_is_ready():
            raise RuntimeError('trajectory action unavailable')

    def command(self, req):
        op = req.get('op')
        if op == 'close':
            self.closing = True
            return
        if self.pending or self.goal_handle:
            reply({'ok': False, 'error': 'goal already active'})
            return
        try:
            if op == 'state':
                self.ready()
                reply({'ok': True, 'positions': self.positions, 'status': self.status})
            elif op == 'move':
                self.ready()
                target = [float(v) for v in req['target']]
                expected = [float(v) for v in req['expected']]
                duration = float(req['duration'])
                if len(target) != 6 or len(expected) != 6 or not all(math.isfinite(v) for v in target + expected) or not math.isfinite(duration) or duration < 0.1 or duration > 20:
                    raise ValueError('invalid target or duration')
                if max(abs(a-b) for a,b in zip(expected, self.positions)) > math.radians(0.5):
                    raise RuntimeError('arm moved since the path was checked')
                if max(abs(a-b) for a,b in zip(target, self.positions)) > math.radians(7):
                    raise RuntimeError('step exceeds 7 degrees')
                if not self.action.server_is_ready():
                    raise RuntimeError('trajectory action unavailable')
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = NAMES
                point = JointTrajectoryPoint()
                point.positions = target
                point.time_from_start.sec = int(duration)
                point.time_from_start.nanosec = int((duration-int(duration))*1e9)
                goal.trajectory.points = [point]
                self.pending = self.action.send_goal_async(goal)
                self.deadline = time.monotonic() + duration + 8.
            else:
                raise ValueError('unknown command')
        except Exception as exc:
            reply({'ok': False, 'error': str(exc)})

    def tick(self):
        if self.pending and self.pending.done():
            try:
                self.goal_handle = self.pending.result()
                if self.goal_handle is None or not self.goal_handle.accepted:
                    raise RuntimeError('trajectory rejected')
                self.result = self.goal_handle.get_result_async()
            except Exception as exc:
                if not self.closing:
                    reply({'ok': False, 'error': str(exc)})
                self.goal_handle = None
            self.pending = None
        if self.goal_handle and (self.closing or time.monotonic() > self.deadline) and not self.cancel_sent:
            self.goal_handle.cancel_goal_async()
            self.cancel_sent = True
        if self.result and self.result.done():
            try:
                response = self.result.result()
                if self.cancel_sent or response.status != 4 or response.result.error_code != 0:
                    raise RuntimeError(f'trajectory stopped or failed: {response.result.error_string}')
                if time.monotonic() - self.joint_time > 1.0:
                    raise RuntimeError('joint state stale after motion')
                reply({'ok': True, 'positions': self.positions})
            except Exception as exc:
                if not self.closing:
                    reply({'ok': False, 'error': str(exc)})
            self.goal_handle = self.result = None
            self.cancel_sent = False


def main():
    # If our parent disappears on Linux, ask the server to cancel rather than
    # leave a trajectory running just because the client vanished.
    try:
        import ctypes
        libc = ctypes.CDLL(None)
        original_parent = os.getppid()
        libc.prctl(1, signal.SIGTERM)
        if os.getppid() != original_parent:
            return
    except Exception:
        pass
    rclpy.init()
    node = Bridge()
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'closing', True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'closing', True))
    threading.Thread(target=reader, daemon=True).start()
    try:
        close_time = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            while not requests.empty():
                node.command(requests.get_nowait())
            node.tick()
            if node.closing:
                close_time = close_time or time.monotonic()
                if (not node.pending and not node.handle) or time.monotonic() - close_time > 5.:
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
