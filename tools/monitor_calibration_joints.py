#!/usr/bin/env python3
"""Read-only joint feedback while positioning the arm for calibration."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class Monitor(Node):
    def __init__(self):
        super().__init__('calibration_joint_monitor')
        self.last = 0.0
        self.create_subscription(JointState, '/rebot_arm_rs/joint_states', self.show,
                                 qos_profile_sensor_data)

    def show(self, msg):
        if time.monotonic() - self.last < 0.3:
            return
        self.last = time.monotonic()
        try:
            q = [math.degrees(msg.position[list(msg.name).index(f'joint{i}')])
                 for i in range(1, 7)]
        except (ValueError, IndexError):
            return
        line = '  '.join(f'J{i+1}={v:+6.1f}' for i, v in enumerate(q))
        print('\r' + line + '    J4 target: greater than -75 deg     ', end='', flush=True)


def main():
    rclpy.init()
    node = Monitor()
    print('Read-only joint feedback. Ctrl+C closes this window; no robot commands.', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()


if __name__ == '__main__':
    main()
