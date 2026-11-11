#!/usr/bin/env python3
"""Read-only live joint-angle display for HOME teaching."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

class Monitor(Node):
    def __init__(self):
        super().__init__('home_joint_monitor')
        self.create_subscription(JointState, '/rebot_arm_rs/joint_states', self.show, qos_profile_sensor_data)
        self.last = 0.

    def show(self, msg):
        if time.monotonic() - self.last < .3:
            return
        self.last = time.monotonic()
        try:
            q = [math.degrees(msg.position[list(msg.name).index(f'joint{i}')]) for i in range(1, 7)]
        except (ValueError, IndexError):
            return
        status = 'OK (J2/J3 >= 12 deg)' if q[1] >= 12 and q[2] >= 12 else 'ADJUST J2 AND J3 TO >= 12 deg'
        print('\r' + '  '.join(f'J{i+1}={v:+6.1f}' for i,v in enumerate(q)) + '  ' + status + ' '*8, end='', flush=True)

def main():
    rclpy.init()
    node=Monitor()
    print('只读 HOME 角度预览；Ctrl+C 只关闭本窗口，不控制机械臂。', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()

if __name__=='__main__':
    main()
