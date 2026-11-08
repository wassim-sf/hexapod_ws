#!/usr/bin/env python3
"""Check all 18 joints live: publish /joint_states and read foot positions from TF.

Run with the ROS environment sourced, while display.launch.py is up with gui:=false.
Expected motion comes from the kinematic definition, and exact positions from an independent
forward-kinematics calculation here that uses only the YAML geometry. Exits non-zero on failure.
"""
import math
import random
import sys
import time

import numpy as np
import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

P = yaml.safe_load(open(get_package_share_directory('hexapod_description') + '/config/robot_parameters.yaml'))
NAMES = [f'leg_{i}_{j}' for i in range(1, 7) for j in ('coxa', 'femur', 'tibia')]


def foot_fk(leg, q):
    """Foot position in base_link, written from the convention rather than the URDF."""
    g = P['leg_geometry']
    c, f, t = g['coxa_length'], g['femur_length'], g['tibia_length']
    radial = c + f * math.cos(q[1]) + t * math.sin(q[1] + q[2])
    up = f * math.sin(q[1]) - t * math.cos(q[1] + q[2])
    a = leg['yaw'] + q[0]
    return np.array([leg['x'] + radial * math.cos(a), leg['y'] + radial * math.sin(a), P['body']['hip_z'] + up])


class Checker(Node):
    def __init__(self):
        super().__init__('hexapod_joint_checker')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def settle(self, q):
        """Publish q until TF reflects it, and return foot positions by leg id."""
        stamp_start = self.get_clock().now()
        deadline = time.time() + 3.0
        msg = JointState(name=NAMES, position=[float(v) for v in q])
        pos = {}
        while time.time() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            if (self.get_clock().now() - stamp_start).nanoseconds > 3e8:
                try:
                    for leg in P['legs']:
                        t = self.buf.lookup_transform('base_link', f"leg_{leg['id']}_foot_link", rclpy.time.Time()).transform.translation
                        pos[leg['id']] = np.array([t.x, t.y, t.z])
                    return pos
                except tf2_ros.TransformException:
                    pass
        raise RuntimeError('no TF for the feet')


def main():
    rclpy.init()
    node = Checker()
    fails = 0
    zero = np.zeros(18)
    base = node.settle(zero)
    print('Neutral foot positions (mm):')
    for leg in P['legs']:
        expected = foot_fk(leg, (0, 0, 0))
        err = np.linalg.norm(base[leg['id']] - expected)
        print(f"  leg_{leg['id']} {leg['name']:13s} {np.round(base[leg['id']] * 1000, 1)}  err {err * 1000:.3f}")
        fails += err > 1e-4

    print('\nEach joint at +0.4 rad, others at zero. Foot displacement in the leg frame (mm):')
    print(f"{'joint':14s} {'radial':>8s} {'tangent':>8s} {'up':>8s}  {'expected':28s} result")
    for i, name in enumerate(NAMES):
        leg = P['legs'][i // 3]
        kind = name.rsplit('_', 1)[1]
        q = zero.copy()
        q[i] = 0.4
        d = node.settle(q)[leg['id']] - base[leg['id']]
        y = leg['yaw']
        radial = d[0] * math.cos(y) + d[1] * math.sin(y)
        tangent = -d[0] * math.sin(y) + d[1] * math.cos(y)
        up = d[2]
        if kind == 'coxa':
            ok, text = tangent > 0.01 and abs(up) < 1e-6, 'foot swings CCW seen from above'
        elif kind == 'femur':
            ok, text = up > 0.01 and abs(tangent) < 1e-6, 'foot rises'
        else:
            ok, text = radial > 0.01 and abs(tangent) < 1e-6, 'foot swings outward'
        print(f'{name:14s} {radial * 1000:8.1f} {tangent * 1000:8.1f} {up * 1000:8.1f}  {text:28s} {"PASS" if ok else "FAIL"}')
        fails += not ok

    print('\nRandom poses within limits, TF vs independent FK:')
    random.seed(7)
    worst = 0.0
    for _ in range(10):
        q = []
        for _leg in range(6):
            q += [random.uniform(P['joints'][j]['lower'], P['joints'][j]['upper']) for j in ('coxa', 'femur', 'tibia')]
        pos = node.settle(q)
        for k, leg in enumerate(P['legs']):
            worst = max(worst, np.linalg.norm(pos[leg['id']] - foot_fk(leg, q[3 * k:3 * k + 3])))
    print(f'  worst foot error over 60 samples: {worst * 1000:.4f} mm')
    fails += worst > 1e-4

    for pose in ('standby', 'laydown'):
        q = P['poses'][pose]
        for j, v in zip(('coxa', 'femur', 'tibia'), q):
            assert P['joints'][j]['lower'] <= v <= P['joints'][j]['upper'], (pose, j)
    print('  standby and laydown are within limits')

    print(f"\n{'ALL PASS' if not fails else str(fails) + ' FAILURES'}")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
