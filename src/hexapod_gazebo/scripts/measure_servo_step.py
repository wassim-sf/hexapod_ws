#!/usr/bin/env python3
"""Measure the servo model's 30 degree step response, on a lifted leg of the standing robot.

Run with the simulation up and no controller running. Publishes on /joint_commands, the only command path.
Prints rise, settling (within 5% of the step) and overshoot for each joint, both ways.
"""
import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = [f'leg_{i}_{j}' for i in range(1, 7) for j in ('coxa', 'femur', 'tibia')]


class Step(Node):
    def __init__(self):
        super().__init__('servo_step')
        p = yaml.safe_load(open(get_package_share_directory('hexapod_description') + '/config/robot_parameters.yaml'))
        self.base = [v for _ in range(6) for v in p['poses']['standby']]
        self.pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.joint = {}
        self.samples = []
        self.index = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 100)

    def on_state(self, msg):
        self.joint = dict(zip(msg.name, msg.position))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.index is not None:
            self.samples.append((t, self.joint[JOINTS[self.index]]))

    def send(self, q, seconds):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < seconds:
            self.pub.publish(Float64MultiArray(data=[float(v) for v in q]))
            rclpy.spin_once(self, timeout_sec=0.02)


def analyse(samples, start, step):
    samples = [s for s in samples if s[0] >= samples[0][0]]
    t0, q0 = samples[0]
    final = q0 + step
    band = 0.05 * abs(step)
    settled = t0
    for t, q in samples:
        if abs(q - final) > band:
            settled = t
    rise = next((t - t0 for t, q in samples if abs(q - q0) >= 0.9 * abs(step)), float('nan'))
    overshoot = max(0.0, max((q - final) * (1 if step > 0 else -1) for _, q in samples))
    steady = sum(q for _, q in samples[-20:]) / 20 - final
    return settled - t0, rise, math.degrees(overshoot), math.degrees(steady)


def main():
    rclpy.init()
    node = Step()
    node.send(node.base, 4.0)  # stand from where the sim left it
    step = math.radians(30.0)
    # Leg 2 is lifted well clear, so that the servos see only its own weight, and +-30 deg reaches neither the ground nor a stop.
    lifted = list(node.base)
    lifted[3 + 1] = math.radians(62.0)
    lifted[3 + 2] = math.radians(30.0)
    node.send(lifted, 1.5)
    print(f"{'joint':14s} {'dir':>4s} {'settle s':>9s} {'rise s':>8s} {'overshoot deg':>14s} {'steady error deg':>17s}")
    for index, sign in ((3, 1), (3, -1), (4, -1), (4, 1), (5, 1), (5, -1)):
        q = list(lifted)
        node.send(q, 1.0)
        node.samples = []
        node.index = index
        q[index] += sign * step
        node.send(q, 1.5)
        node.index = None
        settle, rise, over, steady = analyse(node.samples, 0.0, sign * step)
        print(f'{JOINTS[index]:14s} {"+" if sign > 0 else "-":>4s} {settle:9.3f} {rise:8.3f} {over:14.2f} {steady:17.2f}', flush=True)
    # The speed cap: a step far too big for the lag to hide it. The datasheet speed should bound the peak.
    q = list(lifted)
    q[3] = math.radians(-50.0)
    node.send(q, 1.0)
    node.samples = []
    node.index = 3
    q[3] = math.radians(50.0)
    node.send(q, 1.0)
    node.index = None
    speed = max(abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(node.samples, node.samples[1:]) if b[0] > a[0])
    print(f'100 deg coxa step: peak speed {speed:.2f} rad/s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
