#!/usr/bin/env python3
"""Record a run for the benchmark: one CSV row per 50 Hz tick, plus an optional rosbag. Ground truth is only used here.

Parameters: scenario, run_id (default: time), output_dir, bag, cmd_timeout, fall_height, fall_tilt_deg.
CSV columns are in FIELDS plus one per joint for command, position, velocity and effort.
"""
import csv
import datetime
import math
import pathlib
import signal
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, String

JOINTS = [f'leg_{i}_{j}' for i in range(1, 7) for j in ('coxa', 'femur', 'tibia')]
FIELDS = ['time', 'scenario', 'run_id', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'true_vx', 'true_vy', 'true_wz', 'x', 'y', 'z', 'yaw', 'imu_roll', 'imu_pitch',
          'true_roll', 'true_pitch', 'mech_power', 'fallen', 'controller_state', 'violations']
BAG_TOPICS = ['/joint_commands', '/joint_states', '/servo_model/efforts', '/cmd_vel', '/ground_truth/odom', '/imu/data', '/controller_state', '/controller_safety',
              *[f'/foot_contact/leg_{i}' for i in range(1, 7)]]


def roll_pitch_yaw(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        self.scenario = self.declare_parameter('scenario', 'flat').value
        self.run_id = self.declare_parameter('run_id', '').value or datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        folder = pathlib.Path(self.declare_parameter('output_dir', 'runs').value) / f'{self.scenario}_{self.run_id}'
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.fall_height = self.declare_parameter('fall_height', 0.05).value
        self.fall_tilt = math.radians(self.declare_parameter('fall_tilt_deg', 30.0).value)
        folder.mkdir(parents=True, exist_ok=True)
        self.file = open(folder / 'metrics.csv', 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(FIELDS + [f'foot_{i}' for i in range(1, 7)] + [f'cmd_{j}' for j in JOINTS] + [f'pos_{j}' for j in JOINTS] + [f'vel_{j}' for j in JOINTS] + [f'eff_{j}' for j in JOINTS])
        self.bag = None
        if self.declare_parameter('bag', True).value:
            self.bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(folder / 'bag'), '--use-sim-time', *BAG_TOPICS], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.cmd_vel = Twist()
        self.odom = None
        self.imu = None
        self.joints = {}
        self.command = [math.nan] * 18
        self.effort = [0.0] * 18
        self.feet = [False] * 6
        self.state = ''
        self.violations = 0
        self.fallen = False
        self.cmd_vel_at = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', lambda m: setattr(self, 'imu', m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Float64MultiArray, '/joint_commands', lambda m: setattr(self, 'command', list(m.data)), 10)
        self.create_subscription(Float64MultiArray, '/servo_model/efforts', lambda m: setattr(self, 'effort', list(m.data)), 10)
        self.create_subscription(String, '/controller_state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(String, '/controller_safety', self.on_violation, 10)
        for i in range(6):
            self.create_subscription(Bool, f'/foot_contact/leg_{i + 1}', lambda m, k=i: self.feet.__setitem__(k, m.data), 10)
        self.create_timer(0.02, self.tick)
        self.create_timer(1.0, self.file.flush)  # a run that is killed keeps nearly all of its data
        self.get_logger().info(f'Writing {folder}')

    def on_cmd_vel(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_at = self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg):
        self.joints = {n: (p, v) for n, p, v in zip(msg.name, msg.position, msg.velocity)}

    def on_violation(self, _msg):
        self.violations += 1

    def tick(self):
        if self.odom is None or len(self.joints) < 18:
            return
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        roll, pitch, yaw = roll_pitch_yaw(pose.orientation)
        imu_roll, imu_pitch = (roll_pitch_yaw(self.imu.orientation)[:2] if self.imu else (math.nan, math.nan))
        positions = [self.joints[j][0] for j in JOINTS]
        velocities = [self.joints[j][1] for j in JOINTS]
        # Mechanical power: the positive work the motors do. Braking is not recovered by hobby servos.
        power = sum(max(e * v, 0.0) for e, v in zip(self.effort, velocities))
        if not self.fallen and self.state in ('stand', 'walk', 'stopping') and (pose.position.z < self.fall_height or abs(roll) > self.fall_tilt or abs(pitch) > self.fall_tilt):
            self.fallen = True
            self.get_logger().error(f'FALL at z={pose.position.z:.3f} roll={math.degrees(roll):.1f} pitch={math.degrees(pitch):.1f}')
        t = self.get_clock().now().nanoseconds * 1e-9
        command = self.cmd_vel if t - self.cmd_vel_at <= self.cmd_timeout else Twist()  # the same timeout the controller applies
        self.writer.writerow([f'{t:.3f}', self.scenario, self.run_id, command.linear.x, command.linear.y, command.angular.z, twist.linear.x, twist.linear.y, twist.angular.z,
                              pose.position.x, pose.position.y, pose.position.z, yaw, imu_roll, imu_pitch, roll, pitch, power, int(self.fallen), self.state, self.violations,
                              *[int(v) for v in self.feet], *self.command, *positions, *velocities, *self.effort])

    def close(self):
        self.file.close()
        if self.bag:
            self.bag.send_signal(signal.SIGINT)
            try:
                self.bag.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.bag.kill()


def main():
    rclpy.init()
    signal.signal(signal.SIGTERM, signal.getsignal(signal.SIGINT))
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()


if __name__ == '__main__':
    main()
