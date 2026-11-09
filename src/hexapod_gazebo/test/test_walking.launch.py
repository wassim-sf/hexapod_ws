"""Stand, walk 1 m, turn 90 degrees, then walk 60 s, all in headless Gazebo with the default launch."""
import csv
import math
import os
import pathlib
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
import rclpy.executors
import rclpy.parameter
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger


RUNS = os.environ.get('HEXAPOD_RUNS', tempfile.mkdtemp(prefix='hexapod_runs_'))
RUN_ID = time.strftime('%Y%m%d_%H%M%S')


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    share = get_package_share_directory('hexapod_controller')
    walk = IncludeLaunchDescription(PythonLaunchDescriptionSource(f'{share}/launch/walk.launch.py'),
                                    launch_arguments={'gui': 'false', 'record': 'true', 'scenario': 'launch_test', 'bag': 'true', 'run_id': RUN_ID, 'output_dir': RUNS}.items())
    return launch.LaunchDescription([walk, launch_testing.actions.ReadyToTest()])


class TestWalking(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('walking_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        cls.exec = rclpy.executors.SingleThreadedExecutor()
        cls.exec.add_node(cls.node)
        cls.odom = None
        cls.state = ''
        cls.ready_at = None
        cls.violations = []
        cls.commands = []
        cls.feet = {i: False for i in range(1, 7)}
        cls.min_feet = 6
        n = cls.node
        n.create_subscription(Odometry, '/ground_truth/odom', cls.on_odom, qos_profile_sensor_data)
        n.create_subscription(String, '/controller_state', lambda m: setattr(cls, 'state', m.data), 10)
        n.create_subscription(Bool, '/servo_ready', cls.on_ready, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        n.create_subscription(String, '/controller_safety', lambda m: cls.violations.append(m.data), 10)
        n.create_subscription(Float64MultiArray, '/joint_commands', lambda m: cls.commands.append((n.get_clock().now().nanoseconds * 1e-9, len(m.data))), 10)
        for i in range(1, 7):
            n.create_subscription(Bool, f'/foot_contact/leg_{i}', lambda m, k=i: cls.feet.__setitem__(k, m.data), 10)
        cls.pub = n.create_publisher(Twist, '/cmd_vel', 10)
        cls.wall_start = None
        cls.sim_start = None
        cls.report = []

    @classmethod
    def tearDownClass(cls):
        print('\n'.join(cls.report), flush=True)
        path = os.environ.get('HEXAPOD_TEST_REPORT')
        if path:
            pathlib.Path(path).write_text('\n'.join(cls.report) + '\n')
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_ready(cls, msg):
        if msg.data and cls.ready_at is None:
            cls.ready_at = cls.now()

    @classmethod
    def on_odom(cls, msg):
        cls.odom = msg

    @classmethod
    def now(cls):
        return cls.node.get_clock().now().nanoseconds * 1e-9

    @classmethod
    def spin(cls, seconds, vx=0.0, wz=0.0):
        """Spin for simulated seconds while publishing the command at 20 Hz."""
        start = cls.now()
        wall = time.monotonic()
        last = 0.0
        while cls.now() - start < seconds:
            cls.exec.spin_once(timeout_sec=0.005)
            if time.monotonic() - wall > 30 * seconds + 60:
                raise TimeoutError('simulation stopped advancing')
            if vx or wz:
                t = cls.now()
                if t - last > 0.05:
                    last = t
                    msg = Twist()
                    msg.linear.x, msg.angular.z = float(vx), float(wz)
                    cls.pub.publish(msg)

    @classmethod
    def position(cls):
        p = cls.odom.pose.pose.position
        return p.x, p.y, p.z

    @classmethod
    def yaw(cls):
        q = cls.odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    @classmethod
    def tilt(cls):
        q = cls.odom.pose.pose.orientation
        r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        p = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        return abs(r), abs(p)

    def stay_upright(self):
        z = self.position()[2]
        self.assertGreater(z, 0.05, 'body fell')
        self.assertLess(max(self.tilt()), math.radians(30), 'body tipped')

    def test_1_stands_within_5_s(self):
        cls = type(self)
        wall = time.monotonic()
        while self.now() == 0.0 or self.odom is None:
            self.exec.spin_once(timeout_sec=0.05)
            self.assertLess(time.monotonic() - wall, 120, 'no clock or ground truth')
        cls.sim_start, cls.wall_start = self.now(), time.monotonic()
        while self.state != 'stand':
            self.exec.spin_once(timeout_sec=0.01)
            self.assertLess(self.now(), 5.0, f'still {self.state!r} at sim t=5 s')
        reached = self.now()
        z_at_stand = self.position()[2]
        self.assertGreater(z_at_stand, 0.07, 'the state says stand, but the body has not risen')
        self.spin(0.5)
        z = self.position()[2]
        cls.report.append(f'stand: reached at sim t={reached:.2f} s (servos ready at t={cls.ready_at:.2f} s), body height {z * 1000:.1f} mm, wall {time.monotonic() - wall:.1f} s from launch')
        self.assertGreater(z, 0.07)
        self.assertTrue(all(self.feet.values()), f'feet {self.feet}')

    def test_2_walks_1_m(self):
        cls = type(self)
        command = 0.05
        cls.spin(1.0, vx=command)  # ramp up
        x0, y0, _ = self.position()
        t0 = self.now()
        while self.position()[0] - x0 < 1.0:
            cls.spin(0.1, vx=command)
            self.stay_upright()
            self.assertLess(self.now() - t0, 40.0, 'too slow')
        speed = (self.position()[0] - x0) / (self.now() - t0)
        lateral = self.position()[1] - y0
        error = abs(speed - command) / command
        cls.report.append(f'walk 1 m at {command} m/s: true speed {speed:.4f} m/s, error {error * 100:.1f}%, lateral drift {lateral * 1000:.1f} mm')
        self.assertLess(error, 0.20)
        cls.spin(3.0)

    def test_3_turns_90(self):
        cls = type(self)
        rate = 0.20
        y0 = self.yaw()
        t0 = self.now()
        cls.spin(math.pi / 2 / rate, wz=rate)  # the ramps up and down cancel, so this commands 90 degrees
        cls.spin(3.0)
        turned = math.degrees((self.yaw() - y0 + math.pi) % (2 * math.pi) - math.pi)
        cls.report.append(f'turn: commanded 90 deg, turned {turned:.1f} deg in {self.now() - t0:.1f} s')
        self.stay_upright()
        self.assertGreater(turned, 75.0)
        self.assertLess(turned, 105.0)

    def test_4_walks_60_s_without_a_fall_or_violation(self):
        cls = type(self)
        before = len(self.violations)
        t0 = self.now()
        wall = time.monotonic()
        min_feet = 6
        while self.now() - t0 < 60.0:
            cls.spin(0.1, vx=0.05)
            self.stay_upright()
            min_feet = min(min_feet, sum(self.feet.values()))
        rtf = (self.now() - t0) / (time.monotonic() - wall)
        cls.report.append(f'60 s walk: no fall, {len(self.violations) - before} violations, fewest switches pressed {min_feet}, real-time factor {rtf:.2f}')
        self.assertEqual(len(self.violations), before, self.violations[before:before + 3])
        self.assertGreaterEqual(rtf, 0.5)

    def test_5_command_path(self):
        cls = type(self)
        times = [t for t, n in self.commands[-1500:]]
        lengths = {n for t, n in self.commands}
        rate = (len(times) - 1) / (times[-1] - times[0])
        cls.report.append(f'/joint_commands: {rate:.1f} Hz, length {lengths}')
        self.assertEqual(lengths, {18})
        self.assertAlmostEqual(rate, 50.0, delta=3.0)

    def test_6_controller_reads_no_ground_truth(self):
        info = self.node.get_publishers_info_by_topic('/joint_commands')
        self.assertEqual([i.node_name for i in info], ['hexapod_controller'])
        subs = [i.node_name for i in self.node.get_subscriptions_info_by_topic('/ground_truth/odom')]
        self.assertNotIn('hexapod_controller', subs)
        self.assertNotIn('servo_model_controller', subs)

    @unittest.skip('not verified yet, see docs/TODO.md')
    def test_7_sits_down(self):
        cls = type(self)
        client = self.node.create_client(Trigger, '/hexapod/sit')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        self.exec.spin_until_future_complete(future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        t0 = self.now()
        while self.state != 'laydown':
            cls.spin(0.1)
            self.assertLess(self.now() - t0, 20.0, f'still {self.state!r}')
        cls.spin(1.0)
        z = self.position()[2]
        cls.report.append(f'sit down: laydown after {self.now() - t0:.1f} s, body height {z * 1000:.1f} mm')
        self.assertLess(z, 0.05)
        self.assertLess(max(self.tilt()), math.radians(5))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_metrics_were_recorded(self):
        folder = pathlib.Path(RUNS) / f'launch_test_{RUN_ID}'
        with open(folder / 'metrics.csv', encoding='utf-8') as handle:
            rows = list(csv.DictReader(handle))
        self.assertGreater(len(rows), 1000)
        self.assertEqual(rows[-1]['fallen'], '0')
        self.assertEqual(rows[-1]['violations'], '0')
        self.assertTrue((folder / 'bag' / 'metadata.yaml').exists())
