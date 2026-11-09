#!/usr/bin/env python3
"""Run headless scenarios and summarise them: the gait tuning sweep now, the benchmark worlds in step 5.

One run:   run_scenario.py --world slope_10 --vx 0.05 --seconds 30 --set step_height=0.03
Sweep:     run_scenario.py --sweep sweep.yaml --jobs 4     (a YAML list of {name, vx, wz, world, gait, set: {...}})

Each run gets its own ROS domain and Gazebo partition, so runs can go in parallel. The ground truth used in the
summary comes from the metrics logger's CSV, and never reaches the controller.
"""
import argparse
import concurrent.futures
import csv
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from std_msgs.msg import String

STALL = 1.0787
JOINTS = [f'leg_{i}_{j}' for i in range(1, 7) for j in ('coxa', 'femur', 'tibia')]
COLUMNS = ['name', 'cmd m/s', 'true m/s', 'err %', 'femur peak %', 'femur p99 %', 'tibia peak %', 'mean W', 'tilt peak deg', 'z mm', 'cmd rad/s', 'fell', 'viol']


def drive(vx, wz, seconds):
    """Wait for the stand state, then publish the command for this many simulated seconds, then stop."""
    rclpy.init()
    node = rclpy.create_node('scenario_driver', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    state = {'value': ''}
    node.create_subscription(String, '/controller_state', lambda m: state.update(value=m.data), 10)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    now = lambda: node.get_clock().now().nanoseconds * 1e-9
    wall = time.monotonic()
    while not state['value'].startswith('stand') or state['value'].startswith('standing'):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() - wall > 120:
            sys.exit('the robot never reached the stand state')
    start = last = now()
    message = Twist()
    message.linear.x, message.angular.z = vx, wz
    while now() - start < seconds + 1.0:
        rclpy.spin_once(node, timeout_sec=0.01)
        if now() - last > 0.05:
            last = now()
            pub.publish(message)
        if time.monotonic() - wall > 40 * seconds + 120:
            sys.exit('the simulation stopped advancing')
    node.destroy_node()
    rclpy.shutdown()


def summarise(path, name=''):
    with open(path, encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    walking = [r for r in rows if r['controller_state'].startswith('walk')]
    if len(walking) < 150:
        return dict(zip(COLUMNS, [name, *['-'] * 10, int(rows[-1]['fallen']) if rows else '-', int(rows[-1]['violations']) if rows else '-']))
    steady = walking[100:]  # past the ramp
    get = lambda key, part=steady: np.array([float(r[key]) for r in part])
    effort = np.abs([get('eff_' + j) for j in JOINTS])
    commands = np.array([get('cmd_' + j) for j in JOINTS])
    t = get('time')
    moved = np.hypot(np.diff(get('x')), np.diff(get('y'))).sum()
    speed = moved / (t[-1] - t[0])
    command = float(np.hypot(get('cmd_vx'), get('cmd_vy')).mean())
    return dict(zip(COLUMNS, [
        name, round(command, 4), round(float(speed), 4), round(100 * abs(speed - command) / command, 1) if command > 1e-6 else '-',
        round(100 * float(effort[1::3].max()) / STALL), round(100 * float(np.percentile(effort[1::3], 99)) / STALL), round(100 * float(effort[2::3].max()) / STALL),
        round(float(get('mech_power').mean()), 2), round(float(np.degrees(np.abs([get('true_roll'), get('true_pitch')]).max())), 2), round(1000 * float(get('z').mean()), 1),
        round(float((np.abs(np.diff(commands, axis=1)) / np.maximum(np.diff(t), 0.02)).max()), 2), int(rows[-1]['fallen']), int(rows[-1]['violations'])]))


def run(spec, index, output_dir, seconds):
    name = spec.get('name', spec.get('world', 'run'))
    env = dict(os.environ, ROS_DOMAIN_ID=str(100 + index), GZ_PARTITION=f'scenario_{os.getpid()}_{index}')
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as handle:
        yaml.safe_dump({'hexapod_controller': {'ros__parameters': spec.get('set', {})}}, handle)
    run_id = time.strftime('%Y%m%d_%H%M%S')
    launch = subprocess.Popen(['ros2', 'launch', 'hexapod_controller', 'walk.launch.py', 'gui:=false', 'record:=true', 'bag:=false', f'world:={spec.get("world", "flat")}',
                               f'gait:={spec.get("gait", "tripod")}', f'overrides:={handle.name}', f'scenario:={name}', f'run_id:={run_id}', f'output_dir:={output_dir}'],
                              env=env, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        subprocess.run([sys.executable, __file__, '--drive', str(spec.get('vx', 0.05)), str(spec.get('wz', 0.0)), str(seconds)], env=env, check=True)
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
    return summarise(pathlib.Path(output_dir) / f'{name}_{run_id}' / 'metrics.csv', name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--drive', nargs=3, type=float, help=argparse.SUPPRESS)
    parser.add_argument('--sweep', help='YAML list of runs')
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--world', default='flat')
    parser.add_argument('--gait', default='tripod')
    parser.add_argument('--vx', type=float, default=0.05)
    parser.add_argument('--wz', type=float, default=0.0)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--set', nargs='*', default=[], metavar='PARAM=VALUE')
    parser.add_argument('--output-dir', default='runs')
    args = parser.parse_args()
    if args.drive:
        drive(*args.drive)
        return
    if args.sweep:
        specs = yaml.safe_load(pathlib.Path(args.sweep).read_text())
    else:
        values = {k: yaml.safe_load(v) for k, v in (item.split('=', 1) for item in args.set)}
        specs = [{'name': args.world, 'world': args.world, 'gait': args.gait, 'vx': args.vx, 'wz': args.wz, 'set': values}]
    output = str(pathlib.Path(args.output_dir).resolve())
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        results = list(pool.map(lambda pair: run(pair[1], pair[0], output, args.seconds), enumerate(specs)))
    print('| ' + ' | '.join(COLUMNS) + ' |')
    print('|' + '---|' * len(COLUMNS))
    for result in results:
        print('| ' + ' | '.join(str(result[c]) for c in COLUMNS) + ' |')


if __name__ == '__main__':
    main()
