#!/usr/bin/env python3
"""Save a GIF from the camera of `walk.launch.py camera:=true`, walking through the /cmd_vel path.

Usage: record_gif.py out.gif [--vx 0.05] [--seconds 11] [--fps 10] [--speedup 1.5] [--width 480]
Needs python3-pil. Records from the first sim frame, so the stand-up is included.
"""
import argparse

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from PIL import Image as Pil
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output')
    parser.add_argument('--vx', type=float, default=0.05)
    parser.add_argument('--seconds', type=float, default=11.0, help='of simulated walking, after the stand-up')
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--speedup', type=float, default=1.5)
    parser.add_argument('--width', type=int, default=480)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('gif_recorder', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    frames = []
    period = args.speedup / args.fps
    state = {'value': '', 'next': 0.0, 'walk_at': None}

    def on_image(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t + 1e-6 < state['next'] or msg.encoding != 'rgb8':
            return
        state['next'] = t + period
        image = Pil.fromarray(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3))
        frames.append(image.resize((args.width, args.width * msg.height // msg.width), Pil.LANCZOS))

    node.create_subscription(Image, '/gif_camera', on_image, qos_profile_sensor_data)
    node.create_subscription(String, '/controller_state', lambda m: state.update(value=m.data), 10)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    now = lambda: node.get_clock().now().nanoseconds * 1e-9
    message = Twist()
    message.linear.x = args.vx
    last = 0.0
    while True:
        rclpy.spin_once(node, timeout_sec=0.01)
        if state['walk_at'] is None and state['value'] == 'stand':
            state['walk_at'] = now()
        if state['walk_at'] is not None:
            if now() - state['walk_at'] > args.seconds:
                break
            if now() - last > 0.05:
                last = now()
                pub.publish(message)
    palette = frames[len(frames) // 2].convert('P', palette=Pil.ADAPTIVE, colors=64)
    quantised = [f.quantize(palette=palette, dither=Pil.NONE) for f in frames]
    quantised[0].save(args.output, save_all=True, append_images=quantised[1:], duration=int(1000 / args.fps), loop=0, optimize=True)
    print(f'{args.output}: {len(frames)} frames')


if __name__ == '__main__':
    main()
