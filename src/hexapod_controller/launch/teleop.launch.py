"""Drive the robot. Run walk.launch.py first. Keyboard in a new terminal window by default; joystick:=true uses a gamepad."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick = LaunchConfiguration('joystick')
    return LaunchDescription([
        DeclareLaunchArgument('joystick', default_value='false'),
        DeclareLaunchArgument('terminal', default_value='x-terminal-emulator -e'),
        Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard', prefix=LaunchConfiguration('terminal'), output='screen',
             parameters=[{'speed': 0.03, 'turn': 0.2}], condition=UnlessCondition(joystick)),
        Node(package='joy', executable='joy_node', condition=IfCondition(joystick)),
        Node(package='teleop_twist_joy', executable='teleop_node', condition=IfCondition(joystick),
             parameters=[{'axis_linear.x': 1, 'axis_linear.y': 0, 'axis_angular.yaw': 3, 'scale_linear.x': 0.06, 'scale_linear.y': 0.06, 'scale_angular.yaw': 0.4,
                          'enable_button': 4}]),
    ])
