"""Start the hexapod in Gazebo with its one command path: /joint_commands -> servo model -> joint efforts.

Arguments: world:=flat|slope_10|rough|step_2cm|low_friction, gui:=true|false,
command_delay:=<s> and deadband:=<rad> override servo_model.yaml, camera:=true adds a fixed camera on /gif_camera,
debug_trajectory:=true loads joint_trajectory_controller inactive (debugging only; see README).

The robot is written into a temporary copy of the world rather than spawned at runtime, because Gazebo Harmonic's
contact system only registers the contact sensors that exist when the world loads.
"""
import math
import os
import pathlib
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

JOINTS = [f'leg_{i}_{j}' for i in range(1, 7) for j in ('coxa', 'femur', 'tibia')]

CAMERA = '''<model name="gif_camera"><static>true</static><pose>0.15 -0.95 0.42 0 0.36 1.57</pose><link name="link">
  <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>15</update_rate><topic>gif_camera</topic>
    <camera><horizontal_fov>0.9</horizontal_fov><image><width>640</width><height>360</height></image><clip><near>0.05</near><far>20</far></clip></camera>
  </sensor></link></model>
'''


def controller_params(desc, overrides):
    """Resolve the servo model's numbers from the two YAML files, into a parameter file for the spawner."""
    robot = yaml.safe_load(open(f'{desc}/config/robot_parameters.yaml'))
    servo = yaml.safe_load(open(f'{desc}/config/servo_model.yaml'))['servo_model']
    servo.update(overrides)
    params = {
        'joints': JOINTS,
        'kp': float(servo['kp']), 'kd': float(servo['kd']),
        'stall_torque': float(robot['servo']['stall_torque_6v']) * float(servo['stall_torque_scale']),
        'no_load_speed': float(robot['servo']['speed_6v']) * float(servo['speed_scale']),
        'deadband': float(servo['deadband']), 'command_delay': float(servo['command_delay']), 'response_tau': float(servo['response_tau']),
        'use_sim_time': True,
    }
    handle = tempfile.NamedTemporaryFile('w', suffix='_servo_model.yaml', delete=False)
    yaml.safe_dump({'servo_model_controller': {'ros__parameters': params}}, handle)
    handle.close()
    return handle.name, robot


def world_with_robot(world_file, urdf, z, camera):
    """A temporary copy of the world that includes the robot, converted from its URDF."""
    folder = tempfile.mkdtemp(prefix='hexapod_world_')
    urdf_path = pathlib.Path(folder) / 'robot.urdf'
    urdf_path.write_text(urdf)
    sdf_path = pathlib.Path(folder) / 'robot.sdf'
    sdf_path.write_text(subprocess.run(['gz', 'sdf', '-p', str(urdf_path)], check=True, capture_output=True, text=True).stdout)
    include = f'<include><uri>file://{sdf_path}</uri><name>hexapod</name><pose>0 0 {z} 0 0 0</pose></include>\n{CAMERA if camera else ""}</world>'
    world_path = pathlib.Path(folder) / 'world.sdf'
    world_path.write_text(pathlib.Path(world_file).read_text().replace('</world>', include))
    return str(world_path)


def launch_setup(context):
    desc = get_package_share_directory('hexapod_description')
    share = get_package_share_directory('hexapod_gazebo')
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    camera = LaunchConfiguration('camera').perform(context) == 'true'
    overrides = {name: float(value) for name in ('command_delay', 'deadband') if (value := LaunchConfiguration(name).perform(context))}
    params_file, robot = controller_params(desc, overrides)
    controllers = f'{share}/config/controllers.yaml'
    urdf = subprocess.run(['xacro', f'{desc}/urdf/hexapod.urdf.xacro', 'sim:=true', f'controllers_file:={controllers}'], check=True, capture_output=True, text=True).stdout
    # Start lying on the battery bars, where the laydown pose puts the feet on the ground.
    spawn_z = -float(robot['body']['body_bottom_z']) + 0.001

    world_file = world_with_robot(f'{share}/worlds/{world}.sdf', urdf, spawn_z, camera)
    gz_args = f'-r {world_file}' + ('' if gui else ' -s') + (' --headless-rendering' if camera and not gui else '')
    gazebo = ExecuteProcess(cmd=['gz', 'sim', *gz_args.split()], output='screen', shell=False)
    robot_state = Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
                       parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='log',
                  parameters=[{'config_file': f'{share}/config/bridge.yaml', 'use_sim_time': True}])
    broadcaster = Node(package='controller_manager', executable='spawner', output='log',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'], parameters=[{'use_sim_time': True}])
    servo = Node(package='controller_manager', executable='spawner', output='log',
                 arguments=['servo_model_controller', '--param-file', params_file, '--controller-manager-timeout', '60'], parameters=[{'use_sim_time': True}])
    switches = Node(package='hexapod_gazebo', executable='foot_switch_sim.py', output='log', parameters=[{'use_sim_time': True}])
    after_broadcaster = [servo]
    if LaunchConfiguration('debug_trajectory').perform(context) == 'true':
        after_broadcaster.append(Node(package='controller_manager', executable='spawner', output='log', parameters=[{'use_sim_time': True}],
                                      arguments=['debug_trajectory_controller', '--inactive', '--param-file', f'{share}/config/debug_trajectory.yaml', '--controller-manager-timeout', '60']))
    extras = [Node(package='ros_gz_image', executable='image_bridge', arguments=['gif_camera'], output='log', parameters=[{'use_sim_time': True}])] if camera else []
    return [gazebo, robot_state, bridge, switches, broadcaster, *extras,
            RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=after_broadcaster))]


def generate_launch_description():
    desc = get_package_share_directory('hexapod_description')
    prefix = pathlib.Path(desc).parent
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('debug_trajectory', default_value='false'),
        DeclareLaunchArgument('command_delay', default_value=''),
        DeclareLaunchArgument('deadband', default_value=''),
        DeclareLaunchArgument('camera', default_value='false'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', f'{prefix}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'),
        OpaqueFunction(function=launch_setup),
    ])
