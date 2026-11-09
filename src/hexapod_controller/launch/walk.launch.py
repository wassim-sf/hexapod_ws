"""The simulation plus the classical controller.

Arguments: world, gui, gait, camera, command_delay, deadband; overrides (a parameter file on top of config/controller.yaml);
record:=true with scenario, run_id, bag and output_dir to run the metrics logger.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def controller_node(context):
    share = get_package_share_directory('hexapod_controller')
    files = [f'{share}/config/controller.yaml']
    overrides = LaunchConfiguration('overrides').perform(context)
    if overrides:
        files.append(overrides)
    return [Node(package='hexapod_controller', executable='controller_node', name='hexapod_controller', output='screen',
                 parameters=[*files, {'gait': LaunchConfiguration('gait')}])]


def generate_launch_description():
    gazebo = get_package_share_directory('hexapod_gazebo')
    args = {name: default for name, default in (('world', 'flat'), ('gui', 'true'), ('gait', 'tripod'), ('overrides', ''), ('record', 'false'), ('scenario', 'flat'),
                                                ('run_id', ''), ('bag', 'false'), ('output_dir', 'runs'), ('camera', 'false'), ('command_delay', ''), ('deadband', ''))}
    logger_params = {'use_sim_time': True, 'scenario': ParameterValue(LaunchConfiguration('scenario'), value_type=str), 'run_id': ParameterValue(LaunchConfiguration('run_id'), value_type=str),
                     'output_dir': ParameterValue(LaunchConfiguration('output_dir'), value_type=str), 'bag': ParameterValue(LaunchConfiguration('bag'), value_type=bool)}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in args.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(f'{gazebo}/launch/sim.launch.py'),
                                 launch_arguments={name: LaunchConfiguration(name) for name in ('world', 'gui', 'camera', 'command_delay', 'deadband')}.items()),
        OpaqueFunction(function=controller_node),
        Node(package='hexapod_gazebo', executable='metrics_logger.py', output='log', parameters=[logger_params],
             condition=IfCondition(LaunchConfiguration('record'))),
    ])
