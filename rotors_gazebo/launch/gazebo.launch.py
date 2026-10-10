# Starts Gazebo Sim with one of the worlds in rotors_gazebo/worlds and bridges the
# simulation clock to ROS 2.
#
#   ros2 launch rotors_gazebo gazebo.launch.py world:=basic gui:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_gazebo(context):
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context)

    world_file = os.path.join(get_package_share_directory('rotors_gazebo'), 'worlds', world + '.sdf')
    gz_args = f'{world_file} -v {verbose}'
    if not paused:
        gz_args += ' -r'
    if not gui:
        gz_args += ' -s'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='basic',
                              description='Name of a world in rotors_gazebo/worlds (without .sdf)'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('paused', default_value='false',
                              description='Start with the simulation paused'),
        DeclareLaunchArgument('verbose', default_value='1', description='Gazebo verbosity (0-4)'),
        OpaqueFunction(function=launch_gazebo),
    ])
