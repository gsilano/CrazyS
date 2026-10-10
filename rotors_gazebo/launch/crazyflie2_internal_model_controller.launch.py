# Crazyflie flown by the Internal Model Controller along a quintic spline from the
# ground to 1 m of altitude (config/spline_trajectory.yaml).
#
#   ros2 launch rotors_gazebo crazyflie2_internal_model_controller.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('rotors_gazebo')
    ns = LaunchConfiguration('namespace').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'spawn_crazyflie.launch.py')),
            launch_arguments={'namespace': ns}.items()),
        Node(package='rotors_control', executable='position_controller_node', namespace=ns,
             output='screen',
             parameters=[os.path.join(share, 'config', 'crazyflie_internal_model_controller.yaml'),
                         {'enable_internal_model_controller': True, 'use_sim_time': True}]),
        Node(package='rotors_gazebo', executable='hovering_example_spline', namespace=ns,
             output='screen',
             parameters=[os.path.join(share, 'config', 'spline_trajectory.yaml'),
                         {'use_sim_time': True}]),
        Node(package='rotors_gazebo', executable='quaternion_to_rpy', namespace=ns,
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='crazyflie2'),
        DeclareLaunchArgument('world', default_value='basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
