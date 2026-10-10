# Crazyflie hovering 1 m above the ground with the firmware position controller.
#
#   ros2 launch rotors_gazebo crazyflie2_hovering_example.launch.py
#   ros2 launch rotors_gazebo crazyflie2_hovering_example.launch.py enable_state_estimator:=true
#
# With enable_state_estimator:=true the attitude comes from the IMU (with the MPU-9250
# noise) through the complementary filter of the firmware, otherwise from the ideal
# odometry.

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
    state_estimator = LaunchConfiguration('enable_state_estimator').perform(context).lower() == 'true'
    gains = 'controller_crazyflie2_with_stateEstimator.yaml' if state_estimator \
        else 'controller_crazyflie2.yaml'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'spawn_crazyflie.launch.py')),
            launch_arguments={'namespace': ns,
                              'noisy_imu': str(state_estimator).lower()}.items()),
        Node(package='rotors_control', executable='position_controller_node', namespace=ns,
             output='screen',
             parameters=[os.path.join(share, 'config', gains),
                         {'enable_state_estimator': state_estimator, 'use_sim_time': True}]),
        Node(package='rotors_gazebo', executable='hovering_example', namespace=ns, output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rotors_gazebo', executable='quaternion_to_rpy', namespace=ns,
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='crazyflie2'),
        DeclareLaunchArgument('world', default_value='basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('enable_state_estimator', default_value='false',
                              description='Use the IMU and the complementary filter'),
        OpaqueFunction(function=nodes),
    ])
