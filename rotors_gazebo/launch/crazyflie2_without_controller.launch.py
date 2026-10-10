# Crazyflie without any controller: the motors are driven directly through
# /crazyflie2/command/motor_speed (actuator_msgs/Actuators, field velocity [rad/s]).
#
#   ros2 launch rotors_gazebo crazyflie2_without_controller.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('rotors_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='crazyflie2'),
        DeclareLaunchArgument('world', default_value='basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('forster_parameters', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'spawn_crazyflie.launch.py')),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'forster_parameters': LaunchConfiguration('forster_parameters')}.items()),
    ])
