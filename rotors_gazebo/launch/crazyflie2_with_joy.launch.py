# Crazyflie piloted with a joystick: joy_node -> rotors_joy_interface ->
# roll_pitch_yawrate_thrust_crazyflie_node -> motors.
#
#   ros2 launch rotors_gazebo crazyflie2_with_joy.launch.py

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
        Node(package='joy', executable='joy_node', namespace=ns,
             parameters=[{'use_sim_time': True}]),
        Node(package='rotors_joy_interface', executable='rotors_joy_interface_crazyflie',
             namespace=ns, output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='rotors_control', executable='roll_pitch_yawrate_thrust_crazyflie_node',
             namespace=ns, output='screen',
             parameters=[os.path.join(share, 'config',
                                      'roll_pitch_yawrate_thrust_controller_crazyflie2.yaml'),
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
