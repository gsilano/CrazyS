# Crazyflie flown by the Lee geometric controller.
#
#   ros2 launch rotors_gazebo crazyflie2_lee_geometric_controller.launch.py trajectory:=semicircle
#
# trajectory: one of the files in config/trajectories (hovering, linear,
# linear_polynomial, semicircle, semicircle_polynomial). Default: hovering.
# forster_parameters: Forster's propeller coefficients, the ones used to tune the
# controller. With false the default CrazyS model is used and tracking gets worse.

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
    trajectory = LaunchConfiguration('trajectory').perform(context)
    forster = LaunchConfiguration('forster_parameters').perform(context).lower() == 'true'
    motors = 'crazyflie2_forster_motor_parameters.yaml' if forster \
        else 'crazyflie2_motor_parameters.yaml'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'spawn_crazyflie.launch.py')),
            launch_arguments={'namespace': ns,
                              'forster_parameters': str(forster).lower()}.items()),
        Node(package='rotors_control', executable='lee_geometric_controller_node', namespace=ns,
             output='screen',
             parameters=[os.path.join(share, 'config', 'crazyflie2_lee_geometric_controller.yaml'),
                         os.path.join(share, 'config', motors),
                         {'use_sim_time': True}]),
        Node(package='rotors_gazebo', executable='piecewise_trajectory_generator', namespace=ns,
             output='screen',
             parameters=[{'trajectory_file': os.path.join(share, 'config', 'trajectories',
                                                          trajectory + '.yaml'),
                          'use_sim_time': True}]),
        Node(package='rotors_gazebo', executable='quaternion_to_rpy', namespace=ns,
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='crazyflie2'),
        DeclareLaunchArgument('world', default_value='basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('trajectory', default_value='hovering',
                              description='File in config/trajectories, without .yaml'),
        DeclareLaunchArgument('forster_parameters', default_value='true',
                              description="Use Forster's propeller coefficients"),
        OpaqueFunction(function=nodes),
    ])
