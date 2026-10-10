# Swarm of five Crazyflie, each one flown by its own Internal Model Controller along a
# quintic spline that takes it 1 m above its spawn point.
#
#   ros2 launch rotors_gazebo crazyflie2_swarm_hovering_example.launch.py
#
# Namespaces: crazyflie2_1 ... crazyflie2_5.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWN_POSITIONS = [(-0.5, -0.5), (-1.0, -0.5), (-0.5, -1.0), (-1.5, -1.0), (-0.5, -1.5)]


def generate_launch_description():
    share = get_package_share_directory('rotors_gazebo')
    actions = [
        DeclareLaunchArgument('world', default_value='basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui')}.items()),
    ]
    for i, (x, y) in enumerate(SPAWN_POSITIONS, start=1):
        ns = f'crazyflie2_{i}'
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, 'launch', 'spawn_crazyflie.launch.py')),
                launch_arguments={'namespace': ns, 'x': str(x), 'y': str(y)}.items()),
            Node(package='rotors_control', executable='position_controller_node', namespace=ns,
                 output='screen',
                 parameters=[os.path.join(share, 'config',
                                          'crazyflie_internal_model_controller.yaml'),
                             {'enable_internal_model_controller': True, 'use_sim_time': True}]),
            Node(package='rotors_gazebo', executable='hovering_example_spline', namespace=ns,
                 output='screen',
                 parameters=[os.path.join(share, 'config', 'spline_trajectory.yaml'),
                             {'position_initial.x': x, 'position_initial.y': y,
                              'position_final.x': x, 'position_final.y': y,
                              'use_sim_time': True}]),
        ]
    return LaunchDescription(actions)
