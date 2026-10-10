# Three Crazyflie, each with its own Lee controller and trajectory (S. Grossi's thesis,
# Section 6.7). The trajectories (config/trajectories/swarm_crazyflie2_<i>.yaml) start at
# the same simulation time, so the drones move together.
#
#   ros2 launch rotors_gazebo crazyflie2_swarm_lee_geometric_controller.launch.py
#
# Namespaces: crazyflie2_1, crazyflie2_2, crazyflie2_3.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWN_POSITIONS = [(0.0, 0.0), (1.0, 0.0), (1.0, -1.0)]


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
                launch_arguments={'namespace': ns, 'x': str(x), 'y': str(y),
                                  'forster_parameters': 'true'}.items()),
            Node(package='rotors_control', executable='lee_geometric_controller_node',
                 namespace=ns, output='screen',
                 parameters=[
                     os.path.join(share, 'config', 'crazyflie2_lee_geometric_controller.yaml'),
                     os.path.join(share, 'config', 'crazyflie2_forster_motor_parameters.yaml'),
                     {'use_sim_time': True}]),
            Node(package='rotors_gazebo', executable='piecewise_trajectory_generator',
                 namespace=ns, output='screen',
                 parameters=[{'trajectory_file': os.path.join(
                                  share, 'config', 'trajectories', f'swarm_crazyflie2_{i}.yaml'),
                              'use_sim_time': True}]),
        ]
    return LaunchDescription(actions)
