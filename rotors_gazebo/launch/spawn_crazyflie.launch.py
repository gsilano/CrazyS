# Spawns one Crazyflie in a running Gazebo Sim world and bridges its topics to ROS 2:
#   /<namespace>/odometry             nav_msgs/Odometry       (Gazebo -> ROS)
#   /<namespace>/imu                  sensor_msgs/Imu         (Gazebo -> ROS)
#   /<namespace>/command/motor_speed  actuator_msgs/Actuators (ROS -> Gazebo)
#   /<namespace>/joint_states         sensor_msgs/JointState  (Gazebo -> ROS)
#
#   ros2 launch rotors_gazebo spawn_crazyflie.launch.py namespace:=crazyflie2 x:=0 y:=0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def spawn(context):
    ns = LaunchConfiguration('namespace').perform(context)
    xacro_file = os.path.join(get_package_share_directory('rotors_description'),
                              'urdf', 'crazyflie2.urdf.xacro')
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' namespace:=', ns,
        ' forster_parameters:=', LaunchConfiguration('forster_parameters'),
        ' noisy_imu:=', LaunchConfiguration('noisy_imu'),
        ' odometry_rate:=', LaunchConfiguration('odometry_rate'),
    ]), value_type=str)

    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, output='screen',
            parameters=[{'robot_description': robot_description,
                         'frame_prefix': ns + '/',
                         'use_sim_time': True}]),
        Node(
            package='ros_gz_sim', executable='create', namespace=ns, output='screen',
            arguments=['-name', ns, '-topic', 'robot_description',
                       '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                       '-z', LaunchConfiguration('z')]),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', namespace=ns,
            name='bridge', output='screen',
            arguments=[
                f'/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{ns}/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
                f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='crazyflie2',
                              description='Model name, also used as topic namespace'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('forster_parameters', default_value='false',
                              description="Use Forster's propeller coefficients"),
        DeclareLaunchArgument('noisy_imu', default_value='false',
                              description='Add the MPU-9250 noise model to the IMU'),
        DeclareLaunchArgument('odometry_rate', default_value='1000',
                              description='Odometry publishing rate [Hz]'),
        OpaqueFunction(function=spawn),
    ])
