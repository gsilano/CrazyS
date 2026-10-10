[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/gsilano/CrazyS/actions/workflows/build.yml/badge.svg)](https://github.com/gsilano/CrazyS/actions/workflows/build.yml)

CrazyS for ROS 2
================

This branch (`dev/ros2-jazzy`) is the port of [CrazyS](https://github.com/gsilano/CrazyS) to **ROS 2 Jazzy** and **Gazebo Sim (Harmonic)** on Ubuntu 24.04. CrazyS simulates the [Crazyflie 2.x](https://www.bitcraze.io/) nano-quadcopter together with its flight controllers. The ROS 1 versions are on the `master`, `dev/gazebo9`, `dev/ros-melodic` and `dev/ros-noetic` branches.

The ROS 1 version is built on the RotorS Gazebo classic plugins. Gazebo Sim already ships a port of the RotorS motor model (`MulticopterMotorModel`), so the ROS 2 version uses the systems of Gazebo Sim and [ros_gz](https://github.com/gazebosim/ros_gz) instead of custom plugins. The dynamics are the same: with the same motor commands the drone moves as in the ROS 1 simulator.

If you use CrazyS in your research, please cite the papers listed in the [Publications page](https://github.com/gsilano/CrazyS/wiki/Publications).

Installation
------------

1. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (`ros-jazzy-desktop`) and the Gazebo Harmonic integration:

```console
$ sudo apt install ros-jazzy-ros-gz ros-jazzy-actuator-msgs ros-jazzy-xacro ros-jazzy-joy \
    python3-colcon-common-extensions python3-rosdep
```

2. Clone and build:

```console
$ mkdir -p ~/crazys_ws/src && cd ~/crazys_ws/src
$ git clone -b dev/ros2-jazzy https://github.com/gsilano/CrazyS.git
$ cd ~/crazys_ws
$ rosdep install --from-paths src --ignore-src -y
$ colcon build
$ source install/setup.bash
```

The `mav_comm` repository is no longer needed: the two messages CrazyS used from it (`DroneState` and `RollPitchYawrateThrustCrazyflie`) are now in `rotors_comm`.

Examples
--------

Every launch file accepts `gui:=false` to run Gazebo without the graphical interface.

| Command | What it does |
|---|---|
| `ros2 launch rotors_gazebo crazyflie2_hovering_example.launch.py` | Hovering 1 m above the ground with the controller of the Crazyflie firmware |
| `ros2 launch rotors_gazebo crazyflie2_hovering_example.launch.py enable_state_estimator:=true` | Same, with the attitude estimated from the IMU by the complementary filter of the firmware |
| `ros2 launch rotors_gazebo crazyflie2_internal_model_controller.launch.py` | Internal Model Controller following a quintic spline |
| `ros2 launch rotors_gazebo crazyflie2_lee_geometric_controller.launch.py trajectory:=semicircle` | Lee geometric controller; trajectories: `hovering`, `linear`, `linear_polynomial`, `semicircle`, `semicircle_polynomial` |
| `ros2 launch rotors_gazebo crazyflie2_swarm_hovering_example.launch.py` | Five drones with the Internal Model Controller |
| `ros2 launch rotors_gazebo crazyflie2_swarm_lee_geometric_controller.launch.py` | Three drones with the Lee controller |
| `ros2 launch rotors_gazebo crazyflie2_with_joy.launch.py` | Piloting with a joystick |
| `ros2 launch rotors_gazebo crazyflie2_without_controller.launch.py` | Only the drone: send the motor speeds yourself |

A waypoint can be sent to the firmware position controller with

```console
$ ros2 run rotors_gazebo waypoint_publisher <x> <y> <z> <yaw_deg> --ros-args -r __ns:=/crazyflie2
```

> **Note** The firmware position controller has the same limits as in the ROS 1 version: the altitude oscillates for about 30 s after take-off, lateral waypoints are not tracked well, and in long flights (more than 30-40 s) the position slowly drifts away. This happens with the ROS 1 version too (`dev/ros-noetic`), so it comes from the controller tuning and not from the port. For trajectories and long flights use the Lee geometric controller or the Internal Model Controller, which are stable.

To record a flight use `ros2 bag record /crazyflie2/odometry /crazyflie2/command/motor_speed` (the CSV logging of the ROS 1 version has been removed).

How it works
------------

`spawn_crazyflie.launch.py` creates the drone in Gazebo from `rotors_description/urdf/crazyflie2.urdf.xacro` and starts a `ros_gz_bridge` for its topics. Everything lives in the namespace of the drone (`crazyflie2` by default):

| Topic | Type | Direction |
|---|---|---|
| `odometry` | `nav_msgs/Odometry` | Gazebo → ROS, 1 kHz, ideal (pose in the world frame, twist in the body frame) |
| `imu` | `sensor_msgs/Imu` | Gazebo → ROS, 500 Hz, ideal or with the MPU-9250 noise (`noisy_imu:=true`) |
| `joint_states` | `sensor_msgs/JointState` | Gazebo → ROS, propeller joints |
| `command/motor_speed` | `actuator_msgs/Actuators` | ROS → Gazebo, propeller speeds [rad/s] in the `velocity` field |
| `command/trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | waypoint for the firmware position controller |
| `drone_state` | `rotors_comm/DroneState` | reference for the Internal Model Controller and the Lee controller |

The controllers are in `rotors_control`. The control library does not depend on ROS, and the nodes only convert messages. The gains are in `rotors_gazebo/config`, written as ROS 2 parameter files.

The propeller model has two sets of coefficients: the default CrazyS ones and the ones identified by J. Forster (ETH Zurich, 2015), selected with `forster_parameters:=true`. The Lee controller was tuned with Forster's coefficients, so its launch files select them.

Differences from the ROS 1 version
----------------------------------

* Only the Crazyflie is ported. The other RotorS vehicles, the fixed-wing aircraft, the VI sensor, the octomap example, `rotors_evaluation` and `rqt_rotors` are not part of this branch.
* The Mellinger controller is not ported: in ROS 1 it was a draft that never worked.
* State estimator mode: in ROS 1 the ideal odometry overwrote the attitude estimated from the IMU, and in Noetic the drone flipped after a few seconds. Here only position and linear velocity come from the odometry, and the gyroscope bias is estimated on the ground before take-off, as the firmware does.
* Position controller: fixed the lower saturation of the thrust correction (`+1718` instead of `-1718`).
* Lee controller: the control step is triggered by the time stamps of the odometry, and the dirty derivatives use the real time between samples. Under load (e.g. in the swarm), the ROS 2 timers drifted by up to ±20% with respect to the simulation time, and the fourth order derivatives made the drone unstable.
* `waypoint_publisher_file` sends the waypoints one at a time (in ROS 1 they were sent in a single message, but the controller only used the first one).

Credits
-------

CrazyS was developed by Giuseppe Silano, Emanuele Aucone and Luigi Iannelli, with contributions by Benjamin Rodriguez, Ria Sonecha and Daniele Morra. The Lee geometric controller was developed by Stefano Grossi in his MSc thesis at the University of Modena and Reggio Emilia (2021), supervised by Lorenzo Sabattini, Beatrice Capelli and Federico Pratissoli.

> **Note** The ROS 2 port was carried out with the help of Claude, an AI model by Anthropic, used through Claude Code. The commits written this way list Claude as author or co-author. Every example in the table above was run in Gazebo Harmonic and compared with the ROS 1 version, but the port has not been tested on a real Crazyflie. If something does not work as expected, please open an [issue](https://github.com/gsilano/CrazyS/issues).

License
-------

Apache License 2.0, see [LICENSE](LICENSE).
