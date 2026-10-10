[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg?style=flat-square)](https://www.firsttimersonly.com/)
[![All Contributors](https://img.shields.io/badge/all_contributors-6-orange.svg?style=flat-square)](#contributors)

CrazyS
===============

| Branch | ROS and Gazebo | Versions | Build |
|---|---|---|---|
| `master` | Kinetic + Gazebo 7 | 4.x | [![Kinetic + Gazebo 7](https://github.com/gsilano/CrazyS/actions/workflows/build-kinetic-gazebo7.yml/badge.svg?branch=master)](https://github.com/gsilano/CrazyS/actions/workflows/build-kinetic-gazebo7.yml) |
| `dev/gazebo9` | Kinetic + Gazebo 9 | 5.x | [![Kinetic + Gazebo 9](https://github.com/gsilano/CrazyS/actions/workflows/build-kinetic-gazebo9.yml/badge.svg?branch=dev/gazebo9)](https://github.com/gsilano/CrazyS/actions/workflows/build-kinetic-gazebo9.yml) |
| `dev/ros-melodic` | Melodic + Gazebo 9 | 6.x | [![Melodic + Gazebo 9](https://github.com/gsilano/CrazyS/actions/workflows/build-melodic.yml/badge.svg?branch=dev/ros-melodic)](https://github.com/gsilano/CrazyS/actions/workflows/build-melodic.yml) |
| `dev/ros-noetic` | Noetic + Gazebo 11 | 7.x | [![Noetic + Gazebo 11](https://github.com/gsilano/CrazyS/actions/workflows/build-noetic.yml/badge.svg?branch=dev/ros-noetic)](https://github.com/gsilano/CrazyS/actions/workflows/build-noetic.yml) |
| `dev/ros2-jazzy` | ROS 2 Jazzy + Gazebo Harmonic | 8.x | [![ROS 2 Jazzy + Gazebo Harmonic](https://github.com/gsilano/CrazyS/actions/workflows/build-jazzy.yml/badge.svg?branch=dev/ros2-jazzy)](https://github.com/gsilano/CrazyS/actions/workflows/build-jazzy.yml) |

Each branch is built on every push and pull request to it, so a red badge points to the branch that broke. On top of that, the [weekly check](https://github.com/gsilano/CrazyS/actions/workflows/weekly.yml) builds all the branches every Monday, to catch problems caused by the dependencies.

CrazyS extends the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator) with a model of the [Crazyflie 2.0](https://www.bitcraze.io/crazyflie-2/) nano-quadcopter for the Gazebo simulator. It can also be used as a guide for adding new aircraft to RotorS.

Besides the physical model of the drone, CrazyS includes its flight control system, taken from the [2018.01.1](https://github.com/bitcraze/crazyflie-firmware/releases/tag/2018.01.1) firmware release, so control algorithms can be tested in indoor and outdoor scenarios before flying the real drone. The `crazyflie2_hovering_example.launch` file is a simple starting point. The code is released under the Apache license.

CrazyS was developed on Ubuntu 16.04 with ROS Kinetic, and it also works with ROS Melodic (Ubuntu 18.04), ROS Noetic (Ubuntu 20.04) and, on the `dev/ros2-jazzy` branch, ROS 2 Jazzy (Ubuntu 24.04). Instructions for ROS Indigo (Ubuntu 14.04) are kept below, but that setup is no longer recommended.

The instructions to get started are below. More examples are in the [wiki](https://github.com/gsilano/CrazyS/wiki).

If you use CrazyS in your research, please cite the papers below. The [Publications page](https://github.com/gsilano/CrazyS/wiki/Publications) lists them together with the works that used the platform.

```console
@INBOOK{Silano2019ROSVolume4,
  author = {Silano, G. and Iannelli, L.},
  editor = {Koubaa, A.},
  title = {Robot Operating System (ROS): The Complete Reference (Volume 4)},
  chapter = {{CrazyS: a software-in-the-loop simulation platform for the Crazyflie 2.0 nano-quadcopter}},
  publisher = {Springer International Publishing},
  pages = {81--115},
  isbn = {978-3-030-20190-6},
  doi = {10.1007/978-3-030-20190-6_4},
  link = {https://link.springer.com/chapter/10.1007/978-3-030-20190-6_4},
  year = {2020}
}

@INPROCEEDINGS{Silano2018MED,
  author = {Silano, G. and Aucone, E. and Iannelli, L.},
  booktitle = {2018 26th Mediterranean Conference on Control and Automation (MED)},
  title = {{CrazyS: A Software-In-The-Loop Platform for the Crazyflie 2.0 Nano-Quadcopter}},
  year = {2018},
  pages = {352--357},
  doi = {10.1109/MED.2018.8442759},
  issn = {2473-3504},
  month = June,
  link = {https://ieeexplore.ieee.org/document/8442759},  
}
```

To facilitate the use of the repository, in addition to the installation instructions, the following are links to two virtual machines created using Oracle VirtualBox.

[Ubuntu 16.04 with ROS Kinetic and Gazebo 7](https://mega.nz/#!oxdwzaIY!C27ekXwbObbiU37BrVQSjeRMP2HCACVgQj1WbvVmXlA)

[Ubuntu 16.04 with ROS Kinetic and Gazebo 9](https://mega.nz/#!550ASCrZ!JYm1mFFlYa9fNh_Cb4Z6p7l9k6Vu5tpRJB3NjMoSF1A)

```
USER: user
PASS: password
Keyboard layout: Italian
Language: English
```

Installation Instructions - Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic
------------------------------------------------------------------------------

The ROS 2 version lives on the `dev/ros2-jazzy` branch. It uses the systems of Gazebo Sim (Harmonic) instead of the RotorS plugins and does not need `mav_comm`. Only the Crazyflie and its controllers are ported; the other RotorS vehicles are not. The examples are listed in [Basic Usage with ROS 2](#basic-usage-with-ros-2) below; the [README of that branch](https://github.com/gsilano/CrazyS/tree/dev/ros2-jazzy) also describes the topics and the differences from the ROS 1 version.

```console
$ sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-actuator-msgs ros-jazzy-xacro ros-jazzy-joy python3-colcon-common-extensions
$ mkdir -p ~/crazys_ws/src && cd ~/crazys_ws/src
$ git clone -b dev/ros2-jazzy https://github.com/gsilano/CrazyS.git
$ cd ~/crazys_ws && colcon build && source install/setup.bash
$ ros2 launch rotors_gazebo crazyflie2_hovering_example.launch.py
```

Installation Instructions - Ubuntu 20.04 with ROS Noetic and Gazebo 11
-----------------------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Noetic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
$ sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
$ sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python3-rosdep python3-wstool ros-noetic-ros libgoogle-glog-dev
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
```

3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ rosdep update
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
$ catkin build
```

4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

> In the event the `cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -u -e ode` appear, the solution proposed in [#40](https://github.com/gsilano/CrazyS/issues/40) temporany fixes the issue.

```console
$ cp build/rotors_gazebo_plugins/libmav_msgs.so devel/lib/
```

Installation Instructions - Ubuntu 18.04 with ROS Melodic and Gazebo 9
-----------------------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros
$ sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev
$ sudo apt install ros-melodic-control-toolbox ros-melodic-octomap-mapping ros-melodic-mavlink
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b dev/ros-melodic https://github.com/gsilano/CrazyS.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
```

3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ sudo apt install ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-rotors-description
$ sudo apt install ros-melodic-mav-msgs ros-melodic-rotors-control
$ sudo apt install ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation
$ sudo apt install ros-melodic-rotors-joy-interface ros-melodic-rotors-hil-interface
$ sudo apt install ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs
$ rosdep update
$ catkin build
```

4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

5. Update the pre-installed Gazebo version. This fixes the issue with the `error in REST request for accessing api.ignition.org`

```console
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt update
$ sudo apt install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

> In the event that the simulation does not start, the problem may be related to Gazebo and missing packages. Therefore, run the following commands. More details are reported in [#25](https://github.com/gsilano/CrazyS/issues/25).

```console
$ sudo apt-get remove ros-melodic-gazebo* gazebo*
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

> In the event the `cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -u -e ode` appear, the solution proposed in [#40](https://github.com/gsilano/CrazyS/issues/40) temporany fixes the issue.

```console
$ cp build/rotors_gazebo_plugins/libmav_msgs.so devel/lib/
```

Installation Instructions - Ubuntu 16.04 with ROS Kinetic and Gazebo 7
---------------------------------------------------------
1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros
$ sudo apt-get install ros-kinetic-mavlink python-catkin-tools protobuf-compiler
$ sudo apt-get install libgoogle-glog-dev ros-kinetic-control-toolbox ros-kinetic-octomap-mapping
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
 ```

 2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone https://github.com/gsilano/CrazyS.git
$ git clone -b crazys https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i
$ catkin build
```

> **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ cd ~/catkin_ws/
$ catkin build
```

 4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
Installation Instructions - Ubuntu 16.04 with ROS Kinetic and Gazebo 9
---------------------------------------------------------
To use the code developed and stored in this repository with ROS Kinetic and Gazebo 9, first follow what is reported in the previous section. Then, use the instruction below.

1. Remove Gazebo 7 and all related packages, and then install Gazebo 9:

```console
$ sudo apt-get remove ros-kinetic-gazebo* gazebo*
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9 gazebo9-* ros-kinetic-gazebo9-*
$ sudo apt upgrade
```
> **Note** Remove `ros-kinetic-gazebo9-*` from the command `sudo apt-get install gazebo9 gazebo9-* ros-kinetic-gazebo9-*` if fetch problems should appear during the installation.

 2. Additional packages are required to build the package.

```console
$ sudo apt-get install libeigen3-dev ros-kinetic-image-view ros-kinetic-parrot-arsdk libprotobuf-dev libprotoc-dev ros-kinetic-joy-teleop ros-kinetic-nav-msgs ros-kinetic-mav-msgs libyaml-cpp-dev ros-kinetic-nodelet ros-kinetic-mav-planning-msgs ros-kinetic-urdf ros-kinetic-image-transport ros-kinetic-roslint ros-kinetic-angles ros-kinetic-cv-bridge ros-kinetic-tf2-geometry-msgs ros-kinetic-xacro ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev ros-kinetic-camera-info-manager ros-kinetic-cmake-modules ros-kinetic-gazebo-msgs ros-kinetic-mavros-msgs ros-kinetic-control-toolbox ros-kinetic-mav-msgs ros-kinetic-libmavconn ros-kinetic-mavros ros-kinetic-octomap-msgs ros-kinetic-geographic-msgs ros-kinetic-mavlink ros-kinetic-mavros-extras ros-kinetic-mav-planning-msgs ros-kinetic-joy
```
> **Note** Missing packages can be found and then installed by using the command `rosdep check --from-paths src` into the `catkin_ws` folder.

 3. Make Gazebo 9 compatible with ROS Kinetic Kame

```console
$ cd ~
$ mkdir -p ros-kinetic-gazebo9-pkgs
$ cd ros-kinetic-gazebo9-pkgs
$ git clone -b feature/ros-kinetic-gazebo9-pkgs https://github.com/gsilano/BebopS.git
$ cd BebopS
$ chmod 777 gazebo9.sh
$ ./gazebo9.sh
$ cd ~
$ sudo rm -rf ros-kinetic-gazebo9-pkgs # delete the folder after the installation
```

4. Clean the workspace and compile again the code

```console
$ cd ~/catkin_ws
$ catkin clean # digit y when required
$ cd ~/catkin_ws/src/CrazyS
$ git checkout dev/gazebo9
$ cd ~/catkin_ws/src/mav_comm
$ git checkout med18_gazebo9
$ cd ~/catkin_ws
$ catkin build
$ source ~/.bashrc
```

> **Note** In case the `ERROR[rotors_gazebo_plugins]` error is displayed, run the following commands
>```console
>$ sudo apt-get install ros-kinetic-gazebo9-plugins
>$ sudo apt-get install apt ros-kinetic-gazebo9-ros
>$ sudo apt-get install apt ros-kinetic-gazebo9-dev
>```

This guide can be used a basis for fixing what has been discussed in [ethz-asl/rotors_simulator#506](https://github.com/ethz-asl/rotors_simulator/pull/506).

Installation Instructions - Ubuntu 14.04 with ROS Indigo
--------------------------------------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c ’echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list’
$ sudo apt-key adv --keyserver hkp://ha.pool.skskeyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy
$ sudo apt-get install ros-indigo-octomap-ros python-wstool python-catkin-tools
$ sudo apt-get install protobuf compiler libgoogle-glog-dev
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ catkin init
```
> **Note** for setups with multiple workspaces please refer to the [official documentation](http://docs.ros.org/independent/api/rosinstall/html/) by replacing `rosws` by `wstool`.

 3. Get the simulator and additional dependencies

```console
$ cd ~/catkin_ws/src
$ git clone https://github.com/gsilano/CrazyS.git
$ git clone -b crazys https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i
```

> **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

> **Note** if you want to use `wstool` you can replace the above commands with
    ```console
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```
> **Note** if you want to build and use the `gazebo_mavlink_interface` plugin you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace.
    ```
    https://github.com/mavlink/mavros
    ```
 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ cd ~/catkin_ws/
$ catkin init  # If you haven't done this before.
$ catkin build
```
 > **Note** if you are getting errors related to "future" package, you may need python future:
    ```console
    sudo apt-get install python-pip
    pip install --upgrade pip
    pip install future
    ```

 5. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Basic Usage
-----------

The commands below are for the ROS 1 branches (Kinetic, Melodic and Noetic). For ROS 2 Jazzy see [Basic Usage with ROS 2](#basic-usage-with-ros-2).

To start the simulation run

```console
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch
```

> **Note** The first time, Gazebo can take a long time to start because it downloads some models. Running `gazebo` once in a terminal before launching the simulation avoids problems.

By default the state estimator is off and the controller uses the ideal odometry sensor. To use the Crazyflie IMU and the complementary filter instead, set `enable_state_estimator`:

```console
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch enable_state_estimator:=true
```

After 5 s the drone takes off and hovers one meter above the ground, keeping x and y close to zero.

How it works: the `hovering_example` node publishes the reference (x_r, y_r, z_r and \psi_r) on `command/trajectory`. The `position_controller` node, i.e., the Crazyflie controller, reads it together with the drone state (`odometry_sensor1/odometry`) and computes the motor speeds (\omega_1, \omega_2, \omega_3 and \omega_4), which are sent to Gazebo on `command/motor_speed`. With the state estimator off, attitude (\phi_k, \theta_k and \psi_k) and angular velocity (p_k, q_k and r_k) come straight from the ideal odometry sensor.

The other launch files in `rotors_gazebo/launch` load the RotorS multicopters with different sensors; they are described in the [RotorS](https://github.com/ethz-asl/rotors_simulator) repository.

The `world_name` argument selects a `.world` file in `rotors_gazebo/worlds`. Unless the launch file name says otherwise, the empty `basic.world` is used.

Set `csvFilesStoring:=true` to save the flight data in CSV files in the home directory (the path is set in `position_controller.cpp`). `csvFilesStoringTime` sets how long to record and `user_account` the user name used in the path. Logging works with and without the complementary filter.

```console
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch csvFilesStoring:=true
```

The Internal Model Controller ([#27](https://github.com/gsilano/CrazyS/issues/27), [#28](https://github.com/gsilano/CrazyS/issues/28), [#29](https://github.com/gsilano/CrazyS/issues/29), [#48](https://github.com/gsilano/CrazyS/issues/48)) can be started with

```console
$ roslaunch rotors_gazebo crazyflie2_internal_model_controller.launch
```

or

```console
$ roslaunch rotors_gazebo crazyflie2_internal_model_controller_vi_sensor.launch
```

and a simple swarm example with

```console
$ roslaunch rotors_gazebo crazyflie2_swarm_hovering_example.launch
```

### Lee's geometric controller

The package also includes the geometric controller on SE(3) by [Lee, Leok and McClamroch](https://doi.org/10.1109/CDC.2010.5717652), developed and tuned for the Crazyflie by Stefano Grossi in his MSc thesis (University of Modena and Reggio Emilia, 2021), see [#86](https://github.com/gsilano/CrazyS/pull/86). To try it

```console
$ roslaunch rotors_gazebo crazyflie2_lee_geometric_controller.launch
```

Use the `trajectory` argument to pick one of the scenarios in `rotors_gazebo/resource/trajectories`: `hovering` (default), `linear`, `linear_polynomial`, `semicircle` and `semicircle_polynomial`. The `_polynomial` versions use a cubic timing law, so the velocity is zero at the start and at the end of every segment.

```console
$ roslaunch rotors_gazebo crazyflie2_lee_geometric_controller.launch trajectory:=semicircle_polynomial
```

The `lee_geometric_controller_node` (`rotors_control`) reads the reference from the `drone_state` topic (`mav_msgs/DroneState`) and the drone state from `odometry`, and publishes the motor speeds on `command/motor_speed` at 100 Hz. The heading comes from the yaw of the reference orientation. Gains, vehicle parameters and filter time constants are in `crazyflie2_lee_geometric_controller.yaml`.

The reference is published by the `piecewise_trajectory_generator` node (`rotors_gazebo`). A trajectory is just a list of `line`, `arc` and `hold` segments in a YAML file, so new scenarios don't need any code. The syntax is described at the top of `piecewise_trajectory_generator.cpp`.

The controller was tuned with the propeller coefficients identified by J. Forster (Bachelor's thesis, ETH Zurich, 2015), so the launch file sets `forster_parameters:=true`, which loads `crazyflie2_forster.xacro`. With the default CrazyS model (`forster_parameters:=false`) the rotor drag is much higher and, since the controller has no integral action, the drone lags behind the reference.

There is also a swarm example with three drones, each one with its own controller and trajectory

```console
$ roslaunch rotors_gazebo crazyflie2_swarm_lee_geometric_controller.launch
```

> **Note** There is also a draft of the Mellinger's controller implementation in the package. This is a NOT WORKING example. As soon as the problems are resolved, a working version will be made available.

The package also provides a launch file for piloting the Crazyflie using a PC joystick. To run the simulation simple copy and paste the command in the following in a terminal window

```console
$ roslaunch rotors_gazebo crazyflie2_with_joy.launch
```

> **Note** As explained in [#90](https://github.com/gsilano/CrazyS/issues/90), the recent updates in the xacro file format may break the simulation. To avoid these issue, simply downgrade the version of `xacro` to the 1.13.3 version. Remove `xacro` and `xacro-1.14.11.egg-info` directories from `/opt/ros/noetic/lib/python3/dist-packages` folder using `sudo rm -r ...` command and install the 1.13.3 version of xacro using `pip3 install xacro==1.13.3`. This avoid further modifications to the launch files.

Basic Usage with ROS 2
----------------------

On the `dev/ros2-jazzy` branch the launch files are Python files started with `ros2 launch`. After building the workspace (see the [installation instructions](#installation-instructions---ubuntu-2404-with-ros-2-jazzy-and-gazebo-harmonic)) and sourcing `install/setup.bash`, start the hovering example with

```console
$ ros2 launch rotors_gazebo crazyflie2_hovering_example.launch.py
```

As in ROS 1, the drone takes off and hovers one meter above the ground with the controller of the Crazyflie firmware. The other examples:

| Command | What it does |
|---|---|
| `ros2 launch rotors_gazebo crazyflie2_hovering_example.launch.py enable_state_estimator:=true` | Hovering with the attitude estimated from the IMU by the complementary filter of the firmware |
| `ros2 launch rotors_gazebo crazyflie2_internal_model_controller.launch.py` | Internal Model Controller following a quintic spline |
| `ros2 launch rotors_gazebo crazyflie2_lee_geometric_controller.launch.py trajectory:=semicircle` | Lee's geometric controller; trajectories: `hovering`, `linear`, `linear_polynomial`, `semicircle`, `semicircle_polynomial` |
| `ros2 launch rotors_gazebo crazyflie2_swarm_hovering_example.launch.py` | Five drones with the Internal Model Controller |
| `ros2 launch rotors_gazebo crazyflie2_swarm_lee_geometric_controller.launch.py` | Three drones with Lee's controller |
| `ros2 launch rotors_gazebo crazyflie2_with_joy.launch.py` | Piloting with a joystick |
| `ros2 launch rotors_gazebo crazyflie2_without_controller.launch.py` | Only the drone: send the motor speeds yourself |

Every launch file accepts `gui:=false` to run Gazebo without the graphical interface. A waypoint for the firmware position controller can be sent with

```console
$ ros2 run rotors_gazebo waypoint_publisher <x> <y> <z> <yaw_deg> --ros-args -r __ns:=/crazyflie2
```

with x, y, z in meters and the yaw in degrees. To record a flight use `ros2 bag record /crazyflie2/odometry /crazyflie2/command/motor_speed`; the CSV logging of the ROS 1 version is not available.

> **Note** The firmware position controller has the same limits in ROS 2 as in ROS 1: the altitude oscillates for about 30 s after take-off, lateral waypoints are tracked poorly and in long flights the position slowly drifts. For trajectories and long flights use Lee's controller or the Internal Model Controller.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionality, although the platform is fully compatible with the Gazebo 9. Before running the script, consider the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

Credits
-------

The Lee geometric controller was developed by Stefano Grossi in his MSc thesis at the University of Modena and Reggio Emilia (2021), supervised by Lorenzo Sabattini, Beatrice Capelli and Federico Pratissoli. His original code is available at [sgrossi/estensione_crazys](https://github.com/sgrossi/estensione_crazys).

> **Note** The integration of the controller in the 4.1.0, 5.1.0, 6.1.0 and 7.1.0 releases (bug fixes, trajectory generator, launch files, documentation and the GitHub Actions workflow) was carried out with the help of Claude, an AI model by Anthropic, used through Claude Code. The commits written this way list Claude as author or co-author. The controller was built and tested in Gazebo simulation, not on a real Crazyflie. If something does not work as expected, please open an issue.

Bugs & Feature Requests
--------------

Please report bugs and request features by using the [Issue Tracker](https://github.com/gsilano/CrazyS/issues). Furthermore, please see
the [Contributing.md](https://github.com/gsilano/CrazyS/blob/master/CONTRIBUTING.md) file if you plan to help us to improve
CrazyS features.

YouTube videos
--------------

In this section a video providing the effectiveness of the platform and how it works is reported. Further videos can be found in the related YouTube channel. Have fun! :)

[![CrazyS, an extension of the ROS package RotorS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter](https://github.com/gsilano/CrazyS/wiki/img/img_YouTube_MED18.png)](https://youtu.be/qsrYCUSQ-S4 "CrazyS, an extension of the ROS package RotorS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter")
