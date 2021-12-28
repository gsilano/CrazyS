[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg?style=flat-square)](https://www.firsttimersonly.com/)
[![All Contributors](https://img.shields.io/badge/all_contributors-6-orange.svg?style=flat-square)](#contributors)

CrazyS
===============

| CrazyS build status | [![Build Status](https://github.com/gsilano/CrazyS/actions/workflows/kinetic.yml/badge.svg)](https://github.com/gsilano/CrazyS/actions/workflows/kinetic.yml) | [![Build Status](https://github.com/gsilano/CrazyS/actions/workflows/melodic.yml/badge.svg)](https://github.com/gsilano/CrazyS/actions/workflows/melodic.yml) | [![Build Status](https://github.com/gsilano/CrazyS/actions/workflows/noetic.yml/badge.svg)](https://github.com/gsilano/CrazyS/actions/workflows/noetic.yml)
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|  

CrazyS is an extension of the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to modeling, developing and integrating the [Crazyflie 2.0](https://www.bitcraze.io/crazyflie-2/) nano-quadcopter in the physics based simulation environment Gazebo. The contribution can be also considered as a reference guide for expanding the RotorS functionalities in the Unmanned Aerial Vehicles (UAVs) field, by facilitating the integration of new aircraft.

Such simulation platform allows to understand quickly the behavior of the flight control system by comparing and evaluating different indoor and outdoor scenarios, with a details level quite close to reality. The proposed extension expands RotorS capabilities by considering the Crazyflie 2.0 physical model and its flight control system, as well (the [2018.01.1](https://github.com/bitcraze/crazyflie-firmware/releases/tag/2018.01.1) firmware release).

A simple case study is considered (`crazyflie2_hovering_example.launch`) in order to show how the package works and the validity of the employed dynamical model together the control architecture of the quadrotor. The code is released under Apache license, thus making it available for scientific and educational activities.

The platform was developed using Ubuntu 16.04 and the Kinetic Kame version of ROS, but it is also fully compatible with Ubuntu 18.04 along with the Melodic Morenia distribution of ROS and Ubuntu 20.0.4 along with the Noetic Ninjemys release. Although backwards compatibility is guarantee, i.e., the platform is fully compatible with Indigo Igloo version of ROS and Ubuntu 14.04, such configuration is not recommended since the ROS support is expected to be closed in April 2019.

Below we provide the instructions necessary for getting started. See [CrazyS' wiki](https://github.com/gsilano/CrazyS/wiki) for more instructions and examples.

If you are using this simulator for research purposes especially for your publication, please take a look at the [Publications page](https://github.com/gsilano/CrazyS/wiki/Publications). The page contains the core papers and all related works (using the platform). For the sake of simplicity, here the references to the core papers are reported.

```console
@INBOOKk{Silano2019ROSVolume4,
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

Installation Instructions - Ubuntu 20.04 with ROS Neotic and Gazebo 11
-----------------------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
$ sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
$ sudo apt install python3-vcstool python-catkin-tools protobuf-compiler libgoogle-glog-dev
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

Launching the simulation is quite simple, so as customizing it: it is enough to run in a terminal the command

```console
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database. To avoid any problems when starting the simulation for the first time, you may run the `gazebo` command in the terminal line.

By default the state estimator is disabled since on-board Crazyflie's sensors are replaced by the odometry one. For running the simulation by taking into account the Crazyflie's IMU and the complementary filter, it is enough to give a command that turns on the flag `enable state estimator`:

```console
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch enable_state_estimator:=true
```

The visual outcome will see the nano-quadcopter taking off after 5s (time after which the hovering example node publishes the trajectory to follow) and flying one meter above the ground, at the same time keeping near to zero the position components along x and y-axis.

The whole process is the following: the desired trajectory coordinates (x_r, y_r, z_r and \psi_r) are published by the `hovering_example` node on the topic `command/trajectory`, to whom the `position_controller` node (i.e., the Crazyflie controller) is subscribed. The drone state (`odometry_sensor1/odometry` topic) and the references are used to run the control strategy designed for the position tracking. The outputs of the control algorithm consist into the actuation commands (\omega_1, \omega_2, \omega_3 and \omega_4) sent to Gazebo (`command/motor_speed`) for the physical simulation and the corresponding graphical rendering, so to visually update the aircraft position and orientation. When the state estimator is turned off, the drone orientation (\phi_k, \theta_k and \psi_k) and angular velocities (p_k, q_k and r_k) published on the topic odometry are replaced by the ideal values coming from the odometry sensor.

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/CrazyS/rotors_gazebo/launch`. Such scenarios are better explained in the [RotorS](https://github.com/ethz-asl/rotors_simulator) repository.

The `world_name` argument looks for a `.world` file with a corresponding name in `~/catkin_ws/src/CrazyS/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

Using the `csvFilesStoring` variable is possible to enable (true) or disable (false) the data storage. The log files are saved in the home directory (the path can be easily changed modifying the `position_controller.cpp` file). The recording time can be set via the `csvFilesStoringTime` while the user account can be set via the `user_account` variable. Of course, the log features can be used with and without the complementary filter.

```console
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch csvFilesStoring:=true
```

An alternative controller is available on the repository. For running the simulation by using the Internal Model Control as described in [#27](https://github.com/gsilano/CrazyS/issues/27), [#28](https://github.com/gsilano/CrazyS/issues/28), [#29](https://github.com/gsilano/CrazyS/issues/29), and [#48](https://github.com/gsilano/CrazyS/issues/48) simply run

```console
$ roslaunch rotors_gazebo crazyflie2_internal_model_controller.launch
```

or

```console
$ roslaunch rotors_gazebo crazyflie2_internal_model_controller_vi_sensor.launch
```

while a simple swarm example is available at

```console
$ roslaunch rotors_gazebo crazyflie2_swarm_hovering_example.launch
```

> **Note** There is also a draft of the Mellinger's controller implementation in the package. This is a NOT WORKING example. As soon as the problems are resolved, a working version will be made available.

The package also provides a launch file for piloting the Crazyflie using a PC joystick. To run the simulation simple copy and paste the command in the following in a terminal window

```console
$ roslaunch rotors_gazebo crazyflie2_with_joy.launch
```

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionality, although the platform is fully compatible with the Gazebo 9. Before running the script, consider the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

Bugs & Feature Requests
--------------

Please report bugs and request features by using the [Issue Tracker](https://github.com/gsilano/CrazyS/issues). Furthermore, please see
the [Contributing.md](https://github.com/gsilano/CrazyS/blob/master/CONTRIBUTING.md) file if you plan to help us to improve
CrazyS features.

YouTube videos
--------------

In this section a video providing the effectiveness of the platform and how it works is reported. Further videos can be found in the related YouTube channel. Have fun! :)

[![CrazyS, an extension of the ROS package RotorS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter](https://github.com/gsilano/CrazyS/wiki/img/img_YouTube_MED18.png)](https://youtu.be/qsrYCUSQ-S4 "CrazyS, an exntension of the ROS pakcage RotrS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter")
