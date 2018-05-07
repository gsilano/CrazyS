[![Build Status](https://travis-ci.org/gsilano/CrazyS.svg?branch=master)](https://travis-ci.org/gsilano/CrazyS)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

CrazyS
===============

CrazyS is an extension of the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to modeling, developing and integrating the [Crazyflie 2.0](https://www.bitcraze.io/crazyflie-2/) nano-quadcopter in the physics based simulation environment Gazebo.

Such simulation platform allows to understand quickly the behavior of the flight control system by comparing and evaluating different indoor and outdoor scenarios, with details level quite close to reality. The proposed extension expands RotorS capabilities by considering the Crazyflie 2.0 physical model and its flight control system, as well.

A simple case study is considered (`crazyflie2_hovering_example.launch`) in order to show how the package works and the validity of the employed dynamical model together the control architecture of the quadcopter.

The code is released under Apache license, thus making it available for scientific and educational activities.

Below we provide the instructions necessary for getting started. See CrazyS' wiki for more instructions and examples (https://github.com/gsilano/CrazyS/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@INPROCEEDINGS{Silano2018,
author = {G. Silano and E. Aucone and L. Iannelli},
booktitle = {2018 26th Mediterranean Conference on Control and Automation (MED)}, 
title = {CrazyS: a software-in-the-loop platform for the Crazyflie 2.0 nano-quadcopter},
year = {2018}, 
volume = {}, 
number = {}, 
month = {June},
}
```
Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/kinetic/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 $ wget https://raw.githubusercontent.com/gsilano/rotors_simulator/master/rotors_hil.rosinstall
 $ wstool merge rotors_hil.rosinstall
 $ wstool update
 ```

  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```
Installation Instructions - Ubuntu 14.04 with ROS Indigo
--------------------------------------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/indigo/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/gsilano/CrazyS.git
 $ git clone https://github.com/gsilano/mav_comm.git
 ```
  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

  > **Note** if you want to use `wstool` you can replace the above commands with
    ```
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```
  > **Note** if you want to build and use the `gazebo_mavlink_interface` plugin you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace.
    ```
    https://github.com/mavlink/mavros
    ```
 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin init  # If you haven't done this before.
   $ catkin build
   ```
   > **Note** if you are getting errors related to "future" package, you may need python future:
    ```
    sudo apt-get install python-pip
    pip install --upgrade pip
    pip install future
    ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```
Basic Usage
-----------

Launch the simulator with the Crazyflie 2.0 model in a basic world.

```
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database.

The whole process is the following: the desired trajectory coordinates (![equation](http://latex.codecogs.com/gif.latex?%24x_r%24%2C%20%24y_r%24%2C%20%24z_r%24%2C%20%24%5Cpsi_r%24)) are published by the hovering example node on the topic `command/trajectory`, to whom the position controller
node (i.e., the Crazyflie controller) is subscribed. The drone state (`odometry_sensor1/odometry` topic) and the references are used to run the control strategy designed for the path tracking. The outputs of the control algorithm consists into the actuation commands (![equation](http://latex.codecogs.com/gif.latex?%24%5Comega_1%24%2C%20%24%5Comega_2%24%2C%20%24%5Comega_3%24%2C%20%24%5Comega_4%24)) sent to Gazebo (`command/motor_speed`) for the virtual rendering, so to update the aircraft position and orientation.

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/CrazyS/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/CrazyS/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, as there are the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

YouTube videos
--------------

In such section a video proving the effectiveness of the platform is reported. Further videos can be found in the related YouTube channel. Have fun! :)

[![CrazyS, an exntension of the ROS pakcage RotrS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter](https://github.com/gsilano/CrazyS/wiki/img/img_YouTube_MED18.png)](https://www.youtube.com/watch?v=pda-tuULewM "CrazyS, an exntension of the ROS pakcage RotrS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter")
