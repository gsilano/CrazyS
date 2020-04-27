## Installation Instructions - Arch Linux with ROS Melodic and Gazebo 10

To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Before Start

Since a lot of packages will be installed, it is recommended to use an AUR helper like [yay](https://aur.archlinux.org/packages/yay/) or [pikaur](https://aur.archlinux.org/packages/pikaur/). A larger more verbose list of AUR helpers can be found within the ArchLinux wiki page for [AUR helpers](https://wiki.archlinux.org/index.php/AUR_helpers). This tutorial will assume the use of yay as the AUR helper.

It is also recommended to use the [arch4edu](https://wiki.archlinux.org/index.php/Unofficial_user_repositories#arch4edu) repository. They are hosting many packages related to education and research, including robotics. Adding a repository allows installing binaries of packages, instead of compiling them from source. This will greatly speed up your installation time. [Visit here](https://github.com/arch4edu/arch4edu/wiki/Add-arch4edu-to-your-Archlinux) to add and use arch4edu.

Another option is the [oscloud](https://wiki.archlinux.org/index.php/Unofficial_user_repositories#oscloud) repository.

2. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ yay -S ros-melodic-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ yay -S ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink
$ yay -S python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
$ yay -S python-rosinstall python-rosinstall-generator build-essential
```

3. If you don't have ROS workspace yet you can do so by

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

4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ yay -S ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-mav-msgs ros-melodic-rotors-control
$ yay -S ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation ros-melodic-rotors-joy-interface
$ yay -S ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs ros-melodic-rotors-description ros-melodic-rotors-hil-interface
$ rosdep update
$ catkin build
```

5. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

6. Run the hovering example
```console
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch
```

In the event that the simulation does not start, the problem may be related to Gazebo and missing packages. It is recommended to set options `debug` and `verbose` to `true` in `crazyflie2_hovering_example.launch` to further inspect the issue, in the command-line output of the simulation. 

If the problem does not fall in one of the following categories you might need to investigate on your own. Inspect the outputs and open an issue. Some details are reported in [#25](https://github.com/gsilano/CrazyS/issues/25) and in [#40](https://github.com/gsilano/CrazyS/issues/40).

#### Missing `libmav_msgs`

As a result of using `catkin build` (instead of `catkin_make`), it is possible that the dependency for `libmav_msgs` is not satisfied. If `libmav_msgs.so` is not present in `devel/lib/` you need to copy it there or link to it from `build/rotors_gazebo_plugins/libmav_msgs.so`. Use the following commands to create a soft link to the shared object.

```console
cd ~/catkin_wc
ln -s build/rotors_gazebo_plugins/libmav_msgs.so devel/lib/libmav_msgs.so
```

#### Dependency on an older library version
Arch Linux is a rolling release. If any other shared libraries cannot be located it is quite possible that they have been updated while some other dependency still depends on the previous version. In such cases it is common to create a soft link in place of the previous version. Below an example of the process for `libopencv_imgproc.so`. The system has version 4.3 while the binary is looking for 4.2.

```console
# Example
ln -s /usr/lib/libopencv_imgproc.so.4.3  /usr/lib/libopencv_imgproc.so.4.2
```

#### Blanket installation
As a last resort, a blanket installation of all the `ros-melodic-*` packages can be done as follows:
```console
$ yay -Ssq ros-melodic-             # First, inspect the packages
$ yay -S $(yay -Ssq ros-melodic-)   # Then, install
```

## Contributors
* [Giuseppe Silano](https://giuseppesilano.net/)
* @KomaGR