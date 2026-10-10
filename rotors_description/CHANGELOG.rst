^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

8.0.0 (2026-10-10)
------------------
* Port to ROS 2 Jazzy and Gazebo Sim (Harmonic). The Crazyflie model uses the MulticopterMotorModel, OdometryPublisher, Imu and JointStatePublisher systems of Gazebo Sim instead of the RotorS plugins
* Remove the models of the other multicopters, the fixed-wing aircraft and the VI sensor
* Contributors: Giuseppe Silano

7.1.0 (2026-10-10)
------------------
* Add the forster_parameters argument to crazyflie2_base.xacro to load crazyflie2_forster.xacro
* Contributors: Stefano Grossi, Lorenzo Sabattini, Federico Pratissoli, Beatrice Capelli, Giuseppe Silano

7.0.1 (2021-12-26)
------------------
* Initial release of CrazyS for ROS Noetic. Add CMake 3.0 as required. Add version to the project name.
* Contributors: Daniele Morra, Giuseppe Silano
