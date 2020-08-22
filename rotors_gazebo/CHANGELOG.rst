^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.0.8 (2020-08-22)
------------------

6.0.7 (2020-05-13)
------------------
* Add a simple scenario in which using the Octomap plugin with the Crazyflie 2.0
* Fixed issue related to the world file #50
* Contributors: Giuseppe Silano

6.0.6 (2020-05-02)
------------------
* Add launch file for simulating the Crazyflie 2.0 with a VI sensor. In addition, an RVIZ configuration file allows to see the camera images and point cloud data
* Contributors: Giuseppe Silano

6.0.5 (2020-04-23)
------------------
* Add a swarm example with the Crazyflie 2.0
* The spline trajectory generator now is a shared library
* Add a trajectory generator when using splines for the swarm example
* Contributors: Giuseppe Silano

6.0.4 (2020-04-14)
------------------
* Add INDI and Mellinger's (it does not work yet) controllers to the spaw_mav_crazyflie.launch file
* Add spline trajectory generator
* Add launch files to run the Internal Model and Mellinger's controllers
* Add resource files for the above controllers and trajectory generator
* Contributors: Ria Sonecha, Giuseppe Silano

6.0.3 (2020-03-22)
------------------
* Add data saving features in crazyflie2_hovering_example.launch
* Delete useless plots in crazyflie2_hovering_example.launch
* Contributors: Giuseppe Silano

6.0.2 (2020-02-09)
------------------
* Add resource file for the Crazyflie's on-board controller when the joystick interface is active
* Add lunch file for piloting the Crazyflie with the joystick
* Contributors: Giuseppe Silano

6.0.1 (2019-12-28)
------------------
* Fix issue related to "xacro.py is deprecated; please use xacro instead"
* Contributors: Giuseppe Silano

4.0.6 (2019-01-04)
------------------

4.0.5 (2018-12-17)
------------------
* The launch file that allows Crazyflie simulation when Matlab and Robotics System Toolbox are in the loop has been inserted in the repository
* Contributors: Giuseppe Silano

4.0.4 (2018-09-30)
------------------
* Fix issue #6
* The gains in the "controller_crazyflie2_with_stateEstimator.yaml" file have been changed.
* Contributors: Giuseppe Silano

4.0.3 (2018-06-04)
-----------------
* added the basic_crazyflie.world file by moving the sampling time from 0.01 to 0.001 seconds
* the crazyflie control gains have been split into two files for the simulation with and without state estimator, respectively
* the crazyfie_hovering_example.launch file now is able to simulate both the ideal and the real sensors. In other words, the crazyflie with and without the state estimator (the complementary filter)
* the bug in the orientation plot has been fixed
* two yaml file have been made to simulate the behavior of the drone with and without the complementary filter
* Contributors: Giuseppe Silano

4.0.2 (2018-02-23)
------------------
* modified the controller_crazyflie2.yaml, adding the integral gain on yaw position controller
* Contributors: Giuseppe Silano, Luigi Iannelli

4.0.1 (2018-02-02)
------------------
* initial package Ubuntu release
* added the Crazyflie 2.0 hovering example launch file
* added the Crazyflie 2.0 parameter and controller yaml files
* added the Quaternion to RPY node (XYZ conversion and not ZYX)
* Contributors: Giuseppe Silano, Emanuele Aucone, Benjamin Rodriguez, Luigi Iannelli
