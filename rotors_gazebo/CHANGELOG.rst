^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
