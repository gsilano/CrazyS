^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.0.8 (2020-08-22)
------------------
* Add crazyflie2_forster.xacro file with Forster's parameters as described in #56
* Contributors: Tonirayn

6.0.7 (2020-05-13)
------------------

6.0.6 (2020-05-02)
------------------
* Add a VI sensor for the Crazyflie without shape and visual part
* Contributors: Giuseppe Silano

6.0.5 (2020-04-23)
------------------
* Fix issue according to https://github.com/gsilano/CrazyS/issues/44
* Changed the "measurement_divisor" (from 10 to 1) at line 895 in the component_snippets.xacro file
* Contributors: Giuseppe Silano

6.0.4 (2020-04-14)
------------------
* Add ideal imu sensor for the Crazyflie
* Contributors: Ria Sonecha, Giuseppe Silano

6.0.3 (2020-03-22)
------------------

6.0.2 (2020-02-09)
------------------

6.0.1 (2019-12-28)
------------------
* Deleted pi constant in the component_snippest file.
* Contributors: Giuseppe Silano

4.0.6 (2019-01-04)
------------------

4.0.5 (2018-12-17)
------------------

4.0.4 (2018-09-30)
------------------

4.0.3 (2018-06-04)
-------------------
* An if else structure has been added to the the crazyflie_base.xacro file. Such structure allows to able or unable thw real sensors based on the simulation request. In other words, when the complementary filter is unable, the ideal sensors are simulated and vice-versa.
* Contributors: Giuseppe Silano

4.0.2 (2018-02-23)
-------------------
* added the Crazyflie 2.0 IMU (MPU-9250) inside the component_snippest.xacro file. The IMU substitutes the ideal sensor previously employed in the crazyfie_base.xacro
* Contributors: Giuseppe Silano, Luigi Iannelli

4.0.1 (2018-02-02)
------------------
* initial Ubuntu package release
* added Crazyflie 2.0 urdf: crazyflie2.xacro and crazyflie2_base.xacro. The MPU-9250 IMU plugin is under developing.
* added Crazyflie 2.0 mesh file
* Contributors: Giuseppe Silano, Emanuele Aucone, Benjamin Rodriguez, Luigi Iannelli
