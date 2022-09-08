^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.1.0 (2021-11-09)
------------------

6.0.9 (2021-05-23)
------------------

6.0.8 (2020-08-22)
------------------

6.0.7 (2020-05-13)
------------------

6.0.6 (2020-05-02)
------------------

6.0.5 (2020-04-23)
------------------
* Contributors: fix missing input error when running the control algorithm
* Contributors: Giuseppe Silano

6.0.4 (2020-04-14)
------------------
* Improvements in the include folder
* Add INDI and Mellinger's (it does not work yet) controllers
* Contributors: Ria Sonecha, Giuseppe Silano

6.0.3 (2020-03-22)
------------------
* Add data saving feature
* Contributors: Giuseppe Silano

6.0.2 (2020-01-18)
------------------
* Add data saving feature
* Contributors: Giuseppe Silano

6.0.2 (2020-02-09)
------------------
* Fix typo in the position_controller_node with the enable_state_estimator variable #24
* Add RollPitchYawRateThrust controller library for piloting the Crazyflie using the joystick #30
* Contributors: Giuseppe Silano

6.0.1 (2019-12-28)
------------------

4.0.6 (2019-01-04)
------------------
* Changes in the position controller libray to fix issue in the building process
* Contributors: Giuseppe Silano

4.0.5 (2018-12-17)
------------------
* The position_controller_node_with_stateEstimator and position_controller_node_without_stateEstimator files were rewritten as an unique controller: the position_controller_node
* The comments have been reviewed improving the readability of the code
* Some bugs have been fixed
* Contributors: Giuseppe Silano

4.0.4 (2018-09-30)
------------------
* Names update. Some cpp files have been renamed in a more general way making them independent of Crazyflie (e.g., the complementary filter).
* Contributors: Giuseppe Silano

4.0.3 (2018-06-04)
------------------
* bug fixing in the controller position node and in the controller position, complementary filter and sensfusion6 algorithms
* the controller position node algorithm has been split into two parts: with and without state estimator
* the position controller algorithm now is able to handle both ideal and real simulations (with and without real sensors)
* the control algorithm has been split into two parts: high and low level. In this way, the high level part can be restructured without changes in the low one. Indeed, the Crazyflie on board control architectured does not change unless the firmware is changed
* Contributors: Giuseppe Silano, Luigi Iannelli

4.0.2 (2018-02-23)
------------------
* added the Crazyflie 2.0 default state estimator: complementary filter, in according to the last firmware release 2018.01.01
* modified the position controller in order to take into account the state estimator
* Contributors: Giuseppe Silano, Luigi Iannelli

4.0.1 (2018-02-02)
------------------
* added Crazyflie 2.0 position controller. The lower level controller is the same of the Crazyflie 2.0 firmware (released 2018.01.01)
* started from the 2.1.1 (2017-04-27) release version of RotorS
* Contributors: Giuseppe Silano, Emanuele Aucone, Benjamin Rodriguez, Luigi Iannelli
