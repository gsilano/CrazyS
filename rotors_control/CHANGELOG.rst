^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2018-06-04)
------------------
* bug fixing in the controller position node and in the controller position, complementary filter and sensfusion6 algorithms
* the controller position node algorithm has been split into two parts: with and without state estimator
* the position controller algorithm now is able to handle both ideal and real simulations (with and without real sensors)
* Contributors: Giuseppe Silano, Luigi Iannelli

0.0.2 (2018-02-23)
------------------
* added the Crazyflie 2.0 default state estimator: complementary filter, in according to the last firmware release 2018.01.01
* modified the position controller in order to take into account the state estimator
* Contributors: Giuseppe Silano, Luigi Iannelli

0.0.1 (2018-02-02)
------------------
* added Crazyflie 2.0 position controller. The lower level controller is the same of the Crazyflie 2.0 firmware (released 2018.01.01)
* started from the 2.1.1 (2017-04-27) release version of RotorS
* Contributors: Giuseppe Silano, Emanuele Aucone, Benjamin Rodriguez, Luigi Iannelli

