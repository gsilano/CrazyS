^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.1.0 (2026-10-10)
------------------
* Add Lee's geometric controller (lee_geometric_controller_node), from Stefano Grossi's MSc thesis (#86)
* Lee controller: one control step every Ts seconds of the odometry time stamps instead of a ROS timer; the dirty derivatives use the time between the samples
* Contributors: Stefano Grossi, Lorenzo Sabattini, Federico Pratissoli, Beatrice Capelli, Giuseppe Silano

7.0.1 (2021-12-26)
------------------
* Initial release of CrazyS for ROS Noetic. Add CMake 3.0 as required. Add version to the project name.
* Contributors: Daniele Morra, Giuseppe Silano
