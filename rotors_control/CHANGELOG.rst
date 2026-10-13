^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.1.0 (2026-10-10)
------------------
* Add Lee's geometric controller (lee_geometric_controller_node), from Stefano Grossi's MSc thesis (#86)
* Lee controller: one control step every Ts seconds of the odometry time stamps instead of a ROS timer; the dirty derivatives use the time between the samples
* Position controller without state estimator: the integrators use the time between two odometry messages. The controller runs at every message (1 kHz) but integrated with a 10 ms step, so all the integral gains were ten times too large and the drone drifted away in long flights
* Position controller: fix the lower saturation of the thrust correction, which gave +1718 instead of -1718
* Contributors: Stefano Grossi, Lorenzo Sabattini, Federico Pratissoli, Beatrice Capelli, Giuseppe Silano

7.0.1 (2021-12-26)
------------------
* Initial release of CrazyS for ROS Noetic. Add CMake 3.0 as required. Add version to the project name.
* Contributors: Daniele Morra, Giuseppe Silano
