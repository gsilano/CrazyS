^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

8.0.0 (2026-10-10)
------------------
* Port to ROS 2 Jazzy. The control library no longer depends on ROS, the nodes use rclcpp
* Keep the Crazyflie controllers: firmware position controller (with and without state estimator), Internal Model Controller, Lee geometric controller and the roll/pitch/yaw rate/thrust controller used with the joystick
* Remove the RotorS controllers for the other multicopters and the Mellinger controller draft, which never worked
* State estimator mode: the odometry no longer overwrites the attitude estimated from the IMU, and the gyroscope bias is estimated on the ground before take-off
* Position controller: fix the lower saturation of the thrust correction, which gave +1718 instead of -1718
* Position controller without state estimator: the integrators use the time between two odometry messages. The controller runs at every message (1 kHz) but integrated with a 10 ms step, so all the integral gains were ten times too large and the drone drifted away in long flights
* Lee geometric controller: the control step is triggered by the time stamps of the odometry and the dirty derivatives use the real time between samples, so the controller stays stable when the computer is loaded (e.g. in the swarm example)
* Remove the CSV logging of the position controller, use ros2 bag instead
* Add unit tests of the control library
* Contributors: Giuseppe Silano

7.1.0 (2026-10-10)
------------------
* Add Lee's geometric controller (lee_geometric_controller_node), from Stefano Grossi's MSc thesis (#86)
* Contributors: Stefano Grossi, Lorenzo Sabattini, Federico Pratissoli, Beatrice Capelli, Giuseppe Silano

7.0.1 (2021-12-26)
------------------
* Initial release of CrazyS for ROS Noetic. Add CMake 3.0 as required. Add version to the project name.
* Contributors: Daniele Morra, Giuseppe Silano
