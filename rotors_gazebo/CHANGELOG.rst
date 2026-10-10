^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

8.0.0 (2026-10-10)
------------------
* Port to ROS 2 Jazzy and Gazebo Sim (Harmonic): Python launch files, ros_gz_bridge for odometry, IMU, joint states and motor commands
* Port the hovering_example, hovering_example_spline, waypoint_publisher, waypoint_publisher_file, quaternion_to_rpy and piecewise_trajectory_generator nodes. hovering_example_spline also covers the swarm case (hovering_example_spline_swarm is removed); waypoint_publisher_file now sends the waypoints one at a time
* The piecewise_trajectory_generator reads the trajectory from the YAML file given by the trajectory_file parameter
* Swarm example of the Lee controller: drone 1 waits for drone 2 to complete its half circle, since their paths crossed
* Remove the worlds, models and launch files of the other vehicles
* Contributors: Giuseppe Silano

7.1.0 (2026-10-10)
------------------
* Add the piecewise_trajectory_generator node: DroneState references built from line, arc and hold segments in a YAML file
* Add the trajectories used in Stefano Grossi's thesis (resource/trajectories)
* Add crazyflie2_lee_geometric_controller.launch and crazyflie2_swarm_lee_geometric_controller.launch
* Add the forster_parameters argument to spawn_mav_crazyflie.launch
* Contributors: Stefano Grossi, Lorenzo Sabattini, Federico Pratissoli, Beatrice Capelli, Giuseppe Silano

7.0.1 (2021-12-26)
------------------
* Initial release of CrazyS for ROS Noetic. Add CMake 3.0 as required. Add version to the project name.
* Contributors: Daniele Morra, Giuseppe Silano
