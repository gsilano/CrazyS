^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.1.0 (2026-10-10)
------------------
* Add the piecewise_trajectory_generator node: DroneState references built from line, arc and hold segments in a YAML file
* Add the trajectories used in Stefano Grossi's thesis (resource/trajectories)
* Add crazyflie2_lee_geometric_controller.launch and crazyflie2_swarm_lee_geometric_controller.launch
* Swarm trajectories: drone 1 waits 8 s before leaving, so it does not cross drone 2
* Add the forster_parameters argument to spawn_mav_crazyflie.launch
* Contributors: Stefano Grossi, Lorenzo Sabattini, Federico Pratissoli, Beatrice Capelli, Giuseppe Silano

7.0.1 (2021-12-26)
------------------
* Initial release of CrazyS for ROS Noetic. Add CMake 3.0 as required. Add version to the project name.
* Contributors: Daniele Morra, Giuseppe Silano
