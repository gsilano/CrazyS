# Reference state of the drone, used by the Internal Model Controller and the Lee
# geometric controller. Port of mav_msgs/DroneState (gsilano/mav_comm).
#
# Frames: x forward, y left, z up.

std_msgs/Header header

geometry_msgs/Vector3 position              # [m]
geometry_msgs/Vector3 linear_velocity       # [m/s]
geometry_msgs/Vector3 linear_acceleration   # [m/s^2]

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity      # [rad/s]
geometry_msgs/Vector3 angular_acceleration  # [rad/s^2]
