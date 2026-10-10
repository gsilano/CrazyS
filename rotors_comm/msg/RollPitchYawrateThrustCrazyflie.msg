# Attitude and thrust command for the Crazyflie, sent by the joystick interface.
# Port of mav_msgs/RollPitchYawrateThrustCrazyflie (gsilano/mav_comm).
#
# Rotation convention z-y'-x'': yaw around the fixed z axis, pitch around the new
# y axis, roll around the new x axis.

std_msgs/Header header

float64 roll      # [rad]
float64 pitch     # [rad]
float64 yaw_rate  # [rad/s]
float64 thrust    # [PWM]
