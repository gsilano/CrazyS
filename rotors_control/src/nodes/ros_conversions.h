/*
 * Copyright 2026 Giuseppe Silano, Czech Technical University in Prague, Czech Republic
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Conversions between ROS 2 messages and the data types of the control library,
// shared by the controller nodes.

#ifndef ROTORS_CONTROL_ROS_CONVERSIONS_H
#define ROTORS_CONTROL_ROS_CONVERSIONS_H

#include <string>

#include <actuator_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rotors_comm/msg/drone_state.hpp>
#include <rotors_comm/msg/roll_pitch_yawrate_thrust_crazyflie.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include "rotors_control/common.h"

namespace rotors_control {

// Topic names, relative to the node namespace (same names as the ROS 1 version).
static const char kOdometryTopic[] = "odometry";
static const char kImuTopic[] = "imu";
static const char kCommandMotorSpeedTopic[] = "command/motor_speed";
static const char kCommandTrajectoryTopic[] = "command/trajectory";
static const char kDroneStateTopic[] = "drone_state";
static const char kCommandRollPitchYawrateThrustTopic[] = "command/roll_pitch_yawrate_thrust";

inline void EigenOdometryFromMsg(const nav_msgs::msg::Odometry& msg, EigenOdometry* odometry) {
  odometry->timeStampSec = msg.header.stamp.sec;
  odometry->timeStampNsec = msg.header.stamp.nanosec;
  odometry->position = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                       msg.pose.pose.position.z);
  odometry->orientation = Eigen::Quaterniond(msg.pose.pose.orientation.w,
                                             msg.pose.pose.orientation.x,
                                             msg.pose.pose.orientation.y,
                                             msg.pose.pose.orientation.z);
  odometry->velocity = Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                                       msg.twist.twist.linear.z);
  odometry->angular_velocity = Eigen::Vector3d(msg.twist.twist.angular.x,
                                               msg.twist.twist.angular.y,
                                               msg.twist.twist.angular.z);
}

inline void EigenDroneStateFromMsg(const rotors_comm::msg::DroneState& msg,
                                   EigenDroneState* state) {
  state->position_W = Eigen::Vector3f(msg.position.x, msg.position.y, msg.position.z);
  state->orientation_W_B = Eigen::Quaterniond(msg.orientation.w, msg.orientation.x,
                                              msg.orientation.y, msg.orientation.z);
  state->velocity = Eigen::Vector3f(msg.linear_velocity.x, msg.linear_velocity.y,
                                    msg.linear_velocity.z);
  state->acceleration = Eigen::Vector3f(msg.linear_acceleration.x, msg.linear_acceleration.y,
                                        msg.linear_acceleration.z);
  state->angular_velocity_B = Eigen::Vector3f(msg.angular_velocity.x, msg.angular_velocity.y,
                                              msg.angular_velocity.z);
  state->angular_acceleration_B = Eigen::Vector3f(msg.angular_acceleration.x,
                                                  msg.angular_acceleration.y,
                                                  msg.angular_acceleration.z);
}

// Only the first point of the trajectory is used, as in the ROS 1 version.
inline void EigenTrajectoryPointFromMsg(
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& msg, EigenTrajectoryPoint* point) {
  if (msg.transforms.empty()) {
    return;
  }
  const auto& t = msg.transforms.front();
  point->position_W = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  point->orientation_W_B = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y,
                                              t.rotation.z);
}

inline actuator_msgs::msg::Actuators ActuatorsMsg(const Eigen::Vector4d& rotor_velocities,
                                                  const rclcpp::Time& stamp) {
  actuator_msgs::msg::Actuators msg;
  msg.header.stamp = stamp;
  for (int i = 0; i < rotor_velocities.size(); i++) {
    msg.velocity.push_back(rotor_velocities[i]);
  }
  return msg;
}

}  // namespace rotors_control

#endif  // ROTORS_CONTROL_ROS_CONVERSIONS_H
