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

// Helpers shared by the reference generators.

#ifndef ROTORS_GAZEBO_TRAJECTORY_UTILS_H
#define ROTORS_GAZEBO_TRAJECTORY_UTILS_H

#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

namespace rotors_gazebo {

// Topic names, relative to the node namespace.
static const char kCommandTrajectoryTopic[] = "command/trajectory";
static const char kDroneStateTopic[] = "drone_state";
static const char kOdometryTopic[] = "odometry";

// One-point trajectory with the given position and yaw, as built by
// mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw() in ROS 1.
inline trajectory_msgs::msg::MultiDOFJointTrajectory TrajectoryFromPositionYaw(
    double x, double y, double z, double yaw, const rclcpp::Time& stamp) {
  trajectory_msgs::msg::MultiDOFJointTrajectory msg;
  msg.header.stamp = stamp;
  msg.joint_names.push_back("base_link");
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
  geometry_msgs::msg::Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.translation.z = z;
  transform.rotation.z = std::sin(yaw / 2.0);
  transform.rotation.w = std::cos(yaw / 2.0);
  point.transforms.push_back(transform);
  point.velocities.resize(1);
  point.accelerations.resize(1);
  msg.points.push_back(point);
  return msg;
}

// Waits until the simulation clock runs (i.e. /clock is received and Gazebo is not
// paused) and the simulation time is at least start_time seconds.
inline bool WaitForSimTime(const rclcpp::Node::SharedPtr& node, double start_time) {
  while (rclcpp::ok() && node->now().seconds() < start_time) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return rclcpp::ok();
}

// Waits until somebody subscribes to the given publisher.
template <typename PublisherT>
inline bool WaitForSubscriber(const rclcpp::Node::SharedPtr& node, const PublisherT& publisher) {
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  while (rclcpp::ok() && publisher->get_subscription_count() == 0) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), steady_clock, 2000,
                         "Waiting for a subscriber on %s.", publisher->get_topic_name());
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return rclcpp::ok();
}

}  // namespace rotors_gazebo

#endif  // ROTORS_GAZEBO_TRAJECTORY_UTILS_H
