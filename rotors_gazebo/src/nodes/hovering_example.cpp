/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

// Publishes a single waypoint (by default 1 m above the origin) on command/trajectory
// 5 s after the simulation starts. Parameters: x, y, z [m], yaw [rad], delay [s].

#include <memory>

#include "trajectory_utils.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hovering_example");
  const double x = node->declare_parameter("x", 0.0);
  const double y = node->declare_parameter("y", 0.0);
  const double z = node->declare_parameter("z", 1.0);
  const double yaw = node->declare_parameter("yaw", 0.0);
  const double delay = node->declare_parameter("delay", 5.0);

  auto trajectory_pub = node->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      rotors_gazebo::kCommandTrajectoryTopic, 10);
  RCLCPP_INFO(node->get_logger(), "Started hovering example.");

  // Let the Gazebo GUI show up and the controller start.
  if (!rotors_gazebo::WaitForSimTime(node, delay) ||
      !rotors_gazebo::WaitForSubscriber(node, trajectory_pub)) {
    return 0;
  }

  RCLCPP_INFO(node->get_logger(), "Publishing waypoint on namespace %s: [%f, %f, %f].",
              node->get_namespace(), x, y, z);
  trajectory_pub->publish(rotors_gazebo::TrajectoryFromPositionYaw(x, y, z, yaw, node->now()));

  // Give the message some time to leave before shutting down.
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  rclcpp::shutdown();
  return 0;
}
