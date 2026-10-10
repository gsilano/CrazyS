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

// Sends one waypoint on command/trajectory.
//   ros2 run rotors_gazebo waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]

#include <memory>
#include <string>
#include <vector>

#include "trajectory_utils.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waypoint_publisher");

  double delay;
  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stod(args.at(5));
  } else {
    RCLCPP_ERROR(node->get_logger(), "Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]");
    return -1;
  }
  const double x = std::stod(args.at(1));
  const double y = std::stod(args.at(2));
  const double z = std::stod(args.at(3));
  const double yaw = std::stod(args.at(4)) * M_PI / 180.0;

  auto trajectory_pub = node->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      rotors_gazebo::kCommandTrajectoryTopic, 10);

  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delay)));
  if (!rotors_gazebo::WaitForSubscriber(node, trajectory_pub)) {
    return 0;
  }

  RCLCPP_INFO(node->get_logger(), "Publishing waypoint on namespace %s: [%f, %f, %f].",
              node->get_namespace(), x, y, z);
  trajectory_pub->publish(rotors_gazebo::TrajectoryFromPositionYaw(x, y, z, yaw, node->now()));
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  rclcpp::shutdown();
  return 0;
}
