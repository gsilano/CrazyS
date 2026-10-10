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

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Sends the waypoints of a file on command/trajectory, one at a time.
//   ros2 run rotors_gazebo waypoint_publisher_file <waypoint_file>
// Each line of the file is: wait_time [s] x [m] y [m] z [m] yaw [deg]. The first
// waypoint is sent once the simulation runs (plus start_delay seconds, default 5),
// each following one after the wait time of the previous one.
//
// In ROS 1 the whole list was sent in a single message, but the CrazyS position
// controller only uses the first point of a trajectory.

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_utils.h"

namespace {

struct WaypointWithTime {
  double waiting_time;
  double x, y, z, yaw;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waypoint_publisher_file");
  const double start_delay = node->declare_parameter("start_delay", 5.0);

  if (args.size() != 2) {
    RCLCPP_ERROR(node->get_logger(),
                 "Usage: waypoint_publisher_file <waypoint_file>\nThe waypoint file should "
                 "be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg]");
    return -1;
  }

  std::vector<WaypointWithTime> waypoints;
  std::ifstream wp_file(args.at(1));
  if (!wp_file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Unable to open the waypoint file %s", args.at(1).c_str());
    return -1;
  }
  WaypointWithTime wp;
  while (wp_file >> wp.waiting_time >> wp.x >> wp.y >> wp.z >> wp.yaw) {
    wp.yaw *= M_PI / 180.0;
    waypoints.push_back(wp);
  }
  RCLCPP_INFO(node->get_logger(), "Read %zu waypoints.", waypoints.size());

  auto wp_pub = node->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      rotors_gazebo::kCommandTrajectoryTopic, 10);
  if (!rotors_gazebo::WaitForSimTime(node, start_delay) ||
      !rotors_gazebo::WaitForSubscriber(node, wp_pub)) {
    return 0;
  }

  for (const WaypointWithTime& w : waypoints) {
    RCLCPP_INFO(node->get_logger(), "Waypoint [%.2f, %.2f, %.2f], yaw %.1f deg.", w.x, w.y, w.z,
                w.yaw * 180.0 / M_PI);
    wp_pub->publish(rotors_gazebo::TrajectoryFromPositionYaw(w.x, w.y, w.z, w.yaw, node->now()));
    if (!rotors_gazebo::WaitForSimTime(node, node->now().seconds() + w.waiting_time)) {
      break;
    }
  }
  rclcpp::shutdown();
  return 0;
}
