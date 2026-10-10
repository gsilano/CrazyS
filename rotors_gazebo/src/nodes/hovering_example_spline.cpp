/*
 * Copyright 2020 Ria Sonecha, Massachusetts Institute of Technology in Cambridge, MA, USA
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
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

// Publishes on drone_state, at 100 Hz, a quintic spline between an initial and a
// final state (spline_trajectory.yaml). The spline starts at start_time [s] of
// simulation time; once it is over, the final point keeps being published.
// Used with the Internal Model Controller, also in the swarm example.

#include <algorithm>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rotors_comm/msg/drone_state.hpp>

#include "rotors_gazebo/spline_trajectory_generator.h"
#include "trajectory_utils.h"

namespace {

Eigen::Vector3f DeclareVector(const rclcpp::Node::SharedPtr& node, const std::string& name,
                              const std::string& x, const std::string& y, const std::string& z,
                              const Eigen::Vector3f& default_value) {
  return Eigen::Vector3f(
      node->declare_parameter(name + "." + x, static_cast<double>(default_value.x())),
      node->declare_parameter(name + "." + y, static_cast<double>(default_value.y())),
      node->declare_parameter(name + "." + z, static_cast<double>(default_value.z())));
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hovering_example_spline");

  rotors_gazebo::SplineTrajectoryParameters p;
  const double start_time = node->declare_parameter("start_time", 3.0);
  const double sampling_time = node->declare_parameter("sampling_time", 0.01);
  p.time_final = node->declare_parameter("time_final.time", p.time_final);
  p.position_initial = DeclareVector(node, "position_initial", "x", "y", "z", p.position_initial);
  p.position_final = DeclareVector(node, "position_final", "x", "y", "z", p.position_final);
  p.velocity_initial = DeclareVector(node, "velocity_initial", "x", "y", "z", p.velocity_initial);
  p.velocity_final = DeclareVector(node, "velocity_final", "x", "y", "z", p.velocity_final);
  p.acceleration_initial =
      DeclareVector(node, "acceleration_initial", "x", "y", "z", p.acceleration_initial);
  p.acceleration_final =
      DeclareVector(node, "acceleration_final", "x", "y", "z", p.acceleration_final);
  p.orientation_initial =
      DeclareVector(node, "orientation_initial", "roll", "pitch", "yaw", p.orientation_initial);
  p.orientation_final =
      DeclareVector(node, "orientation_final", "roll", "pitch", "yaw", p.orientation_final);
  p.angular_velocity_initial = DeclareVector(node, "angularRate_initial", "roll", "pitch", "yaw",
                                             p.angular_velocity_initial);
  p.angular_velocity_final = DeclareVector(node, "angularRate_final", "roll", "pitch", "yaw",
                                           p.angular_velocity_final);
  p.angular_acceleration_initial = DeclareVector(node, "angularDoubleRate_initial", "roll",
                                                 "pitch", "yaw", p.angular_acceleration_initial);
  p.angular_acceleration_final = DeclareVector(node, "angularDoubleRate_final", "roll", "pitch",
                                               "yaw", p.angular_acceleration_final);

  rotors_gazebo::SplineTrajectoryGenerator generator;
  generator.SetParameters(p);

  auto trajectory_pub =
      node->create_publisher<rotors_comm::msg::DroneState>(rotors_gazebo::kDroneStateTopic, 10);
  RCLCPP_INFO(node->get_logger(), "Started hovering example with spline.");

  if (!rotors_gazebo::WaitForSimTime(node, start_time)) {
    return 0;
  }

  rclcpp::Rate rate(1.0 / sampling_time, node->get_clock());
  rotors_gazebo::SplineReference reference;
  while (rclcpp::ok()) {
    // After time_final the spline is no longer valid: hold the final point.
    double initial_time = 0.0;
    double elapsed = std::min(node->now().seconds() - start_time, p.time_final);
    generator.TrajectoryCallback(&reference, &elapsed, &initial_time);

    rotors_comm::msg::DroneState msg;
    msg.header.stamp = node->now();
    msg.position.x = reference.position_W.x();
    msg.position.y = reference.position_W.y();
    msg.position.z = reference.position_W.z();
    msg.linear_velocity.x = reference.velocity.x();
    msg.linear_velocity.y = reference.velocity.y();
    msg.linear_velocity.z = reference.velocity.z();
    msg.linear_acceleration.x = reference.acceleration.x();
    msg.linear_acceleration.y = reference.acceleration.y();
    msg.linear_acceleration.z = reference.acceleration.z();
    msg.orientation.x = reference.orientation_W_B.x();
    msg.orientation.y = reference.orientation_W_B.y();
    msg.orientation.z = reference.orientation_W_B.z();
    msg.orientation.w = reference.orientation_W_B.w();
    msg.angular_velocity.x = reference.angular_velocity_B.x();
    msg.angular_velocity.y = reference.angular_velocity_B.y();
    msg.angular_velocity.z = reference.angular_velocity_B.z();
    trajectory_pub->publish(msg);

    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
