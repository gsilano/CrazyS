/*
 * Copyright 2021 Stefano Grossi, University of Modena and Reggio Emilia, Italy
 * Copyright 2021 Lorenzo Sabattini, University of Modena and Reggio Emilia, Italy
 * Copyright 2021 Giuseppe Silano, Czech Technical University in Prague, Czech Republic
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

/*
 * Publishes a DroneState reference (position, velocity, acceleration, yaw) on the
 * "drone_state" topic. The trajectory is a list of segments read from the YAML file
 * given by the "trajectory_file" parameter, e.g.:
 *
 *   sampling_time: 0.01     # publishing period [s]
 *   start_time: 3.0         # sim time at which the trajectory starts [s]
 *   yaw: 0.0                # [rad]
 *   segments:
 *     - {type: line, goal: [0.0, 0.0, 1.0], duration: 2.0, profile: linear}
 *     - {type: hold, duration: 4.0}
 *     - {type: line, goal: [1.0, 0.0, 1.0], duration: 2.0, profile: cubic}
 *     - {type: arc, center: [0.0, 0.0], angle: 3.14159265, duration: 5.0, profile: cubic}
 *
 * line: straight line from the current point to "goal" [m]
 * arc:  horizontal arc around "center" [m] (x, y), at constant altitude, from the
 *       current point; "angle" [rad] > 0 means counter-clockwise
 * hold: stay at the current point for "duration" seconds
 *
 * profile "linear" moves at constant speed (velocity steps at the ends), "cubic"
 * uses a third order polynomial with zero velocity at both ends.
 *
 * The first segment starts where the drone is (first "odometry" message, or the
 * "initial_position" entry of the file as fallback). When the last segment is over, the
 * final point keeps being published.
 *
 * The files in resource/trajectories reproduce the tests of S. Grossi's thesis.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rotors_comm/msg/drone_state.hpp>
#include <yaml-cpp/yaml.h>

#include "trajectory_utils.h"

namespace {

enum class SegmentType { kLine, kArc, kHold };
enum class TimingLaw { kLinear, kCubic };

struct Segment {
  SegmentType type;
  TimingLaw profile;
  double duration;        // [s]
  Eigen::Vector3d goal;   // line only [m]
  Eigen::Vector2d center; // arc only [m]
  double angle;           // arc only [rad]

  // Filled in by ComputeSegmentStartPoints().
  Eigen::Vector3d start;
  double start_time;      // relative to the trajectory start [s]
};

// Reference point (position, velocity and acceleration in the inertial frame).
struct ReferencePoint {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
};

// Reads the segments of the trajectory. Errors are reported through *error.
bool ReadSegments(const YAML::Node& list, std::vector<Segment>* segments, std::string* error) {
  if (!list || !list.IsSequence()) {
    *error = "the 'segments' entry is missing or is not a list";
    return false;
  }
  for (std::size_t i = 0; i < list.size(); ++i) {
    const YAML::Node item = list[i];
    const std::string prefix = "segment " + std::to_string(i) + ": ";
    Segment segment;
    segment.goal.setZero();
    segment.center.setZero();
    segment.angle = 0.0;

    if (!item["type"] || !item["duration"]) {
      *error = prefix + "'type' and 'duration' are required";
      return false;
    }
    const std::string type = item["type"].as<std::string>();
    segment.duration = item["duration"].as<double>();
    if (segment.duration <= 0.0) {
      *error = prefix + "the duration must be positive";
      return false;
    }

    segment.profile = TimingLaw::kLinear;
    if (item["profile"]) {
      const std::string profile = item["profile"].as<std::string>();
      if (profile == "cubic") {
        segment.profile = TimingLaw::kCubic;
      } else if (profile != "linear") {
        *error = prefix + "unknown profile '" + profile + "'";
        return false;
      }
    }

    if (type == "line") {
      segment.type = SegmentType::kLine;
      if (!item["goal"] || !item["goal"].IsSequence() || item["goal"].size() != 3) {
        *error = prefix + "'goal' must be a list of 3 numbers";
        return false;
      }
      segment.goal = Eigen::Vector3d(item["goal"][0].as<double>(), item["goal"][1].as<double>(),
                                     item["goal"][2].as<double>());
    } else if (type == "arc") {
      segment.type = SegmentType::kArc;
      if (!item["center"] || !item["center"].IsSequence() || item["center"].size() != 2) {
        *error = prefix + "'center' must be a list of 2 numbers";
        return false;
      }
      if (!item["angle"]) {
        *error = prefix + "'angle' is required for an arc";
        return false;
      }
      segment.center = Eigen::Vector2d(item["center"][0].as<double>(),
                                       item["center"][1].as<double>());
      segment.angle = item["angle"].as<double>();
    } else if (type == "hold") {
      segment.type = SegmentType::kHold;
    } else {
      *error = prefix + "unknown segment type '" + type + "'";
      return false;
    }
    segments->push_back(segment);
  }
  if (segments->empty()) {
    *error = "the trajectory has no segments";
    return false;
  }
  return true;
}

// Normalized path coordinate s in [0, 1] and its first two time derivatives.
void EvaluateTimingLaw(TimingLaw profile, double t, double duration, double* s,
                       double* s_dot, double* s_ddot) {
  const double u = std::min(std::max(t / duration, 0.0), 1.0);
  if (profile == TimingLaw::kLinear) {
    *s = u;
    *s_dot = 1.0 / duration;
    *s_ddot = 0.0;
  } else {
    *s = 3.0 * u * u - 2.0 * u * u * u;
    *s_dot = (6.0 * u - 6.0 * u * u) / duration;
    *s_ddot = (6.0 - 12.0 * u) / (duration * duration);
  }
}

// Point reached at the end of a segment, given its starting point.
Eigen::Vector3d SegmentEndPoint(const Segment& segment) {
  switch (segment.type) {
    case SegmentType::kLine:
      return segment.goal;
    case SegmentType::kArc: {
      const Eigen::Vector2d radius_vector = segment.start.head<2>() - segment.center;
      const Eigen::Vector2d end = segment.center
          + Eigen::Rotation2Dd(segment.angle).toRotationMatrix() * radius_vector;
      return Eigen::Vector3d(end.x(), end.y(), segment.start.z());
    }
    default:
      return segment.start;
  }
}

void ComputeSegmentStartPoints(const Eigen::Vector3d& initial_position,
                               std::vector<Segment>* segments) {
  Eigen::Vector3d start = initial_position;
  double start_time = 0.0;
  for (Segment& segment : *segments) {
    segment.start = start;
    segment.start_time = start_time;
    start = SegmentEndPoint(segment);
    start_time += segment.duration;
  }
}

ReferencePoint EvaluateSegment(const Segment& segment, double t) {
  ReferencePoint point;
  point.velocity.setZero();
  point.acceleration.setZero();

  double s, s_dot, s_ddot;
  EvaluateTimingLaw(segment.profile, t, segment.duration, &s, &s_dot, &s_ddot);
  // Segment finished: no more velocity and acceleration.
  if (t >= segment.duration) {
    s_dot = 0.0;
    s_ddot = 0.0;
  }

  switch (segment.type) {
    case SegmentType::kLine: {
      const Eigen::Vector3d delta = segment.goal - segment.start;
      point.position = segment.start + s * delta;
      point.velocity = s_dot * delta;
      point.acceleration = s_ddot * delta;
      break;
    }
    case SegmentType::kArc: {
      const Eigen::Vector2d radius_vector = segment.start.head<2>() - segment.center;
      const double radius = radius_vector.norm();
      const double theta0 = std::atan2(radius_vector.y(), radius_vector.x());
      const double theta = theta0 + s * segment.angle;
      const double theta_dot = s_dot * segment.angle;
      const double theta_ddot = s_ddot * segment.angle;
      const Eigen::Vector2d radial(std::cos(theta), std::sin(theta));
      const Eigen::Vector2d tangential(-std::sin(theta), std::cos(theta));
      point.position << segment.center + radius * radial, segment.start.z();
      point.velocity << radius * theta_dot * tangential, 0.0;
      point.acceleration << radius * (theta_ddot * tangential - theta_dot * theta_dot * radial),
                            0.0;
      break;
    }
    case SegmentType::kHold:
      point.position = segment.start;
      break;
  }
  return point;
}

// Evaluates the whole trajectory at the time t (relative to its start).
ReferencePoint EvaluateTrajectory(const std::vector<Segment>& segments, double t) {
  for (const Segment& segment : segments) {
    if (t < segment.start_time + segment.duration) {
      return EvaluateSegment(segment, t - segment.start_time);
    }
  }
  return EvaluateSegment(segments.back(), segments.back().duration);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("piecewise_trajectory_generator");
  const std::string trajectory_file = node->declare_parameter("trajectory_file", std::string(""));

  YAML::Node config;
  try {
    config = YAML::LoadFile(trajectory_file);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "Cannot read the trajectory file '%s': %s",
                 trajectory_file.c_str(), e.what());
    return -1;
  }
  const double sampling_time = config["sampling_time"] ? config["sampling_time"].as<double>() : 0.01;
  const double start_time = config["start_time"] ? config["start_time"].as<double>() : 3.0;
  const double yaw = config["yaw"] ? config["yaw"].as<double>() : 0.0;

  std::vector<Segment> segments;
  std::string error;
  if (!ReadSegments(config["segments"], &segments, &error)) {
    RCLCPP_FATAL(node->get_logger(), "Invalid trajectory file '%s': %s", trajectory_file.c_str(),
                 error.c_str());
    return -1;
  }

  auto trajectory_pub =
      node->create_publisher<rotors_comm::msg::DroneState>(rotors_gazebo::kDroneStateTopic, 10);

  // Give Gazebo and the controller some time to start.
  if (!rotors_gazebo::WaitForSimTime(node, start_time)) {
    return 0;
  }

  // Start from where the drone is (read now, when it is resting on the ground).
  Eigen::Vector3d initial_position(0.0, 0.0, 0.0);
  nav_msgs::msg::Odometry odometry;
  if (rclcpp::wait_for_message(odometry, node, rotors_gazebo::kOdometryTopic,
                               std::chrono::seconds(10))) {
    initial_position << odometry.pose.pose.position.x, odometry.pose.pose.position.y,
                        odometry.pose.pose.position.z;
  } else if (config["initial_position"] && config["initial_position"].size() == 3) {
    initial_position << config["initial_position"][0].as<double>(),
                        config["initial_position"][1].as<double>(),
                        config["initial_position"][2].as<double>();
  } else {
    RCLCPP_WARN(node->get_logger(), "No odometry received, starting from the origin.");
  }
  ComputeSegmentStartPoints(initial_position, &segments);

  const double total_duration = segments.back().start_time + segments.back().duration;
  RCLCPP_INFO(node->get_logger(),
              "Trajectory with %zu segments (%.2f s) starting from [%.3f, %.3f, %.3f] at t = %.2f s.",
              segments.size(), total_duration, initial_position.x(), initial_position.y(),
              initial_position.z(), start_time);

  const rclcpp::Time trajectory_start = node->now();

  const Eigen::Quaterniond orientation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  rotors_comm::msg::DroneState msg;
  msg.orientation.w = orientation.w();
  msg.orientation.x = orientation.x();
  msg.orientation.y = orientation.y();
  msg.orientation.z = orientation.z();

  bool completed = false;
  rclcpp::Rate rate(1.0 / sampling_time, node->get_clock());
  while (rclcpp::ok()) {
    const double t = (node->now() - trajectory_start).seconds();
    const ReferencePoint point = EvaluateTrajectory(segments, t);

    msg.header.stamp = node->now();
    msg.position.x = point.position.x();
    msg.position.y = point.position.y();
    msg.position.z = point.position.z();
    msg.linear_velocity.x = point.velocity.x();
    msg.linear_velocity.y = point.velocity.y();
    msg.linear_velocity.z = point.velocity.z();
    msg.linear_acceleration.x = point.acceleration.x();
    msg.linear_acceleration.y = point.acceleration.y();
    msg.linear_acceleration.z = point.acceleration.z();
    trajectory_pub->publish(msg);

    if (!completed && t >= total_duration) {
      completed = true;
      RCLCPP_INFO(node->get_logger(), "Trajectory completed, holding [%.3f, %.3f, %.3f].",
                  point.position.x(), point.position.y(), point.position.z());
    }
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
