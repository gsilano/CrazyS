/*
 * Copyright 2021 Stefano Grossi, University of Modena and Reggio Emilia, Italy
 * Copyright 2021 Lorenzo Sabattini, University of Modena and Reggio Emilia, Italy
 * Copyright 2021 Giuseppe Silano, Czech Technical University in Prague, Czech Republic
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
 * "drone_state" topic. The trajectory is a list of segments loaded from the
 * parameter server, e.g.:
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
 * "initial_position" parameter as fallback). When the last segment is over, the
 * final point keeps being published.
 *
 * The files in resource/trajectories reproduce the tests of S. Grossi's thesis.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

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

double ToDouble(XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  return static_cast<double>(value);
}

bool ReadVector(XmlRpc::XmlRpcValue& segment, const std::string& key, int size,
                Eigen::VectorXd* vector) {
  if (!segment.hasMember(key) || segment[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
      segment[key].size() != size) {
    ROS_ERROR_STREAM("[Trajectory generator] The '" << key << "' field must be a list of "
                     << size << " numbers.");
    return false;
  }
  vector->resize(size);
  for (int i = 0; i < size; ++i) {
    (*vector)(i) = ToDouble(segment[key][i]);
  }
  return true;
}

bool ReadSegments(const ros::NodeHandle& nh, std::vector<Segment>* segments) {
  XmlRpc::XmlRpcValue list;
  if (!nh.getParam("segments", list) || list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("[Trajectory generator] The 'segments' parameter is missing or is not a list.");
    return false;
  }

  for (int i = 0; i < list.size(); ++i) {
    XmlRpc::XmlRpcValue& item = list[i];
    Segment segment;
    segment.goal.setZero();
    segment.center.setZero();
    segment.angle = 0.0;

    if (!item.hasMember("type") || !item.hasMember("duration")) {
      ROS_ERROR("[Trajectory generator] Segment %d needs both 'type' and 'duration'.", i);
      return false;
    }
    const std::string type = item["type"];
    segment.duration = ToDouble(item["duration"]);
    if (segment.duration <= 0.0) {
      ROS_ERROR("[Trajectory generator] Segment %d has a non positive duration.", i);
      return false;
    }

    segment.profile = TimingLaw::kLinear;
    if (item.hasMember("profile")) {
      const std::string profile = item["profile"];
      if (profile == "cubic") {
        segment.profile = TimingLaw::kCubic;
      } else if (profile != "linear") {
        ROS_ERROR("[Trajectory generator] Unknown profile '%s' (segment %d).",
                  profile.c_str(), i);
        return false;
      }
    }

    Eigen::VectorXd vector;
    if (type == "line") {
      segment.type = SegmentType::kLine;
      if (!ReadVector(item, "goal", 3, &vector)) return false;
      segment.goal = vector;
    } else if (type == "arc") {
      segment.type = SegmentType::kArc;
      if (!ReadVector(item, "center", 2, &vector)) return false;
      segment.center = vector;
      if (!item.hasMember("angle")) {
        ROS_ERROR("[Trajectory generator] The arc segment %d needs the 'angle' field.", i);
        return false;
      }
      segment.angle = ToDouble(item["angle"]);
    } else if (type == "hold") {
      segment.type = SegmentType::kHold;
    } else {
      ROS_ERROR("[Trajectory generator] Unknown segment type '%s' (segment %d).",
                type.c_str(), i);
      return false;
    }
    segments->push_back(segment);
  }
  return !segments->empty();
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

bool UnpauseGazebo() {
  std_srvs::Empty srv;
  for (int i = 0; i <= 10; ++i) {
    if (ros::service::call("/gazebo/unpause_physics", srv)) {
      ROS_INFO("[Trajectory generator] Unpaused the Gazebo simulation.");
      return true;
    }
    ROS_INFO("[Trajectory generator] Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "piecewise_trajectory_generator");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Publisher trajectory_pub =
      nh.advertise<mav_msgs::DroneState>(mav_msgs::default_topics::DRONE_STATE, 10);

  double sampling_time, start_time, yaw;
  private_nh.param("sampling_time", sampling_time, 0.01);
  private_nh.param("start_time", start_time, 3.0);
  private_nh.param("yaw", yaw, 0.0);

  std::vector<Segment> segments;
  if (!ReadSegments(private_nh, &segments)) {
    ROS_FATAL("[Trajectory generator] Invalid trajectory description.");
    return -1;
  }

  if (!UnpauseGazebo()) {
    ROS_FATAL("[Trajectory generator] Could not wake up Gazebo.");
    return -1;
  }

  // Start from where the drone is.
  Eigen::Vector3d initial_position(0.0, 0.0, 0.0);
  std::vector<double> initial_position_param;
  nav_msgs::OdometryConstPtr odometry = ros::topic::waitForMessage<nav_msgs::Odometry>(
      mav_msgs::default_topics::ODOMETRY, nh, ros::Duration(5.0));
  if (odometry) {
    initial_position << odometry->pose.pose.position.x, odometry->pose.pose.position.y,
                        odometry->pose.pose.position.z;
  } else if (private_nh.getParam("initial_position", initial_position_param) &&
             initial_position_param.size() == 3) {
    initial_position << initial_position_param[0], initial_position_param[1],
                        initial_position_param[2];
  } else {
    ROS_WARN("[Trajectory generator] No odometry received, starting from the origin.");
  }
  ComputeSegmentStartPoints(initial_position, &segments);

  const double total_duration = segments.back().start_time + segments.back().duration;
  ROS_INFO("[Trajectory generator] Trajectory with %zu segments (%.2f s) starting from "
           "[%.3f, %.3f, %.3f] at t = %.2f s.", segments.size(), total_duration,
           initial_position.x(), initial_position.y(), initial_position.z(), start_time);

  // Give Gazebo and the controller some time to start.
  while (ros::ok() && ros::Time::now().toSec() < start_time) {
    ros::Duration(sampling_time).sleep();
  }
  const ros::Time trajectory_start = ros::Time::now();

  const Eigen::Quaterniond orientation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  mav_msgs::DroneState msg;
  msg.orientation.w = orientation.w();
  msg.orientation.x = orientation.x();
  msg.orientation.y = orientation.y();
  msg.orientation.z = orientation.z();

  bool completed = false;
  ros::Rate rate(1.0 / sampling_time);
  while (ros::ok()) {
    const double t = (ros::Time::now() - trajectory_start).toSec();
    const ReferencePoint point = EvaluateTrajectory(segments, t);

    msg.header.stamp = ros::Time::now();
    msg.position.x = point.position.x();
    msg.position.y = point.position.y();
    msg.position.z = point.position.z();
    msg.linear_velocity.x = point.velocity.x();
    msg.linear_velocity.y = point.velocity.y();
    msg.linear_velocity.z = point.velocity.z();
    msg.linear_acceleration.x = point.acceleration.x();
    msg.linear_acceleration.y = point.acceleration.y();
    msg.linear_acceleration.z = point.acceleration.z();
    trajectory_pub.publish(msg);

    if (!completed && t >= total_duration) {
      completed = true;
      ROS_INFO("[Trajectory generator] Trajectory completed, holding [%.3f, %.3f, %.3f].",
               point.position.x(), point.position.y(), point.position.z());
    }

    rate.sleep();
  }

  return 0;
}
