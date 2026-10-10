/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef INCLUDE_ROTORS_CONTROL_COMMON_H_
#define INCLUDE_ROTORS_CONTROL_COMMON_H_

// Data types shared by the controllers. The control library does not depend on ROS:
// the nodes convert the ROS messages into these structures.

#include <algorithm>
#include <cmath>

#include <Eigen/Eigen>

namespace rotors_control {

// Drone state given by the odometry sensor.
struct EigenOdometry {
  EigenOdometry()
      : timeStampSec(0.0),
        timeStampNsec(0.0),
        position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3d::Zero()),
        angular_velocity(Eigen::Vector3d::Zero()) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timeStampSec;
  double timeStampNsec;
  Eigen::Vector3d position;          // world frame [m]
  Eigen::Quaterniond orientation;    // body to world
  Eigen::Vector3d velocity;          // body frame [m/s]
  Eigen::Vector3d angular_velocity;  // body frame [rad/s]
};

// Waypoint used by the position controller (from trajectory_msgs/MultiDOFJointTrajectory).
struct EigenTrajectoryPoint {
  EigenTrajectoryPoint()
      : position_W(Eigen::Vector3d::Zero()),
        orientation_W_B(Eigen::Quaterniond::Identity()) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position_W;
  Eigen::Quaterniond orientation_W_B;

  double getYaw() const {
    const Eigen::Quaterniond& q = orientation_W_B;
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }
};

// Full reference state (from rotors_comm/DroneState).
struct EigenDroneState {
  EigenDroneState()
      : position_W(Eigen::Vector3f::Zero()),
        orientation_W_B(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3f::Zero()),
        acceleration(Eigen::Vector3f::Zero()),
        angular_velocity_B(Eigen::Vector3f::Zero()),
        angular_acceleration_B(Eigen::Vector3f::Zero()) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f position_W;
  Eigen::Quaterniond orientation_W_B;
  Eigen::Vector3f velocity;
  Eigen::Vector3f acceleration;
  Eigen::Vector3f angular_velocity_B;
  Eigen::Vector3f angular_acceleration_B;
};

// Attitude and thrust command (from rotors_comm/RollPitchYawrateThrustCrazyflie).
struct EigenRollPitchYawrateThrustCrazyflie {
  EigenRollPitchYawrateThrustCrazyflie()
      : roll(0.0), pitch(0.0), yaw_rate(0.0), thrust(0.0) {}

  double roll;      // [rad]
  double pitch;     // [rad]
  double yaw_rate;  // [rad/s]
  double thrust;    // [PWM]
};

// Roll, pitch and yaw (z-y'-x'' convention) of a quaternion. Same result as
// tf::Matrix3x3::getRPY(), used by the ROS 1 version of the controllers.
inline void QuaternionToRPY(double x, double y, double z, double w,
                            double* roll, double* pitch, double* yaw) {
  const Eigen::Matrix3d m = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
  if (std::fabs(m(2, 0)) >= 1.0) {  // gimbal lock
    const double delta = std::atan2(m(0, 0), m(0, 2));
    *yaw = 0.0;
    if (m(2, 0) > 0.0) {
      *pitch = M_PI / 2.0;
      *roll = *pitch + delta;
    } else {
      *pitch = -M_PI / 2.0;
      *roll = -*pitch + delta;
    }
    return;
  }
  *pitch = -std::asin(m(2, 0));
  *roll = std::atan2(m(2, 1) / std::cos(*pitch), m(2, 2) / std::cos(*pitch));
  *yaw = std::atan2(m(1, 0) / std::cos(*pitch), m(0, 0) / std::cos(*pitch));
}

inline double Saturate(double value, double min_value, double max_value) {
  return std::min(std::max(value, min_value), max_value);
}

}  // namespace rotors_control

#endif  // INCLUDE_ROTORS_CONTROL_COMMON_H_
