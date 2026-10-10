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

// Unit tests of the control library. They do not need ROS or Gazebo.

#include <cmath>

#include <gtest/gtest.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_geometric_controller.h"
#include "rotors_control/position_controller.h"

using rotors_control::EigenDroneState;
using rotors_control::EigenOdometry;
using rotors_control::LeeGeometricController;

TEST(Common, QuaternionToRPY) {
  const double roll = 0.1, pitch = -0.2, yaw = 0.3;
  const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  double r, p, y;
  rotors_control::QuaternionToRPY(q.x(), q.y(), q.z(), q.w(), &r, &p, &y);
  EXPECT_NEAR(r, roll, 1e-9);
  EXPECT_NEAR(p, pitch, 1e-9);
  EXPECT_NEAR(y, yaw, 1e-9);
}

TEST(DirtyDerivative, Ramp) {
  // The derivative of a ramp converges to its slope.
  rotors_control::DirtyDerivative derivative;
  derivative.Configure(1, 0.05, 0.01);
  Eigen::Vector3d value;
  for (int k = 0; k < 200; ++k) {
    value = derivative.Update(Eigen::Vector3d(2.0 * k * 0.01, -k * 0.01, 0.0));
  }
  EXPECT_NEAR(value.x(), 2.0, 1e-6);
  EXPECT_NEAR(value.y(), -1.0, 1e-6);
  EXPECT_NEAR(value.z(), 0.0, 1e-9);
}

TEST(LeeGeometricController, MotorsOffWithoutReference) {
  LeeGeometricController controller;
  controller.InitializeParameters();
  controller.SetOdometry(EigenOdometry(), 0.0);
  Eigen::Vector4d rotor_velocities;
  controller.CalculateRotorVelocities(&rotor_velocities);
  EXPECT_EQ(rotor_velocities, Eigen::Vector4d::Zero());
}

TEST(LeeGeometricController, HoverEquilibrium) {
  // Drone at the reference, at rest: the four propellers give exactly the weight.
  LeeGeometricController controller;
  controller.InitializeParameters();
  const auto& p = controller.controller_parameters_;
  const double hover_velocity = std::sqrt(p.mass * p.gravity / (4.0 * p.motor_constant));

  EigenOdometry odometry;
  odometry.position = Eigen::Vector3d(0.5, -0.5, 1.0);
  EigenDroneState reference;
  reference.position_W = odometry.position.cast<float>();

  Eigen::Vector4d rotor_velocities;
  for (int k = 0; k < 100; ++k) {
    const double time = k * p.sampling_time;
    controller.SetTrajectoryPoint(reference, time);
    controller.SetOdometry(odometry, time);
    controller.CalculateRotorVelocities(&rotor_velocities);
  }
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(rotor_velocities[i], hover_velocity, 1e-3);
  }
}

TEST(LeeGeometricController, AltitudeErrorIncreasesThrust) {
  LeeGeometricController controller;
  controller.InitializeParameters();
  const auto& p = controller.controller_parameters_;
  const double hover_velocity = std::sqrt(p.mass * p.gravity / (4.0 * p.motor_constant));

  EigenOdometry odometry;  // at the origin
  EigenDroneState reference;
  reference.position_W = Eigen::Vector3f(0.0f, 0.0f, 0.5f);
  controller.SetTrajectoryPoint(reference, 0.0);
  controller.SetOdometry(odometry, 0.0);
  Eigen::Vector4d rotor_velocities;
  controller.CalculateRotorVelocities(&rotor_velocities);
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(rotor_velocities[i], hover_velocity);
  }
}

TEST(PositionController, MotorsOffWithoutWaypoint) {
  rotors_control::PositionController controller;
  controller.SetControllerGains();
  controller.SetOdometryWithoutStateEstimator(EigenOdometry());
  Eigen::Vector4d rotor_velocities;
  controller.CalculateRotorVelocities(&rotor_velocities);
  EXPECT_EQ(rotor_velocities, Eigen::Vector4d::Zero());
}
