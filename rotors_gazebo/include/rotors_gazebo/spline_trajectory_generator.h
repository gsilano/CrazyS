/*
 * Copyright 2020 Ria Sonecha, MIT, USA
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

// Quintic spline between an initial and a final state (position, velocity,
// acceleration, orientation and angular rates), used by hovering_example_spline.
// The class does not depend on ROS.

#ifndef ROTORS_GAZEBO_SPLINE_TRAJECTORY_GENERATOR_H
#define ROTORS_GAZEBO_SPLINE_TRAJECTORY_GENERATOR_H

#include <Eigen/Eigen>

namespace rotors_gazebo {

// Boundary conditions of the spline. Orientations are roll, pitch, yaw [rad].
struct SplineTrajectoryParameters {
  double time_final = 10.0;  // duration of the spline [s]
  Eigen::Vector3f position_initial = Eigen::Vector3f::Zero();
  Eigen::Vector3f position_final = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  Eigen::Vector3f velocity_initial = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_final = Eigen::Vector3f::Zero();
  Eigen::Vector3f acceleration_initial = Eigen::Vector3f::Zero();
  Eigen::Vector3f acceleration_final = Eigen::Vector3f::Zero();
  Eigen::Vector3f orientation_initial = Eigen::Vector3f::Zero();
  Eigen::Vector3f orientation_final = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity_initial = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity_final = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_acceleration_initial = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_acceleration_final = Eigen::Vector3f::Zero();
};

// Point of the spline.
struct SplineReference {
  Eigen::Vector3f position_W = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();
  Eigen::Quaterniond orientation_W_B = Eigen::Quaterniond::Identity();
  Eigen::Vector3f angular_velocity_B = Eigen::Vector3f::Zero();
};

class SplineTrajectoryGenerator {
 public:
  SplineTrajectoryGenerator();
  ~SplineTrajectoryGenerator();

  void SetParameters(const SplineTrajectoryParameters& parameters);

  // Spline evaluated at (*time_final - *time_init) seconds from its start.
  void TrajectoryCallback(SplineReference* odometry, double* time_final, double* time_init);

 private:
  bool enable_parameter_computation_ = true;

  // Polynomial coefficients
  Eigen::Vector3f a0_, a1_, a2_, a3_, a4_, a5_;
  Eigen::Vector3f b0_, b1_, b2_, b3_, b4_;
  Eigen::Vector3f c0_, c1_, c2_, c3_;
  Eigen::Vector3f g0_, g1_, g2_, g3_, g4_, g5_;
  Eigen::Vector3f h0_, h1_, h2_, h3_, h4_;
  Eigen::Vector3f i0_, i1_, i2_, i3_;

  // Desired orientation [rad]
  double rollDesRad_, pitchDesRad_, yawDesRad_;

  // Boundary conditions
  double time_final_;
  Eigen::Vector3f position_initial_, position_final_, velocity_initial_, velocity_final_,
      acceleration_initial_, acceleration_final_;
  Eigen::Vector3f orientation_initial_, orientation_final_, angular_velocity_initial_,
      angular_velocity_final_, angular_acceleration_initial_, angular_acceleration_final_;

  void ComputeSplineParameters(double* time_spline);
  void Euler2QuaternionCommandTrajectory(double* x, double* y, double* z, double* w) const;
};

}  // namespace rotors_gazebo

#endif  // ROTORS_GAZEBO_SPLINE_TRAJECTORY_GENERATOR_H
