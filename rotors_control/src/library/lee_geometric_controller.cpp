/*
 * Copyright 2021 Stefano Grossi, University of Modena and Reggio Emilia, Italy
 * Copyright 2021 Lorenzo Sabattini, University of Modena and Reggio Emilia, Italy
 * Copyright 2021 Federico Pratissoli, University of Modena and Reggio Emilia, Italy
 * Copyright 2021 Beatrice Capelli, University of Modena and Reggio Emilia, Italy
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

#include "rotors_control/lee_geometric_controller.h"

#include <algorithm>
#include <cmath>

#include <ros/console.h>

namespace rotors_control {

namespace {

// Hat map R^3 -> so(3): Hat(a) * b = a x b.
Eigen::Matrix3d Hat(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew_matrix;
  skew_matrix <<           0, -vector.z(),  vector.y(),
                  vector.z(),           0, -vector.x(),
                 -vector.y(),  vector.x(),           0;
  return skew_matrix;
}

// Vee map so(3) -> R^3, inverse of the hat map.
Eigen::Vector3d Vee(const Eigen::Matrix3d& skew_matrix) {
  return Eigen::Vector3d(skew_matrix(2, 1), skew_matrix(0, 2), skew_matrix(1, 0));
}

// Builds a matrix whose columns are the three input vectors.
Eigen::Matrix3d FromColumns(const Eigen::Vector3d& c1, const Eigen::Vector3d& c2,
                            const Eigen::Vector3d& c3) {
  Eigen::Matrix3d matrix;
  matrix.col(0) = c1;
  matrix.col(1) = c2;
  matrix.col(2) = c3;
  return matrix;
}

}  // namespace

DirtyDerivative::DirtyDerivative()
    : order_(1),
      iteration_(0),
      a1_(0.0),
      a2_(0.0),
      derivative_(Eigen::Vector3d::Zero()),
      previous_input_(Eigen::Vector3d::Zero()) {}

void DirtyDerivative::Configure(unsigned int order, double tau, double sampling_time) {
  order_ = order;
  a1_ = (2.0 * tau - sampling_time) / (2.0 * tau + sampling_time);
  a2_ = 2.0 / (2.0 * tau + sampling_time);
  Reset();
}

void DirtyDerivative::Reset() {
  iteration_ = 0;
  derivative_.setZero();
  previous_input_.setZero();
}

const Eigen::Vector3d& DirtyDerivative::Update(const Eigen::Vector3d& input) {
  ++iteration_;
  if (iteration_ > order_) {
    derivative_ = a1_ * derivative_ + a2_ * (input - previous_input_);
  }
  previous_input_ = input;
  return derivative_;
}

LeeGeometricController::LeeGeometricController()
    : controller_active_(false),
      initialized_params_(false),
      allocation_matrix_(Eigen::Matrix4d::Zero()),
      allocation_matrix_inverse_(Eigen::Matrix4d::Zero()) {}

LeeGeometricController::~LeeGeometricController() {}

void LeeGeometricController::InitializeParameters() {
  const LeeGeometricControllerParameters& p = controller_parameters_;

  // Allocation matrix, from the rotor layout in crazyflie2.xacro (X frame, FLU):
  //   rotor 0: front-right (+l, -l), counter-clockwise
  //   rotor 1: back-right  (-l, -l), clockwise
  //   rotor 2: back-left   (-l, +l), counter-clockwise
  //   rotor 3: front-left  (+l, +l), clockwise
  // with l = arm_length / sqrt(2). In the RotorS motor model a ccw rotor gives a yaw
  // torque of -C_M * thrust, a cw rotor +C_M * thrust.
  const double l = p.arm_length / std::sqrt(2.0);
  const double c_t = p.motor_constant;
  const double c_m = p.moment_constant;
  allocation_matrix_ <<        c_t,       c_t,        c_t,       c_t,
                          -l * c_t,  -l * c_t,    l * c_t,   l * c_t,
                          -l * c_t,   l * c_t,    l * c_t,  -l * c_t,
                        -c_m * c_t, c_m * c_t, -c_m * c_t, c_m * c_t;
  allocation_matrix_inverse_ = allocation_matrix_.inverse();

  // Faster filter for the first derivatives, slower one for the higher orders (as in
  // the thesis).
  const double ts = p.sampling_time;
  position_reference_1dot_.Configure(1, p.tau_first_order, ts);
  position_reference_2dot_.Configure(2, p.tau_higher_order, ts);
  position_reference_3dot_.Configure(3, p.tau_higher_order, ts);
  position_reference_4dot_.Configure(4, p.tau_higher_order, ts);
  heading_reference_1dot_.Configure(1, p.tau_first_order, ts);
  heading_reference_2dot_.Configure(2, p.tau_higher_order, ts);
  velocity_1dot_.Configure(1, p.tau_first_order, ts);
  velocity_2dot_.Configure(2, p.tau_higher_order, ts);

  initialized_params_ = true;
}

void LeeGeometricController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void LeeGeometricController::SetTrajectoryPoint(
    const mav_msgs::EigenDroneState& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void LeeGeometricController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
  assert(rotor_velocities);

  // Motors off until the parameters are loaded and a reference has arrived.
  if (!initialized_params_ || !controller_active_) {
    rotor_velocities->setZero();
    return;
  }

  double thrust;
  Eigen::Vector3d moments;
  ComputeThrustAndMoments(&thrust, &moments);
  ThrustAndMomentsToRotorVelocities(thrust, moments, rotor_velocities);

  ROS_DEBUG("[Lee geometric controller] thrust: %f, moments: [%f, %f, %f]", thrust,
            moments.x(), moments.y(), moments.z());
}

void LeeGeometricController::ComputeThrustAndMoments(double* thrust,
                                                     Eigen::Vector3d* moments) {
  assert(thrust);
  assert(moments);

  const LeeGeometricControllerParameters& p = controller_parameters_;
  const Eigen::Vector3d e3(Eigen::Vector3d::UnitZ());

  // Reference. b1_d is the x-axis of the desired orientation projected on the
  // horizontal plane, i.e. the desired heading.
  const Eigen::Vector3d x_d = command_trajectory_.position_W.cast<double>();
  const Eigen::Matrix3d R_ref = command_trajectory_.orientation_W_B.normalized()
                                    .toRotationMatrix();
  Eigen::Vector3d b1_d(R_ref(0, 0), R_ref(1, 0), 0.0);
  if (b1_d.norm() < 1e-6) {
    b1_d = Eigen::Vector3d::UnitX();
  }
  b1_d.normalize();

  // Drone state. The odometry gives the linear velocity in body frame, we need it in
  // world frame.
  const Eigen::Vector3d x = odometry_.position;
  const Eigen::Matrix3d R = odometry_.orientation.normalized().toRotationMatrix();
  const Eigen::Vector3d v = R * odometry_.velocity;
  const Eigen::Vector3d omega = odometry_.angular_velocity;

  // Time derivatives (dirty derivatives).
  const Eigen::Vector3d x_d_1dot = position_reference_1dot_.Update(x_d);
  const Eigen::Vector3d x_d_2dot = position_reference_2dot_.Update(x_d_1dot);
  const Eigen::Vector3d x_d_3dot = position_reference_3dot_.Update(x_d_2dot);
  const Eigen::Vector3d x_d_4dot = position_reference_4dot_.Update(x_d_3dot);

  const Eigen::Vector3d b1_d_1dot = heading_reference_1dot_.Update(b1_d);
  const Eigen::Vector3d b1_d_2dot = heading_reference_2dot_.Update(b1_d_1dot);

  const Eigen::Vector3d v_1dot = velocity_1dot_.Update(v);
  const Eigen::Vector3d v_2dot = velocity_2dot_.Update(v_1dot);

  // Position, velocity, acceleration and jerk errors.
  const Eigen::Vector3d e_x = x - x_d;
  const Eigen::Vector3d e_v = v - x_d_1dot;
  const Eigen::Vector3d e_a = v_1dot - x_d_2dot;
  const Eigen::Vector3d e_j = v_2dot - x_d_3dot;

  // Desired force A and its first two derivatives. Same as eq. (45)-(46) of the
  // thesis, with the sign of gravity flipped because z points up.
  const Eigen::Vector3d A = -p.k_x * e_x - p.k_v * e_v + p.mass * p.gravity * e3
                            + p.mass * x_d_2dot;
  const Eigen::Vector3d A_1dot = -p.k_x * e_v - p.k_v * e_a + p.mass * x_d_3dot;
  const Eigen::Vector3d A_2dot = -p.k_x * e_a - p.k_v * e_j + p.mass * x_d_4dot;

  // Total thrust: A projected on the current body z-axis.
  *thrust = A.dot(R * e3);

  // Desired orientation R_c = [b1_c, b2_c, b3_c]:
  //   b3_c = A / |A|,  b2_c = (b3_c x b1_d) / |b3_c x b1_d|,  b1_c = b2_c x b3_c
  const double A_norm = A.norm();
  const Eigen::Vector3d b3_c = A / A_norm;
  const Eigen::Vector3d C = b3_c.cross(b1_d);
  const double C_norm = C.norm();
  const Eigen::Vector3d b2_c = C / C_norm;
  const Eigen::Vector3d b1_c = b2_c.cross(b3_c);

  // First derivatives of the desired body axes.
  const double A_dot_A1 = A.dot(A_1dot);
  const Eigen::Vector3d b3_c_1dot = A_1dot / A_norm
                                    - (A_dot_A1 / std::pow(A_norm, 3)) * A;
  const Eigen::Vector3d C_1dot = b3_c_1dot.cross(b1_d) + b3_c.cross(b1_d_1dot);
  const double C_dot_C1 = C.dot(C_1dot);
  const Eigen::Vector3d b2_c_1dot = C_1dot / C_norm
                                    - (C_dot_C1 / std::pow(C_norm, 3)) * C;
  const Eigen::Vector3d b1_c_1dot = b2_c_1dot.cross(b3_c) + b2_c.cross(b3_c_1dot);

  // Second derivatives of the desired body axes.
  const Eigen::Vector3d b3_c_2dot =
      A_2dot / A_norm
      - (2.0 * A_dot_A1 / std::pow(A_norm, 3)) * A_1dot
      - ((A_1dot.squaredNorm() + A.dot(A_2dot)) / std::pow(A_norm, 3)) * A
      + (3.0 * A_dot_A1 * A_dot_A1 / std::pow(A_norm, 5)) * A;
  const Eigen::Vector3d C_2dot = b3_c_2dot.cross(b1_d) + b3_c.cross(b1_d_2dot)
                                 + 2.0 * b3_c_1dot.cross(b1_d_1dot);
  const Eigen::Vector3d b2_c_2dot =
      C_2dot / C_norm
      - (2.0 * C_dot_C1 / std::pow(C_norm, 3)) * C_1dot
      - ((C_1dot.squaredNorm() + C.dot(C_2dot)) / std::pow(C_norm, 3)) * C
      + (3.0 * C_dot_C1 * C_dot_C1 / std::pow(C_norm, 5)) * C;
  const Eigen::Vector3d b1_c_2dot = b2_c_2dot.cross(b3_c) + b2_c.cross(b3_c_2dot)
                                    + 2.0 * b2_c_1dot.cross(b3_c_1dot);

  // Desired angular velocity and acceleration:
  //   R_c^T * dR_c/dt = hat(Omega_c)
  //   R_c^T * d2R_c/dt2 = hat(dOmega_c/dt) + hat(Omega_c)^2
  const Eigen::Matrix3d R_c = FromColumns(b1_c, b2_c, b3_c);
  const Eigen::Matrix3d R_c_1dot = FromColumns(b1_c_1dot, b2_c_1dot, b3_c_1dot);
  const Eigen::Matrix3d R_c_2dot = FromColumns(b1_c_2dot, b2_c_2dot, b3_c_2dot);
  const Eigen::Vector3d omega_c = Vee(R_c.transpose() * R_c_1dot);
  const Eigen::Vector3d omega_c_1dot =
      Vee(R_c.transpose() * R_c_2dot - Hat(omega_c) * Hat(omega_c));

  // Attitude and angular velocity errors, eq. (42) and (44) of the thesis.
  const Eigen::Vector3d e_R = 0.5 * Vee(R_c.transpose() * R - R.transpose() * R_c);
  const Eigen::Vector3d e_omega = omega - R.transpose() * R_c * omega_c;

  // Moments, eq. (47) of the thesis.
  const Eigen::Matrix3d& J = p.inertia;
  *moments = -p.k_R * e_R - p.k_omega * e_omega + omega.cross(J * omega)
             - J * (Hat(omega) * R.transpose() * R_c * omega_c
                    - R.transpose() * R_c * omega_c_1dot);
}

void LeeGeometricController::ThrustAndMomentsToRotorVelocities(
    double thrust, const Eigen::Vector3d& moments,
    Eigen::Vector4d* rotor_velocities) const {
  assert(rotor_velocities);

  const Eigen::Vector4d wrench(thrust, moments.x(), moments.y(), moments.z());
  Eigen::Vector4d squared_velocities = allocation_matrix_inverse_ * wrench;

  // If a motor goes below zero or above its limit, shift all four by the same amount.
  // The moments (attitude) are kept and the total thrust pays for it.
  const double max_squared_velocity = controller_parameters_.max_rotor_velocity
                                      * controller_parameters_.max_rotor_velocity;
  const double min_value = squared_velocities.minCoeff();
  const double max_value = squared_velocities.maxCoeff();
  if (min_value < 0.0) {
    squared_velocities.array() -= min_value;
  } else if (max_value > max_squared_velocity) {
    squared_velocities.array() -= max_value - max_squared_velocity;
  }

  // Clamp whatever is still out of range.
  for (int i = 0; i < squared_velocities.size(); ++i) {
    const double saturated = std::min(std::max(squared_velocities[i], 0.0),
                                      max_squared_velocity);
    (*rotor_velocities)[i] = std::sqrt(saturated);
  }
}

}  // namespace rotors_control
