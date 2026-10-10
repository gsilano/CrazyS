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

/*
 * Lee's geometric controller on SE(3) for the Crazyflie 2.x.
 *
 * Reference: T. Lee, M. Leok, N. H. McClamroch, "Geometric tracking control of a
 * quadrotor UAV on SE(3)", IEEE CDC 2010, pp. 5420-5425.
 * Gains and tuning come from S. Grossi's MSc thesis (University of Modena and
 * Reggio Emilia, 2021).
 *
 * The paper uses a z-down inertial frame. Here everything is written in the frames
 * used by ROS and Gazebo (world ENU, body FLU), so the thrust points along +z_B,
 * gravity along -z_W, and the odometry and the reference are used as they are.
 *
 * The controller takes the desired position and heading (yaw) from a DroneState
 * message and returns the four propeller speeds [rad/s]. Velocity, acceleration,
 * jerk and snap of the reference, and the drone acceleration and jerk, are obtained
 * with dirty derivatives (s / (tau * s + 1), Tustin). The filters use the time stamps
 * of the samples, so delays in the delivery of the messages do not corrupt the
 * derivatives. CalculateRotorVelocities() is meant to be called every Ts seconds.
 */

#ifndef ROTORS_CONTROL_LEE_GEOMETRIC_CONTROLLER_H
#define ROTORS_CONTROL_LEE_GEOMETRIC_CONTROLLER_H

#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"

namespace rotors_control {

// Default values (S. Grossi's thesis). The node overwrites them with the parameters in
// rotors_gazebo/resource/crazyflie2_lee_geometric_controller.yaml.
static constexpr double kDefaultLeeSamplingTime = 0.01;              // [s]
static constexpr double kDefaultLeeGravity = 9.81;                   // [m/s^2]
static constexpr double kDefaultLeeMass = 0.027;                     // [kg]
static constexpr double kDefaultLeeInertiaXx = 1.657171e-05;         // [kg m^2]
static constexpr double kDefaultLeeInertiaYy = 1.657171e-05;         // [kg m^2]
static constexpr double kDefaultLeeInertiaZz = 2.9261652e-05;        // [kg m^2]
static constexpr double kDefaultLeeArmLength = 0.046;                // [m]
static constexpr double kDefaultLeeMotorConstant = 1.71465181e-08;   // [kg m]
static constexpr double kDefaultLeeMomentConstant = 0.004459273;     // [m]
static constexpr double kDefaultLeeMaxRotorVelocity = 3052;          // [rad/s]
static constexpr double kDefaultLeeTauFirstOrder = 0.05;             // [s]
static constexpr double kDefaultLeeTauHigherOrder = 0.10;            // [s]
static constexpr double kDefaultLeeGainPosition = 0.108;             // kx = 4.0 * mass
static constexpr double kDefaultLeeGainVelocity = 0.0972;            // kv = 3.6 * mass
static constexpr double kDefaultLeeGainAttitude = 0.005873333;       // kR = 8.81 / 1500
static constexpr double kDefaultLeeGainAngularRate = 0.001693333;    // kOmega = 2.54 / 1500

// Vehicle parameters and controller gains.
struct LeeGeometricControllerParameters {
  LeeGeometricControllerParameters()
      : sampling_time(kDefaultLeeSamplingTime),
        gravity(kDefaultLeeGravity),
        mass(kDefaultLeeMass),
        inertia(Eigen::Vector3d(kDefaultLeeInertiaXx, kDefaultLeeInertiaYy,
                                kDefaultLeeInertiaZz).asDiagonal()),
        arm_length(kDefaultLeeArmLength),
        motor_constant(kDefaultLeeMotorConstant),
        moment_constant(kDefaultLeeMomentConstant),
        max_rotor_velocity(kDefaultLeeMaxRotorVelocity),
        tau_first_order(kDefaultLeeTauFirstOrder),
        tau_higher_order(kDefaultLeeTauHigherOrder),
        k_x(kDefaultLeeGainPosition),
        k_v(kDefaultLeeGainVelocity),
        k_R(kDefaultLeeGainAttitude),
        k_omega(kDefaultLeeGainAngularRate) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double sampling_time;       // Controller sampling time Ts [s]
  double gravity;             // Gravitational acceleration [m/s^2]
  double mass;                // Drone mass [kg]
  Eigen::Matrix3d inertia;    // Drone inertia matrix J [kg m^2]
  double arm_length;          // Distance between the center of mass and each rotor axis [m]
  double motor_constant;      // Thrust coefficient C_T, thrust = C_T * omega^2 [kg m]
  double moment_constant;     // Drag torque to thrust ratio C_M, torque = C_M * thrust [m]
  double max_rotor_velocity;  // Maximum propeller angular velocity [rad/s]
  double tau_first_order;     // Time constant of the first order dirty derivatives [s]
  double tau_higher_order;    // Time constant of the higher order dirty derivatives [s]
  double k_x;                 // Position gain
  double k_v;                 // Velocity gain
  double k_R;                 // Attitude gain
  double k_omega;             // Angular velocity gain
};

// Dirty derivative of a 3D signal: s / (tau * s + 1) discretized with Tustin,
//   y(k) = a1 * y(k-1) + a2 * (u(k) - u(k-1))
//   a1 = (2 * tau - dt) / (2 * tau + dt),  a2 = 2 / (2 * tau + dt)
// where dt is the time between two samples (the nominal Ts if not given).
// The output stays at zero for the first "order" samples, so a chain of filters
// (velocity -> acceleration -> jerk -> snap) starts only when its input is valid.
class DirtyDerivative {
 public:
  DirtyDerivative();

  void Configure(unsigned int order, double tau, double sampling_time);
  void Reset();
  const Eigen::Vector3d& Update(const Eigen::Vector3d& input, double dt = -1.0);
  const Eigen::Vector3d& value() const { return derivative_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  unsigned int order_;
  unsigned long iteration_;
  double tau_;
  double sampling_time_;
  Eigen::Vector3d derivative_;
  Eigen::Vector3d previous_input_;
};

class LeeGeometricController {
 public:
  LeeGeometricController();
  ~LeeGeometricController();

  // Builds the allocation matrix and sets up the filters. Call it after changing
  // controller_parameters_.
  void InitializeParameters();

  // time: time stamp of the sample [s]
  void SetOdometry(const EigenOdometry& odometry, double time);
  void SetTrajectoryPoint(const mav_msgs::EigenDroneState& command_trajectory, double time);

  // One step of the control law, to be called every sampling_time seconds.
  void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

  LeeGeometricControllerParameters controller_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  bool controller_active_;
  bool initialized_params_;

  // [f; M] = allocation_matrix_ * [w0^2; w1^2; w2^2; w3^2]
  Eigen::Matrix4d allocation_matrix_;
  Eigen::Matrix4d allocation_matrix_inverse_;

  EigenOdometry odometry_;
  mav_msgs::EigenDroneState command_trajectory_;

  // Time stamps of the last samples and of the samples last used by the filters.
  double odometry_time_;
  double reference_time_;
  double last_control_time_;
  double last_reference_time_;
  bool new_reference_;

  // Derivatives of the reference position x_d, of the reference heading b1_d and of
  // the drone velocity v (world frame).
  DirtyDerivative position_reference_1dot_, position_reference_2dot_;
  DirtyDerivative position_reference_3dot_, position_reference_4dot_;
  DirtyDerivative heading_reference_1dot_, heading_reference_2dot_;
  DirtyDerivative velocity_1dot_, velocity_2dot_;

  void ComputeThrustAndMoments(double* thrust, Eigen::Vector3d* moments);
  void ThrustAndMomentsToRotorVelocities(double thrust, const Eigen::Vector3d& moments,
                                         Eigen::Vector4d* rotor_velocities) const;
};

}  // namespace rotors_control

#endif  // ROTORS_CONTROL_LEE_GEOMETRIC_CONTROLLER_H
