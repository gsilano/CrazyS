/*
 * Copyright 2020 Giuseppe, University of Sannio in Benevento, Italy
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

#include "rotors_control/roll_pitch_yawrate_thrust_crazyflie.h"

#include "rotors_control/transform_datatypes.h"

#include <angles/angles.h>

#include <math.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

#define M_PI                                     3.14159265358979323846  /* pi */
#define ANGULAR_MOTOR_COEFFICIENT                0.2685 /* ANGULAR_MOTOR_COEFFICIENT */
#define MOTORS_INTERCEPT                         426.24 /* MOTORS_INTERCEPT [rad/s]*/
#define SAMPLING_TIME                            0.01 /* SAMPLING TIME [s] */
#define MAX_PROPELLERS_ANGULAR_VELOCITY          2618 /* MAX PROPELLERS ANGULAR VELOCITY [rad/s]*/

namespace rotors_control {

RollPitchYawrateThrustControllerCrazyflie::RollPitchYawrateThrustControllerCrazyflie()
    : initialized_params_(false),
    controller_active_(false),
    delta_psi_ki_(0),
    p_command_ki_(0),
    q_command_ki_(0){

      // The control variables are initialized to zero (the inputs come from the joystick)
      roll_pitch_yawrate_thrust_.roll = 0;
      roll_pitch_yawrate_thrust_.pitch = 0;
      roll_pitch_yawrate_thrust_.yaw_rate = 0;
      roll_pitch_yawrate_thrust_.thrust = 0;

}

RollPitchYawrateThrustControllerCrazyflie::~RollPitchYawrateThrustControllerCrazyflie() {}

// Controller gains are entered into local global variables
void RollPitchYawrateThrustControllerCrazyflie::SetControllerGains(){

      attitude_gain_kp_ = Eigen::Vector2f(controller_parameters_.attitude_gain_kp_.x(), controller_parameters_.attitude_gain_kp_.y());
      attitude_gain_ki_ = Eigen::Vector2f(controller_parameters_.attitude_gain_ki_.x(), controller_parameters_.attitude_gain_ki_.y());

      rate_gain_kp_ = Eigen::Vector3f(controller_parameters_.rate_gain_kp_.x(), controller_parameters_.rate_gain_kp_.y(), controller_parameters_.rate_gain_kp_.z());
      rate_gain_ki_ = Eigen::Vector3f(controller_parameters_.rate_gain_ki_.x(), controller_parameters_.rate_gain_ki_.y(), controller_parameters_.rate_gain_ki_.z());
}

void RollPitchYawrateThrustControllerCrazyflie::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
  assert(rotor_velocities);

  // This is to disable the controller if we do not receive a trajectory
  if(!controller_active_){
     *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
  return;
  }

  double PWM_1, PWM_2, PWM_3, PWM_4;
  ControlMixer(&PWM_1, &PWM_2, &PWM_3, &PWM_4);

  double omega_1, omega_2, omega_3, omega_4;
  omega_1 = ((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
  omega_2 = ((PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
  omega_3 = ((PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
  omega_4 = ((PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);

  //The omega values are saturated considering physical constraints of the system
  if(!(omega_1 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_1 > 0))
      if(omega_1 > MAX_PROPELLERS_ANGULAR_VELOCITY)
         omega_1 = MAX_PROPELLERS_ANGULAR_VELOCITY;
      else
         omega_1 = 0;

  if(!(omega_2 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_2 > 0))
      if(omega_2 > MAX_PROPELLERS_ANGULAR_VELOCITY)
         omega_2 = MAX_PROPELLERS_ANGULAR_VELOCITY;
      else
         omega_2 = 0;

  if(!(omega_3 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_3 > 0))
      if(omega_3 > MAX_PROPELLERS_ANGULAR_VELOCITY)
         omega_3 = MAX_PROPELLERS_ANGULAR_VELOCITY;
      else
         omega_3 = 0;

  if(!(omega_4 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_4 > 0))
      if(omega_4 > MAX_PROPELLERS_ANGULAR_VELOCITY)
         omega_4 = MAX_PROPELLERS_ANGULAR_VELOCITY;
      else
         omega_4 = 0;

  ROS_DEBUG("Omega_1: %f Omega_2: %f Omega_3: %f Omega_4: %f", omega_1, omega_2, omega_3, omega_4);
  *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);

}

void RollPitchYawrateThrustControllerCrazyflie::ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4) {
    assert(PWM_1);
    assert(PWM_2);
    assert(PWM_3);
    assert(PWM_4);

    roll_pitch_yawrate_thrust_.thrust = roll_pitch_yawrate_thrust_.thrust * controller_parameters_.thrust_percentage_;

    double delta_phi, delta_theta, delta_psi;
    RateController(&delta_phi, &delta_theta, &delta_psi);

    *PWM_1 = roll_pitch_yawrate_thrust_.thrust - (delta_theta/2) - (delta_phi/2) - delta_psi;
    *PWM_2 = roll_pitch_yawrate_thrust_.thrust + (delta_theta/2) - (delta_phi/2) + delta_psi;
    *PWM_3 = roll_pitch_yawrate_thrust_.thrust + (delta_theta/2) + (delta_phi/2) - delta_psi;
    *PWM_4 = roll_pitch_yawrate_thrust_.thrust - (delta_theta/2) + (delta_phi/2) + delta_psi;

    ROS_DEBUG("Omega: %f, Delta_theta: %f, Delta_phi: %f, delta_psi: %f", roll_pitch_yawrate_thrust_.thrust, delta_theta, delta_phi, delta_psi);
    ROS_DEBUG("PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2, *PWM_3, *PWM_4);

}

void RollPitchYawrateThrustControllerCrazyflie::RateController(double* delta_phi, double* delta_theta, double* delta_psi) {
    assert(delta_phi);
    assert(delta_theta);
    assert(delta_psi);

    double p, q, r;
    p = state_.angularVelocity.x;
    q = state_.angularVelocity.y;
    r = state_.angularVelocity.z;

    double p_command, q_command;
    AttitudeController(&p_command, &q_command);

    // The command comes from the joystick
    double r_command;
    r_command = roll_pitch_yawrate_thrust_.yaw_rate;

    double p_error, q_error, r_error;
    p_error = p_command - p;
    q_error = q_command - q;
    r_error = r_command - r;

    double delta_phi_kp, delta_theta_kp, delta_psi_kp;
    delta_phi_kp = rate_gain_kp_.x() * p_error;
    *delta_phi = delta_phi_kp;

    delta_theta_kp = rate_gain_kp_.y() * q_error;
    *delta_theta = delta_theta_kp;

    delta_psi_kp = rate_gain_kp_.z() * r_error;
    delta_psi_ki_ = delta_psi_ki_ + (rate_gain_ki_.z() * r_error * SAMPLING_TIME);
    *delta_psi = delta_psi_kp + delta_psi_ki_;

}

void RollPitchYawrateThrustControllerCrazyflie::AttitudeController(double* p_command, double* q_command) {
    assert(p_command);
    assert(q_command);

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    double theta_command, phi_command;
    theta_command = roll_pitch_yawrate_thrust_.pitch;
    phi_command = roll_pitch_yawrate_thrust_.roll;

    double phi_error, theta_error;
    phi_error = phi_command - roll;
    theta_error = theta_command - pitch;

    double p_command_kp, q_command_kp;
    p_command_kp = attitude_gain_kp_.x() * phi_error;
    p_command_ki_ = p_command_ki_ + (attitude_gain_ki_.x() * phi_error * SAMPLING_TIME);
    *p_command = p_command_kp + p_command_ki_;

    q_command_kp = attitude_gain_kp_.y() * theta_error;
    q_command_ki_ = q_command_ki_ + (attitude_gain_ki_.y() * theta_error * SAMPLING_TIME);
    *q_command = q_command_kp + q_command_ki_;

    ROS_INFO("Phi_c: %f, Phi_e: %f, Theta_c: %f, Theta_e: %f", phi_command, phi_error, theta_command, theta_error);

}

void RollPitchYawrateThrustControllerCrazyflie::SetOdometry(const EigenOdometry& odometry) {
    odometry_ = odometry;

    // Such function is invoked when the ideal odometry sensor is employed
    SetSensorData();
}

// Odometry values are put in the state structure. The structure contains the aircraft state
void RollPitchYawrateThrustControllerCrazyflie::SetSensorData() {

    // Only the position sensor is ideal, any virtual sensor or systems is available to get it
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];

    state_.attitudeQuaternion.x = odometry_.orientation.x();
    state_.attitudeQuaternion.y = odometry_.orientation.y();
    state_.attitudeQuaternion.z = odometry_.orientation.z();
    state_.attitudeQuaternion.w = odometry_.orientation.w();

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];
}

void RollPitchYawrateThrustControllerCrazyflie::SetRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrustCrazyflie& roll_pitch_yawrate_thrust) {

  roll_pitch_yawrate_thrust_ = roll_pitch_yawrate_thrust;
  controller_active_ = true;
}

void RollPitchYawrateThrustControllerCrazyflie::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    // The estimated quaternion values
    double x, y, z, w;
    x = state_.attitudeQuaternion.x;
    y = state_.attitudeQuaternion.y;
    z = state_.attitudeQuaternion.z;
    w = state_.attitudeQuaternion.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);

}



}
