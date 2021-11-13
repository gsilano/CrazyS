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

#include "rotors_control/lee_position_controller_alpha.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"

#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <chrono>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>

namespace rotors_control{

  // Constructor
  LeePositionControllerAlpha::LeePositionControllerAlpha()
    : Ts_(0),
      gravity_(0),
      mass_(0),
      Jxx_(0),
      Jyy_(0),
      Jzz_(0),
      d_(0),
      maxRotorsVelocity_(0),
      c_motor_(0),
      c_tauf_(0),
      tau_(0),
      kx_(0),
      kv_(0),
      kR_(0),
      kOmega_(0),
      controller_active_(false),
      initialized_params_(false){

        state_.angularAcc.x = 0; // Angular Acceleration x
        state_.angularAcc.y = 0; // Angular Acceleration y
        state_.angularAcc.z = 0; // Angular Acceleration z

        state_.attitude.roll = 0; // Roll
        state_.attitude.pitch = 0; // Pitch
        state_.attitude.yaw = 0; // Yaw

        state_.position.x = 0; // Position.x
        state_.position.y = 0; // Position.y
        state_.position.z = 0; // Position.z

        state_.angularVelocity.x = 0; // Angular velocity x
        state_.angularVelocity.y = 0; // Angular velocity y
        state_.angularVelocity.z = 0; // Angular velocity z

        state_.linearVelocity.x = 0; //Linear velocity x
        state_.linearVelocity.y = 0; //Linear velocity y
        state_.linearVelocity.z = 0; //Linear velocity z

        state_.attitudeQuaternion.x = 0; // Quaternion x
        state_.attitudeQuaternion.y = 0; // Quaternion y
        state_.attitudeQuaternion.z = 0; // Quaternion z
        state_.attitudeQuaternion.w = 0; // Quaternion w

  }

  // Distructor
  LeePositionControllerAlpha::~LeePositionControllerAlpha() {}

  // Set odometry message
  void LeePositionControllerAlpha::SetOdometry(const EigenOdometry& odometry) {
    odometry_ = odometry;

    SetSensorData(); // call internal  SensorData() function
  }

  // Set the reference trajectory
  void LeePositionControllerAlpha::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
      command_trajectory_ = command_trajectory;
      controller_active_= true;
  }

  // Compute the propeller angular velocities
  void LeePositionControllerAlpha::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);

    if (!controller_active_) {
      *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
      return;
    }

    // Desired state
    Eigen::Vector3d pos_ref, att_ref;

    pos_ref << command_trajectory_.position_W[0], command_trajectory_.position_W[1], command_trajectory_.position_W[2];
    att_ref << command_trajectory_.orientation_W_B.x(), command_trajectory_.orientation_W_B.y(), command_trajectory_.orientation_W_B.z();

    // Drone state
    Eigen::Vector3d pos, vel, omega;
    Eigen::Matrix3d R;

    pos << state_.position.x, state_.position.y, state_.position.z; // inertial frame
    vel << state_.linearVelocity.x, state_.linearVelocity.y, state_.linearVelocity.z; // inertial frame
    omega << state_.angularVelocity.x, state_.angularVelocity.y, state_.angularVelocity.z; // body frames
    QuatToRot(R);

    // Dirty derivatives
    CalculateDerivative(&dx1dt_, pos_ref);
    CalculateDerivative(&dx2dt_, dx1dt_.out_);
    CalculateDerivative(&dx3dt_, dx2dt_.out_);
    CalculateDerivative(&dx4dt_, dx3dt_.out_);

    CalculateDerivative(&da1dt_, att_ref);
    CalculateDerivative(&da2dt_, da1dt_.out_);

    CalculateDerivative(&dv1dt_, vel);
    CalculateDerivative(&dv2dt_, dv1dt_.out_);

    // Errors
    Eigen::Vector3d   ex, ev, ea, ej;
    ex = pos - pos_ref; // position error
    ev = vel - dx1dt_.out_; // velocity error
    ea = dv1dt_.out_ - dx2dt_.out_; // acceleration errror
    ej = dv2dt_.out_ - dx3dt_.out_; // jerk error

    // Unit vector along z-axis
    Eigen::Vector3d e3(0,0,1);

    // Computing total trust
    Eigen::Vector3d A;
    double   f;
    A = -(kx_ * ex) -(kv_ * ev) -(mass_ * gravity_ * e3) +(mass_ * dx2dt_.out_);
    f = -A.dot(R*e3);

    // Desired orientation
    Eigen::Vector3d b1c, b2c, b3c, C;
    Eigen::Matrix3d Rc;

    b3c = -A / A.norm();
    C = b3c.cross(att_ref);
    b2c = C / C.norm();
    b1c = -(1/C.norm()) * b3c.cross(C);

    Rc << b1c(0), b2c(0), b3c(0),
          b1c(1), b2c(1), b3c(1),
          b1c(2), b2c(2), b3c(2);

    // First order derivative of in the body frame
    Eigen::Vector3d A_1dot, b3c_1dot, C_1dot, b2c_1dot, b1c_1dot;

    A_1dot   = -kx_ * ev - kv_ * ea + mass_*dx3dt_.out_;
    b3c_1dot = -A_1dot/A.norm() + (A.dot(A_1dot)/pow(A.norm(),3)) *A;
    C_1dot   = b3c_1dot.cross(att_ref) + b3c.cross(da1dt_.out_);
    b2c_1dot = C/C.norm() - (C.dot(C_1dot)/pow(C.norm(), 3)) *C;
    b1c_1dot = b2c_1dot.cross(b3c) + b2c.cross(b3c_1dot);

    // Second order derivative of in the body frame
    Eigen::Vector3d A_2dot, b3c_2dot, C_2dot, b2c_2dot, b1c_2dot;

    A_2dot   = -kx_ * ea - kv_ * ej + mass_ * dx4dt_.out_;
    b3c_2dot = -A_2dot/A.norm() + (2/pow(A.norm(),3))*A.dot(A_1dot)*A_1dot
             + ((pow(A_1dot.norm(),2) + A.dot(A_2dot))/pow(A.norm(),3))*A
             - (3/pow(A.norm(),5))*(pow(A.dot(A_1dot),2))*A;
    C_2dot   = b3c_2dot.cross(att_ref) + b3c.cross(dv2dt_.out_)
               + 2* b3c_1dot.cross(dv1dt_.out_);
    b2c_2dot = C_2dot/C.norm() - (2/pow(C.norm(),3))*C.dot(C_1dot)*C_1dot
             - ((pow(C_2dot.norm(),2) + C.dot(C_2dot))/pow(C.norm(),3))*C
             + (3/pow(C.norm(),5))*(pow(C.dot(C_1dot),2))*C;
    b1c_2dot = b2c_2dot.cross(b3c) + b2c.cross(b3c_2dot)
             + 2*b2c_1dot.cross(b3c_1dot);

    // Building matrices
    Eigen::Matrix3d Rc_1dot, Rc_2dot, Omegac_hat;
    Eigen::Vector3d Omegac, Omegac_1dot;

    Rc_1dot <<  b1c_1dot(0), b2c_1dot(0), b3c_1dot(0),
                b1c_1dot(1), b2c_1dot(1), b3c_1dot(1),
                b1c_1dot(2), b2c_1dot(2), b3c_1dot(2);

    Rc_2dot <<  b1c_2dot(0), b2c_2dot(0), b3c_2dot(0),
                b1c_2dot(1), b2c_2dot(1), b3c_2dot(1),
                b1c_2dot(2), b2c_2dot(2), b3c_2dot(2);

    Eigen::Matrix3d temp = Rc.transpose() * Rc_1dot;
    Vee(temp, Omegac);
    Hat(Omegac, Omegac_hat);
    temp = Rc.transpose() * Rc_2dot - Omegac_hat * Omegac_hat;
    Vee(temp, Omegac_1dot);

    // Orientation errors
    Eigen::Vector3d eR, eOmega;
    temp = Rc.transpose()*R - R.transpose()*Rc;
    Vee(temp, eR);
    eR = 0.5 * eR;
    eOmega = omega - R.transpose()*Rc*Omegac;

    // Momenta
    Eigen::Vector3d M;
    Eigen::Matrix3d Omega_hat;
    Hat(omega, Omega_hat);
    M = - kR_ * eR - kOmega_ * eOmega + omega.cross(J_*omega) - J_ * (Omega_hat*R.transpose()*Rc*Omegac - R.transpose()*Rc*Omegac_1dot);

    // Mapping angular velocity
    FMToAngVelocities(&f, M, Mix_, &maxRotorsVelocity_, *rotor_velocities);

  }

  void LeePositionControllerAlpha::FMToAngVelocities(double* force, Eigen::Vector3d &moments, Eigen::Matrix4d &mix, double* maxVel,
    Eigen::Vector4d &rotor_velocities){
    assert(force);
    assert(maxVel);

    Eigen::Vector4d inputs(*force, moments[0], moments[1], moments[2]);

    rotor_velocities = mix * inputs;

    if (rotor_velocities[0] < 0) {
        rotor_velocities[0] = 0;
    }
    else {
        rotor_velocities[0] = sqrt(rotor_velocities[0]);
        if (rotor_velocities[0] >= *maxVel) {
            rotor_velocities[0] = *maxVel;
        }
    }

    if (rotor_velocities[1] < 0) {
        rotor_velocities[1] = 0;
    }
    else {
        rotor_velocities[1] = sqrt(rotor_velocities[1]);
        if (rotor_velocities[1] >= *maxVel) {
            rotor_velocities[1] = *maxVel;
        }
    }

    if (rotor_velocities[2] < 0) {
        rotor_velocities[2] = 0;
    }
    else {
        rotor_velocities[2] = sqrt(rotor_velocities[2]);
        if (rotor_velocities[2] >= *maxVel) {
            rotor_velocities[2] = *maxVel;
        }
    }

    if (rotor_velocities[3] < 0) {
        rotor_velocities[3] = 0;
    }
    else {
        rotor_velocities[3] = sqrt(rotor_velocities[3]);
        if (rotor_velocities[3] >= *maxVel) {
            rotor_velocities[3] = *maxVel;
        }
    }

  }

  // Vee operation
  void LeePositionControllerAlpha::Vee(Eigen::Matrix3d &SkewSym, Eigen::Vector3d &result) {

      if (abs(SkewSym(0,0)) < 1e-2 || abs(SkewSym(1,1)) < 1e-2 || abs(SkewSym(2,2)) < 1e-2){
      }

      if (abs(SkewSym(1,0)+SkewSym(0,1)) < 1e-2 || abs(SkewSym(2,0)+SkewSym(0,2)) < 1e-2
               || abs(SkewSym(2,1)+SkewSym(1,2)) < 1e-2){
      }
      else{
              std::cout << "Matrix not symmetric" << std::endl;
      }

      result =  Eigen::Vector3d(SkewSym(2,1), SkewSym(0,2), SkewSym(1,0));
  }

  // Hat operation
  void LeePositionControllerAlpha::Hat(Eigen::Vector3d &Vec, Eigen::Matrix3d &result) {

      result <<     0,     -Vec[2],     Vec[1],
                 Vec[2],       0,      -Vec[0],
                -Vec[1],    Vec[0],       0;

  }

  // Quaternion Rotation Matrix
  void LeePositionControllerAlpha::QuatToRot(Eigen::Matrix3d &R){

    // Quaternion components
    double q_w = state_.attitudeQuaternion.w; double q_x = state_.attitudeQuaternion.x;
    double q_y = state_.attitudeQuaternion.y; double q_z = state_.attitudeQuaternion.z;

    // Elements of the rotation matrix
    double R_1_1 = pow(q_w,2) + pow(q_x,2) - pow(q_y,2) - pow(q_z,2);
    double R_2_1 = 2 * (q_x * q_y + q_w * q_z);
    double R_3_1 = 2 * (q_x * q_z - q_w * q_y);

    double R_1_2 = 2 * (q_x * q_y - q_w * q_z);
    double R_2_2 = pow(q_w,2) - pow(q_x,2) + pow(q_y,2) - pow(q_z,2);
    double R_3_2 = 2 * (q_y * q_z + q_w * q_x);

    double R_1_3 = 2 * (q_x * q_z + q_w * q_y);
    double R_2_3 = 2 * (q_y * q_z - q_w * q_x);
    double R_3_3 = pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2);

    R << R_1_1, R_1_2, R_1_3,
          R_2_1, R_2_2, R_2_3,
          R_3_1, R_3_2, R_3_3;

  }

  // From quaternion to Euler angles
  void LeePositionControllerAlpha::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
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

  // Odometry values are added into the state structure. The structure contains the aircraft state
  void LeePositionControllerAlpha::SetSensorData() {

      // Only the position sensor is ideal, any virtual sensor or systems is available to get it
      state_.position.x = odometry_.position[0];
      state_.position.y = odometry_.position[1];
      state_.position.z = odometry_.position[2];

      // rotating linear velocity from body to inertial frame
      double theta, phi, psi;
      Quaternion2Euler(&phi, &theta, &psi);

      state_.linearVelocity.x = (cos(theta) * cos(psi) * odometry_.velocity[0]) +
          ( ( (sin(phi) * sin(theta) * cos(psi) ) - ( cos(phi) * sin(psi) ) ) * odometry_.velocity[1]) +
          ( ( (cos(phi) * sin(theta) * cos(psi) ) + ( sin(phi) * sin(psi) ) ) *  odometry_.velocity[2]);

      state_.linearVelocity.y = (cos(theta) * sin(psi) * odometry_.velocity[0]) +
          ( ( (sin(phi) * sin(theta) * sin(psi) ) + ( cos(phi) * cos(psi) ) ) * odometry_.velocity[1]) +
          ( ( (cos(phi) * sin(theta) * sin(psi) ) - ( sin(phi) * cos(psi) ) ) *  odometry_.velocity[2]);

      state_.linearVelocity.z = (-sin(theta) * odometry_.velocity[0]) + ( sin(phi) * cos(theta) * odometry_.velocity[1]) +
          (cos(phi) * cos(theta) * odometry_.velocity[2]);

      state_.attitudeQuaternion.x = odometry_.orientation.x();
      state_.attitudeQuaternion.y = odometry_.orientation.y();
      state_.attitudeQuaternion.z = odometry_.orientation.z();
      state_.attitudeQuaternion.w = odometry_.orientation.w();

      state_.angularVelocity.x = odometry_.angular_velocity[0];
      state_.angularVelocity.y = odometry_.angular_velocity[1];
      state_.angularVelocity.z = odometry_.angular_velocity[2];
  }

  // Update dirty derivatives
  void LeePositionControllerAlpha::UpdateDirtyDerivatives() {

    // Update dirty derivatives
    dx1dt_.order_ = 1;  dx1dt_.tau_ = tau_;  dx1dt_.ts_ = Ts_;
    dx2dt_.order_ = 2;  dx2dt_.tau_ = 2*tau_;  dx2dt_.ts_ = Ts_;
    dx3dt_.order_ = 3;  dx3dt_.tau_ = 2*tau_;  dx3dt_.ts_ = Ts_;
    dx4dt_.order_ = 4;  dx4dt_.tau_ = 2*tau_;  dx4dt_.ts_ = Ts_;

    da1dt_.order_ = 1;  da1dt_.tau_ = tau_;  da1dt_.ts_ = Ts_;
    da2dt_.order_ = 2;  da2dt_.tau_ = 2*tau_;  da2dt_.ts_ = Ts_;

    dv1dt_.order_ = 1;  dv1dt_.tau_ = tau_;  dv1dt_.ts_ = Ts_;
    dv2dt_.order_ = 2;  dv2dt_.tau_ = 2*tau_;  dv2dt_.ts_ = Ts_;

    UpdateDirtyDerivativesATerms(&dx1dt_);
    UpdateDirtyDerivativesATerms(&dx2dt_);
    UpdateDirtyDerivativesATerms(&dx3dt_);
    UpdateDirtyDerivativesATerms(&dx4dt_);

    UpdateDirtyDerivativesATerms(&da1dt_);
    UpdateDirtyDerivativesATerms(&da2dt_);

    UpdateDirtyDerivativesATerms(&dv1dt_);
    UpdateDirtyDerivativesATerms(&dv2dt_);

  }

  // Computing the a terms for the dirty derivative elements
  void LeePositionControllerAlpha::UpdateDirtyDerivativesATerms(DxDt* derivative) {
    assert(derivative);

    derivative->a1_ = (2*tau_ - Ts_)/(2*tau_ + Ts_);
    derivative->a2_ = 2 / (2*tau_ + Ts_);

  }

  // Computing derivatives
  void LeePositionControllerAlpha::CalculateDerivative(DxDt* derivate_variable, Eigen::Vector3d &in) {
    assert(derivate_variable);

    derivate_variable->in_ = in;

    if (derivate_variable->it_==1){
        derivate_variable->out_ << 0,0,0;
        derivate_variable->x_d1_ << 0,0,0;
    }

    if (derivate_variable->it_ > derivate_variable->order_) {
        // Computing the derivative
        derivate_variable->out_ = derivate_variable->a1_ * derivate_variable->dot_ +
          derivate_variable->a2_ * (derivate_variable->in_ - derivate_variable->x_d1_);
    }

    derivate_variable->it_ = derivate_variable->it_++;
    derivate_variable->x_d1_ = derivate_variable->in_;

  }

  // Initialize parameters
  void LeePositionControllerAlpha::InitializeParameters() {

    Premix_ <<   c_motor_,             c_motor_,             c_motor_,             c_motor_,
                -c_motor_*d_/sqrt(2), -c_motor_*d_/sqrt(2),  c_motor_*d_/sqrt(2),  c_motor_*d_/sqrt(2),
                 c_motor_*d_/sqrt(2), -c_motor_*d_/sqrt(2), -c_motor_*d_/sqrt(2),  c_motor_*d_/sqrt(2),
                 c_tauf_,             -c_tauf_,              c_tauf_,             -c_tauf_;

    Mix_ = Premix_.inverse();

    J_ <<    Jxx_,     0,       0,
              0,      Jyy_,     0,
              0,       0,      Jzz_;

    UpdateDirtyDerivatives(); // updating dirty derivatives

    initialized_params_ = true;
  }


}
