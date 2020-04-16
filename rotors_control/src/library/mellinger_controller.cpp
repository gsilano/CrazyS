/*
 * Copyright 2020 Ria Sonecha, Massachusetts Institute of Technology in Cambridge, MA, USA
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
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

#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/mellinger_controller.h"
#include "rotors_control/controller_parameters_mellinger.h"

#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>

#define SAMPLING_TIME             0.002 /* SAMPLING TIME [s] - 500Hz*/
#define M_PI                      3.14159265358979323846  /* pi [rad]*/
#define MAX_ROTOR_VELOCITY        2618 /* Max rotors velocity [rad/s] */
#define MIN_ROTOR_VELOCITY        0 /* Min rotors velocity [rad/s] */
#define POW_MAX_ROTOR_VELOCITY    MAX_ROTOR_VELOCITY*MAX_ROTOR_VELOCITY /* Squared max rotors velocity [rad/s] */

namespace rotors_control{

  MellingerController::MellingerController()
    : prev_setpoint_omega_roll_(0),
      prev_setpoint_omega_pitch_(0),
      prev_omega_roll_(0),
      prev_omega_pitch_(0),
      i_error_m_x_(0),
      i_error_m_y_(0),
      i_error_m_z_(0),
      i_error_x_(0),
      i_error_y_(0),
      i_error_z_(0){

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

  MellingerController::~MellingerController() {}

  void MellingerController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
      assert(rotor_velocities);

      GetForce(&control_t_.thrust);

      GetMoments(&control_t_.roll, &control_t_.pitch, &control_t_.yawRate);
/*
      if (control_t_.thrust > 0) {
        Clamp(&control_t_.pitch, -M_PI/6, M_PI/6);
        Clamp(&control_t_.roll, -M_PI/6, M_PI/6);
        Clamp(&control_t_.yawRate, -M_PI/6, M_PI/6);

      } else {
        control_t_.roll = 0;
        control_t_.pitch = 0;
        control_t_.yawRate = 0;

        ControllerMellingerReset();
      }*/

      ROS_DEBUG("thrust, roll, pitch, yaw: [%f, %f, %f, %f].", control_t_.thrust, control_t_.roll, control_t_.pitch, control_t_.yawRate);

      double first, second, third, fourth;
      first = (1 / ( 4 * bf_ )) * control_t_.thrust;
      second = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * control_t_.roll;
      third = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * control_t_.pitch;
      fourth = (1 / ( 4 * bf_ * bm_)) * control_t_.yawRate;


      double not_saturated_1, not_saturated_2, not_saturated_3, not_saturated_4;
      not_saturated_1 = first - second - third - fourth;
      not_saturated_2 = first - second + third + fourth;
      not_saturated_3 = first + second + third - fourth;
      not_saturated_4 = first + second - third + fourth;

      // The propellers velocities is limited by taking into account the physical constrains
      double motorMin=not_saturated_1, motorMax=not_saturated_1, motorFix=0;

      if(not_saturated_2 < motorMin) motorMin = not_saturated_2;
      if(not_saturated_2 > motorMax) motorMax = not_saturated_2;

      if(not_saturated_3 < motorMin) motorMin = not_saturated_3;
      if(not_saturated_3 > motorMax) motorMax = not_saturated_3;

      if(not_saturated_4 < motorMin) motorMin = not_saturated_4;
      if(not_saturated_4 > motorMax) motorMax = not_saturated_4;

      if(motorMin < MIN_ROTOR_VELOCITY) motorFix = MIN_ROTOR_VELOCITY - motorMin;
      else if(motorMax > POW_MAX_ROTOR_VELOCITY) motorFix = POW_MAX_ROTOR_VELOCITY - motorMax;

      not_saturated_1 = not_saturated_1 + motorFix;
      not_saturated_2 = not_saturated_2 + motorFix;
      not_saturated_3 = not_saturated_3 + motorFix;
      not_saturated_4 = not_saturated_4 + motorFix;

      // The values have been saturated to avoid the root square of negative values
      double saturated_1, saturated_2, saturated_3, saturated_4;
      if(not_saturated_1 < 0)
        saturated_1 = 0;
      else if(not_saturated_1 > POW_MAX_ROTOR_VELOCITY)
        saturated_1 = POW_MAX_ROTOR_VELOCITY;
      else
        saturated_1 = not_saturated_1;

      if(not_saturated_2 < 0)
        saturated_2 = 0;
      else if(not_saturated_2 > POW_MAX_ROTOR_VELOCITY)
        saturated_2 = POW_MAX_ROTOR_VELOCITY;
      else
        saturated_2 = not_saturated_2;

      if(not_saturated_3 < 0)
        saturated_3 = 0;
      else if(not_saturated_3 > POW_MAX_ROTOR_VELOCITY)
        saturated_3 = POW_MAX_ROTOR_VELOCITY;
      else
        saturated_3 = not_saturated_3;

      if(not_saturated_4 < 0)
        saturated_4 = 0;
      else if(not_saturated_4 > POW_MAX_ROTOR_VELOCITY)
        saturated_4 = POW_MAX_ROTOR_VELOCITY;
      else
        saturated_4 = not_saturated_4;

      double omega_1, omega_2, omega_3, omega_4;
      omega_1 = sqrt(saturated_1);
      omega_2 = sqrt(saturated_2);
      omega_3 = sqrt(saturated_3);
      omega_4 = sqrt(saturated_4);

      *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);

      ROS_DEBUG("Propellers angular velocities: [%f, %f, %f, %f].", omega_1, omega_2, omega_3, omega_4);
  }

  void MellingerController::GetForce(double* u_1){
    assert(u_1);

    tf::Quaternion q(state_.attitudeQuaternion.x, state_.attitudeQuaternion.y, state_.attitudeQuaternion.z, state_.attitudeQuaternion.w);

    tfScalar d = q.length2();
    tfFullAssert(d != tfScalar(0.0));
    tfScalar s = tfScalar(2.0) / d;
    tfScalar xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
    tfScalar wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
    tfScalar xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
    tfScalar yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;

    Eigen::Vector3f z_axis, target_thrust;
    z_axis = Eigen::Vector3f(xz + wy, yz - wx, tfScalar(1.0) - (xx + yy));
    GetThrust(&target_thrust);

    ROS_DEBUG("Z AXIS: [%f, %f, %f]", z_axis[0], z_axis[1], z_axis[2]);
    ROS_DEBUG("THRUST: [%f, %f, %f]", target_thrust[0], target_thrust[1], target_thrust[2]);

    double tempU1;
    Vdot(&tempU1, target_thrust, z_axis);
    *u_1 = tempU1;
    ROS_DEBUG("FORCE: [%f]", tempU1);

  }

  void MellingerController::GetMoments(double* u_2, double* u_3, double* u_4){
    assert(u_2);
    assert(u_3);
    assert(u_4);

    Eigen::Vector3f target_thrust, x_c_desired, z_axis_desired, x_axis_desired, y_axis_desired;
    GetThrust(&target_thrust);

    Vnormalize(&z_axis_desired, target_thrust);
    double rollDesired, pitchDesired, yawDesired;
    Quaternion2EulerCommandTrajectory(&rollDesired, &pitchDesired, &yawDesired);
    x_c_desired.x() = cos(yawDesired);
    x_c_desired.y() = sin(yawDesired);
    x_c_desired.z() = 0;

    Eigen::Vector3f tempVector;
    Vcross(&tempVector, z_axis_desired, x_c_desired);
    Vnormalize(&y_axis_desired, tempVector);
    Vcross(&x_axis_desired, y_axis_desired, z_axis_desired);

    double x, y, z, w;
    // Quaternions - Drone orientation
    x = state_.attitudeQuaternion.x;
    y = state_.attitudeQuaternion.y;
    z = state_.attitudeQuaternion.z;
    w = state_.attitudeQuaternion.w;

    ROS_DEBUG("y axis des: [%f, %f, %f].", y_axis_desired.x(), y_axis_desired.y(), y_axis_desired.z());
    ROS_DEBUG("z axis des: [%f, %f, %f].", z_axis_desired.x(), z_axis_desired.y(), z_axis_desired.z());
    ROS_DEBUG("QUAT: [%f, %f, %f, %f].", x, y, z, w);

    Eigen::Vector3f eR;

    eR.x() = (-1 + 2*pow(x,2) + 2*pow(y,2))*y_axis_desired.z() + z_axis_desired.y()
      - 2*(x*y_axis_desired.x()*z + y*y_axis_desired.y()*z - x*y*z_axis_desired.x()
      + pow(x,2)*z_axis_desired.y() + pow(z,2)*z_axis_desired.y() - y*z*z_axis_desired.z())
      + 2*w*(-(y*y_axis_desired.x()) - z*z_axis_desired.x() + x*(y_axis_desired.y() + z_axis_desired.z()));

    eR.y() = x_axis_desired.z() - z_axis_desired.x() - 2*(pow(x,2)*x_axis_desired.z() + y*(x_axis_desired.z()*y - x_axis_desired.y()*z)
      - (pow(y,2) + pow(z,2))*z_axis_desired.x() + x*(-(x_axis_desired.x()*z) + y*z_axis_desired.y() + z*z_axis_desired.z())
      + w*(x*x_axis_desired.y() + z*z_axis_desired.y() - y*(x_axis_desired.x() + z_axis_desired.z())));

    eR.z() = y_axis_desired.x() - 2*(y*(x*x_axis_desired.x() + y*y_axis_desired.x() - x*y_axis_desired.y())
      + w*(x*x_axis_desired.z() + y*y_axis_desired.z())) + 2*(-(x_axis_desired.z()*y)
      + w*(x_axis_desired.x() + y_axis_desired.y()) + x*y_axis_desired.z())*z - 2*y_axis_desired.x()*pow(z,2)
      + x_axis_desired.y()*(-1 + 2*pow(x,2) + 2*pow(z,2));


    //omega error is the opposite of how it is calculated in the paper
    Eigen::Vector3f ew;
    ew.x() = command_trajectory_.angular_velocity_B[0] - state_.angularVelocity.x;
    ew.y() = command_trajectory_.angular_velocity_B[1] - state_.angularVelocity.y;
    ew.z() = command_trajectory_.angular_velocity_B[2] - state_.angularVelocity.z;

    ROS_DEBUG("eR: [%f, %f, %f].", eR.x(), eR.y(), eR.z());
    ROS_DEBUG("ew: [%f, %f, %f].", ew.x(), ew.y(), ew.z());
    ROS_DEBUG("command trajectory angular velocity: [%f, %f, %f].", command_trajectory_.angular_velocity_B[0], command_trajectory_.angular_velocity_B[1], command_trajectory_.angular_velocity_B[2]);
    ROS_DEBUG("state angular velocity: [%f, %f, %f].", state_.angularVelocity.x, state_.angularVelocity.y, state_.angularVelocity.z);


    double err_d_roll, err_d_pitch;
    /*d part initialized*/
    err_d_roll = ((command_trajectory_.angular_velocity_B[0] - prev_setpoint_omega_roll_)
                  - (state_.angularVelocity.x - prev_omega_roll_)) / SAMPLING_TIME ;
    err_d_pitch = ((command_trajectory_.angular_velocity_B[1] - prev_setpoint_omega_pitch_)
                  - (state_.angularVelocity.y - prev_omega_pitch_)) / SAMPLING_TIME ;

    prev_omega_roll_ = state_.angularVelocity.x;
    prev_omega_pitch_ = state_.angularVelocity.y;
    prev_setpoint_omega_roll_ = command_trajectory_.angular_velocity_B[0];
    prev_setpoint_omega_pitch_ = command_trajectory_.angular_velocity_B[1];

    i_error_m_x_ = i_error_m_x_ - (eR.x() * SAMPLING_TIME);
    Clamp(&i_error_m_x_, -iRangeMXY_.x(), iRangeMXY_.x());
    i_error_m_y_ = i_error_m_y_ - (eR.y() * SAMPLING_TIME);
    Clamp(&i_error_m_y_, -iRangeMXY_.y(), iRangeMXY_.y());
    i_error_m_z_ = i_error_m_z_ - (eR.z() * SAMPLING_TIME);
    Clamp(&i_error_m_z_, -iRangeMZ_, iRangeMZ_);

    ROS_DEBUG("integral error: [%f, %f, %f].", i_error_m_x_, i_error_m_y_, i_error_m_z_);
    ROS_DEBUG("error d: [%f, %f].", err_d_roll, err_d_pitch);

    *u_2 = -krXYPositionController_.x() * eR.x() + kwXYPositionController_.x() * ew.x()
      + ki_mXYPositionController_.x() * i_error_m_x_;//+ kdOmegaRP_.x() * err_d_roll;
    *u_3 = -krXYPositionController_.y() * eR.y() + kwXYPositionController_.y() * ew.y()
      + ki_mXYPositionController_.y() * i_error_m_y_; //+ kdOmegaRP_.y() * err_d_pitch;
    *u_4 = -krZPositionController_ * eR.z() + kwZPositionController_ * ew.z()
      + ki_mZPositionController_ * i_error_m_z_;
  }

  // normalize a vector (make a unit vector).
  void MellingerController::Vnormalize(Eigen::Vector3f* vectorOutput, Eigen::Vector3f vectorInput) {
    	assert(vectorOutput);

      double a_x, a_y, a_z;
      a_x = vectorInput.x();
      a_y = vectorInput.y();
      a_z = vectorInput.z();

      double length, x, y, z;
      length = sqrt((a_x * a_x) + (a_y * a_y) + (a_z * a_z));

      x = a_x/length;
      y = a_y/length;
      z = a_z/length;

      *vectorOutput = Eigen::Vector3f(x, y, z);
  }

  // vector cross product
  void MellingerController::Vcross(Eigen::Vector3f* vectorOutput, Eigen::Vector3f vectorA, Eigen::Vector3f vectorB){
      assert(vectorOutput);

      double firstElement, secondElement, thirdElement;
      firstElement = vectorA.y()*vectorB.z() - vectorA.z()*vectorB.y();
      secondElement = vectorA.z()*vectorB.x() - vectorA.x()*vectorB.z();
      thirdElement = vectorA.x()*vectorB.y() - vectorA.y()*vectorB.x();

      *vectorOutput = Eigen::Vector3f(firstElement, secondElement, thirdElement);
  }

  // subtract vectors
  void MellingerController::Vsub(Eigen::Vector3f* vectorOutput, Eigen::Vector3f vectorA, point_t vectorB){
      assert(vectorOutput);

      double first, second, third;
      first = vectorA.x()-vectorB.x;
      second = vectorA.y()-vectorB.y;
      third = vectorA.z()-vectorB.z;
      *vectorOutput = Eigen::Vector3f(first, second, third);
  }

  void MellingerController::Vdot(double* output, Eigen::Vector3f vectorA, Eigen::Vector3f vectorB){
      assert(output);

      *output = (vectorA.x()*vectorB.x()) + (vectorA.y()*vectorB.y()) + (vectorA.z()*vectorB.z());
  }

  void MellingerController::Clamp(double* value, double min, double max){
      assert(value);

      if (*value < min) *value = min;
      if (*value > max) *value = max;
  }

  void MellingerController::GetThrust(Eigen::Vector3f* target_thrust){
      assert(target_thrust);

      //in section IV of the paper error is defined as current position-desired position
      //here we use desired position-current position
      Eigen::Vector3f r_error, v_error;
      Vsub(&r_error, command_trajectory_.position_W, state_.position);
      Vsub(&v_error, command_trajectory_.velocity, state_.linearVelocity);

      //double g_vehicleMass;
      //g_vehicleMass = m_ * g_;

      i_error_x_ = i_error_x_ + r_error.x() * SAMPLING_TIME ;
      Clamp(&i_error_x_, -iRangeXY_.x(), iRangeXY_.x());
      i_error_y_ = i_error_y_ + r_error.y() * SAMPLING_TIME ;
      Clamp(&i_error_y_, -iRangeXY_.y(), iRangeXY_.y());
      i_error_z_ = i_error_z_ + r_error.z() * SAMPLING_TIME ;
      Clamp(&i_error_z_, -iRangeZ_, iRangeZ_);

      double first, second, third;
      first = m_ * command_trajectory_.acceleration[0]
      + kpXYPositionController_.x() * r_error.x() + kdXYPositionController_.x() * v_error.x()
      + kiXYPositionController_.x() * i_error_x_;

      second = m_ * command_trajectory_.acceleration[1]
      + kpXYPositionController_.y() * r_error.y() + kdXYPositionController_.y() * v_error.y()
      + kiXYPositionController_.y() * i_error_y_;

      third = m_ * (command_trajectory_.acceleration[2] + g_)
      + kpZPositionController_  * r_error.z() + kdZPositionController_  * v_error.z()
      + kiZPositionController_  * i_error_z_;

      *target_thrust = Eigen::Vector3f(first, second, third);
      ROS_DEBUG("v_error: [%f, %f, %f].", v_error[0], v_error[1], v_error[2]);
      ROS_DEBUG("gravity: [%f].", g_);
      ROS_DEBUG("thrust: [%f, %f, %f].", first, second, third);

  }

  // Acceleration comes from an ideal IMU sensor
  void MellingerController::SetSensorData(const sensorData_t& sensors) {

      sensors_ = sensors;

  }

  void MellingerController::SetControllerGains(){

      kpXYPositionController_ = Eigen::Vector2f(controller_parameters_mellinger_.kpXYPositionController_.x(), controller_parameters_mellinger_.kpXYPositionController_.y());
      kdXYPositionController_ = Eigen::Vector2f(controller_parameters_mellinger_.kdXYPositionController_.x(), controller_parameters_mellinger_.kdXYPositionController_.y());
      kiXYPositionController_ = Eigen::Vector2f(controller_parameters_mellinger_.kiXYPositionController_.x(), controller_parameters_mellinger_.kiXYPositionController_.y());

      kpZPositionController_ = controller_parameters_mellinger_.kpZPositionController_;
      kdZPositionController_ = controller_parameters_mellinger_.kdZPositionController_;
      kiZPositionController_ = controller_parameters_mellinger_.kiZPositionController_;

      krXYPositionController_ = Eigen::Vector2f(controller_parameters_mellinger_.krXYPositionController_.x(), controller_parameters_mellinger_.krXYPositionController_.y());
      kwXYPositionController_ = Eigen::Vector2f(controller_parameters_mellinger_.kwXYPositionController_.x(), controller_parameters_mellinger_.kwXYPositionController_.y());
      ki_mXYPositionController_ = Eigen::Vector2f(controller_parameters_mellinger_.ki_mXYPositionController_.x(), controller_parameters_mellinger_.ki_mXYPositionController_.y());

      krZPositionController_ = controller_parameters_mellinger_.krZPositionController_;
      kwZPositionController_ = controller_parameters_mellinger_.kwZPositionController_;
      ki_mZPositionController_ = controller_parameters_mellinger_.ki_mZPositionController_;

      iRangeMXY_ = Eigen::Vector2f(controller_parameters_mellinger_.iRangeMXY_.x(), controller_parameters_mellinger_.iRangeMXY_.y());
      iRangeXY_ = Eigen::Vector2f(controller_parameters_mellinger_.iRangeXY_.x(), controller_parameters_mellinger_.iRangeXY_.y());

      iRangeMZ_ = controller_parameters_mellinger_.iRangeMZ_;
      iRangeZ_ = controller_parameters_mellinger_.iRangeZ_;

      kdOmegaRP_ = Eigen::Vector2f(controller_parameters_mellinger_.kdOmegaRP_.x(), controller_parameters_mellinger_.kdOmegaRP_.y());

  }

  void MellingerController::SetVehicleParameters(){

      bf_ = vehicle_parameters_.bf_;
      l_ = vehicle_parameters_.armLength_;
      bm_ = vehicle_parameters_.bm_;
      m_ = vehicle_parameters_.mass_;
      g_ = vehicle_parameters_.gravity_;
      Ix_ = vehicle_parameters_.inertia_(0,0);
      Iy_ = vehicle_parameters_.inertia_(1,1);
      Iz_ = vehicle_parameters_.inertia_(2,2);
  }

  void MellingerController::SetOdometryWithoutStateEstimator(const EigenOdometry& odometry) {
      odometry_ = odometry;

      SetDroneState();
  }

  void MellingerController::SetDroneState(){

    // Only the position sensor is ideal, any virtual sensor or systems is available to get it
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.attitudeQuaternion.x = odometry_.orientation.x();
    state_.attitudeQuaternion.y = odometry_.orientation.y();
    state_.attitudeQuaternion.z = odometry_.orientation.z();
    state_.attitudeQuaternion.w = odometry_.orientation.w();

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


    ROS_DEBUG("quaternion [%f, %f, %f, %f]", odometry_.orientation.x(), odometry_.orientation.y(), odometry_.orientation.z(), odometry_.orientation.w());

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];

  }

  void MellingerController::SetTrajectoryPoint(const mav_msgs::EigenDroneState& command_trajectory) {
      command_trajectory_= command_trajectory;
      controller_active_= true;
  }

  void MellingerController::Quaternion2EulerCommandTrajectory(double* roll, double* pitch, double* yaw) const {
      assert(roll);
      assert(pitch);
      assert(yaw);

      // The estimated quaternion values
      double x, y, z, w;
      x = command_trajectory_.orientation_W_B.x();
      y = command_trajectory_.orientation_W_B.y();
      z = command_trajectory_.orientation_W_B.z();
      w = command_trajectory_.orientation_W_B.w();

      tf::Quaternion q(x, y, z, w);
      tf::Matrix3x3 m(q);
      m.getRPY(*roll, *pitch, *yaw);

      ROS_DEBUG("Roll Trajectory: %f, Pitch Trajectory: %f, Yaw Trajectory: %f", *roll, *pitch, *yaw);

  }

  void MellingerController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
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

  void MellingerController::ControllerMellingerReset(void) {
    i_error_x_ = 0;
    i_error_y_ = 0;
    i_error_z_ = 0;
    i_error_m_x_ = 0;
    i_error_m_y_ = 0;
    i_error_m_z_ = 0;
  }
}
