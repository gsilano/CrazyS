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

#include "rotors_control/internal_model_controller.h"
#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/stabilizer_types.h"

#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <ros/ros.h>
#include <chrono>
#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>


#define M_PI                      3.14159265358979323846  /* pi */
#define TsP                       10e-3  /* Position control sampling time */
#define TsA                       5e-3 /* Attitude control sampling time */
#define MAX_ROTOR_VELOCITY        2618 /* Max rotors velocity [rad/s] */
#define MIN_ROTOR_VELOCITY        0 /* Min rotors velocity [rad/s] */
#define POW_MAX_ROTOR_VELOCITY    MAX_ROTOR_VELOCITY*MAX_ROTOR_VELOCITY /* Squared max rotors velocity [rad/s] */

using namespace std;

namespace rotors_control {

InternalModelController::InternalModelController()
    : controller_active_(false),
      e_x_(0),
      e_y_(0),
      e_z_(0),
      dot_e_x_(0),
      dot_e_y_(0),
      dot_e_z_(0),
      e_phi_(0),
      e_theta_(0),
      e_psi_(0),
      dot_e_phi_(0),
      dot_e_theta_(0),
      dot_e_psi_(0),
      bf_(0),
      l_(0),
      bm_(0),
      m_(0),
      g_(0),
      Ix_(0),
      Iy_(0),
      Iz_(0),
      beta_x_(0),
      beta_y_(0),
      beta_z_(0),
      beta_phi_(0),
      beta_theta_(0),
      beta_psi_(0),
      alpha_x_(0),
      alpha_y_(0),
      alpha_z_(0),
      alpha_phi_(0),
      alpha_theta_(0),
      alpha_psi_(0),
      mu_x_(0),
      mu_y_(0),
      mu_z_(0),
      mu_phi_(0),
      mu_theta_(0),
      mu_psi_(0),
      K_x_1_(0),
      K_y_1_(0),
      K_z_1_(0),
      K_x_2_(0),
      K_y_2_(0),
      K_z_2_(0),
      U_q_x_(0),
      U_q_y_(0),
      U_q_z_(0),
      control_({0,0,0,0}), //roll, pitch, yaw rate, thrust
      state_({0,  //Position.x
              0,  //Position.y
              0,  //Position.z
              0,  //Linear velocity x
              0,  //Linear velocity y
              0,  //Linear velocity z
              0,  //Quaternion x
              0,  //Quaternion y
              0,  //Quaternion z
              0,  //Quaternion w
              0,  //Angular velocity x
              0,  //Angular velocity y
              0}) //Angular velocity z)
              {

			// Command signals initialization - position
      command_trajectory_.position_W[0] = 0;
      command_trajectory_.position_W[1] = 0;
      command_trajectory_.position_W[2] = 0;

      // velocity
      command_trajectory_.velocity[0] = 0;
      command_trajectory_.velocity[1] = 0;
      command_trajectory_.velocity[2] = 0;

      // Timers set the outer and inner loops working frequency
      timer1_ = n1_.createTimer(ros::Duration(TsA), &InternalModelController::CallbackAttitude, this, false, true);
      timer2_ = n2_.createTimer(ros::Duration(TsP), &InternalModelController::CallbackPosition, this, false, true);


}

//The library Destructor
InternalModelController::~InternalModelController() {}

// The function moves the controller gains read from controller_bebop.yaml file to private variables of the class.
// These variables will be employed during the simulation
void InternalModelController::SetControllerGains(){

    beta_x_ = controller_parameters_im_.beta_xy_.x();
    beta_y_ = controller_parameters_im_.beta_xy_.y();
    beta_z_ = controller_parameters_im_.beta_z_;

    beta_phi_ = controller_parameters_im_.beta_phi_;
    beta_theta_ = controller_parameters_im_.beta_theta_;
    beta_psi_ = controller_parameters_im_.beta_psi_;

    alpha_x_ = 1 - beta_x_;
    alpha_y_ = 1 - beta_y_;
    alpha_z_ = 1 - beta_z_;

    alpha_phi_ = 1 - beta_phi_;
    alpha_theta_ = 1 - beta_theta_;
    alpha_psi_ = 1 - beta_psi_;

    mu_x_ = controller_parameters_im_.mu_xy_.x();
    mu_y_ = controller_parameters_im_.mu_xy_.y();
    mu_z_ = controller_parameters_im_.mu_z_;

    mu_phi_ = controller_parameters_im_.mu_phi_;
    mu_theta_ = controller_parameters_im_.mu_theta_;
    mu_psi_ = controller_parameters_im_.mu_psi_;

    U_q_x_ = controller_parameters_im_.U_q_.x();
    U_q_y_ = controller_parameters_im_.U_q_.y();
    U_q_z_ = controller_parameters_im_.U_q_.z();

    K_x_1_ = 1/mu_x_;
    K_x_2_ = -2 * (beta_x_/mu_x_);

    K_y_1_ = 1/mu_y_;
    K_y_2_ = -2 * (beta_y_/mu_y_);

    K_z_1_ = 1/mu_z_;
    K_z_2_ = -2 * (beta_z_/mu_z_);

    ROS_INFO_ONCE("[Internal Model Controller] Controller gains successful setup");

}

// As SetControllerGains, the function is used to set the vehicle parameters into private variables of the class
void InternalModelController::SetVehicleParameters(){

    bf_ = vehicle_parameters_.bf_;
    l_ = vehicle_parameters_.armLength_;
    bm_ = vehicle_parameters_.bm_;
    m_ = vehicle_parameters_.mass_;
    g_ = vehicle_parameters_.gravity_;
    Ix_ = vehicle_parameters_.inertia_(0,0);
    Iy_ = vehicle_parameters_.inertia_(1,1);
    Iz_ = vehicle_parameters_.inertia_(2,2);

    ROS_INFO_ONCE("[Internal Model Controller] Vehicle parameters successful setup");

}

// The functions is used to convert the drone attitude from quaternion to Euler angles
void InternalModelController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    double x, y, z, w;
    x = odometry_.orientation.x();
    y = odometry_.orientation.y();
    z = odometry_.orientation.z();
    w = odometry_.orientation.w();

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

}

// When a new odometry message comes, the content of the message is stored in private variable. At the same time, the controller is going to be active.
// The attitude of the aircraft is computer (as we said before it move from quaterninon to Euler angles) and also the angular velocity is stored.
void InternalModelController::SetOdometryWithoutStateEstimator(const EigenOdometry& odometry) {

    odometry_ = odometry;
    controller_active_= true;

    Quaternion2Euler(&state_.attitude.roll, &state_.attitude.pitch, &state_.attitude.yaw);

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];

    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];

    ROS_DEBUG("Drone position [x: %f, y: %f, z: %f]", state_.position.x, state_.position.y, state_.position.z);
    ROS_DEBUG("Drone attitude [roll: %f, pitch: %f, yaw: %f]", state_.attitude.roll, state_.attitude.pitch, state_.attitude.yaw);
    ROS_DEBUG("Drone linear velocity [x: %f, y: %f, z: %f]", state_.linearVelocity.x, state_.linearVelocity.y, state_.linearVelocity.z);

}

// The function sets the filter trajectory points
void InternalModelController::SetTrajectoryPoint(const mav_msgs::EigenDroneState& command_trajectory) {

    command_trajectory_ = command_trajectory;

    double psi_r, roll_r, pitch_r;
    Quaternion2EulerCommandTrajectory(&roll_r, &pitch_r, &psi_r);

    ROS_DEBUG("Drone desired position [x_d: %f, y_d: %f, z_d: %f]",  command_trajectory_.position_W[0],
            command_trajectory_.position_W[1], command_trajectory_.position_W[2]);
    ROS_DEBUG("Drone desired attitude [roll_d: %f, pitch_d: %f, yaw_d: %f]", roll_r, pitch_r, psi_r);
    ROS_DEBUG("Drone desired linear velocity [x_d_dot: %f, y_d_dot: %f, z_d_dot: %f]", command_trajectory_.velocity[0],
          command_trajectory_.velocity[1], command_trajectory_.velocity[2]);


}

// The function computes the propellers angular velocity
void InternalModelController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);

    // The controller is inactive if a point to reach is not coming
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
    }

    double u_phi, u_theta, u_psi;
    double u_x, u_y, u_z, u_Terr;
    AttitudeController(&u_phi, &u_theta, &u_psi);
    // control_.roll = phi_r, desired roll angle
    // control_.pitch = theta_r, desired pitch angle
	  PosController(&control_.thrust, &control_.roll, &control_.pitch, &u_x, &u_y, &u_z, &u_Terr);

    ROS_DEBUG("Thrust: %f U_phi: %f U_theta: %f U_psi: %f", control_.thrust, u_phi, u_theta, u_psi);

    double first, second, third, fourth;
    first = (1 / ( 4 * bf_ )) * control_.thrust;
    second = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_phi;
    third = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_theta;
    fourth = (1 / ( 4 * bf_ * bm_)) * u_psi;

    double not_saturated_1, not_saturated_2, not_saturated_3, not_saturated_4;
    not_saturated_1 = first - second - third - fourth;
    not_saturated_2 = first - second + third + fourth;
    not_saturated_3 = first + second + third - fourth;
    not_saturated_4 = first + second - third + fourth;

    // The propellers velocity is limited by taking into account the physical constrains
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

    ROS_DEBUG("Omega_1: %f Omega_2: %f Omega_3: %f Omega_4: %f", omega_1, omega_2, omega_3, omega_4);

}

// The function computes the velocity errors
void InternalModelController::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

   double dot_x, dot_y, dot_z, theta, psi, phi;

   theta = state_.attitude.pitch;
   psi = state_.attitude.yaw;
   phi = state_.attitude.roll;

   // The linear and angular velocity provided as outputs by the ideal odometry sensor are expressed
   // in the body-frame. Therefore, such measurements have to be rotated before computing the error
   dot_x = (cos(theta) * cos(psi) * state_.linearVelocity.x) +
             ( ( (sin(phi) * sin(theta) * cos(psi) ) - ( cos(phi) * sin(psi) ) ) * state_.linearVelocity.y) +
             ( ( (cos(phi) * sin(theta) * cos(psi) ) + ( sin(phi) * sin(psi) ) ) *  state_.linearVelocity.z);

   dot_y = (cos(theta) * sin(psi) * state_.linearVelocity.x) +
             ( ( (sin(phi) * sin(theta) * sin(psi) ) + ( cos(phi) * cos(psi) ) ) * state_.linearVelocity.y) +
             ( ( (cos(phi) * sin(theta) * sin(psi) ) - ( sin(phi) * cos(psi) ) ) *  state_.linearVelocity.z);

   dot_z = (-sin(theta) * state_.linearVelocity.x) + ( sin(phi) * cos(theta) * state_.linearVelocity.y) +
             (cos(phi) * cos(theta) * state_.linearVelocity.z);

   *dot_e_x = command_trajectory_.velocity[0] - dot_x;
   *dot_e_y = command_trajectory_.velocity[1] - dot_y;
   *dot_e_z = command_trajectory_.velocity[2] - dot_z;
}

// The function computes the position errors
void InternalModelController::PositionErrors(double* e_x, double* e_y, double* e_z){
   assert(e_x);
   assert(e_y);
   assert(e_z);

   *e_x = command_trajectory_.position_W[0] - state_.position.x;
   *e_y = command_trajectory_.position_W[1] - state_.position.y;
   *e_z = command_trajectory_.position_W[2] - state_.position.z;

   ROS_DEBUG("Position Error components along the z-axis: [z_d: %f, z: %f]", command_trajectory_.position_W[2], state_.position.z);
   ROS_DEBUG("Position Error: [%f %f %f]", *e_x, *e_y, *e_z);

}

// The function computes the position controller outputs
void InternalModelController::PosController(double* u_T, double* phi_r, double* theta_r, double* u_x, double* u_y, double* u_z, double* u_Terr){
   assert(u_T);
   assert(phi_r);
   assert(theta_r);
   assert(u_x);
   assert(u_y);
   assert(u_z);
   assert(u_Terr);

   //u_x computing
   *u_x = ( (e_x_ * K_x_1_ * K_x_2_)/U_q_x_ ) + ( (dot_e_x_ * K_x_2_)/U_q_x_ );

   if (*u_x > 1 || *u_x <-1)
	   if (*u_x > 1)
		   *u_x = 1;
	   else
		   *u_x = -1;

   *u_x = (*u_x * 1/2) + ( (K_x_1_/U_q_x_) * dot_e_x_ );

   if (*u_x > 1 || *u_x <-1)
	   if (*u_x > 1)
		   *u_x = 1;
	   else
		   *u_x = -1;

   *u_x = m_ * (*u_x * U_q_x_);

   //u_y computing
   *u_y = ( (e_y_ * K_y_1_ * K_y_2_)/U_q_y_ ) + ( (dot_e_y_ * K_y_2_)/U_q_y_ );

   if (*u_y > 1 || *u_y <-1)
	   if (*u_y > 1)
		   *u_y = 1;
	   else
		   *u_y = -1;

   *u_y = (*u_y * 1/2) + ( (K_y_1_/U_q_y_) * dot_e_y_ );

   if (*u_y > 1 || *u_y <-1)
	   if (*u_y > 1)
		   *u_y = 1;
	   else
		   *u_y = -1;

   *u_y = m_* ( *u_y * U_q_y_);

   //u_z computing
   *u_z = ( (e_z_ * K_z_1_ * K_z_2_)/U_q_z_ ) + ( (dot_e_z_ * K_z_2_)/U_q_z_ );

   if (*u_z > 1 || *u_z <-1)
	   if (*u_z > 1)
		   *u_z = 1;
	   else
		   *u_z = -1;

   *u_z = (*u_z * 1/2) + ( (K_z_1_/U_q_z_) * dot_e_z_ );

   if (*u_z > 1 || *u_z <-1)
	   if (*u_z > 1)
		   *u_z = 1;
	   else
		   *u_z = -1;

   *u_z = m_* ( *u_z * U_q_z_);

   //u_Terr computing
   *u_Terr = *u_z + (m_ * g_);

   //u_T computing
   *u_T = sqrt( pow(*u_x,2) + pow(*u_y,2) + pow(*u_Terr,2) );

   ROS_DEBUG("Position Control Signals: [%f %f %f %f %f]", *u_x, *u_y, *u_z, *u_T, *u_Terr);


   double psi_r, temp_a, temp_b;
   Quaternion2EulerCommandTrajectory(&temp_a, &temp_b, &psi_r);

   *theta_r = atan( ( (*u_x * cos(psi_r) ) + ( *u_y * sin(psi_r) ) )  / *u_Terr );

   *phi_r = atan( cos(*theta_r) * ( ( (*u_x * sin(psi_r)) - (*u_y * cos(psi_r)) ) / (*u_Terr) ) );

}

//The function computes the attitude errors
void InternalModelController::AttitudeErrors(double* e_phi, double* e_theta, double* e_psi){
   assert(e_phi);
   assert(e_theta);
   assert(e_psi);

   double roll_r, pitch_r, psi_r;
   Quaternion2EulerCommandTrajectory(&roll_r, &pitch_r, &psi_r);

   *e_phi = roll_r - state_.attitude.roll;
   *e_theta = pitch_r - state_.attitude.pitch;
   *e_psi = psi_r - state_.attitude.yaw;

   ROS_DEBUG("Drone attitude: [roll: %f, pitch: %f, yaw: %f]", state_.attitude.roll, state_.attitude.pitch, state_.attitude.yaw);
   ROS_DEBUG("Angular Error: [e_phi: %f, e_pitch: %f, e_psi: %f]", *e_phi, *e_theta, *e_psi);
   ROS_DEBUG("Desired Attitude: [phi_d: %f, pitch_d: %f, psi_d: %f]", roll_r, pitch_r, psi_r);

}

//The angular velocity errors
void InternalModelController::AngularVelocityErrors(double* dot_e_phi, double* dot_e_theta, double* dot_e_psi){
   assert(dot_e_phi);
   assert(dot_e_theta);
   assert(dot_e_psi);

   double psi_r, roll_r, pitch_r;
   Quaternion2EulerCommandTrajectory(&roll_r, &pitch_r, &psi_r);

   double dot_e_phi_W_s, dot_e_theta_W_s, dot_e_psi_W_s, dot_e_phi_W_d, dot_e_theta_W_d, dot_e_psi_W_d;

  // Drone angular rate rotated from body to fixed frame
   dot_e_phi_W_s = state_.angularVelocity.x + (sin(state_.attitude.roll) * tan(state_.attitude.pitch) * state_.angularVelocity.y)
                   + (cos(state_.attitude.roll) * tan(state_.attitude.pitch) * state_.angularVelocity.z);

   dot_e_theta_W_s = cos(state_.attitude.roll) * state_.angularVelocity.y - sin(state_.attitude.roll) * state_.angularVelocity.z;

   dot_e_psi_W_s = ( sin(state_.attitude.roll) * state_.angularVelocity.y) / cos(state_.attitude.pitch) +
		               ( cos(state_.attitude.roll) * state_.angularVelocity.z) / cos(state_.attitude.pitch);

   // Desired angular rate rotated from body to fixed frame
   dot_e_phi_W_d = command_trajectory_.angular_velocity_B[0] + (sin(roll_r) * tan(pitch_r) * command_trajectory_.angular_velocity_B[1])
                   + (cos(roll_r) * tan(pitch_r) * command_trajectory_.angular_velocity_B[2]);

   dot_e_theta_W_d = cos(roll_r) * command_trajectory_.angular_velocity_B[1] - sin(roll_r) * command_trajectory_.angular_velocity_B[2];

   dot_e_psi_W_d = ( sin(roll_r) * command_trajectory_.angular_velocity_B[1]) / cos(pitch_r) +
		               ( cos(roll_r) * command_trajectory_.angular_velocity_B[2]) / cos(pitch_r);

   *dot_e_phi = dot_e_phi_W_d - dot_e_phi_W_s;
   *dot_e_theta =  dot_e_theta_W_d - dot_e_theta_W_s;
   *dot_e_psi = dot_e_psi_W_d - dot_e_psi_W_s;

    ROS_DEBUG("Desired Attitude Rate: [phi_d_dot: %f, pitch_d_dot: %f, psi_d_dot: %f]", dot_e_phi_W_d, dot_e_theta_W_d, dot_e_psi_W_d);

}

void InternalModelController::Quaternion2EulerCommandTrajectory(double* roll, double* pitch, double* yaw) const {
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

//The function computes the attitude controller outputs
void InternalModelController::AttitudeController(double* u_phi, double* u_theta, double* u_psi){
   assert(u_phi);
   assert(u_theta);
   assert(u_psi);

   *u_phi = Ix_ *   ( ( ( (alpha_phi_/mu_phi_    ) * dot_e_phi_  ) - ( (beta_phi_/pow(mu_phi_,2)    ) * e_phi_  ) ) -
            ( ( (Iy_ - Iz_)/(Ix_ * mu_theta_ * mu_psi_) ) * e_theta_ * e_psi_) );
   *u_theta = Iy_ * ( ( ( (alpha_theta_/mu_theta_) * dot_e_theta_) - ( (beta_theta_/pow(mu_theta_,2)) * e_theta_) ) -
            ( ( (Iz_ - Ix_)/(Iy_ * mu_phi_ * mu_psi_  ) ) * e_phi_   * e_psi_) );
   *u_psi = Iz_ *   ( ( ( (alpha_psi_/mu_psi_    ) * dot_e_psi_  ) - ( (beta_psi_/pow(mu_psi_,2)    ) * e_psi_  ) ) -
            ( ( (Ix_ - Iy_)/(Iz_ * mu_theta_ * mu_phi_) ) * e_theta_ * e_phi_) );

}


//Every TsA this function computes the attitude and the angular rate errors
void InternalModelController::CallbackAttitude(const ros::TimerEvent& event){

     AttitudeErrors(&e_phi_, &e_theta_, &e_psi_);
     AngularVelocityErrors(&dot_e_phi_, &dot_e_theta_, &dot_e_psi_);
}

//Every TsP this function computes the position and velocity errors
void InternalModelController::CallbackPosition(const ros::TimerEvent& event){

     PositionErrors(&e_x_, &e_y_, &e_z_);
     VelocityErrors(&dot_e_x_, &dot_e_y_, &dot_e_z_);

}


}
