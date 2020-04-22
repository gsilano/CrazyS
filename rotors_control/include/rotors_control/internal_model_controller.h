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

#ifndef INTERNAL_MODEL_CONTROLLER_H
#define INTERNAL_MODEL_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/DroneState.h>

#include <string>

#include <ros/time.h>

#include "stabilizer_types.h"
#include "parameters.h"
#include "common.h"

using namespace std;

namespace rotors_control {

// Default values for the Parrot Bebop controller. For more information about the control architecture, please take a look
// at the publications page into the Wiki section.
static const Eigen::Vector2d kPDefaultXYController = Eigen::Vector2d(-26.4259, -26.3627);
static const double kPDefaultAltitudeController = -27.2277;

static const double kPDefaultRollController = -1.7514;
static const double kPDefaultPitchController = -1.7513;
static const double kPDefaultYawRateController = -14.3431;

static const Eigen::Vector2d MuDefaultXYController = Eigen::Vector2d(1, 1);
static const double MuDefaultAltitudeController = 1;

static const double MuDefaultRollController = 0.0544;
static const double MuDefaultPitchController = 0.0543;
static const double MuDefaultYawRateController = 0.44;

static const Eigen::Vector3d UqDefaultXYZ = Eigen::Vector3d(1.1810, 1.1810, 4.6697);

class InternalModelControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InternalModelControllerParameters()
      : beta_xy_(kPDefaultXYController),
        beta_z_(kPDefaultAltitudeController),
        beta_phi_(kPDefaultRollController),
        beta_theta_(kPDefaultPitchController),
        beta_psi_(kPDefaultYawRateController),
        mu_xy_(MuDefaultXYController),
        mu_z_(MuDefaultAltitudeController),
        mu_theta_(MuDefaultPitchController),
        mu_phi_(MuDefaultRollController),
        mu_psi_(MuDefaultYawRateController),
	    U_q_(UqDefaultXYZ){
  }

  Eigen::Vector2d beta_xy_;
  double beta_z_;

  double beta_phi_;
  double beta_theta_;
  double beta_psi_;

  Eigen::Vector2d mu_xy_;
  double mu_z_;

  double mu_phi_;
  double mu_theta_;
  double mu_psi_;

  Eigen::Vector3d U_q_;
};

    class InternalModelController{
        public:
            InternalModelController();
            ~InternalModelController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometryWithoutStateEstimator(const EigenOdometry& odometry);
            void SetTrajectoryPoint(const mav_msgs::EigenDroneState& command_trajectory);
            void SetControllerGains();
            void SetVehicleParameters();

            InternalModelControllerParameters controller_parameters_im_;
            VehicleParameters vehicle_parameters_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            //Boolean variables to active/unactive the controller and the data storage
            bool controller_active_;

            //Gazebo Message for attitude and position
            ros::NodeHandle clientHandlePosition_;
            ros::ServiceClient clientPosition_;

            ros::NodeHandle clientHandleAttitude_;
            ros::ServiceClient clientAttitude_;

            //Controller gains
            double beta_x_, beta_y_, beta_z_;
            double beta_phi_, beta_theta_, beta_psi_;

            double alpha_x_, alpha_y_, alpha_z_;
            double alpha_phi_, alpha_theta_, alpha_psi_;

            double mu_x_, mu_y_, mu_z_;
            double mu_phi_, mu_theta_, mu_psi_;

	          double U_q_x_, U_q_y_, U_q_z_;
	          double K_x_1_, K_x_2_;
	          double K_y_1_, K_y_2_;
	          double K_z_1_, K_z_2_;

            //Position and linear velocity errors
            double e_x_;
            double e_y_;
            double e_z_;
            double dot_e_x_;
            double dot_e_y_;
            double dot_e_z_;

            //Attitude and angular velocity errors
            double e_phi_;
            double e_theta_;
            double e_psi_;
            double dot_e_phi_;
            double dot_e_theta_;
            double dot_e_psi_;

            //Vehicle parameters
            double bf_, m_, g_;
            double l_, bm_;
            double Ix_, Iy_, Iz_;

            ros::NodeHandle n1_;
            ros::NodeHandle n2_;
            ros::Timer timer1_;
            ros::Timer timer2_;

            //Callback functions to compute the errors among axis and angles
            void CallbackAttitude(const ros::TimerEvent& event);
            void CallbackPosition(const ros::TimerEvent& event);

	          state_t state_;
            control_t control_;
            mav_msgs::EigenDroneState command_trajectory_;
            EigenOdometry odometry_;

            void SetOdometryEstimated();
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
            void AttitudeController(double* u_phi, double* u_theta, double* u_psi);
            void AngularVelocityErrors(double* dot_e_phi_, double* dot_e_theta_, double* dot_e_psi_);
            void AttitudeErrors(double* e_phi_, double* e_theta_, double* e_psi_);
            void PosController(double* u_T, double* phi_r, double* theta_r, double* u_x, double* u_y, double* u_z, double* u_Terr);
            void PositionErrors(double* e_x, double* e_y, double* e_z);
            void VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z);
            void Quaternion2EulerCommandTrajectory(double* roll, double* pitch, double* yaw) const;

    };

}
#endif // INTERNAL_MODEL_CONTROLLER_H
