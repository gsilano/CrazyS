/*
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

#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CRAZYFLIE_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CRAZYFLIE_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "common.h"
#include "parameters.h"
#include "stabilizer_types.h"
#include "controller_parameters.h"

namespace rotors_control {

// Default values for the position controller for the Crazyflie2: AttitudeController [phi,theta]
//RateController [p,q,r]
static const Eigen::Vector2d kPDefaultAttitudeController = Eigen::Vector2d(0.0611, 0.0611);
static const Eigen::Vector2d kIDefaultAttitudeController = Eigen::Vector2d(0.0349, 0.0349);

static const Eigen::Vector3d kPDefaultRateController = Eigen::Vector3d(1000, 1000, 1000);
static const Eigen::Vector3d kIDefaultRateController = Eigen::Vector3d(0, 0, 95.6839);

static const double DefaultThrust_percentage = 1.0;

class RollPitchYawrateThrustControllerCrazyflieParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RollPitchYawrateThrustControllerCrazyflieParameters()
      : attitude_gain_kp_(kPDefaultAttitudeController),
        attitude_gain_ki_(kIDefaultAttitudeController),
        rate_gain_kp_(kPDefaultRateController),
        rate_gain_ki_(kIDefaultRateController),
        thrust_percentage_(DefaultThrust_percentage) {
  }

  Eigen::Vector2d attitude_gain_kp_;
  Eigen::Vector2d attitude_gain_ki_;

  Eigen::Vector3d rate_gain_kp_;
  Eigen::Vector3d rate_gain_ki_;

  double thrust_percentage_;
};

    class RollPitchYawrateThrustControllerCrazyflie {
        public:
            RollPitchYawrateThrustControllerCrazyflie();
            ~RollPitchYawrateThrustControllerCrazyflie();
            void InitializeParameters();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometry(const EigenOdometry& odometry);
            void SetRollPitchYawrateThrust(
                const mav_msgs::EigenRollPitchYawrateThrustCrazyflie& roll_pitch_yawrate_thrust);

            void SetControllerGains();

            RollPitchYawrateThrustControllerCrazyflieParameters controller_parameters_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            bool initialized_params_;
            bool controller_active_;
            state_t state_;

            //Controller gains
            Eigen::Vector2f attitude_gain_kp_, attitude_gain_ki_;
            Eigen::Vector3f rate_gain_kp_, rate_gain_ki_;

            double delta_psi_ki_;
            double p_command_ki_, q_command_ki_;

            mav_msgs::EigenRollPitchYawrateThrustCrazyflie roll_pitch_yawrate_thrust_;
            EigenOdometry odometry_;

            void AttitudeController(double* p_command, double* q_command);
            void RateController(double* delta_phi, double* delta_theta, double* delta_psi);
            void ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4);
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
            void SetSensorData();
};
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CRAZYFLIE_H
