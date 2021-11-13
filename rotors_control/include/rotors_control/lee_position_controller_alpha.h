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

#ifndef LEE_POSITION_CONTROLLER_ALPHA_H
#define LEE_POSITION_CONTROLLER_ALPHA_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <iostream>
#include <math.h>

#include <string>

#include <ros/time.h>

#include "common.h"
#include "parameters.h"
#include "stabilizer_types.h"

#include <time.h>

using namespace std;

namespace rotors_control {

  class DxDt {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Inputs for the object
        unsigned int order_;
        double tau_;
        double ts_;

        DxDt(unsigned int DefaultOrder, double DefaultTau, double DefaultTs){ // Constructor with parameters
            order_ = DefaultOrder;
            tau_ = DefaultTau;
            ts_ = DefaultTs;
        }

        DxDt() // Constructor without parameters
          : order_(0),
            tau_(0),
            ts_(0) {
        }

        // Not provided as input
        double it_ = 1;

        double a1_ = (2*tau_ - ts_)/(2*tau_ + ts_);
        double a2_ = 2 / (2*tau_ + ts_);

        Eigen::Vector3d in_, out_; // input and output

        Eigen::Vector3d dot_ = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d x_d1_ = Eigen::Vector3d(0, 0, 0);

  };

  class LeePositionControllerAlpha{
      public:
          LeePositionControllerAlpha();
          ~LeePositionControllerAlpha();
          void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

          void InitializeParameters();
          void SetOdometry(const EigenOdometry& odometry);
          void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);

          // Controller variables
          double Ts_, gravity_, mass_, Jxx_, Jyy_, Jzz_;
          double d_, maxRotorsVelocity_, c_motor_, c_tauf_, tau_;
          double kx_, kv_, kR_, kOmega_;

      private:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          // Flags
          bool controller_active_;
          bool initialized_params_;

          Eigen::Matrix4d Premix_, Mix_;
          Eigen::Matrix3d J_;

          // Header per Msg
          std_msgs::Header act_header_;

          // Odometry
          EigenOdometry odometry_;
          state_t state_;
          mav_msgs::EigenTrajectoryPoint command_trajectory_;

          // Dirty Derivatives
          DxDt dx1dt_, dx2dt_, dx3dt_, dx4dt_; // derivative position
          DxDt da1dt_, da2dt_; // derivative attitude
          DxDt dv1dt_, dv2dt_; // derivative velocity

          // Functions
          void CalculateDerivative(DxDt* derivate_variable, Eigen::Vector3d &in);
          void UpdateDirtyDerivatives();
          void UpdateDirtyDerivativesATerms(DxDt* derivative);
          void SetSensorData();
          void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
          void QuatToRot(Eigen::Matrix3d &R);
          void Hat(Eigen::Vector3d &Vec, Eigen::Matrix3d &result);
          void Vee(Eigen::Matrix3d &SkewSym, Eigen::Vector3d &result);
          void FMToAngVelocities(double* force, Eigen::Vector3d &moments, Eigen::Matrix4d &mix, double* maxVel,
            Eigen::Vector4d &rotor_velocities);

  };

}
#endif // LEE_POSITION_CONTROLLER_ALPHA_H
