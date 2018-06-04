/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
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

#ifndef CONTROLLER_PARAMETERS_H
#define CONTROLLER_PARAMETERS_H

// Default values for the position controller of the Crazyflie2. XYController [x,y], AttitudeController [phi,theta] 
//RateController [p,q,r], YawController[yaw], HoveringController[z]
static const Eigen::Vector2f kPDefaultXYController = Eigen::Vector2f(15, -15);
static const Eigen::Vector2f kIDefaultXYController = Eigen::Vector2f(1, -1);

static const Eigen::Vector2f kPDefaultAttitudeController = Eigen::Vector2f(3.5, 3.5);
static const Eigen::Vector2f kIDefaultAttitudeController = Eigen::Vector2f(2, 2);

static const Eigen::Vector3f kPDefaultRateController = Eigen::Vector3f(110, 110, 110);
static const Eigen::Vector3f kIDefaultRateController = Eigen::Vector3f(0, 0, 16.7);

static const double kPDefaultYawController = 3;
static const double kIDefaultYawController = 1;

static const double kPDefaultHoveringController = 14000;
static const double kIDefaultHoveringController = 15000;
static const double kDDefaultHoveringController = 20000;

namespace rotors_control {

	class PositionControllerParameters {
	 public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	  PositionControllerParameters()
	      : xy_gain_kp_(kPDefaultXYController), 
		xy_gain_ki_(kIDefaultXYController), 
		attitude_gain_kp_(kPDefaultAttitudeController), 
		attitude_gain_ki_(kIDefaultAttitudeController), 
		rate_gain_kp_(kPDefaultRateController), 
		rate_gain_ki_(kIDefaultRateController),  
		yaw_gain_kp_(kPDefaultYawController), 
		yaw_gain_ki_(kIDefaultYawController),
		hovering_gain_kp_(kPDefaultHoveringController), 
		hovering_gain_ki_(kIDefaultHoveringController), 
		hovering_gain_kd_(kDDefaultHoveringController) {
	  }

	  Eigen::Vector2f xy_gain_kp_;
	  Eigen::Vector2f xy_gain_ki_;
	  
	  Eigen::Vector2f attitude_gain_kp_;
	  Eigen::Vector2f attitude_gain_ki_;
	  
	  Eigen::Vector3f rate_gain_kp_;
	  Eigen::Vector3f rate_gain_ki_;
	  
	  double yaw_gain_kp_;
	  double yaw_gain_ki_;
	  
	  double hovering_gain_kp_;
	  double hovering_gain_ki_;
	  double hovering_gain_kd_;
	};

}

#endif // CONTROLLER_PARAMETERS_H
