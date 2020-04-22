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

#ifndef CONTROLLER_PARAMETERS_MELLINGER_H
#define CONTROLLER_PARAMETERS_MELLINGER_H

// Default values for the position controller of the Crazyflie2. XYController [x,y], AttitudeController [phi,theta]
//RateController [p,q,r], YawController[yaw], HoveringController[z]
static const Eigen::Vector2f kpXYDefaultPositionController = Eigen::Vector2f(0.4, 0.4);
static const Eigen::Vector2f kdXYDefaultPositionController = Eigen::Vector2f(0.2, 0.2);
static const Eigen::Vector2f kiXYDefaultPositionController = Eigen::Vector2f(0.05, 0.05);

static const double kpZDefaultPositionController = 1.25;
static const double kdZDefaultPositionController = 0.4;
static const double kiZDefaultPositionController = 0.05;

static const Eigen::Vector2f krXYDefaultPositionController = Eigen::Vector2f(70000, 70000);
static const Eigen::Vector2f kwXYDefaultPositionController = Eigen::Vector2f(20000, 20000);
static const Eigen::Vector2f ki_mXYDefaultPositionController = Eigen::Vector2f(0.0, 0.0);

static const double krZDefaultPositionController = 60000;
static const double kwZDefaultPositionController = 12000;
static const double ki_mZDefaultPositionController = 500;

static const Eigen::Vector2f iRangeMXY = Eigen::Vector2f(1.0, 1.0);
static const Eigen::Vector2f iRangeXY = Eigen::Vector2f(2.0, 2.0);

static const double iRangeMZ = 1500;
static const double iRangeZ = 0.4;

static const Eigen::Vector2f kdOmegaRP = Eigen::Vector2f(200, 200);


namespace rotors_control {

	class MellingerControllerParameters {
	 public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	  MellingerControllerParameters()
	      : kpXYPositionController_(kpXYDefaultPositionController),
					kdXYPositionController_(kdXYDefaultPositionController),
					kiXYPositionController_(kiXYDefaultPositionController),
					kpZPositionController_(kpZDefaultPositionController),
					kdZPositionController_(kdZDefaultPositionController),
					kiZPositionController_(kiZDefaultPositionController),
					krXYPositionController_(krXYDefaultPositionController),
					kwXYPositionController_(kwXYDefaultPositionController),
					ki_mXYPositionController_(ki_mXYDefaultPositionController),
					krZPositionController_(krZDefaultPositionController),
					kwZPositionController_(kwZDefaultPositionController),
					ki_mZPositionController_(ki_mZDefaultPositionController),
					iRangeMXY_(iRangeMXY),
					iRangeXY_(iRangeXY),
					iRangeMZ_(iRangeMZ),
					iRangeZ_(iRangeZ),
					kdOmegaRP_(kdOmegaRP){
	  }


		Eigen::Vector2f kpXYPositionController_;
		Eigen::Vector2f kdXYPositionController_;
		Eigen::Vector2f kiXYPositionController_;

		double kpZPositionController_;
		double kdZPositionController_;
		double kiZPositionController_;

		Eigen::Vector2f krXYPositionController_;
		Eigen::Vector2f kwXYPositionController_;
		Eigen::Vector2f ki_mXYPositionController_;

		double krZPositionController_;
		double kwZPositionController_;
		double ki_mZPositionController_;

		Eigen::Vector2f iRangeMXY_;
		Eigen::Vector2f iRangeXY_;

		double iRangeMZ_;
		double iRangeZ_;

		Eigen::Vector2f kdOmegaRP_;
	};

}

#endif // CONTROLLER_PARAMETERS_MELLINGER_H
