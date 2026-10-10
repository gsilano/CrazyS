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


#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_H_

#include <Eigen/Eigen>

namespace rotors_control {

// Default vehicle parameters of the Crazyflie 2.x (rotors_description/urdf).
static constexpr double kDefaultMass = 0.027;                    // [kg]
static constexpr double kDefaultArmLength = 0.046;               // [m]
static constexpr double kDefaultInertiaXx = 1.657171e-05;        // [kg m^2]
static constexpr double kDefaultInertiaYy = 1.657171e-05;        // [kg m^2]
static constexpr double kDefaultInertiaZz = 2.9261652e-05;       // [kg m^2]
static constexpr double kDefaultRotorForceConstant = 1.28192e-08;  // [kg m]
static constexpr double kDefaultRotorMomentConstant = 5.964552e-03;  // [m]
static constexpr double kDefaultGravity = 9.81;                  // [m/s^2]

class VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VehicleParameters()
      : mass_(kDefaultMass),
        gravity_(kDefaultGravity),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()),
        bm_(kDefaultRotorMomentConstant),
        bf_(kDefaultRotorForceConstant),
        armLength_(kDefaultArmLength) {}
  double mass_;
  double gravity_;
  Eigen::Matrix3d inertia_;
  double bm_;
  double bf_;
  double armLength_;
};

}  // namespace rotors_control

#endif  // INCLUDE_ROTORS_CONTROL_PARAMETERS_H_
