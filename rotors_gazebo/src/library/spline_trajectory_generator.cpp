/*
 * Copyright 2020 Ria Sonecha, Massachusetts Institute of Technology in Cambridge, MA, USA
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
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

#include "rotors_gazebo/spline_trajectory_generator.h"

#include <cassert>
#include <cmath>

#define DELTA_1   0.5
#define ALPHA_1   2
#define ALPHA_2   20
#define ALPHA_3   8
#define ALPHA_4   12
#define ALPHA_5   3
#define BETA_1    30
#define BETA_2    14
#define BETA_3    16
#define BETA_4    2
#define GAMMA_1   12
#define GAMMA_2   6

namespace rotors_gazebo {

  SplineTrajectoryGenerator::SplineTrajectoryGenerator(){

    // Parameters initializion
    a0_.setZero(); a1_.setZero(); a2_.setZero(); a3_.setZero(); a4_.setZero(); a5_.setZero();
    b0_.setZero(); b1_.setZero(); b2_.setZero(); b3_.setZero(); b4_.setZero();
    c0_.setZero(); c1_.setZero(); c2_.setZero(); c3_.setZero();

    g0_.setZero(); g1_.setZero(); g2_.setZero(); g3_.setZero(); g4_.setZero(); g5_.setZero();
    h0_.setZero(); h1_.setZero(); h2_.setZero(); h3_.setZero(); h4_.setZero();
    i0_.setZero(); i1_.setZero(); i2_.setZero(); i3_.setZero();

  }

  SplineTrajectoryGenerator::~SplineTrajectoryGenerator(){}

  void SplineTrajectoryGenerator::TrajectoryCallback(SplineReference* odometry, double* time_final, double* time_init) {
    assert(odometry);
    assert(time_final);
    assert(time_init);

    double time_spline;
    time_spline = *time_final - *time_init;

    if (enable_parameter_computation_){
      ComputeSplineParameters(&time_final_);
      enable_parameter_computation_ = false;

    }


    // Desired Position
    double x_component, y_component, z_component;
    x_component = a5_.x() * pow(time_spline, 5) + a4_.x() * pow(time_spline, 4) + a3_.x() * pow(time_spline, 3) + a2_.x() * pow(time_spline, 2)
                  + a1_.x() * time_spline + a0_.x();
    y_component = a5_.y() * pow(time_spline, 5) + a4_.y() * pow(time_spline, 4) + a3_.y() * pow(time_spline, 3) + a2_.y() * pow(time_spline, 2)
                  + a1_.y() * time_spline + a0_.y();
    z_component = a5_.z() * pow(time_spline, 5) + a4_.z() * pow(time_spline, 4) + a3_.z() * pow(time_spline, 3) + a2_.z() * pow(time_spline, 2)
                  + a1_.z() * time_spline + a0_.z();


    odometry->position_W = Eigen::Vector3f(x_component, y_component, z_component);


    // Desired Linear Velocity
    x_component = b4_.x() * pow(time_spline, 4) + b3_.x() * pow(time_spline, 3) + b2_.x() * pow(time_spline, 2) + b1_.x() * time_spline
                  + b0_.x();
    y_component = b4_.y() * pow(time_spline, 4) + b3_.y() * pow(time_spline, 3) + b2_.y() * pow(time_spline, 2) + b1_.y() * time_spline
                  + b0_.y();
    z_component = b4_.z() * pow(time_spline, 4) + b3_.z() * pow(time_spline, 3) + b2_.z() * pow(time_spline, 2) + b1_.z() * time_spline
                  + b0_.z();


    odometry->velocity = Eigen::Vector3f(x_component, y_component, z_component);


    // Desired Acceleration
    x_component = c3_.x() * pow(time_spline, 3) + c2_.x() * pow(time_spline, 2) + c1_.x() * time_spline + c0_.x();
    y_component = c3_.y() * pow(time_spline, 3) + c2_.y() * pow(time_spline, 2) + c1_.y() * time_spline + c0_.y();
    z_component = c3_.z() * pow(time_spline, 3) + c2_.z() * pow(time_spline, 2) + c1_.z() * time_spline + c0_.z();


    odometry->acceleration = Eigen::Vector3f(x_component, y_component, z_component);


    // Desired Attitute
    rollDesRad_ = g5_.x() * pow(time_spline, 5) + g4_.x() * pow(time_spline, 4) + g3_.x() * pow(time_spline, 3) + g2_.x() * pow(time_spline, 2)
                + g1_.x() * time_spline + g0_.x();
    pitchDesRad_ = g5_.y() * pow(time_spline, 5) + g4_.y() * pow(time_spline, 4) + g3_.y() * pow(time_spline, 3) + g2_.y() * pow(time_spline, 2)
                + g1_.y() * time_spline + g0_.y();
    yawDesRad_ = g5_.z() * pow(time_spline, 5) + g4_.z() * pow(time_spline, 4) + g3_.z() * pow(time_spline, 3) + g2_.z() * pow(time_spline, 2)
                + g1_.z() * time_spline + g0_.z();


    // Converts radians in quaternions
    double x, y, z, w;
    Euler2QuaternionCommandTrajectory(&x, &y, &z, &w);
    odometry->orientation_W_B = Eigen::Quaterniond(w, x, y, z);


    // Desired Angular Velocity
    double roll_component, pitch_component, yaw_component;
    roll_component = h4_.x() * pow(time_spline, 4) + h3_.x() * pow(time_spline, 3) + h2_.x() * pow(time_spline, 2) + h1_.x() * time_spline + h0_.x();
    pitch_component = h4_.y() * pow(time_spline, 4) + h3_.y() * pow(time_spline, 3) + h2_.y() * pow(time_spline, 2) + h1_.y() * time_spline + h0_.y();
    yaw_component = h4_.z() * pow(time_spline, 4) + h3_.z() * pow(time_spline, 3) + h2_.z() * pow(time_spline, 2) + h1_.z() * time_spline + h0_.z();


    // The angular rates are rotated from the inertial to the body frame
    // https://home.aero.polimi.it/trainelli/downloads/Bottasso_ThreeDimensionalRotations.pdf, eqs. 1.206 and 1.208
    double roll_component_B, pitch_component_B, yaw_component_B;
    roll_component_B = roll_component - sin(pitchDesRad_) * yaw_component;
    pitch_component_B = cos(rollDesRad_) * pitch_component + sin(rollDesRad_) * cos(pitchDesRad_) * yaw_component;
    yaw_component_B = -sin(rollDesRad_) * pitch_component + cos(rollDesRad_) * cos(pitchDesRad_) * yaw_component;

    odometry->angular_velocity_B = Eigen::Vector3f(roll_component_B, pitch_component_B, yaw_component_B);


  }

  void SplineTrajectoryGenerator::ComputeSplineParameters(double* time_spline){
      assert(time_spline);

      /*        POSITION       */
      // Parameters used for the position and its derivatives. The coefficient
      // refers to the x, y and z-avis
      a0_ = position_initial_;


      a1_ = velocity_initial_;


      a2_.x() = acceleration_initial_.x() * DELTA_1;
      a2_.y() = acceleration_initial_.y() * DELTA_1;
      a2_.z() = acceleration_initial_.z() * DELTA_1;


      a3_.x() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.x() - position_initial_.x()) -
                (ALPHA_3 * velocity_final_.x() + ALPHA_4 * velocity_initial_.x()) * *time_spline -
                (ALPHA_5 * acceleration_final_.x() - acceleration_initial_.x()) * pow(*time_spline, 2));

      a3_.y() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.y() - position_initial_.y()) -
                (ALPHA_3 * velocity_final_.y() + ALPHA_4 * velocity_initial_.y()) * *time_spline -
                (ALPHA_5 * acceleration_final_.y() - acceleration_initial_.y()) * pow(*time_spline, 2));

      a3_.z() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.z() - position_initial_.z()) -
                (ALPHA_3 * velocity_final_.z() + ALPHA_4 * velocity_initial_.z()) * *time_spline -
                (ALPHA_5 * acceleration_final_.z() - acceleration_initial_.z()) * pow(*time_spline, 2));


      a4_.x() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.x() - position_final_.x()) +
                (BETA_2 * velocity_final_.x() + BETA_3 * velocity_initial_.x()) * *time_spline +
                (BETA_3 * acceleration_final_.x() - BETA_4 * acceleration_initial_.x()) * pow(*time_spline, 2));

      a4_.y() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.y() - position_final_.y()) +
                (BETA_2 * velocity_final_.y() + BETA_3 * velocity_initial_.y()) * *time_spline +
                (BETA_3 * acceleration_final_.y() - BETA_4 * acceleration_initial_.y()) * pow(*time_spline, 2));

      a4_.z() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.z() - position_final_.z()) +
                (BETA_2 * velocity_final_.z() + BETA_3 * velocity_initial_.z()) * *time_spline +
                (BETA_3 * acceleration_final_.z() - BETA_4 * acceleration_initial_.z()) * pow(*time_spline, 2));


      a5_.x() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.x() - position_initial_.x()) -
                GAMMA_2 * (velocity_final_.x() + velocity_initial_.x()) * *time_spline -
                (acceleration_final_.x() - acceleration_initial_.x()) * pow(*time_spline, 2));

      a5_.y() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.y() - position_initial_.y()) -
                GAMMA_2 * (velocity_final_.y() + velocity_initial_.y()) * *time_spline -
                (acceleration_final_.y() - acceleration_initial_.y()) * pow(*time_spline, 2));

      a5_.z() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.z() - position_initial_.z()) -
                GAMMA_2 * (velocity_final_.z() + velocity_initial_.z()) * *time_spline -
                (acceleration_final_.z() - acceleration_initial_.z()) * pow(*time_spline, 2));


      // The coefficients are used for having the velocity reference trajectory. The polynomial is computed
      // derivating the a one. In other words,
      // a5 * x^5 + a4 * x^4 + a3 * x^3 + a2 * x^2 + a1 * x + a0
      // 5 * a5 * x^4 + 4 * a4 * x^3 + 3 * a3 * x^2 + 2 * a2 * x + a1
      // b4 = 5 * a5 -- b3 = 4 * a4 -- b2 = 3 * a3 - b1 = 2 * a2 - b0 = a1
      // and so on
      b4_ = 5 * a5_;
      b3_ = 4 * a4_;
      b2_ = 3 * a3_;
      b1_ = 2 * a2_;
      b0_ = a1_;


      // The coefficients are used for having the acceleration reference trajectory. The polynomial is computed
      // derivating the b one. In other words,
      // 4 * b4 * x^4 + 3 * b3 * x^3 + 2 * b2 * x^2 + b1 * x
      // c3 = 4 * b4 -- c2 = 3 * b3 -- c1 = 2 * b2 - c0 = b1
      c3_ = 4 * b4_;
      c2_ = 3 * b3_;
      c1_ = 2 * b2_;
      c0_ = b1_;


      /*        ORIENTATION      */
      // Parameters used for the orientation and its derivatives. The coefficient
      // refers to the ROLL (X), PITCH (Y) and YAW (Z)
      g0_ = orientation_initial_;


      g1_ = angular_velocity_initial_;


      g2_.x() = angular_acceleration_initial_.x() * DELTA_1;
      g2_.y() = angular_acceleration_initial_.y() * DELTA_1;
      g2_.z() = angular_acceleration_initial_.z() * DELTA_1;


      g3_.x() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.x() - orientation_initial_.x()) -
                (ALPHA_3 * angular_velocity_final_.x() + ALPHA_4 * angular_velocity_initial_.x()) * *time_spline -
                (ALPHA_5 * angular_acceleration_final_.x() - angular_acceleration_initial_.x()) * pow(*time_spline, 2));

      g3_.y() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.y() - orientation_initial_.y()) -
                (ALPHA_3 * angular_velocity_final_.y() + ALPHA_4 * angular_velocity_initial_.y()) * *time_spline -
                (ALPHA_5 * angular_acceleration_final_.y() - angular_acceleration_initial_.y()) * pow(*time_spline, 2));

      g3_.z() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.z() - orientation_initial_.z()) -
                (ALPHA_3 * angular_velocity_final_.z() + ALPHA_4 * angular_velocity_initial_.z()) * *time_spline -
                (ALPHA_5 * angular_acceleration_final_.z() - angular_acceleration_initial_.z()) * pow(*time_spline, 2));


      g4_.x() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.x() - orientation_final_.x()) +
                (BETA_2 * angular_velocity_final_.x() + BETA_3 * angular_velocity_initial_.x()) * *time_spline +
                (BETA_3 * angular_acceleration_final_.x() - BETA_4 * angular_acceleration_initial_.x()) * pow(*time_spline, 2));

      g4_.y() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.y() - orientation_final_.y()) +
                (BETA_2 * angular_velocity_final_.y() + BETA_3 * angular_velocity_initial_.y()) * *time_spline +
                (BETA_3 * angular_acceleration_final_.y() - BETA_4 * angular_acceleration_initial_.y()) * pow(*time_spline, 2));

      g4_.z() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.z() - orientation_final_.z()) +
                (BETA_2 * angular_velocity_final_.z() + BETA_3 * angular_velocity_initial_.z()) * *time_spline +
                (BETA_3 * angular_acceleration_final_.z() - BETA_4 * angular_acceleration_initial_.z()) * pow(*time_spline, 2));


      g5_.x() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.x() - orientation_initial_.x()) -
                GAMMA_2 * (angular_velocity_final_.x() + angular_velocity_initial_.x()) * *time_spline -
                (angular_acceleration_final_.x() - angular_acceleration_initial_.x()) * pow(*time_spline, 2));

      g5_.y() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.y() - orientation_initial_.y()) -
                GAMMA_2 * (angular_velocity_final_.y() + angular_velocity_initial_.y()) * *time_spline -
                (angular_acceleration_final_.y() - angular_acceleration_initial_.y()) * pow(*time_spline, 2));

      g5_.z() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.z() - orientation_initial_.z()) -
                GAMMA_2 * (angular_velocity_final_.z() + angular_velocity_initial_.z()) * *time_spline -
                (angular_acceleration_final_.z() - angular_acceleration_initial_.z()) * pow(*time_spline, 2));


      // The coefficients are used for having the angular velocity reference trajectory. The polynomial is computed
      // derivating the g one. In other words,
      // g5 * x^5 + g4 * x^4 + g3 * x^3 + g2 * x^2 + g1 * x + g0
      // 5 * g5 * x^4 + 4 * g4 * x^3 + 3 * g3 * x^2 + 2 * g2 * x + g1
      // h4 = 5 * g5 -- h3 = 4 * g4 -- h2 = 3 * g3 - h1 = 2 * g2 - h0 = g1
      // and so on
      h4_ = 5 * g5_;
      h3_ = 4 * g4_;
      h2_ = 3 * g3_;
      h1_ = 2 * g2_;
      h0_ = g1_;


      // The coefficients are used for having the angular acceleration reference trajectory. The polynomial is computed
      // derivating the b one. In other words,
      // 4 * h4 * x^4 + 3 * h3 * x^3 + 2 * h2 * x^2 + h1 * x
      // i3 = 4 * h4 -- i2 = 3 * h3 -- i1 = 2 * h2 - i0 = h1
      i3_ = 4 * h4_;
      i2_ = 3 * h3_;
      i1_ = 2 * h2_;
      i0_ = h1_;


  }

  void SplineTrajectoryGenerator::Euler2QuaternionCommandTrajectory(double* x, double* y, double* z, double* w) const {
      assert(x);
      assert(y);
      assert(z);
      assert(w);

      // Abbreviations for the various angular functions
      double cy = cos(yawDesRad_ * 0.5);
      double sy = sin(yawDesRad_ * 0.5);
      double cp = cos(pitchDesRad_ * 0.5);
      double sp = sin(pitchDesRad_ * 0.5);
      double cr = cos(rollDesRad_ * 0.5);
      double sr = sin(rollDesRad_ * 0.5);

      *w = cy * cp * cr + sy * sp * sr;
      *x = cy * cp * sr - sy * sp * cr;
      *y = sy * cp * sr + cy * sp * cr;
      *z = sy * cp * cr - cy * sp * sr;

    }

  void SplineTrajectoryGenerator::SetParameters(const SplineTrajectoryParameters& p) {
    time_final_ = p.time_final;
    position_initial_ = p.position_initial;
    position_final_ = p.position_final;
    velocity_initial_ = p.velocity_initial;
    velocity_final_ = p.velocity_final;
    acceleration_initial_ = p.acceleration_initial;
    acceleration_final_ = p.acceleration_final;
    orientation_initial_ = p.orientation_initial;
    orientation_final_ = p.orientation_final;
    angular_velocity_initial_ = p.angular_velocity_initial;
    angular_velocity_final_ = p.angular_velocity_final;
    angular_acceleration_initial_ = p.angular_acceleration_initial;
    angular_acceleration_final_ = p.angular_acceleration_final;
    enable_parameter_computation_ = true;
  }

}
