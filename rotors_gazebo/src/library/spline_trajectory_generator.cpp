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

#include <thread>
#include <chrono>

#include "rotors_gazebo/Quaternion.h"
#include "rotors_gazebo/transform_datatypes.h"
#include "rotors_gazebo/parameters_ros.h"
#include <nav_msgs/Odometry.h>
#include "rotors_gazebo/Matrix3x3.h"
#include <ros/console.h>
#include <time.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include "rotors_gazebo/spline_trajectory_generator.h"

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
#define SAMPLING_TIME  10e-3       /* SAMPLING CONTROLLER TIME [s] - 100Hz */
#define START_SIMULATION_TIME 3   /* TIME GAZEBO NEEDS TO INITIALIZE THE ENVIRONMENT */

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

  void SplineTrajectoryGenerator::TrajectoryCallback(mav_msgs::EigenDroneState* odometry, double* time_final, double* time_init) {
    assert(odometry);
    assert(time_final);
    assert(time_init);

    double time_spline;
    time_spline = *time_final - *time_init;

    if (enable_parameter_computation_){
      ComputeSplineParameters(&time_final_);
      enable_parameter_computation_ = false;

      ROS_DEBUG("Publishing time_final_: %f", time_final_);
    }

    ROS_DEBUG("Publishing time_spline: %f", time_spline);

    // Desired Position
    double x_component, y_component, z_component;
    x_component = a5_.x() * pow(time_spline, 5) + a4_.x() * pow(time_spline, 4) + a3_.x() * pow(time_spline, 3) + a2_.x() * pow(time_spline, 2)
                  + a1_.x() * time_spline + a0_.x();
    y_component = a5_.y() * pow(time_spline, 5) + a4_.y() * pow(time_spline, 4) + a3_.y() * pow(time_spline, 3) + a2_.y() * pow(time_spline, 2)
                  + a1_.y() * time_spline + a0_.y();
    z_component = a5_.z() * pow(time_spline, 5) + a4_.z() * pow(time_spline, 4) + a3_.z() * pow(time_spline, 3) + a2_.z() * pow(time_spline, 2)
                  + a1_.z() * time_spline + a0_.z();

    ROS_DEBUG("Publishing position spline parameters along x-axis: [%f, %f, %f, %f, %f, %f].", a0_.x(), a1_.x(), a2_.x(), a3_.x(), a4_.x(), a5_.x());
    ROS_DEBUG("Publishing position spline parameters along y-axis: [%f, %f, %f, %f, %f, %f].", a0_.y(), a1_.y(), a2_.y(), a3_.y(), a4_.y(), a5_.y());
    ROS_DEBUG("Publishing position spline parameters along z-axis: [%f, %f, %f, %f, %f, %f].", a0_.z(), a1_.z(), a2_.z(), a3_.z(), a4_.z(), a5_.z());

    odometry->position_W = Eigen::Vector3f(x_component, y_component, z_component);

    ROS_DEBUG("Publishing position waypoint: [%f, %f, %f].", x_component, y_component, z_component);

    // Desired Linear Velocity
    x_component = b4_.x() * pow(time_spline, 4) + b3_.x() * pow(time_spline, 3) + b2_.x() * pow(time_spline, 2) + b1_.x() * time_spline
                  + b0_.x();
    y_component = b4_.y() * pow(time_spline, 4) + b3_.y() * pow(time_spline, 3) + b2_.y() * pow(time_spline, 2) + b1_.y() * time_spline
                  + b0_.y();
    z_component = b4_.z() * pow(time_spline, 4) + b3_.z() * pow(time_spline, 3) + b2_.z() * pow(time_spline, 2) + b1_.z() * time_spline
                  + b0_.z();

    ROS_DEBUG("Publishing velocity spline parameters along x-axis: [%f, %f, %f, %f, %f].", b0_.x(), b1_.x(), b2_.x(), b3_.x(), b4_.x());
    ROS_DEBUG("Publishing velocity spline parameters along y-axis: [%f, %f, %f, %f, %f].", b0_.y(), b1_.y(), b2_.y(), b3_.y(), b4_.y());
    ROS_DEBUG("Publishing velocity spline parameters along z-axis: [%f, %f, %f, %f, %f].", b0_.z(), b1_.z(), b2_.z(), b3_.z(), b4_.z());

    odometry->velocity = Eigen::Vector3f(x_component, y_component, z_component);

    ROS_DEBUG("Publishing velocity waypoint: [%f, %f, %f].", x_component, y_component, z_component);

    // Desired Acceleration
    x_component = c3_.x() * pow(time_spline, 3) + c2_.x() * pow(time_spline, 2) + c1_.x() * time_spline + c0_.x();
    y_component = c3_.y() * pow(time_spline, 3) + c2_.y() * pow(time_spline, 2) + c1_.y() * time_spline + c0_.y();
    z_component = c3_.z() * pow(time_spline, 3) + c2_.z() * pow(time_spline, 2) + c1_.z() * time_spline + c0_.z();

    ROS_DEBUG("Publishing acceleration spline parameters along y-axis: [%f, %f, %f, %f].", c0_.y(), c1_.y(), c2_.y(), c3_.y());
    ROS_DEBUG("Publishing acceleration spline parameters along x-axis: [%f, %f, %f, %f].", c0_.x(), c1_.x(), c2_.x(), c3_.x());
    ROS_DEBUG("Publishing acceleration spline parameters along z-axis: [%f, %f, %f, %f].", c0_.z(), c1_.z(), c2_.z(), c3_.z());

    odometry->acceleration = Eigen::Vector3f(x_component, y_component, z_component);

    ROS_DEBUG("Publishing acceleration waypoint: [%f, %f, %f].", x_component, y_component, z_component);

    // Desired Attitute
    rollDesRad_ = g5_.x() * pow(time_spline, 5) + g4_.x() * pow(time_spline, 4) + g3_.x() * pow(time_spline, 3) + g2_.x() * pow(time_spline, 2)
                + g1_.x() * time_spline + g0_.x();
    pitchDesRad_ = g5_.y() * pow(time_spline, 5) + g4_.y() * pow(time_spline, 4) + g3_.y() * pow(time_spline, 3) + g2_.y() * pow(time_spline, 2)
                + g1_.y() * time_spline + g0_.y();
    yawDesRad_ = g5_.z() * pow(time_spline, 5) + g4_.z() * pow(time_spline, 4) + g3_.z() * pow(time_spline, 3) + g2_.z() * pow(time_spline, 2)
                + g1_.z() * time_spline + g0_.z();

    ROS_DEBUG("Publishing orientation spline parameters along x-axis: [%f, %f, %f, %f, %f, %f].", g0_.x(), g1_.x(), g2_.x(), g3_.x(), g4_.x(), g5_.x());
    ROS_DEBUG("Publishing orientation spline parameters along y-axis: [%f, %f, %f, %f, %f, %f].", g0_.y(), g1_.y(), g2_.y(), g3_.y(), g4_.y(), g5_.y());
    ROS_DEBUG("Publishing orientation spline parameters along z-axis: [%f, %f, %f, %f, %f, %f].", g0_.z(), g1_.z(), g2_.z(), g3_.z(), g4_.z(), g5_.z());

    // Converts radians in quaternions
    double x, y, z, w;
    Euler2QuaternionCommandTrajectory(&x, &y, &z, &w);
    odometry->orientation_W_B = Eigen::Quaterniond(w, x, y, z);

    ROS_DEBUG("Publishing attitude waypoint: [%f, %f, %f].", rollDesRad_, pitchDesRad_, yawDesRad_);

    // Desired Angular Velocity
    double roll_component, pitch_component, yaw_component;
    roll_component = h4_.x() * pow(time_spline, 4) + h3_.x() * pow(time_spline, 3) + h2_.x() * pow(time_spline, 2) + h1_.x() * time_spline + h0_.x();
    pitch_component = h4_.y() * pow(time_spline, 4) + h3_.y() * pow(time_spline, 3) + h2_.y() * pow(time_spline, 2) + h1_.y() * time_spline + h0_.y();
    yaw_component = h4_.z() * pow(time_spline, 4) + h3_.z() * pow(time_spline, 3) + h2_.z() * pow(time_spline, 2) + h1_.z() * time_spline + h0_.z();

    ROS_DEBUG("Publishing angular velocity waypoint: [%f, %f, %f].", roll_component, pitch_component, yaw_component);

    // The angular rates are rotated from the inertial to the body frame
    // https://home.aero.polimi.it/trainelli/downloads/Bottasso_ThreeDimensionalRotations.pdf, eqs. 1.206 and 1.208
    double roll_component_B, pitch_component_B, yaw_component_B;
    roll_component_B = roll_component - sin(pitchDesRad_) * yaw_component;
    pitch_component_B = cos(rollDesRad_) * pitch_component + sin(rollDesRad_) * cos(pitchDesRad_) * yaw_component;
    yaw_component_B = -sin(rollDesRad_) * pitch_component + cos(rollDesRad_) * cos(pitchDesRad_) * yaw_component;

    odometry->angular_velocity_B = Eigen::Vector3f(roll_component_B, pitch_component_B, yaw_component_B);

    ROS_DEBUG("Publishing angular velocity waypoint in the body frame: [%f, %f, %f].", roll_component_B, pitch_component_B, yaw_component_B);

  }

  void SplineTrajectoryGenerator::ComputeSplineParameters(double* time_spline){
      assert(time_spline);

      /*        POSITION       */
      // Parameters used for the position and its derivatives. The coefficient
      // refers to the x, y and z-avis
      a0_ = position_initial_;

      ROS_DEBUG("Publishing position initial: [%f, %f, %f].", position_initial_[0], position_initial_[1], position_initial_[2]);
      ROS_DEBUG("Content of the a0 coefficient: [%f, %f, %f].", a0_[0], a0_[1], a0_[2]);

      a1_ = velocity_initial_;

      ROS_DEBUG("Publishing velocity initial: [%f, %f, %f].", velocity_initial_[0], velocity_initial_[1], velocity_initial_[2]);
      ROS_DEBUG("Content of the a1 coefficient: [%f, %f, %f].", a1_[0], a1_[1], a1_[2]);

      a2_.x() = acceleration_initial_.x() * DELTA_1;
      a2_.y() = acceleration_initial_.y() * DELTA_1;
      a2_.z() = acceleration_initial_.z() * DELTA_1;

      ROS_DEBUG("Publishing acceleration initial: [%f, %f, %f].", acceleration_initial_[0], acceleration_initial_[1], acceleration_initial_[2]);
      ROS_DEBUG("Content of the a2 coefficient: [%f, %f, %f].", a2_[0], a2_[1], a2_[2]);

      a3_.x() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.x() - position_initial_.x()) -
                (ALPHA_3 * velocity_final_.x() + ALPHA_4 * velocity_initial_.x()) * *time_spline -
                (ALPHA_5 * acceleration_final_.x() - acceleration_initial_.x()) * pow(*time_spline, 2));

      a3_.y() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.y() - position_initial_.y()) -
                (ALPHA_3 * velocity_final_.y() + ALPHA_4 * velocity_initial_.y()) * *time_spline -
                (ALPHA_5 * acceleration_final_.y() - acceleration_initial_.y()) * pow(*time_spline, 2));

      a3_.z() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.z() - position_initial_.z()) -
                (ALPHA_3 * velocity_final_.z() + ALPHA_4 * velocity_initial_.z()) * *time_spline -
                (ALPHA_5 * acceleration_final_.z() - acceleration_initial_.z()) * pow(*time_spline, 2));

      ROS_DEBUG("Content of the a3 coefficient: [%f, %f, %f].", a3_[0], a3_[1], a3_[2]);

      a4_.x() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.x() - position_final_.x()) +
                (BETA_2 * velocity_final_.x() + BETA_3 * velocity_initial_.x()) * *time_spline +
                (BETA_3 * acceleration_final_.x() - BETA_4 * acceleration_initial_.x()) * pow(*time_spline, 2));

      a4_.y() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.y() - position_final_.y()) +
                (BETA_2 * velocity_final_.y() + BETA_3 * velocity_initial_.y()) * *time_spline +
                (BETA_3 * acceleration_final_.y() - BETA_4 * acceleration_initial_.y()) * pow(*time_spline, 2));

      a4_.z() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.z() - position_final_.z()) +
                (BETA_2 * velocity_final_.z() + BETA_3 * velocity_initial_.z()) * *time_spline +
                (BETA_3 * acceleration_final_.z() - BETA_4 * acceleration_initial_.z()) * pow(*time_spline, 2));

      ROS_DEBUG("Content of the a4 coefficient: [%f, %f, %f].", a4_[0], a4_[1], a4_[2]);

      a5_.x() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.x() - position_initial_.x()) -
                GAMMA_2 * (velocity_final_.x() + velocity_initial_.x()) * *time_spline -
                (acceleration_final_.x() - acceleration_initial_.x()) * pow(*time_spline, 2));

      a5_.y() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.y() - position_initial_.y()) -
                GAMMA_2 * (velocity_final_.y() + velocity_initial_.y()) * *time_spline -
                (acceleration_final_.y() - acceleration_initial_.y()) * pow(*time_spline, 2));

      a5_.z() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.z() - position_initial_.z()) -
                GAMMA_2 * (velocity_final_.z() + velocity_initial_.z()) * *time_spline -
                (acceleration_final_.z() - acceleration_initial_.z()) * pow(*time_spline, 2));

      ROS_DEBUG("Content of the a5 coefficient: [%f, %f, %f].", a5_[0], a5_[1], a5_[2]);

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

      ROS_DEBUG("Content of the b0 coefficient: [%f, %f, %f].", b0_[0], b0_[1], b0_[2]);
      ROS_DEBUG("Content of the b1 coefficient: [%f, %f, %f].", b1_[0], b1_[1], b1_[2]);
      ROS_DEBUG("Content of the b2 coefficient: [%f, %f, %f].", b2_[0], b2_[1], b2_[2]);
      ROS_DEBUG("Content of the b3 coefficient: [%f, %f, %f].", b3_[0], b3_[1], b3_[2]);
      ROS_DEBUG("Content of the b4 coefficient: [%f, %f, %f].", b4_[0], b4_[1], b4_[2]);

      // The coefficients are used for having the acceleration reference trajectory. The polynomial is computed
      // derivating the b one. In other words,
      // 4 * b4 * x^4 + 3 * b3 * x^3 + 2 * b2 * x^2 + b1 * x
      // c3 = 4 * b4 -- c2 = 3 * b3 -- c1 = 2 * b2 - c0 = b1
      c3_ = 4 * b4_;
      c2_ = 3 * b3_;
      c1_ = 2 * b2_;
      c0_ = b1_;

      ROS_DEBUG("Content of the c0 coefficient: [%f, %f, %f].", c0_[0], c0_[1], c0_[2]);
      ROS_DEBUG("Content of the c1 coefficient: [%f, %f, %f].", c1_[0], c1_[1], c1_[2]);
      ROS_DEBUG("Content of the c2 coefficient: [%f, %f, %f].", c2_[0], c2_[1], c2_[2]);
      ROS_DEBUG("Content of the c3 coefficient: [%f, %f, %f].", c3_[0], c3_[1], c3_[2]);

      /*        ORIENTATION      */
      // Parameters used for the orientation and its derivatives. The coefficient
      // refers to the ROLL (X), PITCH (Y) and YAW (Z)
      g0_ = orientation_initial_;

      ROS_DEBUG("Publishing orientation initial: [%f, %f, %f].", orientation_initial_[0], orientation_initial_[1], orientation_initial_[2]);
      ROS_DEBUG("Content of the g0 coefficient: [%f, %f, %f].", g0_[0], g0_[1], g0_[2]);

      g1_ = angular_velocity_initial_;

      ROS_DEBUG("Publishing angular rate initial: [%f, %f, %f].", angular_velocity_initial_[0], angular_velocity_initial_[1], angular_velocity_initial_[2]);
      ROS_DEBUG("Content of the g1 coefficient: [%f, %f, %f].", g1_[0], g1_[1], g1_[2]);

      g2_.x() = angular_acceleration_initial_.x() * DELTA_1;
      g2_.y() = angular_acceleration_initial_.y() * DELTA_1;
      g2_.z() = angular_acceleration_initial_.z() * DELTA_1;

      ROS_DEBUG("Publishing angular acceleration initial: [%f, %f, %f].", angular_acceleration_initial_[0], angular_acceleration_initial_[1],
                angular_acceleration_initial_[2]);
      ROS_DEBUG("Content of the g2 coefficient: [%f, %f, %f].", g2_[0], g2_[1], g2_[2]);

      g3_.x() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.x() - orientation_initial_.x()) -
                (ALPHA_3 * angular_velocity_final_.x() + ALPHA_4 * angular_velocity_initial_.x()) * *time_spline -
                (ALPHA_5 * angular_acceleration_final_.x() - angular_acceleration_initial_.x()) * pow(*time_spline, 2));

      g3_.y() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.y() - orientation_initial_.y()) -
                (ALPHA_3 * angular_velocity_final_.y() + ALPHA_4 * angular_velocity_initial_.y()) * *time_spline -
                (ALPHA_5 * angular_acceleration_final_.y() - angular_acceleration_initial_.y()) * pow(*time_spline, 2));

      g3_.z() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.z() - orientation_initial_.z()) -
                (ALPHA_3 * angular_velocity_final_.z() + ALPHA_4 * angular_velocity_initial_.z()) * *time_spline -
                (ALPHA_5 * angular_acceleration_final_.z() - angular_acceleration_initial_.z()) * pow(*time_spline, 2));

      ROS_DEBUG("Content of the g3 coefficient: [%f, %f, %f].", g3_[0], g3_[1], g3_[2]);

      g4_.x() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.x() - orientation_final_.x()) +
                (BETA_2 * angular_velocity_final_.x() + BETA_3 * angular_velocity_initial_.x()) * *time_spline +
                (BETA_3 * angular_acceleration_final_.x() - BETA_4 * angular_acceleration_initial_.x()) * pow(*time_spline, 2));

      g4_.y() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.y() - orientation_final_.y()) +
                (BETA_2 * angular_velocity_final_.y() + BETA_3 * angular_velocity_initial_.y()) * *time_spline +
                (BETA_3 * angular_acceleration_final_.y() - BETA_4 * angular_acceleration_initial_.y()) * pow(*time_spline, 2));

      g4_.z() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.z() - orientation_final_.z()) +
                (BETA_2 * angular_velocity_final_.z() + BETA_3 * angular_velocity_initial_.z()) * *time_spline +
                (BETA_3 * angular_acceleration_final_.z() - BETA_4 * angular_acceleration_initial_.z()) * pow(*time_spline, 2));

      ROS_DEBUG("Content of the g4 coefficient: [%f, %f, %f].", g4_[0], g4_[1], g4_[2]);

      g5_.x() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.x() - orientation_initial_.x()) -
                GAMMA_2 * (angular_velocity_final_.x() + angular_velocity_initial_.x()) * *time_spline -
                (angular_acceleration_final_.x() - angular_acceleration_initial_.x()) * pow(*time_spline, 2));

      g5_.y() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.y() - orientation_initial_.y()) -
                GAMMA_2 * (angular_velocity_final_.y() + angular_velocity_initial_.y()) * *time_spline -
                (angular_acceleration_final_.y() - angular_acceleration_initial_.y()) * pow(*time_spline, 2));

      g5_.z() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.z() - orientation_initial_.z()) -
                GAMMA_2 * (angular_velocity_final_.z() + angular_velocity_initial_.z()) * *time_spline -
                (angular_acceleration_final_.z() - angular_acceleration_initial_.z()) * pow(*time_spline, 2));

      ROS_DEBUG("Content of the g5 coefficient: [%f, %f, %f].", g5_[0], g5_[1], g5_[2]);

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

      ROS_DEBUG("Content of the h0 coefficient: [%f, %f, %f].", h0_[0], h0_[1], h0_[2]);
      ROS_DEBUG("Content of the h1 coefficient: [%f, %f, %f].", h1_[0], h1_[1], h1_[2]);
      ROS_DEBUG("Content of the h2 coefficient: [%f, %f, %f].", h2_[0], h2_[1], h2_[2]);
      ROS_DEBUG("Content of the h3 coefficient: [%f, %f, %f].", h3_[0], h3_[1], h3_[2]);
      ROS_DEBUG("Content of the h4 coefficient: [%f, %f, %f].", h4_[0], h4_[1], h4_[2]);

      // The coefficients are used for having the angular acceleration reference trajectory. The polynomial is computed
      // derivating the b one. In other words,
      // 4 * h4 * x^4 + 3 * h3 * x^3 + 2 * h2 * x^2 + h1 * x
      // i3 = 4 * h4 -- i2 = 3 * h3 -- i1 = 2 * h2 - i0 = h1
      i3_ = 4 * h4_;
      i2_ = 3 * h3_;
      i1_ = 2 * h2_;
      i0_ = h1_;

      ROS_DEBUG("Content of the i0 coefficient: [%f, %f, %f].", i0_[0], i0_[1], i0_[2]);
      ROS_DEBUG("Content of the i1 coefficient: [%f, %f, %f].", i1_[0], i1_[1], i1_[2]);
      ROS_DEBUG("Content of the i2 coefficient: [%f, %f, %f].", i2_[0], i2_[1], i2_[2]);
      ROS_DEBUG("Content of the i3 coefficient: [%f, %f, %f].", i3_[0], i3_[1], i3_[2]);

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

      ROS_DEBUG("x Trajectory: %f, y Trajectory: %f, z Trajectory: %f, w Trajectory: %f", *x, *y, *z, *w);
    }

   void SplineTrajectoryGenerator::InitializeParams(){

     ros::NodeHandle pnh("~");

     // Parameters reading from rosparam.
     GetRosParameter(pnh, "time_final/time", time_final_, &time_final_);
     GetRosParameter(pnh, "position_initial/x", position_initial_.x(), &position_initial_.x());
     GetRosParameter(pnh, "position_initial/y", position_initial_.y(), &position_initial_.y());
     GetRosParameter(pnh, "position_initial/z", position_initial_.z(), &position_initial_.z());

     GetRosParameter(pnh, "position_final/x", position_final_.x(), &position_final_.x());
     GetRosParameter(pnh, "position_final/y", position_final_.y(), &position_final_.y());
     GetRosParameter(pnh, "position_final/z", position_final_.z(), &position_final_.z());

     ROS_DEBUG("[Library] Position final x: %f, y: %f, z :%f", position_final_.x(), position_final_.y(),
       position_final_.z());

     GetRosParameter(pnh, "velocity_initial/x", velocity_initial_.x(), &velocity_initial_.x());
     GetRosParameter(pnh, "velocity_initial/y", velocity_initial_.y(), &velocity_initial_.y());
     GetRosParameter(pnh, "velocity_initial/z", velocity_initial_.z(), &velocity_initial_.z());

     GetRosParameter(pnh, "velocity_final/x", velocity_final_.x(), &velocity_final_.x());
     GetRosParameter(pnh, "velocity_final/y", velocity_final_.y(), &velocity_final_.y());
     GetRosParameter(pnh, "velocity_final/z", velocity_final_.z(), &velocity_final_.z());

     GetRosParameter(pnh, "acceleration_initial/x", acceleration_initial_.x(), &acceleration_initial_.x());
     GetRosParameter(pnh, "acceleration_initial/y", acceleration_initial_.y(), &acceleration_initial_.y());
     GetRosParameter(pnh, "acceleration_initial/z", acceleration_initial_.z(), &acceleration_initial_.z());

     GetRosParameter(pnh, "acceleration_final/x", acceleration_final_.x(), &acceleration_final_.x());
     GetRosParameter(pnh, "acceleration_final/y", acceleration_final_.y(), &acceleration_final_.y());
     GetRosParameter(pnh, "acceleration_final/z", acceleration_final_.z(), &acceleration_final_.z());

     GetRosParameter(pnh, "orientation_initial/roll", orientation_initial_.x(), &orientation_initial_.x());
     GetRosParameter(pnh, "orientation_initial/pitch", orientation_initial_.y(), &orientation_initial_.y());
     GetRosParameter(pnh, "orientation_initial/yaw", orientation_initial_.z(), &orientation_initial_.z());

     GetRosParameter(pnh, "orientation_final/roll", orientation_final_.x(), &orientation_final_.x());
     GetRosParameter(pnh, "orientation_final/pitch", orientation_final_.y(), &orientation_final_.y());
     GetRosParameter(pnh, "orientation_final/yaw", orientation_final_.z(), &orientation_final_.z());

     GetRosParameter(pnh, "angularRate_initial/roll", angular_velocity_initial_.x(), &angular_velocity_initial_.x());
     GetRosParameter(pnh, "angularRate_initial/pitch", angular_velocity_initial_.y(), &angular_velocity_initial_.y());
     GetRosParameter(pnh, "angularRate_initial/yaw", angular_velocity_initial_.z(), &angular_velocity_initial_.z());

     GetRosParameter(pnh, "angularRate_final/roll", angular_velocity_final_.x(), &angular_velocity_final_.x());
     GetRosParameter(pnh, "angularRate_final/pitch", angular_velocity_final_.y(), &angular_velocity_final_.y());
     GetRosParameter(pnh, "angularRate_final/yaw", angular_velocity_final_.z(), &angular_velocity_final_.z());

     GetRosParameter(pnh, "angularDoubleRate_initial/roll", angular_acceleration_initial_.x(), &angular_acceleration_initial_.x());
     GetRosParameter(pnh, "angularDoubleRate_initial/pitch", angular_acceleration_initial_.y(), &angular_acceleration_initial_.y());
     GetRosParameter(pnh, "angularDoubleRate_initial/yaw", angular_acceleration_initial_.z(), &angular_acceleration_initial_.z());

     GetRosParameter(pnh, "angularDoubleRate_final/roll", angular_acceleration_final_.x(), &angular_acceleration_final_.x());
     GetRosParameter(pnh, "angularDoubleRate_final/pitch", angular_acceleration_final_.y(), &angular_acceleration_final_.y());
     GetRosParameter(pnh, "angularDoubleRate_final/yaw", angular_acceleration_final_.z(), &angular_acceleration_final_.z());

   }

   template<typename T> inline void SplineTrajectoryGenerator::GetRosParameterHovering(const ros::NodeHandle& nh,
                                                            const std::string& key,
                                                            const T& default_value,
                                                            T* value) {

     ROS_ASSERT(value != nullptr);
     bool have_parameter = nh.getParam(key, *value);
     if (!have_parameter) {
       ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                       << "/" << key << ", setting to default: " << default_value);
       *value = default_value;
     }
   }

}
