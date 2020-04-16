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
#include "rotors_gazebo/hovering_example_spline.h"

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

  HoveringExampleSpline::HoveringExampleSpline(){

    // Parameters initializion
    a0_.setZero(); a1_.setZero(); a2_.setZero(); a3_.setZero(); a4_.setZero(); a5_.setZero();
    b0_.setZero(); b1_.setZero(); b2_.setZero(); b3_.setZero(); b4_.setZero(); b5_.setZero();
    c0_.setZero(); c1_.setZero(); c2_.setZero(); c3_.setZero(); c4_.setZero(); c5_.setZero();

    g0_.setZero(); g1_.setZero(); g2_.setZero(); g3_.setZero(); g4_.setZero(); g5_.setZero();
    h0_.setZero(); h1_.setZero(); h2_.setZero(); h3_.setZero(); h4_.setZero(); h5_.setZero();

  }

  HoveringExampleSpline::~HoveringExampleSpline(){}

  void HoveringExampleSpline::TrajectoryCallback(mav_msgs::EigenDroneState* odometry, double* time_final, double* time_init) {
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

    // Desidered Position
    double first, second, third;
    first = a0_.x() * pow(time_spline, 5) + a1_.x() * pow(time_spline, 4) + a2_.x() * pow(time_spline, 3) + a3_.x() * pow(time_spline, 2)
                + a4_.x() * time_spline + a5_.x();
    second = a0_.y() * pow(time_spline, 5) + a1_.y() * pow(time_spline, 4) + a2_.y() * pow(time_spline, 3) + a3_.y() * pow(time_spline, 2)
                + a4_.y() * time_spline + a5_.y();
    third = a0_.z() * pow(time_spline, 5) + a1_.z() * pow(time_spline, 4) + a2_.z() * pow(time_spline, 3) + a3_.z() * pow(time_spline, 2)
                + a4_.z() * time_spline + a5_.z();

    ROS_DEBUG("Publishing position spline parameters along x: [%f, %f, %f, %f, %f, %f].", a0_.x(), a1_.x(), a2_.x(), a3_.x(), a4_.x(), a5_.x());
    ROS_DEBUG("Publishing position spline parameters along y: [%f, %f, %f, %f, %f, %f].", a0_.y(), a1_.y(), a2_.y(), a3_.y(), a4_.y(), a5_.y());
    ROS_DEBUG("Publishing position spline parameters along z: [%f, %f, %f, %f, %f, %f].", a0_.z(), a1_.z(), a2_.z(), a3_.z(), a4_.z(), a5_.z());

    odometry->position_W = Eigen::Vector3f(first, second, third);

    ROS_DEBUG("Publishing position waypoint: [%f, %f, %f].", first, second, third);

    // Desidered Linear Velocity
    first = b0_.x() * pow(time_spline, 5) + b1_.x() * pow(time_spline, 4) + b2_.x() * pow(time_spline, 3) + b3_.x() * pow(time_spline, 2)
                + b4_.x() * time_spline + b5_.x();
    second = b0_.y() * pow(time_spline, 5) + b1_.y() * pow(time_spline, 4) + b2_.y() * pow(time_spline, 3) + b3_.y() * pow(time_spline, 2)
                + b4_.y() * time_spline + b5_.y();
    third = b0_.z() * pow(time_spline, 5) + b1_.z() * pow(time_spline, 4) + b2_.z() * pow(time_spline, 3) + b3_.z() * pow(time_spline, 2)
                + b4_.z() * time_spline + b5_.z();

    ROS_DEBUG("Publishing velocity spline parameters along x: [%f, %f, %f, %f, %f, %f].", b0_.x(), b1_.x(), b2_.x(), b3_.x(), b4_.x(), b5_.x());
    ROS_DEBUG("Publishing velocity spline parameters along y: [%f, %f, %f, %f, %f, %f].", b0_.y(), b1_.y(), b2_.y(), b3_.y(), b4_.y(), b5_.y());
    ROS_DEBUG("Publishing velocity spline parameters along z: [%f, %f, %f, %f, %f, %f].", b0_.z(), b1_.z(), b2_.z(), b3_.z(), b4_.z(), b5_.z());

    odometry->velocity = Eigen::Vector3f(first, second, third);

    ROS_DEBUG("Publishing velocity waypoint: [%f, %f, %f].", first, second, third);

    // Desidered Acceleration
    first = c0_.x() * pow(time_spline, 5) + c1_.x() * pow(time_spline, 4) + c2_.x() * pow(time_spline, 3) + c3_.x() * pow(time_spline, 2)
                + c4_.x() * time_spline + c5_.x();
    second = c0_.y() * pow(time_spline, 5) + c1_.y() * pow(time_spline, 4) + c2_.y() * pow(time_spline, 3) + c3_.y() * pow(time_spline, 2)
                + c4_.y() * time_spline + c5_.y();
    third = c0_.z() * pow(time_spline, 5) + c1_.z() * pow(time_spline, 4) + c2_.z() * pow(time_spline, 3) + c3_.z() * pow(time_spline, 2)
                + c4_.z() * time_spline + c5_.z();

    odometry->acceleration = Eigen::Vector3f(first, second, third);

    ROS_DEBUG("Publishing acceleration waypoint: [%f, %f, %f].", first, second, third);

    // Desidred Attitute
    rollDesRad_ = g0_.x() * pow(time_spline, 5) + g1_.x() * pow(time_spline, 4) + g2_.x() * pow(time_spline, 3) + g3_.x() * pow(time_spline, 2)
                + g4_.x() * time_spline + g5_.x();
    pitchDesRad_ = g0_.y() * pow(time_spline, 5) + g1_.y() * pow(time_spline, 4) + g2_.y() * pow(time_spline, 3) + g3_.y() * pow(time_spline, 2)
                + g4_.y() * time_spline + g5_.y();
    yawDesRad_ = g0_.z() * pow(time_spline, 5) + g1_.z() * pow(time_spline, 4) + g2_.z() * pow(time_spline, 3) + g3_.z() * pow(time_spline, 2)
                + g4_.z() * time_spline + g5_.z();

    // Converts radians in quaternions
    double x, y, z, w;
    Euler2QuaternionCommandTrajectory(&x, &y, &z, &w);
    odometry->orientation_W_B = Eigen::Quaterniond(w, x, y, z);

    ROS_DEBUG("Publishing attitude waypoint: [%f, %f, %f].", rollDesRad_, pitchDesRad_, yawDesRad_);

    // Desidered Angular Velocity
    first = h0_.x() * pow(time_spline, 5) + h1_.x() * pow(time_spline, 4) + h2_.x() * pow(time_spline, 3) + h3_.x() * pow(time_spline, 2)
                + h4_.x() * time_spline + h5_.x();
    second = h0_.y() * pow(time_spline, 5) + h1_.y() * pow(time_spline, 4) + h2_.y() * pow(time_spline, 3) + h3_.y() * pow(time_spline, 2)
                + h4_.y() * time_spline + h5_.y();
    third = h0_.z() * pow(time_spline, 5) + h1_.z() * pow(time_spline, 4) + h2_.z() * pow(time_spline, 3) + h3_.z() * pow(time_spline, 2)
                + h4_.z() * time_spline + h5_.z();

    double first_B, second_B, third_B;
    first_B = first - sin(pitchDesRad_)*third;
    second_B = cos(rollDesRad_)*second + sin(rollDesRad_)*cos(pitchDesRad_)*third;
    third_B = -sin(rollDesRad_)*second + cos(rollDesRad_)*cos(pitchDesRad_)*third;

    odometry->angular_velocity_B = Eigen::Vector3f(first_B, second_B, third_B);

    ROS_DEBUG("Publishing angular velocity waypoint: [%f, %f, %f].", first, second, third);

  }

  void HoveringExampleSpline::ComputeSplineParameters(double* time_spline){
      assert(time_spline);

      // Parameters used for the position and its derivatives
      a0_ = position_initial_;

      ROS_DEBUG("Publishing position initial: [%f, %f, %f].", position_initial_[0], position_initial_[1], position_initial_[2]);

      a1_ = velocity_initial_;

      a2_.x() = acceleration_initial_.x() * DELTA_1;
      a2_.y() = acceleration_initial_.y() * DELTA_1;
      a2_.z() = acceleration_initial_.z() * DELTA_1;

      ROS_DEBUG("Publishing acceleration initial: [%f, %f, %f].", acceleration_initial_[0], acceleration_initial_[1], acceleration_initial_[2]);

      a3_.x() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.x() - position_initial_.x()) - (ALPHA_3 * velocity_final_.x() + ALPHA_4 * velocity_initial_.x()) * *time_spline
                - (ALPHA_5 * acceleration_final_.x() - acceleration_initial_.x()) * pow(*time_spline, 2));
      a3_.y() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.y() - position_initial_.y()) - (ALPHA_3 * velocity_final_.y() + ALPHA_4 * velocity_initial_.y()) * *time_spline
                - (ALPHA_5 * acceleration_final_.y() - acceleration_initial_.y()) * pow(*time_spline, 2));
      a3_.z() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (position_final_.z() - position_initial_.z()) - (ALPHA_3 * velocity_final_.z() + ALPHA_4 * velocity_initial_.z()) * *time_spline
                - (ALPHA_5 * acceleration_final_.z() - acceleration_initial_.z()) * pow(*time_spline, 2));

      a4_.x() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.x() - position_final_.x()) + (BETA_2 * velocity_final_.x() + BETA_3 * velocity_initial_.x()) * *time_spline
                + (BETA_4 * acceleration_final_.x() - 2 * acceleration_initial_.x()) * pow(*time_spline, 2));
      a4_.y() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.y() - position_final_.y()) + (BETA_2 * velocity_final_.y() + BETA_3 * velocity_initial_.y()) * *time_spline
                + (BETA_4 * acceleration_final_.y() - 2 * acceleration_initial_.y()) * pow(*time_spline, 2));
      a4_.z() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (position_initial_.z() - position_final_.z()) + (BETA_2 * velocity_final_.z() + BETA_3 * velocity_initial_.z()) * *time_spline
                + (BETA_4 * acceleration_final_.z() - 2 * acceleration_initial_.z()) * pow(*time_spline, 2));

      a5_.x() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.x() - position_initial_.x()) - GAMMA_2 * (velocity_final_.x() + velocity_initial_.x()) * *time_spline
                - (acceleration_final_.x() - acceleration_initial_.x()) * pow(*time_spline, 2));
      a5_.y() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.y() - position_initial_.y()) - GAMMA_2 * (velocity_final_.y() + velocity_initial_.y()) * *time_spline
                - (acceleration_final_.y() - acceleration_initial_.y()) * pow(*time_spline, 2));
      a5_.z() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (position_final_.z() - position_initial_.z()) - GAMMA_2 * (velocity_final_.z() + velocity_initial_.z()) * *time_spline
                - (acceleration_final_.z() - acceleration_initial_.z()) * pow(*time_spline, 2));
      //b
      b0_.x() = 0;
      b0_.y() = 0;
      b0_.z() = 0;

      b1_.x() = 5*a0_.x();
      b1_.y() = 5*a0_.y();
      b1_.z() = 5*a0_.z();

      b2_.x() = 4*a1_.x();
      b2_.y() = 4*a1_.y();
      b2_.z() = 4*a1_.z();

      b3_.x() = 3*a2_.x();
      b3_.y() = 3*a2_.y();
      b3_.z() = 3*a2_.z();

      b4_.x() = 2*a3_.x();
      b4_.y() = 2*a3_.y();
      b4_.z() = 2*a3_.z();

      b5_ = a4_;

      //c
      c0_.x() = 0;
      c0_.y() = 0;
      c0_.z() = 0;

      c1_.x() = 0;
      c1_.y() = 0;
      c1_.z() = 0;

      c2_.x() = 4*b1_.x();
      c2_.y() = 4*b1_.y();
      c2_.z() = 4*b1_.z();

      c3_.x() = 3*b2_.x();
      c3_.y() = 3*b2_.y();
      c3_.z() = 3*b2_.z();

      c4_.x() = 2*b3_.x();
      c4_.y() = 2*b3_.y();
      c4_.z() = 2*b3_.z();

      c5_ = b4_;

      // Parameters used for the orientation and its derivatives
      g0_ = orientation_initial_;

      g1_ = angular_velocity_initial_;

      g2_.x() = angular_acceleration_initial_.x() * DELTA_1;
      g2_.y() = angular_acceleration_initial_.y() * DELTA_1;
      g2_.z() = angular_acceleration_initial_.z() * DELTA_1;

      g3_.x() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.x() - orientation_initial_.x()) - (ALPHA_3 * angular_velocity_final_.x() + ALPHA_4 *
                angular_velocity_initial_.x()) * *time_spline - (ALPHA_5 * angular_acceleration_final_.x() - angular_acceleration_initial_.x()) * pow(*time_spline, 2));
      g3_.y() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.y() - orientation_initial_.y()) - (ALPHA_3 * angular_velocity_final_.y() + ALPHA_4 *
                angular_velocity_initial_.y()) * *time_spline - (ALPHA_5 * angular_acceleration_final_.y() - angular_acceleration_initial_.y()) * pow(*time_spline, 2));
      g3_.z() = (1/(ALPHA_1*pow(*time_spline,3))) * (ALPHA_2 * (orientation_final_.z() - orientation_initial_.z()) - (ALPHA_3 * angular_velocity_final_.z() + ALPHA_4 *
                angular_velocity_initial_.z()) * *time_spline - (ALPHA_5 * angular_acceleration_final_.z() - angular_acceleration_initial_.z()) * pow(*time_spline, 2));

      g4_.x() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.x() - orientation_final_.x()) + (BETA_2 * angular_velocity_final_.x() + BETA_3 *
                angular_velocity_initial_.x()) * *time_spline + (BETA_4 * angular_acceleration_final_.x() - 2 * angular_acceleration_initial_.x()) * pow(*time_spline, 2));
      g4_.y() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.y() - orientation_final_.y()) + (BETA_2 * angular_velocity_final_.y() + BETA_3 *
                angular_velocity_initial_.y()) * *time_spline + (BETA_4 * angular_acceleration_final_.y() - 2 * angular_acceleration_initial_.y()) * pow(*time_spline, 2));
      g4_.z() = (1/(ALPHA_1*pow(*time_spline,4))) * (BETA_1 * (orientation_initial_.z() - orientation_final_.z()) + (BETA_2 * angular_velocity_final_.z() + BETA_3 *
                angular_velocity_initial_.z()) * *time_spline + (BETA_4 * angular_acceleration_final_.z() - 2 * angular_acceleration_initial_.z()) * pow(*time_spline, 2));

      g5_.x() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.x() - orientation_initial_.x()) - GAMMA_2 * (angular_velocity_final_.x() +
                angular_velocity_initial_.x()) * *time_spline - (angular_acceleration_final_.x() - angular_acceleration_initial_.x()) * pow(*time_spline, 2));
      g5_.y() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.y() - orientation_initial_.y()) - GAMMA_2 * (angular_velocity_final_.y() +
                angular_velocity_initial_.y()) * *time_spline - (angular_acceleration_final_.y() - angular_acceleration_initial_.y()) * pow(*time_spline, 2));
      g5_.z() = (1/(ALPHA_1*pow(*time_spline,5))) * (GAMMA_1 * (orientation_final_.z() - orientation_initial_.z()) - GAMMA_2 * (angular_velocity_final_.z() +
                angular_velocity_initial_.z()) * *time_spline - (angular_acceleration_final_.z() - angular_acceleration_initial_.z()) * pow(*time_spline, 2));

      //h
      h0_.x() = 0;
      h0_.y() = 0;
      h0_.z() = 0;

      h1_.x() = 5*g0_.x();
      h1_.y() = 5*g0_.y();
      h1_.z() = 5*g0_.z();

      h2_.x() = 4*g1_.x();
      h2_.y() = 4*g1_.y();
      h2_.z() = 4*g1_.z();

      h3_.x() = 3*g2_.x();
      h3_.y() = 3*g2_.y();
      h3_.z() = 3*g2_.z();

      h4_.x() = 2*g3_.x();
      h4_.y() = 2*g3_.y();
      h4_.z() = 2*g3_.z();

      h5_ = g4_;

  }

  void HoveringExampleSpline::Euler2QuaternionCommandTrajectory(double* x, double* y, double* z, double* w) const {
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

   void HoveringExampleSpline::InitializeParams(){

     ros::NodeHandle pnh("~");

     // Parameters reading from rosparam.
     GetRosParameter(pnh, "time_final/time", time_final_, &time_final_);
     GetRosParameter(pnh, "position_initial/x", position_initial_.x(), &position_initial_.x());
     GetRosParameter(pnh, "position_initial/y", position_initial_.y(), &position_initial_.y());
     GetRosParameter(pnh, "position_initial/z", position_initial_.z(), &position_initial_.z());

     GetRosParameter(pnh, "position_final/x", position_final_.x(), &position_final_.x());
     GetRosParameter(pnh, "position_final/y", position_final_.y(), &position_final_.y());
     GetRosParameter(pnh, "position_final/z", position_final_.z(), &position_final_.z());

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

   template<typename T> inline void HoveringExampleSpline::GetRosParameterHovering(const ros::NodeHandle& nh,
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


int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example_spline");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<mav_msgs::DroneState>(
          mav_msgs::default_topics::DRONE_STATE, 10);
  ROS_INFO("Started hovering example with spline.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Trajectory message
  mav_msgs::DroneState trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::EigenDroneState eigen_reference;
  rotors_gazebo::HoveringExampleSpline hovering_example_spline_generator;

  Eigen::Vector3f check_position_final;
  hovering_example_spline_generator.GetRosParameterHovering(nh_private, "position_final/x", check_position_final.x(), &check_position_final.x());
  hovering_example_spline_generator.GetRosParameterHovering(nh_private, "position_final/y", check_position_final.y(), &check_position_final.y());
  hovering_example_spline_generator.GetRosParameterHovering(nh_private, "position_final/z", check_position_final.z(), &check_position_final.z());

  // Wait for 5 seconds to let the Gazebo GUI show up.
  if (ros::Time::now().toSec() < START_SIMULATION_TIME){
    ros::Duration(START_SIMULATION_TIME).sleep();
    hovering_example_spline_generator.InitializeParams();
  }

  double initial_time, final_time;
  initial_time = START_SIMULATION_TIME;

  // Publish the trajectory values until the final values is reached 
  while(true){
    final_time = ros::Time::now().toSec();
    hovering_example_spline_generator.TrajectoryCallback(&eigen_reference, &final_time, &initial_time);

    mav_msgs::eigenDroneFromStateToMsg(&eigen_reference, trajectory_msg);
    trajectory_pub.publish(trajectory_msg);

    ROS_DEBUG("Publishing waypoint from msg: [%f, %f, %f].", trajectory_msg.position.x, trajectory_msg.position.y, trajectory_msg.position.z);
    ROS_DEBUG("Publishing waypoint: [%f, %f, %f].", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2]);

    ros::Duration(SAMPLING_TIME).sleep();

    if(eigen_reference.position_W[0] >= check_position_final[0] && eigen_reference.position_W[1] >= check_position_final[1] && eigen_reference.position_W[2] >= check_position_final[2])
      break;
  }

  ros::spin();

  return 0;
}
