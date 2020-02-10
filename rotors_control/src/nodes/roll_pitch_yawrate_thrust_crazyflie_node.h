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

#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CRAZYFLIE_NODE_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CRAZYFLIE_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrustCrazyflie.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/roll_pitch_yawrate_thrust_crazyflie.h"

namespace rotors_control {

class RollPitchYawrateThrustCrazyflieNode {
 public:
  RollPitchYawrateThrustCrazyflieNode();
  ~RollPitchYawrateThrustCrazyflieNode();

  void InitializeParams();
  void Publish();

 private:

  RollPitchYawrateThrustControllerCrazyflie roll_pitch_yawrate_thrust_crazyflie_;

  // subscribers
  ros::Subscriber cmd_roll_pitch_yawrate_thrust_sub_;
  ros::Subscriber odometry_sub_;

  std::string namespace_;

  // publisher
  ros::Publisher motor_velocity_reference_pub_;

  void RollPitchYawrateThrustCallback(
      const mav_msgs::RollPitchYawrateThrustCrazyflieConstPtr& roll_pitch_yawrate_thrust_crazyflie_reference_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CRAZYFLIE_NODE_H
