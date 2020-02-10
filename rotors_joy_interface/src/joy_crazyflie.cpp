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


#include "rotors_joy_interface/joy_crazyflie.h"

#include <mav_msgs/default_topics.h>

#define MAX_THRUST 8162.0 // propeller angular velocities [PWM]
#define MAX_ROLL 30.0 * M_PI / 180.0 // [rad]
#define MAX_PITCH 30.0 * M_PI / 180.0 // [rad]
#define MAX_YAWRATE 200.0 * M_PI / 180.0 // [rad/s]

Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrustCrazyflie> (
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust = 0;
  current_yaw_vel_ = 0;

  pnh.param("axis_roll_", axes_.roll, 0);
  pnh.param("axis_pitch_", axes_.pitch, 1);
  pnh.param("axis_thrust_", axes_.thrust, 3);
  pnh.param("axis_yaw_rate_", axes_.yaw_rate, 2);

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);

  pnh.param("max_roll", max_.roll, MAX_ROLL);
  pnh.param("max_pitch", max_.pitch, MAX_PITCH);
  pnh.param("max_yaw_rate", max_.rate_yaw, MAX_YAWRATE);
  pnh.param("max_thrust", max_.thrust, MAX_THRUST);

  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable, 8);
  pnh.param("button_ctrl_mode_", buttons_.ctrl_mode, 9);
  pnh.param("button_takeoff_", buttons_.takeoff, 10);
  pnh.param("button_land_", buttons_.land, 11);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust = 0;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;

  //Set pitch and roll
  control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
  control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;

  //Set yaw rate
  if (msg->axes[axes_.yaw_rate] > 0) {
    current_yaw_vel_ = max_.rate_yaw;
  }
  else if (msg->axes[axes_.yaw_rate] < 0) {
    current_yaw_vel_ = -max_.rate_yaw;
  }
  else {
    current_yaw_vel_ = 0;
  }
  control_msg_.yaw_rate = current_yaw_vel_;

  //Set thrust
  if(msg->axes[axes_.thrust] > 0)
    control_msg_.thrust = max_.thrust;
  else
    control_msg_.thrust = 0;


  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface_crazyflie");
  Joy joy;

  ros::spin();

  return 0;
}
