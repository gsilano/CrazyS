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

#include "roll_pitch_yawrate_thrust_crazyflie_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include "rotors_control/parameters_ros.h"

namespace rotors_control {

RollPitchYawrateThrustCrazyflieNode::RollPitchYawrateThrustCrazyflieNode() {

  ROS_INFO_ONCE("Started position controller with the joystick");

  InitializeParams();

  ros::NodeHandle nh;

  // rotors_control/include/rotors_control/common.h
  cmd_roll_pitch_yawrate_thrust_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
                                     &RollPitchYawrateThrustCrazyflieNode::RollPitchYawrateThrustCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &RollPitchYawrateThrustCrazyflieNode::OdometryCallback, this);

  // rotors_control/include/rotors_control/common.h
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
}

RollPitchYawrateThrustCrazyflieNode::~RollPitchYawrateThrustCrazyflieNode() { }

void RollPitchYawrateThrustCrazyflieNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_kp_.x(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_kp_.y(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_ki_.x(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/phi",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_ki_.y(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.attitude_gain_ki_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_kp_.x(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_kp_.y(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_kp_.z(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_ki_.x(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_ki_.y(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_ki_.z(),
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(pnh, "thrust_percentage/p",
                  roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.thrust_percentage_,
                  &roll_pitch_yawrate_thrust_crazyflie_.controller_parameters_.thrust_percentage_);

  roll_pitch_yawrate_thrust_crazyflie_.SetControllerGains();

}

void RollPitchYawrateThrustCrazyflieNode::Publish() {
}

void RollPitchYawrateThrustCrazyflieNode::RollPitchYawrateThrustCallback(const mav_msgs::RollPitchYawrateThrustCrazyflieConstPtr& roll_pitch_yawrate_thrust_crazyflie_reference_msg) {

  mav_msgs::EigenRollPitchYawrateThrustCrazyflie roll_pitch_yawrate_thrust;
  mav_msgs::eigenRollPitchYawrateThrustCrazyflieFromMsg(*roll_pitch_yawrate_thrust_crazyflie_reference_msg, &roll_pitch_yawrate_thrust);
  roll_pitch_yawrate_thrust_crazyflie_.SetRollPitchYawrateThrust(roll_pitch_yawrate_thrust);

}


void RollPitchYawrateThrustCrazyflieNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("RollPitchYawrateThrustCrazyflie got first odometry message.");

    //This functions allows us to put the odometry message into the odometry variable--> _position, _orientation,_velocit_body,
    //_angular_velocity
    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    roll_pitch_yawrate_thrust_crazyflie_.SetOdometry(odometry);

    Eigen::Vector4d ref_rotor_velocities;
    roll_pitch_yawrate_thrust_crazyflie_.CalculateRotorVelocities(&ref_rotor_velocities);

    //creating a new mav message. actuator_msg is used to send the velocities of the propellers.
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    //we use clear because we later want to be sure that we used the previously calculated velocity.
    actuator_msg->angular_velocities.clear();
    //for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
       actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    actuator_msg->header.stamp = odometry_msg->header.stamp;

    motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "roll_pitch_yawrate_thrust_crazyflie_node");

  ros::NodeHandle nh2;

  rotors_control::RollPitchYawrateThrustCrazyflieNode roll_pitch_yawrate_thrust_crazyflie_node;

  ros::spin();

  return 0;
}
