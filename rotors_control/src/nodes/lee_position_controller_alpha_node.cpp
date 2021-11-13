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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_position_controller_alpha_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerAlphaNode::LeePositionControllerAlphaNode() {
  InitializeParams(); // invoke the InitializeParams function in this file

  ros::NodeHandle nh;

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::DRONE_STATE, 1,
      &LeePositionControllerAlphaNode::ReferenceTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &LeePositionControllerAlphaNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

}

LeePositionControllerAlphaNode::~LeePositionControllerAlphaNode() { }

void LeePositionControllerAlphaNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "Ts",
                  lee_position_controller_alpha_.Ts_,
                  &lee_position_controller_alpha_.Ts_);
  GetRosParameter(pnh, "gravity",
                  lee_position_controller_alpha_.gravity_,
                  &lee_position_controller_alpha_.gravity_);
  GetRosParameter(pnh, "mass",
                  lee_position_controller_alpha_.mass_,
                  &lee_position_controller_alpha_.mass_);
  GetRosParameter(pnh, "Jxx",
                  lee_position_controller_alpha_.Jxx_,
                  &lee_position_controller_alpha_.Jxx_);
  GetRosParameter(pnh, "Jyy",
                  lee_position_controller_alpha_.Jyy_,
                  &lee_position_controller_alpha_.Jyy_);
  GetRosParameter(pnh, "Jzz",
                  lee_position_controller_alpha_.Jzz_,
                  &lee_position_controller_alpha_.Jzz_);
  GetRosParameter(pnh, "d",
                  lee_position_controller_alpha_.d_,
                  &lee_position_controller_alpha_.d_);
  GetRosParameter(pnh, "maxRotorsVelocity",
                  lee_position_controller_alpha_.maxRotorsVelocity_,
                  &lee_position_controller_alpha_.maxRotorsVelocity_);
  GetRosParameter(pnh, "c_motor",
                  lee_position_controller_alpha_.c_motor_,
                  &lee_position_controller_alpha_.c_motor_);
  GetRosParameter(pnh, "c_tauf",
                  lee_position_controller_alpha_.c_tauf_,
                  &lee_position_controller_alpha_.c_tauf_);
  GetRosParameter(pnh, "tau",
                  lee_position_controller_alpha_.tau_,
                  &lee_position_controller_alpha_.tau_);
  GetRosParameter(pnh, "kx",
                  lee_position_controller_alpha_.kx_,
                  &lee_position_controller_alpha_.kx_);
  GetRosParameter(pnh, "kv",
                  lee_position_controller_alpha_.kv_,
                  &lee_position_controller_alpha_.kv_);
  GetRosParameter(pnh, "kR",
                  lee_position_controller_alpha_.kR_,
                  &lee_position_controller_alpha_.kR_);
  GetRosParameter(pnh, "kOmega",
                  lee_position_controller_alpha_.kOmega_,
                  &lee_position_controller_alpha_.kOmega_);

  Ts_ = lee_position_controller_alpha_.Ts_;// for main ros rate
  lee_position_controller_alpha_.InitializeParameters();
}

void LeePositionControllerAlphaNode::Publish() {
}

void LeePositionControllerAlphaNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("LeePositionControllerAlpha got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_alpha_.SetOdometry(odometry);

  Eigen::Vector4d ref_rotor_velocities;
  lee_position_controller_alpha_.CalculateRotorVelocities(&ref_rotor_velocities);

  ROS_DEBUG("Rotor velocities [%f] [%f] [%f] [%f]", ref_rotor_velocities[0], ref_rotor_velocities[1],
    ref_rotor_velocities[2], ref_rotor_velocities[3]);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

void LeePositionControllerAlphaNode::ReferenceTrajectoryCallback(const mav_msgs::DroneState& drone_state_msg){
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  if(drone_state_msg.position.z <= 0){
    ROS_WARN_STREAM("Got ReferenceTrajectoryCallback message, but message has no points.");
    return;
    }

  // We can trigger the first command immediately.
  mav_msgs::EigenDroneState eigen_reference;
  mav_msgs::eigenDroneStateFromMsg(drone_state_msg, &eigen_reference);

  // We can trigger the first command immediately.
  lee_position_controller_alpha_.SetTrajectoryPoint(eigen_reference);

  ROS_DEBUG("Drone desired position [x_d: %f, y_d: %f, z_d: %f]", eigen_reference.position_W[0],
          eigen_reference.position_W[1], eigen_reference.position_W[2]);

  ROS_DEBUG("Drone desired attitude in unit quaternion [w: %f, x: %f, y: %f, z: %f]", eigen_reference.orientation_W_B.w(),
          eigen_reference.orientation_W_B.x(), eigen_reference.orientation_W_B.y(), eigen_reference.orientation_W_B.z());

  if (drone_state_msg.position.z > 0) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("LeePositionControllerAlpha got first ReferenceTrajectoryCallback message.");
  }

}

} // ends rotors_control, the namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_alpha_node");

  rotors_control::LeePositionControllerAlphaNode lee_position_controller_alpha_node;

  ros::Rate loop_rate(1/lee_position_controller_alpha_node.Ts_);

  loop_rate.sleep();
  ros::spinOnce();

  return 0;
}
