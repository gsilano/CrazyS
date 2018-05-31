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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h> 
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "position_controller_node_with_stateEstimator.h"

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/complementary_filter_crazyflie2.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] */
#define SAMPLING_TIME  0.01       /* SAMPLING TIME [s] */

namespace rotors_control {

PositionControllerNode::PositionControllerNode() {

    ROS_INFO_ONCE("Started position controller with state estimator");

    InitializeParams();

    ros::NodeHandle nh;

    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PositionControllerNode::OdometryCallback, this);

    imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1, &PositionControllerNode::IMUCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    timer_Attitude_ = n_.createTimer(ros::Duration(ATTITUDE_UPDATE_DT), &PositionControllerNode::CallbackAttitudeEstimation, this, false, true);

    timer_highLevelControl = n_.createTimer(ros::Duration(SAMPLING_TIME), &PositionControllerNode::CallbackHightLevelControl, this, false, true);

    timer_IMUUpdate = n_.createTimer(ros::Duration(RATE_UPDATE_DT), &PositionControllerNode::CallbackIMUUpdate, this, false, true);

}

PositionControllerNode::~PositionControllerNode(){}

void PositionControllerNode::CallbackAttitudeEstimation(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            position_controller_.CallbackAttitudeEstimation();

}

void PositionControllerNode::CallbackHightLevelControl(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            position_controller_.CallbackHightLevelControl();

}

void PositionControllerNode::CallbackIMUUpdate(const ros::TimerEvent& event){

    position_controller_.SetSensorData(sensors_);

    ROS_INFO_ONCE("IMU Message sent to position controller");

    if (waypointHasBeenPublished_){

	    Eigen::Vector4d ref_rotor_velocities;
	    position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

	    //creating a new mav message. actuator_msg is used to send the velocities of the propellers.  
	    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	    //we use clear because we later want to be sure that we used the previously calculated velocity.
	    actuator_msg->angular_velocities.clear();
	    //for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	    for (int i = 0; i < ref_rotor_velocities.size(); i++)
	       actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	    actuator_msg->header.stamp = imu_msg_head_stamp_;
	    
	    motor_velocity_reference_pub_.publish(actuator_msg);

    }

}


void PositionControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO("PositionController got first MultiDOFJointTrajectory message.");
  }
}


void PositionControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "xy_gain_kp/x",
                  position_controller_.controller_parameters_.xy_gain_kp_.x(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(pnh, "xy_gain_kp/y",
                  position_controller_.controller_parameters_.xy_gain_kp_.y(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(pnh, "xy_gain_ki/x",
                  position_controller_.controller_parameters_.xy_gain_ki_.x(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.x());
  GetRosParameter(pnh, "xy_gain_ki/y",
                  position_controller_.controller_parameters_.xy_gain_ki_.y(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.y());

  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  position_controller_.controller_parameters_.attitude_gain_ki_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/phi",
                  position_controller_.controller_parameters_.attitude_gain_ki_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  position_controller_.controller_parameters_.rate_gain_kp_.x(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  position_controller_.controller_parameters_.rate_gain_kp_.y(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  position_controller_.controller_parameters_.rate_gain_kp_.z(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  position_controller_.controller_parameters_.rate_gain_ki_.x(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  position_controller_.controller_parameters_.rate_gain_ki_.y(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  position_controller_.controller_parameters_.rate_gain_ki_.z(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(pnh, "yaw_gain_kp/yaw",
                  position_controller_.controller_parameters_.yaw_gain_kp_,
                  &position_controller_.controller_parameters_.yaw_gain_kp_);
  GetRosParameter(pnh, "yaw_gain_ki/yaw",
                  position_controller_.controller_parameters_.yaw_gain_ki_,
                  &position_controller_.controller_parameters_.yaw_gain_ki_);

  GetRosParameter(pnh, "hovering_gain_kp/z",
                  position_controller_.controller_parameters_.hovering_gain_kp_,
                  &position_controller_.controller_parameters_.hovering_gain_kp_);
  GetRosParameter(pnh, "hovering_gain_ki/z",
                  position_controller_.controller_parameters_.hovering_gain_ki_,
                  &position_controller_.controller_parameters_.hovering_gain_ki_);
  GetRosParameter(pnh, "hovering_gain_kd/z",
                  position_controller_.controller_parameters_.hovering_gain_kd_,
                  &position_controller_.controller_parameters_.hovering_gain_kd_);

  position_controller_.SetControllerGains();
  position_controller_.crazyflie_onboard_controller_.SetControllerGains(position_controller_.controller_parameters_);

}

void PositionControllerNode::Publish(){
}

void PositionControllerNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("PositionController got first imu message.");
    
    //Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;
    
    //Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp_ = imu_msg->header.stamp;	

}

void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    if (waypointHasBeenPublished_){

	    //This functions allows us to put the odometry message into the odometry variable--> _position,
 	    //_orientation,_velocit_body,_angular_velocity
	    EigenOdometry odometry;
	    eigenOdometryFromMsg(odometry_msg, &odometry);
	    position_controller_.SetOdometryWithStateEstimator(odometry);

    }	 
   
}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node_with_stateEstimator");
    
    ros::NodeHandle nh2;
    
    rotors_control::PositionControllerNode position_controller_node;

    ros::spin();

    return 0;
}
