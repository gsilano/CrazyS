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

#ifndef LEE_POSITION_CONTROLLER_ALPHA_NODE_H
#define LEE_POSITION_CONTROLLER_ALPHA_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ros/time.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller_alpha.h"

namespace rotors_control {

  class LeePositionControllerAlphaNode {
   public:
    LeePositionControllerAlphaNode();
    ~LeePositionControllerAlphaNode();

    void InitializeParams();
    void Publish();

   private:

    bool waypointHasBeenPublished_ = false;

    LeePositionControllerAlpha lee_position_controller_alpha_; // instance of the of LeePositionControllerAlpha class in the "library" folder

    ros::NodeHandle n_;

    std::string namespace_;

    double Ts_internal_;

    //subscribers
    ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
    ros::Subscriber odometry_sub_;

    //publisher
    ros::Publisher motor_velocity_reference_pub_;

    mav_msgs::EigenTrajectoryPointDeque commands_;
    std::deque<ros::Duration> command_waiting_times_;
    ros::Timer command_timer_;

    // Callbacks
    void ReferenceTrajectoryCallback(const mav_msgs::DroneState& drone_state_msg);
    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  };
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_ALPHA_NODE_H
