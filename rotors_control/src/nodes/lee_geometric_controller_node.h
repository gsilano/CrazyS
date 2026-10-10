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

#ifndef ROTORS_CONTROL_LEE_GEOMETRIC_CONTROLLER_NODE_H
#define ROTORS_CONTROL_LEE_GEOMETRIC_CONTROLLER_NODE_H

#include <mav_msgs/DroneState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "rotors_control/lee_geometric_controller.h"

namespace rotors_control {

// ROS node for the Lee geometric controller.
//   in:  drone_state (mav_msgs/DroneState), odometry (nav_msgs/Odometry)
//   out: command/motor_speed (mav_msgs/Actuators)
// The control loop runs on a timer at 1 / Ts (sim time), not at the odometry rate,
// because the dirty derivatives assume a fixed sampling time.
class LeeGeometricControllerNode {
 public:
  LeeGeometricControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~LeeGeometricControllerNode();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  LeeGeometricController lee_geometric_controller_;

  bool odometry_received_;

  ros::Subscriber drone_state_sub_;
  ros::Subscriber odometry_sub_;
  ros::Publisher motor_velocity_reference_pub_;
  ros::Timer control_timer_;

  void InitializeParams();

  void DroneStateCallback(const mav_msgs::DroneStateConstPtr& drone_state_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void ControlTimerCallback(const ros::TimerEvent& /*event*/);
};

}  // namespace rotors_control

#endif  // ROTORS_CONTROL_LEE_GEOMETRIC_CONTROLLER_NODE_H
