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
// One control step every Ts seconds of simulation time, triggered by the time stamps of
// the odometry messages (1 kHz) rather than by a ROS timer: the state is then sampled
// at regular times even when the computer is loaded and the messages arrive late.
class LeeGeometricControllerNode {
 public:
  LeeGeometricControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~LeeGeometricControllerNode();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  LeeGeometricController lee_geometric_controller_;

  // Time stamp of the odometry message that triggers the next control step [s].
  double next_control_time_;

  ros::Subscriber drone_state_sub_;
  ros::Subscriber odometry_sub_;
  ros::Publisher motor_velocity_reference_pub_;

  void InitializeParams();

  void DroneStateCallback(const mav_msgs::DroneStateConstPtr& drone_state_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};

}  // namespace rotors_control

#endif  // ROTORS_CONTROL_LEE_GEOMETRIC_CONTROLLER_NODE_H
