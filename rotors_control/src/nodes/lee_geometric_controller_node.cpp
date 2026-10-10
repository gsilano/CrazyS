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

#include "lee_geometric_controller_node.h"

#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeeGeometricControllerNode::LeeGeometricControllerNode(const ros::NodeHandle& nh,
                                                       const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), odometry_received_(false) {
  InitializeParams();

  drone_state_sub_ = nh_.subscribe(mav_msgs::default_topics::DRONE_STATE, 1,
                                   &LeeGeometricControllerNode::DroneStateCallback, this);
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &LeeGeometricControllerNode::OdometryCallback, this);
  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  control_timer_ = nh_.createTimer(
      ros::Duration(lee_geometric_controller_.controller_parameters_.sampling_time),
      &LeeGeometricControllerNode::ControlTimerCallback, this);

  ROS_INFO("[Lee geometric controller] Started.");
}

LeeGeometricControllerNode::~LeeGeometricControllerNode() {}

void LeeGeometricControllerNode::InitializeParams() {
  LeeGeometricControllerParameters& p = lee_geometric_controller_.controller_parameters_;
  double inertia_xx = p.inertia(0, 0);
  double inertia_yy = p.inertia(1, 1);
  double inertia_zz = p.inertia(2, 2);

  // Missing parameters keep the defaults of LeeGeometricControllerParameters.
  GetRosParameter(private_nh_, "sampling_time", p.sampling_time, &p.sampling_time);
  GetRosParameter(private_nh_, "gravity", p.gravity, &p.gravity);
  GetRosParameter(private_nh_, "mass", p.mass, &p.mass);
  GetRosParameter(private_nh_, "inertia/xx", inertia_xx, &inertia_xx);
  GetRosParameter(private_nh_, "inertia/yy", inertia_yy, &inertia_yy);
  GetRosParameter(private_nh_, "inertia/zz", inertia_zz, &inertia_zz);
  GetRosParameter(private_nh_, "arm_length", p.arm_length, &p.arm_length);
  GetRosParameter(private_nh_, "motor_constant", p.motor_constant, &p.motor_constant);
  GetRosParameter(private_nh_, "moment_constant", p.moment_constant, &p.moment_constant);
  GetRosParameter(private_nh_, "max_rotor_velocity", p.max_rotor_velocity,
                  &p.max_rotor_velocity);
  GetRosParameter(private_nh_, "dirty_derivative/tau_first_order", p.tau_first_order,
                  &p.tau_first_order);
  GetRosParameter(private_nh_, "dirty_derivative/tau_higher_order", p.tau_higher_order,
                  &p.tau_higher_order);
  GetRosParameter(private_nh_, "gains/position", p.k_x, &p.k_x);
  GetRosParameter(private_nh_, "gains/velocity", p.k_v, &p.k_v);
  GetRosParameter(private_nh_, "gains/attitude", p.k_R, &p.k_R);
  GetRosParameter(private_nh_, "gains/angular_rate", p.k_omega, &p.k_omega);

  p.inertia = Eigen::Vector3d(inertia_xx, inertia_yy, inertia_zz).asDiagonal();

  if (p.sampling_time <= 0.0) {
    ROS_ERROR("[Lee geometric controller] The sampling time must be positive, using %f s.",
              kDefaultLeeSamplingTime);
    p.sampling_time = kDefaultLeeSamplingTime;
  }

  lee_geometric_controller_.InitializeParameters();
  ROS_INFO("[Lee geometric controller] Parameters loaded, running at %.1f Hz.",
           1.0 / p.sampling_time);
}

void LeeGeometricControllerNode::DroneStateCallback(
    const mav_msgs::DroneStateConstPtr& drone_state_msg) {
  // Same check as the other CrazyS controllers: ignore references below the ground.
  if (drone_state_msg->position.z < 0.0) {
    ROS_WARN_THROTTLE(1.0, "[Lee geometric controller] Discarding a reference point "
                      "with negative altitude.");
    return;
  }

  ROS_INFO_ONCE("[Lee geometric controller] Got the first reference trajectory point.");

  mav_msgs::EigenDroneState eigen_reference;
  mav_msgs::eigenDroneStateFromMsg(*drone_state_msg, &eigen_reference);

  // An empty (all zeros) quaternion means zero yaw.
  if (eigen_reference.orientation_W_B.norm() < 1e-6) {
    eigen_reference.orientation_W_B = Eigen::Quaterniond::Identity();
  }

  lee_geometric_controller_.SetTrajectoryPoint(eigen_reference);

  ROS_DEBUG("[Lee geometric controller] Desired position: [%f, %f, %f]",
            eigen_reference.position_W.x(), eigen_reference.position_W.y(),
            eigen_reference.position_W.z());
}

void LeeGeometricControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg) {
  ROS_INFO_ONCE("[Lee geometric controller] Got the first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_geometric_controller_.SetOdometry(odometry);
  odometry_received_ = true;
}

void LeeGeometricControllerNode::ControlTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!odometry_received_) {
    return;
  }

  Eigen::Vector4d rotor_velocities;
  lee_geometric_controller_.CalculateRotorVelocities(&rotor_velocities);

  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < rotor_velocities.size(); i++) {
    actuator_msg->angular_velocities.push_back(rotor_velocities[i]);
  }
  actuator_msg->header.stamp = ros::Time::now();

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}  // namespace rotors_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_geometric_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::LeeGeometricControllerNode lee_geometric_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
