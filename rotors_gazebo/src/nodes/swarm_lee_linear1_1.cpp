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

#include <thread>
#include <chrono>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define SAMPLING_TIME  10e-3       /* SAMPLING CONTROLLER TIME [s] - 100Hz */
#define START_SIMULATION_TIME 3   /* TIME GAZEBO NEEDS TO INITIALIZE THE ENVIRONMENT */

using namespace Eigen;

int main(int argc, char** argv) {

  ros::init(argc, argv, "swarm_lee_linear1_1");
  ros::NodeHandle nh;

  ros::Publisher trajectory_pub =
      nh.advertise<mav_msgs::DroneState>(
          mav_msgs::default_topics::DRONE_STATE, 10);
  ROS_INFO("Start swarm_lee_linear1_1.");

  double xd, yd, zd;
  double x_b1, y_b1, z_b1;

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

  // Wait for 3 seconds to let the Gazebo GUI show up.
  if (ros::Time::now().toSec() < START_SIMULATION_TIME){
    ros::Duration(START_SIMULATION_TIME).sleep();
  }

  // Trajectory message
  mav_msgs::DroneState trajectory_msg;
  mav_msgs::EigenDroneState eigen_reference;
  double j = 1;

  double t = 3;
  Vector4d Q(0,1,0,0);
  Matrix4d G;
  G <<      1,          0,          0,          0,
            1,          t,          t*t,        t*t*t,
            0,          1,          0,          0,
            0,          1,          2*t,        3*t*t;
  Vector4d A;
  A = G.inverse() * Q;

  double p;
  double s;

  while(j <= t*100){
    p = j / 100;
    j++;
    s = A(0) + A(1)*p + A(2)*p*p + A(3)*p*p*p;

    xd = 0;
    yd = 0;
    zd = s + 0.015;

    x_b1 = 1;
    y_b1 = 0;
    z_b1 = 0;

    trajectory_msg.header.stamp = ros::Time::now();
    eigen_reference.position_W = Eigen::Vector3f(xd, yd, zd);
    eigen_reference.orientation_W_B = Eigen::Quaterniond(0, x_b1, y_b1, z_b1);
    mav_msgs::eigenDroneFromStateToMsg(&eigen_reference, trajectory_msg);

    // Debug eigen_reference
    ROS_DEBUG("Publishing position from msg: [%f, %f, %f].", eigen_reference.position_W[0], eigen_reference.position_W[1],
      eigen_reference.position_W[2]);
    ROS_DEBUG("Publishing orientation: [%f, %f, %f].", eigen_reference.orientation_W_B.x(), eigen_reference.orientation_W_B.y(),
      eigen_reference.orientation_W_B.z());

    // Debug trajectory_message
    ROS_DEBUG("Publishing waypoint from msg: [%f, %f, %f].", trajectory_msg.position.x, trajectory_msg.position.y,
      trajectory_msg.position.z);

    trajectory_pub.publish(trajectory_msg);

    ros::Duration(SAMPLING_TIME).sleep();
  }

  // Wait for 3 seconds to let the Gazebo GUI show up.
  if (ros::Time::now().toSec() < 3*START_SIMULATION_TIME){
    ros::Duration(START_SIMULATION_TIME).sleep();
  }

  t = 5;
  G <<      1,          0,          0,          0,
            1,          t,          t*t,        t*t*t,
            0,          1,          0,          0,
            0,          1,          2*t,        3*t*t;
  A = G.inverse() * Q;

  j = 1;
  while(j <= t*100){
    p = j / 100;
    j++;
    s = A(0) + A(1)*p + A(2)*p*p + A(3)*p*p*p;

    xd = s;
    yd = s;
    zd = 1.015;

    x_b1 = 1;
    y_b1 = 0;
    z_b1 = 0;

    trajectory_msg.header.stamp = ros::Time::now();
    eigen_reference.position_W = Eigen::Vector3f(xd, yd, zd);
    eigen_reference.orientation_W_B = Eigen::Quaterniond(0, x_b1, y_b1, z_b1);
    mav_msgs::eigenDroneFromStateToMsg(&eigen_reference, trajectory_msg);

    // Debug eigen_reference
    ROS_DEBUG("Publishing position from msg: [%f, %f, %f].", eigen_reference.position_W[0], eigen_reference.position_W[1],
      eigen_reference.position_W[2]);
    ROS_DEBUG("Publishing orientation: [%f, %f, %f].", eigen_reference.orientation_W_B.x(), eigen_reference.orientation_W_B.y(),
      eigen_reference.orientation_W_B.z());

    // Debug trajectory_message
    ROS_DEBUG("Publishing waypoint from msg: [%f, %f, %f].", trajectory_msg.position.x, trajectory_msg.position.y,
      trajectory_msg.position.z);

    trajectory_pub.publish(trajectory_msg);

    ros::Duration(SAMPLING_TIME).sleep();
  }

  ros::spin();

  return 0;
}
