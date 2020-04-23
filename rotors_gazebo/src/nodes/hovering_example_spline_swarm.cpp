/*
 * Copyright 2020 Ria Sonecha, Massachusetts Institute of Technology in Cambridge, MA, USA
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
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

#include "rotors_gazebo/Quaternion.h"
#include "rotors_gazebo/transform_datatypes.h"
#include "rotors_gazebo/parameters_ros.h"
#include <nav_msgs/Odometry.h>
#include "rotors_gazebo/Matrix3x3.h"
#include <ros/console.h>
#include <time.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include "rotors_gazebo/spline_trajectory_generator.h"

#define SAMPLING_TIME  10e-3       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

template<typename T> inline void GetRosParameterHovering(const ros::NodeHandle& nh,
                                                         const std::string& key,
                                                         const T& default_value,
                                                         T* value) {

  ROS_ASSERT(value != nullptr);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example_spline_swarm");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<mav_msgs::DroneState>(
          mav_msgs::default_topics::DRONE_STATE, 10);
  ROS_INFO("[SWARM] Started hovering example with spline.");

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

  // Trajectory message
  mav_msgs::DroneState trajectory_msg, trajectory_msg_pre;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::EigenDroneState eigen_reference;
  rotors_gazebo::SplineTrajectoryGenerator hovering_example_spline_generator;

  Eigen::Vector3f check_position_final;
  GetRosParameterHovering(nh_private, "position_final/x", check_position_final.x(),
    &check_position_final.x());
  GetRosParameterHovering(nh_private, "position_final/y", check_position_final.y(),
    &check_position_final.y());
  GetRosParameterHovering(nh_private, "position_final/z", check_position_final.z(),
    &check_position_final.z());

  hovering_example_spline_generator.InitializeParams();

  // Reading the final time. This is stop condition for the spline generator
  double end_generation_time;
  GetRosParameterHovering(nh_private, "time_final/time", end_generation_time,
    &end_generation_time);

  double initial_time, final_time;
  initial_time = 0;

  // Publish the trajectory values until the final values is reached
  while(true){
    final_time = ros::Time::now().toSec();
    hovering_example_spline_generator.TrajectoryCallback(&eigen_reference,
      &final_time, &initial_time);

    mav_msgs::eigenDroneFromStateToMsg(&eigen_reference, trajectory_msg);

    // new message
    if(eigen_reference.position_W[0] <= check_position_final[0] &&
      eigen_reference.position_W[1] <= check_position_final[1] &&
      eigen_reference.position_W[2] <= check_position_final[2]){
      trajectory_pub.publish(trajectory_msg);
      trajectory_msg_pre = trajectory_msg;
    }

    ROS_DEBUG("Publishing waypoint from msg: [%f, %f, %f].",
      trajectory_msg.position.x, trajectory_msg.position.y, trajectory_msg.position.z);
    ROS_DEBUG("Publishing waypoint: [%f, %f, %f].",
      eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2]);

    ros::Duration(SAMPLING_TIME).sleep();

    // Hold the message until the simulation ends
    if(eigen_reference.position_W[0] > check_position_final[0] &&
      eigen_reference.position_W[1] > check_position_final[1] &&
      eigen_reference.position_W[2] > check_position_final[2])
      trajectory_pub.publish(trajectory_msg_pre);

  }

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
