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
#define START_SIMULATION_TIME 3   /* TIME GAZEBO NEEDS TO INITIALIZE THE ENVIRONMENT */

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
  ros::init(argc, argv, "hovering_example_spline");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<mav_msgs::DroneState>(
          mav_msgs::default_topics::DRONE_STATE, 10);
  ROS_INFO("Started hovering example with spline.");

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

  ROS_DEBUG("[Node] Position final x: %f, y: %f, z :%f", check_position_final.x(),
    check_position_final.y(), check_position_final.z());

  // Wait for 5 seconds to let the Gazebo GUI show up.
  if (ros::Time::now().toSec() < START_SIMULATION_TIME){
    ros::Duration(START_SIMULATION_TIME).sleep();
    hovering_example_spline_generator.InitializeParams();
  }

  // Reading the final time. This is stop condition for the spline generator
  double end_generation_time;
  GetRosParameterHovering(nh_private, "time_final/time", end_generation_time,
    &end_generation_time);

  double initial_time, final_time;
  initial_time = START_SIMULATION_TIME;

  // Publish the trajectory values until the final values is reached
  while(true){
    final_time = ros::Time::now().toSec();
    hovering_example_spline_generator.TrajectoryCallback(&eigen_reference, &final_time, &initial_time);

    mav_msgs::eigenDroneFromStateToMsg(&eigen_reference, trajectory_msg);

    // new message
    if(eigen_reference.position_W[0] <= check_position_final.x() &&
      eigen_reference.position_W[1] <= check_position_final.y() &&
      eigen_reference.position_W[2] <= check_position_final.z()){
      trajectory_pub.publish(trajectory_msg);
      trajectory_msg_pre = trajectory_msg;
    }

    ROS_DEBUG("Publishing waypoint from msg: [%f, %f, %f].", trajectory_msg.position.x, trajectory_msg.position.y, trajectory_msg.position.z);
    ROS_DEBUG("Publishing waypoint: [%f, %f, %f].", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2]);

    ros::Duration(SAMPLING_TIME).sleep();

    // Hold the message until the simulation ends
    if(eigen_reference.position_W[0] > check_position_final.x() &&
      eigen_reference.position_W[1] > check_position_final.y() &&
      eigen_reference.position_W[2] > check_position_final.z())
      trajectory_pub.publish(trajectory_msg_pre);

  }

  ros::spin();

  return 0;
}
