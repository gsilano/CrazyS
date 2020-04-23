/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2020 Ria Sonecha, MIT, USA
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

#include "position_controller_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

PositionControllerNode::PositionControllerNode() {

    ROS_INFO_ONCE("Started position controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    // This command diplays which controller is enabled
    if(pnh_node.getParam("enable_mellinger_controller", enable_mellinger_controller_)){
      ROS_INFO("Got param 'enable_mellinger_controller': %d", enable_mellinger_controller_);
    }

    if(pnh_node.getParam("enable_internal_model_controller", enable_internal_model_controller_)){
      ROS_INFO("Got param 'enable_internal_model_controller': %d", enable_internal_model_controller_);
    }

    InitializeParams();

    // Topics subscribe
    if (!enable_mellinger_controller_ && !enable_internal_model_controller_){

      cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
        &PositionControllerNode::MultiDofJointTrajectoryCallback, this);

      odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
        &PositionControllerNode::OdometryCallback, this);

    }
    else{

      imu_ideal_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1,
        &PositionControllerNode::IMUMellingerCallback, this);

      cmd_multi_dof_joint_trajectory_spline_sub_ = nh.subscribe(mav_msgs::default_topics::DRONE_STATE, 1,
        &PositionControllerNode::MultiDofJointTrajectoryMellingerCallback, this);

      odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
        &PositionControllerNode::MellingerOdometryCallback, this);

    }

    // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // The subscription to the IMU topic is made only if it is available, when simulating the Crazyflie dynamics considering the also IMU values
    if(pnh_node.getParam("enable_state_estimator", enable_state_estimator_)){
      ROS_INFO("Got param 'enable_state_estimator': %d", enable_state_estimator_);
    }

    if (enable_state_estimator_){
        imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1, &PositionControllerNode::IMUCallback, this);

        //Timers allow to set up the working frequency of the control system
        timer_Attitude_ = n_.createTimer(ros::Duration(ATTITUDE_UPDATE_DT), &PositionControllerNode::CallbackAttitudeEstimation, this, false, true);

        timer_highLevelControl = n_.createTimer(ros::Duration(SAMPLING_TIME), &PositionControllerNode::CallbackHightLevelControl, this, false, true);

        timer_IMUUpdate = n_.createTimer(ros::Duration(RATE_UPDATE_DT), &PositionControllerNode::CallbackIMUUpdate, this, false, true);

    }


}

PositionControllerNode::~PositionControllerNode(){}


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

void PositionControllerNode::MultiDofJointTrajectoryMellingerCallback(const mav_msgs::DroneState& drone_state_msg){
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    if(drone_state_msg.position.z <= 0){
      ROS_WARN_STREAM("Got MultiDOFJointTrajectoryMellinger message, but message has no points.");
      return;
    }

    // We can trigger the first command immediately.
    mav_msgs::EigenDroneState eigen_reference;
    mav_msgs::eigenDroneStateFromMsg(drone_state_msg, &eigen_reference);
    if (enable_mellinger_controller_)
      mellinger_controller_.SetTrajectoryPoint(eigen_reference);
    else
      internal_model_controller_.SetTrajectoryPoint(eigen_reference);

    ROS_DEBUG("Drone desired position [x_d: %f, y_d: %f, z_d: %f]", eigen_reference.position_W[0],
            eigen_reference.position_W[1], eigen_reference.position_W[2]);

    if (drone_state_msg.position.z > 0) {
      waypointHasBeenPublished_ = true;
      if (enable_mellinger_controller_)
        ROS_INFO_ONCE("PositionController got first [Mellinger] MultiDOFJointTrajectory message.");
      else
        ROS_INFO_ONCE("PositionController got first [InternalModel] MultiDOFJointTrajectory message.");
    }
}


void PositionControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  if(!enable_mellinger_controller_ && !enable_internal_model_controller_){

    // Parameters reading from rosparam.
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
    GetRosParameter(pnh, "attitude_gain_ki/theta",
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

    ROS_INFO_ONCE("[Position Controller] Set controller gains and vehicle parameters");

    //Reading the parameters come from the launch file
    bool dataStoringActive;
    double dataStoringTime;
    std::string user;

    if (pnh.getParam("user_account", user)){
    ROS_INFO("Got param 'user_account': %s", user.c_str());
    position_controller_.user_ = user;
    }
    else
       ROS_ERROR("Failed to get param 'user'");

    if (pnh.getParam("csvFilesStoring", dataStoringActive)){
    ROS_INFO("Got param 'csvFilesStoring': %d", dataStoringActive);
    position_controller_.dataStoring_active_ = dataStoringActive;
    }
    else
       ROS_ERROR("Failed to get param 'csvFilesStoring'");

    if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
    ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
    position_controller_.dataStoringTime_ = dataStoringTime;
    }
    else
       ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

    position_controller_.SetLaunchFileParameters();

   }

 if(enable_mellinger_controller_){

    // Parameters reading from rosparam.
    GetRosParameter(pnh, "kp_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.kpXYPositionController_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.kpXYPositionController_.x());
    GetRosParameter(pnh, "kp_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.kpXYPositionController_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.kpXYPositionController_.y());

    GetRosParameter(pnh, "kd_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.kdXYPositionController_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.kdXYPositionController_.x());
    GetRosParameter(pnh, "kd_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.kdXYPositionController_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.kdXYPositionController_.y());

    GetRosParameter(pnh, "ki_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.kiXYPositionController_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.kiXYPositionController_.x());
    GetRosParameter(pnh, "ki_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.kiXYPositionController_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.kiXYPositionController_.y());

    GetRosParameter(pnh, "kp_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.kpZPositionController_,
                    &mellinger_controller_.controller_parameters_mellinger_.kpZPositionController_);
    GetRosParameter(pnh, "kd_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.kdZPositionController_,
                    &mellinger_controller_.controller_parameters_mellinger_.kdZPositionController_);
    GetRosParameter(pnh, "ki_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.kiZPositionController_,
                    &mellinger_controller_.controller_parameters_mellinger_.kiZPositionController_);

    GetRosParameter(pnh, "kr_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.krXYPositionController_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.krXYPositionController_.x());
    GetRosParameter(pnh, "kr_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.krXYPositionController_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.krXYPositionController_.y());

    GetRosParameter(pnh, "kw_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.kwXYPositionController_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.kwXYPositionController_.x());
    GetRosParameter(pnh, "kw_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.kwXYPositionController_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.kwXYPositionController_.y());

    GetRosParameter(pnh, "ki_m_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.ki_mXYPositionController_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.ki_mXYPositionController_.x());
    GetRosParameter(pnh, "ki_m_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.ki_mXYPositionController_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.ki_mXYPositionController_.y());

    GetRosParameter(pnh, "kr_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.krZPositionController_,
                    &mellinger_controller_.controller_parameters_mellinger_.krZPositionController_);
    GetRosParameter(pnh, "kw_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.kwZPositionController_,
                    &mellinger_controller_.controller_parameters_mellinger_.kwZPositionController_);
    GetRosParameter(pnh, "ki_m_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.ki_mZPositionController_,
                    &mellinger_controller_.controller_parameters_mellinger_.ki_mZPositionController_);

    GetRosParameter(pnh, "i_range_m_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.iRangeMXY_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.iRangeMXY_.x());
    GetRosParameter(pnh, "i_range_m_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.iRangeMXY_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.iRangeMXY_.y());

    GetRosParameter(pnh, "i_range_xy/x",
                    mellinger_controller_.controller_parameters_mellinger_.iRangeXY_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.iRangeXY_.x());
    GetRosParameter(pnh, "i_range_xy/y",
                    mellinger_controller_.controller_parameters_mellinger_.iRangeXY_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.iRangeXY_.y());

    GetRosParameter(pnh, "i_range_m_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.iRangeMZ_,
                    &mellinger_controller_.controller_parameters_mellinger_.iRangeMZ_);
    GetRosParameter(pnh, "i_range_z/z",
                    mellinger_controller_.controller_parameters_mellinger_.iRangeZ_,
                    &mellinger_controller_.controller_parameters_mellinger_.iRangeZ_);

    GetRosParameter(pnh, "kd_omega_rp/r",
                    mellinger_controller_.controller_parameters_mellinger_.kdOmegaRP_.x(),
                    &mellinger_controller_.controller_parameters_mellinger_.kdOmegaRP_.x());
    GetRosParameter(pnh, "kd_omega_rp/p",
                    mellinger_controller_.controller_parameters_mellinger_.kdOmegaRP_.y(),
                    &mellinger_controller_.controller_parameters_mellinger_.kdOmegaRP_.y());

    mellinger_controller_.SetControllerGains();

    //Analogously, the object "vehicle_parameters_" is created
    GetFullVehicleParameters(pnh, &mellinger_controller_.vehicle_parameters_);
    mellinger_controller_.SetVehicleParameters();
    ROS_INFO_ONCE("[Mellinger Controller] Set controller gains and vehicle parameters");

    }

  if (enable_internal_model_controller_){
    // Read parameters from rosparam. The parameters are read by the YAML file and they
    // are used to create the "controller_parameters_" object
    GetRosParameter(pnh, "beta_xy/beta_x",
                    internal_model_controller_.controller_parameters_im_.beta_xy_.x(),
                    &internal_model_controller_.controller_parameters_im_.beta_xy_.x());
    GetRosParameter(pnh, "beta_xy/beta_y",
                    internal_model_controller_.controller_parameters_im_.beta_xy_.y(),
                    &internal_model_controller_.controller_parameters_im_.beta_xy_.y());
    GetRosParameter(pnh, "beta_z/beta_z",
                    internal_model_controller_.controller_parameters_im_.beta_z_,
                    &internal_model_controller_.controller_parameters_im_.beta_z_);

    GetRosParameter(pnh, "beta_phi/beta_phi",
                    internal_model_controller_.controller_parameters_im_.beta_phi_,
                    &internal_model_controller_.controller_parameters_im_.beta_phi_);
    GetRosParameter(pnh, "beta_theta/beta_theta",
                    internal_model_controller_.controller_parameters_im_.beta_theta_,
                    &internal_model_controller_.controller_parameters_im_.beta_theta_);
    GetRosParameter(pnh, "beta_psi/beta_psi",
                    internal_model_controller_.controller_parameters_im_.beta_psi_,
                    &internal_model_controller_.controller_parameters_im_.beta_psi_);

    GetRosParameter(pnh, "mu_xy/mu_x",
                    internal_model_controller_.controller_parameters_im_.mu_xy_.x(),
                    &internal_model_controller_.controller_parameters_im_.mu_xy_.x());
    GetRosParameter(pnh, "mu_xy/mu_y",
                    internal_model_controller_.controller_parameters_im_.mu_xy_.y(),
                    &internal_model_controller_.controller_parameters_im_.mu_xy_.y());
    GetRosParameter(pnh, "mu_z/mu_z",
                    internal_model_controller_.controller_parameters_im_.mu_z_,
                    &internal_model_controller_.controller_parameters_im_.mu_z_);

    GetRosParameter(pnh, "mu_phi/mu_phi",
                    internal_model_controller_.controller_parameters_im_.mu_phi_,
                    &internal_model_controller_.controller_parameters_im_.mu_phi_);
    GetRosParameter(pnh, "mu_theta/mu_theta",
                    internal_model_controller_.controller_parameters_im_.mu_theta_,
                    &internal_model_controller_.controller_parameters_im_.mu_theta_);
    GetRosParameter(pnh, "mu_psi/mu_psi",
                    internal_model_controller_.controller_parameters_im_.mu_psi_,
                    &internal_model_controller_.controller_parameters_im_.mu_psi_);

    GetRosParameter(pnh, "U_xyz/U_x",
                    internal_model_controller_.controller_parameters_im_.U_q_.x(),
                    &internal_model_controller_.controller_parameters_im_.U_q_.x());
    GetRosParameter(pnh, "U_xyz/U_y",
                    internal_model_controller_.controller_parameters_im_.U_q_.y(),
                    &internal_model_controller_.controller_parameters_im_.U_q_.y());
    GetRosParameter(pnh, "U_xyz/U_z",
                    internal_model_controller_.controller_parameters_im_.U_q_.z(),
                    &internal_model_controller_.controller_parameters_im_.U_q_.z());

    internal_model_controller_.SetControllerGains();

    //Analogously, the object "vehicle_parameters_" is created
    GetFullVehicleParameters(pnh, &internal_model_controller_.vehicle_parameters_);
    internal_model_controller_.SetVehicleParameters();
    ROS_INFO_ONCE("[Internal Model Controller] Set controller gains and vehicle parameters");

  }

  if (enable_state_estimator_)
    position_controller_.crazyflie_onboard_controller_.SetControllerGains(position_controller_.controller_parameters_);

}

void PositionControllerNode::Publish(){
}

void PositionControllerNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("PositionController got first imu message.");

    // Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;

    // Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp_ = imu_msg->header.stamp;

}

void PositionControllerNode::IMUMellingerCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("MellingerController got first imu message.");

    // Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;

    // Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp_ = imu_msg->header.stamp;

    position_controller_.SetSensorData(sensors_);

}


void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    if (waypointHasBeenPublished_ && enable_state_estimator_){

	    //This functions allows us to put the odometry message into the odometry variable--> _position,
 	    //_orientation,_velocit_body,_angular_velocity
	    EigenOdometry odometry;
	    eigenOdometryFromMsg(odometry_msg, &odometry);
	    position_controller_.SetOdometryWithStateEstimator(odometry);

    }

    if(waypointHasBeenPublished_){

      //This functions allows us to put the odometry message into the odometry variable--> _position,
      //_orientation,_velocit_body,_angular_velocity
      EigenOdometry odometry;
      eigenOdometryFromMsg(odometry_msg, &odometry);
      position_controller_.SetOdometryWithoutStateEstimator(odometry);

      Eigen::Vector4d ref_rotor_velocities;
      position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

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

void PositionControllerNode::MellingerOdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    if(enable_mellinger_controller_)
      ROS_INFO_ONCE("PositionControllerNode got first odometry message (Mellinger).");
    else
      ROS_INFO_ONCE("PositionControllerNode got first odometry message (Internal Model Controller).");

    if(waypointHasBeenPublished_){

      //This functions allows us to put the odometry message into the odometry variable--> _position,
      //_orientation,_velocit_body,_angular_velocity
      EigenOdometry odometry;
      eigenOdometryFromMsg(odometry_msg, &odometry);

      if(enable_mellinger_controller_)
        mellinger_controller_.SetOdometryWithoutStateEstimator(odometry);
      else
        internal_model_controller_.SetOdometryWithoutStateEstimator(odometry);

      Eigen::Vector4d ref_rotor_velocities;

      if(enable_mellinger_controller_)
        mellinger_controller_.CalculateRotorVelocities(&ref_rotor_velocities);
      else
        internal_model_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

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

// The attitude is estimated only if the waypoint has been published
void PositionControllerNode::CallbackAttitudeEstimation(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            position_controller_.CallbackAttitudeEstimation();

}

// The high level control is run only if the waypoint has been published
void PositionControllerNode::CallbackHightLevelControl(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            position_controller_.CallbackHightLevelControl();

}

// IMU messages are sent to the controller with a frequency of 500Hz. In other words, with a sampling time of 0.002 seconds
void PositionControllerNode::CallbackIMUUpdate(const ros::TimerEvent& event){

    position_controller_.SetSensorData(sensors_);

    ROS_INFO_ONCE("IMU Message sent to position controller");

    if (waypointHasBeenPublished_){

	    Eigen::Vector4d ref_rotor_velocities;
	    position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

	    // A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
	    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	    // The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular velocities).
	    actuator_msg->angular_velocities.clear();
	    // for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	    for (int i = 0; i < ref_rotor_velocities.size(); i++)
	       actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	    actuator_msg->header.stamp = imu_msg_head_stamp_;

	    motor_velocity_reference_pub_.publish(actuator_msg);

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
