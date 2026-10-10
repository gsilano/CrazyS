/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2020 Ria Sonecha, MIT, USA
 * Copyright 2026 Giuseppe Silano, Czech Technical University in Prague, Czech Republic
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

// Position controller node of the Crazyflie. Three modes, selected by parameters:
//
//  - default: the controller of the Crazyflie firmware (2018.01.1) with the ideal
//    odometry. Reference on command/trajectory, one control step per odometry message.
//  - enable_state_estimator: same controller, but attitude and angular rate come from
//    the IMU through the complementary filter of the firmware. The position and the
//    linear velocity still come from the odometry. The gyroscope bias is estimated on
//    the ground, before the first waypoint arrives.
//  - enable_internal_model_controller: the Internal Model Controller (reference on
//    drone_state).
//
// The output is always command/motor_speed.

#include <memory>

#include <sensor_msgs/msg/imu.hpp>

#include "rotors_control/internal_model_controller.h"
#include "rotors_control/position_controller.h"
#include "ros_conversions.h"

namespace rotors_control {

// Rates of the firmware tasks when the state estimator is enabled.
static constexpr double kAttitudeEstimationPeriod = 0.004;  // [s], complementary filter
static constexpr double kImuUpdatePeriod = 0.002;           // [s], rate estimation and motors
static constexpr double kHighLevelControlPeriod = 0.01;     // [s], position control

class PositionControllerNode : public rclcpp::Node {
 public:
  PositionControllerNode() : Node("position_controller_node") {
    enable_state_estimator_ = declare_parameter("enable_state_estimator", false);
    enable_internal_model_controller_ =
        declare_parameter("enable_internal_model_controller", false);

    motor_velocity_reference_pub_ = create_publisher<actuator_msgs::msg::Actuators>(
        kCommandMotorSpeedTopic, 1);

    if (enable_internal_model_controller_) {
      InitializeInternalModelController();
      drone_state_sub_ = create_subscription<rotors_comm::msg::DroneState>(
          kDroneStateTopic, 1,
          std::bind(&PositionControllerNode::DroneStateCallback, this, std::placeholders::_1));
      attitude_timer_ = rclcpp::create_timer(
          this, get_clock(), rclcpp::Duration::from_seconds(kAttitudeSamplingTime),
          [this]() { internal_model_controller_.CallbackAttitude(); });
      position_timer_ = rclcpp::create_timer(
          this, get_clock(), rclcpp::Duration::from_seconds(kPositionSamplingTime),
          [this]() { internal_model_controller_.CallbackPosition(); });
    } else {
      InitializePositionController();
      trajectory_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
          kCommandTrajectoryTopic, 1,
          std::bind(&PositionControllerNode::TrajectoryCallback, this, std::placeholders::_1));
    }

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdometryTopic, 1,
        std::bind(&PositionControllerNode::OdometryCallback, this, std::placeholders::_1));

    if (enable_state_estimator_ && !enable_internal_model_controller_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          kImuTopic, 1,
          std::bind(&PositionControllerNode::ImuCallback, this, std::placeholders::_1));
      attitude_timer_ = rclcpp::create_timer(
          this, get_clock(), rclcpp::Duration::from_seconds(kAttitudeEstimationPeriod),
          [this]() {
            if (waypoint_received_) position_controller_.CallbackAttitudeEstimation();
          });
      position_timer_ = rclcpp::create_timer(
          this, get_clock(), rclcpp::Duration::from_seconds(kHighLevelControlPeriod),
          [this]() {
            if (waypoint_received_) position_controller_.CallbackHightLevelControl();
          });
      imu_timer_ = rclcpp::create_timer(
          this, get_clock(), rclcpp::Duration::from_seconds(kImuUpdatePeriod),
          std::bind(&PositionControllerNode::ImuUpdateCallback, this));
    }

    RCLCPP_INFO(get_logger(), "Started (%s%s).",
                enable_internal_model_controller_ ? "internal model controller"
                                                  : "position controller",
                enable_state_estimator_ ? ", with state estimator" : "");
  }

 private:
  bool enable_state_estimator_;
  bool enable_internal_model_controller_;
  bool waypoint_received_ = false;
  bool imu_received_ = false;

  // Gyroscope bias, estimated while the drone waits on the ground for the first
  // waypoint, as the Crazyflie firmware does at start-up.
  Eigen::Vector3d gyro_bias_sum_ = Eigen::Vector3d::Zero();
  int gyro_bias_samples_ = 0;

  PositionController position_controller_;
  InternalModelController internal_model_controller_;
  sensorData_t sensors_{};

  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_velocity_reference_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<rotors_comm::msg::DroneState>::SharedPtr drone_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr attitude_timer_;
  rclcpp::TimerBase::SharedPtr position_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;

  double Param(const std::string& name, double default_value) {
    return declare_parameter(name, default_value);
  }

  void InitializePositionController() {
    PositionControllerParameters& p = position_controller_.controller_parameters_;
    p.xy_gain_kp_.x() = Param("xy_gain_kp.x", p.xy_gain_kp_.x());
    p.xy_gain_kp_.y() = Param("xy_gain_kp.y", p.xy_gain_kp_.y());
    p.xy_gain_ki_.x() = Param("xy_gain_ki.x", p.xy_gain_ki_.x());
    p.xy_gain_ki_.y() = Param("xy_gain_ki.y", p.xy_gain_ki_.y());
    p.attitude_gain_kp_.x() = Param("attitude_gain_kp.phi", p.attitude_gain_kp_.x());
    p.attitude_gain_kp_.y() = Param("attitude_gain_kp.theta", p.attitude_gain_kp_.y());
    p.attitude_gain_ki_.x() = Param("attitude_gain_ki.phi", p.attitude_gain_ki_.x());
    p.attitude_gain_ki_.y() = Param("attitude_gain_ki.theta", p.attitude_gain_ki_.y());
    p.rate_gain_kp_.x() = Param("rate_gain_kp.p", p.rate_gain_kp_.x());
    p.rate_gain_kp_.y() = Param("rate_gain_kp.q", p.rate_gain_kp_.y());
    p.rate_gain_kp_.z() = Param("rate_gain_kp.r", p.rate_gain_kp_.z());
    p.rate_gain_ki_.x() = Param("rate_gain_ki.p", p.rate_gain_ki_.x());
    p.rate_gain_ki_.y() = Param("rate_gain_ki.q", p.rate_gain_ki_.y());
    p.rate_gain_ki_.z() = Param("rate_gain_ki.r", p.rate_gain_ki_.z());
    p.yaw_gain_kp_ = Param("yaw_gain_kp.yaw", p.yaw_gain_kp_);
    p.yaw_gain_ki_ = Param("yaw_gain_ki.yaw", p.yaw_gain_ki_);
    p.hovering_gain_kp_ = Param("hovering_gain_kp.z", p.hovering_gain_kp_);
    p.hovering_gain_ki_ = Param("hovering_gain_ki.z", p.hovering_gain_ki_);
    p.hovering_gain_kd_ = Param("hovering_gain_kd.z", p.hovering_gain_kd_);
    position_controller_.SetControllerGains();
    if (enable_state_estimator_) {
      position_controller_.crazyflie_onboard_controller_.SetControllerGains(p);
    }
  }

  void InitializeInternalModelController() {
    InternalModelControllerParameters& p = internal_model_controller_.controller_parameters_im_;
    p.beta_xy_.x() = Param("beta_xy.beta_x", p.beta_xy_.x());
    p.beta_xy_.y() = Param("beta_xy.beta_y", p.beta_xy_.y());
    p.beta_z_ = Param("beta_z.beta_z", p.beta_z_);
    p.beta_phi_ = Param("beta_phi.beta_phi", p.beta_phi_);
    p.beta_theta_ = Param("beta_theta.beta_theta", p.beta_theta_);
    p.beta_psi_ = Param("beta_psi.beta_psi", p.beta_psi_);
    p.mu_xy_.x() = Param("mu_xy.mu_x", p.mu_xy_.x());
    p.mu_xy_.y() = Param("mu_xy.mu_y", p.mu_xy_.y());
    p.mu_z_ = Param("mu_z.mu_z", p.mu_z_);
    p.mu_phi_ = Param("mu_phi.mu_phi", p.mu_phi_);
    p.mu_theta_ = Param("mu_theta.mu_theta", p.mu_theta_);
    p.mu_psi_ = Param("mu_psi.mu_psi", p.mu_psi_);
    p.U_q_.x() = Param("U_xyz.U_x", p.U_q_.x());
    p.U_q_.y() = Param("U_xyz.U_y", p.U_q_.y());
    p.U_q_.z() = Param("U_xyz.U_z", p.U_q_.z());
    internal_model_controller_.SetControllerGains();

    VehicleParameters& v = internal_model_controller_.vehicle_parameters_;
    v.mass_ = Param("mass", v.mass_);
    v.inertia_(0, 0) = Param("inertia.xx", v.inertia_(0, 0));
    v.inertia_(1, 1) = Param("inertia.yy", v.inertia_(1, 1));
    v.inertia_(2, 2) = Param("inertia.zz", v.inertia_(2, 2));
    v.bf_ = Param("bf", v.bf_);
    v.bm_ = Param("bm", v.bm_);
    v.armLength_ = Param("l", v.armLength_);
    internal_model_controller_.SetVehicleParameters();
  }

  void TrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg) {
    if (msg.points.empty()) {
      RCLCPP_WARN(get_logger(), "Got a MultiDOFJointTrajectory message without points.");
      return;
    }
    EigenTrajectoryPoint reference;
    EigenTrajectoryPointFromMsg(msg.points.front(), &reference);
    position_controller_.SetTrajectoryPoint(reference);
    if (!waypoint_received_) {
      RCLCPP_INFO(get_logger(), "Got the first waypoint.");
      waypoint_received_ = true;
    }
  }

  void DroneStateCallback(const rotors_comm::msg::DroneState& msg) {
    if (msg.position.z <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Ignoring a reference point below the ground.");
      return;
    }
    EigenDroneState reference;
    EigenDroneStateFromMsg(msg, &reference);
    internal_model_controller_.SetTrajectoryPoint(reference);
    if (!waypoint_received_) {
      RCLCPP_INFO(get_logger(), "Got the first reference point.");
      waypoint_received_ = true;
    }
  }

  void OdometryCallback(const nav_msgs::msg::Odometry& msg) {
    if (!waypoint_received_) {
      return;
    }
    EigenOdometry odometry;
    EigenOdometryFromMsg(msg, &odometry);

    Eigen::Vector4d rotor_velocities;
    if (enable_internal_model_controller_) {
      internal_model_controller_.SetOdometryWithoutStateEstimator(odometry);
      internal_model_controller_.CalculateRotorVelocities(&rotor_velocities);
    } else if (enable_state_estimator_) {
      // Only position and linear velocity are taken from the odometry, the motors are
      // updated by ImuUpdateCallback().
      position_controller_.SetOdometryWithStateEstimator(odometry);
      return;
    } else {
      position_controller_.SetOdometryWithoutStateEstimator(odometry);
      position_controller_.CalculateRotorVelocities(&rotor_velocities);
    }
    motor_velocity_reference_pub_->publish(ActuatorsMsg(rotor_velocities, msg.header.stamp));
  }

  void ImuCallback(const sensor_msgs::msg::Imu& msg) {
    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y,
                               msg.angular_velocity.z);
    if (!waypoint_received_) {
      gyro_bias_sum_ += gyro;
      ++gyro_bias_samples_;
    }
    const Eigen::Vector3d bias =
        gyro_bias_samples_ > 0 ? Eigen::Vector3d(gyro_bias_sum_ / gyro_bias_samples_)
                               : Eigen::Vector3d::Zero();
    sensors_.gyro.x = gyro.x() - bias.x();
    sensors_.gyro.y = gyro.y() - bias.y();
    sensors_.gyro.z = gyro.z() - bias.z();
    sensors_.acc.x = msg.linear_acceleration.x;
    sensors_.acc.y = msg.linear_acceleration.y;
    sensors_.acc.z = msg.linear_acceleration.z;
    imu_received_ = true;
  }

  void ImuUpdateCallback() {
    if (!imu_received_) {
      return;
    }
    position_controller_.SetSensorData(sensors_);
    if (!waypoint_received_) {
      return;
    }
    Eigen::Vector4d rotor_velocities;
    position_controller_.CalculateRotorVelocities(&rotor_velocities);
    motor_velocity_reference_pub_->publish(ActuatorsMsg(rotor_velocities, now()));
  }
};

}  // namespace rotors_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rotors_control::PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
