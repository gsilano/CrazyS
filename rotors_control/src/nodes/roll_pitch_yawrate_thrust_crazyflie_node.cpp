/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
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

// Attitude and rate controller of the Crazyflie driven by roll, pitch, yaw rate and
// thrust commands (e.g. from the joystick interface).
//   in:  command/roll_pitch_yawrate_thrust (rotors_comm/RollPitchYawrateThrustCrazyflie),
//        odometry (nav_msgs/Odometry)
//   out: command/motor_speed (actuator_msgs/Actuators), one message per odometry message

#include <memory>

#include "rotors_control/roll_pitch_yawrate_thrust_crazyflie.h"
#include "ros_conversions.h"

namespace rotors_control {

class RollPitchYawrateThrustCrazyflieNode : public rclcpp::Node {
 public:
  RollPitchYawrateThrustCrazyflieNode() : Node("roll_pitch_yawrate_thrust_crazyflie_node") {
    InitializeParams();
    command_sub_ = create_subscription<rotors_comm::msg::RollPitchYawrateThrustCrazyflie>(
        kCommandRollPitchYawrateThrustTopic, 1,
        std::bind(&RollPitchYawrateThrustCrazyflieNode::CommandCallback, this,
                  std::placeholders::_1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdometryTopic, 1,
        std::bind(&RollPitchYawrateThrustCrazyflieNode::OdometryCallback, this,
                  std::placeholders::_1));
    motor_velocity_reference_pub_ = create_publisher<actuator_msgs::msg::Actuators>(
        kCommandMotorSpeedTopic, 1);
    RCLCPP_INFO(get_logger(), "Started.");
  }

 private:
  RollPitchYawrateThrustControllerCrazyflie controller_;

  rclcpp::Subscription<rotors_comm::msg::RollPitchYawrateThrustCrazyflie>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_velocity_reference_pub_;

  void InitializeParams() {
    RollPitchYawrateThrustControllerCrazyflieParameters& p = controller_.controller_parameters_;
    p.attitude_gain_kp_.x() = declare_parameter("attitude_gain_kp.phi", p.attitude_gain_kp_.x());
    p.attitude_gain_kp_.y() =
        declare_parameter("attitude_gain_kp.theta", p.attitude_gain_kp_.y());
    p.attitude_gain_ki_.x() = declare_parameter("attitude_gain_ki.phi", p.attitude_gain_ki_.x());
    p.attitude_gain_ki_.y() =
        declare_parameter("attitude_gain_ki.theta", p.attitude_gain_ki_.y());
    p.rate_gain_kp_.x() = declare_parameter("rate_gain_kp.p", p.rate_gain_kp_.x());
    p.rate_gain_kp_.y() = declare_parameter("rate_gain_kp.q", p.rate_gain_kp_.y());
    p.rate_gain_kp_.z() = declare_parameter("rate_gain_kp.r", p.rate_gain_kp_.z());
    p.rate_gain_ki_.x() = declare_parameter("rate_gain_ki.p", p.rate_gain_ki_.x());
    p.rate_gain_ki_.y() = declare_parameter("rate_gain_ki.q", p.rate_gain_ki_.y());
    p.rate_gain_ki_.z() = declare_parameter("rate_gain_ki.r", p.rate_gain_ki_.z());
    p.thrust_percentage_ = declare_parameter("thrust_percentage.p", p.thrust_percentage_);
    controller_.SetControllerGains();
  }

  void CommandCallback(const rotors_comm::msg::RollPitchYawrateThrustCrazyflie& msg) {
    EigenRollPitchYawrateThrustCrazyflie command;
    command.roll = msg.roll;
    command.pitch = msg.pitch;
    command.yaw_rate = msg.yaw_rate;
    command.thrust = msg.thrust;
    controller_.SetRollPitchYawrateThrust(command);
  }

  void OdometryCallback(const nav_msgs::msg::Odometry& msg) {
    EigenOdometry odometry;
    EigenOdometryFromMsg(msg, &odometry);
    controller_.SetOdometry(odometry);
    Eigen::Vector4d rotor_velocities;
    controller_.CalculateRotorVelocities(&rotor_velocities);
    motor_velocity_reference_pub_->publish(ActuatorsMsg(rotor_velocities, msg.header.stamp));
  }
};

}  // namespace rotors_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rotors_control::RollPitchYawrateThrustCrazyflieNode>());
  rclcpp::shutdown();
  return 0;
}
