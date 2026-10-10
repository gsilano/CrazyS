/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2026 Giuseppe Silano, Czech Technical University in Prague, Czech Republic
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

// Converts the joystick input (sensor_msgs/Joy on "joy") into roll, pitch, yaw rate
// and thrust commands for the Crazyflie (rotors_comm/RollPitchYawrateThrustCrazyflie on
// "command/roll_pitch_yawrate_thrust").
//
// Parameters: axis_roll, axis_pitch, axis_thrust, axis_yaw_rate, axis_direction_roll,
// axis_direction_pitch, max_roll [rad], max_pitch [rad], max_yaw_rate [rad/s],
// max_thrust [PWM].

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rotors_comm/msg/roll_pitch_yawrate_thrust_crazyflie.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace {

constexpr double kMaxThrust = 8162.0;                 // [PWM]
constexpr double kMaxRoll = 30.0 * M_PI / 180.0;      // [rad]
constexpr double kMaxPitch = 30.0 * M_PI / 180.0;     // [rad]
constexpr double kMaxYawRate = 200.0 * M_PI / 180.0;  // [rad/s]

}  // namespace

class JoyCrazyflie : public rclcpp::Node {
 public:
  JoyCrazyflie() : Node("rotors_joy_interface_crazyflie") {
    axis_roll_ = declare_parameter("axis_roll", 0);
    axis_pitch_ = declare_parameter("axis_pitch", 1);
    axis_thrust_ = declare_parameter("axis_thrust", 3);
    axis_yaw_rate_ = declare_parameter("axis_yaw_rate", 2);
    roll_direction_ = declare_parameter("axis_direction_roll", -1);
    pitch_direction_ = declare_parameter("axis_direction_pitch", 1);
    max_roll_ = declare_parameter("max_roll", kMaxRoll);
    max_pitch_ = declare_parameter("max_pitch", kMaxPitch);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", kMaxYawRate);
    max_thrust_ = declare_parameter("max_thrust", kMaxThrust);

    ctrl_pub_ = create_publisher<rotors_comm::msg::RollPitchYawrateThrustCrazyflie>(
        "command/roll_pitch_yawrate_thrust", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, [this](const sensor_msgs::msg::Joy& msg) { JoyCallback(msg); });
  }

 private:
  int axis_roll_, axis_pitch_, axis_thrust_, axis_yaw_rate_;
  int roll_direction_, pitch_direction_;
  double max_roll_, max_pitch_, max_yaw_rate_, max_thrust_;

  rclcpp::Publisher<rotors_comm::msg::RollPitchYawrateThrustCrazyflie>::SharedPtr ctrl_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  double Axis(const sensor_msgs::msg::Joy& msg, int index) const {
    return index >= 0 && index < static_cast<int>(msg.axes.size()) ? msg.axes[index] : 0.0;
  }

  void JoyCallback(const sensor_msgs::msg::Joy& msg) {
    rotors_comm::msg::RollPitchYawrateThrustCrazyflie command;
    command.header.stamp = now();
    command.header.frame_id = "rotors_joy_frame";
    command.roll = Axis(msg, axis_roll_) * max_roll_ * roll_direction_;
    command.pitch = Axis(msg, axis_pitch_) * max_pitch_ * pitch_direction_;

    // Yaw rate and thrust are on/off, as in the ROS 1 version.
    const double yaw_axis = Axis(msg, axis_yaw_rate_);
    command.yaw_rate = yaw_axis > 0.0 ? max_yaw_rate_ : (yaw_axis < 0.0 ? -max_yaw_rate_ : 0.0);
    command.thrust = Axis(msg, axis_thrust_) > 0.0 ? max_thrust_ : 0.0;

    ctrl_pub_->publish(command);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyCrazyflie>());
  rclcpp::shutdown();
  return 0;
}
