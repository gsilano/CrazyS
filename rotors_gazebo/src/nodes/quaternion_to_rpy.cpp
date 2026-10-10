/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
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

// Publishes on orientation_rpy (geometry_msgs/Vector3) the roll, pitch and yaw angles of
// the drone in degrees, in [0, 360), read from the odometry.

#include <algorithm>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class QuaternionToRpy : public rclcpp::Node {
 public:
  QuaternionToRpy() : Node("quaternion_to_rpy") {
    rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3>("orientation_rpy", 1);
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 1, [this](const nav_msgs::msg::Odometry& msg) { Callback(msg); });
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  static double ToPositiveDegrees(double angle) {
    double degrees = angle * 180.0 / M_PI;
    return degrees < 0.0 ? degrees + 360.0 : degrees;
  }

  void Callback(const nav_msgs::msg::Odometry& msg) {
    const auto& q = msg.pose.pose.orientation;
    const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = ToPositiveDegrees(roll);
    rpy.y = ToPositiveDegrees(pitch);
    rpy.z = ToPositiveDegrees(yaw);
    rpy_pub_->publish(rpy);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuaternionToRpy>());
  rclcpp::shutdown();
  return 0;
}
