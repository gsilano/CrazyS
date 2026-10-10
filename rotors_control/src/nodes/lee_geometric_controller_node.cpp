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

// ROS 2 node for the Lee geometric controller.
//   in:  drone_state (rotors_comm/DroneState), odometry (nav_msgs/Odometry)
//   out: command/motor_speed (actuator_msgs/Actuators)
// One control step every Ts seconds of simulation time, triggered by the time stamps of
// the odometry messages (1 kHz) rather than by a ROS timer: the state is then sampled
// at regular times even when the computer is loaded and the messages arrive late.

#include <memory>

#include "rotors_control/lee_geometric_controller.h"
#include "ros_conversions.h"

namespace rotors_control {

class LeeGeometricControllerNode : public rclcpp::Node {
 public:
  LeeGeometricControllerNode() : Node("lee_geometric_controller_node") {
    InitializeParams();

    drone_state_sub_ = create_subscription<rotors_comm::msg::DroneState>(
        kDroneStateTopic, 1,
        std::bind(&LeeGeometricControllerNode::DroneStateCallback, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdometryTopic, 1,
        std::bind(&LeeGeometricControllerNode::OdometryCallback, this, std::placeholders::_1));
    motor_velocity_reference_pub_ = create_publisher<actuator_msgs::msg::Actuators>(
        kCommandMotorSpeedTopic, 1);

    RCLCPP_INFO(get_logger(), "Started, running at %.1f Hz (simulation time).",
                1.0 / controller_.controller_parameters_.sampling_time);
  }

 private:
  LeeGeometricController controller_;
  double next_control_time_ = -1.0;

  rclcpp::Subscription<rotors_comm::msg::DroneState>::SharedPtr drone_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_velocity_reference_pub_;

  void InitializeParams() {
    // Missing parameters keep the defaults of LeeGeometricControllerParameters.
    LeeGeometricControllerParameters& p = controller_.controller_parameters_;
    p.sampling_time = declare_parameter("sampling_time", p.sampling_time);
    p.gravity = declare_parameter("gravity", p.gravity);
    p.mass = declare_parameter("mass", p.mass);
    const double inertia_xx = declare_parameter("inertia.xx", p.inertia(0, 0));
    const double inertia_yy = declare_parameter("inertia.yy", p.inertia(1, 1));
    const double inertia_zz = declare_parameter("inertia.zz", p.inertia(2, 2));
    p.inertia = Eigen::Vector3d(inertia_xx, inertia_yy, inertia_zz).asDiagonal();
    p.arm_length = declare_parameter("arm_length", p.arm_length);
    p.motor_constant = declare_parameter("motor_constant", p.motor_constant);
    p.moment_constant = declare_parameter("moment_constant", p.moment_constant);
    p.max_rotor_velocity = declare_parameter("max_rotor_velocity", p.max_rotor_velocity);
    p.tau_first_order = declare_parameter("dirty_derivative.tau_first_order", p.tau_first_order);
    p.tau_higher_order =
        declare_parameter("dirty_derivative.tau_higher_order", p.tau_higher_order);
    p.k_x = declare_parameter("gains.position", p.k_x);
    p.k_v = declare_parameter("gains.velocity", p.k_v);
    p.k_R = declare_parameter("gains.attitude", p.k_R);
    p.k_omega = declare_parameter("gains.angular_rate", p.k_omega);

    if (p.sampling_time <= 0.0) {
      RCLCPP_ERROR(get_logger(), "The sampling time must be positive, using %f s.",
                   kDefaultLeeSamplingTime);
      p.sampling_time = kDefaultLeeSamplingTime;
    }
    controller_.InitializeParameters();
  }

  void DroneStateCallback(const rotors_comm::msg::DroneState& msg) {
    // Same check as the other CrazyS controllers: ignore references below the ground.
    if (msg.position.z < 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Ignoring a reference point below the ground.");
      return;
    }
    EigenDroneState reference;
    EigenDroneStateFromMsg(msg, &reference);
    // An empty (all zeros) quaternion means zero yaw.
    if (reference.orientation_W_B.norm() < 1e-6) {
      reference.orientation_W_B = Eigen::Quaterniond::Identity();
    }
    controller_.SetTrajectoryPoint(reference, rclcpp::Time(msg.header.stamp).seconds());
    RCLCPP_INFO_ONCE(get_logger(), "Got the first reference point.");
  }

  void OdometryCallback(const nav_msgs::msg::Odometry& msg) {
    const double time = rclcpp::Time(msg.header.stamp).seconds();
    EigenOdometry odometry;
    EigenOdometryFromMsg(msg, &odometry);
    controller_.SetOdometry(odometry, time);

    const double ts = controller_.controller_parameters_.sampling_time;
    if (next_control_time_ >= 0.0 && time < next_control_time_ - 1e-6) {
      return;
    }
    // Next step Ts seconds later. If messages were lost, restart from now.
    next_control_time_ = (next_control_time_ < 0.0 || time - next_control_time_ > ts)
                             ? time + ts : next_control_time_ + ts;

    Eigen::Vector4d rotor_velocities;
    controller_.CalculateRotorVelocities(&rotor_velocities);
    motor_velocity_reference_pub_->publish(ActuatorsMsg(rotor_velocities, msg.header.stamp));
  }
};

}  // namespace rotors_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rotors_control::LeeGeometricControllerNode>());
  rclcpp::shutdown();
  return 0;
}
