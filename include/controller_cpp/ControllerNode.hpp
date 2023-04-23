// Copyright 2023 (c) StressOverflow
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROLLER_CPP__CONTROLLERNODE_HPP_
#define CONTROLLER_CPP__CONTROLLERNODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"

#include "ds4_driver_msgs/msg/status.hpp"
#include "ds4_driver_msgs/msg/feedback.hpp"

#include "rclcpp/rclcpp.hpp"

namespace controller_cpp
{

using namespace std::chrono_literals;  // NOLINT

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:
  bool controller_connected_ = false;
  bool last_controller_connected_ = false;
  bool controller_enabled_ = false;
  bool last_controller_enabled_ = false;

  enum class ControllerState
  {
    DISCONNECTED,
    CONNECTED,
    IDLE,
    ENABLED,
    DISABLED,
    ERROR
  };

  ControllerState c_state_ = ControllerState::DISCONNECTED;
  ControllerState last_c_state_ = ControllerState::DISCONNECTED;
  rclcpp::Time c_state_ts_;

  float max_linear_vel_;
  float max_angular_vel_;
  float controller_timeout_;

  static constexpr float ABS_MAX_LINEAR_VEL_ = 1.0f;
  static constexpr float ABS_MAX_ANGULAR_VEL_ = 1.5f;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<ds4_driver_msgs::msg::Feedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_1_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_2_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;

  rclcpp::Subscription<ds4_driver_msgs::msg::Status>::SharedPtr controller_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  ds4_driver_msgs::msg::Status::UniquePtr last_controller_status_;

  void controller_callback(ds4_driver_msgs::msg::Status::UniquePtr msg);
  void control_cycle();

  float value_map(float value, float in_min, float in_max, float out_min, float out_max);

  void send_feedback(ControllerState);

  void send_controller_feedback(ControllerState);
  void send_kobuki_feedback(ControllerState);

  void controller_connected_feedback_c();
  void controller_disconnected_feedback_c();
  void controller_idle_feedback_c();
  void controller_enabled_feedback_c();
  void controller_disabled_feedback_c();
  void controller_error_feedback_c();

  void controller_connected_feedback_k();
  void controller_disconnected_feedback_k();
  void controller_idle_feedback_k();
  void controller_enabled_feedback_k();
  void controller_disabled_feedback_k();
  void controller_error_feedback_k();
};

}  // namespace controller_cpp

#endif  // CONTROLLER_CPP__CONTROLLERNODE_HPP_
