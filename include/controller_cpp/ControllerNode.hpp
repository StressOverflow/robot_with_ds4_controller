// Copyright 2023 StressOverflow
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

  float max_linear_vel_;
  float max_angular_vel_;

  static constexpr float ABS_MAX_LINEAR_VEL_ = 1.0f;
  static constexpr float ABS_MAX_ANGULAR_VEL_ = 1.5f;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<ds4_driver_msgs::msg::Feedback>::SharedPtr feedback_pub_;
  rclcpp::Subscription<ds4_driver_msgs::msg::Status>::SharedPtr controller_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time controller_ts_;

  ds4_driver_msgs::msg::Status::UniquePtr last_controller_status_;

  void controller_callback(ds4_driver_msgs::msg::Status::UniquePtr msg);
  void control_cycle();

  float value_map(float value, float in_min, float in_max, float out_min, float out_max);

  void controller_connected_feedback();
  void controller_disconnected_feedback();
  void controller_enabled_feedback();
  void controller_disabled_feedback();
};

}  // namespace controller_cpp

#endif  // CONTROLLER_CPP__CONTROLLERNODE_HPP_
