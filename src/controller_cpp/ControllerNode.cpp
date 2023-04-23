// Copyright 2023 (c) StressOverflow
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <utility>
#include <chrono>

#include "controller_cpp/ControllerNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"

#include "ds4_driver_msgs/msg/status.hpp"
#include "ds4_driver_msgs/msg/feedback.hpp"

#include "rclcpp/rclcpp.hpp"

namespace controller_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

ControllerNode::ControllerNode()
: Node("controller_node")
{
  declare_parameter<float>("max_linear_vel", 0.5f);
  declare_parameter<float>("max_angular_vel", 1.0f);
  declare_parameter<float>("controller_timeout", 0.25f);

  get_parameter("max_linear_vel", max_linear_vel_);
  get_parameter("max_angular_vel", max_angular_vel_);
  get_parameter("controller_timeout", controller_timeout_);

  max_linear_vel_ = std::clamp(std::abs(max_linear_vel_), 0.0f, ABS_MAX_LINEAR_VEL_);
  max_angular_vel_ = std::clamp(std::abs(max_angular_vel_), 0.0f, ABS_MAX_ANGULAR_VEL_);

  RCLCPP_INFO(get_logger(), "Max linear velocity: %f", max_linear_vel_);
  RCLCPP_INFO(get_logger(), "Max angular velocity: %f", max_angular_vel_);
  RCLCPP_INFO(get_logger(), "Controller timeout: %f", controller_timeout_);

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  feedback_pub_ = create_publisher<ds4_driver_msgs::msg::Feedback>("controller_feedback", 10);
  led_pub_1_ = create_publisher<kobuki_ros_interfaces::msg::Led>("kobuki_led_1", 10);
  led_pub_2_ = create_publisher<kobuki_ros_interfaces::msg::Led>("kobuki_led_2", 10);
  sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);

  controller_sub_ = create_subscription<ds4_driver_msgs::msg::Status>(
    "controller_status", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::controller_callback, this, _1));

  timer_ = create_wall_timer(25ms, std::bind(&ControllerNode::control_cycle, this));
}

void
ControllerNode::controller_callback(ds4_driver_msgs::msg::Status::UniquePtr msg)
{
  last_controller_status_ = std::move(msg);
}

void
ControllerNode::control_cycle()
{
  /*
   * Do nothing until the first controller read
   */
  if (last_controller_status_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist out_vel;
  ds4_driver_msgs::msg::Feedback controller_feedback;

  auto elapsed = now() - rclcpp::Time(last_controller_status_->header.stamp);

  controller_connected_ = elapsed < rclcpp::Duration::from_seconds(controller_timeout_);
  controller_enabled_ = controller_connected_ && last_controller_status_->button_cross;

  if (last_controller_connected_ != controller_connected_) {
    if (controller_connected_) {
      send_feedback(ControllerState::CONNECTED);
    } else {
      send_feedback(ControllerState::DISCONNECTED);
    }
  }

  if (last_controller_enabled_ != controller_enabled_) {
    if (controller_enabled_) {
      send_feedback(ControllerState::ENABLED);
    } else {
      send_feedback(ControllerState::DISABLED);
    }
  }

  last_controller_connected_ = controller_connected_;
  last_controller_enabled_ = controller_enabled_;

  if (last_c_state_ == ControllerState::DISABLED && now() - c_state_ts_ > 60s) {
    send_feedback(ControllerState::IDLE);
  }

  if (!controller_connected_ || !controller_enabled_) {
    out_vel.linear.x = 0.0;
    out_vel.angular.z = 0.0;
    vel_pub_->publish(out_vel);
    return;
  }

  float left_trigger = last_controller_status_->axis_l2;
  float right_trigger = last_controller_status_->axis_r2;
  float left_stick = last_controller_status_->axis_left_x;  // Left: 1.0, Right: -1.0

  bool no_move = left_trigger != 0.0 && right_trigger != 0.0;

  if (no_move) {
    out_vel.linear.x = 0.0;
  } else {
    /*
     * At this point, one of the two values is guaranteed to be zero.
     */
    out_vel.linear.x = value_map(right_trigger, 0.0, 1.0, 0.0, max_linear_vel_) + value_map(
      left_trigger, 0.0, 1.0, 0.0, -max_linear_vel_);
  }

  out_vel.angular.z = value_map(left_stick, -1.0, 1.0, -max_angular_vel_, max_angular_vel_);

  vel_pub_->publish(out_vel);

  RCLCPP_DEBUG(get_logger(), "Publishing: '%f' m/s", out_vel.linear.x);
  RCLCPP_DEBUG(get_logger(), "Publishing: '%f' rad/s", out_vel.angular.z);
}

float
ControllerNode::value_map(float value, float in_min, float in_max, float out_min, float out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void
ControllerNode::send_feedback(ControllerState state)
{
  send_controller_feedback(state);
  send_kobuki_feedback(state);

  c_state_ts_ = now();
  c_state_ = state;
  if (state != ControllerState::ERROR) {
    last_c_state_ = state;
  }
}

void
ControllerNode::send_controller_feedback(ControllerState state)
{
  switch (state) {
    case ControllerState::DISCONNECTED:
      controller_disconnected_feedback_c();
      break;
    case ControllerState::CONNECTED:
      controller_connected_feedback_c();
      break;
    case ControllerState::IDLE:
      controller_idle_feedback_c();
      break;
    case ControllerState::ENABLED:
      controller_enabled_feedback_c();
      break;
    case ControllerState::DISABLED:
      controller_disabled_feedback_c();
      break;
    case ControllerState::ERROR:
      controller_error_feedback_c();
      break;
    default:
      break;
  }
}

void
ControllerNode::send_kobuki_feedback(ControllerState state)
{
  switch (state) {
    case ControllerState::DISCONNECTED:
      controller_connected_feedback_k();
      break;
    case ControllerState::CONNECTED:
      controller_disconnected_feedback_k();
      break;
    case ControllerState::IDLE:
      controller_idle_feedback_k();
      break;
    case ControllerState::ENABLED:
      controller_enabled_feedback_k();
      break;
    case ControllerState::DISABLED:
      controller_disabled_feedback_k();
      break;
    case ControllerState::ERROR:
      controller_error_feedback_k();
      break;
    default:
      break;
  }
}

void
ControllerNode::controller_connected_feedback_c()
{
  ds4_driver_msgs::msg::Feedback controller_feedback;

  controller_feedback.set_rumble = true;
  controller_feedback.rumble_small = 1.0f;
  controller_feedback.rumble_duration = 1.0f;

  controller_feedback.set_led = true;
  controller_feedback.led_r = 0.0f;
  controller_feedback.led_g = 0.0f;
  controller_feedback.led_b = 255.0f;

  controller_feedback.set_led_flash = true;
  controller_feedback.led_flash_on = 0.0f;
  controller_feedback.led_flash_off = 0.0f;

  feedback_pub_->publish(controller_feedback);

  RCLCPP_INFO(get_logger(), "Controller connected");
}

void
ControllerNode::controller_disconnected_feedback_c()
{
  ds4_driver_msgs::msg::Feedback controller_feedback;

  controller_feedback.set_rumble = true;
  controller_feedback.rumble_big = 0.0f;
  controller_feedback.rumble_duration = 0.0f;

  controller_feedback.set_led = true;
  controller_feedback.led_r = 0.0f;
  controller_feedback.led_g = 0.0f;
  controller_feedback.led_b = 0.0f;

  controller_feedback.set_led_flash = true;
  controller_feedback.led_flash_on = 0.0f;
  controller_feedback.led_flash_off = 0.0f;

  feedback_pub_->publish(controller_feedback);

  RCLCPP_INFO(get_logger(), "Controller disconnected");
}

void
ControllerNode::controller_idle_feedback_c()
{
  ds4_driver_msgs::msg::Feedback controller_feedback;

  controller_feedback.set_rumble = true;
  controller_feedback.rumble_big = 0.0f;
  controller_feedback.rumble_duration = 0.0f;

  controller_feedback.set_led = true;
  controller_feedback.led_r = 0.0f;
  controller_feedback.led_g = 0.0f;
  controller_feedback.led_b = 60.0f;

  controller_feedback.set_led_flash = true;
  controller_feedback.led_flash_on = 0.0f;
  controller_feedback.led_flash_off = 0.0f;

  feedback_pub_->publish(controller_feedback);

  RCLCPP_INFO(get_logger(), "Controller idle");
}

void
ControllerNode::controller_enabled_feedback_c()
{
  ds4_driver_msgs::msg::Feedback controller_feedback;

  controller_feedback.set_rumble = true;
  controller_feedback.rumble_big = 1.0f;
  controller_feedback.rumble_duration = 0.25f;

  controller_feedback.set_led = true;
  controller_feedback.led_r = 0.0f;
  controller_feedback.led_g = 255.0f;
  controller_feedback.led_b = 0.0f;

  controller_feedback.set_led_flash = true;
  controller_feedback.led_flash_on = 0.0f;
  controller_feedback.led_flash_off = 0.0f;

  feedback_pub_->publish(controller_feedback);

  RCLCPP_INFO(get_logger(), "Controller enabled");
}

void
ControllerNode::controller_disabled_feedback_c()
{
  ds4_driver_msgs::msg::Feedback controller_feedback;

  controller_feedback.set_led = true;
  controller_feedback.led_r = 255.0f;
  controller_feedback.led_g = 140.0f;
  controller_feedback.led_b = 0.0f;

  controller_feedback.set_led_flash = true;
  controller_feedback.led_flash_on = 0.05f;
  controller_feedback.led_flash_off = 2.0f;

  feedback_pub_->publish(controller_feedback);

  RCLCPP_INFO(get_logger(), "Controller disabled");
}

void
ControllerNode::controller_error_feedback_c()
{
  ds4_driver_msgs::msg::Feedback controller_feedback;

  controller_feedback.set_rumble = true;
  controller_feedback.rumble_big = 1.0f;
  controller_feedback.rumble_duration = 0.5f;

  controller_feedback.set_led = true;
  controller_feedback.led_r = 255.0f;
  controller_feedback.led_g = 0.0f;
  controller_feedback.led_b = 0.0f;

  controller_feedback.set_led_flash = true;
  controller_feedback.led_flash_on = 0.25f;
  controller_feedback.led_flash_off = 0.25f;

  feedback_pub_->publish(controller_feedback);

  RCLCPP_INFO(get_logger(), "Controller error");
}

void
ControllerNode::controller_connected_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::GREEN;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::ORANGE;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::ON;

  led_pub_1_->publish(output_led_1);
  led_pub_2_->publish(output_led_2);

  sound_pub_->publish(output_sound);
}

void
ControllerNode::controller_disconnected_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::RED;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::BLACK;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::OFF;

  led_pub_1_->publish(output_led_1);
  led_pub_2_->publish(output_led_2);

  sound_pub_->publish(output_sound);
}

void
ControllerNode::controller_idle_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::ORANGE;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::BLACK;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::RECHARGE;

  led_pub_1_->publish(output_led_1);
  led_pub_2_->publish(output_led_2);

  sound_pub_->publish(output_sound);
}

void
ControllerNode::controller_enabled_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::GREEN;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::GREEN;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::CLEANINGSTART;

  led_pub_1_->publish(output_led_1);
  led_pub_2_->publish(output_led_2);

  sound_pub_->publish(output_sound);
}

void
ControllerNode::controller_disabled_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::GREEN;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::RED;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::CLEANINGEND;

  led_pub_1_->publish(output_led_1);
  led_pub_2_->publish(output_led_2);

  sound_pub_->publish(output_sound);
}

void
ControllerNode::controller_error_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::RED;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::RED;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::ERROR;

  led_pub_1_->publish(output_led_1);
  led_pub_2_->publish(output_led_2);

  sound_pub_->publish(output_sound);
}

}  // namespace controller_cpp
