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

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "kobuki_ros_interfaces/msg/wheel_drop_event.hpp"
#include "kobuki_ros_interfaces/msg/cliff_event.hpp"

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
  declare_parameter<bool>("enable_leds", true);
  declare_parameter<bool>("enable_sounds", true);

  get_parameter("max_linear_vel", max_linear_vel_);
  get_parameter("max_angular_vel", max_angular_vel_);
  get_parameter("controller_timeout", controller_timeout_);
  get_parameter("enable_leds", enable_leds_);
  get_parameter("enable_sounds", enable_sounds_);

  max_linear_vel_ = std::clamp(std::abs(max_linear_vel_), 0.0f, ABS_MAX_LINEAR_VEL_);
  max_angular_vel_ = std::clamp(std::abs(max_angular_vel_), 0.0f, ABS_MAX_ANGULAR_VEL_);

  RCLCPP_INFO(get_logger(), "Max linear velocity: %f", max_linear_vel_);
  RCLCPP_INFO(get_logger(), "Max angular velocity: %f", max_angular_vel_);
  RCLCPP_INFO(get_logger(), "Controller timeout: %f", controller_timeout_);
  RCLCPP_INFO(get_logger(), "Enable LEDs: %s", enable_leds_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Enable sounds: %s", enable_sounds_ ? "true" : "false");

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
  feedback_pub_ = create_publisher<ds4_driver_msgs::msg::Feedback>("controller_feedback", 1);
  led_pub_1_ = create_publisher<kobuki_ros_interfaces::msg::Led>("kobuki_led_1", 1);
  led_pub_2_ = create_publisher<kobuki_ros_interfaces::msg::Led>("kobuki_led_2", 1);
  sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 1);

  controller_sub_ = create_subscription<ds4_driver_msgs::msg::Status>(
    "controller_status", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::controller_callback, this, _1));

  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "input_bumper", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::bumper_callback, this, _1));

  wheel_drop_sub_ = create_subscription<kobuki_ros_interfaces::msg::WheelDropEvent>(
    "input_wheel_drop", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::wheel_drop_callback, this, _1));
  
  cliff_sub_ = create_subscription<kobuki_ros_interfaces::msg::CliffEvent>(
    "input_cliff", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::cliff_callback, this, _1));

  timer_ = create_wall_timer(25ms, std::bind(&ControllerNode::control_cycle, this));

  warn_ts_ = now();
  e_stop_ts_ = now();
  c_state_ts_ = now();
}

void
ControllerNode::controller_callback(ds4_driver_msgs::msg::Status::UniquePtr msg)
{
  last_controller_status_ = std::move(msg);
}

void
ControllerNode::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  bumper_map_[msg->bumper] = msg->state;
}

void
ControllerNode::wheel_drop_callback(kobuki_ros_interfaces::msg::WheelDropEvent::UniquePtr msg)
{
  wheel_drop_map_[msg->wheel] = msg->state;
}

void
ControllerNode::cliff_callback(kobuki_ros_interfaces::msg::CliffEvent::UniquePtr msg)
{
  cliff_map_[msg->sensor] = msg->state;
}

void
ControllerNode::set_controller_state(ControllerState state)
{
  if (state == c_state_) {
    return;
  }

  prev_c_state_ = c_state_;
  c_state_ = state;
  c_state_ts_ = now();

  send_feedback(c_state_);
}

void
ControllerNode::set_emergency_stop(EmergencyStop stop)
{
  if (stop == last_e_stop_) {
    return;
  }

  e_stop_ = stop;
  last_e_stop_ = stop;
  e_stop_ts_ = now();
}

void
ControllerNode::check_for_emergency_stop()
{
  for (const auto& [key, value] : bumper_map_) {
    if (value == kobuki_ros_interfaces::msg::BumperEvent::PRESSED) {
      set_emergency_stop(EmergencyStop::BUMPER);
      break;
    }
  }

  for (const auto& [key, value] : wheel_drop_map_) {
    if (value == kobuki_ros_interfaces::msg::WheelDropEvent::DROPPED) {
      set_emergency_stop(EmergencyStop::WHEEL_DROP);
      break;
    }
  }

  for (const auto& [key, value] : cliff_map_) {
    if (value == kobuki_ros_interfaces::msg::CliffEvent::CLIFF) {
      set_emergency_stop(EmergencyStop::CLIFF);
      break;
    }
  }
}

void
ControllerNode::clean_emergency_stop(int32_t l1, int32_t r1)
{
  if (e_stop_ == EmergencyStop::NONE) {
    return;
  }

  if (l1 && r1) {
    set_emergency_stop(EmergencyStop::NONE);
  }
}

void
ControllerNode::es_warn_user(std::string warning_prompt)
{
  if ((now() - warn_ts_) < warning_prompt_timeout_) {
    return;
  }

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::ERROR;

  if (enable_sounds_) {
    sound_pub_->publish(output_sound);
  }

  RCLCPP_WARN(get_logger(), "[EMERGENCY STOP] %s", warning_prompt.c_str());

  warn_ts_ = now();
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

  auto elapsed = now() - rclcpp::Time(last_controller_status_->header.stamp);

  int32_t button_cross = last_controller_status_->button_cross;
  int32_t button_l1 = last_controller_status_->button_l1;
  int32_t button_r1 = last_controller_status_->button_r1;
  float left_trigger = last_controller_status_->axis_l2;
  float right_trigger = last_controller_status_->axis_r2;
  float left_stick = last_controller_status_->axis_left_x;  // Left: 1.0, Right: -1.0

  bool controller_connected = elapsed < rclcpp::Duration::from_seconds(controller_timeout_);
  bool controller_enabled = controller_connected && button_cross;

  if (c_state_ == ControllerState::ENABLED) {
    check_for_emergency_stop();
  }

  switch (c_state_)
  {
    case ControllerState::CONNECTED:
      if (!controller_connected) {
        set_controller_state(ControllerState::DISCONNECTED);
      }

      if (controller_enabled) {
        set_controller_state(ControllerState::ENABLED);
      }

      if ((now() - c_state_ts_) > idle_timeout_) {
        set_controller_state(ControllerState::IDLE);
      }

      stop_robot();
      break;
    case ControllerState::DISCONNECTED:
      if (controller_connected) {
        set_controller_state(ControllerState::CONNECTED);
      }

      stop_robot();
      break;
    case ControllerState::IDLE:
      if (!controller_connected) {
        set_controller_state(ControllerState::DISCONNECTED);
      }

      if (controller_enabled) {
        set_controller_state(ControllerState::ENABLED);
      }

      stop_robot();
      break;
    case ControllerState::ENABLED:
      if (!controller_connected) {
        set_controller_state(ControllerState::DISCONNECTED);
      }

      if (!controller_enabled) {
        set_controller_state(ControllerState::DISABLED);
      }

      if (e_stop_ != EmergencyStop::NONE) {
        set_controller_state(ControllerState::ERROR);
      }

      if (left_trigger > 0.0 && right_trigger > 0.0) {
        set_emergency_stop(EmergencyStop::BOTH_TRIGGERS);
        set_controller_state(ControllerState::ERROR);
      }

      drive_robot(left_stick, left_trigger, right_trigger);
      break;
    case ControllerState::DISABLED:
      if (!controller_connected) {
        set_controller_state(ControllerState::DISCONNECTED);
      }

      if (controller_enabled) {
        set_controller_state(ControllerState::ENABLED);
      }

      if ((now() - c_state_ts_) > idle_timeout_) {
        set_controller_state(ControllerState::IDLE);
      }

      stop_robot();
      break;
    case ControllerState::ERROR:
      if (e_stop_ == EmergencyStop::NONE) {
        set_controller_state(prev_c_state_);
      }

      switch (e_stop_)
      {
        case EmergencyStop::BUMPER:
        case EmergencyStop::CLIFF:
        case EmergencyStop::WHEEL_DROP:
          es_warn_user("Move the robot to a safe position and press L1 + R1 to clear.");
          clean_emergency_stop(button_l1, button_r1);
          stop_robot();
          break;
        case EmergencyStop::BOTH_TRIGGERS:
          es_warn_user("Release both triggers to clear.");
          if (left_trigger == 0.0 && right_trigger == 0.0) {
            set_emergency_stop(EmergencyStop::NONE);
          }
          stop_robot();
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void
ControllerNode::stop_robot()
{
  drive_robot(0.0, 0.0, 0.0);
}

void
ControllerNode::drive_robot(float left_stick, float left_trigger, float right_trigger)
{
  geometry_msgs::msg::Twist out_vel;

  out_vel.linear.x = value_map(right_trigger, 0.0, 1.0, 0.0, max_linear_vel_) + value_map(
    left_trigger, 0.0, 1.0, 0.0, -max_linear_vel_);

  out_vel.angular.z = value_map(left_stick, -1.0, 1.0, -max_angular_vel_, max_angular_vel_);

  vel_pub_->publish(out_vel);
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
}

void
ControllerNode::clean_controller_feedback()
{
  ds4_driver_msgs::msg::Feedback controller_feedback;

  controller_feedback.set_rumble = true;
  controller_feedback.rumble_small = 0.0f;
  controller_feedback.rumble_duration = 0.0f;

  controller_feedback.set_led = true;
  controller_feedback.led_r = 0.0f;
  controller_feedback.led_g = 0.0f;
  controller_feedback.led_b = 0.0f;

  controller_feedback.set_led_flash = true;
  controller_feedback.led_flash_on = 0.0f;
  controller_feedback.led_flash_off = 0.0f;

  feedback_pub_->publish(controller_feedback);
}

void
ControllerNode::send_controller_feedback(ControllerState state)
{
  clean_controller_feedback();

  switch (state) {
    case ControllerState::CONNECTED:
      controller_connected_feedback_c();
      break;
    case ControllerState::DISCONNECTED:
      controller_disconnected_feedback_c();
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
    case ControllerState::CONNECTED:
      controller_connected_feedback_k();
      break;
    case ControllerState::DISCONNECTED:
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
  controller_feedback.rumble_big = 0.5f;
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

  controller_feedback.set_rumble = true;
  controller_feedback.rumble_big = 0.5f;
  controller_feedback.rumble_duration = 0.50f;

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
  controller_feedback.rumble_duration = 0.75f;

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

  output_sound.value = kobuki_ros_interfaces::msg::Sound::CLEANINGEND;

  if (enable_leds_) {
    led_pub_1_->publish(output_led_1);
    led_pub_2_->publish(output_led_2);
  }

  if (enable_sounds_) {
    sound_pub_->publish(output_sound);
  }
}

void
ControllerNode::controller_disconnected_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::RED;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::BLACK;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::CLEANINGSTART;

  if (enable_leds_) {
    led_pub_1_->publish(output_led_1);
    led_pub_2_->publish(output_led_2);
  }

  if (enable_sounds_) {
    sound_pub_->publish(output_sound);
  }
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

  if (enable_leds_) {
    led_pub_1_->publish(output_led_1);
    led_pub_2_->publish(output_led_2);
  }

  if (enable_sounds_) {
    sound_pub_->publish(output_sound);
  }
}

void
ControllerNode::controller_enabled_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::GREEN;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::GREEN;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::ON;

  if (enable_leds_) {
    led_pub_1_->publish(output_led_1);
    led_pub_2_->publish(output_led_2);
  }

  if (enable_sounds_) {
    sound_pub_->publish(output_sound);
  }
}

void
ControllerNode::controller_disabled_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::GREEN;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::ORANGE;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::OFF;

  if (enable_leds_) {
    led_pub_1_->publish(output_led_1);
    led_pub_2_->publish(output_led_2);
  }

  if (enable_sounds_) {
    sound_pub_->publish(output_sound);
  }
}

void
ControllerNode::controller_error_feedback_k()
{
  kobuki_ros_interfaces::msg::Led output_led_1;
  kobuki_ros_interfaces::msg::Led output_led_2;

  kobuki_ros_interfaces::msg::Sound output_sound;

  output_led_1.value = kobuki_ros_interfaces::msg::Led::GREEN;
  output_led_2.value = kobuki_ros_interfaces::msg::Led::RED;

  output_sound.value = kobuki_ros_interfaces::msg::Sound::ERROR;

  if (enable_leds_) {
    led_pub_1_->publish(output_led_1);
    led_pub_2_->publish(output_led_2);
  }

  if (enable_sounds_) {
    sound_pub_->publish(output_sound);
  }
}

}  // namespace controller_cpp
