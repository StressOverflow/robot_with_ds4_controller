// Copyright 2023 StressOverflow
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
#include "ds4_driver_msgs/msg/Status.msg"

#include "rclcpp/rclcpp.hpp"


namespace controller_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

AvoidObstacleNode::AvoidObstacleNode()
: Node("avoid_obstacle"),
  state_(INNIT)
{
  declare_parameter<double>("yaw_time", 7.5);
  declare_parameter<double>("back_time", 2.5);
  declare_parameter<double>("dodge_time", 20);
  declare_parameter<double>("scan_timeout", 1);

  declare_parameter<float>("speed_linear", 0.1f);
  declare_parameter<float>("speed_angular", 0.3f);
  declare_parameter<float>("obstacle_distance", 1.0f);

  declare_parameter<float>("speed_linear_factor", 0.75f);
  declare_parameter<float>("speed_angular_factor", 0.75f);

  get_parameter("yaw_time", yaw_time_);
  get_parameter("back_time", back_time_);
  get_parameter("dodge_time", dodge_time_);
  get_parameter("scan_timeout", scan_timeout_);

  get_parameter("speed_linear", speed_linear_);
  get_parameter("speed_angular", speed_angular_);
  get_parameter("obstacle_distance", obstacle_distance_);

  get_parameter("speed_linear_factor", speed_linear_factor_);
  get_parameter("speed_angular_factor", speed_angular_factor_);

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("status_led", 10);
  sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::scan_callback, this, _1));

  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::button_callback, this, _1));

  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "input_bumper", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::bumper_callback, this, _1));

  wheel_drop_sub_ = create_subscription<kobuki_ros_interfaces::msg::WheelDropEvent>(
    "input_wheel_drop", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::wheel_drop_callback, this, _1));

  timer_ = create_wall_timer(25ms, std::bind(&AvoidObstacleNode::control_cycle, this));

  state_ts_ = now();
}

void
AvoidObstacleNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
AvoidObstacleNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_event_ = std::move(msg);
}

void
AvoidObstacleNode::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  last_bumper_event_ = std::move(msg);
}

void
AvoidObstacleNode::wheel_drop_callback(kobuki_ros_interfaces::msg::WheelDropEvent::UniquePtr msg)
{
  last_wheel_drop_event_ = std::move(msg);
}


void
AvoidObstacleNode::control_cycle()
{
  /*
   * Do nothing until the first sensor read
   */
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case INNIT:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      RCLCPP_DEBUG(get_logger(), "INNIT -> READY");
      log_parameters();
      go_state(READY);
      break;
    case READY:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      if (check_ready_2_forward()) {
        RCLCPP_DEBUG(get_logger(), "READY -> FORWARD");
        go_state(FORWARD);
      }
      break;
    case FORWARD:
      out_vel.linear.x = speed_linear_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_DEBUG(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_forward_2_stop()) {
        RCLCPP_DEBUG(get_logger(), "FORWARD -> STOP");
        go_state(STOP);
      }
      if (check_forward_2_yaw_turn_in()) {
        RCLCPP_DEBUG(get_logger(), "FORWARD -> YAW TURN IN");
        go_state(YAW_TURN_IN);
      }
      break;
    case BACK:
      out_vel.linear.x = -speed_linear_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_DEBUG(get_logger(), "BACK -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_back_2_yaw_turn_in()) {
        RCLCPP_DEBUG(get_logger(), "BACK -> YAW TURN IN");
        go_state(YAW_TURN_IN);
      }
      break;
    case YAW_TURN_IN:
      out_vel.angular.z = speed_angular_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_DEBUG(get_logger(), "YAW TURN IN -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_yaw_turn_in_2_dodge_turn()) {
        RCLCPP_DEBUG(get_logger(), "YAW TURN IN -> DODGE TURN");
        go_state(DODGE_TURN);
      }
      break;
    case DODGE_TURN:
      out_vel.angular.z = -speed_angular_ * speed_angular_factor_;
      out_vel.linear.x = speed_linear_ * speed_linear_factor_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_DEBUG(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_dodge_2_yaw_new_obstacle()) {
        RCLCPP_DEBUG(get_logger(), "DODGE TURN -> YAW TURN IN");
        go_state(YAW_TURN_IN);
      }
      if (check_dodge_2_yaw_turn_out()) {
        RCLCPP_DEBUG(get_logger(), "DODGE TURN -> YAW TURN OUT");
        go_state(YAW_TURN_OUT);
      }
      break;
    case YAW_TURN_OUT:
      out_vel.angular.z = speed_angular_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_DEBUG(get_logger(), "YAW TURN OUT -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_yaw_turn_out_2_forward()) {
        RCLCPP_DEBUG(get_logger(), "YAW TURN OUT -> FORWARD");
        go_state(FORWARD);
      }
      break;
    case STOP:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      if (check_stop_2_forward()) {
        RCLCPP_DEBUG(get_logger(), "STOP -> FORWARD");
        go_state(FORWARD);
      }
      break;
    case EMERGENCY_STOP:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      if (check_emergency_2_back()) {
        clean_emergencies();
        RCLCPP_DEBUG(get_logger(), "EMERGENCY STOP -> BACK");
        go_state(BACK);
      }
      if (check_emergency_2_ready()) {
        clean_emergencies();
        RCLCPP_DEBUG(get_logger(), "EMERGENCY STOP -> READY");
        go_state(READY);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}

void
AvoidObstacleNode::log_parameters()
{
  RCLCPP_INFO(get_logger(), "PARAMETERS FOR THIS RUN");

  RCLCPP_INFO(get_logger(), "Yaw time: %f s", yaw_time_);
  RCLCPP_INFO(get_logger(), "Back time: %f s", back_time_);
  RCLCPP_INFO(get_logger(), "Dodge time: %f s", dodge_time_);
  RCLCPP_INFO(get_logger(), "Scan timeout: %f s", scan_timeout_);

  RCLCPP_INFO(get_logger(), "Linear speed: %f m/s", speed_linear_);
  RCLCPP_INFO(get_logger(), "Angular speed: %f m/s", speed_angular_);
  RCLCPP_INFO(get_logger(), "Obstacle distance: %f m", obstacle_distance_);

  RCLCPP_INFO(get_logger(), "Linear speed factor: %f", speed_linear_factor_);
  RCLCPP_INFO(get_logger(), "Angular speed factor: %f", speed_angular_factor_);
}

void
AvoidObstacleNode::clean_emergencies()
{
  last_bumper_event_ = nullptr;
  last_wheel_drop_event_ = nullptr;
}

void
AvoidObstacleNode::go_state(int new_state)
{
  state_ = new_state;
  manage_user_feedback(new_state);
  state_ts_ = now();
  RCLCPP_DEBUG(get_logger(), "CHANGED STATE");
}

void
AvoidObstacleNode::manage_user_feedback(int new_state)
{
  change_status_led(new_state);
  change_status_sound(new_state);
}

void
AvoidObstacleNode::change_status_led(int new_state)
{
  kobuki_ros_interfaces::msg::Led out_led;

  /*
   * The message is manipulated depending on the state that is
   * parsed. Some of them could have the same output, and thus the order matters.
   * This order may differ on the numerical order they have been defined.
   *
   * Then, whatever message was built, it is published.
   */

  switch (new_state) {
    case READY:
      out_led.value = kobuki_ros_interfaces::msg::Led::ORANGE;
      break;
    case BACK:
    case STOP:
    case EMERGENCY_STOP:
      out_led.value = kobuki_ros_interfaces::msg::Led::RED;
      break;
    case FORWARD:
    case YAW_TURN_IN:
    case YAW_TURN_OUT:
    case DODGE_TURN:
      out_led.value = kobuki_ros_interfaces::msg::Led::GREEN;
      break;
    /*
     * There may be cases in which you do not want to update
     * anything, so the method will return without publish anything.
     * Any other case should be contemplated on the switch statement.
     */
    default:
      return;
  }

  led_pub_->publish(out_led);
}

void
AvoidObstacleNode::change_status_sound(int new_state)
{
  kobuki_ros_interfaces::msg::Sound out_sound;

  /*
   * The message is manipulated depending on the state that is
   * parsed. Some of them could have the same output, and thus the order matters.
   * This order may differ on the numerical order they have been defined.
   *
   * Then, whatever message was built, it is published.
   */

  switch (new_state) {
    case READY:
    case FORWARD:
    case YAW_TURN_IN:
    case YAW_TURN_OUT:
    case DODGE_TURN:
      out_sound.value = kobuki_ros_interfaces::msg::Sound::ON;
      break;
    case BACK:
    case STOP:
    case EMERGENCY_STOP:
      out_sound.value = kobuki_ros_interfaces::msg::Sound::OFF;
      break;
    /*
     * There may be cases in which you do not want to update
     * anything, so the method will return without publish anything.
     * Any other case should be contemplated on the switch statement.
     */
    default:
      return;
  }

  sound_pub_->publish(out_sound);
}

bool
AvoidObstacleNode::check_ready_2_forward()
{
  if (last_button_event_ == nullptr) {return false;}

  return last_button_event_->state;
}

bool
AvoidObstacleNode::check_forward_2_yaw_turn_in()
{
  if (last_scan_ == nullptr) {return false;}

  size_t pos = 0;

  return last_scan_->ranges[pos] < obstacle_distance_;
}

bool
AvoidObstacleNode::check_forward_2_stop()
{
  /*
   * Stop if no sensor readings in a given timeout.
   */
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > rclcpp::Duration::from_seconds(scan_timeout_);
}

bool
AvoidObstacleNode::check_stop_2_forward()
{
  /*
   * Going forward if sensor readings are available again.
   */
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < rclcpp::Duration::from_seconds(scan_timeout_);
}

bool
AvoidObstacleNode::check_yaw_turn_in_2_dodge_turn()
{
  return (now() - state_ts_ ) > rclcpp::Duration::from_seconds(yaw_time_);
}

bool
AvoidObstacleNode::check_dodge_2_yaw_new_obstacle()
{
  /*
   * A new obstacle while dodging is nothing but the same
   * check procedure of a conventional obstacle. Thus the call to avoid
   * code duplication.
   */
  return check_forward_2_yaw_turn_in();
}

bool
AvoidObstacleNode::check_dodge_2_yaw_turn_out()
{
  return (now() - state_ts_ ) > rclcpp::Duration::from_seconds(dodge_time_);
}

bool
AvoidObstacleNode::check_yaw_turn_out_2_forward()
{
  /*
   * The yaw time in the "in" turn is the same as in the "out" turn,
   * and thus the call to avoid code duplication.
   */
  return check_yaw_turn_in_2_dodge_turn();
}

bool
AvoidObstacleNode::check_any_2_emergency_stop()
{
  /*
   * Before handling the emergency, the first step is to detect any.
   * But sometimes one of them could happen without the other.
   * So every possible emergency is tested and stored in a variable
   * before returned.
   *
   * Once only one emergency is true, it is inmediately returned to be handled.
   */
  bool result = false;

  if (last_bumper_event_ != nullptr) {result = last_bumper_event_->state;}

  if (result) {return true;}

  if (last_wheel_drop_event_ != nullptr) {result = last_wheel_drop_event_->state;}

  return result;
}

bool
AvoidObstacleNode::check_emergency_2_ready()
{
  if (last_wheel_drop_event_ == nullptr) {return false;}

  return !last_wheel_drop_event_->state;
}

bool
AvoidObstacleNode::check_emergency_2_back()
{
  if (last_bumper_event_ == nullptr) {return false;}
  /*
   * The following line must be added since the emergency could be a wheel drop,
   * however with the robot in the air, if the bumper is pressed, the robot returns
   * to the behaviour tree as usual.
   *
   * With a few emergency states this is no problem, but maybe when a lot of them could be
   * possible, a better aproach could be to raise a 'code' where every resume procedure
   * will check BEFORE attempting to clean an emergency caused by another procedure.
   *
   * For future versions.
   */
  if (last_wheel_drop_event_ != nullptr && last_wheel_drop_event_->state) {return false;}

  return last_bumper_event_->state;
}

bool
AvoidObstacleNode::check_back_2_yaw_turn_in()
{
  return (now() - state_ts_ ) > rclcpp::Duration::from_seconds(back_time_);
}

}  // namespace controller_cpp
