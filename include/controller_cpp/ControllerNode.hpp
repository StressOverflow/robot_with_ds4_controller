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

#include "ds4_driver_msgs/msg/Status.msg"

#include "rclcpp/rclcpp.hpp"

namespace controller_cpp
{

using namespace std::chrono_literals;  // NOLINT

class AvoidObstacleNode : public rclcpp::Node
{
public:
  AvoidObstacleNode();

private:
  double yaw_time_;
  double back_time_;
  double dodge_time_;
  double scan_timeout_;

  float speed_linear_;
  float speed_angular_;
  float obstacle_distance_;

  const rclcpp::Duration TURNING_TIME {2s};
  const rclcpp::Duration BACKING_TIME {2s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 1.0f;
  static constexpr float OBSTACLE_DISTANCE = 1.0f;

  float speed_linear_factor_;
  float speed_angular_factor_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::WheelDropEvent>::SharedPtr wheel_drop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static const int INNIT = 0;
  static const int READY = 1;
  static const int FORWARD = 2;
  static const int BACK = 3;
  static const int YAW_TURN_IN = 4;
  static const int YAW_TURN_OUT = 5;
  static const int DODGE_TURN = 6;
  static const int STOP = 7;
  static const int EMERGENCY_STOP = 8;
  int state_;
  rclcpp::Time state_ts_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr last_button_event_;
  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bumper_event_;
  kobuki_ros_interfaces::msg::WheelDropEvent::UniquePtr last_wheel_drop_event_;

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);
  void wheel_drop_callback(kobuki_ros_interfaces::msg::WheelDropEvent::UniquePtr msg);
  void control_cycle();

  void log_parameters();
  void clean_emergencies();
  void go_state(int new_state);
  void manage_user_feedback(int new_state);
  void change_status_led(int new_state);
  void change_status_sound(int new_state);
  bool check_ready_2_forward();
  bool check_forward_2_yaw_turn_in();
  bool check_forward_2_stop();
  bool check_stop_2_forward();
  bool check_yaw_turn_in_2_dodge_turn();
  bool check_dodge_2_yaw_new_obstacle();
  bool check_dodge_2_yaw_turn_out();
  bool check_yaw_turn_out_2_forward();
  bool check_any_2_emergency_stop();
  bool check_emergency_2_ready();
  bool check_emergency_2_back();
  bool check_back_2_yaw_turn_in();
};

}  // namespace controller_cpp

#endif  // CONTROLLER_CPP__CONTROLLERNODE_HPP_
