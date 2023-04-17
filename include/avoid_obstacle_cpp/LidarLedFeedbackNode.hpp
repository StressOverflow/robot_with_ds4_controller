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

#ifndef AVOID_OBSTACLE_CPP__LIDARLEDFEEDBACKNODE_HPP_
#define AVOID_OBSTACLE_CPP__LIDARLEDFEEDBACKNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lidar_led_feedback_cpp
{

class LidarLedFeedbackNode : public rclcpp::Node
{
public:
  LidarLedFeedbackNode();

private:
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();

  float obstacle_distance_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

}  // namespace lidar_led_feedback_cpp

#endif  // AVOID_OBSTACLE_CPP__LIDARLEDFEEDBACKNODE_HPP_
