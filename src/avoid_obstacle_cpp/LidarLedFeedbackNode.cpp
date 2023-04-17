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

#include "avoid_obstacle_cpp/LidarLedFeedbackNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"

#include "rclcpp/rclcpp.hpp"

namespace lidar_led_feedback_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

LidarLedFeedbackNode::LidarLedFeedbackNode()
: Node("lidar_led_feedback")
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&LidarLedFeedbackNode::scan_callback, this, _1));

  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("lidar_led", 10);
  timer_ = create_wall_timer(50ms, std::bind(&LidarLedFeedbackNode::control_cycle, this));

  declare_parameter<float>("obstacle_distance", 1.0f);

  get_parameter("obstacle_distance", obstacle_distance_);
}
void
LidarLedFeedbackNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
LidarLedFeedbackNode::control_cycle()
{
  kobuki_ros_interfaces::msg::Led out_led;

  out_led.value = kobuki_ros_interfaces::msg::Led::BLACK;

  /*
   * Do nothing until the first sensor read
   */
  if (last_scan_ == nullptr) {
    led_pub_->publish(out_led);
    return;
  }

  size_t pos = 0;

  if (last_scan_->ranges[pos] < obstacle_distance_) {
    out_led.value = kobuki_ros_interfaces::msg::Led::RED;
  } else {
    out_led.value = kobuki_ros_interfaces::msg::Led::GREEN;
  }

  led_pub_->publish(out_led);
}

}  // namespace lidar_led_feedback_cpp
