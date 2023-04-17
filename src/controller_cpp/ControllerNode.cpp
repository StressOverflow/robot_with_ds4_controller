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
#include "ds4_driver_msgs/msg/status.hpp"

#include "rclcpp/rclcpp.hpp"


namespace controller_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

ControllerNode::ControllerNode()
: Node("controllerNode")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);

  controller_sub_ = create_subscription<ds4_driver_msgs::msg::Status>(
    "controller_status", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::controller_callback, this, _1));
  
  timer_ = create_wall_timer(25ms, std::bind(&ControllerNode::control_cycle, this));

  controller_ts_ = now();
}

void
ControllerNode::controller_callback(ds4_driver_msgs::msg::Status::UniquePtr msg)
{
  last_controller_status_ = std::move(msg);
  controller_ts_ = now();
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

  if (now() - controller_ts_ > 25ms) {
    out_vel.linear.x = 0.0;
    out_vel.angular.z = 0.0;

    vel_pub_->publish(out_vel);

    return;
  }

  float left_trigger = last_controller_status_->axis_l2;
  float right_trigger = last_controller_status_->axis_r2;
  float left_stick = last_controller_status_->axis_left_x; // Left: 1.0, Right: -1.0

  
  bool no_move = left_trigger != 0.0 && right_trigger != 0.0 ||
                 left_trigger == 0.0 && right_trigger == 0.0;

  if (no_move) {
    out_vel.linear.x = 0.0;
  } else {
    /*
     * At this point, one of the two values is guaranteed to be zero.
     */
    out_vel.linear.x = value_map(right_trigger, 0.0, 1.0, 0.0, MAX_LINEAR_VEL_) + 
                       value_map(left_trigger, 0.0, 1.0, 0.0, -MAX_LINEAR_VEL_);
  }

  out_vel.angular.z = value_map(left_stick, -1.0, 1.0, -MAX_ANGULAR_VEL_, MAX_ANGULAR_VEL_);

  vel_pub_->publish(out_vel);
}

float
ControllerNode::value_map(float value, float in_min, float in_max, float out_min, float out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

}  // namespace controller_cpp
