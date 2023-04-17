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
   * Do nothing until the first sensor read
   */
  if (last_controller_status_ == nullptr) {
    return;
  }

  //geometry_msgs::msg::Twist out_vel;
  if(last_controller_status_->button_cross){
    RCLCPP_INFO(this->get_logger(), "Cross");
  }

  //vel_pub_->publish(out_vel);
}

}  // namespace controller_cpp
