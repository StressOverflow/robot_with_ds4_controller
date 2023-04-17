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

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<ds4_driver_msgs::msg::Status>::SharedPtr controller_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  ds4_driver_msgs::msg::Status::UniquePtr last_controller_status_;

  void controller_callback(ds4_driver_msgs::msg::Status::UniquePtr msg);
  void control_cycle();
};

}  // namespace controller_cpp

#endif  // CONTROLLER_CPP__CONTROLLERNODE_HPP_
