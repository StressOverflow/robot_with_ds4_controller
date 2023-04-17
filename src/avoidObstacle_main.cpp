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

#include <memory>

#include "avoid_obstacle_cpp/AvoidObstacleNode.hpp"
#include "avoid_obstacle_cpp/LidarLedFeedbackNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto avoidObstacle_node = std::make_shared<avoid_obstacle_cpp::AvoidObstacleNode>();
  auto lidarLedFeedback_node = std::make_shared<lidar_led_feedback_cpp::LidarLedFeedbackNode>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(avoidObstacle_node);
  executor.add_node(lidarLedFeedback_node);

  RCLCPP_INFO(avoidObstacle_node->get_logger(), "Init Avoid Obstacle Node");
  RCLCPP_INFO(lidarLedFeedback_node->get_logger(), "Init Lidar Led Feedback Node");

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
