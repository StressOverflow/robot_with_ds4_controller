# Copyright 2023 StressOverflow
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('avoid_obstacle_cpp'),
        'config',
        'params.yaml'
        )

    avoidObstacles_cmd = Node(package='avoid_obstacle_cpp',
                              executable='avoidObstacle',
                              output='screen',
                              parameters=[{
                                'use_sim_time': True
                              }, params_file],
                              remappings=[
                                ('input_scan', '/scan_filtered'),
                                ('output_vel', '/cmd_vel'),
                                ('input_button', '/events/button'),
                                ('lidar_led', '/commands/led2'),
                                ('status_led', '/commands/led1'),
                                ('output_sound', '/commands/sound'),
                                ('input_bumper', '/events/bumper'),
                                ('input_wheel_drop', '/events/wheel_drop')
                              ])

    ld = LaunchDescription()
    ld.add_action(avoidObstacles_cmd)

    return ld
