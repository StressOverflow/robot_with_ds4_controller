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
        get_package_share_directory('map_with_controller'),
        'config',
        'params.yaml'
        )

    avoidObstacles_cmd = Node(package='map_with_controller',
                              executable='controller',
                              output='screen',
                              parameters=[{
                                'use_sim_time': False
                              }, params_file],
                              remappings=[
                                ('output_vel', '/cmd_vel'),
                                ('controller_status', '/status'),
                                ('controller_feedback', '/set_feedback'),
                              ])

    ld = LaunchDescription()
    ld.add_action(avoidObstacles_cmd)

    return ld
