# Copyright 2023 (c) StressOverflow
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

    controller_cmd = Node(package='map_with_controller',
                          executable='controller',
                          output='screen',
                          parameters=[{
                            'use_sim_time': False
                          }, params_file],
                          remappings=[
                            ('output_vel', '/cmd_vel'),
                            ('kobuki_led_1', '/commands/led1'),
                            ('kobuki_led_2', '/commands/led2'),
                            ('output_sound', '/commands/sound'),
                            ('input_bumper', '/events/bumper'),
                            ('input_wheel_drop', '/events/wheel_drop'),
                            ('input_cliff', '/events/cliff'),
                            ('controller_status', '/status'),
                            ('controller_feedback', '/set_feedback'),
                          ])

    ds4_driver = Node(package='ds4_driver',
                      executable='ds4_driver_node.py',
                      output='screen',
                      parameters=[{
                          'use_sim_time': False
                      }])

    ld = LaunchDescription()
    ld.add_action(controller_cmd)
    ld.add_action(ds4_driver)

    return ld
