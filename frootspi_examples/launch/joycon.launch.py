# Copyright 2021 Roots
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joydev = LaunchConfiguration('joydev')
    declare_joydev = DeclareLaunchArgument(
        'joydev', default_value='/dev/input/js0',
        description='Device file for JoyStick Controller'
    )

    joy_config = os.path.join(
        get_package_share_directory('frootspi_joycon'),
        'config',
        'dualshock3.yaml'
    )

    joycon_node = Node(
        package='frootspi_joycon',
        executable='joycon.py',
        parameters=[joy_config],
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev': joydev}]
    )

    return LaunchDescription([
        declare_joydev,
        joy_node,
        joycon_node
        ])
