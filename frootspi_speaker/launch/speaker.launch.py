# Copyright 2023 Roots
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
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    voices_path = os.path.join(
        get_package_share_directory('frootspi_speaker'),
        'voices'
        )

    declare_voices_path = DeclareLaunchArgument(
        'voices_path', default_value=voices_path,
        description=('Set voice file directory.')
    )

    declare_file_list_name = DeclareLaunchArgument(
        'file_list_name', default_value="voice_file_list.csv",
        description=('Set voice file list name.')
    )

    speaker = Node(
        package='frootspi_speaker',
        executable='speaker.py',
        output='screen',
        parameters=[{
            'voices_path': LaunchConfiguration('voices_path'),
            'file_list_name': LaunchConfiguration('file_list_name'),
                     }]
    )

    return launch.LaunchDescription([
        declare_voices_path,
        declare_file_list_name,
        speaker
    ])