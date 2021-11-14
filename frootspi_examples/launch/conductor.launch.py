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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declare_arg_robot_id = DeclareLaunchArgument(
        'id', default_value='0',
        description=('Set own ID.')
    )
    push_ns = PushRosNamespace(['robot', LaunchConfiguration('id')])

    # robot_id = LaunchConfiguration('robot_id')
    container = ComposableNodeContainer(
        name='frootspi_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',  # component_container_mtはmulti threads
        composable_node_descriptions=[
            ComposableNode(
                package='frootspi_conductor',
                plugin='frootspi_conductor::Conductor',
                name='frootspi_conductor',
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_arg_robot_id,
        push_ns,
        container
        ])
