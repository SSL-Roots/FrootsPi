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
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='frootspi_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',  # component_container_mtはmulti threads
        composable_node_descriptions=[
            ComposableNode(
                package='frootspi_hardware',
                plugin='frootspi_hardware::Driver',
                name='hardware_driver',
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                package='frootspi_joycon',
                plugin='frootspi_joycon::JoyCon',
                name='frootspi_joycon',
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
        ],
        output='screen',
    )

    configure_driver_node = ExecuteProcess(
        cmd=['ros2 lifecycle set hardware_driver configure'],
        shell=True,
        output='screen',
    )

    activate_driver_node = ExecuteProcess(
        cmd=['ros2 lifecycle set hardware_driver activate'],
        shell=True,
        output='screen',
    )

    configure_joycon_node = ExecuteProcess(
        cmd=['ros2 lifecycle set frootspi_joycon configure'],
        shell=True,
        output='screen',
    )

    activate_joycon_node = ExecuteProcess(
        cmd=['ros2 lifecycle set frootspi_joycon activate'],
        shell=True,
        output='screen',
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=container,
                on_start=[configure_driver_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=configure_driver_node,
                on_exit=[activate_driver_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=container,
                on_start=[configure_joycon_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=configure_joycon_node,
                on_exit=[activate_joycon_node],
            )
        ),
        container
        ])
