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
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declare_arg_robot_id = DeclareLaunchArgument(
        'id', default_value='11',
        description=('Set own ID.')
    )
    push_ns = PushRosNamespace(['robot', LaunchConfiguration('id')])

    gpio_config = os.path.join(
        get_package_share_directory('frootspi_hardware'),
        'config',
        'gpio.yaml'
    )

    container = ComposableNodeContainer(
        name='frootspi_container',
        namespace=['robot', LaunchConfiguration('id')],
        package='rclcpp_components',
        executable='component_container',  # component_container_mtはmulti threads
        sigterm_timeout='20',  # 終了時の放電時間だけCtrl+C入力後の猶予を設ける
        composable_node_descriptions=[
            ComposableNode(
                package='frootspi_hardware',
                plugin='frootspi_hardware::Driver',
                name='hardware_driver',
                parameters=[gpio_config],
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                package='frootspi_wheel',
                plugin='frootspi_wheel::WheelNode',
                name='wheel',
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                package='frootspi_conductor',
                plugin='frootspi_conductor::Conductor',
                name='frootspi_conductor',
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                package='frootspi_kicker',
                plugin='frootspi_kicker::KickerNode',
                name='kicker',
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
        ],
        output='screen',
    )

    start_pigpiod = ExecuteProcess(
        cmd=['sudo pigpiod -s 1'],  # サンプリングレートを1usecに変更
        shell=True,
        output='screen',
    )

    start_socket_can0 = ExecuteProcess(
        cmd=['sudo ip link set can0 up type can bitrate 1000000'],
        shell=True,
        output='screen',
    )

    configure_hardware_node = ExecuteProcess(
        cmd=[['ros2 lifecycle set robot', LaunchConfiguration('id'), '/hardware_driver configure']],
        shell=True,
        output='screen',
    )

    activate_hardware_node = ExecuteProcess(
        cmd=[['ros2 lifecycle set robot', LaunchConfiguration('id'), '/hardware_driver activate']],
        shell=True,
        output='screen',
    )

    return LaunchDescription([
        declare_arg_robot_id,
        push_ns,
        start_pigpiod,
        start_socket_can0,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=container,
                on_start=[configure_hardware_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=configure_hardware_node,
                on_exit=[activate_hardware_node],
            )
        ),
        container
        ])
