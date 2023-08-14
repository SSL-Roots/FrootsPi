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
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode

import yaml
from pathlib import Path

DEFAULT_FILE_PATH = Path.home() / Path("robot_config.yaml")


def get_configuration_from_file(file_path):
    # TODO: 引数でコンフィグファイルのパスを指定する

    # yamlで書かれた設定ファイルから値を取得する
    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)
        print(f"Read robot config from {file_path}")
    return config


def generate_launch_description():
    # configからRobot IDを取得
    config = get_configuration_from_file(DEFAULT_FILE_PATH)
    robot_id = config['robot_id']

    push_ns = PushRosNamespace(['robot', str(robot_id)])

    gpio_config = os.path.join(
        get_package_share_directory('frootspi_hardware'),
        'config',
        'gpio.yaml'
    )

    container = ComposableNodeContainer(
        name='frootspi_container',
        namespace=['robot', str(robot_id)],
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

    start_socket_can0 = ExecuteProcess(
        cmd=['sudo ip link set can0 up type can bitrate 1000000'],
        shell=True,
        output='screen',
    )

    speaker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('frootspi_speaker'),
            '/launch/speaker.launch.py'])
    )

    return LaunchDescription([
        push_ns,
        start_socket_can0,
        container,
        speaker
        ])
