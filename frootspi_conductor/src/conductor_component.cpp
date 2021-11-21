// Copyright 2021 Roots
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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "frootspi_conductor/conductor_component.hpp"
#include "rclcpp/rclcpp.hpp"

namespace frootspi_conductor
{

Conductor::Conductor(const rclcpp::NodeOptions & options)
: Node("frootspi_conductor", options)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  sub_command_ = create_subscription<RobotCommand>(
    "command", 10, std::bind(&Conductor::callback_commands, this, _1));

  pub_target_velocity_ = create_publisher<geometry_msgs::msg::Twist>("target_velocity", 10);
  pub_dribble_power_ = create_publisher<frootspi_msgs::msg::DribblePower>("dribble_power", 10);
  pub_kick_command_ = create_publisher<frootspi_msgs::msg::KickCommand>("kick_command", 10);
}

void Conductor::callback_commands(const RobotCommand::SharedPtr msg)
{
  // RobotCommandをFrootsPiの各ハードウェア向けのトピックに分解してPublishする
  // 速度
  auto target_velocity = std::make_unique<geometry_msgs::msg::Twist>();
  target_velocity->linear.x = msg->velocity_x;
  target_velocity->linear.y = msg->velocity_y;
  target_velocity->angular.z = msg->velocity_theta;
  pub_target_velocity_->publish(std::move(target_velocity));

  // キックパワーが0より大きいとき、kick_commandをpublishする
  if (msg->kick_power > 0.0) {
    auto kick_command = std::make_unique<frootspi_msgs::msg::KickCommand>();
    kick_command->kick_power = msg->kick_power;

    // 0: 未定, 1: ストレート, 2: チップ, 3: 放電
    if (msg->chip_kick) {
      kick_command->kick_type = frootspi_msgs::msg::KickCommand::KICK_TYPE_CHIP;
    } else {
      kick_command->kick_type = frootspi_msgs::msg::KickCommand::KICK_TYPE_STRAIGHT;
    }
    pub_kick_command_->publish(std::move(kick_command));
  }

  // ドリブルパワー
  auto dribble_power = std::make_unique<frootspi_msgs::msg::DribblePower>();
  dribble_power->power = msg->dribble_power;
  pub_dribble_power_->publish(std::move(dribble_power));
}

}  // namespace frootspi_conductor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_conductor::Conductor)
