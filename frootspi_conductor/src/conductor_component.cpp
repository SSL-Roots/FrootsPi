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
#include <string>
#include <utility>
#include <vector>

#include "frootspi_conductor/conductor_component.hpp"
#include "rclcpp/rclcpp.hpp"

namespace frootspi_conductor
{

Conductor::Conductor(const rclcpp::NodeOptions & options)
: Node("frootspi_conductor", options) {
  using namespace std::placeholders;  // for _1, _2, _3...

  sub_command_ = create_subscription<RobotCommand>(
    "command", 1, std::bind(&Conductor::callback_commands, this, _1));

  pub_target_velocity_ = create_publisher<geometry_msgs::msg::Twist>("target_velocity", 1);
  pub_dribble_power_ = create_publisher<frootspi_msgs::msg::DribblePower>("dribble_power", 1);
  pub_kick_power_ = create_publisher<std_msgs::msg::Float32>("kick_power", 1);
  pub_kick_flag_ = create_publisher<std_msgs::msg::Int16>("kick_flag", 1);
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

  // キックパワーが0より大きいとき、kick_powerとkick_flagをpublishする
  if (msg->kick_power > 0.0) {
    auto kick_power = std::make_unique<std_msgs::msg::Float32>();
    kick_power->data = msg->kick_power;
    pub_kick_power_->publish(std::move(kick_power));

    // キックフラグ
    auto kick_flag = std::make_unique<std_msgs::msg::Int16>();
    // 現状はチップキックかどうかだけ実装
    // 0: 未定, 1: ストレート, 2: チップ, 3: 放電
    if (msg->chip_kick) {
      kick_flag->data = 2;
    } else {
      kick_flag->data = 1;
    }
    pub_kick_flag_->publish(std::move(kick_flag));
  }

  // ドリブルパワー
  auto dribble_power = std::make_unique<frootspi_msgs::msg::DribblePower>();
  dribble_power->power = msg->dribble_power;
  pub_dribble_power_->publish(std::move(dribble_power));
}

}  // namespace frootspi_conductor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_conductor::Conductor)
