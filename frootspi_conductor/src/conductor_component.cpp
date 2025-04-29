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
#include "rclcpp/qos.hpp"

namespace frootspi_conductor
{
using namespace std::chrono_literals;

Conductor::Conductor(const rclcpp::NodeOptions & options)
: Node("frootspi_conductor", options)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  sub_command_ = create_subscription<RobotCommand>(
    "command",
    rclcpp::QoS(10).best_effort(),
    std::bind(&Conductor::callback_commands, this, _1));

  pub_target_velocity_ = create_publisher<geometry_msgs::msg::Twist>("target_velocity", 10);
  pub_dribble_power_ = create_publisher<frootspi_msgs::msg::DribblePower>("dribble_power", 10);
  pub_kick_command_ = create_publisher<frootspi_msgs::msg::KickCommand>("kick_command", 10);
  client_charge_request_ = create_client<std_srvs::srv::SetBool>("capacitor_charge_request");

  // 充電許可サービスを定期的にコールする関数
  polling_timer_ = create_wall_timer(5s, std::bind(&Conductor::on_polling_timer, this));

  // 通信タイムアウト検知用のタイマー
  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);
  sub_command_timestamp_ = steady_clock_.now();
  timeout_has_printed_ = false;
}

void Conductor::on_polling_timer()
{
  // CON-SAIからのコマンドのタイムアウト処理
  bool enable_charge = false;
  if (steady_clock_.now().seconds() - sub_command_timestamp_.seconds() >= 5.0) {
    // 通信タイムアウト
    if (timeout_has_printed_ == false) {
      RCLCPP_WARN(
        this->get_logger(),
        "通信タイムアウトのため、キャパシタの充電を停止させます");
      timeout_has_printed_ = true;
    }
    enable_charge = false;
  } else {
    timeout_has_printed_ = false;
    enable_charge = true;
  }

  // 充電許可サービスをコールする
  if (client_charge_request_->wait_for_service(std::chrono::seconds(1))) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable_charge;
    auto result = client_charge_request_->async_send_request(request);
  }
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

  // 通信タイムアウト処理用に、タイムスタンプを取得する
  sub_command_timestamp_ = steady_clock_.now();
}

}  // namespace frootspi_conductor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_conductor::Conductor)
