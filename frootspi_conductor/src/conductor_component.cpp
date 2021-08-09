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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace frootspi_conductor
{

Conductor::Conductor(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("frootspi_conductor", options),
  my_id_(-1)
{
}

void Conductor::callback_commands(const frootspi_msgs::msg::FrootsPiCommands::SharedPtr msg)
{
  // コマンドの有無を判定
  for(auto command : msg->commands){
    // 自身のIDと一致しているか判定
    if(command.robot_id == my_id_){
      // publish
      // 速度
      auto target_velocity = std::make_unique<geometry_msgs::msg::Twist>();
      target_velocity->linear.x = command.vel_x;
      target_velocity->linear.y = command.vel_y;
      target_velocity->angular.z = command.vel_angular;
      pub_target_velocity_->publish(std::move(target_velocity));

      // ドリブルパワー
      auto dribble_power = std::make_unique<frootspi_msgs::msg::DribblePower>();
      dribble_power->power = command.dribble_power;
      pub_dribble_power_->publish(std::move(dribble_power));

      // キックパワー
      auto kick_power = std::make_unique<std_msgs::msg::Float32>();
      kick_power->data = command.kick_power;
      pub_kick_power_->publish(std::move(kick_power));

      // キックフラグ
      auto kick_flag = std::make_unique<std_msgs::msg::Int16>();
      // 現状はチップキックかどうかだけ実装
      // 0: 未定, 1: ストレート, 2: チップ, 3: 放電
      kick_flag->data = (int)(command.chip_enable) + 1;
      pub_kick_flag_->publish(std::move(kick_flag));

      break;
    }
  }

}

CallbackReturn Conductor::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  sub_frootspi_commands_ = create_subscription<frootspi_msgs::msg::FrootsPiCommands>(
    "frootspi_commands", 1, std::bind(&Conductor::callback_commands, this, _1));

  declare_parameter("my_id", -1);
  my_id_ = get_parameter("my_id").get_value<int>();

  pub_target_velocity_ = create_publisher<geometry_msgs::msg::Twist>("target_velocity", 1);
  pub_dribble_power_ = create_publisher<frootspi_msgs::msg::DribblePower>("dribble_power", 1);
  pub_kick_power_ = create_publisher<std_msgs::msg::Float32>("kick_power", 1);
  pub_kick_flag_ = create_publisher<std_msgs::msg::Int16>("kick_flag", 1);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Conductor::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  pub_target_velocity_->on_activate();
  pub_dribble_power_->on_activate();
  pub_kick_power_->on_activate();
  pub_kick_flag_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Conductor::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  pub_target_velocity_->on_deactivate();
  pub_dribble_power_->on_deactivate();
  pub_kick_power_->on_deactivate();
  pub_kick_flag_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Conductor::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  // pub_target_velocity_->reset();
  pub_dribble_power_.reset();
  pub_kick_power_.reset();
  pub_kick_flag_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Conductor::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  // pub_target_velocity_->reset();
  pub_dribble_power_.reset();
  pub_kick_power_.reset();
  pub_kick_flag_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace frootspi_conductor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_conductor::Conductor)
