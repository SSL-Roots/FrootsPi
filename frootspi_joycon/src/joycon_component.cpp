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

#include "frootspi_joycon/joycon_component.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace frootspi_joycon
{

JoyCon::JoyCon(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("frootspi_joycon", options)
{
}

CallbackReturn JoyCon::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  pub_command_ = create_publisher<frootspi_msgs::msg::FrootsPiCommand>("command", 1);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyCon::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  pub_command_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyCon::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  pub_command_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyCon::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  pub_command_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyCon::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  pub_command_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace frootspi_joycon

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_joycon::JoyCon)
