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

#include "frootspi_hardware/driver_component.hpp"

#include <algorithm>
#include <memory>
#include <chrono>
#include <functional>
#include <iostream>
#include <utility>
#include <string>
#include <vector>
#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace frootspi_hardware
{

Driver::Driver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hardware_driver", options),
  pi_(-1)
{
}

void Driver::on_polling_timer()
{
  int switch_state = gpio_read(pi_, 19);

  if(switch_state >= 0){
    auto pub_sw_msg = std::make_unique<std_msgs::msg::Int16>();
    pub_sw_msg->data = switch_state;

    switch_state_pub_->publish(std::move(pub_sw_msg));
  } 
}

CallbackReturn Driver::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  polling_timer_ = create_wall_timer(1ms, std::bind(&Driver::on_polling_timer, this));
  // Don't actually start publishing data until activated
  polling_timer_->cancel();

  switch_state_pub_ = create_publisher<std_msgs::msg::Int16>("switch_state", 1);

  pi_ = pigpio_start(NULL, NULL);

  if(pi_ < 0){
    RCLCPP_ERROR(this->get_logger(), "Failed to connect pigpiod.");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  switch_state_pub_->on_activate();
  polling_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  switch_state_pub_->on_deactivate();
  polling_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  switch_state_pub_.reset();
  polling_timer_.reset();

  pigpio_stop(pi_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  switch_state_pub_.reset();
  polling_timer_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace frootspi_hardware

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_hardware::Driver)
