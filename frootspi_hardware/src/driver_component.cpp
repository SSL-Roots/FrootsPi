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


#include <pigpiod_if2.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "frootspi_hardware/driver_component.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

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
  auto ball_detection_msg = std::make_unique<frootspi_msgs::msg::BallDetection>();
  ball_detection_msg->detected = true;

  pub_ball_detection_->publish(std::move(ball_detection_msg));
}

void Driver::callback_dribble_power(const frootspi_msgs::msg::DribblePower::SharedPtr msg)
{
  std::cout << "ドリブルパワーは" << std::to_string(msg->power) << std::endl;
}

void Driver::callback_wheel_velocities(const frootspi_msgs::msg::WheelVelocities::SharedPtr msg)
{
  std::cout << "車輪目標回転速度は、左前" << std::to_string(msg->front_left);
  std::cout << "、真ん中後" << std::to_string(msg->back_center);
  std::cout << "、右前" << std::to_string(msg->back_center) << std::endl;;
}

void Driver::on_kick(const frootspi_msgs::srv::Kick::Request::SharedPtr request,
               frootspi_msgs::srv::Kick::Response::SharedPtr response)
{
  std::cout << "キックの種類:" << std::to_string(request->kick_type);
  std::cout << ", キックパワー:" << std::to_string(request->kick_power) << std::endl;

  response->success = true;
  response->message = "キック成功したで";
}


CallbackReturn Driver::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  polling_timer_ = create_wall_timer(1ms, std::bind(&Driver::on_polling_timer, this));
  // Don't actually start publishing data until activated
  polling_timer_->cancel();

  pub_ball_detection_ = create_publisher<frootspi_msgs::msg::BallDetection>("ball_detection", 1);
  pub_battery_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("battery_voltage", 1);
  pub_ups_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("ups_voltage", 1);
  pub_kicker_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("kicker_voltage", 1);
  pub_switches_state_ = create_publisher<frootspi_msgs::msg::SwitchesState>("switches_state", 1);
  pub_present_wheel_velocities_ = create_publisher<frootspi_msgs::msg::WheelVelocities>("present_wheel_velocities", 1);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 1);

  sub_dribble_power_ = create_subscription<frootspi_msgs::msg::DribblePower>(
    "dribble_power", 1, std::bind(&Driver::callback_dribble_power, this, _1));
  sub_target_wheel_velocities_ = create_subscription<frootspi_msgs::msg::WheelVelocities>(
    "target_wheel_velocities", 1, std::bind(&Driver::callback_wheel_velocities, this, _1));

  srv_kick_ = create_service<frootspi_msgs::srv::Kick>("kick", std::bind(&Driver::on_kick, this, _1, _2));

  pi_ = pigpio_start(NULL, NULL);

  if (pi_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect pigpiod.");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  pub_ball_detection_->on_activate();
  pub_battery_voltage_->on_activate();
  pub_ups_voltage_->on_activate();
  pub_kicker_voltage_->on_activate();
  pub_switches_state_->on_activate();
  pub_present_wheel_velocities_->on_activate();
  pub_imu_->on_activate();

  polling_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  pub_ball_detection_->on_deactivate();
  pub_battery_voltage_->on_deactivate();
  pub_ups_voltage_->on_deactivate();
  pub_kicker_voltage_->on_deactivate();
  pub_switches_state_->on_deactivate();
  pub_present_wheel_velocities_->on_deactivate();
  pub_imu_->on_deactivate();
  polling_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  pub_ball_detection_.reset();
  pub_battery_voltage_.reset();
  pub_ups_voltage_.reset();
  pub_kicker_voltage_.reset();
  pub_switches_state_.reset();
  pub_present_wheel_velocities_.reset();
  pub_imu_.reset();
  polling_timer_.reset();

  pigpio_stop(pi_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  pub_ball_detection_.reset();
  pub_battery_voltage_.reset();
  pub_ups_voltage_.reset();
  pub_kicker_voltage_.reset();
  pub_switches_state_.reset();
  pub_present_wheel_velocities_.reset();
  pub_imu_.reset();
  polling_timer_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace frootspi_hardware

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_hardware::Driver)
