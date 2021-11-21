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

#ifndef FROOTSPI_KICKER__FROOTSPI_KICKER_COMPONENT_HPP_
#define FROOTSPI_KICKER__FROOTSPI_KICKER_COMPONENT_HPP_

#include <memory>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "frootspi_msgs/msg/battery_voltage.hpp"
#include "frootspi_msgs/msg/ball_detection.hpp"
#include "frootspi_msgs/msg/switches_state.hpp"
#include "frootspi_msgs/msg/kick_command.hpp"
#include "frootspi_msgs/srv/kick.hpp"
#include "frootspi_msgs/srv/set_kicker_charging.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"

#include "frootspi_kicker/visibility_control.h"

namespace frootspi_kicker
{

class KickerNode : public rclcpp::Node
{
public:
  FROOTSPI_KICKER_PUBLIC
  explicit KickerNode(const rclcpp::NodeOptions & options);

private:
  // publishers

  // subscribers
  rclcpp::Subscription<frootspi_msgs::msg::BallDetection>::SharedPtr sub_ball_detection_;
  rclcpp::Subscription<frootspi_msgs::msg::SwitchesState>::SharedPtr sub_switch_state_;
  rclcpp::Subscription<frootspi_msgs::msg::BatteryVoltage>::SharedPtr sub_kicker_voltage_;
  rclcpp::Subscription<frootspi_msgs::msg::KickCommand>::SharedPtr sub_kick_command_;
  // servers
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_capacitor_charge_request_;
  // clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr clnt_ball_detection_led_;
  rclcpp::Client<frootspi_msgs::srv::SetKickerCharging>::SharedPtr clnt_set_kicker_charging_;
  rclcpp::Client<frootspi_msgs::srv::Kick>::SharedPtr clnt_kick_;

  // subscription callbacks
  void callback_ball_detection(const frootspi_msgs::msg::BallDetection::SharedPtr msg);
  void callback_switch_state(const frootspi_msgs::msg::SwitchesState::SharedPtr msg);
  void callback_kicker_voltage(const frootspi_msgs::msg::BatteryVoltage::SharedPtr msg);
  void callback_kick_command(const frootspi_msgs::msg::KickCommand::SharedPtr msg);

  // servers callbacks
  void on_capacitor_charge_request(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  // client callbacks

  
  void set_kick(int set_kick_type_, float set_kick_power);
  void set_charge(bool charge_enable);

  // variable
  bool ball_detection_;
  bool ball_detection_pre_;
  bool charge_enable_from_conductor_;
  bool charge_restart_trigger_;
  bool charge_restart_status_;
  bool charge_restart_status_pre_;
  bool is_kicking_;
  bool is_release_;
  float capacitor_voltage_;
  bool set_charge_enable_;
  bool set_charge_enable_pre_;
  frootspi_msgs::msg::SwitchesState switches_state_;
};

}  // namespace frootspi_kicker

#endif  // FROOTSPI_KICKER__FROOTSPI_KICKER_COMPONENT_HPP_
