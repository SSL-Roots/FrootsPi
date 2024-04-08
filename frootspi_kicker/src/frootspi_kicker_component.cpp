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


#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "frootspi_kicker/frootspi_kicker_component.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace frootspi_kicker
{

KickerNode::KickerNode(const rclcpp::NodeOptions & options)
: Node("kicker", options)
{
  using namespace std::placeholders;  // for _1, _2, _3...
  // init publishers

  // init subscribers
  sub_ball_detection_ = create_subscription<frootspi_msgs::msg::BallDetection>(
    "ball_detection", 10, std::bind(&KickerNode::callback_ball_detection, this, _1));
  sub_kicker_voltage_ = create_subscription<frootspi_msgs::msg::BatteryVoltage>(
    "kicker_voltage", 10, std::bind(&KickerNode::callback_kicker_voltage, this, _1));
  sub_kick_command_ = create_subscription<frootspi_msgs::msg::KickCommand>(
    "kick_command", 10, std::bind(&KickerNode::callback_kick_command, this, _1));

  // init servers
  srv_capacitor_charge_request_ = create_service<std_srvs::srv::SetBool>(
    "capacitor_charge_request", std::bind(&KickerNode::on_capacitor_charge_request, this, _1, _2));

  // init clients
  clnt_ball_detection_led_ = create_client<std_srvs::srv::SetBool>("set_center_led");
  clnt_set_kicker_charging_ = create_client<frootspi_msgs::srv::SetKickerCharging>(
    "set_kicker_charging");
  clnt_kick_ = create_client<frootspi_msgs::srv::Kick>("kick");

  // init param
  ball_detection_ = false;
  ball_detection_pre_ = false;
  charge_restart_status_ = false;
  charge_restart_status_pre_ = false;
  charge_enable_from_conductor_ = false;
  is_kicking_ = false;
  is_release_ = false;
  set_charge_enable_ = false;
  set_charge_enable_pre_ = false;
}

void KickerNode::set_kick(int set_kick_type_, float set_kick_power)
{
  // kick request
  if (clnt_kick_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_DEBUG(this->get_logger(), "kicer drive.");
    auto kick_request = std::make_shared<frootspi_msgs::srv::Kick::Request>();

    kick_request->kick_type = set_kick_type_;
    kick_request->kick_power = set_kick_power;

    is_kicking_ = true;

    auto kick_result = clnt_kick_->async_send_request(kick_request);
    is_kicking_ = false;
    is_release_ = true;
  }
}

void KickerNode::set_charge(bool charge_enable)
{
  // 充電指示が出ていて電圧が低いとき　再充電が必要なのか状態判定
  if ((capacitor_voltage_ < 190) && (charge_enable == true)) {
    charge_restart_status_ = true;
  } else {
    charge_restart_status_ = false;
  }

  // 再充電の状態判定よりトリガ生成
  if ((charge_restart_status_ == true) && (charge_restart_status_pre_ == false)) {
    charge_restart_trigger_ = true;
  } else {
    charge_restart_trigger_ = false;
  }

  charge_restart_status_pre_ = charge_restart_status_;

  if (charge_restart_trigger_ == true) {
    charge_enable = false;
  }

  set_charge_enable_ = charge_enable;
  // charge request
  if (set_charge_enable_ != set_charge_enable_pre_) {
    if (clnt_set_kicker_charging_->wait_for_service(std::chrono::seconds(1))) {
      auto charge_request = std::make_shared<frootspi_msgs::srv::SetKickerCharging::Request>();

      charge_request->start_charging = set_charge_enable_;

      auto charge_result = clnt_set_kicker_charging_->async_send_request(charge_request);
    }
  }
  set_charge_enable_pre_ = set_charge_enable_;
}

void KickerNode::callback_ball_detection(const frootspi_msgs::msg::BallDetection::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "ball detection received.");

  // calc
  if (msg->detected == true) {
    ball_detection_ = true;
  } else {
    ball_detection_ = false;
    is_release_ = false;
  }

  // service request
  if (ball_detection_ != ball_detection_pre_) {
    if (clnt_ball_detection_led_->wait_for_service(std::chrono::seconds(1))) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = ball_detection_;

      auto result = clnt_ball_detection_led_->async_send_request(request);
    }
  }
  ball_detection_pre_ = ball_detection_;
}

void KickerNode::callback_kicker_voltage(const frootspi_msgs::msg::BatteryVoltage::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "kicer voltage received.");
  capacitor_voltage_ = msg->voltage;
}

void KickerNode::callback_kick_command(const frootspi_msgs::msg::KickCommand::SharedPtr msg)
{
  // キック可否判定 (ボールがあって、電圧が190V以上で、前回のキックは正常に行われている)
  RCLCPP_INFO(this->get_logger(), "kicer request.");
  if ((ball_detection_ == true) &&
    (is_release_ == false) &&
    (is_kicking_ == false))
  {
    set_kick(msg->kick_type, msg->kick_power);
  }
}

void KickerNode::on_capacitor_charge_request(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  bool charge_enable;

  if (request->data == true) {
    response->message = "キャパシタ充電を許可しました";
    charge_enable_from_conductor_ = true;
  } else {
    response->message = "キャパシタ充電を不許可にしました";
    charge_enable_from_conductor_ = false;
  }

  if ((charge_enable_from_conductor_ == true) && (is_kicking_ == false))
  {
    charge_enable = true;
  } else {
    charge_enable = false;
  }
  // 充電要求判定
  set_charge(charge_enable);

  response->success = true;
}

}  // namespace frootspi_kicker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_kicker::KickerNode)
