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
#include <stdio.h>

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
    "ball_detection", 1, std::bind(&KickerNode::callback_ball_detection, this, _1));

  sub_switch_state_ = create_subscription<frootspi_msgs::msg::SwitchesState>(
    "switches_state", 1, std::bind(&KickerNode::callback_switch_state, this, _1));

  // init servers
  srv_capacitor_charge_request_ = create_service<std_srvs::srv::SetBool>(
    "capacitor_charge_request", std::bind(&KickerNode::on_capacitor_charge_request, this, _1, _2));

  // init clients
  clnt_ball_detection_led_ = create_client<std_srvs::srv::SetBool>("set_center_led");

}

void KickerNode::callback_ball_detection(const frootspi_msgs::msg::BallDetection::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "ball detection received.");

  //calc
  if(msg->detected == true){
    ball_detection_ = true;
  } else {
    ball_detection_ = false;
  }

  // service request
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = ball_detection_;
  
  while (!clnt_ball_detection_led_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()){
    RCLCPP_INFO(this->get_logger(), "waiting for service to apper...");
  }

  auto result = clnt_ball_detection_led_->async_send_request(request, 
    std::bind(&KickerNode::callback_res_ball_led,this,std::placeholders::_1));

}

void KickerNode::callback_switch_state(const frootspi_msgs::msg::SwitchesState::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "switch status received.");

  // DipSWによる充電指示を確認
  if(msg->turned_on_dip0 == true ){
    charge_enable_from_dipsw_ = true;
  } else{
    charge_enable_from_dipsw_ = false;
  }

  // SWによる放電指示を確認
  // if(msg->pushed_button0 == true){
  //   // 放電は直接サービス呼んでもいいかも
  //   discharge_enable_from_sw_ = true;
  // }
}

void KickerNode::on_capacitor_charge_request(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (request->data == true) {
    response->message = "キャパシタ充電を許可しました";
    charge_enable_from_conductor_ = true;
  } else {
    response->message = "キャパシタ充電を不許可にしました";
    charge_enable_from_conductor_ = false;
  }
  response->success = true;
}


void KickerNode::callback_res_ball_led(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
{
  RCLCPP_DEBUG(this->get_logger(), future.get()->message);
}


}  // namespace frootspi_kicker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_kicker::KickerNode)
