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

void KickerNode::callback_res_ball_led(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
{
  RCLCPP_DEBUG(this->get_logger(), future.get()->message);
}


}  // namespace frootspi_kicker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_kicker::KickerNode)
