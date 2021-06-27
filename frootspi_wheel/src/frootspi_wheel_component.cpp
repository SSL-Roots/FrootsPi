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
#include <utility>
#include <vector>

#include "frootspi_msgs/msg/wheel_velocities.hpp"
#include "frootspi_wheel/frootspi_wheel_component.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace frootspi_wheel
{

WheelNode::WheelNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("wheel", options)
{
}


CallbackReturn WheelNode::on_configure(const rclcpp_lifecycle::State &)
{  
  RCLCPP_DEBUG(this->get_logger(), "on_configure() is called.");

  using namespace std::placeholders;  // for _1, _2, _3...

  // init publishers
  pub_wheel_velocities_ = create_publisher<frootspi_msgs::msg::WheelVelocities>(
    "wheel_velocities", 1);

  // init subscribers
  sub_target_velocity_ = create_subscription<geometry_msgs::msg::Twist>(
  "target_velocity", 1, std::bind(&WheelNode::callback_target_velocity, this, _1));

  std::cout << "Configured!" << std::endl;

  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "on_activate() is called.");

  pub_wheel_velocities_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "on_deactivate() is called.");

  pub_wheel_velocities_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "on_cleanup() is called.");

  pub_wheel_velocities_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "on_shutdown() is called.");

  pub_wheel_velocities_.reset();
  return CallbackReturn::SUCCESS;
}

void WheelNode::callback_target_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "target velocity received.");

  // calc
  double v0, v1, v2;
  WheelVector wv;
  wv.robotVelToWheelRotateVels(msg->linear.x, msg->linear.y, msg->angular.z, &v0, &v1, &v2);

  // publish
  auto wheel_velocities_msg = std::make_unique<frootspi_msgs::msg::WheelVelocities>();
  wheel_velocities_msg->front_left = v0;
  wheel_velocities_msg->front_right= v1;
  wheel_velocities_msg->back_center= v2;
  pub_wheel_velocities_->publish(std::move(wheel_velocities_msg));
}


void WheelVector::robotVelToWheelRotateVels(const double & vx, const double & vy, const double & vw, double * v0, double * v1, double * v2)
{
  /*
  * ロボット速度から各車輪モータの回転速度に変換する
  */

  double v0_mps= (CONST_V0_COEFFICIENT_VX_ * vx) + (CONST_V0_COEFFICIENT_VY_ * vy) + (CONST_MACHINE_RADIUS_ * vw );
  double v1_mps= (CONST_V1_COEFFICIENT_VX_ * vx) + (CONST_V1_COEFFICIENT_VY_ * vy) + (CONST_MACHINE_RADIUS_ * vw );
  double v2_mps= (CONST_V2_COEFFICIENT_VX_ * vx) + (CONST_V2_COEFFICIENT_VY_ * vy) + (CONST_MACHINE_RADIUS_ * vw );

  double v0_radps = v0_mps / CONST_WHEEL_RADIUS_ * CONST_GEAR_RATIO_;
  double v1_radps = v1_mps / CONST_WHEEL_RADIUS_ * CONST_GEAR_RATIO_;
  double v2_radps = v2_mps / CONST_WHEEL_RADIUS_ * CONST_GEAR_RATIO_;

  *v0 = v0_radps;
  *v1 = v1_radps;
  *v2 = v2_radps;
}


}  // namespace frootspi_wheel

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_wheel::WheelNode)
