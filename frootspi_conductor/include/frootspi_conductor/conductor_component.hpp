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

#ifndef FROOTSPI_CONDUCTOR__CONDUCTOR_COMPONENT_HPP_
#define FROOTSPI_CONDUCTOR__CONDUCTOR_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "frootspi_conductor/visibility_control.h"
#include "frootspi_msgs/msg/froots_pi_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "frootspi_msgs/msg/dribble_power.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace frootspi_conductor
{

using RobotCommand = consai_frootspi_msgs::msg::RobotCommand;

class Conductor : public rclcpp_lifecycle::LifecycleNode
{
public:
  FROOTSPI_CONDUCTOR_PUBLIC
  explicit Conductor(const rclcpp::NodeOptions & options);

private:
  void callback_commands(const RobotCommand::SharedPtr msg);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

  std::shared_ptr<rclcpp::Subscription<RobotCommand>>
  sub_command_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>>
  pub_target_velocity_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::DribblePower>>
  pub_dribble_power_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>>
  pub_kick_power_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>>
  pub_kick_flag_;

  int kick_command_;
};

}  // namespace frootspi_conductor

#endif  // FROOTSPI_CONDUCTOR__CONDUCTOR_COMPONENT_HPP_
