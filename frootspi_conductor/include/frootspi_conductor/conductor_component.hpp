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

#include "frootspi_conductor/visibility_control.h"
#include "frootspi_msgs/msg/froots_pi_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace frootspi_conductor
{

class Conductor : public rclcpp_lifecycle::LifecycleNode
{
public:
  FROOTSPI_CONDUCTOR_PUBLIC
  explicit Conductor(const rclcpp::NodeOptions & options);

private:
  void callback_commands(const frootspi_msgs::msg::FrootsPiCommands::SharedPtr msg);

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

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::FrootsPiCommand>>
  pub_command_;
  std::shared_ptr<rclcpp::Subscription<frootspi_msgs::msg::FrootsPiCommands>>
  sub_frootspi_commands_;

  int my_id_;
};

}  // namespace frootspi_conductor

#endif  // FROOTSPI_CONDUCTOR__CONDUCTOR_COMPONENT_HPP_
