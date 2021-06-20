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

#ifndef FROOTSPI_JOYCON__JOYCON_COMPONENT_HPP_
#define FROOTSPI_JOYCON__JOYCON_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "frootspi_joycon/visibility_control.h"
#include "frootspi_msgs/msg/froots_pi_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace frootspi_joycon
{

class JoyCon : public rclcpp_lifecycle::LifecycleNode
{
public:
  FROOTSPI_JOYCON_PUBLIC
  explicit JoyCon(const rclcpp::NodeOptions & options);

private:
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
};

}  // namespace frootspi_joycon

#endif  // FROOTSPI_JOYCON__JOYCON_COMPONENT_HPP_
