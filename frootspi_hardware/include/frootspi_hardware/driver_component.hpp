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

#ifndef FROOTSPI_HARDWARE__DRIVER_COMPONENT_HPP_
#define FROOTSPI_HARDWARE__DRIVER_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "frootspi_hardware/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "frootspi_msgs/msg/ball_detection.hpp"

namespace frootspi_hardware
{

class Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  FROOTSPI_HARDWARE_PUBLIC
  explicit Driver(const rclcpp::NodeOptions & options);

protected:
  void on_polling_timer();

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

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::BallDetection>> pub_ball_detection_;

  rclcpp::TimerBase::SharedPtr polling_timer_;

  int pi_;
};

}  // namespace frootspi_hardware

#endif  // FROOTSPI_HARDWARE__DRIVER_COMPONENT_HPP_
