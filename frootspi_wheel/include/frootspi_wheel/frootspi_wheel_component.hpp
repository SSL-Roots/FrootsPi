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

#ifndef FROOTSPI_WHEEL__FROOTSPI_WHEEL_COMPONENT_HPP_
#define FROOTSPI_WHEEL__FROOTSPI_WHEEL_COMPONENT_HPP_


#include "frootspi_msgs/msg/wheel_velocities.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "frootspi_wheel/visibility_control.h"

namespace frootspi_wheel
{

class WheelNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    FROOTSPI_WHEEL_PUBLIC
    explicit WheelNode(const rclcpp::NodeOptions & options);

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

  // publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::WheelVelocities>>
  pub_wheel_velocities_;

  // subscribers
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>>
  sub_target_velocity_;

  // subscription callbacks
  void callback_target_velocity(const geometry_msgs::msg::Twist::SharedPtr msg);
};


class WheelVector
{
public:
  void robotVelToWheelRotateVels(const double & vx, const double & vy, const double & vw, double * v0, double * v1, double * v2);
  
private:
  const double CONST_V0_COEFFICIENT_VX_ = -0.5,       CONST_V0_COEFFICIENT_VY_ = 0.8660254;
  const double CONST_V1_COEFFICIENT_VX_ = -0.5,       CONST_V1_COEFFICIENT_VY_ = -0.8660254;
  const double CONST_V2_COEFFICIENT_VX_ = 1,          CONST_V2_COEFFICIENT_VY_ = 0;
  const double CONST_MACHINE_RADIUS_  = 0.07;
  const double CONST_WHEEL_RADIUS_    = 28;
  const double CONST_GEAR_RATIO_      = 2.4;
};

}  // namespace frootspi_wheel

#endif  // FROOTSPI_WHEEL__FROOTSPI_WHEEL_COMPONENT_HPP_
