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

#include <memory>

#include "frootspi_msgs/msg/wheel_velocities.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "frootspi_wheel/visibility_control.h"

namespace frootspi_wheel
{

class WheelNode : public rclcpp::Node
{
public:
  FROOTSPI_WHEEL_PUBLIC
  explicit WheelNode(const rclcpp::NodeOptions & options);

private:
  // publishers
  rclcpp::Publisher<frootspi_msgs::msg::WheelVelocities>::SharedPtr pub_wheel_velocities_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_target_velocity_;

  // subscription callbacks
  void callback_target_velocity(const geometry_msgs::msg::Twist::SharedPtr msg);
};


class WheelVector
{
public:
  void robotVelToWheelRotateVels(
    const double & vx, const double & vy, const double & vw,
    double * v0, double * v1, double * v2);
};

}  // namespace frootspi_wheel

#endif  // FROOTSPI_WHEEL__FROOTSPI_WHEEL_COMPONENT_HPP_
