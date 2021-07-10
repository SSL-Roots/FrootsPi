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
#include "frootspi_hardware/io_expander.hpp"
#include "frootspi_msgs/msg/ball_detection.hpp"
#include "frootspi_msgs/msg/battery_voltage.hpp"
#include "frootspi_msgs/msg/dribble_power.hpp"
#include "frootspi_msgs/msg/switches_state.hpp"
#include "frootspi_msgs/msg/wheel_velocities.hpp"
#include "frootspi_msgs/srv/kick.hpp"
#include "frootspi_msgs/srv/set_kicker_charging.hpp"
#include "frootspi_msgs/srv/set_lcd_text.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace frootspi_hardware
{

class Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  FROOTSPI_HARDWARE_PUBLIC
  explicit Driver(const rclcpp::NodeOptions & options);

  FROOTSPI_HARDWARE_PUBLIC
  ~Driver();

private:
  void on_polling_timer();
  void callback_dribble_power(const frootspi_msgs::msg::DribblePower::SharedPtr msg);
  void callback_wheel_velocities(const frootspi_msgs::msg::WheelVelocities::SharedPtr msg);
  void on_kick(
    const frootspi_msgs::srv::Kick::Request::SharedPtr request,
    frootspi_msgs::srv::Kick::Response::SharedPtr response);
  void on_set_kicker_charging(
    const frootspi_msgs::srv::SetKickerCharging::Request::SharedPtr request,
    frootspi_msgs::srv::SetKickerCharging::Response::SharedPtr response);
  void on_set_lcd_text(
    const frootspi_msgs::srv::SetLCDText::Request::SharedPtr request,
    frootspi_msgs::srv::SetLCDText::Response::SharedPtr response);
  void on_set_left_led(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void on_set_center_led(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void on_set_right_led(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

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

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::BallDetection>>
  pub_ball_detection_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::BatteryVoltage>>
  pub_battery_voltage_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::BatteryVoltage>>
  pub_ups_voltage_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::BatteryVoltage>>
  pub_kicker_voltage_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::SwitchesState>>
  pub_switches_state_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<frootspi_msgs::msg::WheelVelocities>>
  pub_present_wheel_velocities_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> pub_imu_;

  std::shared_ptr<rclcpp::Subscription<frootspi_msgs::msg::DribblePower>> sub_dribble_power_;
  std::shared_ptr<rclcpp::Subscription<frootspi_msgs::msg::WheelVelocities>>
  sub_target_wheel_velocities_;

  std::shared_ptr<rclcpp::Service<frootspi_msgs::srv::Kick>> srv_kick_;
  std::shared_ptr<rclcpp::Service<frootspi_msgs::srv::SetKickerCharging>> srv_set_kicker_charging_;
  std::shared_ptr<rclcpp::Service<frootspi_msgs::srv::SetLCDText>> srv_set_lcd_text_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_set_left_led_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_set_center_led_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_set_right_led_;

  rclcpp::TimerBase::SharedPtr polling_timer_;

  int pi_;
  int gpio_ball_sensor_;
  IOExpander io_expander_;
};

}  // namespace frootspi_hardware

#endif  // FROOTSPI_HARDWARE__DRIVER_COMPONENT_HPP_
