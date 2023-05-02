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

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "frootspi_hardware/visibility_control.h"
#include "frootspi_hardware/io_expander.hpp"
#include "frootspi_hardware/capacitor_monitor.hpp"
#include "frootspi_hardware/lcd_driver.hpp"
#include "frootspi_hardware/battery_monitor.hpp"
#include "frootspi_hardware/front_display_communicator.hpp"
#include "frootspi_hardware/wheel_controller.hpp"
#include "frootspi_msgs/msg/ball_detection.hpp"
#include "frootspi_msgs/msg/battery_voltage.hpp"
#include "frootspi_msgs/msg/dribble_power.hpp"
#include "frootspi_msgs/msg/switches_state.hpp"
#include "frootspi_msgs/msg/wheel_velocities.hpp"
#include "frootspi_msgs/srv/kick.hpp"
#include "frootspi_msgs/srv/set_kicker_charging.hpp"
#include "frootspi_msgs/srv/set_lcd_text.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace frootspi_hardware
{

class Driver : public rclcpp::Node
{
public:
  FROOTSPI_HARDWARE_PUBLIC
  explicit Driver(const rclcpp::NodeOptions & options);

  FROOTSPI_HARDWARE_PUBLIC
  ~Driver();

private:
  void on_high_rate_polling_timer();
  void on_discharge_kicker_timer();
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
  void on_enable_gain_setting(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  std::shared_ptr<rclcpp::Publisher<frootspi_msgs::msg::BallDetection>>
  pub_ball_detection_;
  std::shared_ptr<rclcpp::Publisher<frootspi_msgs::msg::BatteryVoltage>>
  pub_battery_voltage_;
  std::shared_ptr<rclcpp::Publisher<frootspi_msgs::msg::BatteryVoltage>>
  pub_ups_voltage_;
  std::shared_ptr<rclcpp::Publisher<frootspi_msgs::msg::BatteryVoltage>>
  pub_kicker_voltage_;
  std::shared_ptr<rclcpp::Publisher<frootspi_msgs::msg::SwitchesState>>
  pub_switches_state_;
  std::shared_ptr<rclcpp::Publisher<frootspi_msgs::msg::WheelVelocities>>
  pub_present_wheel_velocities_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> pub_imu_;

  std::shared_ptr<rclcpp::Subscription<frootspi_msgs::msg::DribblePower>> sub_dribble_power_;
  std::shared_ptr<rclcpp::Subscription<frootspi_msgs::msg::WheelVelocities>>
  sub_target_wheel_velocities_;

  std::shared_ptr<rclcpp::Service<frootspi_msgs::srv::Kick>> srv_kick_;
  std::shared_ptr<rclcpp::Service<frootspi_msgs::srv::SetKickerCharging>> srv_set_kicker_charging_;
  std::shared_ptr<rclcpp::Service<frootspi_msgs::srv::SetLCDText>> srv_set_lcd_text_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_set_left_led_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_set_center_led_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_set_right_led_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_enable_gain_setting_;

  // Parameters
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;


  rclcpp::TimerBase::SharedPtr polling_timer_;
  rclcpp::TimerBase::SharedPtr discharge_kicker_timer_;
  rclcpp::Clock steady_clock_;
  rclcpp::Time sub_wheel_timestamp_;
  bool timeout_has_printed_;

  int pi_;
  int gpio_ball_sensor_;
  IOExpander io_expander_;
  BatteryMonitor battery_monitor_;
  LCDDriver lcd_driver_;
  CapacitorMonitor capacitor_monitor_;
  FrontDisplayCommunicator front_display_communicator_;
  bool enable_kicker_charging_;
  int discharge_kick_count_;
  WheelController wheel_controller_;
  int front_display_prescaler_count_;
  int capacitor_monitor_prescaler_count_;
  FrontIndicateData front_indicate_data_;
  // sensor data store
  bool latest_ball_detection_;
};

}  // namespace frootspi_hardware

#endif  // FROOTSPI_HARDWARE__DRIVER_COMPONENT_HPP_
