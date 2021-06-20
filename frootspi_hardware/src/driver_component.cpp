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


#include <pigpiod_if2.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "frootspi_hardware/driver_component.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace frootspi_hardware
{

Driver::Driver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hardware_driver", options),
  pi_(-1)
{
}

void Driver::on_polling_timer()
{
  // ボール検出をパブリッシュ
  auto ball_detection_msg = std::make_unique<frootspi_msgs::msg::BallDetection>();
  ball_detection_msg->detected = true;  // 検出したらtrueをセット
  pub_ball_detection_->publish(std::move(ball_detection_msg));

  // バッテリー電圧をパブリッシュ
  auto battery_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  battery_voltage_msg->voltage = 14.8;  // バッテリー電圧 [v]をセット
  battery_voltage_msg->voltage_status =
    frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_FULL;
  pub_battery_voltage_->publish(std::move(battery_voltage_msg));

  // UPS(無停電電源装置)電圧をパブリッシュ
  auto ups_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  ups_voltage_msg->voltage = 3.4;  // UPS電圧 [v]をセット
  ups_voltage_msg->voltage_status =
    frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_TOO_LOW;
  pub_ups_voltage_->publish(std::move(ups_voltage_msg));

  // キッカー（昇圧回路）電圧をパブリッシュ
  auto kicker_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  kicker_voltage_msg->voltage = 200.0;  // キッカー電圧 [v]をセット
  kicker_voltage_msg->voltage_status =
    frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_CHARGED;
  pub_kicker_voltage_->publish(std::move(kicker_voltage_msg));

  // スイッチ状態をパブリッシュ
  auto switches_state_msg = std::make_unique<frootspi_msgs::msg::SwitchesState>();
  switches_state_msg->pushed_button0 = true;  // プッシュスイッチ0が押されていたらtrue
  switches_state_msg->pushed_button1 = false;  // プッシュスイッチ1が押されていたらtrue
  switches_state_msg->pushed_button2 = true;  // プッシュスイッチ2が押されていたらtrue
  switches_state_msg->pushed_button3 = false;  // プッシュスイッチ3が押されていたらtrue
  switches_state_msg->turned_on_dip0 = true;  // DIPスイッチ0がONならtrue
  switches_state_msg->turned_on_dip1 = false;  // DIPスイッチ1がONならtrue
  switches_state_msg->pushed_shutdown = true;  // シャットダウンスイッチがONならtrue
  pub_switches_state_->publish(std::move(switches_state_msg));

  // オムニホイール回転速度をパブリッシュ
  auto wheel_velocities_msg = std::make_unique<frootspi_msgs::msg::WheelVelocities>();
  wheel_velocities_msg->front_left = 1.0;  // 左前ホイール回転速度 [rad/sec]
  wheel_velocities_msg->front_right = 1.0;  // 右前ホイール回転速度 [rad/sec]
  wheel_velocities_msg->back_center = 1.0;  // 後ホイール回転速度 [rad/sec]
  pub_present_wheel_velocities_->publish(std::move(wheel_velocities_msg));

  // IMUセンサの情報をパブリッシュ
}

void Driver::callback_dribble_power(const frootspi_msgs::msg::DribblePower::SharedPtr msg)
{
  std::cout << "ドリブルパワーは" << std::to_string(msg->power) << std::endl;
}

void Driver::callback_wheel_velocities(const frootspi_msgs::msg::WheelVelocities::SharedPtr msg)
{
  std::cout << "車輪目標回転速度は、左前" << std::to_string(msg->front_left);
  std::cout << "、真ん中後" << std::to_string(msg->back_center);
  std::cout << "、右前" << std::to_string(msg->back_center) << std::endl;
}

void Driver::on_kick(
  const frootspi_msgs::srv::Kick::Request::SharedPtr request,
  frootspi_msgs::srv::Kick::Response::SharedPtr response)
{
  std::cout << "キックの種類:" << std::to_string(request->kick_type);
  std::cout << ", キックパワー:" << std::to_string(request->kick_power) << std::endl;

  response->success = true;
  response->message = "キック成功したで";
}

void Driver::on_set_kicker_charging(
  const frootspi_msgs::srv::SetKickerCharging::Request::SharedPtr request,
  frootspi_msgs::srv::SetKickerCharging::Response::SharedPtr response)
{
  if (request->start_charging) {
    std::cout << "キッカーの充電を開始" << std::endl;
  } else {
    std::cout << "キッカーの充電を停止" << std::endl;
  }

  response->success = true;
  response->message = "充電処理に成功したで";
}

void Driver::on_set_lcd_text(
  const frootspi_msgs::srv::SetLCDText::Request::SharedPtr request,
  frootspi_msgs::srv::SetLCDText::Response::SharedPtr response)
{
  std::cout << "LCDに文字列" << request->text << std::endl;

  response->success = true;
  response->message = "LCDに文字列をセットしたで";
}

void Driver::on_set_left_led(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  std::cout << "left_ledを操作するで:" << request->data << std::endl;

  response->success = true;
  response->message = "LEDをセットしたで";
}

void Driver::on_set_center_led(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  std::cout << "center_ledを操作するで:" << request->data << std::endl;

  response->success = true;
  response->message = "LEDをセットしたで";
}

void Driver::on_set_right_led(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  std::cout << "right_ledを操作するで:" << request->data << std::endl;

  response->success = true;
  response->message = "LEDをセットしたで";
}

CallbackReturn Driver::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  polling_timer_ = create_wall_timer(1ms, std::bind(&Driver::on_polling_timer, this));
  // Don't actually start publishing data until activated
  polling_timer_->cancel();

  pub_ball_detection_ = create_publisher<frootspi_msgs::msg::BallDetection>("ball_detection", 1);
  pub_battery_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("battery_voltage", 1);
  pub_ups_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("ups_voltage", 1);
  pub_kicker_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("kicker_voltage", 1);
  pub_switches_state_ = create_publisher<frootspi_msgs::msg::SwitchesState>("switches_state", 1);
  pub_present_wheel_velocities_ = create_publisher<frootspi_msgs::msg::WheelVelocities>(
    "present_wheel_velocities", 1);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 1);

  sub_dribble_power_ = create_subscription<frootspi_msgs::msg::DribblePower>(
    "dribble_power", 1, std::bind(&Driver::callback_dribble_power, this, _1));
  sub_target_wheel_velocities_ = create_subscription<frootspi_msgs::msg::WheelVelocities>(
    "target_wheel_velocities", 1, std::bind(&Driver::callback_wheel_velocities, this, _1));

  srv_kick_ =
    create_service<frootspi_msgs::srv::Kick>("kick", std::bind(&Driver::on_kick, this, _1, _2));
  srv_set_kicker_charging_ = create_service<frootspi_msgs::srv::SetKickerCharging>(
    "set_kicker_charging", std::bind(&Driver::on_set_kicker_charging, this, _1, _2));
  srv_set_lcd_text_ = create_service<frootspi_msgs::srv::SetLCDText>(
    "set_lcd_text", std::bind(&Driver::on_set_lcd_text, this, _1, _2));
  srv_set_left_led_ = create_service<std_srvs::srv::SetBool>(
    "set_left_led", std::bind(&Driver::on_set_left_led, this, _1, _2));
  srv_set_center_led_ = create_service<std_srvs::srv::SetBool>(
    "set_center_led", std::bind(&Driver::on_set_center_led, this, _1, _2));
  srv_set_right_led_ = create_service<std_srvs::srv::SetBool>(
    "set_right_led", std::bind(&Driver::on_set_right_led, this, _1, _2));

  pi_ = pigpio_start(NULL, NULL);

  if (pi_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect pigpiod.");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  pub_ball_detection_->on_activate();
  pub_battery_voltage_->on_activate();
  pub_ups_voltage_->on_activate();
  pub_kicker_voltage_->on_activate();
  pub_switches_state_->on_activate();
  pub_present_wheel_velocities_->on_activate();
  pub_imu_->on_activate();

  polling_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  pub_ball_detection_->on_deactivate();
  pub_battery_voltage_->on_deactivate();
  pub_ups_voltage_->on_deactivate();
  pub_kicker_voltage_->on_deactivate();
  pub_switches_state_->on_deactivate();
  pub_present_wheel_velocities_->on_deactivate();
  pub_imu_->on_deactivate();
  polling_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  pub_ball_detection_.reset();
  pub_battery_voltage_.reset();
  pub_ups_voltage_.reset();
  pub_kicker_voltage_.reset();
  pub_switches_state_.reset();
  pub_present_wheel_velocities_.reset();
  pub_imu_.reset();
  polling_timer_.reset();

  pigpio_stop(pi_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  pub_ball_detection_.reset();
  pub_battery_voltage_.reset();
  pub_ups_voltage_.reset();
  pub_kicker_voltage_.reset();
  pub_switches_state_.reset();
  pub_present_wheel_velocities_.reset();
  pub_imu_.reset();
  polling_timer_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace frootspi_hardware

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_hardware::Driver)
