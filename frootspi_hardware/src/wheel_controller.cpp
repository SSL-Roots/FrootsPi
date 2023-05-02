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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "frootspi_hardware/wheel_controller.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("wheel_controller");

WheelController::WheelController()
: socket_(-1),
  vel_front_right_(0.0), vel_front_left_(0.0), vel_back_center_(0.0),
  gain_p_(0.0), gain_i_(0.0), gain_d_(0.0),
  is_gain_setting_enabled_(false)
{
}

WheelController::~WheelController()
{
  device_close();
}

bool WheelController::device_open()
{
  // ソケットの生成
  // Protocol Family  : CAN
  // Socket Type : RAW
  // Protocol : CAN_RAW
  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to generate socket.");
    return false;
  }

  // Interface Request
  struct ifreq ifr;
  snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "can0");
  if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to io control for interface request.");
    return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // アドレスの割当
  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to bind an address to the socket.");
    return false;
  }

  return true;
}

bool WheelController::device_close()
{
  if (close(socket_) < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to close socket.");
    return false;
  }
  return true;
}

WheelController::ErrorCode WheelController::set_wheel_velocities(
  const double vel_front_right, const double vel_front_left, const double vel_back_center)
{
  constexpr double GEAR_RAITO = 2.2 / 1.091;  // この数値はモタドラと帳尻を合わせること
  constexpr double LSB = 10;  // 送信データの1bitが、モータ速度の10倍を表す
  constexpr double WHEEL_TO_MOTOR = GEAR_RAITO * LSB;

  if (is_gain_setting_enabled_) {
    return WheelController::ERROR_GAIN_SETTING_MODE_ENABLED;
  }

  vel_front_right_ = vel_front_right;
  vel_front_left_ = vel_front_left;
  vel_back_center_ = vel_back_center;

  int16_t motor_vel_right = vel_front_right * WHEEL_TO_MOTOR;
  int16_t motor_vel_left = vel_front_left * WHEEL_TO_MOTOR;
  int16_t motor_vel_center = vel_back_center * WHEEL_TO_MOTOR;


  struct can_frame frame;
  frame.can_id = 0x1aa;
  frame.can_dlc = 8;
  frame.data[0] = 0xAA;
  frame.data[1] = 0xAA;
  frame.data[2] = 0xFF & motor_vel_left;
  frame.data[3] = 0xFF & (motor_vel_left >> 8);
  frame.data[4] = 0xFF & motor_vel_center;
  frame.data[5] = 0xFF & (motor_vel_center >> 8);
  frame.data[6] = 0xFF & motor_vel_right;
  frame.data[7] = 0xFF & (motor_vel_right >> 8);

  bool can_result;
  can_result = send_can(frame);

  if (!can_result) {
    return WheelController::ERROR_CAN_SEND_FAILED;
  }
  return WheelController::ERROR_NONE;
}


bool WheelController::is_motor_stopping()
{
  return (vel_front_right_ == 0.0) && (vel_front_left_ == 0.0) && (vel_back_center_ == 0.0);
}

/**
 * @brief ゲイン設定モードを有効にする
 * @return エラーコード
 * @retval WheelController::ERROR_NONE 成功
 * @retval WheelController::ERROR_WHEELS_ARE_MOVING 車輪が動いているため、ゲイン設定モードを有効にできない
*/
WheelController::ErrorCode WheelController::enable_gain_setting()
{
  // 車輪が停止していたらenableできる
  if (!is_motor_stopping()) {
    RCLCPP_ERROR(LOGGER, "車輪が停止していないため、ゲイン設定を有効にできません");
    return WheelController::ERROR_WHEELS_ARE_MOVING;
  }

  is_gain_setting_enabled_ = true;
  return WheelController::ERROR_NONE;
}

/**
 * @brief ゲイン設定モードを無効にする
 * @return エラーコード
 * @retval WheelController::ERROR_NONE 成功
*/
WheelController::ErrorCode WheelController::disable_gain_setting()
{
  is_gain_setting_enabled_ = false;
  return WheelController::ERROR_NONE;
}

/**
 * @brief Pゲインを設定する
 * @param[in] gain_p Pゲイン
 * @return エラーコード
 * @retval WheelController::ERROR_NONE 成功
 * @retval WheelController::ERROR_GAIN_SETTING_MODE_DISABLED ゲイン設定モードが無効のため、ゲインを設定できない
 * @retval WheelController::ERROR_CAN_SEND_FAILED CAN送信に失敗した
*/
WheelController::ErrorCode WheelController::set_p_gain(const double gain_p)
{
  if (!is_gain_setting_enabled_) {
    return WheelController::ERROR_GAIN_SETTING_MODE_DISABLED;
  }

  gain_p_ = gain_p;
  bool send_result;
  send_result = send_pid_gain();

  if (!send_result) {
    return WheelController::ERROR_CAN_SEND_FAILED;
  }
  return WheelController::ERROR_NONE;
}

/**
 * @brief Iゲインを設定する
 * @param[in] gain_i Iゲイン
 * @return エラーコード
 * @retval WheelController::ERROR_NONE 成功
 * @retval WheelController::ERROR_GAIN_SETTING_MODE_DISABLED ゲイン設定モードが無効のため、ゲインを設定できない
 * @retval WheelController::ERROR_CAN_SEND_FAILED CAN送信に失敗した
*/
WheelController::ErrorCode WheelController::set_i_gain(const double gain_i)
{
  if (!is_gain_setting_enabled_) {
    return WheelController::ERROR_GAIN_SETTING_MODE_DISABLED;
  }

  gain_i_ = gain_i;
  bool send_result;
  send_result = send_pid_gain();

  if (!send_result) {
    return WheelController::ERROR_CAN_SEND_FAILED;
  }
  return WheelController::ERROR_NONE;
}

/**
 * @brief Dゲインを設定する
 * @param[in] gain_d Dゲイン
 * @return エラーコード
 * @retval WheelController::ERROR_NONE 成功
 * @retval WheelController::ERROR_GAIN_SETTING_MODE_DISABLED ゲイン設定モードが無効のため、ゲインを設定できない
 * @retval WheelController::ERROR_CAN_SEND_FAILED CAN送信に失敗した
*/
WheelController::ErrorCode WheelController::set_d_gain(const double gain_d)
{
  if (!is_gain_setting_enabled_) {
    return WheelController::ERROR_GAIN_SETTING_MODE_DISABLED;
  }

  gain_d_ = gain_d;
  bool send_result;
  send_result = send_pid_gain();

  if (!send_result) {
    return WheelController::ERROR_CAN_SEND_FAILED;
  }
  return WheelController::ERROR_NONE;
}

/**
 * @brief PIDゲインを設定する
 * @param[in] gain_p Pゲイン
 * @param[in] gain_i Iゲイン
 * @param[in] gain_d Dゲイン
 * @return エラーコード
 * @retval WheelController::ERROR_NONE 成功
 * @retval WheelController::ERROR_GAIN_SETTING_MODE_DISABLED ゲイン設定モードが無効のため、ゲインを設定できない
 * @retval WheelController::ERROR_CAN_SEND_FAILED CAN送信に失敗した
*/
WheelController::ErrorCode WheelController::set_pid_gain(const double gain_p, const double gain_i, const double gain_d)
{
  if (!is_gain_setting_enabled_) {
    return WheelController::ERROR_GAIN_SETTING_MODE_DISABLED;
  }

  gain_p_ = gain_p;
  gain_i_ = gain_i;
  gain_d_ = gain_d;
  bool send_result;
  send_result = send_pid_gain();

  if (!send_result) {
    return WheelController::ERROR_CAN_SEND_FAILED;
  }
  return WheelController::ERROR_NONE;
}

bool WheelController::send_pid_gain()
{
  constexpr double COEFFICIENT_P = 10000.0;
  constexpr double COEFFICIENT_I = 1000000.0;
  constexpr double COEFFICIENT_D = 1000000.0;

  int16_t int_gain_p = (int16_t)constrain(gain_p_ * COEFFICIENT_P, 0.0, (double)__INT16_MAX__);
  int16_t int_gain_i = (int16_t)constrain(gain_i_ * COEFFICIENT_I, 0.0, (double)__INT16_MAX__);
  int16_t int_gain_d = (int16_t)constrain(gain_d_ * COEFFICIENT_D, 0.0, (double)__INT16_MAX__);

  struct can_frame frame;
  frame.can_id = 0x1aa;
  frame.can_dlc = 8;
  frame.data[0] = 0xBB;
  frame.data[1] = 0xBB;
  frame.data[2] = 0xFF & int_gain_p;
  frame.data[3] = 0xFF & (int_gain_p >> 8);
  frame.data[4] = 0xFF & int_gain_i;
  frame.data[5] = 0xFF & (int_gain_i >> 8);
  frame.data[6] = 0xFF & int_gain_d;
  frame.data[7] = 0xFF & (int_gain_d >> 8);

  return send_can(frame);
}

bool WheelController::send_can(const struct can_frame & frame)
{
  if (write(socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    RCLCPP_ERROR(LOGGER, "Failed to write.");
    return false;
  }

  // RCLCPP_INFO(LOGGER, "Send Can Data: %02x %02x %02x %02x %02x %02x %02x %02x",
  //   frame.data[0], frame.data[1], frame.data[2], frame.data[3],
  //   frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

  return true;
}


/**
 * @brief 値をminとmaxの範囲に収める
*/
double WheelController::constrain(const double value, const double min, const double max)
{
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}



