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
: socket_(-1)
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

bool WheelController::set_wheel_velocities(
  const double vel_front_right, const double vel_front_left, const double vel_back_center)
{
  constexpr double GEAR_RAITO = 2.2 / 1.091;  // この数値はモタドラと帳尻を合わせること
  constexpr double LSB = 10;  // 送信データの1bitが、モータ速度の10倍を表す
  constexpr double WHEEL_TO_MOTOR = GEAR_RAITO * LSB;

  int16_t motor_vel_right = vel_front_right * WHEEL_TO_MOTOR;
  int16_t motor_vel_left = vel_front_left * WHEEL_TO_MOTOR;
  int16_t motor_vel_center = vel_back_center * WHEEL_TO_MOTOR;


  struct can_frame frame;
  frame.can_id = 0x1aa;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF & motor_vel_right;
  frame.data[1] = 0xFF & (motor_vel_right >> 8);
  frame.data[2] = 0xFF & motor_vel_left;
  frame.data[3] = 0xFF & (motor_vel_left >> 8);
  frame.data[4] = 0xFF & motor_vel_center;
  frame.data[5] = 0xFF & (motor_vel_center >> 8);
  frame.data[6] = 0xFF & motor_vel_right;
  frame.data[7] = 0xFF & (motor_vel_right >> 8);

  if (write(socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    RCLCPP_ERROR(LOGGER, "Failed to write.");
    return false;
  }

  return true;
}
